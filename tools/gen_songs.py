#!/usr/bin/env python3
"""Parse the two golden song MIDIs into emulated-cycle event tables.

Timing model (matches the offline harness): 600 ns per MCU step, 12
cycles per step -> 50 ns per cycle. Tempo map honored (meta 0x51,
piecewise); default 500000 us/quarter when no tempo precedes an event.
Event bytes are the exact bytes to PostMIDI (SysEx stored with leading
F0; a trailing F7 is appended when the file omits it).

Output: src/songs.h with, per song, an event array {cycle, offset,
len} and one concatenated blob of event bytes.
"""
import struct
import sys

NS_PER_CYCLE = 50.0
TAIL_SECONDS = 3.0

# (midi path, c name, harness total steps for validation)
SONGS = [
    ("/home/hatch/workspace/user/files/M_E1M1.mid", "song_e1m1", 164997213),
    ("/home/hatch/workspace/user/files/11_BELMO.MID", "song_belmo", 441939679),
]


def read_varlen(d, p):
    v = 0
    while True:
        b = d[p]
        p += 1
        v = (v << 7) | (b & 0x7F)
        if not (b & 0x80):
            return v, p


def parse(path):
    """Replicate the harness scheduler (jit-test RunWorkload) exactly.

    Upstream SMF_MergeTracks concatenates all tracks, stable-sorts by
    (timestamp, per-track seq_id), and recomputes deltas from timestamps.
    RunWorkload then walks the merged list with integer arithmetic:
        event_ns = sim_ns + 1000 * (delta_ticks * us_per_qn // division)
        while sim_ns < event_ns: sim_ns += 600   # 600 ns/step overshoot
        if tempo: us_per_qn = new (AFTER stepping to the event)
        elif not meta: PostMIDI(status + data bytes)
    Event cycles are sim_ns / 50 (sim_ns is always a multiple of 600,
    hence of 50; 600 ns/step = 12 cycles/step). The 3 s tail runs from
    sim_ns after the LAST merged event -- which is the end-of-track meta,
    not the last MIDI event (BELMO's EOT sits 1440 ticks past its last
    note: 1.875 s of trailing silence the harness still schedules).
    """
    d = open(path, "rb").read()
    assert d[:4] == b"MThd", path
    hlen, fmt, ntrk, div = struct.unpack(">IHHH", d[4:14])
    assert div > 0, "SMPTE division not supported"
    pos = 14 + (hlen - 6) if hlen > 6 else 14
    merged = []  # (tick, per_track_seq, kind, payload); kind: midi/tempo/meta
    for _ in range(ntrk):
        assert d[pos:pos + 4] == b"MTrk", (path, pos)
        tlen = struct.unpack(">I", d[pos + 4:pos + 8])[0]
        end = pos + 8 + tlen
        p = pos + 8
        tick = 0
        run = None
        per_idx = 0
        while p < end:
            delta, p = read_varlen(d, p)
            tick += delta
            st = d[p]
            if st < 0x80:
                st = run
            else:
                p += 1
                # Match upstream smf.cpp exactly: running status is set ONLY
                # by a channel status byte and is NOT cleared by meta/SysEx.
                if st < 0xF0:
                    run = st
            if st == 0xFF:
                mtype = d[p]
                p += 1
                ln, p = read_varlen(d, p)
                payload = d[p:p + ln]
                p += ln
                if mtype == 0x2F:
                    # Upstream: end-of-track stops the track (seek to end).
                    merged.append((tick, per_idx, "meta", None))
                    per_idx += 1
                    break
                if mtype == 0x51 and ln == 3:
                    us = (payload[0] << 16) | (payload[1] << 8) | payload[2]
                    merged.append((tick, per_idx, "tempo", us))
                else:
                    merged.append((tick, per_idx, "meta", None))
                per_idx += 1
            elif st in (0xF0, 0xF7):
                ln, p = read_varlen(d, p)
                payload = d[p:p + ln]
                p += ln
                # Exact byte stream upstream posts: status + payload, no
                # invented trailing F7 (split SysEx stays split).
                merged.append((tick, per_idx, "midi", bytes([st]) + payload))
                per_idx += 1
            elif (st & 0xF0) in (0xC0, 0xD0):
                # Program change / channel pressure: 1 data byte on ANY
                # channel (0xC0-0xCF, 0xD0-0xDF). Upstream switches on
                # (status & 0xF0); matching only 0xC0/0xD0 exactly eats the
                # next delta byte and desyncs the whole track.
                merged.append((tick, per_idx, "midi", bytes([st, d[p]])))
                per_idx += 1
                p += 1
            elif st in (0xF1, 0xF3):
                merged.append((tick, per_idx, "midi", bytes([st, d[p]])))
                per_idx += 1
                p += 1
            elif st == 0xF2:
                merged.append((tick, per_idx, "midi", bytes([st, d[p], d[p + 1]])))
                per_idx += 1
                p += 2
            elif st in (0xF6, 0xF8, 0xF9, 0xFA, 0xFB, 0xFC, 0xFD, 0xFE):
                merged.append((tick, per_idx, "midi", bytes([st])))
                per_idx += 1
            elif st is None:
                raise ValueError(f"{path} track {_}: data byte with no running status at {p}")
            elif st in (0xF4, 0xF5):
                # Upstream panics (exit 1) on undefined Fx statuses.
                raise ValueError(f"{path} track {_}: undefined system status {st:#x} at {p}")
            else:
                # 0x80-0xBF, 0xE0-0xEF: 2 data bytes.
                merged.append((tick, per_idx, "midi", bytes([st, d[p], d[p + 1]])))
                per_idx += 1
                p += 2
        pos = end
    # Stable sort by (tick, per_track_seq) over concatenation order, exactly
    # like upstream's stable_sort by (timestamp, seq_id). Same-tick order
    # cannot change any event time (delta 0); it only matters for two
    # tempos sharing a tick in different tracks -- neither song has that.
    merged.sort(key=lambda e: (e[0], e[1]))
    events = []
    blob = bytearray()
    sim_ns = 0
    prev_tick = 0
    us_per_qn = 500000
    for tick, _, kind, payload in merged:
        event_ns = sim_ns + 1000 * ((tick - prev_tick) * us_per_qn // div)
        prev_tick = tick
        # Smallest multiple of 600 >= event_ns (sim_ns stays a multiple of
        # 600 throughout, so this is exactly the harness's while loop).
        if sim_ns < event_ns:
            sim_ns += ((event_ns - sim_ns + 599) // 600) * 600
        if kind == "tempo":
            us_per_qn = payload
        elif kind == "midi":
            cycle = sim_ns // 50
            off = len(blob)
            blob.extend(payload)
            events.append((cycle, off, len(payload)))
    last_sec = sim_ns / 1e9
    total_steps = (sim_ns + int(TAIL_SECONDS * 1e9)) // 600
    end_cycle = (sim_ns + int(TAIL_SECONDS * 1e9)) // 50
    return events, bytes(blob), last_sec, total_steps, len(events), end_cycle


def main():
    out = []
    out.append("// Generated by tools/gen_songs.py -- do not hand-edit.")
    out.append("// Event cycles: emulated H8 cycles (50 ns each). Blob bytes are")
    out.append("// the exact bytes to PostMIDI, in order, per event.")
    out.append("#pragma once")
    out.append("#include <cstdint>")
    out.append("#include <cstddef>")
    out.append("")
    out.append("struct SongEvent { uint64_t cycle; uint32_t offset; uint32_t len; };")
    out.append("")
    for path, name, harness_steps in SONGS:
        events, blob, last_sec, total_steps, nev, end_cycle = parse(path)
        print(f"{name}: {nev} events, last event {last_sec:.3f}s, "
              f"total+tail {total_steps} steps vs harness {harness_steps} "
              f"(delta {total_steps - harness_steps})")
        out.append(f"static const SongEvent {name}_events[] = {{")
        for cyc, off, ln in events:
            out.append(f"    {{{cyc}ULL, {off}U, {ln}U}},")
        out.append("};")
        out.append(f"static const uint8_t {name}_blob[] = {{")
        for i in range(0, len(blob), 20):
            out.append("    " + ",".join(str(b) for b in blob[i:i + 20]) + ",")
        out.append("};")
        out.append(f"static const size_t {name}_event_count = "
                   f"sizeof({name}_events) / sizeof({name}_events[0]);")
        out.append(f"static const uint64_t {name}_end_cycle = "
                   f"{end_cycle}ULL;")
        out.append("")
    with open("src/songs.h", "w") as f:
        f.write("\n".join(out))
    print("wrote src/songs.h")


if __name__ == "__main__":
    main()
