#!/usr/bin/env python3
"""Summarize a shaped-var3-stats.log (round-3 run sequence) per run.

Usage:
    python tools/analyze_log.py SD/shaped-var3-stats.log [--csv out.csv] [--windows]

What it prints, per run (in run order):
  - pct            median window pct (real-time rate; 100 = keeping up)
  - c2cps          core-2 cycles per MCU step   (step_cyc / steps)   <- THE core-2 comparator
  - lockcps        core-2 lock-spin cycles per MCU step (lock_cyc / steps)
  - c3cpt          core-3 cycles per PCM tick   (pcm_cyc / ticks)    <- THE core-3 comparator
  - c2ipc, c3ipc   instructions per cycle (events / cycles; 0 if the event counters failed)
  - c2mis, c3mis   branch mispredicts per 1000 instructions
  - c2l1d, c3l1d   L1D refills per 1000 instructions
  - lock/s         core-2 lock ops per second, and the contended share
  - fifo_hw        deferred-store FIFO high water (FIFO modes)
  - flags          anything that disqualifies the run: underruns, ring_drops, sound_drops,
                   fifo_full, tchk=FAIL, timer_selftest=FAIL
and the delta of c2cps / c3cpt / pct against run 0 (the baseline).

Decision rule (see docs/optimization-round3-sync-20261031.md):
  a lever is CREDIBLE when c2cps (or c3cpt) drops by >= 5% vs run 0 with no flags,
  the drop is consistent across the song's windows (p10..p90 shown with --windows),
  and pct does not fall. pct saturates at 100, so once a run keeps up the
  cycles-per-step/tick ratios are the only numbers that still rank runs.

Robust to a power cut mid-sequence: every run with at least one window is
reported; run_total lines are used when present, window sums otherwise.
"""
import argparse
import csv
import re
import statistics
import sys
from collections import OrderedDict, defaultdict

KV = re.compile(r'(\w+)=(\S+)')


def parse_line(line):
    line = line.strip()
    if not line:
        return None, None
    # "t=123 win n=0 ..." -> kind is the first bare token after t=
    parts = line.split(' ', 2)
    if len(parts) < 2:
        return None, None
    kind = parts[1]
    fields = OrderedDict()
    for k, v in KV.findall(line):
        fields.setdefault(k, v)
    return kind, fields


def num(v, default=0):
    try:
        return int(v)
    except (TypeError, ValueError):
        try:
            return float(v)
        except (TypeError, ValueError):
            return default


def pct_delta(a, b):
    if not b:
        return 'n/a'
    return f'{(a - b) * 100.0 / b:+.1f}%'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('log')
    ap.add_argument('--csv', help='write per-run table as CSV')
    ap.add_argument('--windows', action='store_true', help='also print per-window distributions')
    args = ap.parse_args()

    boot = {}
    runlist = {}
    windows = defaultdict(list)     # run -> [fields]
    run_totals = {}
    run_lines = {}
    selftest = {}
    events = defaultdict(int)
    with open(args.log, encoding='utf-8', errors='replace') as f:
        for line in f:
            kind, fields = parse_line(line)
            if kind is None:
                continue
            if kind == 'boot':
                boot = fields
            elif kind == 'runlist':
                runlist[num(fields.get('n'))] = fields
            elif kind == 'win':
                if fields.get('paused') == '1':
                    continue
                windows[num(fields.get('var'))].append(fields)
            elif kind == 'run_total':
                run_totals[num(fields.get('n'))] = fields
            elif kind == 'run':
                run_lines[num(fields.get('n'))] = fields
            elif kind == 'timer_selftest':
                selftest = fields
            elif kind in ('sound_drop', 'sound_restart', 'sound_start_fail', 'mister_drop'):
                events[kind] += 1

    if not windows and not run_totals:
        print('no win/run_total lines found; is this a shaped-var3 log?')
        return 1

    print(f"boot: kernel={boot.get('kernel','?')} pmc2={boot.get('pmc2','?')} pmc3={boot.get('pmc3','?')} "
          f"pmev2={boot.get('pmev2','?')} pmev3={boot.get('pmev3','?')} "
          f"timer_selftest={boot.get('timer_selftest', selftest.get('result','?'))} "
          f"(plain_mism={boot.get('selftest_plain_mism','?')} inject_mism={boot.get('selftest_inject_mism','?')}) "
          f"runs={boot.get('runs','?')}")
    if boot.get('timer_selftest', selftest.get('result')) not in ('pass', None):
        print('  !! timer self-test FAILED: lazy-timer runs (timer=1/2) are not trustworthy')
    if events:
        print('  device events: ' + ', '.join(f'{k}={v}' for k, v in sorted(events.items())))

    runs = sorted(set(windows) | set(run_totals))
    rows = []
    for r in runs:
        ws = windows.get(r, [])
        # Skip the first window of each run (startup cushion / partial) if there are enough.
        body = ws[1:] if len(ws) > 3 else ws
        tot = run_totals.get(r)

        def wsum(key):
            return sum(num(w.get(key)) for w in body)

        steps = num(tot.get('steps')) if tot else wsum('steps')
        ticks = num(tot.get('ticks')) if tot else wsum('ticks')
        step_cyc = num(tot.get('step_cyc')) if tot else wsum('step_cyc')
        lock_cyc = num(tot.get('lock_cyc')) if tot else wsum('lock_cyc')
        pcm_cyc = num(tot.get('pcm_cyc')) if tot else wsum('pcm_cyc')
        mix_cyc = num(tot.get('mix_cyc')) if tot else wsum('mix_cyc')
        c2 = [num(tot.get(k)) if tot else wsum(k) for k in ('c2_inst', 'c2_brmis', 'c2_l1d', 'c2_l2d')]
        c3 = [num(tot.get(k)) if tot else wsum(k) for k in ('c3_inst', 'c3_brmis', 'c3_l1d', 'c3_l2d')]
        lock_waits = num(tot.get('lock_waits')) if tot else wsum('lock_waits')
        lock_cont = num(tot.get('lock_cont')) if tot else wsum('lock_cont')
        underruns = num(tot.get('underruns')) if tot else wsum('underruns')
        pcts = [num(w.get('pct')) for w in body] or [num(tot.get('pct'))] if tot else [num(w.get('pct')) for w in body]
        med_pct = statistics.median(pcts) if pcts else 0
        elapsed_s = 5.0 * len(body)
        c2cps = step_cyc / steps if steps else 0
        lockcps = lock_cyc / steps if steps else 0
        c3cpt = pcm_cyc / ticks if ticks else 0
        mixshare = mix_cyc / pcm_cyc * 100 if pcm_cyc else 0
        # core-2 IPC uses the step-loop wall cycles (step_cyc + lock_cyc) since the
        # event window covers the whole step loop.
        c2ipc = c2[0] / (step_cyc + lock_cyc) if (step_cyc + lock_cyc) and c2[0] else 0
        c3ipc = c3[0] / pcm_cyc if pcm_cyc and c3[0] else 0
        c2mis = c2[1] * 1000 / c2[0] if c2[0] else 0
        c3mis = c3[1] * 1000 / c3[0] if c3[0] else 0
        c2l1d = c2[2] * 1000 / c2[0] if c2[0] else 0
        c3l1d = c3[2] * 1000 / c3[0] if c3[0] else 0
        fifo_hw = max([num(w.get('fifo_hw')) for w in ws] or [0])
        fifo_full = max([num(w.get('fifo_full')) for w in ws] or [0])
        ring_drops = max([num(w.get('ring_drops')) for w in ws] or [0])
        sound_drops = sum(num(w.get('sound_drops')) for w in ws)
        tchk = (tot.get('tchk') if tot else None) or (ws[-1].get('tchk') if ws else 'off')
        tchk_mism = num(tot.get('tchk_mism')) if tot else (num(ws[-1].get('tchk_mism')) if ws else 0)
        tchk_cmp = num(tot.get('tchk_cmp')) if tot else (num(ws[-1].get('tchk_cmp')) if ws else 0)
        flags = []
        if underruns:
            flags.append(f'underruns={underruns}')
        if sound_drops:
            flags.append(f'sound_drops={sound_drops}')
        if fifo_full:
            flags.append(f'fifo_full={fifo_full}')
        if tchk == 'FAIL':
            flags.append(f'tchk=FAIL({tchk_mism})')
        if tot is None:
            flags.append('incomplete')
        rl = runlist.get(r, {})
        name = (tot or (ws[-1] if ws else {})).get('name') or rl.get('name') or (run_lines.get(r, {}).get('name')) or f'run{r}'
        rows.append(dict(run=r, name=name, pcm=rl.get('pcm', (ws[-1].get('pcm') if ws else '?')),
                         sync=rl.get('sync', (ws[-1].get('sync') if ws else '?')),
                         timer=rl.get('timer', (ws[-1].get('timer') if ws else '?')),
                         windows=len(ws), pct=med_pct, c2cps=c2cps, lockcps=lockcps, c3cpt=c3cpt,
                         mixshare=mixshare, c2ipc=c2ipc, c3ipc=c3ipc, c2mis=c2mis, c3mis=c3mis,
                         c2l1d=c2l1d, c3l1d=c3l1d,
                         lock_per_s=lock_waits / elapsed_s if elapsed_s else 0,
                         cont_share=lock_cont * 100 / lock_waits if lock_waits else 0,
                         fifo_hw=fifo_hw, ring_drops=ring_drops, tchk=tchk, tchk_cmp=tchk_cmp,
                         flags=' '.join(flags) or '-', pcts=pcts))

    base = rows[0] if rows and rows[0]['run'] == 0 else None
    hdr = f"{'run':>3} {'name':<9} {'p/s/t':<6} {'win':>3} {'pct':>5} {'c2cps':>7} {'d_c2':>7} {'lockcps':>7} {'c3cpt':>7} {'d_c3':>7} {'mix%':>5} {'c2ipc':>5} {'c3ipc':>5} {'c2mis':>5} {'c3mis':>5} {'lock/s':>7} {'cont%':>5} {'fifohw':>6} {'tchk':>5} flags"
    print(hdr)
    print('-' * len(hdr))
    for row in rows:
        d_c2 = pct_delta(row['c2cps'], base['c2cps']) if base else 'n/a'
        d_c3 = pct_delta(row['c3cpt'], base['c3cpt']) if base else 'n/a'
        print(f"{row['run']:>3} {row['name']:<9} {str(row['pcm'])+'/'+str(row['sync'])+'/'+str(row['timer']):<6} {row['windows']:>3} {row['pct']:>5.0f} "
              f"{row['c2cps']:>7.0f} {d_c2:>7} {row['lockcps']:>7.1f} {row['c3cpt']:>7.0f} {d_c3:>7} {row['mixshare']:>5.1f} "
              f"{row['c2ipc']:>5.2f} {row['c3ipc']:>5.2f} {row['c2mis']:>5.1f} {row['c3mis']:>5.1f} "
              f"{row['lock_per_s']:>7.0f} {row['cont_share']:>5.1f} {row['fifo_hw']:>6} {row['tchk']:>5} {row['flags']}")
    print()
    print('Reading the table: a core-2 lever must lower c2cps (and lockcps) with no flags; a core-3 lever must lower c3cpt.')
    print('At 1 GHz and 2M MCU steps/s real time, core 2 keeps up when c2cps + lockcps <= ~500 with core2_pct < 100;')
    print('core 3 keeps up when c3cpt <= ~31,000 (one PCM tick per 31.25 us).')
    if any(r['run'] == 7 for r in rows):
        t = [r for r in rows if r['run'] == 7][0]
        print(f"Timer check run (7): tchk={t['tchk']} compares={t['tchk_cmp']} -> "
              + ('lazy timer is bit-identical to legacy on this song' if t['tchk'] == 'ok' and t['tchk_cmp'] > 0 else 'DO NOT trust lazy-timer runs'))

    if args.windows:
        print()
        for row in rows:
            p = sorted(row['pcts'])
            if not p:
                continue
            q = lambda f: p[min(len(p) - 1, int(f * len(p)))]
            print(f"run {row['run']:>2} {row['name']:<9} pct p10={q(0.1):.0f} p50={q(0.5):.0f} p90={q(0.9):.0f} min={p[0]:.0f} max={p[-1]:.0f} n={len(p)}")

    if args.csv:
        keys = [k for k in rows[0].keys() if k != 'pcts']
        with open(args.csv, 'w', newline='', encoding='utf-8') as f:
            w = csv.DictWriter(f, fieldnames=keys, extrasaction='ignore')
            w.writeheader()
            for row in rows:
                w.writerow(row)
        print(f'wrote {args.csv}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
