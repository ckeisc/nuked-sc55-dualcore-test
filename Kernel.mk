#
# Build kernel -- minimal dual-core Nuked-SC55 validity kernel
# Modeled on the mt32-pi port tree's Kernel.mk, trimmed to three
# translation units and the Circle libs this kernel actually uses
# (no sound, input, net, usb, wlan: no audio device, no USB, no
# network, no MIDI input -- the only inputs are the SD card and one
# GPIO key).
#

include Config.mk

OBJS		:=	src/kernel.o \
			src/main.o \
			src/minimal.o

EXTRACLEAN	+=	src/*.d src/*.o

# The Nuked-SC55 backend headers require C++20 (the backend library
# itself is built separately by the top-level Makefile)
src/minimal.o: STANDARD = -std=c++20 -Wno-aligned-new

include $(CIRCLEHOME)/Rules.mk

CFLAGS		+=	-Werror -Wextra -Wno-unused-parameter

CFLAGS		+=	-I "$(NEWLIBDIR)/include" \
			-I $(STDDEF_INCPATH) \
			-I $(CIRCLESTDLIBHOME)/include \
			-I include \
			-I .

# NOTE: libusb + libinput are linked although this kernel creates no
# USB or input device: the newlib glue (libcirclenewlib.a io.o)
# references CUSBHostController unconditionally, and libusb's device
# objects reference the input behaviour classes. Linking them starts
# no USB stack and no input device (no CUSBHCIDevice instance and no
# input device instance exist in this kernel).
LIBS 		:=	$(CIRCLE_STDLIB_LIBS) \
			$(CIRCLEHOME)/addon/fatfs/libfatfs.a \
			$(CIRCLEHOME)/addon/SDCard/libsdcard.a \
			$(CIRCLEHOME)/lib/fs/libfs.a \
			$(CIRCLEHOME)/lib/input/libinput.a \
			$(CIRCLEHOME)/lib/libcircle.a \
			$(CIRCLEHOME)/lib/sched/libsched.a \
			$(CIRCLEHOME)/lib/sound/libsound.a \
			$(CIRCLEHOME)/lib/usb/libusb.a

-include $(DEPS)

EXTRALIBS	+=	$(NUKEDLIB)

# The minimal TU is compiled against the backend's headers in place (its
# own source tree plus the generated config.h), with the same include
# layout the backend's own build uses. Scoped to this TU (and its dep
# file) so the backend's generated config.h can never shadow the
# kernel's own headers for other translation units.
src/minimal.o: CPPFLAGS += -I $(NUKEDHOME)/src/backend -I $(NUKEDBUILDDIR)/backend
src/minimal.d: CPPFLAGS += -I $(NUKEDHOME)/src/backend -I $(NUKEDBUILDDIR)/backend
