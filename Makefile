#
# Makefile -- minimal dual-core Nuked-SC55 validity kernel
# Modeled on the mt32-pi port tree's Makefile, trimmed to circle-stdlib
# + the Nuked backend only. No USB/network patches (no USB, no network
# in this kernel); the gzip-kernel patch is kept so kernel8.img has the
# same format as the port kernels already flashed.
#

include Config.mk

.DEFAULT_GOAL=all
.PHONY: circle-stdlib nuked all clean veryclean

#
# Apply a patch only if not completely applied already
#
APPLY_PATCH=sh -c '																\
	if patch --dry-run --reverse --force --strip 1 --directory $$1 < $$2 >/dev/null 2>&1; then						\
		echo "Patch $$2 already applied; skipping";											\
	else																	\
		echo "Applying patch $$2 to directory $$1...";											\
		patch --forward --strip 1 --no-backup-if-mismatch -r - --directory $$1 < $$2 || (echo "Patch $$2 failed" >&2 && exit 1);	\
	fi' APPLY_PATCH

REVERSE_PATCH=sh -c '																\
	if patch --dry-run --forward --force --strip 1 --directory $$1 < $$2 >/dev/null 2>&1; then						\
		echo "Patch $$2 already reversed; skipping";											\
	else																	\
		echo "Reversing patch $$2 in directory $$1...";											\
		patch --reverse --strip 1 --no-backup-if-mismatch -r - --directory $$1 < $$2 || (echo "Patch $$2 failed" >&2 && exit 1);	\
	fi' REVERSE_PATCH

#
# Configure circle-stdlib
#
$(CIRCLE_STDLIB_CONFIG) $(CIRCLE_CONFIG)&:
	@echo "Configuring for Raspberry Pi $(RASPBERRYPI) ($(BITS) bit)"
	$(CIRCLESTDLIBHOME)/configure --raspberrypi=$(RASPBERRYPI) --prefix=$(PREFIX)

# Apply patches
	@${APPLY_PATCH} $(CIRCLEHOME) patches/circle-45-gzip-kernel.patch

ifeq ($(strip $(GC_SECTIONS)),1)
# Enable function/data sections for circle-stdlib
	@echo "CFLAGS_FOR_TARGET += -ffunction-sections -fdata-sections" >> $(CIRCLE_STDLIB_CONFIG)
endif

# Enable multi-core
	@echo "DEFINE += -DARM_ALLOW_MULTI_CORE" >> $(CIRCLE_CONFIG)

# Disable delay loop calibration (boot speed improvement)
	@echo "DEFINE += -DNO_CALIBRATE_DELAY" >> $(CIRCLE_CONFIG)

# Improve I/O throughput
	@echo "DEFINE += -DNO_BUSY_WAIT" >> $(CIRCLE_CONFIG)

# The Nuked-SC55 backend pushes the kernel image past Circle's default
# 2 MB KERNEL_MAX_SIZE, which would leave the kernel stack inside .rodata.
# 4 MB is kept deliberately (same as the mt32-pi port) so the image has
# room to grow. GUARD: _end must stay below 0x480000 (checked at build).
	@echo "DEFINE += -DKERNEL_MAX_SIZE=4194304" >> $(CIRCLE_CONFIG)

#
# Build circle-stdlib
#
circle-stdlib: $(CIRCLESTDLIBHOME)/.done

$(CIRCLESTDLIBHOME)/.done: $(CIRCLE_STDLIB_CONFIG)
	@$(MAKE) -C $(CIRCLESTDLIBHOME)
	touch $@

#
# Build Nuked-SC55 (base + dual-core split patch, byte-identical to the
# mt32-pi port's backend tree; patch md5 5d6f043cb6c5430d80b81c8472fbb954)
#
nuked: $(NUKEDBUILDDIR)/.done

$(NUKEDBUILDDIR)/.done: $(CIRCLESTDLIBHOME)/.done
	@${APPLY_PATCH} $(NUKEDHOME) patches/nuked-sc55-0.8.0-circle.patch

	@CFLAGS="$(CFLAGS_EXTERNAL)" \
	CXXFLAGS="$(CFLAGS_EXTERNAL)" \
	cmake -B $(NUKEDBUILDDIR) \
		 $(CMAKE_TOOLCHAIN_FLAGS) \
		 -DCMAKE_CXX_FLAGS_RELEASE="-Ofast" \
		 -DCMAKE_CXX_FLAGS="-include $(CURDIR)/include/nukedcirclecompat.h" \
		 -DCMAKE_BUILD_TYPE=Release \
		 -DBUILD_SHARED_LIBS=OFF \
		 $(NUKEDHOME) \
		 >/dev/null
	@cmake --build $(NUKEDBUILDDIR) --target nuked-sc55-backend
	@touch $@

#
# Build kernel itself
#
all: circle-stdlib nuked
	@$(MAKE) -f Kernel.mk $(KERNEL).img $(KERNEL).hex

#
# Clean kernel only
#
clean:
	@$(MAKE) -f Kernel.mk clean

#
# Clean kernel and all dependencies
#
mrproper: clean
# Reverse patches
	@${REVERSE_PATCH} $(CIRCLEHOME) patches/circle-45-gzip-kernel.patch

# Clean circle-stdlib
	@if [ -f $(CIRCLE_STDLIB_CONFIG) ]; then $(MAKE) -C $(CIRCLESTDLIBHOME) mrproper; fi
	@$(RM) $(CIRCLESTDLIBHOME)/.done

# Clean Nuked-SC55
	@${REVERSE_PATCH} $(NUKEDHOME) patches/nuked-sc55-0.8.0-circle.patch
	@$(RM) -r $(NUKEDBUILDDIR)
