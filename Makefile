PIO := /Library/Frameworks/Python.framework/Versions/Current/bin/pio
FREEINK_SDK_URL := https://github.com/Free-Ink/freeink-sdk

.PHONY: build

build: $(PIO) freeink-sdk
	$(PIO) run -e x4pro

# Install the PlatformIO CLI if it isn't already on this machine. Real file
# target (not .PHONY) so make skips it once $(PIO) exists on disk.
$(PIO):
	pip3 install -U platformio

# Clone the FreeInk SDK checkout that platformio.ini's lib_deps symlink into.
# Real directory target (not .PHONY) so it's skipped once already cloned.
freeink-sdk:
	git clone $(FREEINK_SDK_URL) freeink-sdk
