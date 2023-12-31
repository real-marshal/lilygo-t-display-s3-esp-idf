driver_dir := ..
mux_dir := ../i2c-mux-testbed

i2c_mux_sources = $(mux_dir)/i2c_mux.h $(mux_dir)/i2c_mux.c
common_sources = $(driver_dir)/sensirion_config.h $(driver_dir)/sensirion_common.h $(driver_dir)/sensirion_common.c
i2c_sources = $(driver_dir)/sensirion_i2c_hal.h ${driver_dir}/sensirion_i2c.h $(driver_dir)/sensirion_i2c.c
sensirion_test_sources = sensirion_test_setup.cpp $(i2c_mux_sources)

sw_i2c_dir := $(driver_dir)/sample-implementations/GPIO_bit_banging

hw_i2c_impl_src = $(driver_dir)/sample-implementations/linux_user_space/sensirion_i2c_hal.c
sw_i2c_impl_src = $(sw_i2c_dir)/sample-implementations/linux_user_space/sensirion_i2c_gpio.c $(sw_i2c_dir)/sensirion_i2c_hal.c

scd30_sources = $(driver_dir)/scd30_i2c.h $(driver_dir)/scd30_i2c.c

CXXFLAGS ?= $(CFLAGS) -fsanitize=address -I$(mux_dir) -I$(driver_dir) -I$(sw_i2c_dir)
ifdef CI
	CXXFLAGS += -Werror
endif
LDFLAGS ?= -lasan -lstdc++ -lCppUTest -lCppUTestExt

.PHONY: clean test


scd30_test_binaries := scd30_test_hw_i2c scd30_test_sw_i2c

all: $(scd30_test_binaries)

scd30_test_hw_i2c: scd30_i2c_test.cpp $(scd30_sources) $(sensirion_test_sources) $(i2c_sources) $(hw_i2c_impl_src) $(common_sources)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

scd30_test_sw_i2c: scd30_i2c_test.cpp $(scd30_sources) $(sensirion_test_sources) $(i2c_sources) $(sw_i2c_impl_src) $(common_sources)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

test: $(scd30_test_binaries)
	set -ex; for test in $(scd30_test_binaries); do echo $${test}; ./$${test}; echo; done;

clean:
	$(RM) $(scd30_test_binaries)
