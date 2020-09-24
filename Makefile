KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build

CONFIG_SUPPORT_ESP_SERIAL ?= y
CONFIG_ENABLE_MONITOR_PROCESS ?= n

# Default interface is SPI
target ?= spi

# Targets passed overrrides default value
ifeq ($(target), sdio)
	MODULE_NAME=esp32_sdio
endif
ifeq ($(target), spi)
	MODULE_NAME=esp32_spi
endif

ifeq ($(CONFIG_SUPPORT_ESP_SERIAL), y)
	EXTRA_CFLAGS += -DCONFIG_SUPPORT_ESP_SERIAL
endif

ifeq ($(CONFIG_ENABLE_MONITOR_PROCESS), y)
	EXTRA_CFLAGS += -DCONFIG_ENABLE_MONITOR_PROCESS
endif

EXTRA_CFLAGS += -I$(PWD)

ifeq ($(MODULE_NAME), esp32_sdio)
	module_objects += sdio/esp_sdio.o sdio/esp_sdio_api.o
endif

ifeq ($(MODULE_NAME), esp32_spi)
	module_objects += spi/esp_spi.o
endif

obj-m := $(MODULE_NAME).o
$(MODULE_NAME)-y := esp_bt.o main.o $(module_objects)

ifeq ($(CONFIG_SUPPORT_ESP_SERIAL), y)
	$(MODULE_NAME)-y += esp_serial.o esp_rb.o
endif

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(CURDIR) modules $(KBUILD_OPTIONS)

modules_install:
	$(MAKE) INSTALL_MOD_STRIP=1 -C $(KERNEL_SRC) M=$(CURDIR) modules_install

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(CURDIR) clean
