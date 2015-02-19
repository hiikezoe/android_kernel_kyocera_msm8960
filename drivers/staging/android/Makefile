ccflags-y += -I$(src)			# needed for trace events

obj-$(CONFIG_ANDROID_BINDER_IPC)	+= binder.o
obj-$(CONFIG_ASHMEM)			+= ashmem.o
obj-$(CONFIG_ANDROID_LOGGER)		+= logger.o
obj-$(CONFIG_ANDROID_PERSISTENT_RAM)	+= persistent_ram.o
obj-$(CONFIG_ANDROID_RAM_CONSOLE)	+= ram_console.o
obj-$(CONFIG_ANDROID_TIMED_OUTPUT)	+= timed_output.o
obj-$(CONFIG_ANDROID_TIMED_GPIO)	+= timed_gpio.o
obj-$(CONFIG_ANDROID_LOW_MEMORY_KILLER)	+= lowmemorykiller.o
obj-$(CONFIG_ANDROID_SWITCH)		+= switch/
obj-$(CONFIG_ANDROID_INTF_ALARM_DEV)	+= alarm-dev.o
obj-$(CONFIG_PERSISTENT_TRACER)		+= trace_persistent.o
obj-y +=  extension_ram.o

CFLAGS_REMOVE_trace_persistent.o = -pg

CFLAGS_logger.o += -DBUILD_DISPLAY_ID='"$(RESETLOG_BUILD_DISPLAY_ID)"'
CFLAGS_logger.o += -DPRODUCT_MODEL_NAME='"$(RESETLOG_PRODUCT_MODEL_NAME)"'
CFLAGS_extension_ram.o += -DBUILD_DISPLAY_ID='"$(RESETLOG_BUILD_DISPLAY_ID)"'
CFLAGS_extension_ram.o += -DPRODUCT_MODEL_NAME='"$(RESETLOG_PRODUCT_MODEL_NAME)"'
obj-$(CONFIG_ANDROID_BINDER_IPC)	+= binder_ioctl_convert.o

$(obj)/binder_ioctl_convert.c: $(src)/binder_ioctl_convert.c.head $(obj)/binder_ioctl_convert.c.mid $(src)/binder_ioctl_convert.c.tail
	cat $^ > $@

$(obj)/binder_ioctl_convert.c.mid: $(src)/binder.h
	-rm $@
	grep -E "^#define[[:space:]]+BINDER[0-9A-Z_]+[[:space:]]+_IO[A-Z_]*\(.*\)$$" \
		$< | awk {'print "\t{"$$2 ", \"" $$2 "\"}," '} >> $@
