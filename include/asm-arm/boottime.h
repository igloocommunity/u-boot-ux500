
#ifndef BOOTTIME_H
#define BOOTTIME_H

ulong get_raw_timer(void);
extern ulong boottime_ticks_uboot_init;
extern ulong boottime_ticks_load_kernel;
extern ulong boottime_ticks_uboot_done;

#ifdef CONFIG_BOOTTIME
#define boottime_tag_uboot_init() boottime_ticks_uboot_init = get_raw_timer();
#define boottime_tag_load_kernel() boottime_ticks_load_kernel = get_raw_timer();
#define boottime_tag_uboot_done() boottime_ticks_uboot_done = get_raw_timer();
#else
#define boottime_tag_uboot_init()
#define boottime_tag_load_kernel()
#define boottime_tag_uboot_done()
#endif

#endif
