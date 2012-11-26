/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
*/










#include <linux/platform_device.h>
#include <linux/gpio_event.h>

#include <asm/mach-types.h>
#include <mach/hs_io_ctl_a.h>

static struct gpio_event_direct_entry slide_switch_map[] = {
        { GPIO_SLIDE,              SW_LID   }
};

static struct gpio_event_input_info slide_switch_info = {
        .info.func = gpio_event_input_func,
        .info.no_suspend = true,
        .flags = GPIOEDF_ACTIVE_HIGH,
        .type = EV_SW,
        .keymap = slide_switch_map,
#if BITS_PER_LONG < 64
        .debounce_time.tv64 = 25 * NSEC_PER_MSEC,
#else /* BITS_PER_LONG >= 64 */
        .debounce_time.tv.nsec = 25 * NSEC_PER_MSEC,
#endif /* BITS_PER_LONG < 64 */
        .keymap_size = ARRAY_SIZE(slide_switch_map)
};

static struct gpio_event_info *gpio_slide_sw_info[] = {
        &slide_switch_info.info
};

static struct gpio_event_platform_data slide_switch_data = {
	.name		= "slide_switch",
	.info		= gpio_slide_sw_info,
	.info_count	= ARRAY_SIZE(gpio_slide_sw_info)
};

struct platform_device slide_switch_device = {
	.name	= GPIO_EVENT_DEV_NAME,
	.id	= 0,
	.dev	= {
		.platform_data	= &slide_switch_data,
	},
};


