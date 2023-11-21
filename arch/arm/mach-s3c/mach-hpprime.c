// SPDX-License-Identifier: GPL-2.0
// 

#include "irqs-s3c24xx.h"
#include "linux/input-event-codes.h"
#include "linux/mtd/nand.h"
#include "linux/printk.h"
#include "linux/sizes.h"
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/serial_core.h>
#include <linux/serial_s3c.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/mtd/partitions.h>
#include <linux/gpio.h>
#include <linux/fb.h>
#include <linux/delay.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <video/samsung_fimd.h>
#include <asm/irq.h>
#include <asm/mach-types.h>

#include "hardware-s3c24xx.h"
#include "regs-gpio.h"
#include "regs-s3c2443-clock.h"
#include "gpio-samsung.h"

#include <linux/platform_data/leds-s3c24xx.h>
#include <linux/platform_data/i2c-s3c2410.h>

#include "gpio-cfg.h"
#include "devs.h"
#include "cpu.h"
#include <linux/platform_data/mtd-nand-s3c2410.h>
#include "sdhci.h"
#include <linux/platform_data/usb-s3c2410_udc.h>
#include <linux/platform_data/s3c-hsudc.h>
#include <linux/platform_data/simplefb.h>

#include "fb.h"

#include "gpio-cfg.h"
#include "devs.h"
#include "pm.h"

#include "s3c24xx.h"
#include "common-smdk-s3c24xx.h"
 
#include <linux/input.h>
#include <linux/input/goodix.h>
#include <linux/input/matrix_keypad.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>

static struct map_desc smdk2416_iodesc[] __initdata = {
	/* ISA IO Space map (memory space selected by A24) */

	{
		.virtual	= (u32)S3C24XX_VA_ISA_BYTE,
		.pfn		= __phys_to_pfn(S3C2410_CS2),
		.length		= 0x10000,
		.type		= MT_DEVICE,
	}, 
	{
		.virtual	= (u32)S3C24XX_VA_ISA_BYTE + 0x10000,
		.pfn		= __phys_to_pfn(S3C2410_CS2 + (1<<24)),
		.length		= SZ_4M,
		.type		= MT_DEVICE,
	}, 
};

#define UCON (S3C2410_UCON_DEFAULT	| \
		S3C2440_UCON_PCLK	| \
		S3C2443_UCON_RXERR_IRQEN)

#define ULCON (S3C2410_LCON_CS8 | S3C2410_LCON_PNONE)

#define UFCON (S3C2410_UFCON_RXTRIG8	| \
		S3C2410_UFCON_FIFOMODE	| \
		S3C2440_UFCON_TXTRIG16)

static struct s3c2410_uartcfg smdk2416_uartcfgs[] __initdata = {
	[0] = {
		.hwport	     = 0,
		.flags	     = 0,
		.ucon	     = UCON,
		.ulcon	     = ULCON,
		.ufcon	     = UFCON,
	}
};

static void smdk2416_hsudc_gpio_init(void)
{
	printk("<smdk2416_hsudc_gpio_init>\n"); 
	
	s3c_gpio_setpull(S3C2410_GPH(14), S3C_GPIO_PULL_UP); 
	s3c_gpio_cfgpin(S3C2410_GPH(14), S3C_GPIO_SFN(1)); 
	
	gpio_direction_output(S3C2410_GPH(14), 1);
	s3c2410_modify_misccr(S3C2416_MISCCR_SEL_SUSPND, 0);
}

static void smdk2416_hsudc_gpio_uninit(void)
{
	//printk("<smdk2416_hsudc_gpio_uninit>\n");

	s3c2410_modify_misccr(S3C2416_MISCCR_SEL_SUSPND, 1);
	//gpio_direction_output(S3C2410_GPH(14), 0);
	//gpio_direction_input(S3C2410_GPH(14));
	s3c_gpio_setpull(S3C2410_GPH(14), S3C_GPIO_PULL_NONE);
	s3c_gpio_cfgpin(S3C2410_GPH(14), S3C_GPIO_SFN(0)); 

	printk("<smdk2416_hsudc_gpio_uninit>\n");
}

static struct s3c24xx_hsudc_platdata smdk2416_hsudc_platdata = {
	.epnum = 9,
	.gpio_init = smdk2416_hsudc_gpio_init,
	.gpio_uninit = smdk2416_hsudc_gpio_uninit,
};

static struct s3c_fb_pd_win smdk2416_fb_win[] = {
	[0] = {
		.default_bpp	= 16,
		.max_bpp	= 32,
		.xres           = 320,
		.yres           = 240,
	},
};
 

static struct fb_videomode smdk2416_lcd_timing = {
	//.pixclock = 4,
	.refresh = 60,
	
	.left_margin = 64+1, //VIDTCON1_HBPD-1     0x401100
	.right_margin = 17+1, //VIDTCON1_HFPD-1   0x401100
	.hsync_len = 0+1, //VIDTCON1_HSPW-1
	
	.upper_margin = 17+1, //VIDTCON0_VBPD-1    0x110300
	.lower_margin = 3+1, //VIDTCON0_VFPD-1    0x110300
	.vsync_len = 0+1, //VIDTCON0_VSPW-1
	
	.xres = 320,
	.yres = 240,
};


static void s3c2416_fb_gpio_setup_24bpp(void)
{ 
 	//unsigned int gpio;
//
	//for (gpio = S3C2410_GPC(1); gpio <= S3C2410_GPC(4); gpio++) {
	//	s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(2));
	//	s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
	//}
//
	//for (gpio = S3C2410_GPC(8); gpio <= S3C2410_GPC(15); gpio++) {
	//	s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(2));
	//	s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
	//}
//
	//for (gpio = S3C2410_GPD(8); gpio <= S3C2410_GPD(15); gpio++) {
	//	s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(2));
	//	s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
	//}
}

static struct s3c_fb_platdata smdk2416_fb_platdata = {
	.win[0]		= &smdk2416_fb_win[0],
	.vtiming	= &smdk2416_lcd_timing,
	.setup_gpio	= s3c2416_fb_gpio_setup_24bpp,
	.vidcon0	= VIDCON0_VIDOUT_RGB | VIDCON0_PNRMODE_SERIAL_RGB | (1 << 5),
};
 

static struct mtd_partition smdk_default_nand_part[] = {
	[0] = {
		.name	= "HP Boot Code",
		.offset	= 0,
		.size	= SZ_256K,
	},
	[1] = {
		.name	= "Linux Bootloader",
		.offset	= SZ_256K,
		.size	= SZ_512K,
	},
	[2] = {
		.name	= "kernel",
		.offset	= SZ_1M,
		.size	= SZ_8M,
	},
	[3] = {
		.name	= "rootfs",
		.offset = SZ_1M + SZ_8M,
		.size	= 54 * SZ_1M,
	},
	[4] = {
		.name	= "data",
		.offset = MTDPART_OFS_APPEND,
		.size	= 112 * SZ_1M,
	},
	[5] = {
		.name	= "swap",
		.offset = MTDPART_OFS_APPEND,
		.size	= SZ_64M,
	},
	[6] = {
		.name	= "reserved",
		.offset = SZ_256M - SZ_32K,
		.size	= SZ_32K,
	}
};

static struct s3c2410_nand_set smdk_nand_sets[] = {
	[0] = {
		.name		= "NAND",
		.nr_chips	= 1,
		.nr_partitions	= ARRAY_SIZE(smdk_default_nand_part),
		.partitions	= smdk_default_nand_part,
	},
};

static struct s3c2410_platform_nand smdk_nand_info = {
	.tacls		= 20,
	.twrph0		= 30,
	.twrph1		= 20,
	.nr_sets	= ARRAY_SIZE(smdk_nand_sets),
	.sets		= smdk_nand_sets,
	.engine_type	= NAND_ECC_ENGINE_TYPE_NONE,
};

uint32_t keys_define[] = {


	KEY(0, 0, KEY_OK),

	KEY(7, 0, KEY_ENTER),
	KEY(7, 1, KEY_RIGHT),
	
	KEY(6, 0, KEY_O),KEY(6, 1, KEY_R),KEY(6, 2, KEY_Q),KEY(6, 3, KEY_W),
	KEY(6, 4, KEY_V),KEY(6, 5, KEY_U),KEY(6, 6, KEY_UNKNOWN)/*HASH*/,KEY(6, 7, KEY_Z),

	KEY(5, 0, KEY_S),KEY(5, 1, KEY_UNKNOWN)/*CAS*/,
	KEY(5, 2, KEY_MENU),KEY(5, 3, KEY_UNKNOWN)/*VIEW*/,
	KEY(5, 4, KEY_UP),KEY(5, 5, KEY_Y),KEY(5, 6, KEY_EQUAL),KEY(5, 7, KEY_HELP),
	KEY(4, 0, KEY_UNKNOWN)/*NUM*/,KEY(4, 1, KEY_UNKNOWN)/*PLOT*/,KEY(4, 2, KEY_TAB)/*SYMB*/,KEY(4, 3, KEY_HOME),KEY(4, 4, KEY_UNKNOWN)/*APPS*/,KEY(4, 5, KEY_DOWN),KEY(4, 6, KEY_ESC),KEY(4, 7, KEY_N),
	KEY(3, 0, KEY_M),KEY(3, 1, KEY_L),KEY(3, 2, KEY_K),KEY(3, 3, KEY_J),KEY(3, 4, KEY_I),KEY(3, 5, KEY_H),KEY(3, 6, KEY_LEFTSHIFT),KEY(3, 7, KEY_F),
	KEY(2, 0, KEY_BACKSPACE),KEY(2, 1, KEY_D),KEY(2, 2, KEY_C),KEY(2, 3, KEY_CAPSLOCK),KEY(2, 4, KEY_E),KEY(2, 5, KEY_A),KEY(2, 6, KEY_G),KEY(2, 7, KEY_B),
	KEY(1, 0, KEY_P),KEY(1, 1, KEY_APOSTROPHE),KEY(1, 2, KEY_T),KEY(1, 3, KEY_X),KEY(1, 4, KEY_MINUS)/*COLON*/,KEY(1, 5, KEY_SEMICOLON),KEY(1, 6, KEY_SPACE),KEY(1, 7, KEY_LEFT),
};

struct matrix_keymap_data matrix_keyboard_data = {
	.keymap = keys_define,
	.keymap_size = ARRAY_SIZE(keys_define),
};

uint32_t rows[] = {
	S3C2410_GPG(0),	 S3C2410_GPG(1),  S3C2410_GPG(2),  S3C2410_GPG(3),
	S3C2410_GPG(4),	 S3C2410_GPG(5),  S3C2410_GPG(6),  S3C2410_GPG(7), 
};
uint32_t cols[] = {
	S3C2410_GPD(0), S3C2410_GPD(1), S3C2410_GPD(2), S3C2410_GPD(3),
	S3C2410_GPD(4), S3C2410_GPD(5), S3C2410_GPD(6), S3C2410_GPD(7),
};

struct matrix_keypad_platform_data matrix_keypad_platform_data = {
	.keymap_data = &matrix_keyboard_data,
	.row_gpios = rows,
	.col_gpios = cols,

	.num_row_gpios = ARRAY_SIZE(rows),
	.num_col_gpios = ARRAY_SIZE(cols),

	.col_scan_delay_us = 100,
	.debounce_ms = 10,

	.active_low = 0,
	.no_autorepeat = 0,
};

struct platform_device hp_keyboard = {
    .name = "matrix-keypad",
    .id = -1,
    .dev = {
        .platform_data = &matrix_keypad_platform_data,
    },
};
 
//static uint8_t fb_ram[ 320*240*4 ]  __attribute__ ((aligned (PAGE_SIZE)));
//#define fb_ram ((u32)S3C24XX_VA_ISA_BYTE + 0x10000 + SZ_4M)
#define fb_ram (0x30000000 + SZ_32M - SZ_512K)

/* simple-framebuffer */
static struct resource simplefb_resources[] __initdata = { 
	//DEFINE_RES_MEM((uint32_t)&fb_ram[0], 320*240*2), 
	DEFINE_RES_MEM(fb_ram, SZ_512K), 

};
static struct simplefb_platform_data  simplefb_pdata __initdata = {
	.width = 320,
	.height = 240,
	.stride = 320 * 4, //bytes_per_line
	.format = "x8r8g8b8",
};

static struct platform_device_info simplefb_info __initdata = {
	.parent		= &platform_bus,
	.name		= "simple-framebuffer",
	.id		= -1,
	.res		= simplefb_resources,
	.num_res	= ARRAY_SIZE(simplefb_resources),
	.data		= &simplefb_pdata,
	.size_data	= sizeof(simplefb_pdata),
};

static struct platform_device *smdk2416_devices[] __initdata = {
	//&s3c_device_fb,
	&s3c_device_nand,
	&s3c_device_wdt,
	&hp_keyboard,
	&s3c_device_ohci,
	&s3c_device_i2c0, 
	&s3c_device_usb_hsudc,
	&s3c2443_device_dma,
};

 struct goodix_ts_platform_data ts_dat =
 {
	.pin_int = S3C2410_GPF(2),
	.pin_rst = S3C2410_GPF(0),
	.multitouch = 3,
 };

static struct i2c_board_info i2c_devs0[] __initdata = {
	{ 
			I2C_BOARD_INFO("GDIX1001:00", 0x5D),
			.irq = IRQ_EINT2,
			.platform_data = &ts_dat
		},
};

static void __init smdk2416_init_time(void)
{
	s3c2416_init_clocks(12000000);
	s3c24xx_timer_init();
}

static void __init smdk2416_map_io(void)
{
	s3c24xx_init_io(smdk2416_iodesc, ARRAY_SIZE(smdk2416_iodesc));
	s3c24xx_init_uarts(smdk2416_uartcfgs, ARRAY_SIZE(smdk2416_uartcfgs));
	s3c24xx_set_timer_source(S3C24XX_PWM3, S3C24XX_PWM4);
}

static void powercut(void)
{
	u32 cfg;

	printk("PowerOff\n");

	cfg = readl(S3C2443_PWRCFG) & ~S3C2443_PWRCFG_USBPHY;
	writel(cfg, S3C2443_PWRCFG);

	writel(S3C2443_PHYPWR_FSUSPEND, S3C2443_PHYPWR);

	cfg = readl(S3C2443_UCLKCON) & ~S3C2443_UCLKCON_FUNC_CLKEN;
	writel(cfg, S3C2443_UCLKCON);
	//smdk2416_hsudc_gpio_uninit();
}

static void __init smdk2416_machine_init(void)
{
	s3c_i2c0_set_platdata(NULL);
	s3c_fb_set_platdata(&smdk2416_fb_platdata);
	
	
	s3c_nand_set_platdata(&smdk_nand_info);

	//s3c_sdhci0_set_platdata(&smdk2416_hsmmc0_pdata);
	//s3c_sdhci1_set_platdata(&smdk2416_hsmmc1_pdata);

	s3c24xx_hsudc_set_platdata(&smdk2416_hsudc_platdata);
 
	gpio_request(S3C2410_GPB(1), "Display Backlight");
	gpio_direction_output(S3C2410_GPB(1), 1);
 
	  
	i2c_register_board_info(0, i2c_devs0, ARRAY_SIZE(i2c_devs0));

	platform_add_devices(smdk2416_devices, ARRAY_SIZE(smdk2416_devices));
	platform_device_register_full(&simplefb_info);
	
	pm_power_off = powercut;

	s3c_pm_init(); 
}

MACHINE_START(HPPRIMEV2, "HPPRIMEV2")
	/* Maintainer: Repeerc <repeerc@qq.com> */
	.atag_offset	= 0x100,
	.nr_irqs	= NR_IRQS_S3C2416,

	.init_irq	= s3c2416_init_irq,
	.map_io		= smdk2416_map_io,
	.init_machine	= smdk2416_machine_init,
	.init_time	= smdk2416_init_time,
MACHINE_END
