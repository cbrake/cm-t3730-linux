/*
 * CompuLab CM-T35/CM-T3730 modules support
 *
 * Copyright (C) 2009-2011 CompuLab, Ltd.
 * Authors: Mike Rapoport <mike@compulab.co.il>
 *	    Igor Grinberg <grinberg@compulab.co.il>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/input/matrix_keypad.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/wl12xx.h>
#include <linux/opp.h>

#include <linux/i2c/at24.h>
#include <linux/i2c/twl.h>
#include <linux/ds2782_battery.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/machine.h>
#include <linux/mmc/host.h>

#include <linux/spi/spi.h>
#include <linux/spi/tdo24m.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/nand.h>
#include <plat/gpmc.h>
#include <plat/usb.h>
#include <video/omapdss.h>
#include <video/omap-panel-generic-dpi.h>
#include <media/mt9t001.h>
#include <plat/mcspi.h>
#include <plat/omap_device.h>

#include <mach/hardware.h>

#include "mux.h"
#include "sdram-micron-mt46h32m32lf-6.h"
#include "hsmmc.h"
#include "common-board-devices.h"
#include "devices.h"
#include "pm.h"

#define CM_T35_GPIO_PENDOWN		57
#define CM_T35_SMSC911X_CS		5
#define CM_T35_SMSC911X_GPIO		163

/* SB-T35 board specific GPIOs */
#define SB_T35_SMSC911X_CS		4
#define SB_T35_SMSC911X_GPIO		65
#define SB_T35_USB_HUB_RESET_GPIO	167

/* CB-T3 board specific GPIOs */
#define CB_T3_nHP_GPIO			61
#define CB_T3_nPWRGD_GPIO		65
#define CB_T3_GP_LED_GPIO		162
#define CB_T3_nCHRGR_EN_GPIO		164

static int baseboard_is_cb_t3;
static int dvi_en_gpio = -EINVAL;

#if defined(CONFIG_SMSC911X) || defined(CONFIG_SMSC911X_MODULE)
#include <linux/smsc911x.h>
#include <plat/gpmc-smsc911x.h>

static struct omap_smsc911x_platform_data cm_t35_smsc911x_cfg = {
	.id		= 0,
	.cs             = CM_T35_SMSC911X_CS,
	.gpio_irq       = CM_T35_SMSC911X_GPIO,
	.gpio_reset     = -EINVAL,
	.flags		= SMSC911X_USE_32BIT | SMSC911X_SAVE_MAC_ADDRESS,
};

static struct omap_smsc911x_platform_data sb_t35_smsc911x_cfg = {
	.id		= 1,
	.cs             = SB_T35_SMSC911X_CS,
	.gpio_irq       = SB_T35_SMSC911X_GPIO,
	.gpio_reset     = -EINVAL,
	.flags		= SMSC911X_USE_32BIT | SMSC911X_SAVE_MAC_ADDRESS,
};

static void __init cm_t35_init_ethernet(unsigned char *mac)
{
	memcpy(cm_t35_smsc911x_cfg.mac, mac, 6);
	gpmc_smsc911x_init(&cm_t35_smsc911x_cfg);
}

static void __init sb_t35_init_ethernet(unsigned char *mac)
{
	memcpy(sb_t35_smsc911x_cfg.mac, mac, 6);
	gpmc_smsc911x_init(&sb_t35_smsc911x_cfg);
}
#else
static inline void cm_t35_init_ethernet(unsigned char *mac) {}
static inline void sb_t35_init_ethernet(unsigned char *mac) {}
#endif

#if defined(CONFIG_LEDS_GPIO) || defined(CONFIG_LEDS_GPIO_MODULE)
#include <linux/leds.h>

static struct gpio_led cm_t35_leds[] = {
	[0] = {
		.gpio			= 186,
		.name			= "cm-t35:green",
		.default_trigger	= "heartbeat",
		.active_low		= 0,
	},
};

static struct gpio_led_platform_data cm_t35_led_pdata = {
	.num_leds	= ARRAY_SIZE(cm_t35_leds),
	.leds		= cm_t35_leds,
};

static struct platform_device cm_t35_led_device = {
	.name		= "leds-gpio",
	.id		= 0,
	.dev		= {
		.platform_data	= &cm_t35_led_pdata,
	},
};

static void __init cm_t35_init_led(void)
{
	platform_device_register(&cm_t35_led_device);
}

static struct gpio_led cb_t3_leds[] = {
	{
		.gpio			= CB_T3_GP_LED_GPIO,
		.name			= "cb-t3:green",
		.default_trigger	= "mmc1",
		.active_low		= 1,
	},
};

static struct gpio_led_platform_data cb_t3_led_pdata = {
	.num_leds	= ARRAY_SIZE(cb_t3_leds),
	.leds		= cb_t3_leds,
};

static struct platform_device cb_t3_led_device = {
	.name		= "leds-gpio",
	.id		= 1,
	.dev		= {
		.platform_data	= &cb_t3_led_pdata,
	},
};

static void cb_t3_init_led(void)
{
	platform_device_register(&cb_t3_led_device);
}

#else
static inline void cm_t35_init_led(void) {}
static inline void cb_t3_init_led(void) {}
#endif

#if defined(CONFIG_TOUCHSCREEN_ADS7846)
#include <linux/spi/ads7846.h>

static struct ads7846_platform_data cm_t35_ads7846_config __initdata = {
	.x_max			= 0x0fff,
	.y_max			= 0x0fff,
	.x_plate_ohms		= 180,
	.pressure_max		= 255,
	.debounce_max		= 30,
	.debounce_tol		= 10,
	.debounce_rep		= 1,
	.keep_vref_on		= 1,
	.gpio_pendown		= CM_T35_GPIO_PENDOWN,
	.settle_delay_usecs	= 100,
};


static void __init cm_t35_init_touchscreen(void)
{
	omap_ads7846_init(1, CM_T35_GPIO_PENDOWN, 0, &cm_t35_ads7846_config);
}

#else
static inline void cm_t35_init_touchscreen(void) {}
#endif

#if defined(CONFIG_MTD_NAND_OMAP2) || defined(CONFIG_MTD_NAND_OMAP2_MODULE)
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>

static struct mtd_partition cm_t35_nand_partitions[] = {
	{
		.name           = "xloader",
		.offset         = 0,			/* Offset = 0x00000 */
		.size           = 4 * NAND_BLOCK_SIZE,
		.mask_flags     = MTD_WRITEABLE
	},
	{
		.name           = "uboot",
		.offset         = MTDPART_OFS_APPEND,	/* Offset = 0x80000 */
		.size           = 15 * NAND_BLOCK_SIZE,
	},
	{
		.name           = "uboot environment",
		.offset         = MTDPART_OFS_APPEND,	/* Offset = 0x260000 */
		.size           = 2 * NAND_BLOCK_SIZE,
	},
	{
		.name           = "linux",
		.offset         = MTDPART_OFS_APPEND,	/* Offset = 0x2A0000 */
		.size           = 32 * NAND_BLOCK_SIZE,
	},
	{
		.name           = "rootfs",
		.offset         = MTDPART_OFS_APPEND,	/* Offset = 0x6A0000 */
		.size           = MTDPART_SIZ_FULL,
	},
};

static struct omap_nand_platform_data cm_t35_nand_data = {
	.parts			= cm_t35_nand_partitions,
	.nr_parts		= ARRAY_SIZE(cm_t35_nand_partitions),
	.dma_channel		= -1,	/* disable DMA in OMAP NAND driver */
	.cs			= 0,
};

static void __init cm_t35_init_nand(void)
{
	int err = gpmc_nand_init(&cm_t35_nand_data);

	if (err)
		pr_err("CM-T3x: unable to register NAND device: %d\n", err);
}
#else
static inline void cm_t35_init_nand(void) {}
#endif

#define CM_T35_LCD_EN_GPIO 157
#define CM_T35_LCD_BL_GPIO 58
#define CM_T35_DVI_EN_GPIO 54

static int lcd_enabled;
static int dvi_enabled;

static int cm_t35_panel_enable_lcd(struct omap_dss_device *dssdev)
{
	if (dvi_enabled) {
		pr_err("CM-T3x: cannot enable LCD, DVI is enabled\n");
		return -EINVAL;
	}

/*	gpio_set_value(CM_T35_LCD_EN_GPIO, 1); */
	gpio_set_value(CM_T35_LCD_BL_GPIO, 1);

	lcd_enabled = 1;

	return 0;
}

static void cm_t35_panel_disable_lcd(struct omap_dss_device *dssdev)
{
	lcd_enabled = 0;

	gpio_set_value(CM_T35_LCD_BL_GPIO, 0);
/*	gpio_set_value(CM_T35_LCD_EN_GPIO, 0); */
}

static int cm_t35_panel_enable_dvi(struct omap_dss_device *dssdev)
{
	if (baseboard_is_cb_t3) {
		pr_err("CM-T3x: CB-T3 base board does not support DVI\n");
		return -EINVAL;
	}

	if (lcd_enabled) {
		pr_err("CM-T3x: cannot enable DVI, LCD is enabled\n");
		return -EINVAL;
	}

	if (gpio_is_valid(dvi_en_gpio))
		gpio_set_value(dvi_en_gpio, 0);

	dvi_enabled = 1;

	return 0;
}

static void cm_t35_panel_disable_dvi(struct omap_dss_device *dssdev)
{
	/*
	 * Even if we are running on cb-t3, we let the disable code through,
	 * so on any conditions there will be a possibility to disable the DVI
	 * and let the DSS state change.
	 */
	if (baseboard_is_cb_t3)
		pr_warn("CM-T3x: CB-T3 base board does not support DVI\n");

	if (gpio_is_valid(dvi_en_gpio))
		gpio_set_value(dvi_en_gpio, 1);

	dvi_enabled = 0;
}

static int cm_t35_panel_enable_tv(struct omap_dss_device *dssdev)
{
	return 0;
}

static void cm_t35_panel_disable_tv(struct omap_dss_device *dssdev)
{
}

static struct panel_generic_dpi_data lcd_panel = {
	.name			= "toppoly_tdo35s",
	.platform_enable	= cm_t35_panel_enable_lcd,
	.platform_disable	= cm_t35_panel_disable_lcd,
};

static struct omap_dss_device cm_t35_lcd_device = {
	.name			= "lcd",
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.driver_name		= "generic_dpi_panel",
	.data			= &lcd_panel,
	.phy.dpi.data_lines	= 18,
};

static struct panel_generic_dpi_data dvi_panel = {
	.name			= "generic",
	.platform_enable	= cm_t35_panel_enable_dvi,
	.platform_disable	= cm_t35_panel_disable_dvi,
};

static struct omap_dss_device cm_t35_dvi_device = {
	.name			= "dvi",
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.driver_name		= "generic_dpi_panel",
	.data			= &dvi_panel,
	.phy.dpi.data_lines	= 24,
};

static struct omap_dss_device cm_t35_tv_device = {
	.name			= "tv",
	.driver_name		= "venc",
	.type			= OMAP_DISPLAY_TYPE_VENC,
	.phy.venc.type		= OMAP_DSS_VENC_TYPE_SVIDEO,
	.platform_enable	= cm_t35_panel_enable_tv,
	.platform_disable	= cm_t35_panel_disable_tv,
};

static struct omap_dss_device *cm_t35_dss_devices[] = {
	&cm_t35_lcd_device,
	&cm_t35_dvi_device,
	&cm_t35_tv_device,
};

static struct omap_dss_board_info cm_t35_dss_data = {
	.num_devices	= ARRAY_SIZE(cm_t35_dss_devices),
	.devices	= cm_t35_dss_devices,
	.default_device	= &cm_t35_dvi_device,
};

static struct omap2_mcspi_device_config tdo24m_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel	= 1,	/* 0: slave, 1: master */
};

static struct tdo24m_platform_data tdo24m_config = {
	.model = TDO35S,
};

static struct spi_board_info cm_t35_lcd_spi_board_info[] __initdata = {
	{
		.modalias		= "tdo24m",
		.bus_num		= 4,
		.chip_select		= 0,
		.max_speed_hz		= 1000000,
		.controller_data	= &tdo24m_mcspi_config,
		.platform_data		= &tdo24m_config,
	},
};

static struct gpio cm_t35_dss_gpios[] __initdata = {
	{ CM_T35_LCD_EN_GPIO, GPIOF_OUT_INIT_LOW,  "lcd enable"    },
	{ CM_T35_LCD_BL_GPIO, GPIOF_OUT_INIT_LOW,  "lcd bl enable" },
};

static void __init cm_t35_init_display(void)
{
	int err;

	spi_register_board_info(cm_t35_lcd_spi_board_info,
				ARRAY_SIZE(cm_t35_lcd_spi_board_info));

	err = gpio_request_array(cm_t35_dss_gpios,
				 ARRAY_SIZE(cm_t35_dss_gpios));
	if (err) {
		pr_err("CM-T3x: DSS control GPIOs request failed: %d\n", err);
		return;
	}

	gpio_export(CM_T35_LCD_EN_GPIO, 0);
	gpio_export(CM_T35_LCD_BL_GPIO, 0);

	msleep(50);
	gpio_set_value(CM_T35_LCD_EN_GPIO, 1);

	err = omap_display_init(&cm_t35_dss_data);
	if (err) {
		pr_err("CM-T3x: failed to register DSS device: %d\n", err);
		gpio_free_array(cm_t35_dss_gpios, ARRAY_SIZE(cm_t35_dss_gpios));
	}
}

static struct regulator_consumer_supply cm_t35_vmmc1_supply =
	REGULATOR_SUPPLY("vmmc", "omap_hsmmc.0");

static struct regulator_consumer_supply cm_t35_vdac_supply =
	REGULATOR_SUPPLY("vdda_dac", "omapdss_venc");

static struct regulator_consumer_supply cm_t35_vio_supplies[] = {
	REGULATOR_SUPPLY("vcc", "spi1.0"),
	REGULATOR_SUPPLY("vdds_dsi", "omapdss"),
	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dsi1"),
};

static struct regulator_consumer_supply cm_t3730_vaux2_supplies[] = {
	REGULATOR_SUPPLY("vmmc_aux", "omap_hsmmc.1"),
};

/* VAUX2 for the WL12xx combo chip */
static struct regulator_init_data cm_t3730_vaux2 = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= ARRAY_SIZE(cm_t3730_vaux2_supplies),
	.consumer_supplies	= cm_t3730_vaux2_supplies,
};

/* VMMC1 for MMC1 pins CMD, CLK, DAT0..DAT3 (20 mA, plus card == max 220 mA) */
static struct regulator_init_data cm_t35_vmmc1 = {
	.constraints = {
		.min_uV			= 1850000,
		.max_uV			= 3150000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &cm_t35_vmmc1_supply,
};

/* VDAC for DSS driving S-Video (8 mA unloaded, max 65 mA) */
static struct regulator_init_data cm_t35_vdac = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &cm_t35_vdac_supply,
};

static struct regulator_init_data cm_t35_vio = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE,
	},
	.num_consumer_supplies	= ARRAY_SIZE(cm_t35_vio_supplies),
	.consumer_supplies	= cm_t35_vio_supplies,
};

static struct twl4030_usb_data cm_t35_usb_data = {
	.usb_mode	= T2_USB_MODE_ULPI,
};

static uint32_t cm_t35_keymap[] = {
	KEY(0, 0, KEY_A),	KEY(0, 1, KEY_B),	KEY(0, 2, KEY_LEFT),
	KEY(1, 0, KEY_UP),	KEY(1, 1, KEY_ENTER),	KEY(1, 2, KEY_DOWN),
	KEY(2, 0, KEY_RIGHT),	KEY(2, 1, KEY_C),	KEY(2, 2, KEY_D),
};

static struct matrix_keymap_data cm_t35_keymap_data = {
	.keymap			= cm_t35_keymap,
	.keymap_size		= ARRAY_SIZE(cm_t35_keymap),
};

static struct twl4030_keypad_data cm_t35_kp_data = {
	.keymap_data	= &cm_t35_keymap_data,
	.rows		= 3,
	.cols		= 3,
	.rep		= 1,
};

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,

	},
	{
		.mmc		= 2,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.nonremovable	= true,
	},
	{}	/* Terminator */
};

static struct usbhs_omap_board_data usbhs_bdata __initdata = {
	.port_mode[0] = OMAP_EHCI_PORT_MODE_PHY,
	.port_mode[1] = OMAP_EHCI_PORT_MODE_PHY,
	.port_mode[2] = OMAP_USBHS_PORT_MODE_UNUSED,

	.phy_reset  = true,
	.reset_gpio_port[0]  = OMAP_MAX_GPIO_LINES + 6,
	.reset_gpio_port[1]  = OMAP_MAX_GPIO_LINES + 7,
	.reset_gpio_port[2]  = -EINVAL
};

static void __init cm_t35_init_usbh(void)
{
	int err;

	err = gpio_request_one(SB_T35_USB_HUB_RESET_GPIO,
			       GPIOF_OUT_INIT_LOW, "usb hub rst");
	if (err) {
		pr_err("SB-T35: usb hub rst gpio request failed: %d\n", err);
	} else {
		udelay(10);
		gpio_set_value(SB_T35_USB_HUB_RESET_GPIO, 1);
		msleep(1);
	}

	usbhs_init(&usbhs_bdata);
}

#if defined(CONFIG_LIBERTAS_SDIO) || defined(CONFIG_LIBERTAS_SDIO_MODULE)
static void cm_t35_init_wlan(unsigned gpio)
{
	int err;

	err = gpio_request_one(gpio, GPIOF_OUT_INIT_HIGH, "wlan rst");
	if (err) {
		pr_err("CM-T3x: WiFi reset gpio request failed: %d\n", err);
		return;
	}
	gpio_export(gpio, 0);

	mmc[1].ocr_mask = MMC_VDD_32_33;
	udelay(10);
	gpio_set_value_cansleep(gpio, 0);
	udelay(10);
	gpio_set_value_cansleep(gpio, 1);
}
#else
static inline void cm_t35_init_wlan(unsigned gpio) {}
#endif /* CONFIG_LIBERTAS_SDIO */

#if defined(CONFIG_WL12XX_SDIO) || defined(CONFIG_WL12XX_SDIO_MODULE)
#define CM_T3730_WLAN_IRQ	136
#define CM_T3730_WLAN_EN	73

static void __init cm_t3730_init_wlan_mux(void)
{
	omap_mux_init_gpio(CM_T3730_WLAN_IRQ, OMAP_MUX_MODE4 | OMAP_PIN_INPUT);
	omap_mux_init_gpio(CM_T3730_WLAN_EN, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT);
}

static struct wl12xx_platform_data cm_t3730_wlan_pdata = {
	.irq = OMAP_GPIO_IRQ(CM_T3730_WLAN_IRQ),
	.board_ref_clock = WL12XX_REFCLOCK_38,
};

static struct regulator_consumer_supply cm_t3730_vmmc2_supply[] = {
	REGULATOR_SUPPLY("vmmc", "omap_hsmmc.1"),
};

/* VMMC2 for the WL12xx combo chip */
static struct regulator_init_data cm_t3730_vmmc2 = {
	.constraints = {
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies = cm_t3730_vmmc2_supply,
};

static struct fixed_voltage_config cm_t3730_vwlan = {
	.supply_name		= "vwl1271",
	.microvolts		= 1800000, /* 1.80V */
	.gpio			= CM_T3730_WLAN_EN,
	.startup_delay		= 20000, /* 20ms */
	.enable_high		= 1,
	.enabled_at_boot	= 0,
	.init_data		= &cm_t3730_vmmc2,
};

static struct platform_device cm_t3730_wlan_regulator = {
	.name		= "reg-fixed-voltage",
	.id		= 0,
	.dev = {
		.platform_data	= &cm_t3730_vwlan,
	},
};

static struct gpio cm_t3730_wlan_gpios[] = {
	{ -EINVAL, GPIOF_OUT_INIT_HIGH, "wlan pwr" },
	{ CM_T3730_WLAN_IRQ, GPIOF_IN,  "wlan irq" },
};

static void cm_t3730_init_wlan(unsigned gpio)
{
	int err;

	cm_t3730_wlan_gpios[0].gpio = gpio;
	err = gpio_request_array(cm_t3730_wlan_gpios,
				 ARRAY_SIZE(cm_t3730_wlan_gpios));
	if (err) {
		pr_err("CM-T3730: WLAN irq gpio request failed: %d\n", err);
		return;
	}
	gpio_export(CM_T3730_WLAN_EN, 0);

	err = wl12xx_set_platform_data(&cm_t3730_wlan_pdata);
	if (err) {
		pr_err("CM-T3730: wl12xx pdata set failed: %d\n", err);
		goto gpio_free;
	}

	err = platform_device_register(&cm_t3730_wlan_regulator);
	if (err) {
		pr_err("CM-T3730: WLAN EN regulator register failed:%d\n", err);
		goto gpio_free;
	}

	mmc[1].name = "wl1271";
	mmc[1].caps |= MMC_CAP_POWER_OFF_CARD;

	return;

gpio_free:
	gpio_free_array(cm_t3730_wlan_gpios, ARRAY_SIZE(cm_t3730_wlan_gpios));
}
#else
static inline void cm_t3730_init_wlan_mux(void) {}
static inline void cm_t3730_init_wlan(unsigned gpio) {}
#endif /* CONFIG_WL12XX_SDIO */

/* Backup Battery config register */
#define BB_CFG_REG     0x12
/* Charging configuration */
#define VOLTAGE_3V     0x01
#define CURRENT_1mA    0x03
#define CHARGER_EN     0x01

static void __init cm_t35_init_charge(void)
{
	int err;
	u8 reg_val = (CHARGER_EN << 4) | (VOLTAGE_3V << 2) | (CURRENT_1mA);

	err = twl_i2c_write_u8(TWL_MODULE_PM_RECEIVER, reg_val, BB_CFG_REG);
	if (err)
		pr_err("CM-T3x: backup battery charger init failed: %d\n", err);
}

static int cm_t3x_twl_gpio_setup(unsigned gpio)
{
	/* gpio + 0 is "mmc0_cd" (input/IRQ) */
	mmc[0].gpio_cd = gpio + 0;
	omap2_hsmmc_init(mmc);

	cm_t35_init_charge();

	return 0;
}

static int cm_t35_twl_gpio_setup(struct device *dev, unsigned gpio,
				 unsigned ngpio)
{
	cm_t35_init_wlan(gpio + 2);
	return cm_t3x_twl_gpio_setup(gpio);
}

static int cm_t3730_twl_gpio_setup(struct device *dev, unsigned gpio,
				   unsigned ngpio)
{
	cm_t3730_init_wlan(gpio + 2);
	return cm_t3x_twl_gpio_setup(gpio);
}

static struct twl4030_gpio_platform_data cm_t35_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
};

static struct twl4030_codec_audio_data cm_t35_audio_data;

static struct twl4030_codec_data cm_t35_codec_data = {
	.audio_mclk = 26000000,
	.audio = &cm_t35_audio_data,
};

static struct twl4030_madc_platform_data cm_t35_madc_data = {
	.irq_line	= 1,
};

static struct twl4030_platform_data cm_t35_twldata = {
	.irq_base	= TWL4030_IRQ_BASE,
	.irq_end	= TWL4030_IRQ_END,

	/* platform_data for children goes here */
	.keypad		= &cm_t35_kp_data,
	.usb		= &cm_t35_usb_data,
	.gpio		= &cm_t35_gpio_data,
	.codec		= &cm_t35_codec_data,
	.vmmc1		= &cm_t35_vmmc1,
	.vdac		= &cm_t35_vdac,
	.vio		= &cm_t35_vio,
	.madc		= &cm_t35_madc_data,
	.vaux2		= &cm_t3730_vaux2,
};

static struct platform_device cm_t35_madc_hwmon = {
	.name	= "twl4030_madc_hwmon",
};

/* EEPROM layout */
#define EEPROM_1ST_MAC_OFF		4
#define EEPROM_1ST_MAC_LEGACY_OFF	0
#define EEPROM_LAYOUT_VER_OFF		44
#define EEPROM_LAYOUT_VER_LEN		1
#define EEPROM_BOARD_NAME_OFF		128
#define EEPROM_BOARD_NAME_LEN		16

static int eeprom_read(struct memory_accessor *mem_acc, unsigned char *buf,
		       int offset, int size, const char* objname)
{
	ssize_t ret;

	ret = mem_acc->read(mem_acc, buf, offset, size);
	if (ret != size) {
		pr_warn("CM-T3x: EEPROM %s read failed: %d\n", objname, ret);
		return ret;
	}

	return 0;
}

static void eeprom_read_layout_version(struct memory_accessor *mem_acc,
				       unsigned char *layout)
{
	char *objname = "layout version";

	if (eeprom_read(mem_acc, layout, EEPROM_LAYOUT_VER_OFF,
			EEPROM_LAYOUT_VER_LEN, objname))
		memset(layout, 0xFF, EEPROM_LAYOUT_VER_LEN);
}

static void eeprom_read_mac_address(struct memory_accessor *mem_acc,
				    unsigned char *mac)
{
	char *objname = "MAC address";
	int offset = EEPROM_1ST_MAC_OFF;
	unsigned char layout;

	eeprom_read_layout_version(mem_acc, &layout);
	if (layout >= 0x20)
		offset = EEPROM_1ST_MAC_LEGACY_OFF;

	if (eeprom_read(mem_acc, mac, offset, ETH_ALEN, objname))
		memset(mac, 0, ETH_ALEN);
}

static void eeprom_read_board_name(struct memory_accessor *mem_acc,
				   unsigned char *name)
{
	char *objname = "board name";

	if (eeprom_read(mem_acc, name, EEPROM_BOARD_NAME_OFF,
			EEPROM_BOARD_NAME_LEN, objname))
		memset(name, 0, EEPROM_BOARD_NAME_LEN);
}

static void cm_t35_eeprom_setup(struct memory_accessor *mem_acc, void *context)
{
	unsigned char mac[ETH_ALEN];

	eeprom_read_mac_address(mem_acc, mac);
	cm_t35_init_ethernet(mac);
}

static struct at24_platform_data cm_t35_eeprom_pdata = {
	.byte_len	= 256,
	.page_size	= 16,
	.setup		= cm_t35_eeprom_setup,
};

static struct i2c_board_info cm_t35_i2c1_eeprom_info __initdata = {
	I2C_BOARD_INFO("at24", 0x50),
	.platform_data = &cm_t35_eeprom_pdata,
};

static void sb_t35_init_mux(void)
{
	/* nCS and IRQ mux for SB-T35 ethernet */
	omap_mux_init_signal("gpmc_ncs4", OMAP_MUX_MODE0);
	omap_mux_init_gpio(SB_T35_SMSC911X_GPIO,
			   OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP);
}

static void sb_t35_init(struct memory_accessor *mem_acc)
{
	unsigned char mac[ETH_ALEN];
	int dvi_en_gpio_flags = GPIOF_OUT_INIT_HIGH, err;

	sb_t35_init_mux();
	eeprom_read_mac_address(mem_acc, mac);
	sb_t35_init_ethernet(mac);

	if (dvi_enabled)
		dvi_en_gpio_flags = GPIOF_OUT_INIT_LOW;

	err = gpio_request_one(CM_T35_DVI_EN_GPIO, dvi_en_gpio_flags, "dvi en");
	if (err) {
		pr_err("CM-T3x: DVI reset GPIO request failed: %d\n", err);
		return;
	}
	gpio_export(CM_T35_DVI_EN_GPIO, 0);
	dvi_en_gpio = CM_T35_DVI_EN_GPIO;
}

static void cb_t3_init_mux(void)
{
	int mux_mode = OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT;

	/* Stereo enable */
	omap_mux_init_gpio(CB_T3_nHP_GPIO, mux_mode);
	/* General purpose LED */
	omap_mux_init_gpio(CB_T3_GP_LED_GPIO, mux_mode);
	/* Charger enable */
	omap_mux_init_gpio(CB_T3_nCHRGR_EN_GPIO, mux_mode);

	mux_mode = OMAP_MUX_MODE4 | OMAP_PIN_INPUT;

	/* Power good */
	omap_mux_init_gpio(CB_T3_nPWRGD_GPIO, mux_mode);
}

static void cb_t3_init(void)
{
	baseboard_is_cb_t3 = 1;
	cb_t3_init_mux();
	cb_t3_init_led();
}

static void baseboard_eeprom_setup(struct memory_accessor *mem_acc,
				   void *context)
{
	unsigned char baseboard[EEPROM_BOARD_NAME_LEN];

	eeprom_read_board_name(mem_acc, baseboard);
	if (strncmp(baseboard, "CB-T3", EEPROM_BOARD_NAME_LEN) == 0)
		cb_t3_init();
	else
		sb_t35_init(mem_acc);
}

static struct at24_platform_data baseboard_eeprom_pdata = {
	.byte_len       = 256,
	.page_size      = 16,
	.setup          = baseboard_eeprom_setup,
};

#define DS2786_RSNS	18 /* Constant sense resistor value */
struct ds278x_platform_data cm_t35_ds278x_pdata = {
	.rsns = DS2786_RSNS,
};

static struct i2c_board_info cm_t35_i2c3_boardinfo[] __initdata = {
	{
		I2C_BOARD_INFO("at24", 0x50),
		.platform_data = &baseboard_eeprom_pdata,
	},
	{
		I2C_BOARD_INFO("ds2786", 0x36),
		.platform_data = &cm_t35_ds278x_pdata,
	},
};

#if defined(CONFIG_VIDEO_OMAP3) || defined(CONFIG_VIDEO_OMAP3_MODULE)
#include <media/omap3isp.h>

#if defined(CONFIG_VIDEO_MT9T001) || defined(CONFIG_VIDEO_MT9T001_MODULE)
#define CAM_PCA9543APW_ADDR		0x73
#define CAM_PCA9543APW_CTLREG		0x0
#define CAM_PCA9543APW_CTLREG_B0	(1 << 0)

static void mt9t001_i2c_evalboard_setup(struct i2c_client *client)
{
	union i2c_smbus_data val = { .byte = CAM_PCA9543APW_CTLREG_B0 };
	int err;

	err = i2c_smbus_xfer(client->adapter, CAM_PCA9543APW_ADDR, 0,
				I2C_SMBUS_WRITE, CAM_PCA9543APW_CTLREG,
				I2C_SMBUS_BYTE_DATA, &val);
	if (err)
		pr_err("CM-T3x: failed to set camera muxer: %d\n", err);
}

static struct mt9t001_platform_data cm_t35_mt9t001_pdata = {
	.custom_setup = mt9t001_i2c_evalboard_setup,
};
#else
static struct mt9t001_platform_data cm_t35_mt9t001_pdata = {};
#endif /* CONFIG_VIDEO_MT9T001 */

static struct i2c_board_info cm_t35_isp_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("mt9t001", 0x5d),
		.platform_data = &cm_t35_mt9t001_pdata,

	},
	{
		I2C_BOARD_INFO("tvp5150", 0x5c),
	},
};

static struct isp_subdev_i2c_board_info cm_t35_isp_primary_subdevs[] = {
	{
		.board_info = &cm_t35_isp_i2c_boardinfo[0],
		.i2c_adapter_id = 3,
	},
	{ NULL, 0, },
};

static struct isp_subdev_i2c_board_info cm_t35_isp_secondary_subdevs[] = {
	{
		.board_info = &cm_t35_isp_i2c_boardinfo[1],
		.i2c_adapter_id = 3,
	},
	{ NULL, 0, },
};

static struct isp_wbal_pdata cm_t35_isp_mt9t001_wbal = {
	.dgain = 0x200,
	.coef0 = 0x1C,
	.coef1 = 0x1C,
	.coef2 = 0x2C,
	.coef3 = 0x1C,
};

static struct isp_v4l2_subdevs_group cm_t35_isp_subdevs[] = {
	{
		.subdevs = cm_t35_isp_primary_subdevs,
		.wbal = &cm_t35_isp_mt9t001_wbal,
		.interface = ISP_INTERFACE_PARALLEL,
		.bus = {
			.parallel = {
				.clk_pol	= 1,
			},
		},
	},
	{
		.subdevs = cm_t35_isp_secondary_subdevs,
		.interface = ISP_INTERFACE_PARALLEL,
		.bus = {
			.parallel = {
				.fldmode	= 1,
				.bt656		= 1,
			},
		},
	},
	{ NULL, 0, },
};

static struct isp_platform_data cm_t35_isp_pdata = {
	.subdevs = cm_t35_isp_subdevs,
};

static void __init cm_t35_init_camera(void)
{
	int err = omap3_init_camera(&cm_t35_isp_pdata);

	if (err)
		pr_warn("CM-T3x: failed registering camera device: %d\n", err);
}

#else
static inline void cm_t35_init_camera(void) {}
#endif /* CONFIG_VIDEO_OMAP3 */

static void __init cm_t35_init_i2c(void)
{
	int err;

	err = platform_device_register(&cm_t35_madc_hwmon);
	if (err)
		pr_err("CM-T3x: failed registering MADC HWMON: %d\n", err);

	err = i2c_register_board_info(1, &cm_t35_i2c1_eeprom_info, 1);
	if (err)
		pr_err("CM-T3x: failed registering EEPROM: %d\n", err);

	omap_pmic_init(1, 400, "tps65930", INT_34XX_SYS_NIRQ, &cm_t35_twldata);

	omap_register_i2c_bus(3, 400, cm_t35_i2c3_boardinfo,
			      ARRAY_SIZE(cm_t35_i2c3_boardinfo));
}

static void __init cm_t3730_opp_enable(const char *hwmod_name,
				       unsigned long *freqs)
{
	struct omap_hwmod *hwmod;
	struct device *dev;
	int err, i;

	hwmod = omap_hwmod_lookup(hwmod_name);
	if (!hwmod) {
		pr_err("CM-T3x: can't find %s hwmod: %p\n", hwmod_name, hwmod);
		return;
	}

	dev = &hwmod->od->pdev.dev;

	for (i = 0; freqs[i]; i++) {
		err = opp_enable(dev, freqs[i]);
		if (err) {
			pr_err("CM-T3x: failed enabling %s %luMHz: %d\n",
			       hwmod_name, freqs[i], err);
			return;
		}
	}
}

static void __init cm_t3730_opp_init(void)
{
	/* MPU 800MHz and 1GHz */
	unsigned long mpu_freqs[3] = {800000000, 1000000000, 0};
	/* IVA 660MHz and 800MHz */
	unsigned long iva_freqs[3] = {660000000,  800000000, 0};

	/* TODO: MPU 1GHz needs ABB */
	cm_t3730_opp_enable("mpu", mpu_freqs);
	cm_t3730_opp_enable("iva", iva_freqs);
}

static void __init cm_t3x_init_opp(void)
{
	int err = omap3_opp_init();

	if (err)
		pr_err("CM-T3x: opp default init failed: %d\n", err);
}

static void __init cm_t35_init_early(void)
{
	omap2_init_common_infrastructure();
	omap2_init_common_devices(mt46h32m32lf6_sdrc_params,
			     mt46h32m32lf6_sdrc_params);
}

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	/* nCS and IRQ for CM-T35 ethernet */
	OMAP3_MUX(GPMC_NCS5, OMAP_MUX_MODE0),
	OMAP3_MUX(UART3_CTS_RCTX, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP),

	/* PENDOWN GPIO */
	OMAP3_MUX(GPMC_NCS6, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),

	/* mUSB */
	OMAP3_MUX(HSUSB0_CLK, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(HSUSB0_STP, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(HSUSB0_DIR, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(HSUSB0_NXT, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(HSUSB0_DATA0, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(HSUSB0_DATA1, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(HSUSB0_DATA2, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(HSUSB0_DATA3, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(HSUSB0_DATA4, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(HSUSB0_DATA5, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(HSUSB0_DATA6, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(HSUSB0_DATA7, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),

	/* USB hub reset */
	OMAP3_MUX(CAM_WEN, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),

	/* McSPI 1 */
	OMAP3_MUX(MCSPI1_CLK, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(MCSPI1_SIMO, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(MCSPI1_SOMI, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(MCSPI1_CS0, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLDOWN),

	/* McSPI 4 */
	OMAP3_MUX(MCBSP1_CLKR, OMAP_MUX_MODE1 | OMAP_PIN_INPUT),
	OMAP3_MUX(MCBSP1_DX, OMAP_MUX_MODE1 | OMAP_PIN_INPUT),
	OMAP3_MUX(MCBSP1_DR, OMAP_MUX_MODE1 | OMAP_PIN_INPUT),
	OMAP3_MUX(MCBSP1_FSX, OMAP_MUX_MODE1 | OMAP_PIN_INPUT_PULLUP),

	/* McBSP 2 */
	OMAP3_MUX(MCBSP2_FSX, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(MCBSP2_CLKX, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(MCBSP2_DR, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(MCBSP2_DX, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),

	/* serial ports */
	OMAP3_MUX(MCBSP3_CLKX, OMAP_MUX_MODE1 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(MCBSP3_FSX, OMAP_MUX_MODE1 | OMAP_PIN_INPUT),
	OMAP3_MUX(UART1_TX, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(UART1_RX, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(UART1_RTS, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(UART1_CTS, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),

	/* common DSS */
	OMAP3_MUX(DSS_PCLK, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_HSYNC, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_VSYNC, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_ACBIAS, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA6, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA7, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA8, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA9, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA10, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA11, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA12, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA13, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA14, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA15, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA16, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA17, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),

	/* Camera */
	OMAP3_MUX(CAM_HS, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(CAM_VS, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(CAM_XCLKA, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(CAM_PCLK, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(CAM_FLD, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(CAM_D0, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(CAM_D1, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(CAM_D2, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(CAM_D3, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(CAM_D4, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(CAM_D5, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(CAM_D6, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(CAM_D7, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(CAM_D8, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLDOWN),
	OMAP3_MUX(CAM_D9, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLDOWN),
	OMAP3_MUX(CAM_STROBE, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),

	OMAP3_MUX(CAM_D10, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLDOWN),
	OMAP3_MUX(CAM_D11, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLDOWN),

	/* display controls */
	OMAP3_MUX(MCBSP1_FSR, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(GPMC_NCS7, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(GPMC_NCS3, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),

	/* TPS IRQ */
	OMAP3_MUX(SYS_NIRQ, OMAP_MUX_MODE0 | OMAP_WAKEUP_EN | \
		  OMAP_PIN_INPUT_PULLUP),

	/* I2C1 */
	OMAP3_MUX(I2C1_SCL, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP3_MUX(I2C1_SDA, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),

	/* I2C3 */
	OMAP3_MUX(I2C3_SCL, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP3_MUX(I2C3_SCL, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),

	{ .reg_offset = OMAP_MUX_TERMINATOR },
};

static void __init cm_t3x_common_dss_mux_init(int mux_mode)
{
	omap_mux_init_signal("dss_data18", mux_mode);
	omap_mux_init_signal("dss_data19", mux_mode);
	omap_mux_init_signal("dss_data20", mux_mode);
	omap_mux_init_signal("dss_data21", mux_mode);
	omap_mux_init_signal("dss_data22", mux_mode);
	omap_mux_init_signal("dss_data23", mux_mode);
}

static void __init cm_t35_init_mux(void)
{
	int mux_mode = OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT;

	omap_mux_init_signal("dss_data0.dss_data0", mux_mode);
	omap_mux_init_signal("dss_data1.dss_data1", mux_mode);
	omap_mux_init_signal("dss_data2.dss_data2", mux_mode);
	omap_mux_init_signal("dss_data3.dss_data3", mux_mode);
	omap_mux_init_signal("dss_data4.dss_data4", mux_mode);
	omap_mux_init_signal("dss_data5.dss_data5", mux_mode);
	cm_t3x_common_dss_mux_init(mux_mode);

	/* MMC 2 */
	mux_mode = OMAP_MUX_MODE1 | OMAP_PIN_OUTPUT;
	omap_mux_init_signal("sdmmc2_dir_dat0", mux_mode);
	omap_mux_init_signal("sdmmc2_dir_dat1", mux_mode);
	omap_mux_init_signal("sdmmc2_dir_cmd", mux_mode);
	omap_mux_init_signal("sdmmc2_clkin", OMAP_MUX_MODE1 | OMAP_PIN_INPUT);
}

static void __init cm_t3730_init_mux(void)
{
	int mux_mode = OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT;

	omap_mux_init_signal("sys_boot0", mux_mode);
	omap_mux_init_signal("sys_boot1", mux_mode);
	omap_mux_init_signal("sys_boot3", mux_mode);
	omap_mux_init_signal("sys_boot4", mux_mode);
	omap_mux_init_signal("sys_boot5", mux_mode);
	omap_mux_init_signal("sys_boot6", mux_mode);
	cm_t3x_common_dss_mux_init(mux_mode);
}
#else
static inline void cm_t35_init_mux(void) {}
static inline void cm_t3730_init_mux(void) {}
#endif

static struct omap_board_config_kernel cm_t35_config[] __initdata = {
};

static void __init cm_t3x_common_init(void)
{
	omap_board_config = cm_t35_config;
	omap_board_config_size = ARRAY_SIZE(cm_t35_config);
	omap3_mux_init(board_mux, OMAP_PACKAGE_CUS);
	omap_serial_init();
	cm_t3x_init_opp();
	cm_t35_init_i2c();
	cm_t35_init_touchscreen();
	cm_t35_init_led();
	cm_t35_init_display();
	cm_t35_init_nand();

	usb_musb_init(NULL);
	cm_t35_init_usbh();
	cm_t35_init_camera();
}

static void __init cm_t35_init(void)
{
	cm_t3x_common_init();
	cm_t35_init_mux();
	cm_t35_gpio_data.setup = cm_t35_twl_gpio_setup;
}

static void __init cm_t3730_init(void)
{
	cm_t3x_common_init();
	cm_t3730_opp_init();
	cm_t3730_init_mux();

	if ((system_rev & 0xffff) < 120) {
		cm_t35_gpio_data.setup = cm_t35_twl_gpio_setup;
	} else {
		cm_t35_gpio_data.setup = cm_t3730_twl_gpio_setup;
		cm_t3730_init_wlan_mux();
	}
}

MACHINE_START(CM_T35, "Compulab CM-T35")
	.boot_params	= 0x80000100,
	.reserve	= omap_reserve,
	.map_io		= omap3_map_io,
	.init_early	= cm_t35_init_early,
	.init_irq	= omap_init_irq,
	.init_machine	= cm_t35_init,
	.timer		= &omap_timer,
MACHINE_END

MACHINE_START(CM_T3730, "Compulab CM-T3730")
	.boot_params    = 0x80000100,
	.reserve        = omap_reserve,
	.map_io         = omap3_map_io,
	.init_early     = cm_t35_init_early,
	.init_irq       = omap_init_irq,
	.init_machine   = cm_t3730_init,
	.timer          = &omap_timer,
MACHINE_END
