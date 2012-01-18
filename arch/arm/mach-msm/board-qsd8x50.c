/* Copyright (c) 2009-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/android_pmem.h>
#include <linux/bootmem.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/usb/msm_hsusb.h>
#include <linux/err.h>
#include <linux/clkdev.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/io.h>
#include <asm/setup.h>

#include <asm/mach/mmc.h>
#include <mach/vreg.h>
#include <mach/mpp.h>
#include <mach/board.h>
#include <mach/sirc.h>
#include <mach/vreg.h>
#include <mach/mmc.h>

#include "devices.h"
#include "timer.h"
#include "socinfo.h"
#include "msm-keypad-devices.h"
#include "pm.h"
#include "proc_comm.h"
#include <linux/msm_kgsl.h>
#ifdef CONFIG_USB_ANDROID
#include <linux/usb/android_composite.h>
#endif

#define TOUCHPAD_SUSPEND 	34
#define TOUCHPAD_IRQ 		38

#define MSM_PMEM_SF_SIZE	0x1700000

#define SMEM_SPINLOCK_I2C	"S:6"

#define MSM_PMEM_ADSP_SIZE	0x2A05000
#define MSM_FB_SIZE         0x2EE000
#define MSM_AUDIO_SIZE		0x80000
#define MSM_GPU_PHYS_SIZE 	SZ_2M

#ifdef CONFIG_MSM_SOC_REV_A
#define MSM_SMI_BASE		0xE0000000
#else
#define MSM_SMI_BASE		0x00000000
#endif

#define MSM_SHARED_RAM_PHYS	(MSM_SMI_BASE + 0x00100000)

static const resource_size_t qsd8x50_surf_smc91x_base __initdata = 0x70000300;
static const unsigned        qsd8x50_surf_smc91x_gpio __initdata = 156;

/* Leave smc91x resources empty here, as we'll fill them in
 * at run-time: they vary from board to board, and the true
 * configuration won't be known until boot.
 */
static struct resource smc91x_resources[] = {
	[0] = {
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device smc91x_device = {
	.name           = "smc91x",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(smc91x_resources),
	.resource       = smc91x_resources,
};

static int __init msm_init_smc91x(void)
{
	if (machine_is_qsd8x50_surf()) {
		smc91x_resources[0].start = qsd8x50_surf_smc91x_base;
		smc91x_resources[0].end   = qsd8x50_surf_smc91x_base + 0xff;
		smc91x_resources[1].start =
			gpio_to_irq(qsd8x50_surf_smc91x_gpio);
		smc91x_resources[1].end   =
			gpio_to_irq(qsd8x50_surf_smc91x_gpio);
		platform_device_register(&smc91x_device);
	}

	return 0;
}
module_init(msm_init_smc91x);

static int hsusb_phy_init_seq[] = {
	0x08, 0x31,	/* Increase HS Driver Amplitude */
	0x20, 0x32,	/* Enable and set Pre-Emphasis Depth to 10% */
	-1
};

static struct msm_otg_platform_data msm_otg_pdata = {
	.phy_init_seq		= hsusb_phy_init_seq,
	.mode                   = USB_PERIPHERAL,
	.otg_control		= OTG_PHY_CONTROL,
};

static struct platform_device *devices[] __initdata = {
	&msm_device_uart3,
	&msm_device_smd,
	&msm_device_otg,
	&msm_device_hsusb,
	&msm_device_hsusb_host,
};

static struct msm_mmc_gpio sdc1_gpio_cfg[] = {
	{51, "sdc1_dat_3"},
	{52, "sdc1_dat_2"},
	{53, "sdc1_dat_1"},
	{54, "sdc1_dat_0"},
	{55, "sdc1_cmd"},
	{56, "sdc1_clk"}
};

static struct vreg *vreg_mmc;
static unsigned long vreg_sts;

static uint32_t msm_sdcc_setup_power(struct device *dv, unsigned int vdd)
{
	int rc = 0;
	struct platform_device *pdev;

	pdev = container_of(dv, struct platform_device, dev);

	if (vdd == 0) {
		if (!vreg_sts)
			return 0;

		clear_bit(pdev->id, &vreg_sts);

		if (!vreg_sts) {
			rc = vreg_disable(vreg_mmc);
			if (rc)
				pr_err("vreg_mmc disable failed for slot "
						"%d: %d\n", pdev->id, rc);
		}
		return 0;
	}

	if (!vreg_sts) {
		rc = vreg_set_level(vreg_mmc, 2900);
		if (rc)
			pr_err("vreg_mmc set level failed for slot %d: %d\n",
					pdev->id, rc);
		rc = vreg_enable(vreg_mmc);
		if (rc)
			pr_err("vreg_mmc enable failed for slot %d: %d\n",
					pdev->id, rc);
	}
	set_bit(pdev->id, &vreg_sts);
	return 0;
}

static struct msm_mmc_gpio_data sdc1_gpio = {
	.gpio = sdc1_gpio_cfg,
	.size = ARRAY_SIZE(sdc1_gpio_cfg),
};

static struct msm_mmc_platform_data qsd8x50_sdc1_data = {
	.ocr_mask	= MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd	= msm_sdcc_setup_power,
	.gpio_data = &sdc1_gpio,
};

static void __init qsd8x50_init_mmc(void)
{
	vreg_mmc = vreg_get(NULL, "gp5");

	if (IS_ERR(vreg_mmc)) {
		pr_err("vreg get for vreg_mmc failed (%ld)\n",
				PTR_ERR(vreg_mmc));
		return;
	}

	msm_add_sdcc(1, &qsd8x50_sdc1_data, 0, 0);
}
__setup("androidboot.serialno=", board_serialno_setup);
#endif

static struct platform_device smc91x_device = {
	.name           = "smc91x",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(smc91x_resources),
	.resource       = smc91x_resources,
};

#ifdef CONFIG_USB_FUNCTION
static struct usb_function_map usb_functions_map[] = {
	{"diag", 0},
	{"adb", 1},
	{"modem", 2},
	{"nmea", 3},
	{"mass_storage", 4},
	{"ethernet", 5},
};

/* dynamic composition */
static struct usb_composition usb_func_composition[] = {
	{
		.product_id         = 0x9012,
		.functions	    = 0x5, /* 0101 */
	},

	{
		.product_id         = 0x9013,
		.functions	    = 0x15, /* 10101 */
	},

	{
		.product_id         = 0x9014,
		.functions	    = 0x30, /* 110000 */
	},

	{
		.product_id         = 0x9015,
		.functions	    = 0x12, /* 10010 */
	},

	{
		.product_id         = 0x9016,
		.functions	    = 0xD, /* 01101 */
	},

	{
		.product_id         = 0x9017,
		.functions	    = 0x1D, /* 11101 */
	},

	{
		.product_id         = 0xF000,
		.functions	    = 0x10, /* 10000 */
	},

	{
		.product_id         = 0xF009,
		.functions	    = 0x20, /* 100000 */
	},

	{
		.product_id         = 0x9018,
		.functions	    = 0x1F, /* 011111 */
	},

	{
		.product_id         = 0x901A,
		.functions	    = 0x0F, /* 01111 */
	},
};
#endif

static struct msm_handset_platform_data hs_platform_data = {
	.hs_name = "8k_handset",
	.pwr_key_delay_ms = 500, /* 0 will disable end key */
};

static struct platform_device hs_device = {
	.name   = "msm-handset",
	.id     = -1,
	.dev    = {
		.platform_data = &hs_platform_data,
	},
};

#ifdef CONFIG_USB_FS_HOST
static struct msm_gpio fsusb_config[] = {
	{ GPIO_CFG(139, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "fs_dat" },
	{ GPIO_CFG(140, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "fs_se0" },
	{ GPIO_CFG(141, 3, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "fs_oe_n" },
};

static int fsusb_gpio_init(void)
{
	return msm_gpios_request(fsusb_config, ARRAY_SIZE(fsusb_config));
}

static void msm_fsusb_setup_gpio(unsigned int enable)
{
	if (enable)
		msm_gpios_enable(fsusb_config, ARRAY_SIZE(fsusb_config));
	else
		msm_gpios_disable(fsusb_config, ARRAY_SIZE(fsusb_config));

}
#endif

#define MSM_USB_BASE              ((unsigned)addr)

static struct msm_hsusb_platform_data msm_hsusb_pdata = {
#ifdef CONFIG_USB_FUNCTION
	.version	= 0x0100,
	.phy_info	= (USB_PHY_INTEGRATED | USB_PHY_MODEL_180NM),
	.vendor_id          = 0x5c6,
	.product_name       = "Qualcomm HSUSB Device",
	.serial_number      = "1234567890ABCDEF",
	.manufacturer_name  = "Qualcomm Incorporated",
	.compositions	= usb_func_composition,
	.num_compositions = ARRAY_SIZE(usb_func_composition),
	.function_map   = usb_functions_map,
	.num_functions	= ARRAY_SIZE(usb_functions_map),
	.config_gpio    = NULL,

#endif
};

static struct vreg *vreg_usb;
static void msm_hsusb_vbus_power(unsigned phy_info, int on)
{
	msm_device_otg.dev.platform_data = &msm_otg_pdata;
	msm_device_hsusb.dev.parent = &msm_device_otg.dev;
	msm_device_hsusb_host.dev.parent = &msm_device_otg.dev;
	platform_add_devices(devices, ARRAY_SIZE(devices));
	qsd8x50_init_mmc();
}

MACHINE_START(QSD8X50_SURF, "QCT QSD8X50 SURF")
	.atag_offset = 0x100,
	.map_io = qsd8x50_map_io,
	.init_irq = qsd8x50_init_irq,
	.init_machine = qsd8x50_init,
	.timer = &msm_timer,
MACHINE_END

MACHINE_START(QSD8X50A_ST1_5, "QCT QSD8X50A ST1.5")
	.atag_offset = 0x100,
	.map_io = qsd8x50_map_io,
	.init_irq = qsd8x50_init_irq,
	.init_machine = qsd8x50_init,
	.timer = &msm_timer,
MACHINE_END
