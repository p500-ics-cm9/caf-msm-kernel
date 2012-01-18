/* Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __ARCH_ARM_MACH_MSM_CLOCK_7X30_H
#define __ARCH_ARM_MACH_MSM_CLOCK_7X30_H

enum {
	L_ADM_CLK,
	L_ADM_P_CLK,
	L_CE_CLK,
	L_I2C_CLK,
	L_I2C_2_CLK,
	L_QUP_I2C_CLK,
	L_UART1DM_CLK,
	L_UART1DM_P_CLK,
	L_UART2DM_CLK,
	L_UART2DM_P_CLK,
	L_EMDH_CLK,
	L_EMDH_P_CLK,
	L_PMDH_CLK,
	L_PMDH_P_CLK,
	L_GRP_2D_CLK,
	L_GRP_2D_P_CLK,
	L_GRP_3D_SRC_CLK,
	L_GRP_3D_CLK,
	L_GRP_3D_P_CLK,
	L_IMEM_CLK,
	L_SDC1_CLK,
	L_SDC1_P_CLK,
	L_SDC2_CLK,
	L_SDC2_P_CLK,
	L_SDC3_CLK,
	L_SDC3_P_CLK,
	L_SDC4_CLK,
	L_SDC4_P_CLK,
	L_MDP_CLK,
	L_MDP_P_CLK,
	L_MDP_LCDC_PCLK_CLK,
	L_MDP_LCDC_PAD_PCLK_CLK,
	L_MDP_VSYNC_CLK,
	L_MI2S_CODEC_RX_M_CLK,
	L_MI2S_CODEC_RX_S_CLK,
	L_MI2S_CODEC_TX_M_CLK,
	L_MI2S_CODEC_TX_S_CLK,
	L_MI2S_M_CLK,
	L_MI2S_S_CLK,
	L_LPA_CODEC_CLK,
	L_LPA_CORE_CLK,
	L_LPA_P_CLK,
	L_MIDI_CLK,
	L_MDC_CLK,
	L_ROTATOR_IMEM_CLK,
	L_ROTATOR_P_CLK,
	L_SDAC_M_CLK,
	L_SDAC_CLK,
	L_UART1_CLK,
	L_UART2_CLK,
	L_UART3_CLK,
	L_TV_CLK,
	L_TV_DAC_CLK,
	L_TV_ENC_CLK,
	L_HDMI_CLK,
	L_TSIF_REF_CLK,
	L_TSIF_P_CLK,
	L_USB_HS_SRC_CLK,
	L_USB_HS_CLK,
	L_USB_HS_CORE_CLK,
	L_USB_HS_P_CLK,
	L_USB_HS2_CLK,
	L_USB_HS2_CORE_CLK,
	L_USB_HS2_P_CLK,
	L_USB_HS3_CLK,
	L_USB_HS3_CORE_CLK,
	L_USB_HS3_P_CLK,
	L_VFE_CLK,
	L_VFE_P_CLK,
	L_VFE_MDC_CLK,
	L_VFE_CAMIF_CLK,
	L_CAMIF_PAD_P_CLK,
	L_CAM_M_CLK,
	L_JPEG_CLK,
	L_JPEG_P_CLK,
	L_VPE_CLK,
	L_MFC_CLK,
	L_MFC_DIV2_CLK,
	L_MFC_P_CLK,
	L_SPI_CLK,
	L_SPI_P_CLK,
	L_CSI0_CLK,
	L_CSI0_VFE_CLK,
	L_CSI0_P_CLK,
	L_CSI1_CLK,
	L_CSI1_VFE_CLK,
	L_CSI1_P_CLK,
	L_GLBL_ROOT_CLK,

	L_AXI_LI_VG_CLK,
	L_AXI_LI_GRP_CLK,
	L_AXI_LI_JPEG_CLK,
	L_AXI_GRP_2D_CLK,
	L_AXI_MFC_CLK,
	L_AXI_VPE_CLK,
	L_AXI_LI_VFE_CLK,
	L_AXI_LI_APPS_CLK,
	L_AXI_MDP_CLK,
	L_AXI_IMEM_CLK,
	L_AXI_LI_ADSP_A_CLK,
	L_AXI_ROTATOR_CLK,

	L_NR_CLKS
};

enum clk_sources {
	PLL_0 = 0,
	PLL_1,
	PLL_2,
	PLL_3,
	PLL_4,
	PLL_5,
	PLL_6,
	AXI,
	LPXO,
	TCXO,
	NUM_SRC
};

extern int internal_pwr_rail_ctl_auto(unsigned rail_id, bool enable);

extern struct clk_ops soc_clk_ops_7x30;
#define CLK_7X30(clk_name, clk_id, clk_dev, clk_flags) {	\
	.con_id = clk_name, \
	.dev_id = clk_dev, \
	.clk = &(struct clk){ \
		.id = L_7X30_##clk_id, \
		.remote_id = P_##clk_id, \
		.flags = clk_flags, \
		.dbg_name = #clk_id, \
	}, \
	}

#define CLK_7X30S(clk_name, l_id, r_id, clk_dev, clk_flags) {	\
	.con_id = clk_name, \
	.dev_id = clk_dev, \
	.clk = &(struct clk){ \
		.id = L_7X30_##l_id, \
		.remote_id = P_##r_id, \
		.flags = clk_flags, \
		.dbg_name = #l_id, \
		.ops = &clk_ops_pcom, \
	}, \
	}

#define CLK_7X30L(clk_name, l_id, clk_dev, clk_flags) {	\
	.name = clk_name, \
	.id = L_##l_id, \
	.flags = clk_flags, \
	.dev = clk_dev, \
	.dbg_name = #l_id, \
	.ops = &soc_clk_ops_7x30, \
	}

#endif
