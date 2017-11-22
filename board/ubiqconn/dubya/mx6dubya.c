/*
 * Copyright (C) 2012-2015 Freescale Semiconductor, Inc.
 *
 * Author: Fabio Estevam <fabio.estevam@freescale.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <asm/errno.h>
#include <asm/gpio.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/boot_mode.h>
#include <asm/imx-common/video.h>
#include <mmc.h>
#include <pwm.h>
#include <fsl_esdhc.h>

#include <asm/arch/mxc_hdmi.h>
#include <asm/arch/crm_regs.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
#include <i2c.h>
#include <ipu_pixfmt.h>
#include <linux/fb.h>
#include <asm/arch/mx6-ddr.h>
#include <usb.h>
#include <netdev.h>

#ifdef CONFIG_FSL_FASTBOOT
#include <fsl_fastboot.h>
#ifdef CONFIG_ANDROID_RECOVERY
#include <recovery.h>
#endif
#endif /*CONFIG_FSL_FASTBOOT*/

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_48ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC1_PAD_CTRL (PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_SRE_FAST | PAD_CTL_HYS)

#define I2C_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |	\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define OTG_ID_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_47K_UP  | PAD_CTL_SPEED_LOW |		\
	PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define WEAK_PULLDOWN	(PAD_CTL_PUS_100K_DOWN |		\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_HYS | PAD_CTL_SRE_SLOW)

#define WEAK_PULLUP	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_HYS | PAD_CTL_SRE_SLOW)

#define NAND_PAD_CTRL	(PAD_CTL_100K_UP |			\
		PAD_CTL_SPEED

#define I2C_PAD MUX_PAD_CTRL(I2C_PAD_CTRL)

#define LCD_PWR_EN	IMX_GPIO_NR(3, 5)
#define LCD_RESET	IMX_GPIO_NR(2, 16)
#define BL_EN		IMX_GPIO_NR(5, 25)

#define PWM_LCD_CNTRST	0	/* PWM1 */
#define PWM_BACKLIGHT	3	/* PWM4	*/
#define PWM_AUDIO	2	/* PWM3 */

#define PWR_HOLDON	IMX_GPIO_NR(1, 7)
#define WIFI_EN		IMX_GPIO_NR(5, 26)
#define BT_EN		IMX_GPIO_NR(5, 24)

#define DUBYA_FAMILY_ID (4)
#define FAMILY_CFG3	IMX_GPIO_NR(2, 11)
#define FAMILY_CFG2	IMX_GPIO_NR(3, 24)
#define FAMILY_CFG1	IMX_GPIO_NR(3, 26)
#define FAMILY_CFG0	IMX_GPIO_NR(5, 28)
#define MODEL_ID1	IMX_GPIO_NR(5, 22)
#define MODEL_ID0	IMX_GPIO_NR(5, 20)

#define TOUCH_EN	IMX_GPIO_NR(2, 18)

#define ETH_PHY_RESET	IMX_GPIO_NR(4, 6)

enum { ID_DUBYA7 = 0,
       ID_DUBYA9 = 1,
       ID_DUBYA12 = 2,
       ID_DUBYA16 = 3,
       ID_UNKNOW
};

int dram_init(void)
{
	gd->ram_size = imx_ddr_size();
	return 0;
}

static iomux_v3_cfg_t const pwr_pads[] = {
	(MX6_PAD_EIM_A17__GPIO2_IO21 | MUX_PAD_CTRL(WEAK_PULLDOWN)),	/* PWR_KEY_SENSE */
	(MX6_PAD_GPIO_7__GPIO1_IO07 | MUX_PAD_CTRL(NO_PAD_CTRL)),	/* PWR_HOLDON */
};

#if 0
static iomux_v3_cfg_t const hwId_pads[] = {
	MX6_PAD_ENET_TXD0__GPIO1_IO30	| MUX_PAD_CTRL(WEAK_PULLUP),	/* HW_ID0 */
	MX6_PAD_ENET_TXD1__GPIO1_IO29	| MUX_PAD_CTRL(WEAK_PULLUP),	/* HW_ID1 */
	MX6_PAD_KEY_ROW2__GPIO4_IO11	| MUX_PAD_CTRL(WEAK_PULLUP),	/* HW_ID2 */
	MX6_PAD_KEY_ROW0__GPIO4_IO07	| MUX_PAD_CTRL(WEAK_PULLUP),	/* HW_ID3 */
	MX6_PAD_ENET_MDC__GPIO1_IO31	| MUX_PAD_CTRL(WEAK_PULLUP),	/* HW_ID4 */
};
#endif

static iomux_v3_cfg_t const boardId_pads[] = {
	MX6_PAD_SD4_DAT3__GPIO2_IO11	| MUX_PAD_CTRL(NO_PAD_CTRL),	/* BASE_DET */
	MX6_PAD_EIM_D24__GPIO3_IO24	| MUX_PAD_CTRL(NO_PAD_CTRL),	/* CPU-IGPS_TX3, 14.7K PULL DOWN */
	MX6_PAD_EIM_D26__GPIO3_IO26	| MUX_PAD_CTRL(NO_PAD_CTRL),	/* CPU-SNR/STB_BOOT_TX2 */
	MX6_PAD_CSI0_DAT10__GPIO5_IO28	| MUX_PAD_CTRL(NO_PAD_CTRL),	/* CPU-NEMA_TX1*/
	MX6_PAD_CSI0_DAT4__GPIO5_IO22	| MUX_PAD_CTRL(NO_PAD_CTRL),	/* PROD_ID1 */
	MX6_PAD_CSI0_DATA_EN__GPIO5_IO20| MUX_PAD_CTRL(NO_PAD_CTRL),	/* PROD_ID0 */
	MX6_PAD_DISP0_DAT18__GPIO5_IO12	| MUX_PAD_CTRL(NO_PAD_CTRL), 	/* PROD_SNR */
	MX6_PAD_GPIO_6__GPIO1_IO06	| MUX_PAD_CTRL(NO_PAD_CTRL),	/* PROD_GPS */
};

static iomux_v3_cfg_t const hwVer_pads[] = {
	MX6_PAD_DISP0_DAT23__GPIO5_IO17	| MUX_PAD_CTRL(WEAK_PULLDOWN),	/* HW_VER_0 */
	MX6_PAD_DISP0_DAT20__GPIO5_IO14	| MUX_PAD_CTRL(WEAK_PULLDOWN),	/* HW_VER_1/I2S_BITCLK */
	MX6_PAD_SD3_RST__GPIO7_IO08	| MUX_PAD_CTRL(WEAK_PULLDOWN),	/* HW_VER_2 */
	MX6_PAD_DISP0_DAT22__GPIO5_IO16	| MUX_PAD_CTRL(WEAK_PULLDOWN),	/* HW_VER_3/I2S_WCLK */
};

static iomux_v3_cfg_t const enet_pads[] = {
	MX6_PAD_ENET_MDIO__ENET_MDIO		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_MDC__ENET_MDC		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TXC__RGMII_TXC	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD0__RGMII_TD0	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD1__RGMII_TD1	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TX_CTL__RGMII_TX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_REF_CLK__ENET_TX_CLK	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RXC__RGMII_RXC	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD0__RGMII_RD0	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD1__RGMII_RD1	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RX_CTL__RGMII_RX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_GPIO_16__ENET_REF_CLK		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_KEY_COL2__ENET_RX_DATA2	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	/* SMSC LAN8710 PHY Reset */
	MX6_PAD_EIM_DA6__GPIO3_IO06	| MUX_PAD_CTRL(NO_PAD_CTRL),
};

static void setup_iomux_enet(void)
{
	imx_iomux_v3_setup_multiple_pads(enet_pads, ARRAY_SIZE(enet_pads));
	/* Reset SMSC LAN8710 PHY */
	gpio_direction_output(ETH_PHY_RESET , 0);
	mdelay(10);
	gpio_set_value(ETH_PHY_RESET, 1);
	udelay(100);
}

#if (CONFIG_MXC_UART_BASE == UART1_BASE)
static iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_CSI0_DAT10__UART1_TX_DATA	| MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_CSI0_DAT11__UART1_RX_DATA	| MUX_PAD_CTRL(UART_PAD_CTRL),
};
#endif
#if (CONFIG_MXC_UART_BASE == UART5_BASE)
static iomux_v3_cfg_t const uart5_pads[] = {
	MX6_PAD_KEY_COL1__UART5_TX_DATA	| MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_KEY_ROW1__UART5_RX_DATA	| MUX_PAD_CTRL(UART_PAD_CTRL),
};
#endif

static iomux_v3_cfg_t const usdhc1_pads[] = {
	MX6_PAD_SD1_CLK__SD1_CLK	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_CMD__SD1_CMD	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DAT0__SD1_DATA0	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DAT1__SD1_DATA1	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DAT2__SD1_DATA2	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DAT3__SD1_DATA3	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT0__GPIO2_IO08	| MUX_PAD_CTRL(NO_PAD_CTRL),	/* CD */
	MX6_PAD_SD4_DAT4__GPIO2_IO12	| MUX_PAD_CTRL(NO_PAD_CTRL),	/* EN */
	MX6_PAD_EIM_D16__GPIO3_IO16	| MUX_PAD_CTRL(NO_PAD_CTRL),	/* SD_FAULT */
};
#define USDHC1_EN	IMX_GPIO_NR(2, 12)
#define USDHC1_CD	IMX_GPIO_NR(2, 8)
#define USDHC1_FAULT	IMX_GPIO_NR(3, 16)

static iomux_v3_cfg_t const touch_pads[] = {
	MX6_PAD_EIM_A20__GPIO2_IO18	| MUX_PAD_CTRL(NO_PAD_CTRL),	/* TOUCH_EN */
};

static iomux_v3_cfg_t const audio_pads[] = {
	MX6_PAD_SD4_DAT1__PWM3_OUT	| MUX_PAD_CTRL(NO_PAD_CTRL),	/* AUDIO_PWM */
};

static iomux_v3_cfg_t const wl18xx_pads[] = {
	/* USDHC SD2 for WIFI SDIO */
	MX6_PAD_SD2_CLK__SD2_CLK	| MUX_PAD_CTRL(USDHC1_PAD_CTRL),
	MX6_PAD_SD2_CMD__SD2_CMD	| MUX_PAD_CTRL(USDHC1_PAD_CTRL),
	MX6_PAD_SD2_DAT0__SD2_DATA0	| MUX_PAD_CTRL(USDHC1_PAD_CTRL),
	MX6_PAD_SD2_DAT1__SD2_DATA1	| MUX_PAD_CTRL(USDHC1_PAD_CTRL),
	MX6_PAD_SD2_DAT2__SD2_DATA2	| MUX_PAD_CTRL(USDHC1_PAD_CTRL),
	MX6_PAD_SD2_DAT3__SD2_DATA3	| MUX_PAD_CTRL(USDHC1_PAD_CTRL),
	MX6_PAD_CSI0_DAT19__GPIO6_IO05 | MUX_PAD_CTRL(NO_PAD_CTRL),	/* SD2_IRQ */
	/* UART for Bluetooth HCI Transport */
	MX6_PAD_CSI0_DAT12__UART4_TX_DATA	| MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_CSI0_DAT13__UART4_RX_DATA	| MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_CSI0_DAT16__UART4_RTS_B	| MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_CSI0_DAT17__UART4_CTS_B	| MUX_PAD_CTRL(UART_PAD_CTRL),
	/* Enabled Control */
	MX6_PAD_CSI0_DAT8__GPIO5_IO26	| MUX_PAD_CTRL(NO_PAD_CTRL),	/* WIFI_EN */
	MX6_PAD_CSI0_DAT6__GPIO5_IO24	| MUX_PAD_CTRL(NO_PAD_CTRL),	/* BLUETOOTH_EN */
};

static iomux_v3_cfg_t const sonar_pads[] = {
	MX6_PAD_EIM_D26__UART2_TX_DATA	| MUX_PAD_CTRL(UART_PAD_CTRL),	/* CPU-SNR/STB_BOOT_TX2 */
	MX6_PAD_EIM_D27__UART2_RX_DATA	| MUX_PAD_CTRL(UART_PAD_CTRL),	/* CPU-SNR_STB_BOOT_RX2 */
	MX6_PAD_DISP0_DAT19__GPIO5_IO13 | MUX_PAD_CTRL(NO_PAD_CTRL),	/* SNR_BTSTRP_EN	*/
	MX6_PAD_EIM_A24__GPIO5_IO04	| MUX_PAD_CTRL(NO_PAD_CTRL),	/* SNR_RESET		*/
};

static iomux_v3_cfg_t const di0_pads[] = {
	MX6_PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK,	/* DISP0_CLK */
	MX6_PAD_DI0_PIN15__IPU1_DI0_PIN15,		/* DISP0_DRDY, DE */
	MX6_PAD_DI0_PIN2__IPU1_DI0_PIN02,		/* DISP0_HSYNC */
	MX6_PAD_DI0_PIN3__IPU1_DI0_PIN03,		/* DISP0_VSYNC */
};

static iomux_v3_cfg_t const lcd_pads[] = {
	MX6_PAD_EIM_DA5__GPIO3_IO05	| MUX_PAD_CTRL(NO_PAD_CTRL),	/* LCD_PWR_EN */
	MX6_PAD_EIM_A22__GPIO2_IO16	| MUX_PAD_CTRL(NO_PAD_CTRL),	/* LCD_RESET */
	MX6_PAD_GPIO_9__PWM1_OUT	| MUX_PAD_CTRL(NO_PAD_CTRL),	/* LCD_CNTRST_PWM */
	MX6_PAD_SD4_DAT2__PWM4_OUT	| MUX_PAD_CTRL(NO_PAD_CTRL),	/* LCD_BACKLIGHT-DIM_PWM */
	MX6_PAD_CSI0_DAT7__GPIO5_IO25	| MUX_PAD_CTRL(NO_PAD_CTRL),	/* BACKLIGHT_EN_B */
};

static iomux_v3_cfg_t const rgb_pads[] = {
	MX6_PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DI0_PIN15__IPU1_DI0_PIN15 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DI0_PIN2__IPU1_DI0_PIN02 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DI0_PIN3__IPU1_DI0_PIN03 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DI0_PIN4__IPU1_DI0_PIN04 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT0__IPU1_DISP0_DATA00 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT1__IPU1_DISP0_DATA01 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT2__IPU1_DISP0_DATA02 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT3__IPU1_DISP0_DATA03 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT4__IPU1_DISP0_DATA04 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT5__IPU1_DISP0_DATA05 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT6__IPU1_DISP0_DATA06 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT7__IPU1_DISP0_DATA07 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT8__IPU1_DISP0_DATA08 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT9__IPU1_DISP0_DATA09 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT10__IPU1_DISP0_DATA10 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT11__IPU1_DISP0_DATA11 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT12__IPU1_DISP0_DATA12 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT13__IPU1_DISP0_DATA13 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT14__IPU1_DISP0_DATA14 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT15__IPU1_DISP0_DATA15 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT16__IPU1_DISP0_DATA16 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT17__IPU1_DISP0_DATA17 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_EIM_DA5__GPIO3_IO05	| MUX_PAD_CTRL(NO_PAD_CTRL),	/* LCD_PWR_EN */
	MX6_PAD_EIM_A22__GPIO2_IO16	| MUX_PAD_CTRL(NO_PAD_CTRL),	/* LCD_RESET */
	MX6_PAD_GPIO_9__PWM1_OUT	| MUX_PAD_CTRL(NO_PAD_CTRL),	/* LCD_CNTRST_PWM */
	MX6_PAD_SD4_DAT2__PWM4_OUT	| MUX_PAD_CTRL(NO_PAD_CTRL),	/* LCD_BACKLIGHT-DIM_PWM */
	MX6_PAD_CSI0_DAT7__GPIO5_IO25	| MUX_PAD_CTRL(NO_PAD_CTRL),	/* BACKLIGHT_EN_B */
};

/* NAND */
static iomux_v3_cfg_t const nandfc_pads[] = {
	MX6_PAD_NANDF_CLE__NAND_CLE     | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NANDF_ALE__NAND_ALE     | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NANDF_WP_B__NAND_WP_B   | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NANDF_RB0__NAND_READY_B | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NANDF_CS0__NAND_CE0_B   | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_SD4_CMD__NAND_RE_B      | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_SD4_CLK__NAND_WE_B      | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NANDF_D0__NAND_DATA00   | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NANDF_D1__NAND_DATA01   | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NANDF_D2__NAND_DATA02   | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NANDF_D3__NAND_DATA03   | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NANDF_D4__NAND_DATA04   | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NANDF_D5__NAND_DATA05   | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NANDF_D6__NAND_DATA06   | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NANDF_D7__NAND_DATA07   | MUX_PAD_CTRL(NO_PAD_CTRL),
};

#ifdef CONFIG_CMD_NAND
static void setup_gpmi_nand(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

	/* config gpmi nand iomux */
	imx_iomux_v3_setup_multiple_pads(nandfc_pads, ARRAY_SIZE(nandfc_pads));

	/* config gpmi and bch clock to 100 MHz */
	clrsetbits_le32(&mxc_ccm->cs2cdr,
			MXC_CCM_CS2CDR_ENFC_CLK_PODF_MASK |
			MXC_CCM_CS2CDR_ENFC_CLK_PRED_MASK |
			MXC_CCM_CS2CDR_ENFC_CLK_SEL_MASK,
			MXC_CCM_CS2CDR_ENFC_CLK_PODF(0) |
			MXC_CCM_CS2CDR_ENFC_CLK_PRED(3) |
			MXC_CCM_CS2CDR_ENFC_CLK_SEL(3));

	/* enable gpmi and bch clock gating */
	setbits_le32(&mxc_ccm->CCGR4,
			MXC_CCM_CCGR4_RAWNAND_U_BCH_INPUT_APB_MASK |
			MXC_CCM_CCGR4_RAWNAND_U_GPMI_BCH_INPUT_BCH_MASK |
			MXC_CCM_CCGR4_RAWNAND_U_GPMI_BCH_INPUT_GPMI_IO_MASK |
			MXC_CCM_CCGR4_RAWNAND_U_GPMI_INPUT_APB_MASK |
			MXC_CCM_CCGR4_PL301_MX6QPER1_BCH_OFFSET);

	/* enable apbh clock gating */
	setbits_le32(&mxc_ccm->CCGR0, MXC_CCM_CCGR0_APBHDMA_MASK);
}
#endif

static struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		.i2c_mode = MX6_PAD_GPIO_3__I2C3_SCL | I2C_PAD,
		.gpio_mode = MX6_PAD_GPIO_3__GPIO1_IO03 | I2C_PAD,
		.gp = IMX_GPIO_NR(1, 3)
	},
	.sda = {
		.i2c_mode = MX6_PAD_GPIO_6__I2C3_SDA | I2C_PAD,
		.gpio_mode = MX6_PAD_GPIO_6__GPIO1_IO06 | I2C_PAD,
		.gp = IMX_GPIO_NR(1, 6)
	}
};



static void setup_iomux_board(void)
{
#if 0
	imx_iomux_v3_setup_multiple_pads(hwId_pads, ARRAY_SIZE(hwId_pads));
#endif
	imx_iomux_v3_setup_multiple_pads(boardId_pads, ARRAY_SIZE(boardId_pads));
	gpio_direction_input(FAMILY_CFG3);
	gpio_direction_input(FAMILY_CFG2);
	gpio_direction_input(FAMILY_CFG1);
	gpio_direction_input(FAMILY_CFG0);
	gpio_direction_input(MODEL_ID1);
	gpio_direction_input(MODEL_ID0);
}

static int getBoardFamily(void)
{
	int ret = 0;
	ret |= (gpio_get_value(FAMILY_CFG3)) ? 1 << 3 : 0;
	ret |= (gpio_get_value(FAMILY_CFG2)) ? 1 << 2 : 0;
	ret |= (gpio_get_value(FAMILY_CFG1)) ? 1 << 1 : 0;
	ret |= (gpio_get_value(FAMILY_CFG0)) ? 1 << 0 : 0;
	return ret;
}

static int getBoardId(void)
{
	int ret = 0;

	ret |= gpio_get_value(MODEL_ID1) << 1;
	ret |= gpio_get_value(MODEL_ID0) << 0;
	return ret;
}

static void setup_board_version(void)
{
	int boardId = getBoardId();
	setenv("fdt_file", (boardId == ID_DUBYA16) ?
		"imx6q-dubya16.dtb" : "imx6q-dubya.dtb");
}

static void setup_iomux_touch(void)
{
	imx_iomux_v3_setup_multiple_pads(touch_pads, ARRAY_SIZE(touch_pads));
	gpio_direction_output(TOUCH_EN, 0);
};


static void setup_iomux_uart(void)
{

#if (CONFIG_MXC_UART_BASE == UART1_BASE)
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
#endif
#if (CONFIG_MXC_UART_BASE == UART5_BASE)
	imx_iomux_v3_setup_multiple_pads(uart5_pads, ARRAY_SIZE(uart5_pads));
#endif
}

#ifdef CONFIG_FSL_ESDHC
struct fsl_esdhc_cfg usdhc_cfg[1] = {
	{USDHC1_BASE_ADDR},
};

int mmc_get_env_devno(void)
{
	/* because only USDHC0, so always return zero */
	return CONFIG_SYS_MMC_ENV_DEV;
}

int mmc_map_to_kernel_blk(int dev_no)
{
	return dev_no;
}

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	case USDHC1_BASE_ADDR:
		ret = !gpio_get_value(USDHC1_CD);
		break;
	default:
		printf("error USDHC base\n");
		return -1;
	}

	return ret;
}

int board_mmc_init(bd_t *bis)
{
#ifndef CONFIG_SPL_BUILD
	int ret;
	/*
	 * According to the board_mmc_init() the following map is done:
	 * (U-boot device node)    (Physical Port)
	 * mmc0                    SD1
	 */
	imx_iomux_v3_setup_multiple_pads(
	usdhc1_pads, ARRAY_SIZE(usdhc1_pads));
	gpio_direction_output(USDHC1_EN, 1);
	gpio_direction_input(USDHC1_CD);
	gpio_direction_input(USDHC1_FAULT);
	usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);
	usdhc_cfg[0].max_bus_width = 4;
	ret = fsl_esdhc_initialize(bis, &usdhc_cfg[0]);
	if (ret)
		return ret;
	return 0;
#else
	struct src *psrc = (struct src *)SRC_BASE_ADDR;
	unsigned reg = readl(&psrc->sbmr1) >> 11;
	/*
	 * Upon reading BOOT_CFG register the following map is done:
	 * Bit 11 and 12 of BOOT_CFG register can determine the current
	 * mmc port
	 * 0x1                  SD1
	 */
	switch (reg & 0x3) {
	case 0x1:
		imx_iomux_v3_setup_multiple_pads(
			usdhc2_pads, ARRAY_SIZE(usdhc1_pads));
		usdhc_cfg[0].esdhc_base = USDHC1_BASE_ADDR;
		usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);
		gd->arch.sdhc_clk = usdhc_cfg[0].sdhc_clk;
		break;
	}

	return fsl_esdhc_initialize(bis, &usdhc_cfg[0]);
#endif
}
#endif

void board_late_mmc_env_init(void)
{
	char cmd[32];
	char mmcblk[32];
	u32 dev_no = mmc_get_env_devno();

	setenv_ulong("mmcdev", dev_no);

	/* Set mmcblk env */
	sprintf(mmcblk, "/dev/mmcblk%dp2 rootwait rw",
		mmc_map_to_kernel_blk(dev_no));
	setenv("mmcroot", mmcblk);
	sprintf(cmd, "mmc dev %d", dev_no);
	run_command(cmd, 0);
}

/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	return 1;
}

static void setup_fec(void)
{
	int ret;

	if (is_mx6dqp()) {
		/* select ENET MAC0 TX clock from PLL */
		imx_iomux_set_gpr_register(5, 9, 1, 1);
	} else {
		imx_iomux_set_gpr_register(1, 21, 1, 1);
	}
	ret = enable_fec_anatop_clock(0, ENET_125MHZ);
	if (ret)
		printf("Error fec anatop clock settings!\n");
}

int board_eth_init(bd_t *bis)
{
	setup_iomux_enet();

	return cpu_eth_init(bis);
}

#ifdef CONFIG_USB_EHCI_MX6
#define USB_OTHERREGS_OFFSET	0x800
#define UCTRL_PWR_POL		(1 << 9)

static iomux_v3_cfg_t const usb_otg_pads[] = {
	(MX6_PAD_GPIO_1__USB_OTG_ID | MUX_PAD_CTRL(OTG_ID_PAD_CTRL)),
};

static void setup_usb(void)
{
	imx_iomux_v3_setup_multiple_pads(usb_otg_pads,
					 ARRAY_SIZE(usb_otg_pads));
	/*
	 * set daisy chain for otg_pin_id on 6q.
	 * for 6dl, this bit is reserved
	 */
	imx_iomux_set_gpr_register(1, 13, 1, 0);
}

int board_ehci_hcd_init(int port)
{
	u32 *usbnc_usb_ctrl;

	if (port > 1)
		return -EINVAL;

	usbnc_usb_ctrl = (u32 *)(USB_BASE_ADDR + USB_OTHERREGS_OFFSET +
				 port * 4);

	setbits_le32(usbnc_usb_ctrl, UCTRL_PWR_POL);

	return 0;
}

int board_ehci_power(int port, int on)
{
	switch (port) {
	case 0:
		break;
	case 1:
		break;
	default:
		printf("MXC USB port %d not yet supported\n", port);
		return -EINVAL;
	}

	return 0;
}
#endif

int board_early_init_f(void)
{
	imx_iomux_v3_setup_multiple_pads(pwr_pads,
			ARRAY_SIZE(pwr_pads));
	/* pull hold on pin */
	gpio_direction_output(PWR_HOLDON, 1);

	setup_iomux_board();
	setup_iomux_uart();
#if defined(CONFIG_VIDEO_IPUV3)
	setup_display_clock();
#endif
	setup_iomux_touch();

	return 0;
}

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);

#ifdef CONFIG_CMD_NAND
	setup_gpmi_nand();
#endif

#ifdef CONFIG_USB_EHCI_MX6
	setup_usb();
#endif

#ifdef CONFIG_FEC_MXC
	setup_fec();
#endif
	return 0;
}


#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"sd1",	 MAKE_CFGVAL(0x40, 0x28, 0x00, 0x00)},
	{NULL,	 0},
};
#endif

int board_late_init(void)
{
#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif

#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
#endif
	setup_board_version();
	gpio_set_value(TOUCH_EN, 1);
#ifdef CONFIG_VIDEO_IPUV3
	setup_display_lvds();
#endif
	return 0;
}

int checkboard(void)
{
	u32 boardId = 0;
	/*
	 * NEED ENABLED CONFIG_DISPLAY_CPUINFO
	 * to called get_imx_reset_cause();
	 *
	 * check reset cause when not POR,
	 * will skip checkboard() directly return 0
	 *
	 */
/*
	boardId = get_imx_reset_cause();
	if ((0x00001 != boardId) || (0x00011 != boardId))
		return 0;
*/
	boardId = getBoardId();

	if (DUBYA_FAMILY_ID != getBoardFamily())
	{
/*		boardId |= 0xF0; */
	}

	if(ID_DUBYA7 == boardId)
	{
		puts("Board: MX6-DUBYA7\n");
	}
	else if (ID_DUBYA9 == boardId)
	{
		puts("Board: MX6-DUBYA9\n");
	}
	else if (ID_DUBYA12 == boardId)
	{
		puts("Board: MX6-DUBYA12\n");
	}
	else if (ID_DUBYA16 == boardId)
	{
		puts("Board: MX6-DUBYA16\n");
	}
	else {
		puts("Board: Unknown, Not support!!!\n");
		return 1;
	}

	return 0;
}

#if defined(CONFIG_OF_LIBFDT) && defined(CONFIG_OF_BOARD_SETUP)
#include <fdt_support.h>
int fdt_fixup_nfc(int hwrev, void *blob, bd_t *bd)
{
	return 0;
}

int ft_board_setup(void *blob, bd_t *bd)
{
	unsigned long rev;
	rev = getenv_ulong("hwrev", 10, 0);
	return fdt_fixup_nfc((int)rev, blob, bd);
}
#endif /* defined(CONFIG_OF_LIBFDT) && defined(CONFIG_OF_BOARD_SETUP) */

#ifdef CONFIG_FSL_FASTBOOT

void board_fastboot_setup(void)
{
	switch (get_boot_device()) {
#if defined(CONFIG_FASTBOOT_STORAGE_MMC)
	case SD2_BOOT:
	case MMC2_BOOT:
	    if (!getenv("fastboot_dev"))
			setenv("fastboot_dev", "mmc0");
	    if (!getenv("bootcmd"))
			setenv("bootcmd", "boota mmc0");
	    break;
	case SD3_BOOT:
	case MMC3_BOOT:
	    if (!getenv("fastboot_dev"))
			setenv("fastboot_dev", "mmc1");
	    if (!getenv("bootcmd"))
			setenv("bootcmd", "boota mmc1");
	    break;
	case MMC4_BOOT:
	    if (!getenv("fastboot_dev"))
			setenv("fastboot_dev", "mmc2");
	    if (!getenv("bootcmd"))
			setenv("bootcmd", "boota mmc2");
	    break;
#endif /*CONFIG_FASTBOOT_STORAGE_MMC*/
	default:
		printf("unsupported boot devices\n");
		break;
	}

}

#ifdef CONFIG_ANDROID_RECOVERY

#define GPIO_VOL_DN_KEY IMX_GPIO_NR(1, 5)
iomux_v3_cfg_t const recovery_key_pads[] = {
	(MX6_PAD_GPIO_5__GPIO1_IO05 | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

int check_recovery_cmd_file(void)
{
    int button_pressed = 0;
    int recovery_mode = 0;

    recovery_mode = recovery_check_and_clean_flag();

    /* Check Recovery Combo Button press or not. */
	imx_iomux_v3_setup_multiple_pads(recovery_key_pads,
			ARRAY_SIZE(recovery_key_pads));

    gpio_direction_input(GPIO_VOL_DN_KEY);

    if (gpio_get_value(GPIO_VOL_DN_KEY) == 0) { /* VOL_DN key is low assert */
		button_pressed = 1;
		printf("Recovery key pressed\n");
    }

    return recovery_mode || button_pressed;
}

void board_recovery_setup(void)
{
	int bootdev = get_boot_device();

	switch (bootdev) {
#if defined(CONFIG_FASTBOOT_STORAGE_MMC)
	case SD2_BOOT:
	case MMC2_BOOT:
		if (!getenv("bootcmd_android_recovery"))
			setenv("bootcmd_android_recovery",
				"boota mmc0 recovery");
		break;
	case SD3_BOOT:
	case MMC3_BOOT:
		if (!getenv("bootcmd_android_recovery"))
			setenv("bootcmd_android_recovery",
				"boota mmc1 recovery");
		break;
	case MMC4_BOOT:
		if (!getenv("bootcmd_android_recovery"))
			setenv("bootcmd_android_recovery",
				"boota mmc2 recovery");
		break;
#endif /*CONFIG_FASTBOOT_STORAGE_MMC*/
	default:
		printf("Unsupported bootup device for recovery: dev: %d\n",
			bootdev);
		return;
	}

	printf("setup env for recovery..\n");
	setenv("bootcmd", "run bootcmd_android_recovery");
}

#endif /*CONFIG_ANDROID_RECOVERY*/

#endif /*CONFIG_FSL_FASTBOOT*/

#ifdef CONFIG_SPL_BUILD
#include <spl.h>
#include <libfdt.h>

const struct mx6dq_iomux_ddr_regs mx6_ddr_ioregs = {
	.dram_sdclk_0 =  0x00020030,
	.dram_sdclk_1 =  0x00020030,
	.dram_cas =  0x00020030,
	.dram_ras =  0x00020030,
	.dram_reset =  0x00020030,
	.dram_sdcke0 =  0x00003000,
	.dram_sdcke1 =  0x00003000,
	.dram_sdba2 =  0x00000000,
	.dram_sdodt0 =  0x00003030,
	.dram_sdodt1 =  0x00003030,
	.dram_sdqs0 =  0x00000030,
	.dram_sdqs1 =  0x00000030,
	.dram_sdqs2 =  0x00000030,
	.dram_sdqs3 =  0x00000030,
	.dram_sdqs4 =  0x00000030,
	.dram_sdqs5 =  0x00000030,
	.dram_sdqs6 =  0x00000030,
	.dram_sdqs7 =  0x00000030,
	.dram_dqm0 =  0x00020030,
	.dram_dqm1 =  0x00020030,
	.dram_dqm2 =  0x00020030,
	.dram_dqm3 =  0x00020030,
	.dram_dqm4 =  0x00020030,
	.dram_dqm5 =  0x00020030,
	.dram_dqm6 =  0x00020030,
	.dram_dqm7 =  0x00020030,
};

const struct mx6dq_iomux_grp_regs mx6_grp_ioregs = {
	.grp_ddr_type =  0x000C0000,
	.grp_ddrmode_ctl =  0x00020000,
	.grp_ddrpke =  0x00000000,
	.grp_addds =  0x00000030,
	.grp_ctlds =  0x00000030,
	.grp_ddrmode =  0x00020000,
	.grp_b0ds =  0x00000030,
	.grp_b1ds =  0x00000030,
	.grp_b2ds =  0x00000030,
	.grp_b3ds =  0x00000030,
	.grp_b4ds =  0x00000030,
	.grp_b5ds =  0x00000030,
	.grp_b6ds =  0x00000030,
	.grp_b7ds =  0x00000030,
};

const struct mx6_mmdc_calibration mx6_mmcd_calib = {
	.p0_mpwldectrl0 =  0x001F001F,
	.p0_mpwldectrl1 =  0x001F001F,
	.p1_mpwldectrl0 =  0x00440044,
	.p1_mpwldectrl1 =  0x00440044,
	.p0_mpdgctrl0 =  0x434B0350,
	.p0_mpdgctrl1 =  0x034C0359,
	.p1_mpdgctrl0 =  0x434B0350,
	.p1_mpdgctrl1 =  0x03650348,
	.p0_mprddlctl =  0x4436383B,
	.p1_mprddlctl =  0x39393341,
	.p0_mpwrdlctl =  0x35373933,
	.p1_mpwrdlctl =  0x48254A36,
};

static struct mx6_ddr3_cfg mem_ddr = {
	.mem_speed = 1600,
	.density = 4,
	.width = 64,
	.banks = 8,
	.rowaddr = 14,
	.coladdr = 10,
	.pagesz = 2,
	.trcd = 1375,
	.trcmin = 4875,
	.trasmin = 3500,
};

static void ccgr_init(void)
{
	struct mxc_ccm_reg *ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

	writel(0x00C03F3F, &ccm->CCGR0);
	writel(0x0030FC03, &ccm->CCGR1);
	writel(0x0FFFC000, &ccm->CCGR2);
	writel(0x3FF00000, &ccm->CCGR3);
	writel(0x00FFF300, &ccm->CCGR4);
	writel(0x0F0000C3, &ccm->CCGR5);
	writel(0x000003FF, &ccm->CCGR6);
}

static void gpr_init(void)
{
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;

	/* enable AXI cache for VDOA/VPU/IPU */
	writel(0xF00000CF, &iomux->gpr[4]);
	/* set IPU AXI-id0 Qos=0xf(bypass) AXI-id1 Qos=0x7 */
	writel(0x007F007F, &iomux->gpr[6]);
	writel(0x007F007F, &iomux->gpr[7]);
}

/*
 * This section requires the differentiation between iMX6 Sabre boards, but
 * for now, it will configure only for the mx6q variant.
 */
static void spl_dram_init(void)
{
	struct mx6_ddr_sysinfo sysinfo = {
		/* width of data bus:0=16,1=32,2=64 */
		.dsize = mem_ddr.width/32,
		/* config for full 4GB range so that get_mem_size() works */
		.cs_density = 32, /* 32Gb per CS */
		/* single chip select */
		.ncs = 1,
		.cs1_mirror = 0,
		.rtt_wr = 1 /*DDR3_RTT_60_OHM*/,	/* RTT_Wr = RZQ/4 */
#ifdef RTT_NOM_120OHM
		.rtt_nom = 2 /*DDR3_RTT_120_OHM*/,	/* RTT_Nom = RZQ/2 */
#else
		.rtt_nom = 1 /*DDR3_RTT_60_OHM*/,	/* RTT_Nom = RZQ/4 */
#endif
		.walat = 1,	/* Write additional latency */
		.ralat = 5,	/* Read additional latency */
		.mif3_mode = 3,	/* Command prediction working mode */
		.bi_on = 1,	/* Bank interleaving enabled */
		.sde_to_rst = 0x10,	/* 14 cycles, 200us (JEDEC default) */
		.rst_to_cke = 0x23,	/* 33 cycles, 500us (JEDEC default) */
	};

	mx6dq_dram_iocfg(mem_ddr.width, &mx6_ddr_ioregs, &mx6_grp_ioregs);
	mx6_dram_cfg(&sysinfo, &mx6_mmcd_calib, &mem_ddr);
}

void board_init_f(ulong dummy)
{
	/* setup AIPS and disable watchdog */
	arch_cpu_init();

	ccgr_init();
	gpr_init();

	/* iomux and setup of i2c */
	board_early_init_f();

	/* setup GP timer */
	timer_init();

	/* UART clocks enabled and gd valid - init serial console */
	preloader_console_init();

	/* DDR initialization */
	spl_dram_init();

	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

	/* load/boot image from boot device */
	board_init_r(NULL, 0);
}

void reset_cpu(ulong addr)
{
}
#endif
