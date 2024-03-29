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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/bootmem.h>
#include <linux/io.h>
#include <linux/mfd/pmic8058.h>
#include <linux/mfd/marimba.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/input.h>
#include <linux/smsc911x.h>
#include <linux/ofn_atlab.h>
#include <linux/power_supply.h>
#include <linux/fsa9480.h>

#ifdef CONFIG_SENSORS_YDA165
#include <linux/i2c/yda165.h>
#endif
#include <linux/input/pmic8058-keypad.h>
#include <linux/i2c/isa1200.h>
#include <linux/pwm.h>
#include <linux/pmic8058-pwm.h>
#include <linux/i2c/tsc2007.h>
#include <linux/input/kp_flip_switch.h>
#include <linux/leds-pmic8058.h>
#include <linux/input/cy8c_ts.h>
#include <linux/msm_adc.h>
#include <linux/dma-mapping.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/setup.h>

#include <mach/mpp.h>
#include <mach/gpio.h>
#include <mach/board.h>
#include <mach/camera.h>
#include <mach/memory.h>
#include <mach/msm_iomap.h>
#include <mach/msm_hsusb.h>
#include <mach/rpc_hsusb.h>
#include <mach/msm_spi.h>
#include <mach/qdsp5v2/msm_lpa.h>
#include <mach/dma.h>
#include <linux/android_pmem.h>
#include <linux/input/msm_ts.h>
#include <mach/pmic.h>
#include <mach/rpc_pmapp.h>
#include <mach/qdsp5v2/aux_pcm.h>
#include <mach/qdsp5v2/mi2s.h>
#include <mach/qdsp5v2/audio_dev_ctl.h>
#include <mach/msm_battery.h>
#include <mach/rpc_server_handset.h>
#include <mach/msm_tsif.h>
#include <linux/cyttsp.h>

#include <asm/mach/mmc.h>
#include <asm/mach/flash.h>
#include <mach/vreg.h>
#include "devices.h"
#include "timer.h"
#include <mach/socinfo.h>
#ifdef CONFIG_USB_ANDROID
#include <linux/usb/android_composite.h>
#endif
#include "pm.h"
#include "spm.h"
#include <linux/msm_kgsl.h>
#include <mach/dal_axi.h>
#include <mach/msm_serial_hs.h>
#include <mach/msm_reqs.h>
#ifdef CONFIG_SENSORS_AK8975
#include <linux/i2c/ak8975.h>
#endif

/*
 * sec input bridge for ancora force ramdump
 */
#ifdef CONFIG_INPUT_SECBRIDGE
#include <linux/input/sec-input-bridge.h>
#endif

/* 2011-06-20 hyeokseon.yu */
#ifdef CONFIG_CHARGER_SMB328A
#include <linux/smb328a_charger.h>
#ifndef CONFIG_HW_REV_USING_SMB328
#define CONFIG_HW_REV_USING_SMB328 0x06
#endif
#endif

#ifdef CONFIG_SAMSUNG_JACK
#include <linux/sec_jack.h>
#include <mach/msm_rpcrouter.h>
#include <../../../drivers/bluetooth/bluesleep.c>//sc47.yun
#include <asm/atomic.h>
#include <linux/err.h>
#endif
#include <mach/qdsp5v2/mi2s.h>
#include <mach/qdsp5v2/audio_dev_ctl.h>
#include <mach/sdio_al.h>
#include "smd_private.h"
#ifdef CONFIG_KEYPAD_CYPRESS_TOUCH
#include <linux/input/cypress-touchkey.h>
#if 0//sc47.yun def CONFIG_WIFI_CONTROL_FUNC
#define GPIO_BT_RESET			146
#define GPIO_BT_WLAN_REG_ON		144
#define GPIO_WLAN_RESET			127
#define GPIO_BT_WLAN_REG_ON		144
#define GPIO_WLAN_WAKES_MSM_REV05	111 //WLAN_HOST_WAKE
#define GPIO_WLAN_WAKES_MSM_REV06	111
#endif
#endif

/* hk46.song_20110822 ++ */
#ifdef CONFIG_CONDITIONAL_POWER_COLLAPSE_EAR_MIC_TX
#include "pm.h"
#endif
/* hk46.song_20110822 -- */

//sc47.yun start
#define GPIO_WLAN_RESET			127

#define GPIO_BT_WAKE			147
#define GPIO_BT_HOST_WAKE		145
#define GPIO_BT_WLAN_REG_ON		144
#define GPIO_BT_RESET			146

#define GPIO_BT_UART_RTS		134
#define GPIO_BT_UART_CTS		135
#define GPIO_BT_UART_RXD		136
#define GPIO_BT_UART_TXD		137
#define GPIO_BT_PCM_DOUT		138
#define GPIO_BT_PCM_DIN			139
#define GPIO_BT_PCM_SYNC		140
#define GPIO_BT_PCM_CLK			141

#define GPIO_WLAN_LEVEL_LOW			0
#define GPIO_WLAN_LEVEL_HIGH			1
#define GPIO_WLAN_LEVEL_NONE			2
//sc47.yun end

#define WLAN_EN_GPIO		144 //WLAN_BT_EN
#define WLAN_RESET          127 //Reset
#define WLAN_HOST_WAKE		111
struct class *sec_class;
EXPORT_SYMBOL(sec_class);
struct device *switch_dev;
EXPORT_SYMBOL(switch_dev);

#define MSM_PMEM_SF_SIZE	0x1E00000	// MM team, QC request
#define MSM_FB_SIZE			(0xA46000-0x523000)	// ARGB8888 double duffrting
#define MSM_PMEM_ADSP_SIZE      (0x1E20000+0x523000) // MM team request (before 0x1CD0000)
#define MSM_FLUID_PMEM_ADSP_SIZE	0x2800000
#define PMEM_KERNEL_EBI1_SIZE   0x600000
#define MSM_PMEM_AUDIO_SIZE     0x200000

#define PMIC_GPIO_INT		27
#define PMIC_VREG_WLAN_LEVEL	2900
#define PMIC_GPIO_MICBIAS_EN	14 // PM8058_GPIO(15) 
#define PMIC_GPIO_EAR_MICBIAS_EN	16 // PM8058_GPIO(17) 
#define PMIC_GPIO_EARPATH_SEL	13 // PM8058_GPIO(14) 

#define PMIC_GPIO_SD_DET	36
#define PMIC_GPIO_SDC4_EN_N	17  /* PMIC GPIO Number 18 */
#define PMIC_GPIO_HDMI_5V_EN	32  /* PMIC GPIO Number 33 */

#define LCD_ESD_DET_ENABLE		1

#if LCD_ESD_DET_ENABLE // for ancora LCD ESC DET.. 
#define PMIC_GPIO_LCD_ESD_DET	25 /* PMIC GPIO Number 26*/
#endif
#ifdef CONFIG_SENSORS_YAS529_MAGNETIC
#define MSM_GPIO_MSENSE_RST	180  /* MSM GPIO Number 180 */
#endif
#ifdef CONFIG_GYRO_L3G4200D
#define MSM_GPIO_GYRO_INT		82 //  	/* PMIC GPIO NUMBER 29 */
#endif
#ifdef CONFIG_SENSORS_AK8975
#define MSM_GPIO_MSENSE_RST	180  /* MSM GPIO Number 180 */
#endif

#ifdef CONFIG_SAMSUNG_FM_SI4709
#define GPIO_FM_INT 121
#define GPIO_FM_RST 120
#endif

#define MSM_GPIO_EAR_DET		26  
#define MSM_GPIO_SHORT_SENDEND	44 

#define FPGA_SDCC_STATUS       0x8E0001A8

#define FPGA_OPTNAV_GPIO_ADDR	0x8E000026
#define OPTNAV_I2C_SLAVE_ADDR	(0xB0 >> 1)
#define OPTNAV_IRQ		20
#define OPTNAV_CHIP_SELECT	19

/* Macros assume PMIC GPIOs start at 0 */
#define PM8058_GPIO_PM_TO_SYS(pm_gpio)     (pm_gpio + NR_GPIO_IRQS)
#define PM8058_GPIO_SYS_TO_PM(sys_gpio)    (sys_gpio - NR_GPIO_IRQS)

#define PMIC_GPIO_FLASH_BOOST_ENABLE	15	/* PMIC GPIO Number 16 */
#define PMIC_GPIO_HAP_ENABLE   16  /* PMIC GPIO Number 17 */

#define HAP_LVL_SHFT_MSM_GPIO 24

#define PMIC_GPIO_QUICKVX_CLK 37 /* PMIC GPIO 38 */

#define	PM_FLIP_MPP 5 /* PMIC MPP 06 */

/* 2011-06-27 hyeokseon.yu */
#ifdef CONFIG_CHARGER_SMB328A
#define PM8058_IRQ_BASE				(NR_MSM_IRQS + NR_GPIO_IRQS)
#define PMIC_GPIO_CHG_EN		PM8058_GPIO(33)
#define PMIC_GPIO_CHG_STAT		PM8058_GPIO(34)
#endif


#ifdef CONFIG_OPTICAL_GP2A
#define PMIC_GPIO_PROX_EN	15 /* PMIC GPIO 16 */
#define MSM_GPIO_PS_VOUT	118// [HSS]125 for Test
#endif


#ifdef CONFIG_KEYPAD_CYPRESS_TOUCH
#define _3_TOUCH_INT     84
#define _3_TOUCH_SCL_28V 124
#define _3_TOUCH_SDA_28V 125
#define _3_TOUCH_EN      126
#endif
extern int board_hw_revision;
int on_call_flag;
int on_fmradio_flag;
#ifdef CONFIG_SAMSUNG_JACK
static int tx_set_flag=0;
#endif

#ifdef CONFIG_SAMSUNG_JACK
#ifdef GET_JACK_ADC
static struct sec_jack_zone jack_zones[] = {
	[0] = {
		.adc_high	= 990,
		.delay_ms	= 20,
		.check_count	= 15,
		.jack_type	= SEC_HEADSET_3POLE,
	},
	[1] = {
		.adc_high	= 4096,
		.delay_ms	= 20,
		.check_count	= 15,
		.jack_type	= SEC_HEADSET_4POLE,
	},
};
#else
static struct sec_jack_zone jack_zones[] = {
	[0] = {
		.adc_high	= 0,
		.delay_ms	= 20,
		.check_count	= 15,
		.jack_type	= SEC_HEADSET_4POLE,
	},
	[1] = {
		.adc_high	= 1,
		.delay_ms	= 20,
		.check_count	= 15,
		.jack_type	= SEC_HEADSET_3POLE,
	},
};
#endif

#ifdef JACK_REMOTE_KEY
static struct sec_jack_buttons_zone sec_jack_buttons_zones[] = {
	{
		/* 0 <= adc <=150, stable zone */
		.code		= KEY_MEDIA,
		.adc_low	= 0,
		.adc_high	= 150,
	},
	{
		/* 151 <= adc <= 340, stable zone */
		.code		= KEY_VOLUMEUP,
		.adc_low	= 151,
		.adc_high	= 340,
	},
	{
		/* 341 <= adc <= 690, stable zone */
		.code		= KEY_VOLUMEDOWN,
		.adc_low	= 341,
		.adc_high	= 690,
	},
};
#endif

static int sec_jack_get_det_jack_state(void)
{
	return(gpio_get_value(MSM_GPIO_EAR_DET)) ^ 1;
}

static int sec_jack_get_send_key_state(void)
{
	return(gpio_get_value(MSM_GPIO_SHORT_SENDEND)) ^ 1;
}

#if 1
/* soonyong.cho : refer product id and config string of usb from 'arch/arm/plat-samsung/include/plat/devs.h' */
//#  include <plat/devs.h> 
#include <mach/devs.h>// junsang.yoo

#endif

/* hk46.song_20110822 ++ */
#ifdef CONFIG_CONDITIONAL_POWER_COLLAPSE_EAR_MIC_TX
extern struct msm_pm_platform_data *msm_pm_modes;
static u8 prev_idle_enable_state = 0;
static u8 already_ear_micbias_on = 0;
#endif
/* hk46.song_20110822 -- */

static void sec_jack_set_micbias_state(bool state)
{
	if (tx_set_flag == 1)
	{
		state = 1;
	}

/* hk46.song_20110822 ++ */
#ifdef CONFIG_CONDITIONAL_POWER_COLLAPSE_EAR_MIC_TX
	printk("%s()\n", __func__);

	if((state)&&(already_ear_micbias_on==0))
	{
		prev_idle_enable_state = msm_pm_modes[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].idle_enabled;
		msm_pm_modes[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].idle_enabled = 0; 
		printk("[idle_enabled(%d)] prev(%d) -> set to 0\n", MSM_PM_SLEEP_MODE_POWER_COLLAPSE, prev_idle_enable_state);
		already_ear_micbias_on = 1;
	}
	else
	{
		msm_pm_modes[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].idle_enabled = prev_idle_enable_state; 
		printk("[idle_enabled(%d)] set to %d\n", MSM_PM_SLEEP_MODE_POWER_COLLAPSE, prev_idle_enable_state);
		already_ear_micbias_on = 0;
	}
#endif
/* hk46.song_20110822 -- */

	gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_EAR_MICBIAS_EN), state);	
}

#ifdef GET_JACK_ADC
#define SEC_JACK_RPC_TIMEOUT 			5000	/* 5 sec */
#define PMAPP_GENPROG					0x30000089
#define PMAPP_EARJACK_ADC_VERS 			0x00050001
#define ONCRPC_PMAPP_EARJACK_ADC_PROC	34

static struct sec_jack_get_adc_ret_data {
	u32 sec_jack_adc;
};

static struct msm_rpc_client *sec_jack_client;

static int sec_jack_get_adc_ret_func(struct msm_rpc_client *client,
				       void *buf, void *data)
{
	struct sec_jack_get_adc_ret_data *data_ptr, *buf_ptr;

	data_ptr = (struct sec_jack_get_adc_ret_data *)data;
	buf_ptr = (struct sec_jack_get_adc_ret_data *)buf;

	data_ptr->sec_jack_adc = be32_to_cpu(buf_ptr->sec_jack_adc);

	return 0;
}

static u32 sec_jack_get_adc(void)
{
	int rc;

	struct sec_jack_get_adc_ret_data rep;

  // 		pr_err("[HSS] [%s] Start\n", __func__);


	rc = msm_rpc_client_req(sec_jack_client,
			ONCRPC_PMAPP_EARJACK_ADC_PROC,
			NULL, NULL,
			sec_jack_get_adc_ret_func, &rep,
			msecs_to_jiffies(SEC_JACK_RPC_TIMEOUT));

 //  		pr_err("[HSS] %s: PASS: mpp10 get adc. rep.sec_jack_adc = [%d]\n", __func__, rep.sec_jack_adc);

	if (rc < 0) {
		pr_err("%s: FAIL: mpp10 get adc. rc=%d\n", __func__, rc);
		return 0;
	}

	return rep.sec_jack_adc;
}

static int sec_jack_init_rpc(void)
{
  	int rc = 0;

	sec_jack_client = msm_rpc_register_client("sec_jack", 
					PMAPP_GENPROG,
					PMAPP_EARJACK_ADC_VERS,
					1, NULL);

	if (sec_jack_client == NULL) {
		pr_err("%s: FAIL: rpc_register_client. sec_jack_client=NULL\n",
		       __func__);
		return -ENODEV;
	}
	else if (IS_ERR(sec_jack_client)) {
		sec_jack_client = msm_rpc_register_client("sec_jack", 
						PMAPP_GENPROG,
						PMAPP_EARJACK_ADC_VERS,
						1, NULL);
	} 
	if (IS_ERR(sec_jack_client)) {
		rc = PTR_ERR(sec_jack_client);
		pr_err("%s: ERROR: rpc_register_client: rc = %d\n ",
		       __func__, rc);
		sec_jack_client = NULL;
		return rc;
	}
  return rc;
}

#endif

static int sec_jack_get_adc_value(void)
{
#ifdef GET_JACK_ADC
   u32 adc = sec_jack_get_adc();
   return adc;
#else   
	return(gpio_get_value(MSM_GPIO_SHORT_SENDEND)) ^ 1;
#endif
}

void sec_jack_gpio_init(void)
{
	int rc;
   	struct pm8058_gpio micbias_en = {
		.direction      = PM_GPIO_DIR_OUT,
		.pull           = PM_GPIO_PULL_NO,
		.vin_sel        = 2,
		.function       = PM_GPIO_FUNC_NORMAL,
		.inv_int_pol    = 0,
	};
	struct pm8058_gpio ear_micbias_en = {
		.direction      = PM_GPIO_DIR_OUT,
		.pull           = PM_GPIO_PULL_NO,
		.vin_sel        = 2,
		.function       = PM_GPIO_FUNC_NORMAL,
		.inv_int_pol    = 0,
	};
	
	/* detect pin initialization */
	if (gpio_tlmm_config(GPIO_CFG(MSM_GPIO_EAR_DET, 0, GPIO_CFG_INPUT,
				  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE))
		pr_err("%s: gpio_tlmm_config (gpio=%d) failed\n",
		       __func__, MSM_GPIO_EAR_DET);

	/* sendend pin initialization */
	if (gpio_tlmm_config(GPIO_CFG(MSM_GPIO_SHORT_SENDEND, 0, GPIO_CFG_INPUT,
				  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE))
		pr_err("%s: gpio_tlmm_config (gpio=%d) failed\n",
		       __func__, MSM_GPIO_SHORT_SENDEND);
	
	/* micbias_en pin initialization */
	rc = pm8058_gpio_config(PMIC_GPIO_MICBIAS_EN, &micbias_en);
	if (rc) {
		pr_err("%s PMIC_GPIO_MICBIAS_EN config failed\n", __func__);
	}
	gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_MICBIAS_EN), 0);

   	/* ear_micbias_en pin initialization */
	rc = pm8058_gpio_config(PMIC_GPIO_EAR_MICBIAS_EN, &ear_micbias_en);
	if (rc) {
		pr_err("%s PMIC_GPIO_EAR_MICBIAS_EN config failed\n", __func__);
	}
	gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_EAR_MICBIAS_EN), 0);
   
}

static struct sec_jack_platform_data sec_jack_data = {
	.get_det_jack_state	= sec_jack_get_det_jack_state,
	.get_send_key_state	= sec_jack_get_send_key_state,
	.set_micbias_state	= sec_jack_set_micbias_state,
	.get_adc_value	= sec_jack_get_adc_value,
	.zones		= jack_zones,
	.num_zones	= ARRAY_SIZE(jack_zones),
#ifdef JACK_REMOTE_KEY		
	.buttons_zones		= sec_jack_buttons_zones,
	.num_buttons_zones	= ARRAY_SIZE(sec_jack_buttons_zones),
#endif
	.det_int 	= MSM_GPIO_TO_INT(MSM_GPIO_EAR_DET),
	.send_int 	= MSM_GPIO_TO_INT(MSM_GPIO_SHORT_SENDEND),
#ifdef GET_JACK_ADC
      .rpc_init = sec_jack_init_rpc,
#endif	
};

static struct platform_device sec_device_jack = {
	.name           = "sec_jack",
	.id             = -1,
	.dev            = {
		.platform_data  = &sec_jack_data,
	},
};
#endif

#ifdef CONFIG_OPTICAL_GP2A
static int __init opt_gp2a_gpio_init(void)
{
	if (gpio_tlmm_config(GPIO_CFG(MSM_GPIO_PS_VOUT, 0, GPIO_CFG_INPUT, 
            GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE))
		pr_err("[HSS] %s: gpio_tlmm_config (gpio=%d) failed\n",
		       __func__, MSM_GPIO_PS_VOUT);   
}
#endif

static int pm8058_gpios_init(void)
{
	int rc;
#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
	struct pm8058_gpio sdcc_det = {
		.direction      = PM_GPIO_DIR_IN,
		.pull           = PM_GPIO_PULL_UP_1P5,
		.vin_sel        = 2,
		.function       = PM_GPIO_FUNC_NORMAL,
		.inv_int_pol    = 0,
	};
#endif
	struct pm8058_gpio sdc4_en = {
		.direction      = PM_GPIO_DIR_OUT,
		.pull           = PM_GPIO_PULL_NO,
		.vin_sel        = PM_GPIO_VIN_L5,
		.function       = PM_GPIO_FUNC_NORMAL,
		.inv_int_pol    = 0,
		.out_strength   = PM_GPIO_STRENGTH_LOW,
		.output_value   = 0,
	};

	struct pm8058_gpio earpath_sel = {
		.direction      = PM_GPIO_DIR_OUT,
		.pull           = PM_GPIO_PULL_NO,
		.vin_sel        = 2,
		.function       = PM_GPIO_FUNC_NORMAL,
		.inv_int_pol    = 0,
	};

	struct pm8058_gpio haptics_enable = {
		.direction      = PM_GPIO_DIR_OUT,
		.pull           = PM_GPIO_PULL_NO,
		.out_strength   = PM_GPIO_STRENGTH_HIGH,
		.function       = PM_GPIO_FUNC_NORMAL,
		.inv_int_pol    = 0,
		.vin_sel        = 2,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.output_value   = 0,
	};

	struct pm8058_gpio hdmi_5V_en = {
		.direction      = PM_GPIO_DIR_OUT,
		.pull           = PM_GPIO_PULL_NO,
		.vin_sel        = PM_GPIO_VIN_VPH,
		.function       = PM_GPIO_FUNC_NORMAL,
		.out_strength   = PM_GPIO_STRENGTH_LOW,
		.output_value   = 0,
	};
#if LCD_ESD_DET_ENABLE // for ancora LCD ESC DET.. 
	struct pm8058_gpio lcd_det = {
		.direction      = PM_GPIO_DIR_IN,
		.pull 			= PM_GPIO_PULL_UP_1P5, /* Pull up */ 
		.vin_sel        = 2,
		.function       = PM_GPIO_FUNC_NORMAL,
		.inv_int_pol    = 0,
	};
#endif 
#ifdef CONFIG_GYRO_L3G4200D
/*
	struct pm8058_gpio gyro_int = {
		.direction      = PM_GPIO_DIR_IN,
		.pull           = PM_GPIO_PULL_DN,
		.vin_sel        = 2,
		.function       = PM_GPIO_FUNC_NORMAL,
		.inv_int_pol    = 0,
	};
*/	
#endif

#ifdef CONFIG_OPTICAL_GP2A
	struct pm8058_gpio prox_en = {
		.direction      = PM_GPIO_DIR_OUT,
		.pull           = PM_GPIO_PULL_NO,
		.vin_sel        = 2,
		.function       = PM_GPIO_FUNC_NORMAL,
		.inv_int_pol    = 0,
	};
#endif

	struct pm8058_gpio flash_boost_enable = {
		.direction      = PM_GPIO_DIR_OUT,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.output_value   = 0,
		.pull           = PM_GPIO_PULL_NO,
		.vin_sel        = PM_GPIO_VIN_S3,
		.out_strength   = PM_GPIO_STRENGTH_HIGH,
		.function       = PM_GPIO_FUNC_2,
	};

/* 2011-06-27 hyeokseon.yu */
#ifdef CONFIG_CHARGER_SMB328A

	 /* not used, set intput */
	 struct pm8058_gpio chg_en = {
		 .direction 	 = PM_GPIO_DIR_IN,
		 .pull			 = PM_GPIO_PULL_NO,
		 .vin_sel		 = 2,
		 .function		 = PM_GPIO_FUNC_NORMAL,
		 .inv_int_pol	 = 0,
	 };
 
	 /* not used */
	 struct pm8058_gpio chg_stat = {
		 .direction 	 = PM_GPIO_DIR_IN,
		 .pull			 = PM_GPIO_PULL_NO,
		 .vin_sel		 = 2,
		 .function		 = PM_GPIO_FUNC_NORMAL,
		 .inv_int_pol	 = 0,
	 };
#endif


	if (machine_is_msm7x30_fluid()) {
		rc = pm8058_gpio_config(PMIC_GPIO_HAP_ENABLE, &haptics_enable);
		if (rc) {
			pr_err("%s: PMIC GPIO %d write failed\n", __func__,
				(PMIC_GPIO_HAP_ENABLE + 1));
			return rc;
		}
		rc = pm8058_gpio_config(PMIC_GPIO_FLASH_BOOST_ENABLE,
			&flash_boost_enable);
		if (rc) {
			pr_err("%s: PMIC GPIO %d write failed\n", __func__,
				(PMIC_GPIO_FLASH_BOOST_ENABLE + 1));
			return rc;
		}
	}

#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
	if (machine_is_msm7x30_fluid())
		sdcc_det.inv_int_pol = 1;

	rc = pm8058_gpio_config(PMIC_GPIO_SD_DET - 1, &sdcc_det);
	if (rc) {
		pr_err("%s PMIC_GPIO_SD_DET config failed\n", __func__);
		return rc;
	}
#endif

	rc = pm8058_gpio_config(PMIC_GPIO_HDMI_5V_EN, &hdmi_5V_en);
	if (rc) {
		pr_err("%s PMIC_GPIO_HDMI_5V_EN config failed\n", __func__);
		return rc;
	}

	if (machine_is_msm7x30_fluid()) {
		rc = pm8058_gpio_config(PMIC_GPIO_SDC4_EN_N, &sdc4_en);
		if (rc) {
			pr_err("%s PMIC_GPIO_SDC4_EN_N config failed\n",
								 __func__);
			return rc;
		}
		rc = gpio_request(PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_SDC4_EN_N),
				  "sdc4_en");
		if (rc) {
			pr_err("%s PMIC_GPIO_SDC4_EN_N gpio_request failed\n",
				__func__);
			return rc;
		}
		gpio_set_value_cansleep(
			PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_SDC4_EN_N), 0);
	}

	rc = pm8058_gpio_config(PMIC_GPIO_EARPATH_SEL, &earpath_sel);
	if (rc) {
		pr_err("%s PMIC_GPIO_EARPATH_SEL config failed\n", __func__);
		return rc;
	}
	gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_EARPATH_SEL), 1);
#if LCD_ESD_DET_ENABLE // for ancora LCD ESC DET.. 
	rc = pm8058_gpio_config(PMIC_GPIO_LCD_ESD_DET , &lcd_det);
	if (rc) {
		pr_err("%s PMIC_GPIO_LCD_ESD_DET config failed\n", __func__);
		return rc;
	}
#endif
/* 2011-06-27 hyeokseon.yu */
#ifdef CONFIG_CHARGER_SMB328A
 	rc = pm8058_gpio_config(PMIC_GPIO_CHG_EN, &chg_en);
 	if (rc) {
 		pr_err("%s PMIC_GPIO_CHG_EN config failed\n", __func__);
 		return rc;
 	}
 
 	rc = pm8058_gpio_config(PMIC_GPIO_CHG_STAT, &chg_stat);
 	if (rc) {
 		pr_err("%s PMIC_GPIO_CHG_STAT config failed\n", __func__);
 		return rc;
	}
#endif


#ifdef CONFIG_GYRO_L3G4200D
// [HSS] Where will it initialized? : MSM_GPIO_GYRO_INT
/* 
	rc = pm8058_gpio_config(PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_GYRO_INT), &gyro_int);
	if (rc) {
		pr_err("%s PMIC_GPIO_GYRO_INT config failed\n", __func__);
		return rc;
	}
	//gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_GYRO_INT), 0);
*/
	
#endif

#ifdef CONFIG_OPTICAL_GP2A
	rc = pm8058_gpio_config(PMIC_GPIO_PROX_EN, &prox_en);
	if (rc) {
			pr_err("%s PMIC_GPIO_PROX_EN config failed\n", __func__);
			return rc;
	}
      gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_PROX_EN), 1);

      opt_gp2a_gpio_init();

#endif  

#ifdef CONFIG_SAMSUNG_JACK
	sec_jack_gpio_init();
#endif

	return 0;
}

/*virtual key support */
static ssize_t tma300_vkeys_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf,
	__stringify(EV_KEY) ":" __stringify(KEY_BACK) ":50:842:80:100"
	":" __stringify(EV_KEY) ":" __stringify(KEY_MENU) ":170:842:80:100"
	":" __stringify(EV_KEY) ":" __stringify(KEY_HOME) ":290:842:80:100"
	":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":410:842:80:100"
	"\n");
}

static struct kobj_attribute tma300_vkeys_attr = {
	.attr = {
		.mode = S_IRUGO,
	},
	.show = &tma300_vkeys_show,
};

static struct attribute *tma300_properties_attrs[] = {
	&tma300_vkeys_attr.attr,
	NULL
};

static struct attribute_group tma300_properties_attr_group = {
	.attrs = tma300_properties_attrs,
};

static struct kobject *properties_kobj;

#define CYTTSP_TS_GPIO_IRQ	150
static int cyttsp_platform_init(struct i2c_client *client)
{
	int rc = -EINVAL;
	struct vreg *vreg_ldo8, *vreg_ldo15;

	vreg_ldo8 = vreg_get(NULL, "gp7");

	if (!vreg_ldo8) {
		pr_err("%s: VREG L8 get failed\n", __func__);
		return rc;
	}

	rc = vreg_set_level(vreg_ldo8, 1800);
	if (rc) {
		pr_err("%s: VREG L8 set failed\n", __func__);
		goto l8_put;
	}

	rc = vreg_enable(vreg_ldo8);
	if (rc) {
		pr_err("%s: VREG L8 enable failed\n", __func__);
		goto l8_put;
	}

	vreg_ldo15 = vreg_get(NULL, "gp6");

	if (!vreg_ldo15) {
		pr_err("%s: VREG L15 get failed\n", __func__);
		goto l8_disable;
	}

	rc = vreg_set_level(vreg_ldo15, 3050);
	if (rc) {
		pr_err("%s: VREG L15 set failed\n", __func__);
		goto l8_disable;
	}

	rc = vreg_enable(vreg_ldo15);
	if (rc) {
		pr_err("%s: VREG L15 enable failed\n", __func__);
		goto l8_disable;
	}

	/* check this device active by reading first byte/register */
	rc = i2c_smbus_read_byte_data(client, 0x01);
	if (rc < 0) {
		pr_err("%s: i2c sanity check failed\n", __func__);
		goto l8_disable;
	}

	rc = gpio_tlmm_config(GPIO_CFG(CYTTSP_TS_GPIO_IRQ, 0, GPIO_CFG_INPUT,
					GPIO_CFG_PULL_UP, GPIO_CFG_6MA), GPIO_CFG_ENABLE);
	if (rc) {
		pr_err("%s: Could not configure gpio %d\n",
					 __func__, CYTTSP_TS_GPIO_IRQ);
		goto l8_disable;
	}

	rc = gpio_request(CYTTSP_TS_GPIO_IRQ, "ts_irq");
	if (rc) {
		pr_err("%s: unable to request gpio %d (%d)\n",
			__func__, CYTTSP_TS_GPIO_IRQ, rc);
		goto l8_disable;
	}

	/* virtual keys */
	tma300_vkeys_attr.attr.name = "virtualkeys.cyttsp-i2c";
	properties_kobj = kobject_create_and_add("board_properties",
				NULL);
	if (properties_kobj)
		rc = sysfs_create_group(properties_kobj,
			&tma300_properties_attr_group);
	if (!properties_kobj || rc)
		pr_err("%s: failed to create board_properties\n",
				__func__);

	return CY_OK;

l8_disable:
	vreg_disable(vreg_ldo8);
l8_put:
	vreg_put(vreg_ldo8);
	return rc;
}

static int cyttsp_platform_resume(struct i2c_client *client)
{
	/* add any special code to strobe a wakeup pin or chip reset */
	mdelay(10);

	return CY_OK;
}

static struct cyttsp_platform_data cyttsp_data = {
	.panel_maxx = 479,
	.panel_maxy = 799,
	.disp_maxx = 469,
	.disp_maxy = 799,
	.disp_minx = 10,
	.disp_miny = 0,
	.flags = 0,
	.gen = CY_GEN3,	/* or */
	.use_st = CY_USE_ST,
	.use_mt = CY_USE_MT,
	.use_hndshk = CY_SEND_HNDSHK,
	.use_trk_id = CY_USE_TRACKING_ID,
	.use_sleep = CY_USE_SLEEP,
	.use_gestures = CY_USE_GESTURES,
	/* activate up to 4 groups
	 * and set active distance
	 */
	.gest_set = CY_GEST_GRP1 | CY_GEST_GRP2 |
				CY_GEST_GRP3 | CY_GEST_GRP4 |
				CY_ACT_DIST,
	/* change act_intrvl to customize the Active power state
	 * scanning/processing refresh interval for Operating mode
	 */
	.act_intrvl = CY_ACT_INTRVL_DFLT,
	/* change tch_tmout to customize the touch timeout for the
	 * Active power state for Operating mode
	 */
	.tch_tmout = CY_TCH_TMOUT_DFLT,
	/* change lp_intrvl to customize the Low Power power state
	 * scanning/processing refresh interval for Operating mode
	 */
	.lp_intrvl = CY_LP_INTRVL_DFLT,
	.resume = cyttsp_platform_resume,
	.init = cyttsp_platform_init,
};

static int pm8058_pwm_config(struct pwm_device *pwm, int ch, int on)
{
	struct pm8058_gpio pwm_gpio_config = {
		.direction      = PM_GPIO_DIR_OUT,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.output_value   = 0,
		.pull           = PM_GPIO_PULL_NO,
		.vin_sel        = PM_GPIO_VIN_S3,
		.out_strength   = PM_GPIO_STRENGTH_HIGH,
		.function       = PM_GPIO_FUNC_2,
	};
	int	rc = -EINVAL;
	int	id, mode, max_mA;

	id = mode = max_mA = 0;
	switch (ch) {
	case 0:
	case 1:
	case 2:
		if (on) {
			id = 24 + ch;
			rc = pm8058_gpio_config(id - 1, &pwm_gpio_config);
			if (rc)
				pr_err("%s: pm8058_gpio_config(%d): rc=%d\n",
				       __func__, id, rc);
		}
		break;

	case 3:
		id = PM_PWM_LED_KPD;
		mode = PM_PWM_CONF_DTEST3;
		max_mA = 200;
		break;

	case 4:
		id = PM_PWM_LED_0;
		mode = PM_PWM_CONF_PWM1;
		max_mA = 40;
		break;

	case 5:
		id = PM_PWM_LED_2;
		mode = PM_PWM_CONF_PWM2;
		max_mA = 40;
		break;

	case 6:
		id = PM_PWM_LED_FLASH;
		mode = PM_PWM_CONF_DTEST3;
		max_mA = 200;
		break;

	default:
		break;
	}

	if (ch >= 3 && ch <= 6) {
		if (!on) {
			mode = PM_PWM_CONF_NONE;
			max_mA = 0;
		}
		rc = pm8058_pwm_config_led(pwm, id, mode, max_mA);
		if (rc)
			pr_err("%s: pm8058_pwm_config_led(ch=%d): rc=%d\n",
			       __func__, ch, rc);
	}

	return rc;
}

static int pm8058_pwm_enable(struct pwm_device *pwm, int ch, int on)
{
	int	rc;

	switch (ch) {
	case 7:
		rc = pm8058_pwm_set_dtest(pwm, on);
		if (rc)
			pr_err("%s: pwm_set_dtest(%d): rc=%d\n",
			       __func__, on, rc);
		break;
	default:
		rc = -EINVAL;
		break;
	}
	return rc;
}

#ifdef CONFIG_INPUT_SECBRIDGE
/*
 * sec-input-bridge
 */
static const struct sec_input_bridge_mkey ancora_mkey_map[] = {
    { .type = EV_KEY, .code = KEY_VOLUMEUP      },
    { .type = EV_KEY, .code = KEY_HOME          },
    { .type = EV_KEY, .code = KEY_VOLUMEDOWN    },
    { .type = EV_KEY, .code = KEY_VOLUMEUP      },
    { .type = EV_KEY, .code = KEY_HOME          },
    { .type = EV_KEY, .code = KEY_VOLUMEDOWN    },
    { .type = EV_KEY, .code = KEY_VOLUMEUP      },
    { .type = EV_KEY, .code = KEY_HOME          },
    { .type = EV_KEY, .code = KEY_VOLUMEDOWN    },
};

static struct sec_input_bridge_platform_data ancora_input_bridge_data = {

    .mkey_map = ancora_mkey_map,
    .num_mkey = ARRAY_SIZE(ancora_mkey_map),

};

static struct platform_device ancora_input_bridge = {
    .name   = "samsung_input_bridge",
    .id     = -1,
    .dev    = {
        .platform_data = &ancora_input_bridge_data,
    },
};
#endif

static const unsigned int ancora_keymap[] = {
	KEY(0, 0, KEY_RESERVED),
	KEY(0, 1, KEY_RESERVED),
	KEY(0, 2, KEY_RESERVED),

	KEY(1, 0, KEY_RESERVED),
	KEY(1, 1, KEY_VOLUMEDOWN),
	KEY(1, 2, KEY_VOLUMEUP),
};

static struct resource resources_keypad[] = {
	{
		.start	= PM8058_KEYPAD_IRQ(PMIC8058_IRQ_BASE),
		.end	= PM8058_KEYPAD_IRQ(PMIC8058_IRQ_BASE),
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= PM8058_KEYSTUCK_IRQ(PMIC8058_IRQ_BASE),
		.end	= PM8058_KEYSTUCK_IRQ(PMIC8058_IRQ_BASE),
		.flags	= IORESOURCE_IRQ,
	},
};

static struct matrix_keymap_data ancora_keymap_data = {
        .keymap_size    = ARRAY_SIZE(ancora_keymap),
        .keymap         = ancora_keymap,
};


static struct pmic8058_keypad_data ancora_keypad_data = {
	.input_name		= "sec_key",
	.input_phys_device	= "ancora_keypad/input0",
	.num_rows		= 5,
	.num_cols		= 5,
	.rows_gpio_start	= 8,
	.cols_gpio_start	= 0,
	.debounce_ms		= {8, 10},
	.scan_delay_ms		= 32,
	.row_hold_ns		= 91500,
	.wakeup			= 1,
	.keymap_data            = &ancora_keymap_data,
};

static struct pm8058_pwm_pdata pm8058_pwm_data = {
	.config		= pm8058_pwm_config,
	.enable		= pm8058_pwm_enable,
};

/* Put sub devices with fixed location first in sub_devices array */
#define	PM8058_SUBDEV_KPD	0
#define	PM8058_SUBDEV_LED	1

static struct pm8058_gpio_platform_data pm8058_gpio_data = {
	.gpio_base	= PM8058_GPIO_PM_TO_SYS(0),
	.irq_base	= PM8058_GPIO_IRQ(PMIC8058_IRQ_BASE, 0),
	.init		= pm8058_gpios_init,
};

static struct pm8058_gpio_platform_data pm8058_mpp_data = {
	.gpio_base	= PM8058_GPIO_PM_TO_SYS(PM8058_GPIOS),
	.irq_base	= PM8058_MPP_IRQ(PMIC8058_IRQ_BASE, 0),
};

static struct pmic8058_led pmic8058_ffa_leds[] = {
	[0] = {
		.name		= "keyboard-backlight",
		.max_brightness = 15,
		.id		= PMIC8058_ID_LED_KB_LIGHT,
	},
};

static struct pmic8058_leds_platform_data pm8058_ffa_leds_data = {
	.num_leds = ARRAY_SIZE(pmic8058_ffa_leds),
	.leds	= pmic8058_ffa_leds,
};

static struct pmic8058_led pmic8058_surf_leds[] = {
	[0] = {
		.name		= "keyboard-backlight",
		.max_brightness = 15,
		.id		= PMIC8058_ID_LED_KB_LIGHT,
	},
	[1] = {
		.name		= "voice:red",
		.max_brightness = 20,
		.id		= PMIC8058_ID_LED_0,
	},
	[2] = {
		.name		= "wlan:green",
		.max_brightness = 20,
		.id		= PMIC8058_ID_LED_2,
	},
};

static struct mfd_cell pm8058_subdevs[] = {
	{	.name = "pm8058-keypad",
		.id		= -1,
		.num_resources	= ARRAY_SIZE(resources_keypad),
		.resources	= resources_keypad,
	},
	{	.name = "pm8058-led",
		.id		= -1,
	},
	{	.name = "pm8058-gpio",
		.id		= -1,
		.platform_data	= &pm8058_gpio_data,
		.data_size	= sizeof(pm8058_gpio_data),
	},
	{	.name = "pm8058-mpp",
		.id		= -1,
		.platform_data	= &pm8058_mpp_data,
		.data_size	= sizeof(pm8058_mpp_data),
	},
	{	.name = "pm8058-pwm",
		.id		= -1,
		.platform_data	= &pm8058_pwm_data,
		.data_size	= sizeof(pm8058_pwm_data),
	},
	{	.name = "pm8058-nfc",
		.id		= -1,
	},
	{	.name = "pm8058-upl",
		.id		= -1,
	},
};

static struct pmic8058_leds_platform_data pm8058_surf_leds_data = {
	.num_leds = ARRAY_SIZE(pmic8058_surf_leds),
	.leds	= pmic8058_surf_leds,
};

static struct pmic8058_led pmic8058_fluid_leds[] = {
	[0] = {
		.name		= "keyboard-backlight",
		.max_brightness = 15,
		.id		= PMIC8058_ID_LED_KB_LIGHT,
	},
	[1] = {
		.name		= "flash:led_0",
		.max_brightness = 15,
		.id		= PMIC8058_ID_FLASH_LED_0,
	},
	[2] = {
		.name		= "flash:led_1",
		.max_brightness = 15,
		.id		= PMIC8058_ID_FLASH_LED_1,
	},
};

static struct pmic8058_leds_platform_data pm8058_fluid_leds_data = {
	.num_leds = ARRAY_SIZE(pmic8058_fluid_leds),
	.leds	= pmic8058_fluid_leds,
};

static struct pm8058_platform_data pm8058_7x30_data = {
	.irq_base = PMIC8058_IRQ_BASE,

	.num_subdevs = ARRAY_SIZE(pm8058_subdevs),
	.sub_devices = pm8058_subdevs,
	.irq_trigger_flags = IRQF_TRIGGER_LOW,	
};

static struct i2c_board_info pm8058_boardinfo[] __initdata = {
	{
		I2C_BOARD_INFO("pm8058-core", 0x55),
		.irq = MSM_GPIO_TO_INT(PMIC_GPIO_INT),
		.platform_data = &pm8058_7x30_data,
	},
};

static struct i2c_board_info cy8info[] __initdata = {
	{
		I2C_BOARD_INFO(CY_I2C_NAME, 0x24),
		.platform_data = &cyttsp_data,
#ifndef CY_USE_TIMER
		.irq = MSM_GPIO_TO_INT(CYTTSP_TS_GPIO_IRQ),
#endif /* CY_USE_TIMER */
	},
};

static struct i2c_board_info msm_camera_boardinfo[] __initdata = {
#ifdef CONFIG_MT9D112
	{
		I2C_BOARD_INFO("mt9d112", 0x78 >> 1),
	},
#endif
#ifdef CONFIG_S5K3E2FX
	{
		I2C_BOARD_INFO("s5k3e2fx", 0x20 >> 1),
	},
#endif
#ifdef CONFIG_MT9P012
	{
		I2C_BOARD_INFO("mt9p012", 0x6C >> 1),
	},
#endif
#ifdef CONFIG_VX6953
	{
		I2C_BOARD_INFO("vx6953", 0x20),
	},
#endif
#ifdef CONFIG_SN12M0PZ
	{
		I2C_BOARD_INFO("sn12m0pz", 0x34 >> 1),
	},
#endif
#if defined(CONFIG_MT9T013) || defined(CONFIG_SENSORS_MT9T013)
	{
		I2C_BOARD_INFO("mt9t013", 0x6C),
	},
#endif
#if defined (CONFIG_SENSOR_MC7)
	{
		I2C_BOARD_INFO("lsi_mc7", 0x3F >> 1),
		.irq = MSM_GPIO_TO_INT(175),
	},
#endif
#if defined (CONFIG_SENSOR_CE147)
	{
		I2C_BOARD_INFO("ce147", 0x78 >> 1),
	},
#endif
#ifdef CONFIG_SENSOR_S5KA3DFX
	{
		I2C_BOARD_INFO("s5ka3dfx_i2c", 0xc4>>1),
	},
#endif
#ifdef CONFIG_SENSOR_S5K4ECGX
	{
		I2C_BOARD_INFO("s5k4ecgx", 0x5A>>1),
	},
#endif
#ifdef CONFIG_SENSOR_SR030PC30
	{
		I2C_BOARD_INFO("sr030pc30_i2c", 0x60>>1),
	},
#endif
#ifdef CONFIG_SENSOR_S5K5CCAF //PCAM
	{
		I2C_BOARD_INFO("s5k5ccaf", 0x78>>1),
	},
#endif
};

static struct i2c_board_info msm_camera_rev01_boardinfo[] __initdata = {    //[diony] rear camera slave adress change (rev0.1)
#ifdef CONFIG_MT9D112
	{
		I2C_BOARD_INFO("mt9d112", 0x78 >> 1),
	},
#endif
#ifdef CONFIG_S5K3E2FX
	{
		I2C_BOARD_INFO("s5k3e2fx", 0x20 >> 1),
	},
#endif
#ifdef CONFIG_MT9P012
	{
		I2C_BOARD_INFO("mt9p012", 0x6C >> 1),
	},
#endif
#ifdef CONFIG_VX6953
	{
		I2C_BOARD_INFO("vx6953", 0x20),
	},
#endif
#ifdef CONFIG_SN12M0PZ
	{
		I2C_BOARD_INFO("sn12m0pz", 0x34 >> 1),
	},
#endif
#if defined(CONFIG_MT9T013) || defined(CONFIG_SENSORS_MT9T013)
	{
		I2C_BOARD_INFO("mt9t013", 0x6C),
	},
#endif
#if defined (CONFIG_SENSOR_MC7)
	{
		I2C_BOARD_INFO("lsi_mc7", 0x3F >> 1),
		.irq = MSM_GPIO_TO_INT(175),
	},
#endif
#if defined (CONFIG_SENSOR_CE147)
	{
		I2C_BOARD_INFO("ce147", 0x78 >> 1),
	},
#endif
#ifdef CONFIG_SENSOR_S5KA3DFX
	{
		I2C_BOARD_INFO("s5ka3dfx_i2c", 0xc4>>1),
	},
#endif
#ifdef CONFIG_SENSOR_S5K4ECGX
	{
		I2C_BOARD_INFO("s5k4ecgx", 0xAC>>1),
	},
#endif
#ifdef CONFIG_SENSOR_SR030PC30
	{
		I2C_BOARD_INFO("sr030pc30_i2c", 0x60>>1),
	},
#endif
#ifdef CONFIG_SENSOR_S5K5CCAF
	{
		I2C_BOARD_INFO("s5k5ccaf", 0x78>>1), 
	},

#endif
};

//#ifdef CONFIG_SAMSUNG_FM_SI4709
static struct i2c_board_info si4709_info[] __initdata = {
  {
		I2C_BOARD_INFO("Si4709", 0x20 >> 1),
		.irq=MSM_GPIO_TO_INT(121),
	},
};
//#endif

#ifdef CONFIG_MSM_CAMERA
static uint32_t camera_off_vcm_gpio_table[] = {
GPIO_CFG(1, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* VCM */
};

static uint32_t camera_off_gpio_table[] = {
#if defined (CONFIG_SENSOR_CE147) || defined (CONFIG_SENSOR_S5K4ECGX) || defined (CONFIG_SENSOR_S5K5CCAF)
#if !defined (CONFIG_USE_QUP_I2C)
	GPIO_CFG(0, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),	/* CAM_SCL */
	GPIO_CFG(1, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),	/* CAM_SDA */
#endif	
	GPIO_CFG(2, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), 	/* CAM PMIC lp8720 enable pull down to zero*/	
	GPIO_CFG(3, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), 	/* CAM EN2*/	
	GPIO_CFG(31, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(132, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	
	GPIO_CFG(164, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),	/* CAM_PM_SCL */
	GPIO_CFG(165, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),	/* CAM_PM_SDA */	
	GPIO_CFG(174, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* CAM_MEGA_nRST; its is pull down to zero*/	
	GPIO_CFG(175, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* CAM_MEGA_EN; its is pull down to zero*/	
	GPIO_CFG(177, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* CAM_EN3 */
	
	GPIO_CFG(4,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT4 */
	GPIO_CFG(5,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT5 */
	GPIO_CFG(6,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT6 */
	GPIO_CFG(7,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT7 */
	GPIO_CFG(8,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT8 */
	GPIO_CFG(9,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT9 */
	GPIO_CFG(10, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT10 */
	GPIO_CFG(11, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT11 */
	GPIO_CFG(12, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* PCLK */
	GPIO_CFG(13, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* HSYNC_IN */
	GPIO_CFG(14, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* VSYNC_IN */
	GPIO_CFG(15, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* MCLK */
#elif defined (CONFIG_SENSOR_MC7)	 
	GPIO_CFG(15, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* MCLK */
	GPIO_CFG(2, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), 	/* CAM PMIC lp8720 enable pull down to zero*/
	GPIO_CFG(0, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),	/* CAM_SCL */
	GPIO_CFG(1, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),	/* CAM_SDA */
	GPIO_CFG(164, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* CAM_PM_SCL */
	GPIO_CFG(165, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* CAM_PM_SDA */
#else
	/* parallel CAMERA interfaces */
	GPIO_CFG(0,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* RST */
	GPIO_CFG(2,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT2 */
	GPIO_CFG(3,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT3 */
	GPIO_CFG(4,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT4 */
	GPIO_CFG(5,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT5 */
	GPIO_CFG(6,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT6 */
	GPIO_CFG(7,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT7 */
	GPIO_CFG(8,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT8 */
	GPIO_CFG(9,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT9 */
	GPIO_CFG(10, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT10 */
	GPIO_CFG(11, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT11 */
	GPIO_CFG(12, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* PCLK */
	GPIO_CFG(13, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* HSYNC_IN */
	GPIO_CFG(14, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* VSYNC_IN */
	GPIO_CFG(15, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* MCLK */
#endif

};

static uint32_t camera_on_vcm_gpio_table[] = {
GPIO_CFG(1, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), /* VCM */
};

static uint32_t camera_on_gpio_table[] = {
#if defined (CONFIG_SENSOR_CE147) || defined (CONFIG_SENSOR_S5K4ECGX) || defined (CONFIG_SENSOR_S5K5CCAF)
#if !defined (CONFIG_USE_QUP_I2C)
	GPIO_CFG(0, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),	/* CAM_SCL */
	GPIO_CFG(1, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),	/* CAM_SDA */
#endif	
	GPIO_CFG(2, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), 	/* CAM PMIC lp8720 enable pull down to zero*/	
	GPIO_CFG(3, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), 	/* CAM EN2*/	
	GPIO_CFG(31, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(132, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	
	GPIO_CFG(164, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),	/* CAM_PM_SCL */
	GPIO_CFG(165, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),	/* CAM_PM_SDA */	
	GPIO_CFG(174, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* CAM_MEGA_nRST; its is pull down to zero*/	
	GPIO_CFG(175, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* CAM_MEGA_EN; its is pull down to zero*/	
	GPIO_CFG(177, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* CAM_EN3 */
	
	GPIO_CFG(4,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT4 */
	GPIO_CFG(5,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT5 */
	GPIO_CFG(6,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT6 */
	GPIO_CFG(7,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT7 */
	GPIO_CFG(8,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT8 */
	GPIO_CFG(9,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT9 */
	GPIO_CFG(10, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT10 */
	GPIO_CFG(11, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT11 */
	GPIO_CFG(12, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* PCLK */
	GPIO_CFG(13, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* HSYNC_IN */
	GPIO_CFG(14, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* VSYNC_IN */	
	GPIO_CFG(15, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* MCLK */
#elif defined (CONFIG_SENSOR_MC7)	 
	/* In GPIO_CFG the func means which function you want to selec; it can be 0, 1, 2.. */
	GPIO_CFG(31, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(132, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(174, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* CAM_MEGA_nRST; its is pull down to zero*/
	GPIO_CFG(15, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* MCLK; this clock is only enabled during sensor ON/OFF sequence*/
	GPIO_CFG(2, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), 	/* CAM PMIC lp8720 enable pull down to zero*/
	GPIO_CFG(0, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),	/* CAM_SCL */
	GPIO_CFG(1, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),	/* CAM_SDA */
	GPIO_CFG(164, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),	/* CAM_PM_SCL */
	GPIO_CFG(165, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),	/* CAM_PM_SDA */
	//GPIO_CFG(117, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* FLASH_SET it is MD0 of MC7 --- Not used */
	//GPIO_CFG(133, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* FLASH_EN its is MD1 of MC7 --- Not used */	
#else	
	/* parallel CAMERA interfaces */
	GPIO_CFG(0,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* RST */
	GPIO_CFG(2,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT2 */
	GPIO_CFG(3,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT3 */
	GPIO_CFG(4,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT4 */
	GPIO_CFG(5,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT5 */
	GPIO_CFG(6,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT6 */
	GPIO_CFG(7,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT7 */
	GPIO_CFG(8,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT8 */
	GPIO_CFG(9,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT9 */
	GPIO_CFG(10, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT10 */
	GPIO_CFG(11, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT11 */
	GPIO_CFG(12, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* PCLK */
	GPIO_CFG(13, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* HSYNC_IN */
	GPIO_CFG(14, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* VSYNC_IN */
	GPIO_CFG(15, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* MCLK */
#endif	
};

static uint32_t camera_off_gpio_fluid_table[] = {
	/* FLUID: CAM_VGA_RST_N */
	GPIO_CFG(31, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/* FLUID: Disable CAMIF_STANDBY */
	GPIO_CFG(143, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
};

static uint32_t camera_on_gpio_fluid_table[] = {
	/* FLUID: CAM_VGA_RST_N */
	GPIO_CFG(31, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/* FLUID: Disable CAMIF_STANDBY */
	GPIO_CFG(143, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA)
};

static void config_gpio_table(uint32_t *table, int len)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n], GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
}

static int config_camera_on_gpios(void)
{
	printk("[CAMDRV] config_camera_on_gpios \n");
	config_gpio_table(camera_on_gpio_table,
		ARRAY_SIZE(camera_on_gpio_table));
#ifdef NOT_USE
	if (adie_get_detected_codec_type() != TIMPANI_ID)
		/* GPIO1 is shared also used in Timpani RF card so
		only configure it for non-Timpani RF card */
		config_gpio_table(camera_on_vcm_gpio_table,
			ARRAY_SIZE(camera_on_vcm_gpio_table));

	if (machine_is_msm7x30_fluid()) {
		config_gpio_table(camera_on_gpio_fluid_table,
			ARRAY_SIZE(camera_on_gpio_fluid_table));
		/* FLUID: turn on 5V booster */
		gpio_set_value(
			PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_FLASH_BOOST_ENABLE), 1);
	}
#endif	
	return 0;
}

static void config_camera_off_gpios(void)
{
		printk("[CAMDRV] config_camera_off_gpios \n");
	config_gpio_table(camera_off_gpio_table,
		ARRAY_SIZE(camera_off_gpio_table));
#ifdef NOT_USE
	if (adie_get_detected_codec_type() != TIMPANI_ID)
		/* GPIO1 is shared also used in Timpani RF card so
		only configure it for non-Timpani RF card */
		config_gpio_table(camera_off_vcm_gpio_table,
			ARRAY_SIZE(camera_off_vcm_gpio_table));

	if (machine_is_msm7x30_fluid()) {
		config_gpio_table(camera_off_gpio_fluid_table,
			ARRAY_SIZE(camera_off_gpio_fluid_table));
		/* FLUID: turn off 5V booster */
		gpio_set_value(
			PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_FLASH_BOOST_ENABLE), 0);
	}
#endif	
}

struct resource msm_camera_resources[] = {
	{
		.start	= 0xA6000000,
		.end	= 0xA6000000 + SZ_1M - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_VFE,
		.end	= INT_VFE,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.flags	= IORESOURCE_DMA,
	}
};

struct msm_camera_device_platform_data msm_camera_device_data = {
	.camera_gpio_on  = config_camera_on_gpios,
	.camera_gpio_off = config_camera_off_gpios,
	.ioext.camifpadphy = 0xAB000000,
	.ioext.camifpadsz  = 0x00000400,
	.ioext.csiphy = 0xA6100000,
	.ioext.csisz  = 0x00000400,
	.ioext.csiirq = INT_CSI,
	.ioclk.mclk_clk_rate = 24000000,
	.ioclk.vfe_clk_rate  = 122880000,
};

static struct msm_camera_sensor_flash_src msm_flash_src_pwm = {
	.flash_sr_type = MSM_CAMERA_FLASH_SRC_PWM,
	._fsrc.pwm_src.freq  = 1000,
	._fsrc.pwm_src.max_load = 300,
	._fsrc.pwm_src.low_load = 30,
	._fsrc.pwm_src.high_load = 100,
	._fsrc.pwm_src.channel = 7,
};

static struct msm_camera_sensor_flash_src msm_flash_src_current_driver = {
	.flash_sr_type = MSM_CAMERA_FLASH_SRC_CURRENT_DRIVER,
	._fsrc.current_driver_src.low_current = 210,
	._fsrc.current_driver_src.high_current = 700,
	._fsrc.current_driver_src.driver_channel = &pm8058_fluid_leds_data,
};

static struct i2c_gpio_platform_data camera_i2c_gpio_data = {
	.scl_pin = 0,
	.sda_pin = 1,
};

static struct platform_device camera_i2c_gpio_device = {  
	.name	= "i2c-gpio",
	.id	= 4,
	.dev	= {
		.platform_data  = &camera_i2c_gpio_data,
	},
};

#if defined (CONFIG_SENSOR_MC7)
static struct msm_camera_sensor_flash_data flash_mc7 = {
//	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_type = MSM_CAMERA_FLASH_NONE,
	.flash_src  = &msm_flash_src_pwm
};

static struct msm_camera_sensor_info msm_camera_sensor_mc7_data = {
	.sensor_name    = "lsi_mc7",
	.sensor_reset   = 174,
	.sensor_pwd     = 175,
	.vcm_pwd        = 0,
	.vcm_enable	= 2,
	.pdata          = &msm_camera_device_data,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	.flash_data     = &flash_mc7,
	.csi_if         = 1
};

static struct platform_device msm_camera_sensor_mc7= {
	.name  	= "msm_camera_mc7",
	.dev   	= {
		.platform_data = &msm_camera_sensor_mc7_data,
	},
};
#endif

#ifdef CONFIG_SENSOR_S5K4ECGX
static struct msm_camera_sensor_flash_data flash_s5k4ecgx = {
	.flash_type = MSM_CAMERA_FLASH_NONE,
	.flash_src  = &msm_flash_src_pwm
};
static struct msm_camera_sensor_info msm_camera_sensor_s5k4ecgx_data = {
        .sensor_name		= "s5k4ecgx",
        .sensor_reset		= 0, 
        .sensor_pwd		= 0, 
        .vcm_pwd		= 0, 
        .pdata			= &msm_camera_device_data,
        .resource       = msm_camera_resources,
        .num_resources  = ARRAY_SIZE(msm_camera_resources),      
        .flash_data		= &flash_s5k4ecgx,
        .csi_if         = 0        
};
 
static struct platform_device msm_camera_sensor_s5k4ecgx = {
        .name      = "msm_camera_s5k4ecgx",
        .dev       = {  
                .platform_data = &msm_camera_sensor_s5k4ecgx_data,
        },   
};
#endif

#if defined (CONFIG_SENSOR_CE147)
static struct msm_camera_sensor_flash_data flash_ce147 = {
//	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_type = MSM_CAMERA_FLASH_NONE,
	.flash_src  = &msm_flash_src_pwm
};

static struct msm_camera_sensor_info msm_camera_sensor_ce147_data = {
	.sensor_name    = "ce147",
	.sensor_reset   = 174,
	.sensor_pwd     = 175,
	.vcm_pwd        = 0,
	.vcm_enable	= 2,
	.pdata          = &msm_camera_device_data,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	.flash_data     = &flash_ce147,
	.csi_if         = 0
};

static struct platform_device msm_camera_sensor_ce147= {
	.name  	= "msm_camera_ce147",
	.dev   	= {
		.platform_data = &msm_camera_sensor_ce147_data,
	},
};
#endif

#ifdef CONFIG_SENSOR_S5K5CCAF //PCAM
static struct msm_camera_sensor_flash_data flash_s5k5ccaf = {
  .flash_type = MSM_CAMERA_FLASH_NONE,
  .flash_src  = &msm_flash_src_pwm
};

/*
sensor_reset CAM_MEGA_EN 126
sensor_pwd   CAM_MEGA_RESET 174


*/
static struct msm_camera_sensor_info msm_camera_sensor_s5k5ccaf_data = {
  .sensor_name    = "s5k5ccaf",
  .sensor_reset   = 175, // was 126,
  .sensor_pwd     = 174, // was 174,
  .vcm_pwd        = 0,
  .vcm_enable     = 2,
  .pdata          = &msm_camera_device_data,
  .resource       = msm_camera_resources,
  .num_resources  = ARRAY_SIZE(msm_camera_resources),
  .flash_data     = &flash_s5k5ccaf,
  .csi_if         = 0
};

static struct platform_device msm_camera_sensor_s5k5ccaf = {
  .name      = "msm_camera_s5k5ccaf",
  .dev       = {  
    .platform_data = &msm_camera_sensor_s5k5ccaf_data,
  },   
};
#endif

#ifdef CONFIG_SENSOR_S5KA3DFX
static struct msm_camera_sensor_flash_data flash_s5ka3dfx = {
//	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_type = MSM_CAMERA_FLASH_NONE,
	.flash_src  = &msm_flash_src_pwm
};

static struct msm_camera_sensor_info msm_camera_sensor_s5ka3dfx_data = {
	.sensor_name    = "s5ka3dfx",
	.sensor_reset   = 0,
	.sensor_pwd     = 85,
	.vcm_pwd        = 1,
	.vcm_enable		= 0,
	.pdata          = &msm_camera_device_data,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	.flash_data     = &flash_s5ka3dfx,
	.csi_if         = 0
};

static struct platform_device msm_camera_sensor_s5ka3dfx = {
	.name  	= "msm_camera_s5ka3dfx",
	.dev   	= {
		.platform_data = &msm_camera_sensor_s5ka3dfx_data,
	},
};
#endif
#ifdef CONFIG_SENSOR_SR030PC30
static struct msm_camera_sensor_flash_data flash_sr030pc30 = {
	.flash_type = MSM_CAMERA_FLASH_NONE,
	.flash_src  = &msm_flash_src_pwm
};
static struct msm_camera_sensor_info msm_camera_sensor_sr030pc30_data = {
	.sensor_name    = "sr030pc30",
	.sensor_reset   = 0,
	.sensor_pwd     = 85,
	.vcm_pwd        = 1,
	.vcm_enable		= 0,
	.pdata          = &msm_camera_device_data,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	.flash_data     = &flash_sr030pc30,
	.csi_if         = 0
};
static struct platform_device msm_camera_sensor_sr030pc30 = {
	.name  	= "msm_camera_sr030pc30",
	.dev   	= {
		.platform_data = &msm_camera_sensor_sr030pc30_data,
	},
};
#endif
#ifdef CONFIG_MT9D112
static struct msm_camera_sensor_flash_data flash_mt9d112 = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src_pwm
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9d112_data = {
	.sensor_name    = "mt9d112",
	.sensor_reset   = 0,
	.sensor_pwd     = 85,
	.vcm_pwd        = 1,
	.vcm_enable     = 0,
	.pdata          = &msm_camera_device_data,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	.flash_data     = &flash_mt9d112,
	.csi_if         = 0
};

static struct platform_device msm_camera_sensor_mt9d112 = {
	.name      = "msm_camera_mt9d112",
	.dev       = {
		.platform_data = &msm_camera_sensor_mt9d112_data,
	},
};
#endif

#ifdef CONFIG_S5K3E2FX
static struct msm_camera_sensor_flash_data flash_s5k3e2fx = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src_pwm,
};

static struct msm_camera_sensor_info msm_camera_sensor_s5k3e2fx_data = {
	.sensor_name    = "s5k3e2fx",
	.sensor_reset   = 0,
	.sensor_pwd     = 85,
	.vcm_pwd        = 1,
	.vcm_enable     = 0,
	.pdata          = &msm_camera_device_data,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	.flash_data     = &flash_s5k3e2fx,
	.csi_if         = 0
};

static struct platform_device msm_camera_sensor_s5k3e2fx = {
	.name      = "msm_camera_s5k3e2fx",
	.dev       = {
		.platform_data = &msm_camera_sensor_s5k3e2fx_data,
	},
};
#endif

#ifdef CONFIG_MT9P012
static struct msm_camera_sensor_flash_data flash_mt9p012 = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src_pwm
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9p012_data = {
	.sensor_name    = "mt9p012",
	.sensor_reset   = 0,
	.sensor_pwd     = 85,
	.vcm_pwd        = 1,
	.vcm_enable     = 1,
	.pdata          = &msm_camera_device_data,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	.flash_data     = &flash_mt9p012,
	.csi_if         = 0
};

static struct platform_device msm_camera_sensor_mt9p012 = {
	.name      = "msm_camera_mt9p012",
	.dev       = {
		.platform_data = &msm_camera_sensor_mt9p012_data,
	},
};
#endif

#ifdef CONFIG_MT9E013
static struct msm_camera_sensor_flash_data flash_mt9e013 = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src_pwm
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9e013_data = {
	.sensor_name    = "mt9e013",
	.sensor_reset   = 0,
	.sensor_pwd     = 85,
	.vcm_pwd        = 1,
	.vcm_enable     = 1,
	.pdata          = &msm_camera_device_data,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	.flash_data     = &flash_mt9e013,
	.csi_if         = 1
};

static struct platform_device msm_camera_sensor_mt9e013 = {
	.name      = "msm_camera_mt9e013",
	.dev       = {
		.platform_data = &msm_camera_sensor_mt9e013_data,
	},
};
#endif


#ifdef CONFIG_VX6953
static struct msm_camera_sensor_flash_data flash_vx6953 = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src_pwm
};
static struct msm_camera_sensor_info msm_camera_sensor_vx6953_data = {
	.sensor_name    = "vx6953",
	.sensor_reset   = 0,
	.sensor_pwd     = 85,
	.vcm_pwd        = 1,
	.vcm_enable		= 0,
	.pdata          = &msm_camera_device_data,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	.flash_data     = &flash_vx6953,
	.csi_if         = 1
};
static struct platform_device msm_camera_sensor_vx6953 = {
	.name  	= "msm_camera_vx6953",
	.dev   	= {
		.platform_data = &msm_camera_sensor_vx6953_data,
	},
};
#endif

#ifdef CONFIG_SN12M0PZ
static struct msm_camera_sensor_flash_data flash_sn12m0pz = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src_current_driver
};
static struct msm_camera_sensor_info msm_camera_sensor_sn12m0pz_data = {
	.sensor_name    = "sn12m0pz",
	.sensor_reset   = 0,
	.sensor_pwd     = 85,
	.vcm_pwd        = 1,
	.vcm_enable     = 1,
	.pdata          = &msm_camera_device_data,
	.flash_data     = &flash_sn12m0pz,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	.csi_if         = 0
};

static struct platform_device msm_camera_sensor_sn12m0pz = {
	.name      = "msm_camera_sn12m0pz",
	.dev       = {
		.platform_data = &msm_camera_sensor_sn12m0pz_data,
	},
};
#endif

#ifdef CONFIG_MT9T013
static struct msm_camera_sensor_flash_data flash_mt9t013 = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src_pwm
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9t013_data = {
	.sensor_name    = "mt9t013",
	.sensor_reset   = 0,
	.sensor_pwd     = 85,
	.vcm_pwd        = 1,
	.vcm_enable     = 0,
	.pdata          = &msm_camera_device_data,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	.flash_data     = &flash_mt9t013,
	.csi_if         = 1
};

static struct platform_device msm_camera_sensor_mt9t013 = {
	.name      = "msm_camera_mt9t013",
	.dev       = {
		.platform_data = &msm_camera_sensor_mt9t013_data,
	},
};
#endif

#ifdef CONFIG_MSM_GEMINI
static struct resource msm_gemini_resources[] = {
	{
		.start  = 0xA3A00000,
		.end    = 0xA3A00000 + 0x0150 - 1,
		.flags  = IORESOURCE_MEM,
	},
	{
		.start  = INT_JPEG,
		.end    = INT_JPEG,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device msm_gemini_device = {
	.name           = "msm_gemini",
	.resource       = msm_gemini_resources,
	.num_resources  = ARRAY_SIZE(msm_gemini_resources),
};
#endif

#ifdef CONFIG_MSM_VPE
static struct resource msm_vpe_resources[] = {
	{
		.start	= 0xAD200000,
		.end	= 0xAD200000 + SZ_1M - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_VPE,
		.end	= INT_VPE,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device msm_vpe_device = {
       .name = "msm_vpe",
       .id   = 0,
       .num_resources = ARRAY_SIZE(msm_vpe_resources),
       .resource = msm_vpe_resources,
};
#endif

#endif /*CONFIG_MSM_CAMERA*/

static struct platform_device msm_vibrator_device = {
	.name 		    = "msm_vibrator",
	.id		    = -1,
};

static uint32_t vibrator_device_gpio_config[] = {
	GPIO_CFG(163, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};
static int __init vibrator_device_gpio_init(void)
{

	config_gpio_table(vibrator_device_gpio_config,
		ARRAY_SIZE(vibrator_device_gpio_config));

	return 0;
}

static struct i2c_gpio_platform_data amp_i2c_gpio_data = {
	.sda_pin    = 171,
	.scl_pin    = 170,
};

static struct platform_device amp_i2c_gpio_device = {  
	.name       = "i2c-gpio",
	.id     = 9,
	.dev        = {
		.platform_data  = &amp_i2c_gpio_data,
	},
};


#ifdef CONFIG_SENSORS_YDA165
#ifdef CONFIG_MACH_ANCORA_TMO
static struct snd_set_ampgain init_ampgain[] = {
	[0] = {
		.in1_gain = 2,
		.in2_gain = 2,
		.hp_att = 31,
		.hp_gainup = 2,
		.sp_att = 29,
		.sp_gainup = 1,
	},
	[1] = { /* [HSS] headset_call, speaker_call */
		.in1_gain = 2,
		.in2_gain = 0,
		.hp_att = 17,
		.hp_gainup = 1,
		.sp_att = 31,
		.sp_gainup = 0,
	},
	[2] = { /* [HSS] headset_speaker */
		.in1_gain = 0,
		.in2_gain = 1,
		.hp_att = 1,
		.hp_gainup = 0,
		.sp_att = 30,
		.sp_gainup = 2,
	},
};
#else
static struct snd_set_ampgain init_ampgain[] = {
	[0] = {
		.in1_gain = 2,
		.in2_gain = 2,
		.hp_att = 31,
		.hp_gainup = 0,
		.sp_att = 31,
		.sp_gainup = 0,
	},
	[1] = { /* [HSS] headset_call, speaker_call */
		.in1_gain = 2,
		.in2_gain = 0,
		.hp_att = 14,
		.hp_gainup = 0,
		.sp_att = 31,
		.sp_gainup = 0,
	},
	[2] = { /* [HSS] headset_speaker */
		.in1_gain = 5,
		.in2_gain = 0,
		.hp_att = 5,
		.hp_gainup = 0,
		.sp_att = 31,
		.sp_gainup = 2,
	},
};
#endif
static struct yda165_i2c_data yda165_data = {
	.ampgain = init_ampgain,
	.num_modes = ARRAY_SIZE(init_ampgain),
};

static struct i2c_board_info yamahaamp_boardinfo[] = {
	{
		I2C_BOARD_INFO("yda165", 0xD8 >> 1),
		.platform_data = &yda165_data,
	},
};
#endif

#ifdef CONFIG_MSM7KV2_AUDIO

static int __init snddev_poweramp_gpio_init(void)
{
	int rc;
	struct vreg *vreg_s3_1;

	pr_info("snddev_poweramp_gpio_init \n");

	vreg_s3_1 = vreg_get(NULL, "s3");		// VREG_1.8V
	if (IS_ERR(vreg_s3_1)) {
		rc = PTR_ERR(vreg_s3_1);
		printk(KERN_ERR "%s: vreg get failed (%d)\n", __func__, rc);
		return rc;
	}
	rc = vreg_enable(vreg_s3_1);
	if (rc)
		pr_err("%s: vreg_enable(s3) failed (%d)\n", __func__, rc);
	return rc;
}

void msm_snddev_tx_route_config(void)
{
	pr_debug("%s()\n", __func__);
	gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_MICBIAS_EN), 1);
	return;
}

void msm_snddev_tx_route_deconfig(void)
{
	pr_debug("%s()\n", __func__);
	gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_MICBIAS_EN), 0);
	return;
}

void msm_snddev_tx_ear_route_config(void)
{
	printk("%s()\n", __func__);

/* hk46.song_20110822 add ++ */
#ifdef CONFIG_CONDITIONAL_POWER_COLLAPSE_EAR_MIC_TX
	if(already_ear_micbias_on == 0)
	{
		prev_idle_enable_state = msm_pm_modes[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].idle_enabled;
		msm_pm_modes[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].idle_enabled = 0; 
		printk("[idle_enabled(%d)] prev(%d) -> set to 0\n", MSM_PM_SLEEP_MODE_POWER_COLLAPSE, prev_idle_enable_state);
		already_ear_micbias_on = 1;
	}
#endif
/* hk46.song_20110822 add -- */

	gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_EAR_MICBIAS_EN), 1);
#ifdef CONFIG_SAMSUNG_JACK
	tx_set_flag = 1;
#endif
	return;
}

void msm_snddev_tx_ear_route_deconfig(void)
{
	printk("%s()\n", __func__);
	if ( ! ( ( sec_jack_get_det_jack_state() ) && (!sec_jack_get_send_key_state()) ) )
/* hk46.song_20110822 ++ */
#ifdef CONFIG_CONDITIONAL_POWER_COLLAPSE_EAR_MIC_TX
	{
		msm_pm_modes[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].idle_enabled = prev_idle_enable_state;
		printk("[idle_enabled(%d)] set to %d\n", MSM_PM_SLEEP_MODE_POWER_COLLAPSE, prev_idle_enable_state);
		already_ear_micbias_on = 0;
#endif
/* hk46.song_20110822 -- */
		gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_EAR_MICBIAS_EN), 0);
/* hk46.song_20110822 ++ */
#ifdef CONFIG_CONDITIONAL_POWER_COLLAPSE_EAR_MIC_TX
	}
#endif
/* hk46.song_20110822 -- */
#ifdef CONFIG_SAMSUNG_JACK
	tx_set_flag = 0;
#endif
	return;
}

void msm_snddev_poweramp_on_speaker(void)
{
#ifdef CONFIG_SENSORS_YDA165
	yda165_speaker_onoff(1);
#endif
	pr_info("%s: power on amplifier\n", __func__);
}

void msm_snddev_poweramp_off_speaker(void)
{
#ifdef CONFIG_SENSORS_YDA165
	yda165_speaker_onoff(0);
#endif
	pr_info("%s: power off amplifier\n", __func__);
}
void msm_snddev_poweramp_on_speaker_call(void)
{
#ifdef CONFIG_SENSORS_YDA165
	yda165_speaker_call_onoff(1);
#endif
	pr_info("%s: power on amplifier\n", __func__);
}
void msm_snddev_poweramp_off_speaker_call(void)
{
#ifdef CONFIG_SENSORS_YDA165
	yda165_speaker_call_onoff(0);
#endif
	pr_info("%s: power off amplifier\n", __func__);
}
void msm_snddev_poweramp_on_headset(void)
{
#ifdef CONFIG_SENSORS_YDA165
	yda165_headset_onoff(1);
#endif
	pr_info("%s: power on amplifier\n", __func__);
}
void msm_snddev_poweramp_off_headset(void)
{
#ifdef CONFIG_SENSORS_YDA165
	yda165_headset_onoff(0);
#endif
	pr_info("%s: power off amplifier\n", __func__);
}
void msm_snddev_poweramp_on_headset_call(void)
{
#ifdef CONFIG_SENSORS_YDA165
	yda165_headset_call_onoff(1);
#endif
	pr_info("%s: power on amplifier\n", __func__);
}
void msm_snddev_poweramp_off_headset_call(void)
{
#ifdef CONFIG_SENSORS_YDA165
	yda165_headset_call_onoff(0);
#endif
	pr_info("%s: power off amplifier\n", __func__);
}
void msm_snddev_poweramp_on_dock(void)
{
#ifdef CONFIG_SENSORS_YDA165
		yda165_headset_onoff(1);
#endif
	pr_info("%s: power on amp\n", __func__);
}
void msm_snddev_poweramp_off_dock(void)
{
#ifdef CONFIG_SENSORS_YDA165
		yda165_headset_onoff(0);
#endif
	pr_info("%s: power off amp\n", __func__);
}
void msm_snddev_poweramp_on_together(void)
{
#ifdef CONFIG_SENSORS_YDA165
	yda165_speaker_headset_onoff(1);
#endif
	pr_info("%s: power on amplifier\n", __func__);
}
void msm_snddev_poweramp_off_together(void)
{
#ifdef CONFIG_SENSORS_YDA165
	yda165_speaker_headset_onoff(0);
#endif
	pr_info("%s: power off amplifier\n", __func__);
}
void msm_snddev_poweramp_on_tty(void)
{
#ifdef CONFIG_SENSORS_YDA165
	yda165_tty_onoff(1);
#endif
	pr_info("%s: power on amplifier\n", __func__);
}
void msm_snddev_poweramp_off_tty(void)
{
#ifdef CONFIG_SENSORS_YDA165
	yda165_tty_onoff(0);
#endif	
	pr_info("%s: power off amplifier\n", __func__);
}

static struct vreg *snddev_vreg_ncp, *snddev_vreg_gp4;

void msm_snddev_hsed_voltage_on(void)
{
	int rc;
#if 0//Block using LDO10, because it is used for vibrator
	snddev_vreg_gp4 = vreg_get(NULL, "gp4");
	if (IS_ERR(snddev_vreg_gp4)) {
		pr_err("%s: vreg_get(%s) failed (%ld)\n",
		__func__, "gp4", PTR_ERR(snddev_vreg_gp4));
		return;
	}
	rc = vreg_enable(snddev_vreg_gp4);
	if (rc)
		pr_err("%s: vreg_enable(gp4) failed (%d)\n", __func__, rc);

	snddev_vreg_ncp = vreg_get(NULL, "ncp");
	if (IS_ERR(snddev_vreg_ncp)) {
		pr_err("%s: vreg_get(%s) failed (%ld)\n",
		__func__, "ncp", PTR_ERR(snddev_vreg_ncp));
		return;
	}
	rc = vreg_enable(snddev_vreg_ncp);
	if (rc)
		pr_err("%s: vreg_enable(ncp) failed (%d)\n", __func__, rc);
#endif
}

void msm_snddev_hsed_voltage_off(void)
{
	int rc;
#if 0//Block using LDO10, because it is used for vibrator
	if (IS_ERR(snddev_vreg_ncp)) {
		pr_err("%s: vreg_get(%s) failed (%ld)\n",
		__func__, "ncp", PTR_ERR(snddev_vreg_ncp));
		return;
	}
	rc = vreg_disable(snddev_vreg_ncp);
	if (rc)
		pr_err("%s: vreg_disable(ncp) failed (%d)\n", __func__, rc);
	vreg_put(snddev_vreg_ncp);

	if (IS_ERR(snddev_vreg_gp4)) {
		pr_err("%s: vreg_get(%s) failed (%ld)\n",
		__func__, "gp4", PTR_ERR(snddev_vreg_gp4));
		return;
	}
	rc = vreg_disable(snddev_vreg_gp4);
	if (rc)
		pr_err("%s: vreg_disable(gp4) failed (%d)\n", __func__, rc);

	vreg_put(snddev_vreg_gp4);
#endif
}

static unsigned aux_pcm_gpio_on[] = {
	GPIO_CFG(138, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),   /* PCM_DOUT */
	GPIO_CFG(139, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),   /* PCM_DIN  */
	GPIO_CFG(140, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),   /* PCM_SYNC */
	GPIO_CFG(141, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),   /* PCM_CLK  */
};

static int __init aux_pcm_gpio_init(void)
{
	int pin, rc;

	pr_info("aux_pcm_gpio_init \n");
	for (pin = 0; pin < ARRAY_SIZE(aux_pcm_gpio_on); pin++) {
		rc = gpio_tlmm_config(aux_pcm_gpio_on[pin],
					GPIO_CFG_ENABLE);
		if (rc) {
			printk(KERN_ERR
				"%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, aux_pcm_gpio_on[pin], rc);
		}
	}
	return rc;
}

static struct msm_gpio mi2s_clk_gpios[] = {
};

static struct msm_gpio mi2s_rx_data_lines_gpios[] = {
};

static struct msm_gpio mi2s_tx_data_lines_gpios[] = {
};

int mi2s_config_clk_gpio(void)
{
	int rc = 0;

	rc = msm_gpios_request_enable(mi2s_clk_gpios,
			ARRAY_SIZE(mi2s_clk_gpios));
	if (rc) {
		pr_err("%s: enable mi2s clk gpios  failed\n",
					__func__);
		return rc;
	}
	return 0;
}

int  mi2s_unconfig_data_gpio(u32 direction, u8 sd_line_mask)
{
	int i, rc = 0;
	sd_line_mask &= MI2S_SD_LINE_MASK;

	switch (direction) {
	case DIR_TX:
		msm_gpios_disable_free(mi2s_tx_data_lines_gpios, 1);
		break;
	case DIR_RX:
		i = 0;
		while (sd_line_mask) {
			if (sd_line_mask & 0x1)
				msm_gpios_disable_free(
					mi2s_rx_data_lines_gpios + i , 1);
			sd_line_mask = sd_line_mask >> 1;
			i++;
		}
		break;
	default:
		pr_err("%s: Invaild direction  direction = %u\n",
						__func__, direction);
		rc = -EINVAL;
		break;
	}
	return rc;
}

int mi2s_config_data_gpio(u32 direction, u8 sd_line_mask)
{
	int i , rc = 0;
	u8 sd_config_done_mask = 0;

	sd_line_mask &= MI2S_SD_LINE_MASK;

	switch (direction) {
	case DIR_TX:
		if ((sd_line_mask & MI2S_SD_0) || (sd_line_mask & MI2S_SD_1) ||
		   (sd_line_mask & MI2S_SD_2) || !(sd_line_mask & MI2S_SD_3)) {
			pr_err("%s: can not use SD0 or SD1 or SD2 for TX"
				".only can use SD3. sd_line_mask = 0x%x\n",
				__func__ , sd_line_mask);
			rc = -EINVAL;
		} else {
			rc = msm_gpios_request_enable(mi2s_tx_data_lines_gpios,
							 1);
			if (rc)
				pr_err("%s: enable mi2s gpios for TX failed\n",
					   __func__);
		}
		break;
	case DIR_RX:
		i = 0;
		while (sd_line_mask && (rc == 0)) {
			if (sd_line_mask & 0x1) {
				rc = msm_gpios_request_enable(
					mi2s_rx_data_lines_gpios + i , 1);
				if (rc) {
					pr_err("%s: enable mi2s gpios for"
					 "RX failed.  SD line = %s\n",
					 __func__,
					 (mi2s_rx_data_lines_gpios + i)->label);
					mi2s_unconfig_data_gpio(DIR_RX,
						sd_config_done_mask);
				} else
					sd_config_done_mask |= (1 << i);
			}
			sd_line_mask = sd_line_mask >> 1;
			i++;
		}
		break;
	default:
		pr_err("%s: Invaild direction  direction = %u\n",
			__func__, direction);
		rc = -EINVAL;
		break;
	}
	return rc;
}

int mi2s_unconfig_clk_gpio(void)
{
	msm_gpios_disable_free(mi2s_clk_gpios, ARRAY_SIZE(mi2s_clk_gpios));
	return 0;
}

#endif /* CONFIG_MSM7KV2_AUDIO */

static int __init buses_init(void)
{
	if (gpio_tlmm_config(GPIO_CFG(PMIC_GPIO_INT, 1, GPIO_CFG_INPUT,
				  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE))
		pr_err("%s: gpio_tlmm_config (gpio=%d) failed\n",
		       __func__, PMIC_GPIO_INT);

	pm8058_7x30_data.sub_devices[PM8058_SUBDEV_KPD].platform_data
		= &ancora_keypad_data;
	pm8058_7x30_data.sub_devices[PM8058_SUBDEV_KPD].data_size
		= sizeof(ancora_keypad_data);

	i2c_register_board_info(6 /* I2C_SSBI ID */, pm8058_boardinfo,
				ARRAY_SIZE(pm8058_boardinfo));

	return 0;
}

#define TIMPANI_RESET_GPIO	1

static struct vreg *vreg_marimba_1;
static struct vreg *vreg_marimba_2;
static struct vreg *vreg_marimba_3;

static struct msm_gpio timpani_reset_gpio_cfg[] = {
{ GPIO_CFG(TIMPANI_RESET_GPIO, 0, GPIO_CFG_OUTPUT,
	GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "timpani_reset"} };

static int config_timpani_reset(void)
{
	int rc;

	rc = msm_gpios_request_enable(timpani_reset_gpio_cfg,
				ARRAY_SIZE(timpani_reset_gpio_cfg));
	if (rc < 0) {
		printk(KERN_ERR
			"%s: msm_gpios_request_enable failed (%d)\n",
				__func__, rc);
	}
	return rc;
}

static unsigned int msm_timpani_setup_power(void)
{
	int rc;

	rc = config_timpani_reset();
	if (rc < 0)
		goto out;

	rc = vreg_enable(vreg_marimba_1);
	if (rc) {
		printk(KERN_ERR "%s: vreg_enable() = %d\n",
					__func__, rc);
		goto out;
	}
	rc = vreg_enable(vreg_marimba_2);
	if (rc) {
		printk(KERN_ERR "%s: vreg_enable() = %d\n",
					__func__, rc);
		goto fail_disable_vreg_marimba_1;
	}

	rc = gpio_direction_output(TIMPANI_RESET_GPIO, 1);
	if (rc < 0) {
		printk(KERN_ERR
			"%s: gpio_direction_output failed (%d)\n",
				__func__, rc);
		msm_gpios_free(timpani_reset_gpio_cfg,
				ARRAY_SIZE(timpani_reset_gpio_cfg));
		vreg_disable(vreg_marimba_2);
	} else
		goto out;


fail_disable_vreg_marimba_1:
	vreg_disable(vreg_marimba_1);

out:
	return rc;
};

static void msm_timpani_shutdown_power(void)
{
	int rc;

	rc = vreg_disable(vreg_marimba_1);
	if (rc) {
		printk(KERN_ERR "%s: return val: %d\n",
					__func__, rc);
	}
	rc = vreg_disable(vreg_marimba_2);
	if (rc) {
		printk(KERN_ERR "%s: return val: %d\n",
					__func__, rc);
	}

	rc = gpio_direction_output(TIMPANI_RESET_GPIO, 0);
	if (rc < 0) {
		printk(KERN_ERR
			"%s: gpio_direction_output failed (%d)\n",
				__func__, rc);
	}

	msm_gpios_free(timpani_reset_gpio_cfg,
				   ARRAY_SIZE(timpani_reset_gpio_cfg));
};

static unsigned int msm_bahama_setup_power(void)
{
	int rc;

	rc = vreg_enable(vreg_marimba_3);
	if (rc) {
		printk(KERN_ERR "%s: vreg_enable() = %d\n",
				__func__, rc);
	}

	return rc;
};

static unsigned int msm_bahama_shutdown_power(int value)
{
	int rc = 0;

	if (value != BAHAMA_ID) {
		rc = vreg_disable(vreg_marimba_3);
		if (rc) {
			printk(KERN_ERR "%s: return val: %d\n",
					__func__, rc);
		}
	}

	return rc;
};

static struct msm_gpio marimba_svlte_config_clock[] = {
	{ GPIO_CFG(34, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		"MARIMBA_SVLTE_CLOCK_ENABLE" },
};

static unsigned int msm_marimba_gpio_config_svlte(int gpio_cfg_marimba)
{
	if (machine_is_msm8x55_svlte_surf() ||
		machine_is_msm8x55_svlte_ffa()) {
		if (gpio_cfg_marimba)
			gpio_set_value(GPIO_PIN
				(marimba_svlte_config_clock->gpio_cfg), 1);
		else
			gpio_set_value(GPIO_PIN
				(marimba_svlte_config_clock->gpio_cfg), 0);
	}

	return 0;
};

static unsigned int msm_marimba_setup_power(void)
{
	int rc;

	rc = vreg_enable(vreg_marimba_1);
	if (rc) {
		printk(KERN_ERR "%s: vreg_enable() = %d \n",
					__func__, rc);
		goto out;
	}
	rc = vreg_enable(vreg_marimba_2);
	if (rc) {
		printk(KERN_ERR "%s: vreg_enable() = %d \n",
					__func__, rc);
		goto out;
	}

	if (machine_is_msm8x55_svlte_surf() || machine_is_msm8x55_svlte_ffa()) {
		rc = msm_gpios_request_enable(marimba_svlte_config_clock,
				ARRAY_SIZE(marimba_svlte_config_clock));
		if (rc < 0) {
			printk(KERN_ERR
				"%s: msm_gpios_request_enable failed (%d)\n",
					__func__, rc);
			return rc;
		}

		rc = gpio_direction_output(GPIO_PIN
			(marimba_svlte_config_clock->gpio_cfg), 0);
		if (rc < 0) {
			printk(KERN_ERR
				"%s: gpio_direction_output failed (%d)\n",
					__func__, rc);
			return rc;
		}
	}

out:
	return rc;
};

static void msm_marimba_shutdown_power(void)
{
	int rc;

	rc = vreg_disable(vreg_marimba_1);
	if (rc) {
		printk(KERN_ERR "%s: return val: %d\n",
					__func__, rc);
	}
	rc = vreg_disable(vreg_marimba_2);
	if (rc) {
		printk(KERN_ERR "%s: return val: %d \n",
					__func__, rc);
	}
};

static int bahama_present(void)
{
	int id;
	switch (id = adie_get_detected_connectivity_type()) {
	case BAHAMA_ID:
		return 1;

	case MARIMBA_ID:
		return 0;

	case TIMPANI_ID:
	default:
	printk(KERN_ERR "%s: unexpected adie connectivity type: %d\n",
			__func__, id);
	return -ENODEV;
	}
}

struct vreg *fm_regulator;
static int fm_radio_setup(struct marimba_fm_platform_data *pdata)
{
	int rc;
	uint32_t irqcfg;
	const char *id = "FMPW";

	int bahama_not_marimba = bahama_present();

	if (bahama_not_marimba == -1) {
		printk(KERN_WARNING "%s: bahama_present: %d\n",
				__func__, bahama_not_marimba);
		return -ENODEV;
	}
	if (bahama_not_marimba)
		fm_regulator = vreg_get(NULL, "s3");
	else
		fm_regulator = vreg_get(NULL, "s2");

	if (IS_ERR(fm_regulator)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
			__func__, PTR_ERR(fm_regulator));
		return -1;
	}
	if (!bahama_not_marimba) {

		rc = pmapp_vreg_level_vote(id, PMAPP_VREG_S2, 1300);

		if (rc < 0) {
			printk(KERN_ERR "%s: voltage level vote failed (%d)\n",
				__func__, rc);
			return rc;
		}
	}
	rc = vreg_enable(fm_regulator);
	if (rc) {
		printk(KERN_ERR "%s: vreg_enable() = %d\n",
					__func__, rc);
		return rc;
	}

	rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_DO,
					  PMAPP_CLOCK_VOTE_ON);
	if (rc < 0) {
		printk(KERN_ERR "%s: clock vote failed (%d)\n",
			__func__, rc);
		goto fm_clock_vote_fail;
	}
	irqcfg = GPIO_CFG(147, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL,
					GPIO_CFG_2MA);
	rc = gpio_tlmm_config(irqcfg, GPIO_CFG_ENABLE);
	if (rc) {
		printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, irqcfg, rc);
		rc = -EIO;
		goto fm_gpio_config_fail;

	}
	return 0;
fm_gpio_config_fail:
	pmapp_clock_vote(id, PMAPP_CLOCK_ID_DO,
				  PMAPP_CLOCK_VOTE_OFF);
fm_clock_vote_fail:
	vreg_disable(fm_regulator);
	return rc;

};

static void fm_radio_shutdown(struct marimba_fm_platform_data *pdata)
{
	int rc;
	const char *id = "FMPW";
	uint32_t irqcfg = GPIO_CFG(147, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP,
					GPIO_CFG_2MA);

	int bahama_not_marimba = bahama_present();
	if (bahama_not_marimba == -1) {
		printk(KERN_WARNING "%s: bahama_present: %d\n",
			__func__, bahama_not_marimba);
		return;
	}

	rc = gpio_tlmm_config(irqcfg, GPIO_CFG_ENABLE);
	if (rc) {
		printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, irqcfg, rc);
	}
	if (fm_regulator != NULL) {
		rc = vreg_disable(fm_regulator);

		if (rc) {
			printk(KERN_ERR "%s: return val: %d\n",
					__func__, rc);
		}
		fm_regulator = NULL;
	}
	rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_DO,
					  PMAPP_CLOCK_VOTE_OFF);
	if (rc < 0)
		printk(KERN_ERR "%s: clock_vote return val: %d\n",
						__func__, rc);
	if (!bahama_not_marimba)	{
		rc = pmapp_vreg_level_vote(id, PMAPP_VREG_S2, 0);

		if (rc < 0)
			printk(KERN_ERR "%s: vreg level vote return val: %d\n",
						__func__, rc);
	}
}

static struct marimba_fm_platform_data marimba_fm_pdata = {
	.fm_setup =  fm_radio_setup,
	.fm_shutdown = fm_radio_shutdown,
	.irq = MSM_GPIO_TO_INT(147),
	.vreg_s2 = NULL,
	.vreg_xo_out = NULL,
};


/* Slave id address for FM/CDC/QMEMBIST
 * Values can be programmed using Marimba slave id 0
 * should there be a conflict with other I2C devices
 * */
#define MARIMBA_SLAVE_ID_FM_ADDR	0x2A
#define MARIMBA_SLAVE_ID_CDC_ADDR	0x77
#define MARIMBA_SLAVE_ID_QMEMBIST_ADDR	0X66

#define BAHAMA_SLAVE_ID_FM_ADDR         0x2A
#define BAHAMA_SLAVE_ID_QMEMBIST_ADDR   0x7B

static const char *tsadc_id = "MADC";
static const char *vregs_tsadc_name[] = {
	"gp12",
	"s2",
};
static struct vreg *vregs_tsadc[ARRAY_SIZE(vregs_tsadc_name)];

static const char *vregs_timpani_tsadc_name[] = {
	"s3",
	"gp12",
	"gp16"
};
static struct vreg *vregs_timpani_tsadc[ARRAY_SIZE(vregs_timpani_tsadc_name)];


static int marimba_tsadc_power(int vreg_on)
{
	int i, rc = 0;
	int tsadc_adie_type = adie_get_detected_codec_type();

	if (tsadc_adie_type == TIMPANI_ID) {
		for (i = 0; i < ARRAY_SIZE(vregs_timpani_tsadc_name); i++) {
			if (!vregs_timpani_tsadc[i]) {
				pr_err("%s: vreg_get %s failed(%d)\n",
				__func__, vregs_timpani_tsadc_name[i], rc);
				goto vreg_fail;
			}

			rc = vreg_on ? vreg_enable(vregs_timpani_tsadc[i]) :
				  vreg_disable(vregs_timpani_tsadc[i]);
			if (rc < 0) {
				pr_err("%s: vreg %s %s failed(%d)\n",
					__func__, vregs_timpani_tsadc_name[i],
				       vreg_on ? "enable" : "disable", rc);
				goto vreg_fail;
			}
		}
		/* Vote for D0 and D1 buffer */
		rc = pmapp_clock_vote(tsadc_id, PMAPP_CLOCK_ID_D1,
			vreg_on ? PMAPP_CLOCK_VOTE_ON : PMAPP_CLOCK_VOTE_OFF);
		if (rc)	{
			pr_err("%s: unable to %svote for d1 clk\n",
				__func__, vreg_on ? "" : "de-");
			goto do_vote_fail;
		}
		rc = pmapp_clock_vote(tsadc_id, PMAPP_CLOCK_ID_DO,
			vreg_on ? PMAPP_CLOCK_VOTE_ON : PMAPP_CLOCK_VOTE_OFF);
		if (rc)	{
			pr_err("%s: unable to %svote for d1 clk\n",
				__func__, vreg_on ? "" : "de-");
			goto do_vote_fail;
		}
	} else if (tsadc_adie_type == MARIMBA_ID) {
		for (i = 0; i < ARRAY_SIZE(vregs_tsadc_name); i++) {
			if (!vregs_tsadc[i]) {
				pr_err("%s: vreg_get %s failed (%d)\n",
					__func__, vregs_tsadc_name[i], rc);
				goto vreg_fail;
			}

			rc = vreg_on ? vreg_enable(vregs_tsadc[i]) :
				  vreg_disable(vregs_tsadc[i]);
			if (rc < 0) {
				pr_err("%s: vreg %s %s failed (%d)\n",
					__func__, vregs_tsadc_name[i],
				       vreg_on ? "enable" : "disable", rc);
				goto vreg_fail;
			}
		}
		/* If marimba vote for DO buffer */
		rc = pmapp_clock_vote(tsadc_id, PMAPP_CLOCK_ID_DO,
			vreg_on ? PMAPP_CLOCK_VOTE_ON : PMAPP_CLOCK_VOTE_OFF);
		if (rc)	{
			pr_err("%s: unable to %svote for d0 clk\n",
				__func__, vreg_on ? "" : "de-");
			goto do_vote_fail;
		}
	} else {
		pr_err("%s:Adie %d not supported\n",
				__func__, tsadc_adie_type);
		return -ENODEV;
	}

	msleep(5); /* ensure power is stable */

	return 0;

do_vote_fail:
vreg_fail:
	while (i) {
		if (vreg_on) {
			if (tsadc_adie_type == TIMPANI_ID)
				vreg_disable(vregs_timpani_tsadc[--i]);
			else if (tsadc_adie_type == MARIMBA_ID)
				vreg_disable(vregs_tsadc[--i]);
		} else {
			if (tsadc_adie_type == TIMPANI_ID)
				vreg_enable(vregs_timpani_tsadc[--i]);
			else if (tsadc_adie_type == MARIMBA_ID)
				vreg_enable(vregs_tsadc[--i]);
		}
	}

	return rc;
}

static int marimba_tsadc_vote(int vote_on)
{
	int rc = 0;

	if (adie_get_detected_codec_type() == MARIMBA_ID) {
		int level = vote_on ? 1300 : 0;
		rc = pmapp_vreg_level_vote(tsadc_id, PMAPP_VREG_S2, level);
		if (rc < 0)
			pr_err("%s: vreg level %s failed (%d)\n",
			__func__, vote_on ? "on" : "off", rc);
	}

	return rc;
}

static int marimba_tsadc_init(void)
{
	int i, rc = 0;
	int tsadc_adie_type = adie_get_detected_codec_type();

	if (tsadc_adie_type == TIMPANI_ID) {
		for (i = 0; i < ARRAY_SIZE(vregs_timpani_tsadc_name); i++) {
			vregs_timpani_tsadc[i] = vreg_get(NULL,
						vregs_timpani_tsadc_name[i]);
			if (IS_ERR(vregs_timpani_tsadc[i])) {
				pr_err("%s: vreg get %s failed (%ld)\n",
				       __func__, vregs_timpani_tsadc_name[i],
				       PTR_ERR(vregs_timpani_tsadc[i]));
				rc = PTR_ERR(vregs_timpani_tsadc[i]);
				goto vreg_get_fail;
			}
		}
	} else if (tsadc_adie_type == MARIMBA_ID) {
		for (i = 0; i < ARRAY_SIZE(vregs_tsadc_name); i++) {
			vregs_tsadc[i] = vreg_get(NULL, vregs_tsadc_name[i]);
			if (IS_ERR(vregs_tsadc[i])) {
				pr_err("%s: vreg get %s failed (%ld)\n",
				       __func__, vregs_tsadc_name[i],
				       PTR_ERR(vregs_tsadc[i]));
				rc = PTR_ERR(vregs_tsadc[i]);
				goto vreg_get_fail;
			}
		}
	} else {
		pr_err("%s:Adie %d not supported\n",
				__func__, tsadc_adie_type);
		return -ENODEV;
	}

	return 0;

vreg_get_fail:
	while (i) {
		if (tsadc_adie_type == TIMPANI_ID)
			vreg_put(vregs_timpani_tsadc[--i]);
		else if (tsadc_adie_type == MARIMBA_ID)
			vreg_put(vregs_tsadc[--i]);
	}
	return rc;
}

static int marimba_tsadc_exit(void)
{
	int i, rc = 0;
	int tsadc_adie_type = adie_get_detected_codec_type();

	if (tsadc_adie_type == TIMPANI_ID) {
		for (i = 0; i < ARRAY_SIZE(vregs_timpani_tsadc_name); i++) {
			if (vregs_tsadc[i])
				vreg_put(vregs_timpani_tsadc[i]);
		}
	} else if (tsadc_adie_type == MARIMBA_ID) {
		for (i = 0; i < ARRAY_SIZE(vregs_tsadc_name); i++) {
			if (vregs_tsadc[i])
				vreg_put(vregs_tsadc[i]);
		}
		rc = pmapp_vreg_level_vote(tsadc_id, PMAPP_VREG_S2, 0);
		if (rc < 0)
			pr_err("%s: vreg level off failed (%d)\n",
						__func__, rc);
	} else {
		pr_err("%s:Adie %d not supported\n",
				__func__, tsadc_adie_type);
		rc = -ENODEV;
	}

	return rc;
}




static struct msm_ts_platform_data msm_ts_data = {
	.min_x          = 0,
	.max_x          = 4096,
	.min_y          = 0,
	.max_y          = 4096,
	.min_press      = 0,
	.max_press      = 255,
	.inv_x          = 4096,
	.inv_y          = 4096,
	.can_wakeup	= false,
};

static struct marimba_tsadc_platform_data marimba_tsadc_pdata = {
	.marimba_tsadc_power =  marimba_tsadc_power,
	.init		     =  marimba_tsadc_init,
	.exit		     =  marimba_tsadc_exit,
	.level_vote	     =  marimba_tsadc_vote,
	.tsadc_prechg_en = true,
	.can_wakeup	= false,
	.setup = {
		.pen_irq_en	=	true,
		.tsadc_en	=	true,
	},
	.params2 = {
		.input_clk_khz		=	2400,
		.sample_prd		=	TSADC_CLK_3,
	},
	.params3 = {
		.prechg_time_nsecs	=	6400,
		.stable_time_nsecs	=	6400,
		.tsadc_test_mode	=	0,
	},
	.tssc_data = &msm_ts_data,
};

static struct vreg *vreg_codec_s4;
static int msm_marimba_codec_power(int vreg_on)
{
	int rc = 0;

	if (!vreg_codec_s4) {

		vreg_codec_s4 = vreg_get(NULL, "s4");

		if (IS_ERR(vreg_codec_s4)) {
			printk(KERN_ERR "%s: vreg_get() failed (%ld)\n",
				__func__, PTR_ERR(vreg_codec_s4));
			rc = PTR_ERR(vreg_codec_s4);
			goto  vreg_codec_s4_fail;
		}
	}

	if (vreg_on) {
		rc = vreg_enable(vreg_codec_s4);
		if (rc)
			printk(KERN_ERR "%s: vreg_enable() = %d \n",
					__func__, rc);
		goto vreg_codec_s4_fail;
	} else {
		rc = vreg_disable(vreg_codec_s4);
		if (rc)
			printk(KERN_ERR "%s: vreg_disable() = %d \n",
					__func__, rc);
		goto vreg_codec_s4_fail;
	}

vreg_codec_s4_fail:
	return rc;
}

static struct marimba_codec_platform_data mariba_codec_pdata = {
	.marimba_codec_power =  msm_marimba_codec_power,
#ifdef CONFIG_MARIMBA_CODEC
	.snddev_profile_init = msm_snddev_init,
#endif
};

static struct marimba_platform_data marimba_pdata = {
	.slave_id[MARIMBA_SLAVE_ID_FM]       = MARIMBA_SLAVE_ID_FM_ADDR,
	.slave_id[MARIMBA_SLAVE_ID_CDC]	     = MARIMBA_SLAVE_ID_CDC_ADDR,
	.slave_id[MARIMBA_SLAVE_ID_QMEMBIST] = MARIMBA_SLAVE_ID_QMEMBIST_ADDR,
	.slave_id[SLAVE_ID_BAHAMA_FM]        = BAHAMA_SLAVE_ID_FM_ADDR,
	.slave_id[SLAVE_ID_BAHAMA_QMEMBIST]  = BAHAMA_SLAVE_ID_QMEMBIST_ADDR,
	.marimba_setup = msm_marimba_setup_power,
	.marimba_shutdown = msm_marimba_shutdown_power,
	.bahama_setup = msm_bahama_setup_power,
	.bahama_shutdown = msm_bahama_shutdown_power,
	.marimba_gpio_config = msm_marimba_gpio_config_svlte,
	.fm = &marimba_fm_pdata,
	.codec = &mariba_codec_pdata,
};

static void __init msm7x30_init_marimba(void)
{
	int rc;

	vreg_marimba_1 = vreg_get(NULL, "s3");
	if (IS_ERR(vreg_marimba_1)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
			__func__, PTR_ERR(vreg_marimba_1));
		return;
	}
	rc = vreg_set_level(vreg_marimba_1, 1800);

	vreg_marimba_2 = vreg_get(NULL, "gp16");
	if (IS_ERR(vreg_marimba_1)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
			__func__, PTR_ERR(vreg_marimba_1));
		return;
	}
	rc = vreg_set_level(vreg_marimba_2, 1200);

	vreg_marimba_3 = vreg_get(NULL, "usb2");
	if (IS_ERR(vreg_marimba_3)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
			__func__, PTR_ERR(vreg_marimba_3));
		return;
	}
	rc = vreg_set_level(vreg_marimba_3, 1800);
}

static struct marimba_codec_platform_data timpani_codec_pdata = {
	.marimba_codec_power =  msm_marimba_codec_power,
#ifdef CONFIG_TIMPANI_CODEC
	.snddev_profile_init = msm_snddev_init_timpani,
#endif
};

static struct marimba_platform_data timpani_pdata = {
	.slave_id[MARIMBA_SLAVE_ID_CDC]	= MARIMBA_SLAVE_ID_CDC_ADDR,
	.slave_id[MARIMBA_SLAVE_ID_QMEMBIST] = MARIMBA_SLAVE_ID_QMEMBIST_ADDR,
	.marimba_setup = msm_timpani_setup_power,
	.marimba_shutdown = msm_timpani_shutdown_power,
	.codec = &timpani_codec_pdata,
	.tsadc = &marimba_tsadc_pdata,
};

#define TIMPANI_I2C_SLAVE_ADDR	0xD

static struct i2c_board_info msm_i2c_gsbi7_timpani_info[] = {
	{
		I2C_BOARD_INFO("timpani", TIMPANI_I2C_SLAVE_ADDR),
		.platform_data = &timpani_pdata,
	},
};

#ifdef CONFIG_MSM7KV2_AUDIO
static struct resource msm_aictl_resources[] = {
	{
		.name = "aictl",
		.start = 0xa5000100,
		.end = 0xa5000100,
		.flags = IORESOURCE_MEM,
	}
};

static struct resource msm_mi2s_resources[] = {
	{
		.name = "hdmi",
		.start = 0xac900000,
		.end = 0xac900038,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "codec_rx",
		.start = 0xac940040,
		.end = 0xac940078,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "codec_tx",
		.start = 0xac980080,
		.end = 0xac9800B8,
		.flags = IORESOURCE_MEM,
	}

};

static struct msm_lpa_platform_data lpa_pdata = {
	.obuf_hlb_size = 0x2BFF8,
	.dsp_proc_id = 0,
	.app_proc_id = 2,
	.nosb_config = {
		.llb_min_addr = 0,
		.llb_max_addr = 0x3ff8,
		.sb_min_addr = 0,
		.sb_max_addr = 0,
	},
	.sb_config = {
		.llb_min_addr = 0,
		.llb_max_addr = 0x37f8,
		.sb_min_addr = 0x3800,
		.sb_max_addr = 0x3ff8,
	}
};

static struct resource msm_lpa_resources[] = {
	{
		.name = "lpa",
		.start = 0xa5000000,
		.end = 0xa50000a0,
		.flags = IORESOURCE_MEM,
	}
};

static struct resource msm_aux_pcm_resources[] = {

	{
		.name = "aux_codec_reg_addr",
		.start = 0xac9c00c0,
		.end = 0xac9c00c8,
		.flags = IORESOURCE_MEM,
	},
	{
		.name   = "aux_pcm_dout",
		.start  = 138,
		.end    = 138,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_din",
		.start  = 139,
		.end    = 139,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_syncout",
		.start  = 140,
		.end    = 140,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_clkin_a",
		.start  = 141,
		.end    = 141,
		.flags  = IORESOURCE_IO,
	},
};

static struct platform_device msm_aux_pcm_device = {
	.name   = "msm_aux_pcm",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_aux_pcm_resources),
	.resource       = msm_aux_pcm_resources,
};

struct platform_device msm_aictl_device = {
	.name = "audio_interct",
	.id   = 0,
	.num_resources = ARRAY_SIZE(msm_aictl_resources),
	.resource = msm_aictl_resources,
};

struct platform_device msm_mi2s_device = {
	.name = "mi2s",
	.id   = 0,
	.num_resources = ARRAY_SIZE(msm_mi2s_resources),
	.resource = msm_mi2s_resources,
};

struct platform_device msm_lpa_device = {
	.name = "lpa",
	.id   = 0,
	.num_resources = ARRAY_SIZE(msm_lpa_resources),
	.resource = msm_lpa_resources,
	.dev		= {
		.platform_data = &lpa_pdata,
	},
};
#endif /* CONFIG_MSM7KV2_AUDIO */

#if 1
static unsigned fsa9480_gpio_on[] = {
	GPIO_CFG(55, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
	GPIO_CFG(89, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
};

static int __init fsa9480_gpio_init(void)
{
	int pin, rc;

	pr_info("fsa9480_gpio_init \n");
	for (pin = 0; pin < ARRAY_SIZE(fsa9480_gpio_on); pin++) {
		rc = gpio_tlmm_config(fsa9480_gpio_on[pin],
					GPIO_CFG_ENABLE);
		if (rc) {
			printk(KERN_ERR
				"%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, fsa9480_gpio_on[pin], rc);
		}
	}
	return rc;
}
#endif

/* 2011-01-27 hyeokseon.yu */
#ifdef CONFIG_MAX17043_FUEL_GAUGE
static unsigned fg17043_gpio_on[] = {
	GPIO_CFG(168, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
	GPIO_CFG(169, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
};

static int __init fg17043_gpio_init(void)
{
	int pin, rc;

//	printk("[hyeokseon]fg17043_gpio_init \n");

		for (pin = 0; pin < ARRAY_SIZE(fg17043_gpio_on); pin++) {
			rc = gpio_tlmm_config(fg17043_gpio_on[pin],
						GPIO_CFG_ENABLE);
			if (rc) {
				printk(KERN_ERR
					"%s: gpio_tlmm_config(%#x)=%d\n",
					__func__, fg17043_gpio_on[pin], rc);
			}
		}
	return rc;
}
#endif

#if defined(CONFIG_CHARGER_SMB328A)

static unsigned fg_smb_gpio_on[] = {
	GPIO_CFG(124, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
	GPIO_CFG(125, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
};

static int __init fg_smb_gpio_init(void)
{
	int pin, rc;

//	printk("[hyeokseon]fg_smb_gpio_init \n");

		for (pin = 0; pin < ARRAY_SIZE(fg_smb_gpio_on); pin++) {
			rc = gpio_tlmm_config(fg_smb_gpio_on[pin],
						GPIO_CFG_ENABLE);
			if (rc) {
				printk(KERN_ERR
					"%s: gpio_tlmm_config(%#x)=%d\n",
					__func__, fg_smb_gpio_on[pin], rc);
			}
		}
	return rc;
}

#endif

#define DEC0_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC1_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
 #define DEC2_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
 #define DEC3_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC4_FORMAT (1<<MSM_ADSP_CODEC_MIDI)

static unsigned int dec_concurrency_table[] = {
	/* Audio LP */
	0,
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_MODE_LP)|
	(1<<MSM_ADSP_OP_DM)),

	/* Concurrency 1 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),

	 /* Concurrency 2 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),

	/* Concurrency 3 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),

	/* Concurrency 4 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),

	/* Concurrency 5 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),

	/* Concurrency 6 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
};

#define DEC_INFO(name, queueid, decid, nr_codec) { .module_name = name, \
	.module_queueid = queueid, .module_decid = decid, \
	.nr_codec_support = nr_codec}

#define DEC_INSTANCE(max_instance_same, max_instance_diff) { \
	.max_instances_same_dec = max_instance_same, \
	.max_instances_diff_dec = max_instance_diff}

static struct msm_adspdec_info dec_info_list[] = {
	DEC_INFO("AUDPLAY4TASK", 17, 4, 1),  /* AudPlay4BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY3TASK", 16, 3, 11),  /* AudPlay3BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY2TASK", 15, 2, 11),  /* AudPlay2BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY1TASK", 14, 1, 11),  /* AudPlay1BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY0TASK", 13, 0, 11), /* AudPlay0BitStreamCtrlQueue */
};

static struct dec_instance_table dec_instance_list[][MSM_MAX_DEC_CNT] = {
	/* Non Turbo Mode */
	{
		DEC_INSTANCE(4, 3), /* WAV */
		DEC_INSTANCE(4, 3), /* ADPCM */
		DEC_INSTANCE(4, 2), /* MP3 */
		DEC_INSTANCE(0, 0), /* Real Audio */
		DEC_INSTANCE(4, 2), /* WMA */
		DEC_INSTANCE(3, 2), /* AAC */
		DEC_INSTANCE(0, 0), /* Reserved */
		DEC_INSTANCE(0, 0), /* MIDI */
		DEC_INSTANCE(4, 3), /* YADPCM */
		DEC_INSTANCE(4, 3), /* QCELP */
		DEC_INSTANCE(4, 3), /* AMRNB */
		DEC_INSTANCE(1, 1), /* AMRWB/WB+ */
		DEC_INSTANCE(4, 3), /* EVRC */
		DEC_INSTANCE(1, 1), /* WMAPRO */
	},
	/* Turbo Mode */
	{
		DEC_INSTANCE(4, 3), /* WAV */
		DEC_INSTANCE(4, 3), /* ADPCM */
		DEC_INSTANCE(4, 3), /* MP3 */
		DEC_INSTANCE(0, 0), /* Real Audio */
		DEC_INSTANCE(4, 3), /* WMA */
		DEC_INSTANCE(4, 3), /* AAC */
		DEC_INSTANCE(0, 0), /* Reserved */
		DEC_INSTANCE(0, 0), /* MIDI */
		DEC_INSTANCE(4, 3), /* YADPCM */
		DEC_INSTANCE(4, 3), /* QCELP */
		DEC_INSTANCE(4, 3), /* AMRNB */
		DEC_INSTANCE(2, 3), /* AMRWB/WB+ */
		DEC_INSTANCE(4, 3), /* EVRC */
		DEC_INSTANCE(1, 2), /* WMAPRO */
	},
};

static struct msm_adspdec_database msm_device_adspdec_database = {
	.num_dec = ARRAY_SIZE(dec_info_list),
	.num_concurrency_support = (ARRAY_SIZE(dec_concurrency_table) / \
					ARRAY_SIZE(dec_info_list)),
	.dec_concurrency_table = dec_concurrency_table,
	.dec_info_list = dec_info_list,
	.dec_instance_list = &dec_instance_list[0][0],
};

static struct platform_device msm_device_adspdec = {
	.name = "msm_adspdec",
	.id = -1,
	.dev    = {
		.platform_data = &msm_device_adspdec_database
	},
};

static struct resource smc91x_resources[] = {
	[0] = {
		.start = 0x8A000300,
		.end = 0x8A0003ff,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start = MSM_GPIO_TO_INT(156),
		.end = MSM_GPIO_TO_INT(156),
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device smc91x_device = {
	.name           = "smc91x",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(smc91x_resources),
	.resource       = smc91x_resources,
};

static struct smsc911x_platform_config smsc911x_config = {
	.phy_interface	= PHY_INTERFACE_MODE_MII,
	.irq_polarity	= SMSC911X_IRQ_POLARITY_ACTIVE_LOW,
	.irq_type	= SMSC911X_IRQ_TYPE_PUSH_PULL,
	.flags		= SMSC911X_USE_32BIT,
};

static struct resource smsc911x_resources[] = {
	[0] = {
		.start		= 0x8D000000,
		.end		= 0x8D000100,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= MSM_GPIO_TO_INT(88),
		.end		= MSM_GPIO_TO_INT(88),
		.flags		= IORESOURCE_IRQ,
	},
};

static struct platform_device smsc911x_device = {
	.name		= "smsc911x",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(smsc911x_resources),
	.resource	= smsc911x_resources,
	.dev		= {
		.platform_data = &smsc911x_config,
	},
};

static struct msm_gpio smsc911x_gpios[] = {
    { GPIO_CFG(172, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "ebi2_addr6" },
    { GPIO_CFG(173, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "ebi2_addr5" },
    { GPIO_CFG(174, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "ebi2_addr4" },
    { GPIO_CFG(175, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "ebi2_addr3" },
    { GPIO_CFG(176, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "ebi2_addr2" },
    { GPIO_CFG(177, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "ebi2_addr1" },
    { GPIO_CFG(178, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "ebi2_addr0" },
    { GPIO_CFG(88, 2, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), "smsc911x_irq"  },
};

static void msm7x30_cfg_smsc911x(void)
{
	int rc;

	rc = msm_gpios_request_enable(smsc911x_gpios,
			ARRAY_SIZE(smsc911x_gpios));
	if (rc)
		pr_err("%s: unable to enable gpios\n", __func__);
}

#ifdef CONFIG_OPTICAL_GP2A
static struct i2c_gpio_platform_data opt_i2c_gpio_data = {
	.sda_pin    = 173,
	.scl_pin    = 172,
};

static struct platform_device opt_i2c_gpio_device = {  
	.name       = "i2c-gpio",
	.id     = 17,
	.dev        = {
		.platform_data  = &opt_i2c_gpio_data,
	},
};
static struct i2c_board_info opt_i2c_borad_info[] = {
	{
		I2C_BOARD_INFO("gp2a", 0x88>>1),
      	//.platform_data  = &opt_i2c_gpio_data,
	},
};

static struct platform_device opt_gp2a = {
	.name = "gp2a-opt",
	.id = -1,
};
#endif /* CONFIG_OPTICAL_GP2A */
#ifdef CONFIG_GYRO_L3G4200D

static struct i2c_gpio_platform_data gyro_i2c_gpio_data = {
	.scl_pin    = 148,
	.sda_pin    = 149,	
};

static struct platform_device gyro_i2c_gpio_device = {  
	.name       = "i2c-gpio",
	.id     	= 18,
	.dev        = {
		.platform_data  = &gyro_i2c_gpio_data,
	},
};

//static struct l3g4200d_platform_data l3g4200d_platform_data = {};

static struct i2c_board_info gyro_i2c_devices[] = {
	{
		I2C_BOARD_INFO("l3g4200d", 0x68),
		//.platform_data = &l3g4200d_platform_data,
		.platform_data = &gyro_i2c_gpio_data,
	},
};
#endif

#ifdef CONFIG_SAMSUNG_FM_SI4709
static struct i2c_gpio_platform_data fmradio_i2c_gpio_data = {
	.sda_pin    = 123,
	.scl_pin    = 122,
};

static struct platform_device fmradio_i2c_gpio_device = {  
	.name       = "i2c-gpio",
	.id     = 19,
	.dev        = {
		.platform_data  = &fmradio_i2c_gpio_data,
	},
};
#endif

#ifdef CONFIG_SENSORS_BMA023_ACCEL
static struct i2c_gpio_platform_data acc_i2c_gpio_data = {
	.sda_pin    = 149,
	.scl_pin    = 148,
};

static struct platform_device acc_i2c_gpio_device = {  
	.name       = "i2c-gpio",
	.id     	= 8,
	.dev        = {
		.platform_data  = &acc_i2c_gpio_data,
	},
};

static struct i2c_board_info acc_i2c_devices[] = {
	{
		I2C_BOARD_INFO("accelerometer", 0x08), /* [HSS] BMA023 : 0x38, BMA222 : 0x08 */
	},
};
#endif

#ifdef CONFIG_SENSORS_YAS529_MAGNETIC
static struct i2c_gpio_platform_data mag_i2c_gpio_data = {
	.sda_pin    = 88,
	.scl_pin    = 86,
};
static struct platform_device mag_i2c_gpio_device = {  
	.name       = "i2c-gpio",
	.id     	= 12,
	.dev        = {
		.platform_data  = &mag_i2c_gpio_data,
	},
};

static struct i2c_board_info mag_i2c_devices[] = {
	{
		I2C_BOARD_INFO("geomagnetic", 0x2E),		// 0x2E
	},
};

static uint32_t magnetic_device_gpio_config[] = {
	GPIO_CFG(MSM_GPIO_MSENSE_RST, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

static int __init magnetic_device_init(void)
{
#if 1 // [HSS] case of using MSM_GPIO
	config_gpio_table(magnetic_device_gpio_config,
		ARRAY_SIZE(magnetic_device_gpio_config));
#endif	
	return 0;
}
#endif

#ifdef CONFIG_SENSORS_AK8975
static struct akm8975_platform_data akm8975_pdata = {
		.gpio_data_ready_int = MSM_GPIO_MSENSE_RST, //PM8058_GPIO_PM_TO_SYS(PM8058_GPIO(33)), 
};

static struct i2c_gpio_platform_data mag_i2c_gpio_data = {
	.sda_pin    = 88,
	.scl_pin    = 86,
};
static struct platform_device mag_i2c_gpio_device = {  
	.name       = "i2c-gpio",
	.id     	= 12,
	.dev        = {
		.platform_data  = &mag_i2c_gpio_data,
	},
};

static struct i2c_board_info mag_i2c_devices[] = {
	{
		I2C_BOARD_INFO("ak8975", 0x0C),
		.platform_data = &akm8975_pdata,
	},
};

static uint32_t magnetic_device_gpio_config[] = {
	GPIO_CFG(MSM_GPIO_MSENSE_RST, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

static int __init magnetic_device_init(void)
{
#if 1 // [HSS] case of using MSM_GPIO
	config_gpio_table(magnetic_device_gpio_config,
		ARRAY_SIZE(magnetic_device_gpio_config));
#endif	
	return 0;
}
#endif


#if defined(CONFIG_CHARGER_SMB328A)
 

static struct i2c_gpio_platform_data fg_smb_i2c_gpio_data = {
	.sda_pin    = 125,
	.scl_pin    = 124,
};

static struct platform_device fg_smb_i2c_gpio_device = {  
	.name       = "i2c-gpio",
	.id     = 21,
	.dev        = {
		.platform_data  = &fg_smb_i2c_gpio_data,
	},
};

static struct i2c_board_info fg_smb_i2c_devices[] = {
    {
            I2C_BOARD_INFO("smb328a", 0x69>>1),
			.irq = PM8058_GPIO_IRQ(PM8058_IRQ_BASE, PMIC_GPIO_CHG_STAT),
    },
};

#endif /* defined(CONFIG_CHARGER_SMB328A) */

/* 2011-01-27 hyeokseon.yu */
#ifdef CONFIG_MAX17043_FUEL_GAUGE

static struct i2c_gpio_platform_data fuelgauge_i2c_gpio_data = {
	.sda_pin    = 169,
	.scl_pin    = 168,
};

static struct platform_device fuelgauge_i2c_gpio_device = {  
	.name       = "i2c-gpio",
	.id     = 11,
	.dev        = {
		.platform_data  = &fuelgauge_i2c_gpio_data,
	},
};

static struct i2c_board_info fuelgauge_i2c_devices[] = {
    {
            I2C_BOARD_INFO("fuelgauge_max17043", 0x6D>>1),
			.irq = MSM_GPIO_TO_INT(110),
    },
};
#endif


#if 1
static struct i2c_gpio_platform_data micro_usb_i2c_gpio_data = {
	.sda_pin    = 55,
	.scl_pin    = 89,
};

static struct platform_device micro_usb_i2c_gpio_device = {  
	.name       = "i2c-gpio",
	.id     = 10,
	.dev        = {
		.platform_data  = &micro_usb_i2c_gpio_data,
	},
};
static struct i2c_board_info micro_usb_i2c_devices[] = {
	{
		I2C_BOARD_INFO("fsa9480", 0x4A>>1),
		.irq = MSM_GPIO_TO_INT(142),
	},
};
#endif

#if 1
static struct usb_mass_storage_platform_data usb_mass_storage_pdata = {
	.nluns          = 0x02,
//	.buf_size       = 16384,
	.vendor         = "SAMSUNG",
	.product        = "SGH-T679 Card",
	.release        = 0xffff,
};

static struct platform_device mass_storage_device = {
	.name           = "usb_mass_storage",
	.id             = -1,
	.dev            = {
		.platform_data          = &usb_mass_storage_pdata,
	},
};
#endif
#ifdef CONFIG_USB_ANDROID
static char *usb_functions_default[] = {
	"diag",
	"modem",
//	"nmea",
//	"rmnet",
	"usb_mass_storage",
};

static char *usb_functions_default_adb[] = {
	"diag",
	"adb",
	"modem",
//	"nmea",
//	"rmnet",
	"usb_mass_storage",
};

static char *usb_functions_rndis[] = {
	"rndis",
};

static char *usb_functions_rndis_adb[] = {
	"rndis",
	"adb",
};

static char *usb_functions_mtp_only[] = {
	"usb_mtp_gadget",
};
static char *usb_functions_kies[] = {
//	"usb_mtp_gadget",
	"modem",
	"usb_mtp_gadget",	
};

static char *usb_functions_ums[] = {
	"usb_mass_storage",
};

#if 1

/* Do not use below compoiste */
#else
static char *usb_functions_ums_adb[] = {
	"usb_mass_storage",
	"adb",
};
#endif


#if 1
/* soonyong.cho : Variables for samsung composite such as kies, mtp, ums, etc... */
/* kies mode */
static char *usb_functions_acm_mtp[] = {
	"acm",
	"mtp",
};
/* debug mode */
static char *usb_functions_acm_ums_adb[] = {
	"acm",
	"usb_mass_storage",
	"adb",
};
/* mtp only mode */
static char *usb_functions_mtp[] = {
	"mtp",
};
/* dm only mode */
static char *usb_functions_dm[] = {
	"diag",
};
static char *usb_functions_adb_diag[] = {
	"acm",
	"diag",		
};

#endif


static char *usb_functions_all[] = {
#if 1
	/* soonyong.cho : Every function driver for samsung composite.
	 *			  Number of to enable function features have to be same as below.
	 */
		"acm",
		"usb_mass_storage",
		"adb",
		"rndis",
		"mtp",
		"diag",
#else /* original */
	
	

//	"acm_modem",
//	"nmea",
	"modem",	// acm_modem
	"diag",
	"usb_mass_storage",
	"adb",
    "usb_mtp_gadget",
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
#ifdef CONFIG_USB_ANDROID_RMNET
	"rmnet",
#endif


#endif /* CONFIG_USB_ANDROID_SAMSUNG_COMPOSITE */

};

static struct android_usb_product usb_products[] = {
	
#if 1
	/* soonyong.cho : Please modify below value correctly if you customize composite */
#  ifdef CONFIG_USB_ANDROID_SAMSUNG_ESCAPE /* USE DEVGURU HOST DRIVER */
		{
			.product_id = SAMSUNG_DEBUG_PRODUCT_ID,
			.num_functions	= ARRAY_SIZE(usb_functions_acm_ums_adb),
			.functions	= usb_functions_acm_ums_adb,
			.bDeviceClass	= 0xEF,
			.bDeviceSubClass= 0x02,
			.bDeviceProtocol= 0x01,
			.s		= ANDROID_DEBUG_CONFIG_STRING,
			.mode		= USBSTATUS_ADB,
		},
		{
			.product_id = SAMSUNG_KIES_PRODUCT_ID,
			.num_functions	= ARRAY_SIZE(usb_functions_acm_mtp),
			.functions	= usb_functions_acm_mtp,
			.bDeviceClass	= 0xEF,
			.bDeviceSubClass= 0x02,
			.bDeviceProtocol= 0x01,
			.s		= ANDROID_KIES_CONFIG_STRING,
			.mode		= USBSTATUS_SAMSUNG_KIES,
		},
		{
			.product_id = SAMSUNG_UMS_PRODUCT_ID,
			.num_functions	= ARRAY_SIZE(usb_functions_ums),
			.functions	= usb_functions_ums,
			.bDeviceClass	= USB_CLASS_PER_INTERFACE,
			.bDeviceSubClass= 0,
			.bDeviceProtocol= 0,
			.s		= ANDROID_UMS_CONFIG_STRING,
			.mode		= USBSTATUS_UMS,
		},
		{
			.product_id = SAMSUNG_RNDIS_PRODUCT_ID,
			.num_functions	= ARRAY_SIZE(usb_functions_rndis),
			.functions	= usb_functions_rndis,
#    ifdef CONFIG_USB_ANDROID_SAMSUNG_RNDIS_WITH_MS_COMPOSITE
			.bDeviceClass	= 0xEF,
			.bDeviceSubClass= 0x02,
			.bDeviceProtocol= 0x01,
#    else
#      ifdef CONFIG_USB_ANDROID_RNDIS_WCEIS
			.bDeviceClass	= USB_CLASS_WIRELESS_CONTROLLER,
#      else
			.bDeviceClass	= USB_CLASS_COMM,
#      endif
			.bDeviceSubClass= 0,
			.bDeviceProtocol= 0,
#    endif
			.s		= ANDROID_RNDIS_CONFIG_STRING,
			.mode		= USBSTATUS_VTP,
		},
		{
			.product_id = SAMSUNG_MTP_PRODUCT_ID,
			.num_functions	= ARRAY_SIZE(usb_functions_mtp),
			.functions	= usb_functions_mtp,
			.bDeviceClass	= USB_CLASS_PER_INTERFACE,
			.bDeviceSubClass= 0,
			.bDeviceProtocol= 0x01,
			.s		= ANDROID_MTP_CONFIG_STRING,
			.mode		= USBSTATUS_MTPONLY,
		},
	
#  else  /* USE MCCI HOST DRIVER */
		{
			.product_id = SAMSUNG_DEBUG_PRODUCT_ID, /* change sequence */
			.num_functions	= ARRAY_SIZE(usb_functions_acm_ums_adb),
			.functions	= usb_functions_acm_ums_adb,
			.bDeviceClass	= USB_CLASS_COMM,
			.bDeviceSubClass= 0,
			.bDeviceProtocol= 0,
			.s		= ANDROID_DEBUG_CONFIG_STRING,
			.mode		= USBSTATUS_ADB,
		},
		{
			.product_id = SAMSUNG_KIES_PRODUCT_ID, /* change sequence */
			.num_functions	= ARRAY_SIZE(usb_functions_acm_mtp),
			.functions	= usb_functions_acm_mtp,
			.bDeviceClass	= USB_CLASS_COMM,
			.bDeviceSubClass= 0,
			.bDeviceProtocol= 0,
			.s		= ANDROID_KIES_CONFIG_STRING,
			.mode		= USBSTATUS_SAMSUNG_KIES,
	
		},
		{
			.product_id = SAMSUNG_UMS_PRODUCT_ID,
			.num_functions	= ARRAY_SIZE(usb_functions_ums),
			.functions	= usb_functions_ums,
			.bDeviceClass	= USB_CLASS_PER_INTERFACE,
			.bDeviceSubClass= 0,
			.bDeviceProtocol= 0,
			.s		= ANDROID_UMS_CONFIG_STRING,
			.mode		= USBSTATUS_UMS,
		},
		{
			.product_id = SAMSUNG_RNDIS_PRODUCT_ID,
			.num_functions	= ARRAY_SIZE(usb_functions_rndis),
			.functions	= usb_functions_rndis,
#    ifdef CONFIG_USB_ANDROID_RNDIS_WCEIS
			.bDeviceClass	= USB_CLASS_WIRELESS_CONTROLLER,
#    else
			.bDeviceClass	= USB_CLASS_COMM,
#    endif
			.bDeviceSubClass= 0,
			.bDeviceProtocol= 0,
			.s		= ANDROID_RNDIS_CONFIG_STRING,
			.mode		= USBSTATUS_VTP,
		},
		{
			.product_id = SAMSUNG_MTP_PRODUCT_ID,
			.num_functions	= ARRAY_SIZE(usb_functions_mtp),
			.functions	= usb_functions_mtp,
			.bDeviceClass	= USB_CLASS_PER_INTERFACE,
			.bDeviceSubClass= 0,
			.bDeviceProtocol= 0x01,
			.s		= ANDROID_MTP_CONFIG_STRING,
			.mode		= USBSTATUS_MTPONLY,
		},
		#if 1
		{
			.product_id = 0x689E, /* change sequence */
			.num_functions	= ARRAY_SIZE(usb_functions_adb_diag),
			.functions	= usb_functions_adb_diag,
			.bDeviceClass	= USB_CLASS_COMM,
			.bDeviceSubClass= 0,
			.bDeviceProtocol= 0,
			.s		= ANDROID_DEBUG_CONFIG_STRING,
			.mode		= USBSTATUS_DM,
		},

		#else
		{
			.product_id = 0xaaaa,
			.num_functions	= ARRAY_SIZE(usb_functions_dm),
			.functions	= usb_functions_dm,
			.bDeviceClass	= USB_CLASS_PER_INTERFACE,
			.bDeviceSubClass= 0,
			.bDeviceProtocol= 0x01,
			.s		= "DM Only",
			.mode		= USBSTATUS_DM,
		},
		#endif
#  endif
#else /* original */
/*
	{
		.product_id	= 0x9026,
		.num_functions	= ARRAY_SIZE(usb_functions_default),
		.functions	= usb_functions_default,
	},
*/
	{
		.product_id	= 0x689E,
		.num_functions	= ARRAY_SIZE(usb_functions_default),
		.functions	= usb_functions_default,
	},
	{
		.product_id	= 0x689E,
		.num_functions	= ARRAY_SIZE(usb_functions_default_adb),
		.functions	= usb_functions_default_adb,
	},
	{
		.product_id	= 0x5A0F,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_only),
		.functions	= usb_functions_mtp_only,
	},
	{
		.product_id	= 0x6881,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis),
		.functions	= usb_functions_rndis,
	},
	{
		.product_id	= 0x6877,
		.num_functions	= ARRAY_SIZE(usb_functions_kies),
		.functions	= usb_functions_kies,
	},	
/*	
	{
		.product_id	= 0x9024,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_adb),
		.functions	= usb_functions_rndis_adb,
	},
*/
			
			
#endif /* CONFIG_USB_ANDROID_SAMSUNG_COMPOSITE */

};
#if 0
static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.nluns		= 1,
	.vendor		= "Qualcomm Incorporated",
	.product        = "Mass storage",
	.release	= 0x0100,
};

static struct platform_device usb_mass_storage_device = {
	.name           = "usb_mass_storage",
	.id             = -1,
	.dev            = {
		.platform_data          = &mass_storage_pdata,
	},
};
#endif

static struct usb_ether_platform_data rndis_pdata = {
	/* ethaddr is filled by board_serialno_setup */
	.vendorID	= 0x04E8,
	.vendorDescr	= "Qualcomm Incorporated",
};

static struct platform_device rndis_device = {
	.name	= "rndis",
	.id	= -1,
	.dev	= {
		.platform_data = &rndis_pdata,
	},
};

#define MAX_USB_SERIAL_NUM	17

// serial number should be changed as real device for commercial release
static char device_serial[MAX_USB_SERIAL_NUM] = "0123456789ABCDEF";


static struct android_usb_platform_data android_usb_pdata = {
#if 1
	/* soonyong.cho : refered from S1 */
		.vendor_id		= SAMSUNG_VENDOR_ID,
		.product_id 	= SAMSUNG_KIES_PRODUCT_ID,
		.manufacturer_name	= "SAMSUNG",
		.product_name		= "SAMSUNG_Android",
#else
	
	.vendor_id	= 0x04E8, // Samsung Vendor ID
	.product_id	= 0x689E,
	.version	= 0x0100,
	.product_name	= "Samsung Android USB Device",
	.manufacturer_name = "SAMSUNG Electronics Co., Ltd.",
#endif /* CONFIG_USB_ANDROID_SAMSUNG_COMPOSITE */
	
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,

	.num_functions = ARRAY_SIZE(usb_functions_all),
	.functions = usb_functions_all,
	
#if 1

	/* soonyong.cho : refered from S1 */
		.serial_number		= device_serial,
#else
	.serial_number = "1234567890ABCDEF",
#endif
};

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id		= -1,
	.dev		= {
		.platform_data = &android_usb_pdata,
	},
};

#if 1
void __init s3c_usb_set_serial(void)
{
# ifdef CONFIG_USB_ANDROID_RNDIS
	int i;
	char *src;
# endif

	sprintf(device_serial, "%08X%08X", system_serial_high,
			system_serial_low);
# ifdef CONFIG_USB_ANDROID_RNDIS
	/* create a fake MAC address from our serial number.
	 * first byte is 0x02 to signify locally administered.
	 */
	src = device_serial;
	rndis_pdata.ethaddr[0] = 0x02;
	for (i = 0; *src; i++) {
		/* XOR the USB serial across the remaining bytes */
		rndis_pdata.ethaddr[i % (ETH_ALEN - 1) + 1] ^= *src++;
	}
# endif
}
#endif /* CONFIG_USB_ANDROID_SAMSUNG_COMPOSITE */


static int __init board_serialno_setup(char *serialno)
{
	int i;
	char *src = serialno;

	/* create a fake MAC address from our serial number.
	 * first byte is 0x02 to signify locally administered.
	 */
	rndis_pdata.ethaddr[0] = 0x02;
	for (i = 0; *src; i++) {
		/* XOR the USB serial across the remaining bytes */
		rndis_pdata.ethaddr[i % (ETH_ALEN - 1) + 1] ^= *src++;
	}

	android_usb_pdata.serial_number = serialno;
	return 1;
}
__setup("androidboot.serialno=", board_serialno_setup);

#endif

#if defined (CONFIG_TOUCHSCREEN_MELFAS_MCS8000)

#define GPIO_TOUCH_INT	119
#define GPIO_I2C0_SCL   70
#define GPIO_I2C0_SDA   71
#define TSP_LDO_ON 133

static uint32_t touch_gpios[] = {
	GPIO_CFG(GPIO_TOUCH_INT,  1, GPIO_CFG_OUTPUT,  GPIO_CFG_PULL_UP, GPIO_CFG_16MA), /* lcdc_pclk */
	GPIO_CFG(GPIO_I2C0_SCL,  1, GPIO_CFG_OUTPUT,  GPIO_CFG_PULL_UP, GPIO_CFG_16MA), /* lcdc_hsync*/
	GPIO_CFG(GPIO_I2C0_SDA,  1, GPIO_CFG_OUTPUT,  GPIO_CFG_PULL_UP, GPIO_CFG_16MA), /* lcdc_vsync*/
	GPIO_CFG(TSP_LDO_ON,  1, GPIO_CFG_OUTPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_16MA), /* lcdc_den */
};

static struct i2c_gpio_platform_data touch_i2c_gpio_data = {
    .sda_pin    = 71,
    .scl_pin    = 70,
};

static struct platform_device touch_i2c_gpio_device = {  
    .name       = "i2c-gpio",
    .id     = 14,
    .dev        = {
        .platform_data  = &touch_i2c_gpio_data,
    },
};

static struct i2c_board_info touch_i2c_devices[] __initdata= {
  {
	I2C_BOARD_INFO("mcs8000_i2c", 0x48),
        .irq = MSM_GPIO_TO_INT( 119 ),       
  }
};

#ifdef NOT_USE
static struct platform_device mcs8000_ts_device = {
	.name		= "mcs8000-ts",
	.id 		= -1,
};
#endif

#endif



#if defined (CONFIG_TOUCHSCREEN_QT602240)
static struct i2c_gpio_platform_data touchscreen_i2c_gpio_data = {
	.sda_pin    = 71,
	.scl_pin    = 70,
};

static struct platform_device touchscreen_i2c_gpio_device = {  
	.name       = "i2c-gpio",
	.id     = 14,
	.dev        = {
		.platform_data  = &touchscreen_i2c_gpio_data,
	},
};

#ifdef NOT_USE
static struct platform_device touchscreen_device_qt602240 = {
	.name = "qt602240-ts",
	.id = -1,
};
#endif

static struct i2c_board_info qt602240_touch_boardinfo[] __initdata = {
	{
		I2C_BOARD_INFO("qt602240-ts", 0x4A ),
		.irq = MSM_GPIO_TO_INT(119),
	}
};

#ifdef CONFIG_KEYPAD_CYPRESS_TOUCH
static struct i2c_gpio_platform_data touch_keypad_i2c_platdata = {
	.sda_pin		= _3_TOUCH_SDA_28V,
	.scl_pin		= _3_TOUCH_SCL_28V,
	.udelay 		= 0, /* 250KHz */
	.sda_is_open_drain	= 0,
	.scl_is_open_drain	= 0,
	.scl_is_output_only	= 0,
};

static struct platform_device touch_keypad_i2c_device = {
	.name			= "i2c-gpio",
	.id			= 20,
	.dev.platform_data	= &touch_keypad_i2c_platdata,
};
static void touch_keypad_gpio_init(void)
{
	int ret = 0;

	gpio_tlmm_config(GPIO_CFG(_3_TOUCH_EN, 0, GPIO_CFG_OUTPUT,
				  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_set_value(_3_TOUCH_EN, 1);

	printk("touch_keypad_gpio_init.\n");	
}

static void touch_keypad_onoff(int onoff)
{
//	gpio_direction_output(_3_TOUCH_EN, onoff);
	gpio_tlmm_config(GPIO_CFG(_3_TOUCH_EN, 0, GPIO_CFG_OUTPUT,
				  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

	gpio_set_value(_3_TOUCH_EN, onoff);

	if (onoff == TOUCHKEY_OFF)
		msleep(30);
	else
		msleep(25);

	printk("touch_keypad_onoff %d .\n",onoff);		
}

static const int touch_keypad_code[] = {
	KEY_MENU,
	KEY_BACK,
};

static struct touchkey_platform_data touchkey_data = {
	.keycode_cnt = ARRAY_SIZE(touch_keypad_code),
	.keycode = touch_keypad_code,
	.touchkey_onoff = touch_keypad_onoff,
};

static struct i2c_board_info touchkey_info[] __initdata = {
	{
		I2C_BOARD_INFO(CYPRESS_TOUCHKEY_DEV_NAME, 0x20),
		.platform_data = &touchkey_data,			
		.irq = MSM_GPIO_TO_INT(_3_TOUCH_INT),
	},
};

#endif

static int oliver_tsp_ldo_on(void)
{
	int rc = 0;
	struct vreg *vreg_ldo2, *vreg_ldo10 = NULL;

	printk("[TSP] M1 TSP LDO init\n");
	// VREG_TSP_1.8V
	vreg_ldo2 = vreg_get(NULL, "gp4");		
	if (IS_ERR(vreg_ldo2)) {
		rc = PTR_ERR(vreg_ldo2);
		pr_err("%s: gp6 vreg get failed (%d)\n",
		       __func__, rc);
		return rc;
	}

	// VREG_TSP_A3.0V
	// Oliver Board rev univ01 ldo10 is gp11
	vreg_ldo10 = vreg_get(NULL, "xo_out"); 
	if (IS_ERR(vreg_ldo10)) {
		rc = PTR_ERR(vreg_ldo10);
		pr_err("%s: gp9 vreg get failed (%d)\n",
		       __func__, rc);
		return rc;
	}

	rc = vreg_set_level(vreg_ldo2, 1800);
	if (rc) {
		pr_err("%s: vreg LDO2 set level failed (%d)\n",
		       __func__, rc);
		return rc;
	}

	rc = vreg_set_level(vreg_ldo10, 3000);
	if (rc) {
		pr_err("%s: vreg LDO10 set level failed (%d)\n",
		       __func__, rc);
		return rc;
	}

	rc = vreg_enable(vreg_ldo2);

	if (rc) {
		pr_err("%s: LDO2 vreg enable failed (%d)\n",
		       __func__, rc);
		return rc;
	}

	rc = vreg_enable(vreg_ldo10);

	if (rc) {
		pr_err("%s: LDO10 vreg enable failed (%d)\n",
		       __func__, rc);
		return rc;
	}

	mdelay(5);		/* ensure power is stable */

	return rc;

}
#endif

static struct msm_gpio optnav_config_data[] = {
	{ GPIO_CFG(OPTNAV_CHIP_SELECT, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	"optnav_chip_select" },
};

static void __iomem *virtual_optnav;

static int optnav_gpio_setup(void)
{
	int rc = -ENODEV;
	rc = msm_gpios_request_enable(optnav_config_data,
			ARRAY_SIZE(optnav_config_data));

	/* Configure the FPGA for GPIOs */
	virtual_optnav = ioremap(FPGA_OPTNAV_GPIO_ADDR, 0x4);
	if (!virtual_optnav) {
		pr_err("%s:Could not ioremap region\n", __func__);
		return -ENOMEM;
	}
	/*
	 * Configure the FPGA to set GPIO 19 as
	 * normal, active(enabled), output(MSM to SURF)
	 */
	writew(0x311E, virtual_optnav);
	return rc;
}

static void optnav_gpio_release(void)
{
	msm_gpios_disable_free(optnav_config_data,
		ARRAY_SIZE(optnav_config_data));
	iounmap(virtual_optnav);
}

static struct vreg *vreg_gp7;
static struct vreg *vreg_gp4;
static struct vreg *vreg_gp9;
static struct vreg *vreg_usb3_3;

static int optnav_enable(void)
{
	int rc;
	/*
	 * Enable the VREGs L8(gp7), L10(gp4), L12(gp9), L6(usb)
	 * for I2C communication with keyboard.
	 */
	vreg_gp7 = vreg_get(NULL, "gp7");
	rc = vreg_set_level(vreg_gp7, 1800);
	if (rc) {
		pr_err("%s: vreg_set_level failed \n", __func__);
		goto fail_vreg_gp7;
	}

	rc = vreg_enable(vreg_gp7);
	if (rc) {
		pr_err("%s: vreg_enable failed \n", __func__);
		goto fail_vreg_gp7;
	}

	vreg_gp4 = vreg_get(NULL, "gp4");
	rc = vreg_set_level(vreg_gp4, 2600);
	if (rc) {
		pr_err("%s: vreg_set_level failed \n", __func__);
		goto fail_vreg_gp4;
	}

	rc = vreg_enable(vreg_gp4);
	if (rc) {
		pr_err("%s: vreg_enable failed \n", __func__);
		goto fail_vreg_gp4;
	}

	vreg_gp9 = vreg_get(NULL, "gp9");
	rc = vreg_set_level(vreg_gp9, 1800);
	if (rc) {
		pr_err("%s: vreg_set_level failed \n", __func__);
		goto fail_vreg_gp9;
	}

	rc = vreg_enable(vreg_gp9);
	if (rc) {
		pr_err("%s: vreg_enable failed \n", __func__);
		goto fail_vreg_gp9;
	}

	vreg_usb3_3 = vreg_get(NULL, "usb");
	rc = vreg_set_level(vreg_usb3_3, 3300);
	if (rc) {
		pr_err("%s: vreg_set_level failed \n", __func__);
		goto fail_vreg_3_3;
	}

	rc = vreg_enable(vreg_usb3_3);
	if (rc) {
		pr_err("%s: vreg_enable failed \n", __func__);
		goto fail_vreg_3_3;
	}

	/* Enable the chip select GPIO */
	gpio_set_value(OPTNAV_CHIP_SELECT, 1);
	gpio_set_value(OPTNAV_CHIP_SELECT, 0);

	return 0;

fail_vreg_3_3:
	vreg_disable(vreg_gp9);
fail_vreg_gp9:
	vreg_disable(vreg_gp4);
fail_vreg_gp4:
	vreg_disable(vreg_gp7);
fail_vreg_gp7:
	return rc;
}

static void optnav_disable(void)
{
	vreg_disable(vreg_usb3_3);
	vreg_disable(vreg_gp9);
	vreg_disable(vreg_gp4);
	vreg_disable(vreg_gp7);

	gpio_set_value(OPTNAV_CHIP_SELECT, 1);
}

static struct ofn_atlab_platform_data optnav_data = {
	.gpio_setup    = optnav_gpio_setup,
	.gpio_release  = optnav_gpio_release,
	.optnav_on     = optnav_enable,
	.optnav_off    = optnav_disable,
	.rotate_xy     = 0,
	.function1 = {
		.no_motion1_en		= true,
		.touch_sensor_en	= true,
		.ofn_en			= true,
		.clock_select_khz	= 1500,
		.cpi_selection		= 1200,
	},
	.function2 =  {
		.invert_y		= false,
		.invert_x		= true,
		.swap_x_y		= false,
		.hold_a_b_en		= true,
		.motion_filter_en       = true,
	},
};

static int hdmi_comm_power(int on, int show);

#if defined (CONFIG_SENSOR_MC7) || defined (CONFIG_SENSOR_CE147) || defined (CONFIG_SENSOR_S5K4ECGX) || defined (CONFIG_SENSOR_S5K5CCAF)
static struct i2c_board_info msm_cam_pm_lp8720_info[] = {
	{
		I2C_BOARD_INFO("cam_pm_lp8720", 0x7D),
	},
};

static struct i2c_gpio_platform_data cam_pm_lp8720_data = {
	.scl_pin    = 164,
	.sda_pin    = 165,
};

static struct platform_device cam_pm_lp8720_i2c_device = {  
	.name	= "i2c-gpio",
	.id	= 13,
	.dev	= {
		.platform_data  = &cam_pm_lp8720_data,
	},
};
#endif

static struct i2c_board_info msm_i2c_board_info[] = {
	{
		I2C_BOARD_INFO("m33c01", OPTNAV_I2C_SLAVE_ADDR),
		.irq		= MSM_GPIO_TO_INT(OPTNAV_IRQ),
		.platform_data = &optnav_data,
	},
};

static struct i2c_board_info msm_marimba_board_info[] = {
	{
		I2C_BOARD_INFO("marimba", 0xc),
		.platform_data = &marimba_pdata,
	}
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

};
static struct msm_hsusb_platform_data msm_hsusb_pdata = {
	.version	= 0x0100,
	.phy_info	= USB_PHY_INTEGRATED | USB_PHY_MODEL_45NM,
	.vendor_id	= 0x5c6,
	.product_name	= "Qualcomm HSUSB Device",
	.serial_number	= "1234567890ABCDEF",
	.manufacturer_name
			= "Qualcomm Incorporated",
	.compositions	= usb_func_composition,
	.num_compositions
			= ARRAY_SIZE(usb_func_composition),
	.function_map	= usb_functions_map,
	.num_functions	= ARRAY_SIZE(usb_functions_map),
	.core_clk	= 1,
};
#endif

static struct msm_handset_platform_data hs_platform_data = {
	.hs_name = "sec_power_key",
	.pwr_key_delay_ms = 500, /* 0 will disable end key */
};

static struct platform_device hs_device = {
	.name   = "msm-handset",
	.id     = -1,
	.dev    = {
		.platform_data = &hs_platform_data,
	},
};

static struct msm_pm_platform_data msm_pm_data[MSM_PM_SLEEP_MODE_NR] = {
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].supported = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].suspend_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].latency = 8594,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].residency = 23740,

	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].supported = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].suspend_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].latency = 4594,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].residency = 23740,

	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE].supported = 1,
#ifdef CONFIG_MSM_STANDALONE_POWER_COLLAPSE
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE].suspend_enabled = 0,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE].idle_enabled = 1,
#else /*CONFIG_MSM_STANDALONE_POWER_COLLAPSE*/
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE].suspend_enabled = 0,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE].idle_enabled = 0,
#endif /*CONFIG_MSM_STANDALONE_POWER_COLLAPSE*/
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE].latency = 500,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE].residency = 6000,

	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].supported = 1,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].suspend_enabled
		= 1,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].idle_enabled = 0,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency = 443,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].residency = 1098,

	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].supported = 1,
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].suspend_enabled = 1,
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].latency = 2,
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].residency = 0,
};

#ifdef CONFIG_USB_EHCI_MSM
static void msm_hsusb_vbus_power(unsigned phy_info, int on)
{
        int rc;
        static int vbus_is_on;
        struct pm8058_gpio usb_vbus = {
                .direction      = PM_GPIO_DIR_OUT,
                .pull           = PM_GPIO_PULL_NO,
                .output_buffer  = PM_GPIO_OUT_BUF_CMOS,
                .output_value   = 1,
                .vin_sel        = 2,
                .out_strength   = PM_GPIO_STRENGTH_MED,
                .function       = PM_GPIO_FUNC_NORMAL,
                .inv_int_pol    = 0,
        };

        /* If VBUS is already on (or off), do nothing. */
        if (unlikely(on == vbus_is_on))
                return;

        if (on) {
                rc = pm8058_gpio_config(36, &usb_vbus);
                if (rc) {
                        pr_err("%s PMIC GPIO 36 write failed\n", __func__);
                        return;
                }
	} else {
		gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(36), 0);
	}

        vbus_is_on = on;
}

static struct msm_usb_host_platform_data msm_usb_host_pdata = {
        .phy_info   = (USB_PHY_INTEGRATED | USB_PHY_MODEL_45NM),
        .vbus_power = msm_hsusb_vbus_power,
        .power_budget   = 180,
};
#endif

#ifdef CONFIG_USB_MSM_OTG_72K
static int hsusb_rpc_connect(int connect)
{
	if (connect)
		return msm_hsusb_rpc_connect();
	else
		return msm_hsusb_rpc_close();
}
#endif

#ifdef CONFIG_USB_MSM_OTG_72K
static struct vreg *vreg_3p3;
static int msm_hsusb_ldo_init(int init)
{
	uint32_t version = 0;
	int def_vol = 3400;

	version = socinfo_get_version();

	if (SOCINFO_VERSION_MAJOR(version) >= 2 &&
			SOCINFO_VERSION_MINOR(version) >= 1) {
		def_vol = 3075;
		pr_debug("%s: default voltage:%d\n", __func__, def_vol);
	}

	if (init) {
		vreg_3p3 = vreg_get(NULL, "usb");
		if (IS_ERR(vreg_3p3))
			return PTR_ERR(vreg_3p3);
		vreg_set_level(vreg_3p3, def_vol);
	} else
		vreg_put(vreg_3p3);

	return 0;
}

static int msm_hsusb_ldo_enable(int enable)
{
	static int ldo_status;

	if (!vreg_3p3 || IS_ERR(vreg_3p3))
		return -ENODEV;

	if (ldo_status == enable)
		return 0;

	ldo_status = enable;

	if (enable)
		return vreg_enable(vreg_3p3);

	return vreg_disable(vreg_3p3);
}

static int msm_hsusb_ldo_set_voltage(int mV)
{
	static int cur_voltage = 3400;

	if (!vreg_3p3 || IS_ERR(vreg_3p3))
		return -ENODEV;

	if (cur_voltage == mV)
		return 0;

	cur_voltage = mV;

	pr_debug("%s: (%d)\n", __func__, mV);

	return vreg_set_level(vreg_3p3, mV);
}
#endif

#ifndef CONFIG_USB_EHCI_MSM
static int msm_hsusb_pmic_notif_init(void (*callback)(int online), int init);
#endif
static struct msm_otg_platform_data msm_otg_pdata = {
	.rpc_connect	= hsusb_rpc_connect,

#ifndef CONFIG_USB_EHCI_MSM
	.pmic_vbus_notif_init         = msm_hsusb_pmic_notif_init,
#else
	.vbus_power = msm_hsusb_vbus_power,
#endif
	.core_clk		 = 1,
	.pemp_level		 = PRE_EMPHASIS_WITH_20_PERCENT,
	.cdr_autoreset		 = CDR_AUTO_RESET_DISABLE,
	.drv_ampl		 = HS_DRV_AMPLITUDE_75_PERCENT,
	.se1_gating		 = SE1_GATING_DISABLE,
	.chg_vbus_draw		 = hsusb_chg_vbus_draw,
	.chg_connected		 = hsusb_chg_connected,
	.chg_init		 = hsusb_chg_init,
	.ldo_enable		 = msm_hsusb_ldo_enable,
	.ldo_init		 = msm_hsusb_ldo_init,
	.ldo_set_voltage	 = msm_hsusb_ldo_set_voltage,
};

#ifdef CONFIG_USB_GADGET
static struct msm_hsusb_gadget_platform_data msm_gadget_pdata;
#endif
#ifndef CONFIG_USB_EHCI_MSM
typedef void (*notify_vbus_state) (int);
notify_vbus_state notify_vbus_state_func_ptr;
int vbus_on_irq;
static irqreturn_t pmic_vbus_on_irq(int irq, void *data)
{
	pr_info("%s: vbus notification from pmic\n", __func__);

	(*notify_vbus_state_func_ptr) (1);

	return IRQ_HANDLED;
}
static int msm_hsusb_pmic_notif_init(void (*callback)(int online), int init)
{
	int ret;

	if (init) {
		if (!callback)
			return -ENODEV;

		notify_vbus_state_func_ptr = callback;
		vbus_on_irq = platform_get_irq_byname(&msm_device_otg,
			"vbus_on");
		if (vbus_on_irq <= 0) {
			pr_err("%s: unable to get vbus on irq\n", __func__);
			return -ENODEV;
		}

		ret = request_any_context_irq(vbus_on_irq, pmic_vbus_on_irq,
			IRQF_TRIGGER_RISING, "msm_otg_vbus_on", NULL);
		if (ret < 0) {
			pr_info("%s: request_irq for vbus_on"
				"interrupt failed\n", __func__);
			return ret;
		}
		msm_otg_pdata.pmic_vbus_irq = vbus_on_irq;
		return 0;
	} else {
		free_irq(vbus_on_irq, 0);
		notify_vbus_state_func_ptr = NULL;
		return 0;
	}
}
#endif

static struct android_pmem_platform_data android_pmem_pdata = {
	.name = "pmem",
	.allocator_type = PMEM_ALLOCATORTYPE_ALLORNOTHING,
	.cached = 1,
};

static struct platform_device android_pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = { .platform_data = &android_pmem_pdata },
};


static int display_ldo_control(int on)
{
	int rc = 0;
	struct vreg *vreg_ldo15, *vreg_ldo17 = NULL;


	// VREG_LCD_2.8V
	vreg_ldo15 = vreg_get(NULL, "gp6");
	if (IS_ERR(vreg_ldo15)) {
		rc = PTR_ERR(vreg_ldo15);
		pr_err("%s: gp6 vreg get failed (%d)\n",
		       __func__, rc);
		return rc;
	}

	// VREG_LCD_1.8V
	vreg_ldo17 = vreg_get(NULL, "gp11");
	if (IS_ERR(vreg_ldo17)) {
		rc = PTR_ERR(vreg_ldo17);
		pr_err("%s: gp9 vreg get failed (%d)\n",
		       __func__, rc);
		return rc;
	}

	rc = vreg_set_level(vreg_ldo15, 3000);
	if (rc) {
		pr_err("%s: vreg LDO15 set level failed (%d)\n",
		       __func__, rc);
		return rc;
	}

	rc = vreg_set_level(vreg_ldo17, 1800);
	if (rc) {
		pr_err("%s: vreg LDO17 set level failed (%d)\n",
		       __func__, rc);
		return rc;
	}

	if (on)
		rc = vreg_enable(vreg_ldo17);
	else
		rc = vreg_disable(vreg_ldo17);

	if (rc) {
		pr_err("%s: LDO17 vreg enable failed (%d)\n",
		       __func__, rc);
		return rc;
	}

	if (on)
		rc = vreg_enable(vreg_ldo15);
	else
		rc = vreg_disable(vreg_ldo15);

	if (rc) {
		pr_err("%s: LDO15 vreg enable failed (%d)\n",
		       __func__, rc);
		return rc;
	}

	mdelay(5);		/* ensure power is stable */

	return rc;
}

static int lcdc_gpio_array_num[] = {
				45, /* spi_clk */
				46, /* spi_cs  */
				47, /* spi_mosi */
				129, /* spi_miso */
				};

static struct msm_gpio lcdc_gpio_config_data[] = {
	{ GPIO_CFG(45, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "spi_clk" },
	{ GPIO_CFG(46, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "spi_cs0" },
	{ GPIO_CFG(47, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "spi_mosi" },
	{ GPIO_CFG(129, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcd_reset" },
};

/* GPIO TLMM: Status */
#define GPIO_ENABLE     0
#define GPIO_DISABLE    1

/* PMIC LDO : Status */
#define LCD_LDO_ENABLE     1
#define LCD_LDO_DISABLE    0

static void config_lcdc_gpio_table(uint32_t *table, int len, unsigned enable)
{
	int n, rc;


	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n],
			enable ? GPIO_ENABLE : GPIO_DISABLE);
		if (rc) {
			printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
#if 0
	mdelay(10);

	display_ldo_control(enable ? LCD_LDO_ENABLE : LCD_LDO_DISABLE);
#endif	

}

static void lcdc_config_gpios(int enable)
{
	config_lcdc_gpio_table(lcdc_gpio_config_data,
		ARRAY_SIZE(lcdc_gpio_config_data), enable);
}

static struct msm_panel_common_pdata lcdc_panel_data = {
	.panel_config_gpio = lcdc_config_gpios,
	.gpio_num          = lcdc_gpio_array_num,
};

#ifdef CONFIG_FB_MSM_LCDC_S6D16A0X22_WVGA_PANEL
static struct platform_device lcdc_s6d16a0x22_panel_device = {
	.name   = "lcdc_s6d16a0x22_wvga",
	.id     = 0,
	.dev    = {
		.platform_data = &lcdc_panel_data,
	}
};
#endif

#ifdef CONFIG_FB_MSM_LCDC_SAMSUNG_ANCORA_PANEL
static struct platform_device lcdc_samsung_panel_device = {
	.name   = "lcdc_samsung_ancora",
	.id     = 0,
	.dev    = {
		.platform_data = &lcdc_panel_data,
	}
};
#endif

static struct msm_gpio dtv_panel_gpios[] = {
	{ GPIO_CFG(120, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "wca_mclk" },
	{ GPIO_CFG(121, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "wca_sd0" },
	{ GPIO_CFG(122, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "wca_sd1" },
	{ GPIO_CFG(123, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "wca_sd2" },
	{ GPIO_CFG(124, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "dtv_pclk" },
	{ GPIO_CFG(125, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_en" },
	{ GPIO_CFG(126, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_vsync" },
	{ GPIO_CFG(127, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_hsync" },
	{ GPIO_CFG(128, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_data0" },
	{ GPIO_CFG(129, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_data1" },
	{ GPIO_CFG(130, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_data2" },
	{ GPIO_CFG(131, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_data3" },
	{ GPIO_CFG(132, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_data4" },
	{ GPIO_CFG(160, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_data5" },
	{ GPIO_CFG(161, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_data6" },
	{ GPIO_CFG(162, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_data7" },
	{ GPIO_CFG(163, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_data8" },
	{ GPIO_CFG(164, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_data9" },
	{ GPIO_CFG(165, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_dat10" },
	{ GPIO_CFG(166, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_dat11" },
	{ GPIO_CFG(167, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_dat12" },
	{ GPIO_CFG(168, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_dat13" },
	{ GPIO_CFG(169, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_dat14" },
	{ GPIO_CFG(170, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_dat15" },
	{ GPIO_CFG(171, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_dat16" },
	{ GPIO_CFG(172, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_dat17" },
	{ GPIO_CFG(173, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_dat18" },
	{ GPIO_CFG(174, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_dat19" },
	{ GPIO_CFG(175, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_dat20" },
	{ GPIO_CFG(176, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_dat21" },
	{ GPIO_CFG(177, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_dat22" },
	{ GPIO_CFG(178, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_dat23" },
};


#ifdef HDMI_RESET
static unsigned dtv_reset_gpio =
	GPIO_CFG(37, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);
#endif

static int gpio_set(const char *label, const char *name, int level, int on)
{
	struct vreg *vreg = vreg_get(NULL, label);
	int rc;

	if (IS_ERR(vreg)) {
		rc = PTR_ERR(vreg);
		pr_err("%s: vreg %s get failed (%d)\n",
			__func__, name, rc);
		return rc;
	}

	rc = vreg_set_level(vreg, level);
	if (rc) {
		pr_err("%s: vreg %s set level failed (%d)\n",
			__func__, name, rc);
		return rc;
	}

	if (on)
		rc = vreg_enable(vreg);
	else
		rc = vreg_disable(vreg);
	if (rc)
		pr_err("%s: vreg %s enable failed (%d)\n",
			__func__, name, rc);
	return rc;
}

static int dtv_panel_power(int on)
{
	int flag_on = !!on;
	static int dtv_power_save_on;
	struct vreg *vreg_ldo17, *vreg_ldo8;
	int rc;

	if (dtv_power_save_on == flag_on)
		return 0;

	dtv_power_save_on = flag_on;
	pr_info("%s: %d >>\n", __func__, on);

#ifdef HDMI_RESET
	if (on) {
		rc = gpio_tlmm_config(dtv_reset_gpio, GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
				       __func__, dtv_reset_gpio, rc);
			return rc;
		}

		/* bring reset line low to hold reset*/
		gpio_set_value_cansleep(37, 0);
	}
#endif

	if (on) {
		rc = msm_gpios_enable(dtv_panel_gpios,
				ARRAY_SIZE(dtv_panel_gpios));
		if (rc < 0) {
			printk(KERN_ERR "%s: gpio enable failed: %d\n",
				__func__, rc);
			return rc;
		}
	} else {
		rc = msm_gpios_disable(dtv_panel_gpios,
				ARRAY_SIZE(dtv_panel_gpios));
		if (rc < 0) {
			printk(KERN_ERR "%s: gpio disable failed: %d\n",
				__func__, rc);
			return rc;
		}
	}

#ifdef HDMI_RESET
	if (on) {
		gpio_set_value_cansleep(37, 1);	/* bring reset line high */
		mdelay(10);		/* 10 msec before IO can be accessed */
	}
#endif
	pr_info("%s: %d <<\n", __func__, on);

	return rc;
}

static struct lcdc_platform_data dtv_pdata = {
	.lcdc_power_save   = dtv_panel_power,
};

static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
       .inject_rx_on_wakeup = 1,
       .rx_to_inject = 0xFD,
};

static struct resource msm_fb_resources[] = {
	{
		.flags  = IORESOURCE_DMA,
	}
};

static int msm_fb_detect_panel(const char *name)
{
#if defined (CONFIG_FB_MSM_LCDC_S6D16A0X22_WVGA_PANEL)
	if (!strcmp(name, "lcdc_s6d16a0x22_wvga"))
		return 0;
	else
#endif

#if defined (CONFIG_FB_MSM_LCDC_SAMSUNG_ANCORA_PANEL)
	if (!strcmp(name, "lcdc_samsung_ancora"))
		return 0;
	else
#endif
		return -ENODEV;
}

static struct msm_fb_platform_data msm_fb_pdata = {
	.detect_client = msm_fb_detect_panel,
	.mddi_prescan = 1,
};

static struct platform_device msm_fb_device = {
	.name   = "msm_fb",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_fb_resources),
	.resource       = msm_fb_resources,
	.dev    = {
		.platform_data = &msm_fb_pdata,
	}
};

static struct platform_device msm_migrate_pages_device = {
	.name   = "msm_migrate_pages",
	.id     = -1,
};

static struct android_pmem_platform_data android_pmem_kernel_ebi1_pdata = {
       .name = PMEM_KERNEL_EBI1_DATA_NAME,
	/* if no allocator_type, defaults to PMEM_ALLOCATORTYPE_BITMAP,
	* the only valid choice at this time. The board structure is
	* set to all zeros by the C runtime initialization and that is now
	* the enum value of PMEM_ALLOCATORTYPE_BITMAP, now forced to 0 in
	* include/linux/android_pmem.h.
	*/
       .cached = 0,
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
       .name = "pmem_adsp",
       .allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
       .cached = 0,
};

static struct android_pmem_platform_data android_pmem_audio_pdata = {
       .name = "pmem_audio",
       .allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
       .cached = 0,
};

static struct platform_device android_pmem_kernel_ebi1_device = {
       .name = "android_pmem",
       .id = 1,
       .dev = { .platform_data = &android_pmem_kernel_ebi1_pdata },
};

static struct platform_device android_pmem_adsp_device = {
       .name = "android_pmem",
       .id = 2,
       .dev = { .platform_data = &android_pmem_adsp_pdata },
};

static struct platform_device android_pmem_audio_device = {
       .name = "android_pmem",
       .id = 4,
       .dev = { .platform_data = &android_pmem_audio_pdata },
};

static struct kgsl_platform_data kgsl_pdata = {
#ifdef CONFIG_MSM_NPA_SYSTEM_BUS
	/* NPA Flow IDs */
	.high_axi_3d = MSM_AXI_FLOW_3D_GPU_HIGH,
	.high_axi_2d = MSM_AXI_FLOW_2D_GPU_HIGH,
#else
	/* AXI rates in KHz */
	.high_axi_3d = 192000,
	.high_axi_2d = 192000,
#endif
	.max_grp2d_freq = 0,
	.min_grp2d_freq = 0,
	.set_grp2d_async = NULL, /* HW workaround, run Z180 SYNC @ 192 MHZ */
	.max_grp3d_freq = 245760000,
	.min_grp3d_freq = 192 * 1000*1000,
	.set_grp3d_async = set_grp3d_async,
	.imem_clk_name = "imem_clk",
	.grp3d_clk_name = "grp_clk",
	.grp3d_pclk_name = "grp_pclk",
#ifdef CONFIG_MSM_KGSL_2D
	.grp2d0_clk_name = "grp_2d_clk",
	.grp2d0_pclk_name = "grp_2d_pclk",
#else
	.grp2d0_clk_name = NULL,
#endif
	.idle_timeout_3d = HZ/20,
	.idle_timeout_2d = HZ/10,
	.nap_allowed = false,
#ifdef CONFIG_KGSL_PER_PROCESS_PAGE_TABLE
	.pt_va_size = SZ_32M,
	/* Maximum of 32 concurrent processes */
	.pt_max_count = 32,
#else
	.pt_va_size = SZ_128M,
	/* We only ever have one pagetable for everybody */
	.pt_max_count = 1,
#endif
};

static struct resource kgsl_resources[] = {
	{
		.name = "kgsl_reg_memory",
		.start = 0xA3500000, /* 3D GRP address */
		.end = 0xA351ffff,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "kgsl_yamato_irq",
		.start = INT_GRP_3D,
		.end = INT_GRP_3D,
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "kgsl_2d0_reg_memory",
		.start = 0xA3900000, /* Z180 base address */
		.end = 0xA3900FFF,
		.flags = IORESOURCE_MEM,
	},
	{
		.name  = "kgsl_2d0_irq",
		.start = INT_GRP_2D,
		.end = INT_GRP_2D,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device msm_device_kgsl = {
	.name = "kgsl",
	.id = -1,
	.num_resources = ARRAY_SIZE(kgsl_resources),
	.resource = kgsl_resources,
	.dev = {
		.platform_data = &kgsl_pdata,
	},
};

#if defined(CONFIG_CRYPTO_DEV_QCRYPTO) || \
		defined(CONFIG_CRYPTO_DEV_QCRYPTO_MODULE) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV_MODULE)

#define QCE_SIZE		0x10000
#define QCE_0_BASE		0xA8400000

#define QCE_HW_KEY_SUPPORT	1

#define QCE_SHARE_CE_RESOURCE	0
#define QCE_CE_SHARED		0

static struct resource qce_resources[] = {
	[0] = {
		.start = QCE_0_BASE,
		.end = QCE_0_BASE + QCE_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.name = "crypto_channels",
		.start = DMOV_CE_IN_CHAN,
		.end = DMOV_CE_OUT_CHAN,
		.flags = IORESOURCE_DMA,
	},
	[2] = {
		.name = "crypto_crci_in",
		.start = DMOV_CE_IN_CRCI,
		.end = DMOV_CE_IN_CRCI,
		.flags = IORESOURCE_DMA,
	},
	[3] = {
		.name = "crypto_crci_out",
		.start = DMOV_CE_OUT_CRCI,
		.end = DMOV_CE_OUT_CRCI,
		.flags = IORESOURCE_DMA,
	},
	[4] = {
		.name = "crypto_crci_hash",
		.start = DMOV_CE_HASH_CRCI,
		.end = DMOV_CE_HASH_CRCI,
		.flags = IORESOURCE_DMA,
	},
};

#endif

#if defined(CONFIG_CRYPTO_DEV_QCRYPTO) || \
		defined(CONFIG_CRYPTO_DEV_QCRYPTO_MODULE)
static struct msm_ce_hw_support qcrypto_ce_hw_suppport = {
	.ce_shared = QCE_CE_SHARED,
	.shared_ce_resource = QCE_SHARE_CE_RESOURCE,
	.hw_key_support = QCE_HW_KEY_SUPPORT,
};
static struct platform_device qcrypto_device = {
	.name		= "qcrypto",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(qce_resources),
	.resource	= qce_resources,
	.dev		= {
		.coherent_dma_mask = DMA_BIT_MASK(32),
		.platform_data = &qcrypto_ce_hw_suppport,
	},
};
#endif

#if defined(CONFIG_CRYPTO_DEV_QCEDEV) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV_MODULE)
static struct msm_ce_hw_support qcedev_ce_hw_suppport = {
	.ce_shared = QCE_CE_SHARED,
	.shared_ce_resource = QCE_SHARE_CE_RESOURCE,
	.hw_key_support = QCE_HW_KEY_SUPPORT,
};
static struct platform_device qcedev_device = {
	.name		= "qce",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(qce_resources),
	.resource	= qce_resources,
	.dev		= {
		.coherent_dma_mask = DMA_BIT_MASK(32),
		.platform_data = &qcedev_ce_hw_suppport,
	},
};
#endif

static struct msm_gpio fluid_vee_reset_gpio[] = {
	{ GPIO_CFG(20, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "vee_reset" },
};

static unsigned char quickvx_mddi_client = 1;

static unsigned quickvx_vlp_gpio =
	GPIO_CFG(97, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL,	GPIO_CFG_2MA);

static struct pm8058_gpio pmic_quickvx_clk_gpio = {
	.direction      = PM_GPIO_DIR_OUT,
	.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
	.output_value   = 1,
	.pull           = PM_GPIO_PULL_NO,
	.vin_sel        = PM_GPIO_VIN_S3,
	.out_strength   = PM_GPIO_STRENGTH_HIGH,
	.function       = PM_GPIO_FUNC_2,
};

static int display_common_power(int on)
{
	int rc = 0, flag_on = !!on;
	static int display_common_power_save_on;
	struct vreg *vreg_ldo15, *vreg_ldo17 = NULL;

	if (display_common_power_save_on == flag_on)
		return 0;

	display_common_power_save_on = flag_on;

	// VREG_LCD_2.8V
	vreg_ldo15 = vreg_get(NULL, "gp6");
	if (IS_ERR(vreg_ldo15)) {
		rc = PTR_ERR(vreg_ldo15);
		pr_err("%s: gp6 vreg get failed (%d)\n",
		       __func__, rc);
		return rc;
	}

	// VREG_LCD_1.8V
	vreg_ldo17 = vreg_get(NULL, "gp11");
	if (IS_ERR(vreg_ldo17)) {
		rc = PTR_ERR(vreg_ldo17);
		pr_err("%s: gp9 vreg get failed (%d)\n",
		       __func__, rc);
		return rc;
	}

	rc = vreg_set_level(vreg_ldo15, 3000);
	if (rc) {
		pr_err("%s: vreg LDO15 set level failed (%d)\n",
		       __func__, rc);
		return rc;
	}

	rc = vreg_set_level(vreg_ldo17, 1800);
	if (rc) {
		pr_err("%s: vreg LDO17 set level failed (%d)\n",
		       __func__, rc);
		return rc;
	}

	if (on)
		rc = vreg_enable(vreg_ldo17);
	else
		rc = vreg_disable(vreg_ldo17);

	if (rc) {
		pr_err("%s: LDO17 vreg enable failed (%d)\n",
		       __func__, rc);
		return rc;
	}

	if (on)
		rc = vreg_enable(vreg_ldo15);
	else
		rc = vreg_disable(vreg_ldo15);

	if (rc) {
		pr_err("%s: LDO15 vreg enable failed (%d)\n",
		       __func__, rc);
		return rc;
	}

	mdelay(5);		/* ensure power is stable */

	return rc;
}

static int msm_fb_mddi_sel_clk(u32 *clk_rate)
{
	*clk_rate *= 2;
	return 0;
}

static int msm_fb_mddi_client_power(u32 client_id)
{
	struct vreg *vreg_ldo20;
	int rc;

	printk(KERN_NOTICE "\n client_id = 0x%x", client_id);
	/* Check if it is Quicklogic client */
	if (client_id == 0xc5835800)
		printk(KERN_NOTICE "\n Quicklogic MDDI client");
	else {
		printk(KERN_NOTICE "\n Non-Quicklogic MDDI client");
		quickvx_mddi_client = 0;
		gpio_set_value(97, 0);
		gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(
			PMIC_GPIO_QUICKVX_CLK), 0);
#if 1
		vreg_ldo20 = vreg_get(NULL, "gp13");

		if (IS_ERR(vreg_ldo20)) {
			rc = PTR_ERR(vreg_ldo20);
			pr_err("%s: gp13 vreg get failed (%d)\n",
				   __func__, rc);
			return rc;
		}
		rc = vreg_set_level(vreg_ldo20, 1500);
		if (rc) {
			pr_err("%s: vreg LDO20 set level failed (%d)\n",
			       __func__, rc);
			return rc;
		}
		rc = vreg_enable(vreg_ldo20);
		if (rc) {
			pr_err("%s: LDO20 vreg enable failed (%d)\n",
			       __func__, rc);
			return rc;
		}
#endif	
	}
	return 0;
}

static struct mddi_platform_data mddi_pdata = {
	.mddi_power_save = display_common_power,
	.mddi_sel_clk = msm_fb_mddi_sel_clk,
	.mddi_client_power = msm_fb_mddi_client_power,
};

int mdp_core_clk_rate_table[] = {
	122880000,
	122880000,
	192000000,
};

static struct msm_panel_common_pdata mdp_pdata = {
	.hw_revision_addr = 0xac001270,
	.gpio = 30,
	.mdp_core_clk_rate = 122880000,
	.mdp_core_clk_table = mdp_core_clk_rate_table,
	.num_mdp_clk = ARRAY_SIZE(mdp_core_clk_rate_table),
};

static struct msm_gpio lcd_panel_on_gpios[] = {
	 GPIO_CFG(18, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	 GPIO_CFG(19, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	 GPIO_CFG(20, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	 GPIO_CFG(21, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	 GPIO_CFG(22, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	 GPIO_CFG(23, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	 GPIO_CFG(24, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	 GPIO_CFG(25, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	 GPIO_CFG(90, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 
	 GPIO_CFG(91, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 
	 GPIO_CFG(92, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	 GPIO_CFG(93, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	 GPIO_CFG(94, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	 GPIO_CFG(95, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	 GPIO_CFG(96, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	 GPIO_CFG(97, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	 GPIO_CFG(98, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	 GPIO_CFG(99, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	 GPIO_CFG(100, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	 GPIO_CFG(101, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	 GPIO_CFG(102, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	 GPIO_CFG(103, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	 GPIO_CFG(104, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	 GPIO_CFG(105, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	 GPIO_CFG(106, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	 GPIO_CFG(107, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	 GPIO_CFG(108, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	 GPIO_CFG(109, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

static struct msm_gpio lcd_panel_off_gpios[] = {
	 GPIO_CFG(18, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	 GPIO_CFG(19, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	 GPIO_CFG(20, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 
	 GPIO_CFG(21, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 
	 GPIO_CFG(22, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 
	 GPIO_CFG(23, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 
	 GPIO_CFG(24, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 
	 GPIO_CFG(25, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 
	 GPIO_CFG(90, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 
	 GPIO_CFG(91, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	 GPIO_CFG(92, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 
	 GPIO_CFG(93, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	 GPIO_CFG(94, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	 GPIO_CFG(95, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	 GPIO_CFG(96, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	 GPIO_CFG(97, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 
	 GPIO_CFG(98, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 
	 GPIO_CFG(99, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 
	 GPIO_CFG(100, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	 GPIO_CFG(101, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 
	 GPIO_CFG(102, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 
	 GPIO_CFG(103, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	 GPIO_CFG(104, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 
	 GPIO_CFG(105, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	 GPIO_CFG(106, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	 GPIO_CFG(107, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	 GPIO_CFG(108, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	 GPIO_CFG(109, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};
static int lcdc_common_panel_power(int on)
{
	int rc, i;
	struct msm_gpio *gp;
	
	rc = display_common_power(on);

	if (rc < 0) {
		printk(KERN_ERR "%s display_common_power failed: %d\n",
				__func__, rc);
		return rc;
	}

	if(on)
	{
		config_gpio_table(lcd_panel_on_gpios,
			ARRAY_SIZE(lcd_panel_on_gpios));

	}
	else
	{
		config_gpio_table(lcd_panel_off_gpios,
			ARRAY_SIZE(lcd_panel_off_gpios));
	
	}
#if 0	
	if (on) {
		rc = msm_gpios_enable(lcd_panel_gpios,
				ARRAY_SIZE(lcd_panel_gpios));
		if (rc < 0) {
			printk(KERN_ERR "%s: gpio enable failed: %d\n",
					__func__, rc);
		}
	} else {	/* off */
		gp = lcd_panel_gpios;
		for (i = 0; i < ARRAY_SIZE(lcd_panel_gpios); i++) {
			/* ouput low */
			gpio_set_value(GPIO_PIN(gp->gpio_cfg), 0);
			gp++;
		}
	}
#endif	
//	printk("######### SEXYKYU lcdc_common_panel_power - ######## \n");
	return rc;
}

static int lcdc_panel_power(int on)
{
	int flag_on = !!on;
	static int lcdc_power_save_on;

//	return 0;	

	if (lcdc_power_save_on == flag_on)
		return 0;

	lcdc_power_save_on = flag_on;

	return lcdc_common_panel_power(on);
}

static struct lcdc_platform_data lcdc_pdata = {
	.lcdc_power_save   = lcdc_panel_power,
};

static int atv_dac_power(int on)
{
	int rc = 0;
	struct vreg *vreg_s4, *vreg_ldo9;

	vreg_s4 = vreg_get(NULL, "s4");
	if (IS_ERR(vreg_s4)) {
		rc = PTR_ERR(vreg_s4);
		pr_err("%s: s4 vreg get failed (%d)\n",
			__func__, rc);
		return -1;
	}
	vreg_ldo9 = vreg_get(NULL, "gp1");
	if (IS_ERR(vreg_ldo9)) {
		rc = PTR_ERR(vreg_ldo9);
		pr_err("%s: ldo9 vreg get failed (%d)\n",
			__func__, rc);
		return rc;
	}

	if (on) {
		rc = vreg_enable(vreg_s4);
		if (rc) {
			pr_err("%s: s4 vreg enable failed (%d)\n",
				__func__, rc);
			return rc;
		}
		rc = vreg_enable(vreg_ldo9);
		if (rc) {
			pr_err("%s: ldo9 vreg enable failed (%d)\n",
				__func__, rc);
			return rc;
		}
	} else {
		rc = vreg_disable(vreg_ldo9);
		if (rc) {
			pr_err("%s: ldo9 vreg disable failed (%d)\n",
				   __func__, rc);
			return rc;
		}
		rc = vreg_disable(vreg_s4);
		if (rc) {
			pr_err("%s: s4 vreg disable failed (%d)\n",
				   __func__, rc);
			return rc;
		}
	}
	return 0;
}

static struct tvenc_platform_data atv_pdata = {
	.poll		 = 1,
	.pm_vid_en	 = atv_dac_power,
};



static void __init msm_fb_add_devices(void)
{
	msm_fb_register_device("mdp", &mdp_pdata);
	msm_fb_register_device("lcdc", &lcdc_pdata);
	//msm_fb_register_device("dtv", &dtv_pdata);
	msm_fb_register_device("tvenc", &atv_pdata);
#ifdef CONFIG_FB_MSM_TVOUT
	msm_fb_register_device("tvout_device", NULL);
#endif
}

#if 0//Disable sc47.yun
#if defined(CONFIG_MARIMBA_CORE) && \
   (defined(CONFIG_MSM_BT_POWER) || defined(CONFIG_MSM_BT_POWER_MODULE))
static struct platform_device msm_bt_power_device = {
	.name = "bt_power",
	.id     = -1
};

enum {
	BT_RFR,
	BT_CTS,
	BT_RX,
	BT_TX,
};

static struct msm_gpio bt_config_power_on[] = {
	{ GPIO_CFG(134, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),
		"UART1DM_RFR" },
	{ GPIO_CFG(135, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),
		"UART1DM_CTS" },
	{ GPIO_CFG(136, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),
		"UART1DM_Rx" },
	{ GPIO_CFG(137, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),
		"UART1DM_Tx" }
};

static struct msm_gpio bt_config_power_off[] = {
	{ GPIO_CFG(134, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,   GPIO_CFG_2MA),
		"UART1DM_RFR" },
	{ GPIO_CFG(135, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,   GPIO_CFG_2MA),
		"UART1DM_CTS" },
	{ GPIO_CFG(136, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,   GPIO_CFG_2MA),
		"UART1DM_Rx" },
	{ GPIO_CFG(137, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,   GPIO_CFG_2MA),
		"UART1DM_Tx" }
};

static struct msm_gpio bt_config_clock[] = {
	{ GPIO_CFG(34, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL,    GPIO_CFG_2MA),
		"BT_REF_CLOCK_ENABLE" },
};

static int marimba_bt(int on)
{
	int rc;
	int i;
	struct marimba config = { .mod_id = MARIMBA_SLAVE_ID_MARIMBA };

	struct marimba_config_register {
		u8 reg;
		u8 value;
		u8 mask;
	};

	struct marimba_variant_register {
		const size_t size;
		const struct marimba_config_register *set;
	};

	const struct marimba_config_register *p;

	u8 version;

	const struct marimba_config_register v10_bt_on[] = {
		{ 0xE5, 0x0B, 0x0F },
		{ 0x05, 0x02, 0x07 },
		{ 0x06, 0x88, 0xFF },
		{ 0xE7, 0x21, 0x21 },
		{ 0xE3, 0x38, 0xFF },
		{ 0xE4, 0x06, 0xFF },
	};

	const struct marimba_config_register v10_bt_off[] = {
		{ 0xE5, 0x0B, 0x0F },
		{ 0x05, 0x08, 0x0F },
		{ 0x06, 0x88, 0xFF },
		{ 0xE7, 0x00, 0x21 },
		{ 0xE3, 0x00, 0xFF },
		{ 0xE4, 0x00, 0xFF },
	};

	const struct marimba_config_register v201_bt_on[] = {
		{ 0x05, 0x08, 0x07 },
		{ 0x06, 0x88, 0xFF },
		{ 0xE7, 0x21, 0x21 },
		{ 0xE3, 0x38, 0xFF },
		{ 0xE4, 0x06, 0xFF },
	};

	const struct marimba_config_register v201_bt_off[] = {
		{ 0x05, 0x08, 0x07 },
		{ 0x06, 0x88, 0xFF },
		{ 0xE7, 0x00, 0x21 },
		{ 0xE3, 0x00, 0xFF },
		{ 0xE4, 0x00, 0xFF },
	};

	const struct marimba_config_register v210_bt_on[] = {
		{ 0xE9, 0x01, 0x01 },
		{ 0x06, 0x88, 0xFF },
		{ 0xE7, 0x21, 0x21 },
		{ 0xE3, 0x38, 0xFF },
		{ 0xE4, 0x06, 0xFF },
	};

	const struct marimba_config_register v210_bt_off[] = {
		{ 0x06, 0x88, 0xFF },
		{ 0xE7, 0x00, 0x21 },
		{ 0xE9, 0x00, 0x01 },
		{ 0xE3, 0x00, 0xFF },
		{ 0xE4, 0x00, 0xFF },
	};

	const struct marimba_variant_register bt_marimba[2][4] = {
		{
			{ ARRAY_SIZE(v10_bt_off), v10_bt_off },
			{ 0, NULL },
			{ ARRAY_SIZE(v201_bt_off), v201_bt_off },
			{ ARRAY_SIZE(v210_bt_off), v210_bt_off }
		},
		{
			{ ARRAY_SIZE(v10_bt_on), v10_bt_on },
			{ 0, NULL },
			{ ARRAY_SIZE(v201_bt_on), v201_bt_on },
			{ ARRAY_SIZE(v210_bt_on), v210_bt_on }
		}
	};

	on = on ? 1 : 0;

	rc = marimba_read_bit_mask(&config, 0x11,  &version, 1, 0x1F);
	if (rc < 0) {
		printk(KERN_ERR
			"%s: version read failed: %d\n",
			__func__, rc);
		return rc;
	}

	if ((version >= ARRAY_SIZE(bt_marimba[on])) ||
	    (bt_marimba[on][version].size == 0)) {
		printk(KERN_ERR
			"%s: unsupported version\n",
			__func__);
		return -EIO;
	}

	p = bt_marimba[on][version].set;

	printk(KERN_INFO "%s: found version %d\n", __func__, version);

	for (i = 0; i < bt_marimba[on][version].size; i++) {
		u8 value = (p+i)->value;
		rc = marimba_write_bit_mask(&config,
			(p+i)->reg,
			&value,
			sizeof((p+i)->value),
			(p+i)->mask);
		if (rc < 0) {
			printk(KERN_ERR
				"%s: reg %d write failed: %d\n",
				__func__, (p+i)->reg, rc);
			return rc;
		}
		printk(KERN_INFO "%s: reg 0x%02x value 0x%02x mask 0x%02x\n",
				__func__, (p+i)->reg,
				value, (p+i)->mask);
	}
	return 0;
}

static const char *vregs_bt_name[] = {
	"gp16",
	"s2",
	"wlan"
};
static struct vreg *vregs_bt[ARRAY_SIZE(vregs_bt_name)];

static int bluetooth_power_regulators(int on)
{
	int i, rc;

	for (i = 0; i < ARRAY_SIZE(vregs_bt_name); i++) {
		rc = on ? vreg_enable(vregs_bt[i]) :
			  vreg_disable(vregs_bt[i]);
		if (rc < 0) {
			printk(KERN_ERR "%s: vreg %s %s failed (%d)\n",
				__func__, vregs_bt_name[i],
			       on ? "enable" : "disable", rc);
			return -EIO;
		}
	}
	return 0;
	}

static int bluetooth_power(int on)
{
	int rc;
	const char *id = "BTPW";

	if (on) {
		rc = pmapp_vreg_level_vote(id, PMAPP_VREG_S2, 1300);
		if (rc < 0) {
			printk(KERN_ERR "%s: vreg level on failed (%d)\n",
				__func__, rc);
			return rc;
		}

		rc = bluetooth_power_regulators(on);
		if (rc < 0)
			return -EIO;

		rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_DO,
					  PMAPP_CLOCK_VOTE_ON);
		if (rc < 0)
			return -EIO;

		gpio_set_value_cansleep(GPIO_PIN(bt_config_clock->gpio_cfg), 1);

		rc = marimba_bt(on);
		if (rc < 0)
			return -EIO;

		msleep(10);

		rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_DO,
					  PMAPP_CLOCK_VOTE_PIN_CTRL);
		if (rc < 0)
			return -EIO;

		gpio_set_value_cansleep(GPIO_PIN(bt_config_clock->gpio_cfg), 0);

		rc = msm_gpios_enable(bt_config_power_on,
			ARRAY_SIZE(bt_config_power_on));

		if (rc < 0)
			return rc;

	} else {
		rc = msm_gpios_enable(bt_config_power_off,
					ARRAY_SIZE(bt_config_power_off));
		if (rc < 0)
			return rc;

		/* check for initial RFKILL block (power off) */
		if (platform_get_drvdata(&msm_bt_power_device) == NULL)
			goto out;

		rc = marimba_bt(on);
		if (rc < 0)
			return -EIO;

		//gpio_set_value_cansleep(GPIO_PIN(bt_config_clock->gpio_cfg), 0);

		rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_DO,
					  PMAPP_CLOCK_VOTE_OFF);
		if (rc < 0)
			return -EIO;

		rc = bluetooth_power_regulators(on);
		if (rc < 0)
			return -EIO;

		rc = pmapp_vreg_level_vote(id, PMAPP_VREG_S2, 0);
		if (rc < 0) {
			printk(KERN_INFO "%s: vreg level off failed (%d)\n",
				__func__, rc);
		}
	}

out:
	printk(KERN_DEBUG "Bluetooth power switch: %d\n", on);

	return 0;
}

static void __init bt_power_init(void)
{
	int i, rc;

	for (i = 0; i < ARRAY_SIZE(vregs_bt_name); i++) {
		vregs_bt[i] = vreg_get(NULL, vregs_bt_name[i]);
		if (IS_ERR(vregs_bt[i])) {
			printk(KERN_ERR "%s: vreg get %s failed (%ld)\n",
			       __func__, vregs_bt_name[i],
			       PTR_ERR(vregs_bt[i]));
			return;
		}
	}

	rc = msm_gpios_request_enable(bt_config_clock,
				ARRAY_SIZE(bt_config_clock));
	if (rc < 0) {
		printk(KERN_ERR
			"%s: msm_gpios_request_enable failed (%d)\n",
				__func__, rc);
		return;
	}

	rc = gpio_direction_output(GPIO_PIN(bt_config_clock->gpio_cfg),
					0);
	if (rc < 0) {
		printk(KERN_ERR
			"%s: gpio_direction_output failed (%d)\n",
				__func__, rc);
		return;
	}

	msm_bt_power_device.dev.platform_data = &bluetooth_power;
}
#else
#define bt_power_init(x) do {} while (0)
#endif
#else //sc47.yun. Power or Sleep at Bluetooth 
static struct resource bluesleep_resources[] = {
{
    .name = "gpio_host_wake",
    .start = GPIO_BT_HOST_WAKE,
    .end = GPIO_BT_HOST_WAKE,
    .flags = IORESOURCE_IO,
    },
    {
    .name = "gpio_ext_wake",
    .start = GPIO_BT_WAKE,//81,//35,
    .end = GPIO_BT_WAKE,//81, //35,
    .flags = IORESOURCE_IO,
    },
    {
    .name = "host_wake",
    .start = MSM_GPIO_TO_INT(GPIO_BT_HOST_WAKE),
    .end = MSM_GPIO_TO_INT(GPIO_BT_HOST_WAKE),
    .flags = IORESOURCE_IRQ,
    },
};

static struct platform_device msm_bluesleep_device = {
    .name = "bluesleep",
    .id = -1,
    .num_resources = ARRAY_SIZE(bluesleep_resources),
    .resource = bluesleep_resources,
};


extern int bluesleep_start(void);
extern void bluesleep_stop(void);
static struct platform_device msm_bt_power_device = {
.name = "bt_power",
//sc47.yun .id     = -1
};

static unsigned bt_config_power_on[] = {
    GPIO_CFG(GPIO_BT_WAKE,     0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),    /* WAKE */
    GPIO_CFG(GPIO_BT_UART_RTS, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),    /* RFR */
    GPIO_CFG(GPIO_BT_UART_CTS, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),    /* CTS */
    GPIO_CFG(GPIO_BT_UART_RXD, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),    /* Rx */
    GPIO_CFG(GPIO_BT_UART_TXD, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),    /* Tx */
    GPIO_CFG(GPIO_BT_PCM_DOUT, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),    /* PCM_DOUT */
    GPIO_CFG(GPIO_BT_PCM_DIN,  1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),    /* PCM_DIN */
    GPIO_CFG(GPIO_BT_PCM_SYNC, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),    /* PCM_SYNC */
    GPIO_CFG(GPIO_BT_PCM_CLK,  1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),    /* PCM_CLK */
    GPIO_CFG(GPIO_BT_HOST_WAKE,0, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),    /* HOST_WAKE */
};

static unsigned bt_config_power_off[] = {
    GPIO_CFG(GPIO_BT_WAKE,     0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),    /* WAKE */
    GPIO_CFG(GPIO_BT_UART_RTS, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),    /* RFR */
    GPIO_CFG(GPIO_BT_UART_CTS, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),    /* CTS */
    GPIO_CFG(GPIO_BT_UART_RXD, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),    /* Rx */
    GPIO_CFG(GPIO_BT_UART_TXD, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),    /* Tx */
    GPIO_CFG(GPIO_BT_PCM_DOUT, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),    /* PCM_DOUT */
    GPIO_CFG(GPIO_BT_PCM_DIN,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),    /* PCM_DIN */
    GPIO_CFG(GPIO_BT_PCM_SYNC, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),    /* PCM_SYNC */
    GPIO_CFG(GPIO_BT_PCM_CLK,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),    /* PCM_CLK */
    GPIO_CFG(GPIO_BT_HOST_WAKE,0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),    /* HOST_WAKE */
};

static int bluetooth_power(int on)
{
    struct vreg *vreg_bt;
    int pin, rc;

    pr_info("bluetooth_power \n");

    printk(KERN_DEBUG "%s\n", __func__);

    if (on) {
        config_gpio_table(bt_config_power_on, ARRAY_SIZE(bt_config_power_on));
        pr_info("bluetooth_power BT_WAKE:%d, HOST_WAKE:%d, REG_ON:%d\n", gpio_get_value(GPIO_BT_WAKE), gpio_get_value(GPIO_BT_HOST_WAKE), gpio_get_value(GPIO_BT_WLAN_REG_ON));
        
        gpio_direction_output(GPIO_BT_WAKE, GPIO_WLAN_LEVEL_HIGH);
        gpio_direction_output(GPIO_BT_WLAN_REG_ON, GPIO_WLAN_LEVEL_HIGH);
//        mdelay(150);
        usleep(150000);//sc47.yun
        gpio_direction_output(GPIO_BT_RESET, GPIO_WLAN_LEVEL_HIGH);

        pr_info("bluetooth_power BT_WAKE:%d, HOST_WAKE:%d, REG_ON:%d\n", gpio_get_value(GPIO_BT_WAKE), gpio_get_value(GPIO_BT_HOST_WAKE), gpio_get_value(GPIO_BT_WLAN_REG_ON));   
//sc47.yun        mdelay(100);   
     
        bluesleep_start();
    } 
    else {    
        bluesleep_stop();
        gpio_direction_output(GPIO_BT_RESET, GPIO_WLAN_LEVEL_LOW);/* BT_VREG_CTL */

        if( gpio_get_value(GPIO_WLAN_RESET) == GPIO_WLAN_LEVEL_LOW ) //SEC_BLUETOOTH : pjh_2010.06.30
        {
            gpio_direction_output(GPIO_BT_WLAN_REG_ON, GPIO_WLAN_LEVEL_LOW);/* BT_RESET */
            mdelay(150);
        }
        gpio_direction_output(GPIO_BT_WAKE, GPIO_WLAN_LEVEL_LOW);/* BT_VREG_CTL */

        config_gpio_table(bt_config_power_off, ARRAY_SIZE(bt_config_power_off));
    }
    return 0;
}

static void __init bt_power_init(void)
{
    pr_info("bt_power_init \n");

    msm_bt_power_device.dev.platform_data = &bluetooth_power;
}
//sc47.yun
static int bluetooth_gpio_init(void)
{
    pr_info("bluetooth_gpio_init on system_rev:%d\n", system_rev);

    config_gpio_table(bt_config_power_on, ARRAY_SIZE(bt_config_power_on));
    return 0;
}
//sc47.yun
#endif//Finish Power or Sleep at Bluetooth sc47.yun
static struct msm_psy_batt_pdata msm_psy_batt_data = {
	.voltage_min_design 	= 3400,
	.voltage_max_design	= 4200,
	.avail_chg_sources   	= AC_CHG | USB_CHG ,
	.batt_technology        = POWER_SUPPLY_TECHNOLOGY_LION,
};

static struct platform_device ancora_batt_device = {
	.name 		    = "ancora-battery",
	.id		    = -1,
	.dev.platform_data  = &msm_psy_batt_data,
};

static char *msm_adc_fluid_device_names[] = {
	"LTC_ADC1",
	"LTC_ADC2",
	"LTC_ADC3",
};

static char *msm_adc_surf_device_names[] = {
	"XO_ADC",
};

static struct msm_adc_platform_data msm_adc_pdata;

static struct platform_device msm_adc_device = {
	.name   = "msm_adc",
	.id = -1,
	.dev = {
		.platform_data = &msm_adc_pdata,
	},
};

static void ancora_switch_init(void)
{
	sec_class = class_create(THIS_MODULE, "sec");
	
	if (IS_ERR(sec_class))
		pr_err("Failed to create class(sec)!\n");

	switch_dev = device_create(sec_class, NULL, 0, NULL, "switch");
	
	if (IS_ERR(switch_dev))
		pr_err("Failed to create device(switch)!\n");

};

#ifdef CONFIG_MSM_SDIO_AL
static struct msm_gpio mdm2ap_status = {
	GPIO_CFG(77, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	"mdm2ap_status"
};


static int configure_mdm2ap_status(int on)
{
	if (on)
		return msm_gpios_request_enable(&mdm2ap_status, 1);
	else {
		msm_gpios_disable_free(&mdm2ap_status, 1);
		return 0;
	}
}

static int get_mdm2ap_status(void)
{
	return gpio_get_value(GPIO_PIN(mdm2ap_status.gpio_cfg));
}

static struct sdio_al_platform_data sdio_al_pdata = {
	.config_mdm2ap_status = configure_mdm2ap_status,
	.get_mdm2ap_status = get_mdm2ap_status,
	.allow_sdioc_version_major_2 = 1,
	.peer_sdioc_version_minor = 0x0001,
	.peer_sdioc_version_major = 0x0003,
	.peer_sdioc_boot_version_minor = 0x0001,
	.peer_sdioc_boot_version_major = 0x0002,	
};

struct platform_device msm_device_sdio_al = {
	.name = "msm_sdio_al",
	.id = -1,
	.dev		= {
		.platform_data	= &sdio_al_pdata,
	},
};

#endif /* CONFIG_MSM_SDIO_AL */

static struct platform_device *uart3_device[] __initdata = {
	&msm_device_uart3,
};

static struct platform_device *devices[] __initdata = {
	&msm_device_smd,
	&msm_device_dmov,
	&smc91x_device,
	&smsc911x_device,
	&msm_device_nand,
#ifdef CONFIG_USB_FUNCTION
	&msm_device_hsusb_peripheral,
	&mass_storage_device,
#endif
#ifdef CONFIG_USB_MSM_OTG_72K
	&msm_device_otg,
#ifdef CONFIG_USB_GADGET
	&msm_device_gadget_peripheral,
#endif
#endif
#ifdef CONFIG_USB_ANDROID

	&mass_storage_device,
	&rndis_device,
	&usb_diag_device,
	&android_usb_device,

#endif
#ifdef CONFIG_I2C_SSBI
	&msm_device_ssbi6,
	&msm_device_ssbi7,
#endif
	&android_pmem_device,
	&msm_fb_device,
	&msm_migrate_pages_device,
#ifdef CONFIG_MSM_ROTATOR
	&msm_rotator_device,
#endif
#ifdef CONFIG_FB_MSM_LCDC_S6D16A0X22_WVGA_PANEL
	&lcdc_s6d16a0x22_panel_device,
#endif
#ifdef CONFIG_FB_MSM_LCDC_SAMSUNG_ANCORA_PANEL
	&lcdc_samsung_panel_device,
#endif
	&android_pmem_kernel_ebi1_device,
	&android_pmem_adsp_device,
	&android_pmem_audio_device,
	&msm_device_i2c,
	&msm_device_i2c_2,
	&msm_device_uart_dm1,
	&hs_device,
	&amp_i2c_gpio_device,
#ifdef CONFIG_SAMSUNG_FM_SI4709
	&fmradio_i2c_gpio_device,
#endif
#ifdef CONFIG_MSM7KV2_AUDIO
	&msm_aictl_device,
	&msm_mi2s_device,
	&msm_lpa_device,
	&msm_aux_pcm_device,
#endif
	&msm_device_adspdec,
#if defined (CONFIG_USE_QUP_I2C)
	&qup_device_i2c,
#endif
#if defined (CONFIG_TOUCHSCREEN_QT602240)
	&touchscreen_i2c_gpio_device,
#endif
#ifdef CONFIG_TOUCHSCREEN_MELFAS_MCS8000
	&touch_i2c_gpio_device,
#endif
#ifdef CONFIG_KEYPAD_CYPRESS_TOUCH
	&touch_keypad_i2c_device,
#endif
#if 1
	&micro_usb_i2c_gpio_device,
#endif
#if defined (CONFIG_SENSOR_MC7) || defined (CONFIG_SENSOR_CE147) || defined (CONFIG_SENSOR_S5K4ECGX) || defined (CONFIG_SENSOR_S5K5CCAF)
	&cam_pm_lp8720_i2c_device,
#if !defined (CONFIG_USE_QUP_I2C)
	&camera_i2c_gpio_device,
#endif
#endif
#ifdef CONFIG_OPTICAL_GP2A
	&opt_i2c_gpio_device,
	&opt_gp2a,
#endif
#ifdef CONFIG_SENSORS_BMA023_ACCEL
	&acc_i2c_gpio_device,
#endif
#ifdef CONFIG_GYRO_L3G4200D
	&gyro_i2c_gpio_device,
#endif
#ifdef CONFIG_SENSORS_YAS529_MAGNETIC
	&mag_i2c_gpio_device,
#endif
#ifdef CONFIG_SENSORS_AK8975
	&mag_i2c_gpio_device,
#endif
/* 2011-01-27 hyeokseon.yu */
#ifdef CONFIG_MAX17043_FUEL_GAUGE 
	&fuelgauge_i2c_gpio_device,
#endif

/* 2011-06-20 hyeokseon.yu */
#if defined(CONFIG_CHARGER_SMB328A)
	&fg_smb_i2c_gpio_device,
#endif

#if defined(CONFIG_MARIMBA_CORE) && \
   (defined(CONFIG_MSM_BT_POWER) || defined(CONFIG_MSM_BT_POWER_MODULE))
//sc47.yun    &msm_bt_power_device,
#endif
	&msm_bt_power_device,//sc47.yun
	&msm_bluesleep_device, //sc47.yun
	&msm_device_kgsl,
#if defined (CONFIG_SENSOR_S5K4ECGX)
	&msm_camera_sensor_s5k4ecgx,
#endif
#if defined (CONFIG_SENSOR_MC7)
	&msm_camera_sensor_mc7,
#endif	
#if defined (CONFIG_SENSOR_CE147)
	&msm_camera_sensor_ce147,
#endif	
#ifdef CONFIG_SENSOR_S5KA3DFX
	&msm_camera_sensor_s5ka3dfx,
#endif
#ifdef CONFIG_SENSOR_SR030PC30
	&msm_camera_sensor_sr030pc30,
#endif
#ifdef CONFIG_MT9T013
	&msm_camera_sensor_mt9t013,
#endif
#ifdef CONFIG_MT9D112
	&msm_camera_sensor_mt9d112,
#endif
#ifdef CONFIG_S5K3E2FX
	&msm_camera_sensor_s5k3e2fx,
#endif
#ifdef CONFIG_MT9P012
	&msm_camera_sensor_mt9p012,
#endif
#ifdef CONFIG_VX6953
	&msm_camera_sensor_vx6953,
#endif
#ifdef CONFIG_SN12M0PZ
	&msm_camera_sensor_sn12m0pz,
#endif
#ifdef CONFIG_SENSOR_S5K5CCAF //PCAM			
	&msm_camera_sensor_s5k5ccaf,
#endif
	&msm_device_vidc_720p,
#ifdef CONFIG_MSM_GEMINI
	&msm_gemini_device,
#endif
#ifdef CONFIG_MSM_VPE
	&msm_vpe_device,
#endif
#if defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE)
	&msm_device_tsif,
#endif
#ifdef CONFIG_MSM_SDIO_AL
	&msm_device_sdio_al,
#endif
	&msm_vibrator_device,

#if defined(CONFIG_CRYPTO_DEV_QCRYPTO) || \
		defined(CONFIG_CRYPTO_DEV_QCRYPTO_MODULE)
	&qcrypto_device,
#endif

#if defined(CONFIG_CRYPTO_DEV_QCEDEV) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV_MODULE)
	&qcedev_device,
#endif

	&ancora_batt_device,
	&msm_adc_device,
	&msm_ebi0_thermal,
	&msm_ebi1_thermal,
#ifdef CONFIG_SAMSUNG_JACK
	&sec_device_jack,
#endif

#ifdef CONFIG_INPUT_SECBRIDGE
    &ancora_input_bridge,
#endif
};


static struct msm_gpio msm_i2c_gpios_hw[] = {
//	{ GPIO_CFG(70, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "i2c_scl" },
//	{ GPIO_CFG(71, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "i2c_sda" },
};

static struct msm_gpio msm_i2c_gpios_io[] = {
//	{ GPIO_CFG(70, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "i2c_scl" },
//	{ GPIO_CFG(71, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "i2c_sda" },
};


static struct msm_gpio qup_i2c_gpios_io[] = {
	{ GPIO_CFG(0, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "qup_scl" },
	{ GPIO_CFG(1, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "qup_sda" },
};
static struct msm_gpio qup_i2c_gpios_hw[] = {
	{ GPIO_CFG(0, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "qup_scl" },
	{ GPIO_CFG(1, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "qup_sda" },
};

static void
msm_i2c_gpio_config(int adap_id, int config_type)
{
	struct msm_gpio *msm_i2c_table;

	/* Each adapter gets 2 lines from the table */
	if (adap_id > 0)
		return;
	if (config_type)
		msm_i2c_table = &msm_i2c_gpios_hw[adap_id*2];
	else
		msm_i2c_table = &msm_i2c_gpios_io[adap_id*2];
	msm_gpios_enable(msm_i2c_table, 2);
}
/*This needs to be enabled only for OEMS*/
#ifndef CONFIG_QUP_EXCLUSIVE_TO_CAMERA
static struct vreg *qup_vreg;
#endif
static void
qup_i2c_gpio_config(int adap_id, int config_type)
{
	int rc = 0;
	struct msm_gpio *qup_i2c_table;
	/* Each adapter gets 2 lines from the table */
	if (adap_id != 4)
		return;
	if (config_type)
		qup_i2c_table = qup_i2c_gpios_hw;
	else
		qup_i2c_table = qup_i2c_gpios_io;
	rc = msm_gpios_enable(qup_i2c_table, 2);
	if (rc < 0)
		printk(KERN_ERR "QUP GPIO enable failed: %d\n", rc);
	/*This needs to be enabled only for OEMS*/
#ifndef CONFIG_QUP_EXCLUSIVE_TO_CAMERA
	if (qup_vreg) {
		int rc = vreg_set_level(qup_vreg, 1800);
		if (rc) {
			pr_err("%s: vreg LVS1 set level failed (%d)\n",
			__func__, rc);
		}
/*
		rc = vreg_enable(qup_vreg);
		if (rc) {
			pr_err("%s: vreg_enable() = %d \n",
			__func__, rc);
		}
*/		
	}
#endif
}

static struct msm_i2c_platform_data msm_i2c_pdata = {
	.clk_freq = 100000,
	.pri_clk = 70,
	.pri_dat = 71,
	.rmutex  = 1,
	.rsl_id = "D:I2C02000021",
	.msm_i2c_config_gpio = msm_i2c_gpio_config,
};

static void __init msm_device_i2c_init(void)
{
	if (msm_gpios_request(msm_i2c_gpios_hw, ARRAY_SIZE(msm_i2c_gpios_hw)))
		pr_err("failed to request I2C gpios\n");

	msm_device_i2c.dev.platform_data = &msm_i2c_pdata;
}

static struct msm_i2c_platform_data msm_i2c_2_pdata = {
	.clk_freq = 100000,
	.rmutex  = 1,
	.rsl_id = "D:I2C02000022",
	.msm_i2c_config_gpio = msm_i2c_gpio_config,
};

static void __init msm_device_i2c_2_init(void)
{
	msm_device_i2c_2.dev.platform_data = &msm_i2c_2_pdata;
}

static struct msm_i2c_platform_data qup_i2c_pdata = {
	.clk_freq = 200000,
	.pclk = "camif_pad_pclk",
	.msm_i2c_config_gpio = qup_i2c_gpio_config,
};

static void __init qup_device_i2c_init(void)
{
	if (msm_gpios_request(qup_i2c_gpios_hw, ARRAY_SIZE(qup_i2c_gpios_hw)))
		pr_err("failed to request I2C gpios\n");

	qup_device_i2c.dev.platform_data = &qup_i2c_pdata;
	/*This needs to be enabled only for OEMS*/
#ifndef CONFIG_QUP_EXCLUSIVE_TO_CAMERA
	qup_vreg = vreg_get(NULL, "gp13");
	if (IS_ERR(qup_vreg)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
			__func__, PTR_ERR(qup_vreg));
	}
#endif
}

#ifdef CONFIG_I2C_SSBI
static struct msm_ssbi_platform_data msm_i2c_ssbi6_pdata = {
	.rsl_id = "D:PMIC_SSBI",
	.controller_type = MSM_SBI_CTRL_SSBI2,
};

static struct msm_ssbi_platform_data msm_i2c_ssbi7_pdata = {
	.rsl_id = "D:CODEC_SSBI",
	.controller_type = MSM_SBI_CTRL_SSBI,
};
#endif

static struct msm_acpu_clock_platform_data msm7x30_clock_data = {
	.acpu_switch_time_us = 50,
	.vdd_switch_time_us = 62,
};

static void __init msm7x30_init_irq(void)
{
	msm_init_irq();
}

static struct msm_gpio msm_nand_ebi2_cfg_data[] = {
	{GPIO_CFG(86, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "ebi2_cs1"},
	{GPIO_CFG(115, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "ebi2_busy1"},
};

struct vreg *vreg_s3;
struct vreg *vreg_mmc;

#if (defined(CONFIG_MMC_MSM_SDC1_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC2_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC3_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC4_SUPPORT))

struct sdcc_gpio {
	struct msm_gpio *cfg_data;
	uint32_t size;
	struct msm_gpio *sleep_cfg_data;
};
#if defined(CONFIG_MMC_MSM_SDC1_SUPPORT)
static struct msm_gpio sdc1_lvlshft_cfg_data[] = {
	{GPIO_CFG(35, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA), "sdc1_lvlshft"},
};
#endif
static struct msm_gpio sdc1_cfg_data[] = {
	{GPIO_CFG(38, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "sdc1_clk"},
	{GPIO_CFG(39, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc1_cmd"},
	{GPIO_CFG(40, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc1_dat_3"},
	{GPIO_CFG(41, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc1_dat_2"},
	{GPIO_CFG(42, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc1_dat_1"},
	{GPIO_CFG(43, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc1_dat_0"},
};

static struct msm_gpio sdc2_cfg_data[] = {
	{GPIO_CFG(64, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "sdc2_clk"},
	{GPIO_CFG(65, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_cmd"},
	{GPIO_CFG(66, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_3"},
	{GPIO_CFG(67, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_2"},
	{GPIO_CFG(68, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_1"},
	{GPIO_CFG(69, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_0"},

#ifdef CONFIG_MMC_MSM_SDC2_8_BIT_SUPPORT
	{GPIO_CFG(115, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_4"},
	{GPIO_CFG(114, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_5"},
	{GPIO_CFG(113, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_6"},
	{GPIO_CFG(112, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_7"},
#endif
};

static struct msm_gpio sdc3_cfg_data[] = {
	{GPIO_CFG(110, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "sdc3_clk"},
	{GPIO_CFG(111, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_cmd"},
	//{GPIO_CFG(116, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_dat_3"},
	{GPIO_CFG(117, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_dat_2"},
	{GPIO_CFG(118, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_dat_1"},
	{GPIO_CFG(119, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_dat_0"},
};

static struct msm_gpio sdc3_sleep_cfg_data[] = {
	{GPIO_CFG(110, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
			"sdc3_clk"},
	{GPIO_CFG(111, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
			"sdc3_cmd"},
	//{GPIO_CFG(116, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	//		"sdc3_dat_3"},
	{GPIO_CFG(117, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
			"sdc3_dat_2"},
	{GPIO_CFG(118, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
			"sdc3_dat_1"},
	{GPIO_CFG(119, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
			"sdc3_dat_0"},
};

static struct msm_gpio sdc4_cfg_data[] = {
	{GPIO_CFG(58, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "sdc4_clk"},
	{GPIO_CFG(59, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "sdc4_cmd"},
	{GPIO_CFG(60, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "sdc4_dat_3"},
	{GPIO_CFG(61, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "sdc4_dat_2"},
	{GPIO_CFG(62, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "sdc4_dat_1"},
	{GPIO_CFG(63, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "sdc4_dat_0"},
};

static struct sdcc_gpio sdcc_cfg_data[] = {
	{
		.cfg_data = sdc1_cfg_data,
		.size = ARRAY_SIZE(sdc1_cfg_data),
		.sleep_cfg_data = NULL,
	},
	{
		.cfg_data = sdc2_cfg_data,
		.size = ARRAY_SIZE(sdc2_cfg_data),
		.sleep_cfg_data = NULL,
	},
	{
		.cfg_data = sdc3_cfg_data,
		.size = ARRAY_SIZE(sdc3_cfg_data),
		.sleep_cfg_data = sdc3_sleep_cfg_data,
	},
	{
		.cfg_data = sdc4_cfg_data,
		.size = ARRAY_SIZE(sdc4_cfg_data),
		.sleep_cfg_data = NULL,
	},
};

struct sdcc_vreg {
	struct vreg *vreg_data;
	unsigned level;
};

static struct sdcc_vreg sdcc_vreg_data[4];

static unsigned long vreg_sts, gpio_sts;

static uint32_t msm_sdcc_setup_gpio(int dev_id, unsigned int enable)
{
	int rc = 0;
	struct sdcc_gpio *curr;

	if((dev_id == 1)&& (gpio_get_value(GPIO_WLAN_RESET)))
	{
		return 0;
	}
	
	curr = &sdcc_cfg_data[dev_id - 1];

	if (!(test_bit(dev_id, &gpio_sts)^enable))
		return rc;

	if (enable) {
		set_bit(dev_id, &gpio_sts);
		rc = msm_gpios_request_enable(curr->cfg_data, curr->size);
		if (rc)
			printk(KERN_ERR "%s: Failed to turn on GPIOs for slot %d\n",
				__func__,  dev_id);
	} else {
		clear_bit(dev_id, &gpio_sts);
		if (curr->sleep_cfg_data) {
			msm_gpios_enable(curr->sleep_cfg_data, curr->size);
			msm_gpios_free(curr->sleep_cfg_data, curr->size);
		} else {
			msm_gpios_free(curr->cfg_data, curr->size);
		}
	}

	return rc;
}

static uint32_t msm_sdcc_setup_vreg(int dev_id, unsigned int enable)
{
	int rc = 0;
	struct sdcc_vreg *curr;
	static int enabled_once[] = {0, 0, 0, 0};

	curr = &sdcc_vreg_data[dev_id - 1];

	if (!(test_bit(dev_id, &vreg_sts)^enable))
		return rc;

	if (!enable || enabled_once[dev_id - 1])
		return 0;

	if (enable) {
		set_bit(dev_id, &vreg_sts);
		rc = vreg_set_level(curr->vreg_data, curr->level);
		if (rc) {
			printk(KERN_ERR "%s: vreg_set_level() = %d \n",
					__func__, rc);
		}
		rc = vreg_enable(curr->vreg_data);
		if (rc) {
			printk(KERN_ERR "%s: vreg_enable() = %d \n",
					__func__, rc);
		}
		enabled_once[dev_id - 1] = 1;
	} else {
		clear_bit(dev_id, &vreg_sts);
		rc = vreg_disable(curr->vreg_data);
		if (rc) {
			printk(KERN_ERR "%s: vreg_disable() = %d \n",
					__func__, rc);
		}
	}
	return rc;
}

static uint32_t msm_sdcc_setup_power(struct device *dv, unsigned int vdd)
{
	int rc = 0;
	struct platform_device *pdev;

	pdev = container_of(dv, struct platform_device, dev);
	rc = msm_sdcc_setup_gpio(pdev->id, (vdd ? 1 : 0));
	if (rc)
		goto out;

	if (pdev->id == 4) /* S3 is always ON and cannot be disabled */
		rc = msm_sdcc_setup_vreg(pdev->id, (vdd ? 1 : 0));
out:
	return rc;
}

#if defined(CONFIG_MMC_MSM_SDC1_SUPPORT) && \
	defined(CONFIG_CSDIO_VENDOR_ID) && \
	defined(CONFIG_CSDIO_DEVICE_ID) && \
	(CONFIG_CSDIO_VENDOR_ID == 0x70 && CONFIG_CSDIO_DEVICE_ID == 0x1117)

#define MBP_ON  1
#define MBP_OFF 0

#define MBP_RESET_N \
	GPIO_CFG(44, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA)
#define MBP_INT0 \
	GPIO_CFG(46, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA)

#define MBP_MODE_CTRL_0 \
	GPIO_CFG(35, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA)
#define MBP_MODE_CTRL_1 \
	GPIO_CFG(36, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA)
#define MBP_MODE_CTRL_2 \
	GPIO_CFG(34, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA)
#define TSIF_EN \
	GPIO_CFG(35, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN,	GPIO_CFG_2MA)
#define TSIF_DATA \
	GPIO_CFG(36, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN,	GPIO_CFG_2MA)
#define TSIF_CLK \
	GPIO_CFG(34, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)

static struct msm_gpio mbp_cfg_data[] = {
	{GPIO_CFG(44, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
		"mbp_reset"},
	{GPIO_CFG(85, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
		"mbp_io_voltage"},
};

static int mbp_config_gpios_pre_init(int enable)
{
	int rc = 0;

	if (enable) {
		rc = msm_gpios_request_enable(mbp_cfg_data,
			ARRAY_SIZE(mbp_cfg_data));
		if (rc) {
			printk(KERN_ERR
				"%s: Failed to turnon GPIOs for mbp chip(%d)\n",
				__func__, rc);
		}
	} else
		msm_gpios_disable_free(mbp_cfg_data, ARRAY_SIZE(mbp_cfg_data));
	return rc;
}

static int mbp_setup_rf_vregs(int state)
{
	struct vreg *vreg_rf = NULL;
	struct vreg *vreg_rf_switch	= NULL;
	int rc;

	vreg_rf = vreg_get(NULL, "s2");
	if (IS_ERR(vreg_rf)) {
		pr_err("%s: s2 vreg get failed (%ld)",
				__func__, PTR_ERR(vreg_rf));
		return -EFAULT;
	}
	vreg_rf_switch = vreg_get(NULL, "rf");
	if (IS_ERR(vreg_rf_switch)) {
		pr_err("%s: rf vreg get failed (%ld)",
				__func__, PTR_ERR(vreg_rf_switch));
		return -EFAULT;
	}

	if (state) {
		rc = vreg_set_level(vreg_rf, 1300);
		if (rc) {
			pr_err("%s: vreg s2 set level failed (%d)\n",
					__func__, rc);
			return rc;
		}

		rc = vreg_enable(vreg_rf);
		if (rc) {
			printk(KERN_ERR "%s: vreg_enable(s2) = %d\n",
					__func__, rc);
		}

		rc = vreg_set_level(vreg_rf_switch, 2600);
		if (rc) {
			pr_err("%s: vreg rf switch set level failed (%d)\n",
					__func__, rc);
			return rc;
		}
		rc = vreg_enable(vreg_rf_switch);
		if (rc) {
			printk(KERN_ERR "%s: vreg_enable(rf) = %d\n",
					__func__, rc);
		}
	} else {
		(void) vreg_disable(vreg_rf);
		(void) vreg_disable(vreg_rf_switch);
	}
	return 0;
}

static int mbp_setup_vregs(int state)
{
	struct vreg *vreg_analog = NULL;
	struct vreg *vreg_io = NULL;
	int rc;

	vreg_analog = vreg_get(NULL, "gp4");
	if (IS_ERR(vreg_analog)) {
		pr_err("%s: gp4 vreg get failed (%ld)",
				__func__, PTR_ERR(vreg_analog));
		return -EFAULT;
	}
	vreg_io = vreg_get(NULL, "s3");
	if (IS_ERR(vreg_io)) {
		pr_err("%s: s3 vreg get failed (%ld)",
				__func__, PTR_ERR(vreg_io));
		return -EFAULT;
	}
	if (state) {
		rc = vreg_set_level(vreg_analog, 2600);
		if (rc) {
			pr_err("%s: vreg_set_level failed (%d)",
					__func__, rc);
		}
		rc = vreg_enable(vreg_analog);
		if (rc) {
			pr_err("%s: analog vreg enable failed (%d)",
					__func__, rc);
		}
		rc = vreg_set_level(vreg_io, 1800);
		if (rc) {
			pr_err("%s: vreg_set_level failed (%d)",
					__func__, rc);
		}
		rc = vreg_enable(vreg_io);
		if (rc) {
			pr_err("%s: io vreg enable failed (%d)",
					__func__, rc);
		}
	} else {
		rc = vreg_disable(vreg_analog);
		if (rc) {
			pr_err("%s: analog vreg disable failed (%d)",
					__func__, rc);
		}
		rc = vreg_disable(vreg_io);
		if (rc) {
			pr_err("%s: io vreg disable failed (%d)",
					__func__, rc);
		}
	}
	return rc;
}

static int mbp_set_tcxo_en(int enable)
{
	int rc;
	const char *id = "UBMC";
	struct vreg *vreg_analog = NULL;

	rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_A1,
		enable ? PMAPP_CLOCK_VOTE_ON : PMAPP_CLOCK_VOTE_OFF);
	if (rc < 0) {
		printk(KERN_ERR "%s: unable to %svote for a1 clk\n",
			__func__, enable ? "" : "de-");
		return -EIO;
	}
	if (!enable) {
		vreg_analog = vreg_get(NULL, "gp4");
		if (IS_ERR(vreg_analog)) {
			pr_err("%s: gp4 vreg get failed (%ld)",
					__func__, PTR_ERR(vreg_analog));
			return -EFAULT;
		}

		(void) vreg_disable(vreg_analog);
	}
	return rc;
}

static void mbp_set_freeze_io(int state)
{
	if (state)
		gpio_set_value(85, 0);
	else
		gpio_set_value(85, 1);
}

static int mbp_set_core_voltage_en(int enable)
{
	int rc;
	struct vreg *vreg_core1p2 = NULL;

	vreg_core1p2 = vreg_get(NULL, "gp16");
	if (IS_ERR(vreg_core1p2)) {
		pr_err("%s: gp16 vreg get failed (%ld)",
				__func__, PTR_ERR(vreg_core1p2));
		return -EFAULT;
	}
	if (enable) {
		rc = vreg_set_level(vreg_core1p2, 1200);
		if (rc) {
			pr_err("%s: vreg_set_level failed (%d)",
					__func__, rc);
		}
		(void) vreg_enable(vreg_core1p2);

		return 80;
	} else {
		gpio_set_value(85, 1);
		return 0;
	}
	return rc;
}

static void mbp_set_reset(int state)
{
	if (state)
		gpio_set_value(GPIO_PIN(MBP_RESET_N), 0);
	else
		gpio_set_value(GPIO_PIN(MBP_RESET_N), 1);
}

static int mbp_config_interface_mode(int state)
{
	if (state) {
		gpio_tlmm_config(MBP_MODE_CTRL_0, GPIO_CFG_ENABLE);
		gpio_tlmm_config(MBP_MODE_CTRL_1, GPIO_CFG_ENABLE);
		gpio_tlmm_config(MBP_MODE_CTRL_2, GPIO_CFG_ENABLE);
		gpio_set_value(GPIO_PIN(MBP_MODE_CTRL_0), 0);
		gpio_set_value(GPIO_PIN(MBP_MODE_CTRL_1), 1);
		gpio_set_value(GPIO_PIN(MBP_MODE_CTRL_2), 0);
	} else {
		gpio_tlmm_config(MBP_MODE_CTRL_0, GPIO_CFG_DISABLE);
		gpio_tlmm_config(MBP_MODE_CTRL_1, GPIO_CFG_DISABLE);
		gpio_tlmm_config(MBP_MODE_CTRL_2, GPIO_CFG_DISABLE);
	}
	return 0;
}

static int mbp_setup_adc_vregs(int state)
{
	struct vreg *vreg_adc = NULL;
	int rc;

	vreg_adc = vreg_get(NULL, "s4");
	if (IS_ERR(vreg_adc)) {
		pr_err("%s: s4 vreg get failed (%ld)",
				__func__, PTR_ERR(vreg_adc));
		return -EFAULT;
	}
	if (state) {
		rc = vreg_set_level(vreg_adc, 2200);
		if (rc) {
			pr_err("%s: vreg_set_level failed (%d)",
					__func__, rc);
		}
		rc = vreg_enable(vreg_adc);
		if (rc) {
			pr_err("%s: enable vreg adc failed (%d)",
					__func__, rc);
		}
	} else {
		rc = vreg_disable(vreg_adc);
		if (rc) {
			pr_err("%s: disable vreg adc failed (%d)",
					__func__, rc);
		}
	}
	return rc;
}

static int mbp_power_up(void)
{
	int rc;

	rc = mbp_config_gpios_pre_init(MBP_ON);
	if (rc)
		goto exit;
	pr_debug("%s: mbp_config_gpios_pre_init() done\n", __func__);

	rc = mbp_setup_vregs(MBP_ON);
	if (rc)
		goto exit;
	pr_debug("%s: gp4 (2.6) and s3 (1.8) done\n", __func__);

	rc = mbp_set_tcxo_en(MBP_ON);
	if (rc)
		goto exit;
	pr_debug("%s: tcxo clock done\n", __func__);

	mbp_set_freeze_io(MBP_OFF);
	pr_debug("%s: set gpio 85 to 1 done\n", __func__);

	udelay(100);
	mbp_set_reset(MBP_ON);

	udelay(300);
	rc = mbp_config_interface_mode(MBP_ON);
	if (rc)
		goto exit;
	pr_debug("%s: mbp_config_interface_mode() done\n", __func__);

	udelay(100 + mbp_set_core_voltage_en(MBP_ON));
	pr_debug("%s: power gp16 1.2V done\n", __func__);

	mbp_set_freeze_io(MBP_ON);
	pr_debug("%s: set gpio 85 to 0 done\n", __func__);

	udelay(100);

	rc = mbp_setup_rf_vregs(MBP_ON);
	if (rc)
		goto exit;
	pr_debug("%s: s2 1.3V and rf 2.6V done\n", __func__);

	rc = mbp_setup_adc_vregs(MBP_ON);
	if (rc)
		goto exit;
	pr_debug("%s: s4 2.2V  done\n", __func__);

	udelay(200);

	mbp_set_reset(MBP_OFF);
	pr_debug("%s: close gpio 44 done\n", __func__);

	msleep(20);
exit:
	return rc;
}

static int mbp_power_down(void)
{
	int rc;
	struct vreg *vreg_adc = NULL;

	vreg_adc = vreg_get(NULL, "s4");
	if (IS_ERR(vreg_adc)) {
		pr_err("%s: s4 vreg get failed (%ld)",
				__func__, PTR_ERR(vreg_adc));
		return -EFAULT;
	}

	mbp_set_reset(MBP_ON);
	pr_debug("%s: mbp_set_reset(MBP_ON) done\n", __func__);

	udelay(100);

	rc = mbp_setup_adc_vregs(MBP_OFF);
	if (rc)
		goto exit;
	pr_debug("%s: vreg_disable(vreg_adc) done\n", __func__);

	udelay(5);

	rc = mbp_setup_rf_vregs(MBP_OFF);
	if (rc)
		goto exit;
	pr_debug("%s: mbp_setup_rf_vregs(MBP_OFF) done\n", __func__);

	udelay(5);

	mbp_set_freeze_io(MBP_OFF);
	pr_debug("%s: mbp_set_freeze_io(MBP_OFF) done\n", __func__);

	udelay(100);
	rc = mbp_set_core_voltage_en(MBP_OFF);
	if (rc)
		goto exit;
	pr_debug("%s: mbp_set_core_voltage_en(MBP_OFF) done\n", __func__);

	gpio_set_value(85, 1);

	rc = mbp_set_tcxo_en(MBP_OFF);
	if (rc)
		goto exit;
	pr_debug("%s: mbp_set_tcxo_en(MBP_OFF) done\n", __func__);

	rc = mbp_config_gpios_pre_init(MBP_OFF);
	if (rc)
		goto exit;
exit:
	return rc;
}

static void (*mbp_status_notify_cb)(int card_present, void *dev_id);
static void *mbp_status_notify_cb_devid;
static int mbp_power_status;
static int mbp_power_init_done;

static uint32_t mbp_setup_power(struct device *dv,
	unsigned int power_status)
{
	int rc = 0;
	struct platform_device *pdev;

	pdev = container_of(dv, struct platform_device, dev);

	if (power_status == mbp_power_status)
		goto exit;
	if (power_status) {
		pr_debug("turn on power of mbp slot");
		rc = mbp_power_up();
		mbp_power_status = 1;
	} else {
		pr_debug("turn off power of mbp slot");
		rc = mbp_power_down();
		mbp_power_status = 0;
	}
exit:
	return rc;
};

int mbp_register_status_notify(void (*callback)(int, void *),
	void *dev_id)
{
	mbp_status_notify_cb = callback;
	mbp_status_notify_cb_devid = dev_id;
	return 0;
}

static unsigned int mbp_status(struct device *dev)
{
	return mbp_power_status;
}


static uint32_t msm_sdcc_setup_power_mbp(struct device *dv, unsigned int vdd)
{
	struct platform_device *pdev;
	uint32_t rc = 0;

	pdev = container_of(dv, struct platform_device, dev);
	rc = msm_sdcc_setup_power(dv, vdd);
	if (rc) {
		pr_err("%s: Failed to setup power (%d)\n",
			__func__, rc);
		goto out;
	}
	if (!mbp_power_init_done) {
		mbp_setup_power(dv, 1);
		mbp_setup_power(dv, 0);
		mbp_power_init_done = 1;
	}
	if (vdd >= 0x8000) {
		rc = mbp_setup_power(dv, (0x8000 == vdd) ? 0 : 1);
		if (rc) {
			pr_err("%s: Failed to config mbp chip power (%d)\n",
				__func__, rc);
			goto out;
		}
		if (mbp_status_notify_cb) {
			mbp_status_notify_cb(mbp_power_status,
				mbp_status_notify_cb_devid);
		}
	}
out:
	/* should return 0 only */
	return 0;
}

#endif
static void (*wlan_status_notify_cb)(int card_present, void *dev_id);
static void *wlan_status_notify_cb_devid;
int wlan_register_status_notify(void (*callback)(int, void *),
	void *dev_id)
{
	wlan_status_notify_cb = callback;
	wlan_status_notify_cb_devid = dev_id;
	return 0;
}

static unsigned int wlan_status(struct device *dev)
{
	printk("wlan_status called....\n");
	return gpio_get_value (WLAN_RESET);
}

#endif

#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
static unsigned int msm7x30_sdcc_slot_status(struct device *dev)
{
	int rc;
	rc = gpio_get_value(116);
	rc = rc?0:1 ;

	return rc;
}
#endif

static int msm_sdcc_get_wpswitch(struct device *dv)
{
	void __iomem *wp_addr = 0;
	uint32_t ret = 0;
	struct platform_device *pdev;

	if (!(machine_is_msm7x30_surf()))
		return -1;
	pdev = container_of(dv, struct platform_device, dev);

	wp_addr = ioremap(FPGA_SDCC_STATUS, 4);
	if (!wp_addr) {
		pr_err("%s: Could not remap %x\n", __func__, FPGA_SDCC_STATUS);
		return -ENOMEM;
	}

	ret = (((readl(wp_addr) >> 4) >> (pdev->id-1)) & 0x01);
	pr_info("%s: WP Status for Slot %d = 0x%x \n", __func__,
							pdev->id, ret);
	iounmap(wp_addr);

	return ret;
}
#endif

#if defined(CONFIG_MMC_MSM_SDC1_SUPPORT)
#if defined(CONFIG_CSDIO_VENDOR_ID) && \
	defined(CONFIG_CSDIO_DEVICE_ID) && \
	(CONFIG_CSDIO_VENDOR_ID == 0x70 && CONFIG_CSDIO_DEVICE_ID == 0x1117)
static struct mmc_platform_data msm7x30_sdc1_data = {
	.ocr_mask	= MMC_VDD_165_195 | MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd	= msm_sdcc_setup_power_mbp,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.status	        = mbp_status,
	.register_status_notify = mbp_register_status_notify,
#ifdef CONFIG_MMC_MSM_SDC1_DUMMY52_REQUIRED
	.dummy52_required = 1,
#endif
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 24576000,
	.nonremovable	= 0,
};
#else
static struct mmc_platform_data msm7x30_sdc1_data = {
	.ocr_mask	= MMC_VDD_165_195 | MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd	= msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.status	        = wlan_status,
	.register_status_notify = wlan_register_status_notify,
#ifdef CONFIG_MMC_MSM_SDC1_DUMMY52_REQUIRED
	.dummy52_required = 1,
#endif
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 24576000,
	.nonremovable	= 0,
};
#endif
#endif

#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
static struct mmc_platform_data msm7x30_sdc2_data = {
	.ocr_mask	= MMC_VDD_165_195 | MMC_VDD_27_28,
	.translate_vdd	= msm_sdcc_setup_power,
#ifdef CONFIG_MMC_MSM_SDC2_8_BIT_SUPPORT
	.mmc_bus_width  = MMC_CAP_8_BIT_DATA,
#else
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
#endif
#ifdef CONFIG_MMC_MSM_SDC2_DUMMY52_REQUIRED
	.dummy52_required = 1,
#endif
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 49152000,
	.nonremovable	= 1,
};
#endif

#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
static struct mmc_platform_data msm7x30_sdc3_data = {
	.ocr_mask	= MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd	= msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
#ifdef CONFIG_MMC_MSM_SDIO_SUPPORT
	.sdiowakeup_irq = MSM_GPIO_TO_INT(118),
#endif
#ifdef CONFIG_MMC_MSM_SDC3_DUMMY52_REQUIRED
	.dummy52_required = 1,
#endif
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 49152000,
	.nonremovable	= 0,
};
#endif

#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
static struct mmc_platform_data msm7x30_sdc4_data = {
	.ocr_mask	= MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd	= msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
	.status      = msm7x30_sdcc_slot_status,
	.status_irq  = MSM_GPIO_TO_INT(116),
	.irq_flags   = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
#endif
	.wpswitch    = msm_sdcc_get_wpswitch,
#ifdef CONFIG_MMC_MSM_SDC4_DUMMY52_REQUIRED
	.dummy52_required = 1,
#endif
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 49152000,
	.nonremovable	= 0,
};
#endif

#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
static void msm_sdc1_lvlshft_enable(void)
{
	int rc;

	/* Enable LDO5, an input to the FET that powers slot 1 */
	rc = vreg_set_level(vreg_mmc, 2850);
	if (rc)
		printk(KERN_ERR "%s: vreg_set_level() = %d \n",	__func__, rc);

	rc = vreg_enable(vreg_mmc);
	if (rc)
		printk(KERN_ERR "%s: vreg_enable() = %d \n", __func__, rc);

	/* Enable GPIO 35, to turn on the FET that powers slot 1 */
	rc = msm_gpios_request_enable(sdc1_lvlshft_cfg_data,
				ARRAY_SIZE(sdc1_lvlshft_cfg_data));
	if (rc)
		printk(KERN_ERR "%s: Failed to enable GPIO 35\n", __func__);

	rc = gpio_direction_output(GPIO_PIN(sdc1_lvlshft_cfg_data[0].gpio_cfg),
				1);
	if (rc)
		printk(KERN_ERR "%s: Failed to turn on GPIO 35\n", __func__);
}
#endif

int msm_wlan_gpio_init ( void )
{
	printk(KERN_ERR "%s: msm_wlan_gpio_init\n", __func__);
	if (gpio_tlmm_config (GPIO_CFG(WLAN_EN_GPIO, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), GPIO_CFG_ENABLE))
	{
		printk (KERN_ERR "%s: Unable configure WLAN_EN_GPIO\n", __func__);
		return -EIO;
	}
	if (gpio_request (WLAN_EN_GPIO, "wlan_en"))
	{
		printk (KERN_ERR "%s: Unable to request WLAN_EN_GPIO", __func__);
		return -EINVAL;
	}

#if 0
	if (gpio_tlmm_config (GPIO_CFG(WLAN_RESET, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), GPIO_CFG_ENABLE))
	{
		printk (KERN_ERR "%s: Unable configure WLAN_RESET \n", __func__);
		return -EIO;
	}
	if (gpio_request (WLAN_RESET, "wlan_reset"))
	{
		printk (KERN_ERR "%s: Unable to request WLAN_RESET ", __func__);
		return -EINVAL;
	}
#endif
	if (gpio_tlmm_config (GPIO_CFG(WLAN_HOST_WAKE, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA), GPIO_CFG_ENABLE))
	{
		printk (KERN_ERR "%s: Unable configure WLAN_WAKEUP \n", __func__);
		return -EIO;
	}
	if (gpio_request (WLAN_HOST_WAKE, "wlan_wakeup"))
	{
		printk (KERN_ERR "%s: Unable to request WLAN_WAKEUP ", __func__);
		return -EINVAL;
	}

	gpio_set_value (WLAN_EN_GPIO, 0);
	gpio_set_value (WLAN_RESET, 0);

	return 0;
}
void wlan_setup_power(int on, int flag)
{
	if (flag == 1)
	{
		printk ("Before %s: WLAN_EN_GPIO value before set is %d on=%d WLAN_RESET=%d \n", __func__, gpio_get_value (WLAN_EN_GPIO),on,gpio_get_value (WLAN_RESET));
		if(on)
		{
			if (gpio_tlmm_config (GPIO_CFG(WLAN_EN_GPIO, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), GPIO_CFG_ENABLE))
			{
					printk (KERN_ERR "%s: Unable configure WLAN_EN_GPIO\n", __func__);
					return -EIO;
			}

			gpio_set_value (WLAN_EN_GPIO, on);
		}
		else
		{
			if(!gpio_get_value(GPIO_BT_RESET))
			{
				printk("Switching OFF Enable pin \n");
				if (gpio_tlmm_config (GPIO_CFG(WLAN_EN_GPIO, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), GPIO_CFG_ENABLE))
				{
						printk (KERN_ERR "%s: Unable configure WLAN_EN_GPIO\n", __func__);
						return -EIO;
				}

				gpio_set_value (WLAN_EN_GPIO, on);
			}
			else
			{
				printk("BT is Enabled\n");
			}
		}
		printk ("%s: WLAN_EN_GPIO value before set is %d on=%d \n", __func__, gpio_get_value (WLAN_EN_GPIO),on);
		if (gpio_tlmm_config (GPIO_CFG(WLAN_RESET, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), GPIO_CFG_ENABLE))
		{
				printk (KERN_ERR "%s: Unable configure WLAN_RESET \n", __func__);
				return -EIO;
		}

		mdelay (100);
		gpio_set_value (WLAN_RESET, 0);
		mdelay (100);
		printk("Just Resetting WIFI ONCE,before POWER ON\n");
		//
		gpio_set_value (WLAN_RESET, on);
		mdelay (10);
		//gpio_set_value (WLAN_RESET, 0);
		printk ("After %s: WLAN_EN_GPIO value before set is %d on=%d WLAN_RESET=%d \n", __func__, gpio_get_value (WLAN_EN_GPIO),on,gpio_get_value (WLAN_RESET));
		if (wlan_status_notify_cb) {
			printk ("%s: calling detect change\n", __func__);
			wlan_status_notify_cb(gpio_get_value (WLAN_EN_GPIO),
				wlan_status_notify_cb_devid);
		}
	}
	else
	{

		printk("Reseting WLAN...\n");
		if (gpio_tlmm_config (GPIO_CFG(WLAN_RESET, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), GPIO_CFG_ENABLE))
		{
			printk (KERN_ERR "%s: Unable configure WLAN_RESET \n", __func__);
			return -EIO;
		}

		gpio_set_value (WLAN_RESET, on);
	}
}
EXPORT_SYMBOL (wlan_setup_power);

#define WLAN_STATIC_BUF	// this feature for using static buffer on wifi driver /Kernel/drivers/net/wireless/bcm4330
#ifdef WLAN_STATIC_BUF

#define PREALLOC_WLAN_SEC_NUM		4
#define PREALLOC_WLAN_BUF_NUM		160
#define PREALLOC_WLAN_SECTION_HEADER	24


#define WLAN_SECTION_SIZE_0	(PREALLOC_WLAN_BUF_NUM * 128)
#define WLAN_SECTION_SIZE_1	(PREALLOC_WLAN_BUF_NUM * 128)
#define WLAN_SECTION_SIZE_2	(PREALLOC_WLAN_BUF_NUM * 512)
#define WLAN_SECTION_SIZE_3	(PREALLOC_WLAN_BUF_NUM * 1024)

#define WLAN_SKB_BUF_NUM	17

static struct sk_buff *wlan_static_skb[WLAN_SKB_BUF_NUM];

#endif /* WLAN_STATIC_BUF */

#ifdef WLAN_STATIC_BUF

struct wifi_mem_prealloc {
	void *mem_ptr;
	unsigned long size;
};

static struct wifi_mem_prealloc wifi_mem_array[PREALLOC_WLAN_SEC_NUM] = {
	{NULL, (WLAN_SECTION_SIZE_0 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_1 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_2 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_3 + PREALLOC_WLAN_SECTION_HEADER)}
};

void *wlan_mem_prealloc(int section, unsigned long size)
{
	if (section == PREALLOC_WLAN_SEC_NUM)
		return wlan_static_skb;

	if ((section < 0) || (section > PREALLOC_WLAN_SEC_NUM))
		return NULL;

	if (wifi_mem_array[section].size < size)
		return NULL;

	return wifi_mem_array[section].mem_ptr;
}
EXPORT_SYMBOL(wlan_mem_prealloc);

#define DHD_SKB_HDRSIZE 		336
#define DHD_SKB_1PAGE_BUFSIZE	((PAGE_SIZE*1)-DHD_SKB_HDRSIZE)
#define DHD_SKB_2PAGE_BUFSIZE	((PAGE_SIZE*2)-DHD_SKB_HDRSIZE)
#define DHD_SKB_4PAGE_BUFSIZE	((PAGE_SIZE*4)-DHD_SKB_HDRSIZE)

static int init_wifi_mem(void)
{
	int i;
	int j;

	for (i = 0; i < 8; i++) {
		wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_1PAGE_BUFSIZE);
		if (!wlan_static_skb[i])
			goto err_skb_alloc;
	}
	
	for (; i < 16; i++) {
		wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_2PAGE_BUFSIZE);
		if (!wlan_static_skb[i])
			goto err_skb_alloc;
	}
	
	wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_4PAGE_BUFSIZE);
	if (!wlan_static_skb[i])
		goto err_skb_alloc;

	for (i = 0 ; i < PREALLOC_WLAN_SEC_NUM ; i++) {
		wifi_mem_array[i].mem_ptr =
				kmalloc(wifi_mem_array[i].size, GFP_KERNEL);

		if (!wifi_mem_array[i].mem_ptr)
			goto err_mem_alloc;
	}
	printk("%s: WIFI MEM Allocated\n", __FUNCTION__);
	return 0;

 err_mem_alloc:
	pr_err("Failed to mem_alloc for WLAN\n");
	for (j = 0 ; j < i ; j++)
		kfree(wifi_mem_array[j].mem_ptr);

	i = WLAN_SKB_BUF_NUM;

 err_skb_alloc:
	pr_err("Failed to skb_alloc for WLAN\n");
	for (j = 0 ; j < i ; j++)
		dev_kfree_skb(wlan_static_skb[j]);

	return -ENOMEM;
}
#endif /* WLAN_STATIC_BUF */
static void __init msm7x30_init_mmc(void)
{
	vreg_s3 = vreg_get(NULL, "s3");
	if (IS_ERR(vreg_s3)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
		       __func__, PTR_ERR(vreg_s3));
		return;
	}

	vreg_mmc = vreg_get(NULL, "gp10");
	if (IS_ERR(vreg_mmc)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
		       __func__, PTR_ERR(vreg_mmc));
		return;
	}
#if 1
		if (msm_wlan_gpio_init ())
			printk (KERN_ERR "%s: Unable to initialize wlan GPIO's\n", __func__);
		else
			printk (KERN_ERR "%s: Initialized wlan GPIO's\n", __func__);
#endif

#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
	if (machine_is_msm7x30_fluid()) {
		msm7x30_sdc1_data.ocr_mask =  MMC_VDD_27_28 | MMC_VDD_28_29;
		msm_sdc1_lvlshft_enable();
	}
	sdcc_vreg_data[0].vreg_data = vreg_s3;
	sdcc_vreg_data[0].level = 1800;
	msm_add_sdcc(1, &msm7x30_sdc1_data);
#endif
#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
	if (machine_is_msm8x55_svlte_surf())
		msm7x30_sdc2_data.msmsdcc_fmax =  24576000;
	sdcc_vreg_data[1].vreg_data = vreg_s3;
	sdcc_vreg_data[1].level = 1800;
	msm_add_sdcc(2, &msm7x30_sdc2_data);
#endif
#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
	sdcc_vreg_data[2].vreg_data = vreg_s3;
	sdcc_vreg_data[2].level = 1800;
	msm_sdcc_setup_gpio(3, 1);
	msm_add_sdcc(3, &msm7x30_sdc3_data);
#endif
#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
	sdcc_vreg_data[3].vreg_data = vreg_mmc;
	sdcc_vreg_data[3].level = 2850;
	msm_add_sdcc(4, &msm7x30_sdc4_data);
#endif

}

static void __init msm7x30_init_nand(void)
{
	char *build_id;
	struct flash_platform_data *plat_data;

	build_id = socinfo_get_build_id();
	if (build_id == NULL) {
		pr_err("%s: Build ID not available from socinfo\n", __func__);
		return;
	}

	if (build_id[8] == 'C' &&
			!msm_gpios_request_enable(msm_nand_ebi2_cfg_data,
			ARRAY_SIZE(msm_nand_ebi2_cfg_data))) {
		plat_data = msm_device_nand.dev.platform_data;
		plat_data->interleave = 1;
		printk(KERN_INFO "%s: Interleave mode Build ID found\n",
			__func__);
	}
}

#ifdef CONFIG_SERIAL_MSM_CONSOLE
static struct msm_gpio uart2_config_data[] = {
	{ GPIO_CFG(49, 2, GPIO_CFG_OUTPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "UART2_RFR"},
	{ GPIO_CFG(50, 2, GPIO_CFG_INPUT,   GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "UART2_CTS"},
	{ GPIO_CFG(51, 2, GPIO_CFG_INPUT,   GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "UART2_Rx"},
	{ GPIO_CFG(52, 2, GPIO_CFG_OUTPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "UART2_Tx"},
};

static void msm7x30_init_uart2(void)
{
	msm_gpios_request_enable(uart2_config_data,
			ARRAY_SIZE(uart2_config_data));

}
#endif

/* 2011-01-20 hyeokseon.yu */
static struct msm_gpio uart3_config_data[] = {
	{ GPIO_CFG(53, 1, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "UART3_Rx"},
	{ GPIO_CFG(54, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "UART3_Tx"},
};

static void msm7x30_init_uart3(void)
{
	msm_gpios_request_enable(uart3_config_data,
			ARRAY_SIZE(uart3_config_data));
}

/* TSIF begin */
#if defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE)

#define TSIF_B_SYNC      GPIO_CFG(37, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_B_DATA      GPIO_CFG(36, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_B_EN        GPIO_CFG(35, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_B_CLK       GPIO_CFG(34, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)

static const struct msm_gpio tsif_gpios[] = {
	{ .gpio_cfg = TSIF_B_CLK,  .label =  "tsif_clk", },
	{ .gpio_cfg = TSIF_B_EN,   .label =  "tsif_en", },
	{ .gpio_cfg = TSIF_B_DATA, .label =  "tsif_data", },
	{ .gpio_cfg = TSIF_B_SYNC, .label =  "tsif_sync", },
};

static struct msm_tsif_platform_data tsif_platform_data = {
	.num_gpios = ARRAY_SIZE(tsif_gpios),
	.gpios = tsif_gpios,
	.tsif_pclk = "tsif_pclk",
	.tsif_ref_clk = "tsif_ref_clk",
};
#endif /* defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE) */
/* TSIF end   */

static void __init pmic8058_leds_init(void)
{
	if (machine_is_msm7x30_surf()) {
		pm8058_7x30_data.sub_devices[PM8058_SUBDEV_LED].platform_data
			= &pm8058_surf_leds_data;
		pm8058_7x30_data.sub_devices[PM8058_SUBDEV_LED].data_size
			= sizeof(pm8058_surf_leds_data);
	} else if (!machine_is_msm7x30_fluid()) {
		pm8058_7x30_data.sub_devices[PM8058_SUBDEV_LED].platform_data
			= &pm8058_ffa_leds_data;
		pm8058_7x30_data.sub_devices[PM8058_SUBDEV_LED].data_size
			= sizeof(pm8058_ffa_leds_data);
	} else if (machine_is_msm7x30_fluid()) {
		pm8058_7x30_data.sub_devices[PM8058_SUBDEV_LED].platform_data
			= &pm8058_fluid_leds_data;
		pm8058_7x30_data.sub_devices[PM8058_SUBDEV_LED].data_size
			= sizeof(pm8058_fluid_leds_data);
	}
}

static struct msm_spm_platform_data msm_spm_data __initdata = {
	.reg_base_addr = MSM_SAW_BASE,

	.reg_init_values[MSM_SPM_REG_SAW_CFG] = 0x05,
	.reg_init_values[MSM_SPM_REG_SAW_SPM_CTL] = 0x18,
	.reg_init_values[MSM_SPM_REG_SAW_SPM_SLP_TMR_DLY] = 0x00006666,
	.reg_init_values[MSM_SPM_REG_SAW_SPM_WAKE_TMR_DLY] = 0xFF000666,

	.reg_init_values[MSM_SPM_REG_SAW_SLP_CLK_EN] = 0x01,
	.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_PRECLMP_EN] = 0x03,
	.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_POSTCLMP_EN] = 0x00,

	.reg_init_values[MSM_SPM_REG_SAW_SLP_CLMP_EN] = 0x01,
	.reg_init_values[MSM_SPM_REG_SAW_SLP_RST_EN] = 0x00,
	.reg_init_values[MSM_SPM_REG_SAW_SPM_MPM_CFG] = 0x00,

	.awake_vlevel = 0xF2,
	.retention_vlevel = 0xE0,
	.collapse_vlevel = 0x72,
	.retention_mid_vlevel = 0xE0,
	.collapse_mid_vlevel = 0xE0,

	.vctl_timeout_us = 50,
};

#if defined(CONFIG_TOUCHSCREEN_TSC2007) || \
	defined(CONFIG_TOUCHSCREEN_TSC2007_MODULE)

#define TSC2007_TS_PEN_INT	20

static struct msm_gpio tsc2007_config_data[] = {
	{ GPIO_CFG(TSC2007_TS_PEN_INT, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	"tsc2007_irq" },
};

static struct vreg *vreg_tsc_s3;
static struct vreg *vreg_tsc_s2;

static int tsc2007_init(void)
{
	int rc;

	vreg_tsc_s3 = vreg_get(NULL, "s3");
	if (IS_ERR(vreg_tsc_s3)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
		       __func__, PTR_ERR(vreg_tsc_s3));
		return -ENODEV;
	}

	rc = vreg_set_level(vreg_tsc_s3, 1800);
	if (rc) {
		pr_err("%s: vreg_set_level failed \n", __func__);
		goto fail_vreg_set_level;
	}

	rc = vreg_enable(vreg_tsc_s3);
	if (rc) {
		pr_err("%s: vreg_enable failed \n", __func__);
		goto fail_vreg_set_level;
	}

	vreg_tsc_s2 = vreg_get(NULL, "s2");
	if (IS_ERR(vreg_tsc_s2)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
		       __func__, PTR_ERR(vreg_tsc_s2));
		goto fail_vreg_get;
	}

	rc = vreg_set_level(vreg_tsc_s2, 1300);
	if (rc) {
		pr_err("%s: vreg_set_level failed \n", __func__);
		goto fail_vreg_s2_level;
	}

	rc = vreg_enable(vreg_tsc_s2);
	if (rc) {
		pr_err("%s: vreg_enable failed \n", __func__);
		goto fail_vreg_s2_level;
	}

	rc = msm_gpios_request_enable(tsc2007_config_data,
			ARRAY_SIZE(tsc2007_config_data));
	if (rc) {
		pr_err("%s: Unable to request gpios\n", __func__);
		goto fail_gpio_req;
	}

	return 0;

fail_gpio_req:
	vreg_disable(vreg_tsc_s2);
fail_vreg_s2_level:
	vreg_put(vreg_tsc_s2);
fail_vreg_get:
	vreg_disable(vreg_tsc_s3);
fail_vreg_set_level:
	vreg_put(vreg_tsc_s3);
	return rc;
}

static int tsc2007_get_pendown_state(void)
{
	int rc;

	rc = gpio_get_value(TSC2007_TS_PEN_INT);
	if (rc < 0) {
		pr_err("%s: MSM GPIO %d read failed\n", __func__,
						TSC2007_TS_PEN_INT);
		return rc;
	}

	return (rc == 0 ? 1 : 0);
}

static void tsc2007_exit(void)
{
	vreg_disable(vreg_tsc_s3);
	vreg_put(vreg_tsc_s3);
	vreg_disable(vreg_tsc_s2);
	vreg_put(vreg_tsc_s2);

	msm_gpios_disable_free(tsc2007_config_data,
		ARRAY_SIZE(tsc2007_config_data));
}

static int tsc2007_power_shutdown(bool enable)
{
	int rc;

	if (enable == false) {
		rc = vreg_enable(vreg_tsc_s2);
		if (rc) {
			pr_err("%s: vreg_enable failed\n", __func__);
			return rc;
		}
		rc = vreg_enable(vreg_tsc_s3);
		if (rc) {
			pr_err("%s: vreg_enable failed\n", __func__);
			vreg_disable(vreg_tsc_s2);
			return rc;
		}
		/* Voltage settling delay */
		msleep(20);
	} else {
		rc = vreg_disable(vreg_tsc_s2);
		if (rc) {
			pr_err("%s: vreg_disable failed\n", __func__);
			return rc;
		}
		rc = vreg_disable(vreg_tsc_s3);
		if (rc) {
			pr_err("%s: vreg_disable failed\n", __func__);
			vreg_enable(vreg_tsc_s2);
			return rc;
		}
	}

	return rc;
}

static struct tsc2007_platform_data tsc2007_ts_data = {
	.model = 2007,
	.x_plate_ohms = 300,
	.irq_flags    = IRQF_TRIGGER_LOW,
	.init_platform_hw = tsc2007_init,
	.exit_platform_hw = tsc2007_exit,
	.power_shutdown	  = tsc2007_power_shutdown,
	.invert_x	  = true,
	.invert_y	  = true,
	/* REVISIT: Temporary fix for reversed pressure */
	.invert_z1	  = true,
	.invert_z2	  = true,
	.get_pendown_state = tsc2007_get_pendown_state,
};

static struct i2c_board_info tsc_i2c_board_info[] = {
	{
		I2C_BOARD_INFO("tsc2007", 0x48),
		.irq		= MSM_GPIO_TO_INT(TSC2007_TS_PEN_INT),
		.platform_data = &tsc2007_ts_data,
	},
};
#endif

static const char *vregs_isa1200_name[] = {
	"gp7",
	"gp10",
};

static const int vregs_isa1200_val[] = {
	1800,
	2600,
};
static struct vreg *vregs_isa1200[ARRAY_SIZE(vregs_isa1200_name)];

static int isa1200_power(int vreg_on)
{
	int i, rc = 0;

	for (i = 0; i < ARRAY_SIZE(vregs_isa1200_name); i++) {
		if (!vregs_isa1200[i]) {
			pr_err("%s: vreg_get %s failed (%d)\n",
				__func__, vregs_isa1200_name[i], rc);
			goto vreg_fail;
		}

		rc = vreg_on ? vreg_enable(vregs_isa1200[i]) :
			  vreg_disable(vregs_isa1200[i]);
		if (rc < 0) {
			pr_err("%s: vreg %s %s failed (%d)\n",
				__func__, vregs_isa1200_name[i],
			       vreg_on ? "enable" : "disable", rc);
			goto vreg_fail;
		}
	}
	return 0;

vreg_fail:
	while (i)
		vreg_disable(vregs_isa1200[--i]);
	return rc;
}

static int isa1200_dev_setup(bool enable)
{
	int i, rc;

	if (enable == true) {
		for (i = 0; i < ARRAY_SIZE(vregs_isa1200_name); i++) {
			vregs_isa1200[i] = vreg_get(NULL,
						vregs_isa1200_name[i]);
			if (IS_ERR(vregs_isa1200[i])) {
				pr_err("%s: vreg get %s failed (%ld)\n",
					__func__, vregs_isa1200_name[i],
					PTR_ERR(vregs_isa1200[i]));
				rc = PTR_ERR(vregs_isa1200[i]);
				goto vreg_get_fail;
			}
			rc = vreg_set_level(vregs_isa1200[i],
					vregs_isa1200_val[i]);
			if (rc) {
				pr_err("%s: vreg_set_level() = %d\n",
					__func__, rc);
				goto vreg_get_fail;
			}
		}

		rc = gpio_tlmm_config(GPIO_CFG(HAP_LVL_SHFT_MSM_GPIO, 0,
				GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
				GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("%s: Could not configure gpio %d\n",
					__func__, HAP_LVL_SHFT_MSM_GPIO);
			goto vreg_get_fail;
		}

		rc = gpio_request(HAP_LVL_SHFT_MSM_GPIO, "haptics_shft_lvl_oe");
		if (rc) {
			pr_err("%s: unable to request gpio %d (%d)\n",
					__func__, HAP_LVL_SHFT_MSM_GPIO, rc);
			goto vreg_get_fail;
		}

		gpio_set_value(HAP_LVL_SHFT_MSM_GPIO, 1);
	} else {
		for (i = 0; i < ARRAY_SIZE(vregs_isa1200_name); i++)
			vreg_put(vregs_isa1200[i]);

		gpio_free(HAP_LVL_SHFT_MSM_GPIO);
	}

	return 0;
vreg_get_fail:
	while (i)
		vreg_put(vregs_isa1200[--i]);
	return rc;
}
static struct isa1200_platform_data isa1200_1_pdata = {
	.name = "vibrator",
	.power_on = isa1200_power,
	.dev_setup = isa1200_dev_setup,
	.pwm_ch_id = 1, /*channel id*/
	/*gpio to enable haptic*/
	.hap_en_gpio = PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_HAP_ENABLE),
	.max_timeout = 15000,
	.mode_ctrl = PWM_GEN_MODE,
	.pwm_fd = {
		.pwm_div = 256,
	},
	.is_erm = false,
	.smart_en = true,
	.ext_clk_en = true,
	.chip_en = 1,
};

static struct i2c_board_info msm_isa1200_board_info[] = {
	{
		I2C_BOARD_INFO("isa1200_1", 0x90>>1),
		.platform_data = &isa1200_1_pdata,
	},
};


static int kp_flip_mpp_config(void)
{
	return pm8058_mpp_config_digital_in(PM_FLIP_MPP,
		PM8058_MPP_DIG_LEVEL_S3, PM_MPP_DIN_TO_INT);
}

static struct flip_switch_pdata flip_switch_data = {
	.name = "kp_flip_switch",
	.flip_gpio = PM8058_GPIO_PM_TO_SYS(PM8058_GPIOS) + PM_FLIP_MPP,
	.left_key = KEY_OPEN,
	.right_key = KEY_CLOSE,
	.active_low = 0,
	.wakeup = 1,
	.flip_mpp_config = kp_flip_mpp_config,
};

static struct platform_device flip_switch_device = {
	.name   = "kp_flip_switch",
	.id	= -1,
	.dev    = {
		.platform_data = &flip_switch_data,
	}
};

static const char *vregs_tma300_name[] = {
	"gp6",
	"gp7",
};

static const int vregs_tma300_val[] = {
	3050,
	1800,
};
static struct vreg *vregs_tma300[ARRAY_SIZE(vregs_tma300_name)];

static int tma300_power(int vreg_on)
{
	int i, rc = -EINVAL;

	for (i = 0; i < ARRAY_SIZE(vregs_tma300_name); i++) {
		/* Never disable gp6 for fluid as lcd has a problem with it */
		if (!i && !vreg_on)
			continue;
		if (!vregs_tma300[i]) {
			printk(KERN_ERR "%s: vreg_get %s failed (%d)\n",
				__func__, vregs_tma300_name[i], rc);
			return rc;
		}

		rc = vreg_on ? vreg_enable(vregs_tma300[i]) :
			  vreg_disable(vregs_tma300[i]);
		if (rc < 0) {
			printk(KERN_ERR "%s: vreg %s %s failed (%d)\n",
				__func__, vregs_tma300_name[i],
			       vreg_on ? "enable" : "disable", rc);
			return rc;
		}
	}

	return 0;
}

#define TS_GPIO_IRQ 150

static int tma300_dev_setup(bool enable)
{
	int i, rc;

	if (enable) {
		/* get voltage sources */
		for (i = 0; i < ARRAY_SIZE(vregs_tma300_name); i++) {
			vregs_tma300[i] = vreg_get(NULL, vregs_tma300_name[i]);
			if (IS_ERR(vregs_tma300[i])) {
				pr_err("%s: vreg get %s failed (%ld)\n",
					__func__, vregs_tma300_name[i],
					PTR_ERR(vregs_tma300[i]));
				rc = PTR_ERR(vregs_tma300[i]);
				goto vreg_get_fail;
			}
			rc = vreg_set_level(vregs_tma300[i],
					vregs_tma300_val[i]);
			if (rc) {
				pr_err("%s: vreg_set_level() = %d\n",
					__func__, rc);
				i++;
				goto vreg_get_fail;
			}
		}

		/* enable interrupt gpio */
		rc = gpio_tlmm_config(GPIO_CFG(TS_GPIO_IRQ, 0, GPIO_CFG_INPUT,
				GPIO_CFG_PULL_UP, GPIO_CFG_6MA), GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("%s: Could not configure gpio %d\n",
					__func__, TS_GPIO_IRQ);
			goto vreg_get_fail;
		}

		/* virtual keys */
		tma300_vkeys_attr.attr.name = "virtualkeys.msm_tma300_ts";
		properties_kobj = kobject_create_and_add("board_properties",
					NULL);
		if (!properties_kobj) {
			pr_err("%s: failed to create a kobject"
					"for board_properites\n", __func__);
			rc = -ENOMEM;
			goto vreg_get_fail;
		}
		rc = sysfs_create_group(properties_kobj,
				&tma300_properties_attr_group);
		if (rc) {
			pr_err("%s: failed to create a sysfs entry %s\n",
					__func__, tma300_vkeys_attr.attr.name);
			kobject_put(properties_kobj);
			goto vreg_get_fail;
		}
	} else {
		/* put voltage sources */
		for (i = 0; i < ARRAY_SIZE(vregs_tma300_name); i++)
			vreg_put(vregs_tma300[i]);
		/* destroy virtual keys */
		if (properties_kobj) {
			sysfs_remove_group(properties_kobj,
				&tma300_properties_attr_group);
			kobject_put(properties_kobj);
		}
	}
	return 0;

vreg_get_fail:
	while (i)
		vreg_put(vregs_tma300[--i]);
	return rc;
}

static struct cy8c_ts_platform_data cy8ctma300_pdata = {
	.power_on = tma300_power,
	.dev_setup = tma300_dev_setup,
	.ts_name = "msm_tma300_ts",
	.dis_min_x = 0,
	.dis_max_x = 479,
	.dis_min_y = 0,
	.dis_max_y = 799,
	.res_x	 = 479,
	.res_y	 = 1009,
	.min_tid = 1,
	.max_tid = 255,
	.min_touch = 0,
	.max_touch = 255,
	.min_width = 0,
	.max_width = 255,
	.invert_y = 1,
	.nfingers = 4,
	.irq_gpio = TS_GPIO_IRQ,
	.resout_gpio = -1,
};

static struct i2c_board_info cy8ctma300_board_info[] = {
	{
		I2C_BOARD_INFO("cy8ctma300", 0x2),
		.platform_data = &cy8ctma300_pdata,
	}
};

extern int no_console;

static void __init msm7x30_init(void)
{
	int rc;
	unsigned smem_size;
	uint32_t usb_hub_gpio_cfg_value = GPIO_CFG(56,
						0,
						GPIO_CFG_OUTPUT,
						GPIO_CFG_NO_PULL,
						GPIO_CFG_2MA);
	uint32_t soc_version = 0;

	if (socinfo_init() < 0)
		printk(KERN_ERR "%s: socinfo_init() failed!\n",
		       __func__);

	soc_version = socinfo_get_version();

	msm_clock_init(msm_clocks_7x30, msm_num_clocks_7x30);
#ifdef CONFIG_SERIAL_MSM_CONSOLE
	//msm7x30_init_uart2();
#endif
	if (!no_console) {
		msm7x30_init_uart3(); // 2011-01-20 hyeokseon.yu
	}
	msm_spm_init(&msm_spm_data, 1);
	msm_acpu_clock_init(&msm7x30_clock_data);
	if (machine_is_msm7x30_surf() || machine_is_msm7x30_fluid())
		msm7x30_cfg_smsc911x();
#ifdef CONFIG_USB_FUNCTION
	msm_hsusb_pdata.swfi_latency =
		msm_pm_data
		[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;
	msm_device_hsusb_peripheral.dev.platform_data = &msm_hsusb_pdata;
#endif

#ifdef CONFIG_USB_MSM_OTG_72K
	if (SOCINFO_VERSION_MAJOR(soc_version) >= 2 &&
			SOCINFO_VERSION_MINOR(soc_version) >= 1) {
		pr_debug("%s: SOC Version:2.(1 or more)\n", __func__);
		msm_otg_pdata.ldo_set_voltage = 0;
	}

	msm_device_otg.dev.platform_data = &msm_otg_pdata;
#ifdef CONFIG_USB_GADGET
	msm_otg_pdata.swfi_latency =
 	msm_pm_data
 	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;
	msm_device_gadget_peripheral.dev.platform_data = &msm_gadget_pdata;
#endif
#endif
	msm_uart_dm1_pdata.wakeup_irq = gpio_to_irq(136);
	msm_device_uart_dm1.dev.platform_data = &msm_uart_dm1_pdata;
#if defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE)
	msm_device_tsif.dev.platform_data = &tsif_platform_data;
#endif
	if (machine_is_msm7x30_fluid()) {
		msm_adc_pdata.dev_names = msm_adc_fluid_device_names;
		msm_adc_pdata.num_adc = ARRAY_SIZE(msm_adc_fluid_device_names);
	} else {
		msm_adc_pdata.dev_names = msm_adc_surf_device_names;
		msm_adc_pdata.num_adc = ARRAY_SIZE(msm_adc_surf_device_names);
	}

	platform_add_devices(devices, ARRAY_SIZE(devices));
	if(!no_console) {
		platform_add_devices(uart3_device, ARRAY_SIZE(uart3_device));
	}
#ifdef CONFIG_USB_EHCI_MSM
	msm_add_host(0, &msm_usb_host_pdata);
#endif
	msm7x30_init_mmc();
	msm7x30_init_nand();

	msm_fb_add_devices();
	msm_pm_set_platform_data(msm_pm_data, ARRAY_SIZE(msm_pm_data));
	msm_device_i2c_init();
	msm_device_i2c_2_init();
#if defined (CONFIG_USE_QUP_I2C)
	qup_device_i2c_init();
#endif	
	buses_init();
	msm7x30_init_marimba();
#ifdef CONFIG_MSM7KV2_AUDIO
	snddev_poweramp_gpio_init();
	aux_pcm_gpio_init();
#endif
#if 1
	fsa9480_gpio_init();
#endif
/* 2011-01-27 hyeokseon.yu */
#ifdef CONFIG_MAX17043_FUEL_GAUGE 
	fg17043_gpio_init();
#endif

#if defined(CONFIG_CHARGER_SMB328A)
	fg_smb_gpio_init();
#endif	

	ancora_switch_init();

#if defined (CONFIG_TOUCHSCREEN_QT602240)
	oliver_tsp_ldo_on();
#endif

#ifdef CONFIG_SENSORS_YDA165
	i2c_register_board_info(9, yamahaamp_boardinfo,
		ARRAY_SIZE(yamahaamp_boardinfo));
	pr_info("yda165:register yamaha amp device \n");
#endif
#ifdef CONFIG_SAMSUNG_FM_SI4709
  i2c_register_board_info(19, si4709_info,
		ARRAY_SIZE(si4709_info));
  pr_info("si4709:register fm radio si4709 device \n");
#endif

	i2c_register_board_info(0, msm_i2c_board_info,
			ARRAY_SIZE(msm_i2c_board_info));

	if (!machine_is_msm8x55_svlte_ffa())
		marimba_pdata.tsadc = &marimba_tsadc_pdata;

	if (machine_is_msm7x30_fluid())
		i2c_register_board_info(0, cy8info,
					ARRAY_SIZE(cy8info));

	i2c_register_board_info(2, msm_marimba_board_info,
			ARRAY_SIZE(msm_marimba_board_info));
	
#ifdef CONFIG_TIMPANI_CODEC
	i2c_register_board_info(2, msm_i2c_gsbi7_timpani_info,
			ARRAY_SIZE(msm_i2c_gsbi7_timpani_info));
#endif
if(board_hw_revision > 1) /* [diony][Ancora] REV0.0 : 0xAC,  REV0.1 : 0x5A */
{
	i2c_register_board_info(4 /* QUP ID */, msm_camera_rev01_boardinfo,
				ARRAY_SIZE(msm_camera_rev01_boardinfo));
}
else
{
	i2c_register_board_info(4 /* QUP ID */, msm_camera_boardinfo,
				ARRAY_SIZE(msm_camera_boardinfo));
}
	
#if defined (CONFIG_TOUCHSCREEN_QT602240)
	i2c_register_board_info(0, qt602240_touch_boardinfo,
		ARRAY_SIZE(qt602240_touch_boardinfo));
#endif

#if defined (CONFIG_TOUCHSCREEN_MELFAS_MCS8000)
	i2c_register_board_info(14, touch_i2c_devices, 
			ARRAY_SIZE(touch_i2c_devices));
#endif
	
#ifdef CONFIG_SENSORS_YAS529_MAGNETIC
   if(board_hw_revision >0)
   {
      	i2c_register_board_info(12, mag_i2c_devices,
      		ARRAY_SIZE(mag_i2c_devices));
      	pr_info("i2c_register_board_info 12 \n");
   }
#endif
#ifdef CONFIG_SENSORS_AK8975
   {
      	i2c_register_board_info(12, mag_i2c_devices,
      		ARRAY_SIZE(mag_i2c_devices));
      	pr_info("i2c_register_board_info 12 \n");
   }
#endif

#ifdef CONFIG_SENSORS_BMA023_ACCEL
   if(board_hw_revision >0)
   {
      	i2c_register_board_info(8, acc_i2c_devices,
      		ARRAY_SIZE(acc_i2c_devices));
      	pr_info("i2c_register_board_info 8 \n");
   }
#endif

#ifdef CONFIG_GYRO_L3G4200D
	i2c_register_board_info(18, gyro_i2c_devices,
		ARRAY_SIZE(gyro_i2c_devices));
	pr_info("i2c_register_board_info 18 \n");	
#endif

#if 1
	i2c_register_board_info(10, micro_usb_i2c_devices, 
			ARRAY_SIZE(micro_usb_i2c_devices));
      	printk("i2c_register_board_info 10 \n");
#endif

/* 2011-01-27 hyeokseon.yu */
#ifdef CONFIG_MAX17043_FUEL_GAUGE
	i2c_register_board_info(11, fuelgauge_i2c_devices,
			ARRAY_SIZE(fuelgauge_i2c_devices));
#endif


/* 2011-06-20 hyeokseon.yu */
#if defined(CONFIG_CHARGER_SMB328A)
   if(board_hw_revision >= CONFIG_HW_REV_USING_SMB328)
   {
	i2c_register_board_info(21, fg_smb_i2c_devices,
			ARRAY_SIZE(fg_smb_i2c_devices));
   }
#endif



#ifdef CONFIG_OPTICAL_GP2A
   if(board_hw_revision >0)
   {
      	i2c_register_board_info(17, opt_i2c_borad_info,
      		ARRAY_SIZE(opt_i2c_borad_info));
      	pr_info("i2c_register_board_info 17 \n");
   }
#endif


#if defined (CONFIG_SENSOR_MC7) || defined (CONFIG_SENSOR_CE147) || defined (CONFIG_SENSOR_S5K4ECGX) || defined (CONFIG_SENSOR_S5K5CCAF)
	i2c_register_board_info(13, msm_cam_pm_lp8720_info,
			ARRAY_SIZE(msm_cam_pm_lp8720_info));
#endif

	bt_power_init();
#ifdef CONFIG_I2C_SSBI
	msm_device_ssbi6.dev.platform_data = &msm_i2c_ssbi6_pdata;
	msm_device_ssbi7.dev.platform_data = &msm_i2c_ssbi7_pdata;
#endif
	if (machine_is_msm7x30_fluid())
		i2c_register_board_info(0, msm_isa1200_board_info,
			ARRAY_SIZE(msm_isa1200_board_info));

#if defined(CONFIG_TOUCHSCREEN_TSC2007) || \
	defined(CONFIG_TOUCHSCREEN_TSC2007_MODULE)
	if (machine_is_msm8x55_svlte_ffa())
		i2c_register_board_info(2, tsc_i2c_board_info,
				ARRAY_SIZE(tsc_i2c_board_info));
#endif

	if (machine_is_msm7x30_surf())
		platform_device_register(&flip_switch_device);
	pmic8058_leds_init();
	vibrator_device_gpio_init();

	if (machine_is_msm7x30_fluid()) {
		/* Initialize platform data for fluid v2 hardware */
		if (SOCINFO_VERSION_MAJOR(
				socinfo_get_platform_version()) == 2) {
			cy8ctma300_pdata.res_y = 920;
			cy8ctma300_pdata.invert_y = 0;
		}
		i2c_register_board_info(0, cy8ctma300_board_info,
			ARRAY_SIZE(cy8ctma300_board_info));
	}

#ifdef CONFIG_SENSORS_YAS529_MAGNETIC
   if(board_hw_revision >0)
   {
      	magnetic_device_init();	// yas529 nRST gpio pin configue
   }
#endif
#ifdef CONFIG_SENSORS_AK8975
   {
      	magnetic_device_init();	// ak8975 nRST gpio pin configue
   }
#endif
#ifdef CONFIG_KEYPAD_CYPRESS_TOUCH
	/* Touch Key */
	touch_keypad_gpio_init();
	i2c_register_board_info(20, touchkey_info, ARRAY_SIZE(touchkey_info));
	
#endif

	if (machine_is_msm8x55_svlte_surf() || machine_is_msm8x55_svlte_ffa()) {
		rc = gpio_tlmm_config(usb_hub_gpio_cfg_value, GPIO_CFG_ENABLE);
		if (rc)
			pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, usb_hub_gpio_cfg_value, rc);
	}

	boot_reason = *(unsigned int *)
		(smem_get_entry(SMEM_POWER_ON_STATUS_INFO, &smem_size));
	printk(KERN_NOTICE "Boot Reason = 0x%02x\n", boot_reason);
#ifdef WLAN_STATIC_BUF
	init_wifi_mem();
#endif
}

static unsigned pmem_sf_size = MSM_PMEM_SF_SIZE;
static int __init pmem_sf_size_setup(char *p)
{
	pmem_sf_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_sf_size", pmem_sf_size_setup);

static unsigned fb_size = MSM_FB_SIZE;
static int __init fb_size_setup(char *p)
{
	fb_size = memparse(p, NULL);
	return 0;
}
early_param("fb_size", fb_size_setup);

static unsigned pmem_adsp_size = MSM_PMEM_ADSP_SIZE;
static int __init pmem_adsp_size_setup(char *p)
{
	pmem_adsp_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_adsp_size", pmem_adsp_size_setup);

static unsigned fluid_pmem_adsp_size = MSM_FLUID_PMEM_ADSP_SIZE;
static int __init fluid_pmem_adsp_size_setup(char *p)
{
	fluid_pmem_adsp_size = memparse(p, NULL);
	return 0;
}
early_param("fluid_pmem_adsp_size", fluid_pmem_adsp_size_setup);

static unsigned pmem_audio_size = MSM_PMEM_AUDIO_SIZE;
static int __init pmem_audio_size_setup(char *p)
{
	pmem_audio_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_audio_size", pmem_audio_size_setup);

static unsigned pmem_kernel_ebi1_size = PMEM_KERNEL_EBI1_SIZE;
static int __init pmem_kernel_ebi1_size_setup(char *p)
{
	pmem_kernel_ebi1_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_kernel_ebi1_size", pmem_kernel_ebi1_size_setup);

static void __init msm7x30_allocate_memory_regions(void)
{
	void *addr;
	unsigned long size;
/*
   Request allocation of Hardware accessible PMEM regions
   at the beginning to make sure they are allocated in EBI-0.
   This will allow 7x30 with two mem banks enter the second
   mem bank into Self-Refresh State during Idle Power Collapse.

    The current HW accessible PMEM regions are
    1. Frame Buffer.
       LCDC HW can access msm_fb_resources during Idle-PC.

    2. Audio
       LPA HW can access android_pmem_audio_pdata during Idle-PC.
*/
	size = fb_size ? : MSM_FB_SIZE;
	addr = alloc_bootmem(size);
	msm_fb_resources[0].start = __pa(addr);
	msm_fb_resources[0].end = msm_fb_resources[0].start + size - 1;
	pr_info("allocating %lu bytes at %p (%lx physical) for fb\n",
		size, addr, __pa(addr));

	size = pmem_audio_size;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_audio_pdata.start = __pa(addr);
		android_pmem_audio_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for audio "
			"pmem arena\n", size, addr, __pa(addr));
	}

	size = pmem_kernel_ebi1_size;
	if (size) {
		addr = alloc_bootmem_aligned(size, 0x100000);
		android_pmem_kernel_ebi1_pdata.start = __pa(addr);
		android_pmem_kernel_ebi1_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for kernel"
			" ebi1 pmem arena\n", size, addr, __pa(addr));
	}

	size = pmem_sf_size;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_pdata.start = __pa(addr);
		android_pmem_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for sf "
			"pmem arena\n", size, addr, __pa(addr));
	}

	if machine_is_msm7x30_fluid()
		size = fluid_pmem_adsp_size;
	else
		size = pmem_adsp_size;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_adsp_pdata.start = __pa(addr);
		android_pmem_adsp_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for adsp "
			"pmem arena\n", size, addr, __pa(addr));
	}
}

static void __init msm7x30_map_io(void)
{
#if defined (CONFIG_APPSBOOT_3M_CONFIG)
	msm_shared_ram_phys = 0x00300000;
#else
	msm_shared_ram_phys = 0x00100000;
#endif
	msm_map_msm7x30_io();
	msm7x30_allocate_memory_regions();
}

MACHINE_START(ANCORA, "GT-I8150 Board")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io  = MSM_DEBUG_UART_PHYS,
	.io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params = PHYS_OFFSET + 0x100,
	.map_io = msm7x30_map_io,
	.init_irq = msm7x30_init_irq,
	.init_machine = msm7x30_init,
	.timer = &msm_timer,
MACHINE_END

MACHINE_START(MSM7X30_SURF, "QCT MSM7X30 SURF")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io  = MSM_DEBUG_UART_PHYS,
	.io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params = PHYS_OFFSET + 0x100,
	.map_io = msm7x30_map_io,
	.init_irq = msm7x30_init_irq,
	.init_machine = msm7x30_init,
	.timer = &msm_timer,
MACHINE_END

MACHINE_START(MSM7X30_FFA, "QCT MSM7X30 FFA")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io  = MSM_DEBUG_UART_PHYS,
	.io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params = PHYS_OFFSET + 0x100,
	.map_io = msm7x30_map_io,
	.init_irq = msm7x30_init_irq,
	.init_machine = msm7x30_init,
	.timer = &msm_timer,
MACHINE_END

MACHINE_START(MSM7X30_FLUID, "QCT MSM7X30 FLUID")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io  = MSM_DEBUG_UART_PHYS,
	.io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params = PHYS_OFFSET + 0x100,
	.map_io = msm7x30_map_io,
	.init_irq = msm7x30_init_irq,
	.init_machine = msm7x30_init,
	.timer = &msm_timer,
MACHINE_END

MACHINE_START(MSM8X55_SURF, "QCT MSM8X55 SURF")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io  = MSM_DEBUG_UART_PHYS,
	.io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params = PHYS_OFFSET + 0x100,
	.map_io = msm7x30_map_io,
	.init_irq = msm7x30_init_irq,
	.init_machine = msm7x30_init,
	.timer = &msm_timer,
MACHINE_END

MACHINE_START(MSM8X55_FFA, "QCT MSM8X55 FFA")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io  = MSM_DEBUG_UART_PHYS,
	.io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params = PHYS_OFFSET + 0x100,
	.map_io = msm7x30_map_io,
	.init_irq = msm7x30_init_irq,
	.init_machine = msm7x30_init,
	.timer = &msm_timer,
MACHINE_END
MACHINE_START(MSM8X55_SVLTE_SURF, "QCT MSM8X55 SVLTE SURF")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io  = MSM_DEBUG_UART_PHYS,
	.io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params = PHYS_OFFSET + 0x100,
	.map_io = msm7x30_map_io,
	.init_irq = msm7x30_init_irq,
	.init_machine = msm7x30_init,
	.timer = &msm_timer,
MACHINE_END
MACHINE_START(MSM8X55_SVLTE_FFA, "QCT MSM8X55 SVLTE FFA")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io  = MSM_DEBUG_UART_PHYS,
	.io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params = PHYS_OFFSET + 0x100,
	.map_io = msm7x30_map_io,
	.init_irq = msm7x30_init_irq,
	.init_machine = msm7x30_init,
	.timer = &msm_timer,
MACHINE_END

