#ifdef CONFIG_COMPAT

#include <linux/fs.h>
#include <linux/compat.h>

#endif
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include "kd_flashlight.h"
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_typedef.h"
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/leds.h>

#include <mt-plat/mt_gpio.h>


/******************************************************************************
 * Debug configuration
******************************************************************************/
/* availible parameter */
/* ANDROID_LOG_ASSERT */
/* ANDROID_LOG_ERROR */
/* ANDROID_LOG_WARNING */
/* ANDROID_LOG_INFO */
/* ANDROID_LOG_DEBUG */
/* ANDROID_LOG_VERBOSE */

#define TAG_NAME "[strobe_main_sid2_part1.c]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_err(TAG_NAME "%s: " fmt, __func__ , ##arg)

#define DEBUG_LEDS_STROBE
#ifdef DEBUG_LEDS_STROBE
#define PK_DBG PK_DBG_FUNC
#else
#define PK_DBG(a, ...)
#endif

/******************************************************************************
 * local variables
******************************************************************************/

static DEFINE_SPINLOCK(g_strobeSMPLock);	/* cotta-- SMP proection */


static u32 strobe_Res;
static u32 strobe_Timeus;
static BOOL g_strobe_On;

//static int g_duty = -1;
static int g_timeOutTimeMs;

extern int m_duty1;
extern int LED1CloseFlag;
static int m_duty2=0;
static int LED2CloseFlag=0;

static DEFINE_MUTEX(g_strobeSem);


#define STROBE_DEVICE_ID 0xC6


static struct work_struct workTimeOut;

#define GPIO_TORCH_EN 88 

static int g_bLtVersion;

/*****************************************************************************
Functions
*****************************************************************************/
static void work_timeOutFunc(struct work_struct *data);

static struct i2c_client *LM3642_i2c_client;




struct LM3642_platform_data {
	u8 torch_pin_enable;	/* 1:  TX1/TORCH pin isa hardware TORCH enable */
	u8 pam_sync_pin_enable;	/* 1:  TX2 Mode The ENVM/TX2 is a PAM Sync. on input */
	u8 thermal_comp_mode_enable;	/* 1: LEDI/NTC pin in Thermal Comparator Mode */
	u8 strobe_pin_disable;	/* 1 : STROBE Input disabled */
	u8 vout_mode_enable;	/* 1 : Voltage Out Mode enable */
};

struct LM3642_chip_data {
	struct i2c_client *client;

	/* struct led_classdev cdev_flash; */
	/* struct led_classdev cdev_torch; */
	/* struct led_classdev cdev_indicator; */

	struct LM3642_platform_data *pdata;
	struct mutex lock;

	u8 last_flag;
	u8 no_pdata;
};

static int LM3642_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret = 0;
	struct LM3642_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	ret = i2c_smbus_write_byte_data(client, reg, val);
	mutex_unlock(&chip->lock);

	if (ret < 0)
		PK_DBG("failed writing at 0x%02x\n", reg);
	return ret;
}

static int LM3642_read_reg(struct i2c_client *client, u8 reg)
{
	int val = 0;
	struct LM3642_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	val = i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->lock);


	return val;
}




static int LM3642_chip_init(struct LM3642_chip_data *chip)
{


	return 0;
}

static int LM3642_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct LM3642_chip_data *chip;
	struct LM3642_platform_data *pdata = client->dev.platform_data;

	int err = -1;

	PK_DBG("LM3642_probe start--->.\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		PK_DBG("LM3642 i2c functionality check fail.\n");
		return err;
	}

	chip = kzalloc(sizeof(struct LM3642_chip_data), GFP_KERNEL);
	chip->client = client;

	mutex_init(&chip->lock);
	i2c_set_clientdata(client, chip);

	if (pdata == NULL) {	/* values are set to Zero. */
		PK_DBG("LM3642 Platform data does not exist\n");
		pdata = kzalloc(sizeof(struct LM3642_platform_data), GFP_KERNEL);
		chip->pdata = pdata;
		chip->no_pdata = 1;
	}

	chip->pdata = pdata;
	if (LM3642_chip_init(chip) < 0)
		goto err_chip_init;

	LM3642_i2c_client = client;
	PK_DBG("LM3642 Initializing is done\n");

	return 0;

err_chip_init:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
	PK_DBG("LM3642 probe is failed\n");
	return -ENODEV;
}

static int LM3642_remove(struct i2c_client *client)
{
	struct LM3642_chip_data *chip = i2c_get_clientdata(client);

	if (chip->no_pdata)
		kfree(chip->pdata);
	kfree(chip);
	return 0;
}


#define LM3642_NAME "leds-LM3642"
static const struct i2c_device_id LM3642_id[] = {
	{LM3642_NAME, 0},
	{}
};

#ifdef CONFIG_OF
static const struct of_device_id LM3642_of_match[] = {
	{.compatible = "mediatek,strobe_main"},
	{},
};
#endif

static struct i2c_driver LM3642_i2c_driver = {
	.driver = {
		   .name = LM3642_NAME,
#ifdef CONFIG_OF
		   .of_match_table = LM3642_of_match,
#endif
		   },
	.probe = LM3642_probe,
	.remove = LM3642_remove,
	.id_table = LM3642_id,
};
static int __init LM3642_init(void)
{
	PK_DBG("LM3642_init\n");
	return i2c_add_driver(&LM3642_i2c_driver);
}

static void __exit LM3642_exit(void)
{
	i2c_del_driver(&LM3642_i2c_driver);
}


module_init(LM3642_init);
module_exit(LM3642_exit);

MODULE_DESCRIPTION("Flash driver for LM3642");
MODULE_AUTHOR("pw <pengwei@mediatek.com>");
MODULE_LICENSE("GPL v2");

#define e_DutyNum 26
#define TORCHDUTYNUNM 4
static int isMovieMode[e_DutyNum] = {1, 1,  1,  0, 0,0,0,0,0,0,0,0,0,0,0,0,0};
static int torchLEDReg[e_DutyNum] = {9, 17, 25, 35,40,45,50,55,60,65,70,75,80,85,90,95};
static int flashLEDReg[e_DutyNum] = {9, 17, 30, 43, 55, 68, 81, 94, 106,136,132,102,111,120,128,136,145,153,162,170, 187, 204, 222, 238, 255};
                                   //50,100,175,250,325,400,475,550,625,800,775,850,925,1000,750,800,850,900,950,1000,1100,1200,1300,1400,1500
#define LED1_TORCH_VALUE_REG 0x0d//0x0c
#define LED2_TORCH_VALUE_REG 0x0c//0x0d
#define LED1_FLASH_VALUE_REG 0x07//0x06
#define LED2_FLASH_VALUE_REG 0x06//0x07

int readReg(int reg)
{

	int val;

	val = LM3642_read_reg(LM3642_i2c_client, reg);
	return (int)val;
}

int FL_Enable(void)
{

    char buf[2];
    buf[0] = 0x01;
	PK_DBG(" FL_Enable LED1CloseFlag=%d,m_duty1=%d,LED2CloseFlag=%d,m_duty2=%d\n", LED1CloseFlag,m_duty1,LED2CloseFlag,m_duty2);
	if(m_duty1<0)
		m_duty1 = 0;
	if(m_duty1>10)
		m_duty1 = 10;

	if(m_duty2<0)
		m_duty2 = 0;
	if(m_duty2>10)
		m_duty2 = 10;

    if((LED1CloseFlag == 1) && (LED2CloseFlag == 1))
    { 
       //close
	   LM3642_write_reg(LM3642_i2c_client, 0x01, 0xA0);
    }
	else if(LED1CloseFlag == 1)
	{
	  if(isMovieMode[m_duty2] == 1)
	  {
	  	//torch
	  	PK_DBG(" FL_Enable torch=%d\n", __LINE__);
		LM3642_write_reg(LM3642_i2c_client, LED2_TORCH_VALUE_REG, torchLEDReg[m_duty2]);
	  	LM3642_write_reg(LM3642_i2c_client, buf[0], 0xf0);//0xe8
	  }
	  else
	  {
	  	//flash
	  	PK_DBG(" FL_Enable flash=%d\n", __LINE__);
		LM3642_write_reg(LM3642_i2c_client, LED2_FLASH_VALUE_REG, flashLEDReg[m_duty2]);
	  	LM3642_write_reg(LM3642_i2c_client, buf[0], 0xd6);//0xce
	  }
	}
	else if(LED2CloseFlag == 1)
	{
	  if(isMovieMode[m_duty1] == 1)
	  {
	  	//torch
	  	PK_DBG(" FL_Enable torch=%d\n", __LINE__);
		LM3642_write_reg(LM3642_i2c_client, LED1_TORCH_VALUE_REG, torchLEDReg[m_duty1]);
	  	LM3642_write_reg(LM3642_i2c_client, buf[0], 0xe8);//0xf0
	  }
	  else
	  {
	  	//flash
	  	PK_DBG(" FL_Enable flash=%d\n", __LINE__);
		LM3642_write_reg(LM3642_i2c_client, LED1_FLASH_VALUE_REG, flashLEDReg[m_duty1]);
	  	LM3642_write_reg(LM3642_i2c_client, buf[0], 0xce);//0xd6
	  }
	}
	else
	{
	  if(isMovieMode[m_duty1] == 1 && isMovieMode[m_duty2] == 1)
	  {
	  	//torch
	  	PK_DBG(" FL_Enable torch=%d\n", __LINE__);
		LM3642_write_reg(LM3642_i2c_client, LED1_TORCH_VALUE_REG, torchLEDReg[m_duty1]);
		LM3642_write_reg(LM3642_i2c_client, LED2_TORCH_VALUE_REG, torchLEDReg[m_duty2]);
	  	LM3642_write_reg(LM3642_i2c_client, buf[0], 0xf8);
	  }
	  else
	  {
	  	//flash
	  	PK_DBG(" FL_Enable flash=%d\n", __LINE__);
		LM3642_write_reg(LM3642_i2c_client, LED1_FLASH_VALUE_REG, flashLEDReg[m_duty1]);
		LM3642_write_reg(LM3642_i2c_client, LED2_FLASH_VALUE_REG, flashLEDReg[m_duty2]);
	  	LM3642_write_reg(LM3642_i2c_client, buf[0], 0xde);
	  }
	}

	return 0;
}



int FL_Disable(void)
{
	char buf[2];

	buf[0]=0x01;
	buf[1]=0xA0;
	PK_DBG(" FL_Disable LED1CloseFlag=%d,m_duty1=%d,LED2CloseFlag=%d,m_duty2=%d\n", LED1CloseFlag,m_duty1,LED2CloseFlag,m_duty2);
	if(LED1CloseFlag == 1)
	{
	  LM3642_write_reg(LM3642_i2c_client, buf[0], buf[1]);
	}
	else
	{
	  if(isMovieMode[m_duty1] == 1)
	  {
	    //torch mode
	    PK_DBG(" FL_Disable torch=%d\n", __LINE__);
		LM3642_write_reg(LM3642_i2c_client, LED1_TORCH_VALUE_REG, torchLEDReg[m_duty1]);
	    LM3642_write_reg(LM3642_i2c_client, buf[0], 0xe8);//0xf0 led2
	  }
	  else
	  {
	  	//flash
	  	PK_DBG(" FL_Disable flash=%d\n", __LINE__);
		LM3642_write_reg(LM3642_i2c_client, LED1_FLASH_VALUE_REG, flashLEDReg[m_duty1]);
	  	LM3642_write_reg(LM3642_i2c_client, buf[0], 0xce);//0xd6 led2
	  }
	}
	return 0;
}

int FL_dim_duty(kal_uint32 duty)
{
	m_duty2 = duty;
	return 0;
}

int FL_Init(void)
{
	int regVal0;
	char buf[2];

    buf[0]=0x01;
	buf[1]=0xA0;
	LM3642_write_reg(LM3642_i2c_client, buf[0], buf[1]);

	buf[0]=0x03;
	buf[1]=0xf6;
	LM3642_write_reg(LM3642_i2c_client, buf[0], buf[1]);

	buf[0]=0x04;
	buf[1]=0x34;
	LM3642_write_reg(LM3642_i2c_client, buf[0], buf[1]);

	buf[0]=0x06;
	buf[1]=0xaa;
	LM3642_write_reg(LM3642_i2c_client, buf[0], buf[1]);

	buf[0]=0x07;
	buf[1]=0xaa;
	LM3642_write_reg(LM3642_i2c_client, buf[0], buf[1]);

	buf[0]=0x0c;
	buf[1]=0x33;
	LM3642_write_reg(LM3642_i2c_client, buf[0], buf[1]);

	buf[0]=0x0d;
	buf[1]=0x33;
	LM3642_write_reg(LM3642_i2c_client, buf[0], buf[1]);

	regVal0 = LM3642_read_reg(LM3642_i2c_client, 0);

	if (regVal0 == 1)
		g_bLtVersion = 1;
	else
		g_bLtVersion = 0;


	PK_DBG(" FL_Init regVal0=%d isLtVer=%d\n", regVal0, g_bLtVersion);


	if(mt_set_gpio_mode(GPIO_TORCH_EN,GPIO_MODE_00)){PK_DBG("[constant_flashlight] set gpio mode failed!!\n");}
    if(mt_set_gpio_dir(GPIO_TORCH_EN,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!!\n");}
	if(mt_set_gpio_out(GPIO_TORCH_EN,GPIO_OUT_ONE)){PK_DBG("[constant_flashlight] set gpio failed!! \n");}

	return 0;
}


int FL_Uninit(void)
{
    if(mt_set_gpio_out(GPIO_TORCH_EN,GPIO_OUT_ZERO)){PK_DBG("[constant_flashlight] set gpio failed!!\n");}
	//FL_Disable();
	LM3642_write_reg(LM3642_i2c_client, 0x01, 0xa0);
	return 0;
}

static int FL_hasLowPowerDetect(void)
{

	return 1;
}

static int detLowPowerStart(void)
{

/* g_lowPowerLevel=LOW_BATTERY_LEVEL_0; */
	return 0;
}


static int detLowPowerEnd(void)
{

	return 0;
}


/*****************************************************************************
User interface
*****************************************************************************/

static void work_timeOutFunc(struct work_struct *data)
{
	FL_Disable();
	PK_DBG("ledTimeOut_callback\n");
}



enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
	schedule_work(&workTimeOut);
	return HRTIMER_NORESTART;
}

static struct hrtimer g_timeOutTimer;
void timerInit(void)
{
	INIT_WORK(&workTimeOut, work_timeOutFunc);
	g_timeOutTimeMs = 1000;
	hrtimer_init(&g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	g_timeOutTimer.function = ledTimeOutCallback;
}



static int constant_flashlight_ioctl(unsigned int cmd, unsigned long arg)
{
    int temp;
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;

	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC, 0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC, 0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC, 0, int));
/*	PK_DBG
	    ("LM3642 constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",
	     __LINE__, ior_shift, iow_shift, iowr_shift, (int)arg);
*/
	switch (cmd) {

	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n", (int)arg);
		g_timeOutTimeMs = arg;
		break;


	case FLASH_IOC_SET_DUTY:
		PK_DBG("FLASHLIGHT_DUTY: %d\n", (int)arg);
		//FL_dim_duty(arg);
		m_duty2 = arg;
		break;


	case FLASH_IOC_SET_STEP:
		PK_DBG("FLASH_IOC_SET_STEP: %d\n", (int)arg);

		break;

	case FLASH_IOC_SET_ONOFF:
		PK_DBG("FLASHLIGHT_ONOFF: %d\n", (int)arg);
		if (arg == 1) {

			int s;
			int ms;

			if (g_timeOutTimeMs > 1000) {
				s = g_timeOutTimeMs / 1000;
				ms = g_timeOutTimeMs - s * 1000;
			} else {
				s = 0;
				ms = g_timeOutTimeMs;
			}

			if (g_timeOutTimeMs != 0) {
				ktime_t ktime;

				ktime = ktime_set(s, ms * 1000000);
				hrtimer_start(&g_timeOutTimer, ktime, HRTIMER_MODE_REL);
			}
			LED2CloseFlag = 0;
			FL_dim_duty(m_duty2);
			FL_Enable();
		} else {
            LED2CloseFlag = 1;
			FL_dim_duty(m_duty2);
			FL_Disable();
			//FL_Enable();
			hrtimer_cancel(&g_timeOutTimer);
		}
		break;
		
	case FLASH_IOC_HAS_LOW_POWER_DETECT:
		PK_DBG("FLASH_IOC_HAS_LOW_POWER_DETECT");
		temp = FL_hasLowPowerDetect();
		if (copy_to_user((void __user *)arg, (void *)&temp, 4)) {
			PK_DBG(" ioctl copy to user failed\n");
			return -1;
		}
		break;
	case FLASH_IOC_LOW_POWER_DETECT_START:
		detLowPowerStart();
		break;
	case FLASH_IOC_LOW_POWER_DETECT_END:
		i4RetValue = detLowPowerEnd();
		break;
	default:
		PK_DBG(" No such command\n");
		i4RetValue = -EPERM;
		break;
	}
	return i4RetValue;
}




static int constant_flashlight_open(void *pArg)
{
	int i4RetValue = 0;

	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res) {
		FL_Init();
		timerInit();
	}
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
	spin_lock_irq(&g_strobeSMPLock);


	if (strobe_Res) {
		PK_DBG(" busy!\n");
		i4RetValue = -EBUSY;
	} else {
		strobe_Res += 1;
	}


	spin_unlock_irq(&g_strobeSMPLock);
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	return i4RetValue;

}


static int constant_flashlight_release(void *pArg)
{
	PK_DBG(" constant_flashlight_release\n");

	if (strobe_Res) {
		spin_lock_irq(&g_strobeSMPLock);

		strobe_Res = 0;
		strobe_Timeus = 0;

		/* LED On Status */
		g_strobe_On = FALSE;

		spin_unlock_irq(&g_strobeSMPLock);

		FL_Uninit();
	}

	PK_DBG(" Done\n");

	return 0;

}


static FLASHLIGHT_FUNCTION_STRUCT constantFlashlightFunc = {
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl
};



MUINT32 strobeInit_main_sid2_part1(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
	if (pfFunc != NULL)
		*pfFunc = &constantFlashlightFunc;
	return 0;
}
