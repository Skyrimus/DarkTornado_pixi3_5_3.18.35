#include <linux/kernel.h> //constant xx
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
#include "kd_camera_hw.h"
#include <cust_gpio_usage.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/xlog.h>
#include <linux/version.h>
#include <cust_leds.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
#include <linux/mutex.h>
#else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
#include <linux/semaphore.h>
#else
#include <asm/semaphore.h>
#endif
#endif



/******************************************************************************
 * Debug configuration
******************************************************************************/
// availible parameter
// ANDROID_LOG_ASSERT
// ANDROID_LOG_ERROR
// ANDROID_LOG_WARNING
// ANDROID_LOG_INFO
// ANDROID_LOG_DEBUG
// ANDROID_LOG_VERBOSE
#define TAG_NAME "leds_strobe.c"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_WARN(fmt, arg...)        pr_warning(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_NOTICE(fmt, arg...)      pr_notice(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_INFO(fmt, arg...)        pr_info(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_TRC_FUNC(f)              pr_debug(TAG_NAME "<%s>\n", __FUNCTION__)
#define PK_TRC_VERBOSE(fmt, arg...) pr_debug(TAG_NAME fmt, ##arg)
#define PK_ERROR(fmt, arg...)       pr_err(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)


#define DEBUG_LEDS_STROBE
#ifdef  DEBUG_LEDS_STROBE
	#define PK_DBG PK_DBG_FUNC
	#define PK_VER PK_TRC_VERBOSE
	#define PK_ERR PK_ERROR
#else
	#define PK_DBG(a,...)
	#define PK_VER(a,...)
	#define PK_ERR(a,...)
#endif

/******************************************************************************
 * local variables
******************************************************************************/

static DEFINE_SPINLOCK(g_strobeSMPLock); /* cotta-- SMP proection */


static u32 strobe_Res = 0;
static u32 strobe_Timeus = 0;
static BOOL g_strobe_On = 0;

static int g_duty=-1;
static int g_timeOutTimeMs=0;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
static DEFINE_MUTEX(g_strobeSem);
#else
static DECLARE_MUTEX(g_strobeSem);
#endif


#define STROBE_DEVICE_ID 0xC6


static struct work_struct workTimeOut;

/*****************************************************************************
Functions
*****************************************************************************/
#define GPIO_ENF GPIO_CAMERA_FLASH_EN_PIN
#define GPIO_ENT GPIO_CAMERA_FLASH_MODE_PIN
//#define GPIO_FAKE_FLASH GPIO11
#define HQ_TORCH_SET_PWM_MODE
//chenzhecong add for	pwm mode flashlight  start 
#if defined(HQ_TORCH_SET_PWM_MODE)
#include <linux/leds-mt65xx.h>
#endif
//renyufeng add for	pwm mode flashlight  end

    /*CAMERA-FLASH-EN */


extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
static void work_timeOutFunc(struct work_struct *data);

int FL_Enable(void)
{
#if defined(HQ_TORCH_SET_PWM_MODE)
		if(g_duty==0)
		{
			mt65xx_leds_brightness_set(MT65XX_LED_TYPE_TORCH, 255); 	
			//msleep(5);
			//mt65xx_leds_brightness_set(MT65XX_LED_TYPE_TORCH, 230); 	
			mt_set_gpio_out(GPIO_ENF,GPIO_OUT_ZERO);
			PK_DBG(" [ryf]FL_Enable line=%d,and g_duty=%d\n",__LINE__,g_duty);
		}
		/*
		else if(g_duty==1)
		{
			mt65xx_leds_brightness_set(MT65XX_LED_TYPE_TORCH, 60); 	
			mt_set_gpio_out(GPIO_ENF,GPIO_OUT_ONE);
			PK_DBG(" FL_Enable line=%d,and g_duty=%d\n",__LINE__,g_duty);
		}
		else if(g_duty==2)
		{
			mt65xx_leds_brightness_set(MT65XX_LED_TYPE_TORCH, 120); 	
			mt_set_gpio_out(GPIO_ENF,GPIO_OUT_ONE);
			PK_DBG(" FL_Enable line=%d,and g_duty=%d\n",__LINE__,g_duty);
		}
		else if(g_duty==3)
		{
			mt65xx_leds_brightness_set(MT65XX_LED_TYPE_TORCH, 180); 	
			mt_set_gpio_out(GPIO_ENF,GPIO_OUT_ONE);
			PK_DBG(" FL_Enable line=%d,and g_duty=%d\n",__LINE__,g_duty);
		}
		*/
		else
		{
			mt65xx_leds_brightness_set(MT65XX_LED_TYPE_TORCH, 225); 
			mt_set_gpio_out(GPIO_ENF,GPIO_OUT_ONE);
			PK_DBG("[ryf] FL_Enable line=%d,and g_duty=%d\n",__LINE__,g_duty);
		}
		//mt_set_gpio_out(GPIO_FAKE_FLASH,GPIO_OUT_ONE);
#else
		if(g_duty==0)
			{
				mt_set_gpio_out(GPIO_ENT,GPIO_OUT_ONE);
				mt_set_gpio_out(GPIO_ENF,GPIO_OUT_ZERO);
#if 0				
				mt65xx_leds_brightness_set(MT65XX_LED_TYPE_ISINK0, 1);
				mt65xx_leds_brightness_set(MT65XX_LED_TYPE_ISINK1, 1);
				mt65xx_leds_brightness_set(MT65XX_LED_TYPE_ISINK2, 1);
#endif				
			PK_DBG("[ryf] FL_Enable line=%d,and g_duty=%d\n",__LINE__,g_duty);
			}
			else
			{
				mt_set_gpio_out(GPIO_ENT,GPIO_OUT_ONE);
				mt_set_gpio_out(GPIO_ENF,GPIO_OUT_ONE);
#if 0				
				mt65xx_leds_brightness_set(MT65XX_LED_TYPE_ISINK0, 1);
				mt65xx_leds_brightness_set(MT65XX_LED_TYPE_ISINK1, 1);
				mt65xx_leds_brightness_set(MT65XX_LED_TYPE_ISINK2, 1);
#endif				
			PK_DBG("[ryf] FL_Enable line=%d,and g_duty=%d\n",__LINE__,g_duty);
			}
#endif
    return 0;
}

int FL_Disable(void)
{
#if defined(HQ_TORCH_SET_PWM_MODE)
	mt65xx_leds_brightness_set(MT65XX_LED_TYPE_TORCH, 0);	 
#else
	mt_set_gpio_out(GPIO_ENT,GPIO_OUT_ZERO);
#endif
	mt_set_gpio_out(GPIO_ENF,GPIO_OUT_ZERO);
	//mt_set_gpio_out(GPIO_FAKE_FLASH,GPIO_OUT_ZERO);
#if 0
	mt65xx_leds_brightness_set(MT65XX_LED_TYPE_ISINK0, 0);
	mt65xx_leds_brightness_set(MT65XX_LED_TYPE_ISINK1, 0);
	mt65xx_leds_brightness_set(MT65XX_LED_TYPE_ISINK2, 0);
#endif	
	PK_DBG(" FL_Disable line=%d\n",__LINE__);
    return 0;
}

int FL_dim_duty(kal_uint32 duty)
{
	g_duty=duty;
	PK_DBG(" FL_dim_duty line=%d\n",__LINE__);
    return 0;
}


int FL_Init(void)
{


	if(mt_set_gpio_mode(GPIO_ENF,GPIO_MODE_00)){PK_DBG("[constant_flashlight] set gpio mode failed!! \n");}
    if(mt_set_gpio_dir(GPIO_ENF,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
    if(mt_set_gpio_out(GPIO_ENF,GPIO_OUT_ZERO)){PK_DBG("[constant_flashlight] set gpio failed!! \n");}
    /*Init. to disable*/
#if defined(HQ_TORCH_SET_PWM_MODE)
    if(mt_set_gpio_mode(GPIO_ENT,GPIO_MODE_02)){PK_DBG("[constant_flashlight] set pwm mode failed!! \n");}
#else
    if(mt_set_gpio_mode(GPIO_ENT,GPIO_MODE_00)){PK_DBG("[constant_flashlight] set gpio mode failed!! \n");}
    if(mt_set_gpio_dir(GPIO_ENT,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
    if(mt_set_gpio_out(GPIO_ENT,GPIO_OUT_ZERO)){PK_DBG("[constant_flashlight] set gpio failed!! \n");}
#endif
   // if(mt_set_gpio_mode(GPIO_FAKE_FLASH,GPIO_MODE_00)){PK_DBG("[constant_flashlight] set gpio mode failed!! \n");}
   // if(mt_set_gpio_dir(GPIO_FAKE_FLASH,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
   // if(mt_set_gpio_out(GPIO_FAKE_FLASH,GPIO_OUT_ZERO)){PK_DBG("[constant_flashlight] set gpio failed!! \n");}
    INIT_WORK(&workTimeOut, work_timeOutFunc);
    PK_DBG(" FL_Init line=%d\n",__LINE__);
    return 0;
}


int FL_Uninit(void)
{
	FL_Disable();
    return 0;
}

int TORCH_Enable(void)
{
     #if defined(HQ_TORCH_SET_PWM_MODE)
	
		mt65xx_leds_brightness_set(MT65XX_LED_TYPE_TORCH, 255); 	
		//msleep(5);
		//mt65xx_leds_brightness_set(MT65XX_LED_TYPE_TORCH, 230); 	
		mt_set_gpio_out(GPIO_ENF,GPIO_OUT_ZERO);
		PK_DBG(" FL_Enable line=%d,and g_duty=%d\n",__LINE__,g_duty);			
		//mt_set_gpio_out(GPIO_FAKE_FLASH,GPIO_OUT_ONE);
    #else		
		mt_set_gpio_out(GPIO_ENT,GPIO_OUT_ONE);
		mt_set_gpio_out(GPIO_ENF,GPIO_OUT_ZERO);
		PK_DBG(" FL_Enable line=%d\n",__LINE__);			
    #endif
}


/*****************************************************************************
User interface
*****************************************************************************/

static void work_timeOutFunc(struct work_struct *data)
{
    FL_Disable();
    PK_DBG("ledTimeOut_callback\n");
    //printk(KERN_ALERT "work handler function./n");
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
	g_timeOutTimeMs=1000; //1s
	hrtimer_init( &g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
	g_timeOutTimer.function=ledTimeOutCallback;

}



static int constant_flashlight_ioctl(MUINT32 cmd, MUINT32 arg)
{
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;
	kdStrobeDrvArg kdArg;
	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC,0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC,0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC,0, int));
	PK_DBG("constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",__LINE__, ior_shift, iow_shift, iowr_shift, arg);
    switch(cmd)
    {

		case FLASH_IOC_SET_TIME_OUT_TIME_MS:
			PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n",arg);
			g_timeOutTimeMs=arg;
		break;


    	case FLASH_IOC_SET_DUTY :
    		PK_DBG("FLASHLIGHT_DUTY: %d\n",arg);
    		FL_dim_duty(arg);
    		break;


    	case FLASH_IOC_SET_STEP:
    		PK_DBG("FLASH_IOC_SET_STEP: %d\n",arg);

    		break;

    	case FLASH_IOC_SET_ONOFF :
    		PK_DBG("FLASHLIGHT_ONOFF: %d\n",arg);
    		if(arg==1)
    		{

    		    int s;
    		    int ms;
    		    if(g_timeOutTimeMs>1000)
            	{
            		s = g_timeOutTimeMs/1000;
            		ms = g_timeOutTimeMs - s*1000;
            	}
            	else
            	{
            		s = 0;
            		ms = g_timeOutTimeMs;
            	}
				if(g_timeOutTimeMs!=0)
	            {
	            	ktime_t ktime;
					ktime = ktime_set( 0, g_timeOutTimeMs*1000000 );
					hrtimer_start( &g_timeOutTimer, ktime, HRTIMER_MODE_REL );
	            }
    			FL_Enable();
    		}
    		else
    		{
    			FL_Disable();
				hrtimer_cancel( &g_timeOutTimer );
    		}
    		break;
	case FLASHLIGHTIOC_ENABLE_STATUS:
                printk("**********torch g_strobe_on = %d \n", g_strobe_On);
		  kdArg.arg = g_strobe_On;
                copy_to_user((void __user *) arg , (void*)&kdArg , sizeof(kdStrobeDrvArg));
                break;
	case FLASHLIGHT_TORCH_SELECT:
            printk("@@@@@@FLASHLIGHT_TORCH_SELECT@@@@@@\n");
            if (arg){
                    TORCH_Enable();
                    g_strobe_On = TRUE;
            } else {
                    FL_Disable();
                    g_strobe_On = FALSE;
            }
    		break;
		default :
    		PK_DBG(" No such command \n");
    		i4RetValue = -EPERM;
    		break;
    }
    return i4RetValue;
}




static int constant_flashlight_open(void *pArg)
{
    int i4RetValue = 0;
    PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res)
	{
	    FL_Init();
		timerInit();
	}
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
	spin_lock_irq(&g_strobeSMPLock);


    if(strobe_Res)
    {
        PK_ERR(" busy!\n");
        i4RetValue = -EBUSY;
    }
    else
    {
        strobe_Res += 1;
    }


    spin_unlock_irq(&g_strobeSMPLock);
    PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

    return i4RetValue;

}


static int constant_flashlight_release(void *pArg)
{
    PK_DBG(" constant_flashlight_release\n");

    if (strobe_Res)
    {
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


FLASHLIGHT_FUNCTION_STRUCT	constantFlashlightFunc=
{
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl
};


MUINT32 constantFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
    if (pfFunc != NULL)
    {
        *pfFunc = &constantFlashlightFunc;
    }
    return 0;
}



/* LED flash control for high current capture mode*/
ssize_t strobe_VDIrq(void)
{

    return 0;
}

EXPORT_SYMBOL(strobe_VDIrq);


