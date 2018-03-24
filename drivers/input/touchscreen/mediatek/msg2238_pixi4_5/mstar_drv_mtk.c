////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2006-2014 MStar Semiconductor, Inc.
// All rights reserved.
//
// Unless otherwise stipulated in writing, any and all information contained
// herein regardless in any format shall remain the sole proprietary of
// MStar Semiconductor Inc. and be kept in strict confidence
// (??MStar Confidential Information??) by the recipient.
// Any unauthorized act including without limitation unauthorized disclosure,
// copying, use, reproduction, sale, distribution, modification, disassembling,
// reverse engineering and compiling of the contents of MStar Confidential
// Information is unlawful and strictly prohibited. MStar hereby reserves the
// rights to any and all damages, losses, costs and expenses resulting therefrom.
//
////////////////////////////////////////////////////////////////////////////////

/**
 *
 * @file    mstar_drv_mtk.c
 *
 * @brief   This file defines the interface of touch screen
 *
 *
 */

/*=============================================================*/
// INCLUDE FILE
/*=============================================================*/

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/sched.h> ////
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>

#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/namei.h>
#include <linux/vmalloc.h>

#include "tpd.h"

#include <linux/dma-mapping.h>
#include <linux/gpio.h>

#ifdef TIMER_DEBUG
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#endif

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>

#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
#include <mach/md32_ipi.h>
#include <mach/md32_helper.h>
#endif



#include "mstar_drv_platform_interface.h"

/*=============================================================*/
// CONSTANT VALUE DEFINITION
/*=============================================================*/

#define MSG_TP_IC_NAME "msg2xxx" //"msg21xxA" or "msg22xx" or "msg26xxM" or "msg28xx" /* Please define the mstar touch ic name based on the mutual-capacitive ic or self capacitive ic that you are using */
#define I2C_BUS_ID   (1)       // i2c bus id : 0 or 1

#define TPD_OK (0)

/*=============================================================*/
// EXTERN VARIABLE DECLARATION
/*=============================================================*/

#ifdef CONFIG_TP_HAVE_KEY
extern const int g_TpVirtualKey[];

#ifdef CONFIG_ENABLE_REPORT_KEY_WITH_COORDINATE
extern const int g_TpVirtualKeyDimLocal[][4];
#endif //CONFIG_ENABLE_REPORT_KEY_WITH_COORDINATE
#endif //CONFIG_TP_HAVE_KEY

extern struct tpd_device *tpd;

int   MSG_tpd_proximity_detect = 1;//0-->close ; 1--> far away
int 	tpd_proximity_flag_suspend = 0;
int	tpd_proximity_flag = 0;

/*=============================================================*/
// LOCAL VARIABLE DEFINITION
/*=============================================================*/

/*
#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
static int tpd_wb_start_local[TPD_WARP_CNT] = TPD_WARP_START;
static int tpd_wb_end_local[TPD_WARP_CNT] = TPD_WARP_END;
#endif

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
static int tpd_calmat_local[8] = TPD_CALIBRATION_MATRIX;
static int tpd_def_calmat_local[8] = TPD_CALIBRATION_MATRIX;
#endif
*/
struct i2c_client *g_I2cClient = NULL;

#ifdef TPD_PROXIMITY
static int tpd_read_ps(void)
{
	//MSG_tpd_proximity_detect;
	return 0;    
}

static int tpd_get_ps_value(void)
{
	return MSG_tpd_proximity_detect;
}
extern s32 IicWriteData(u8 nSlaveId, u8* pBuf, u16 nSize);
static int tpd_enable_ps(int enable)
{
	u8 bWriteData[4] =
     {
         0x52, 0x00, 0x4a/*0x24*/, 0xA0
     };

     MSG_tpd_proximity_detect=1;

     if(enable==1)
     {
		tpd_proximity_flag =1;
		bWriteData[3] = 0xA0;
		printk("msg_ps enable ps  tpd_proximity_flag =%d\n", tpd_proximity_flag);

     }
     else
     {       
		tpd_proximity_flag = 0;
		bWriteData[3] = 0xA1;
	printk("msg_ps disable ps  tpd_proximity_flag =%d\n", tpd_proximity_flag);
     }
  
     IicWriteData(0x26, &bWriteData[0], 4);
	 return 0;
}

static int tpd_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data *sensor_data;
	
	DBG("[msg ps]command = 0x%02X\n", command);		
	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				DBG("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				DBG("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{	
				value = *(int *)buff_in;
				if(value)
				{		
					if((tpd_enable_ps(1) != 0))
					{
						DBG("enable ps fail: %d\n", err); 
						return -1;
					}
				}
				else
				{
					if((tpd_enable_ps(0) != 0))
					{
						DBG("disable ps fail: %d\n", err); 
						return -1;
					}
				}
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				DBG("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				
				sensor_data = (hwm_sensor_data *)buff_out;				
				
				if((err = tpd_read_ps()))
				{
					err = -1;;
				}
				else
				{
					sensor_data->values[0] = tpd_get_ps_value();
					DBG("huang sensor_data->values[0] 1082 = %d\n", sensor_data->values[0]);
					sensor_data->value_divide = 1;
					sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
				}	
				
			}
			break;
		default:
			DBG("proxmy sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;	
}
#endif

static const struct of_device_id msg2xxx_dt_match[] = {
	{.compatible = "mediatek,cap_touch"},
	{},
};
extern unsigned int msg_tpd_rst_gpio_number ;
extern unsigned int msg_tpd_int_gpio_number ;

static int of_get_msg2xxx_platform_data(struct device *dev)
{
	/*int ret, num;*/

	if (dev->of_node) {
		const struct of_device_id *match;

		match = of_match_device(of_match_ptr(msg2xxx_dt_match), dev);
		if (!match) {
			TPD_DMESG("Error: No device match found\n");
			return -ENODEV;
		}
	}
	//msg_tpd_rst_gpio_number = of_get_named_gpio(dev->of_node, "rst-gpio", 0);
	//msg_tpd_int_gpio_number = of_get_named_gpio(dev->of_node, "int-gpio", 0);
	/*ret = of_property_read_u32(dev->of_node, "rst-gpio", &num);
	if (!ret)
		tpd_rst_gpio_number = num;
	ret = of_property_read_u32(dev->of_node, "int-gpio", &num);
	if (!ret)
		tpd_int_gpio_number = num;
  */
	TPD_DMESG("g_vproc_en_gpio_number %d\n", msg_tpd_rst_gpio_number);
	TPD_DMESG("g_vproc_vsel_gpio_number %d\n", msg_tpd_int_gpio_number);
	return 0;
}

//static int boot_mode = 0;

/*=============================================================*/
// FUNCTION DECLARATION
/*=============================================================*/

/*=============================================================*/
// FUNCTION DEFINITION
/*=============================================================*/

/* probe function is used for matching and initializing input device */
static int /*__devinit*/ tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    s32 nRetVal = 0;

#ifdef TPD_PROXIMITY
    int error;
    struct hwmsen_object obj_ps;
#endif

    TPD_DMESG("TPD probe\n");   
    
    if (client == NULL)
    {
        TPD_DMESG("i2c client is NULL\n");
        return -1;
    }
    g_I2cClient = client;
	// g_I2cClient ->addr = 0x26;
    of_get_msg2xxx_platform_data(&client->dev);


    MsDrvInterfaceTouchDeviceSetIicDataRate(g_I2cClient, 100000); // 100 KHz


    nRetVal = MsDrvInterfaceTouchDeviceProbe(g_I2cClient, id);
	#ifdef TPD_PROXIMITY
	obj_ps.polling = 0;//interrupt mode
	obj_ps.sensor_operate = tpd_ps_operate;
	printk("proxi_fts attach\n");
	if((error = hwmsen_attach(ID_PROXIMITY, &obj_ps)))
	{
		DBG("proxi_fts attach fail = %d\n", error);
		printk("proxi_fts attach fail = %d\n", error);
	}
	else
	{
	//	g_alsps_name = tp_proximity; //ghq add 20140312
		DBG("proxi_fts attach ok = %d\n", error);
		printk("proxi_fts attach ok = %d\n", error);
	}		
#endif
    
    if (nRetVal == 0) // If probe is success, then enable the below flag.
    {
        tpd_load_status = 1;
    }    

    TPD_DMESG("TPD probe done\n");
    
    return TPD_OK;   
}

static int tpd_detect(struct i2c_client *client, struct i2c_board_info *info) 
{
    strcpy(info->type, TPD_DEVICE);    
//    strcpy(info->type, MSG_TP_IC_NAME);
    
    return TPD_OK;
}

static int /*__devexit*/ tpd_remove(struct i2c_client *client)
{   
    TPD_DEBUG("TPD removed\n");
    
    MsDrvInterfaceTouchDeviceRemove(client);
    
    return TPD_OK;
}

//static struct i2c_board_info __initdata i2c_tpd = {I2C_BOARD_INFO(MSG_TP_IC_NAME, (0x4C>>1))};

/* The I2C device list is used for matching I2C device and I2C device driver. */
static const struct i2c_device_id tpd_device_id[] =
{
    {MSG_TP_IC_NAME, 0},
    {}, /* should not omitted */ 
};

MODULE_DEVICE_TABLE(i2c, tpd_device_id);

static struct i2c_driver tpd_i2c_driver = {
    .driver = {
        .of_match_table = of_match_ptr(msg2xxx_dt_match),
        .name = MSG_TP_IC_NAME,
    },
    .probe = tpd_probe,
    .remove = tpd_remove,
    .id_table = tpd_device_id,
    .detect = tpd_detect,
};

static int tpd_local_init(void)
{  
    int retval;
    
    tpd->reg = regulator_get(tpd->tpd_dev, "vtouch");
    retval = regulator_set_voltage(tpd->reg, 2800000, 2800000);
    if (retval != 0) {
    	TPD_DMESG("Failed to set reg-vgp6 voltage: %d\n", retval);
    	return -1;
    }
    
    if (i2c_add_driver(&tpd_i2c_driver) != 0)
    {
        TPD_DMESG("unable to add i2c driver.\n");
         
        return -1;
    }
    
    if (tpd_load_status == 0) 
    {
        TPD_DMESG("add error touch panel driver.\n");

        i2c_del_driver(&tpd_i2c_driver);
        return -1;
    }

    if (tpd_dts_data.use_tpd_button) {
    	tpd_button_setting(tpd_dts_data.tpd_key_num, tpd_dts_data.tpd_key_local,
    	tpd_dts_data.tpd_key_dim_local);
    }

#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
	TPD_DO_WARP = 1;
	memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT * 4);
	memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT * 4);
#endif

#if (defined(CONFIG_TPD_HAVE_CALIBRATION) && !defined(CONFIG_TPD_CUSTOM_CALIBRATION))

	memcpy(tpd_calmat, tpd_def_calmat_local_factory, 8 * 4);
	memcpy(tpd_def_calmat, tpd_def_calmat_local_factory, 8 * 4);

	memcpy(tpd_calmat, tpd_def_calmat_local_normal, 8 * 4);
	memcpy(tpd_def_calmat, tpd_def_calmat_local_normal, 8 * 4);

#endif

	TPD_DMESG("TPD init done end %s, %d\n", __func__, __LINE__);
	tpd_type_cap = 1;
    
    return TPD_OK; 
}

static void tpd_resume(struct device *h)
{
    TPD_DMESG("TPD wake up\n");
    
#ifdef TPD_PROXIMITY
	DBG("msg_ps resume tpd_proximity_flag =%d\n", tpd_proximity_flag);

	if(tpd_proximity_flag_suspend ==1)
	{
		return ;
	}
#endif
    MsDrvInterfaceTouchDeviceResume(h);
#ifdef TPD_PROXIMITY
	if(tpd_proximity_flag)   /////reopen ps
	{
		tpd_enable_ps(1);
	 }
#endif
    
    TPD_DMESG("TPD wake up done\n");
}

static void tpd_suspend(struct device *h)
{
    TPD_DMESG("TPD enter sleep\n");
#ifdef TPD_PROXIMITY
	DBG("msg_ps suspend tpd_proximity_flag =%d\n", tpd_proximity_flag);

	tpd_proximity_flag_suspend = tpd_proximity_flag;
	if(tpd_proximity_flag_suspend ==1)
	{
		return ;
	}
#endif

    MsDrvInterfaceTouchDeviceSuspend(h);

    TPD_DMESG("TPD enter sleep done\n");
} 


static struct device_attribute *msg2xxx_attrs[] = {
#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
	&dev_attr_tpd_scp_ctrl,
#endif
};

static struct tpd_driver_t tpd_device_driver = {
     .tpd_device_name = MSG_TP_IC_NAME,
     .tpd_local_init = tpd_local_init,
     .suspend = tpd_suspend,
     .resume = tpd_resume,
	.attrs = {
		.attr = msg2xxx_attrs,
		.num  = ARRAY_SIZE(msg2xxx_attrs),
	},
};

static int __init tpd_driver_init(void) 
{
    TPD_DMESG("Mstar touch panel driver init\n");

    tpd_get_dts_info();
    if (tpd_driver_add(&tpd_device_driver) < 0)
        TPD_DMESG("TPD add Mstar TP driver failed\n");
    return 0;
}
 
static void __exit tpd_driver_exit(void) 
{
    TPD_DMESG("touch panel driver exit\n");
    
    tpd_driver_remove(&tpd_device_driver);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);
MODULE_LICENSE("GPL");
