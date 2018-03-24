/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/* drivers/hwmon/mt6516/amit/tmd2771.c - TMD2771 ALS/PS driver
 * 
 * Author: MingHsien Hsieh <minghsien.hsieh@mediatek.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <ctp_alsps.h>
#include <alsps.h>
/******************************************************************************
 * configuration
*******************************************************************************/
static struct ctp_alsps_priv *ctp_alsps_obj =NULL;
static struct platform_driver ctp_alsps_alsps_driver;
  int (*ps_enable_nodata)(int en);
  int (*ps_get_data)(int* value, int* status);

/*----------------------------------------------------------------------------*/

/********************************************/





static long ctp_alsps_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{
	return 0;
}

static int ctp_alsps_release(struct inode *inode, struct file *file)
{
        file->private_data = NULL;
        return 0;
}

static int ctp_alsps_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}
static const struct file_operations ctp_alsps_fops = {
        .owner = THIS_MODULE,
        .open = ctp_alsps_open,
        .release = ctp_alsps_release,
        .unlocked_ioctl = ctp_alsps_unlocked_ioctl,
};

static struct miscdevice ctp_alsps_device = {
        .minor = MISC_DYNAMIC_MINOR,
        .name = "als_ps_tp",
        .fops = &ctp_alsps_fops,
};
static int ps_open_report_data(int open)
{
        //should queuq work to report event if  is_report_input_direct=true
        return 0;
}

static int ps_set_delay(u64 ns)
{
        return 0;
}

/*----------------------------------------------------------------------------*/
static int ctp_alsps_for_auto_local_init(void)
{	
	struct ctp_alsps_priv *obj;
	int err = 0;
	struct ps_control_path ps_ctl={0};
	struct ps_data_path ps_data={0};

	printk("hugo %s\n", __func__);
	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	memset(obj, 0, sizeof(*obj));
	ctp_alsps_obj = obj;

          err = misc_register(&ctp_alsps_device);
         if(err)
         {
                 printk("ctp_alsps_device register failed\n");
                 goto exit_misc_device_register_failed;
         }

        ps_ctl.open_report_data= ps_open_report_data;
        ps_ctl.enable_nodata = ps_enable_nodata;
        ps_ctl.set_delay  = ps_set_delay;
		/*Begin ersen.shang 20160113 revert these modifications for gsensor cts verifier test because update the firmware for ft6336s*/
        /*Begin ersen.shang 20151231 for cts verifier psensor testBatchAndFlush fail modify from true to false*/
        ps_ctl.is_report_input_direct = true;
        /*End   ersen.shang 20151231 for cts verifier psensor testBatchAndFlush fail modify from true to false*/
	    /*Begin ersen.shang 20151231 for cts verifier psensor testBatchAndFlush fail modify from 0 to 1*/
		ps_ctl.is_polling_mode = 0;
	    /*End   ersen.shang 20151231 for cts verifier psensor testBatchAndFlush fail modify from 0 to 1*/
		/*End   ersen.shang 20160113 revert these modifications for gsensor cts verifier test because update the firmware for ft6336s*/
		
#ifdef CUSTOM_KERNEL_SENSORHUB
        ps_ctl.is_support_batch = obj->hw->is_batch_supported_ps;
#else
	ps_ctl.is_support_batch = false;
#endif
        err = ps_register_control_path(&ps_ctl);
        if(err)
        {
                printk("register fail = %d\n", err);
                goto exit_sensor_obj_attach_fail;
        }
	
         ps_data.get_data = ps_get_data;
         ps_data.vender_div = 100;
         err = ps_register_data_path(&ps_data);
         if(err)
         {
                 printk("tregister fail = %d\n", err);
                 goto exit_sensor_obj_attach_fail;
         }

         err = batch_register_support_info(ID_PROXIMITY,ps_ctl.is_support_batch, 100, 0);
         if(err)
         {
                 printk("register proximity batch support err = %d\n", err);
         }

        printk("%s: OK\n", __func__);
/*Begin ersen.shang 20151231 for cts verifier psensor testBatchAndFlush fail move these code to line 138*/		
#if 0
	//ctp_alsps_obj.polling = 0; //llf change to interruption mode;
	ps_ctl.is_polling_mode = 1;
#endif	
/*End   ersen.shang 20151231 for cts verifier psensor testBatchAndFlush fail move these code to line 138*/
	return 0;
        exit_sensor_obj_attach_fail:
	exit_misc_device_register_failed:
		misc_deregister(&ctp_alsps_device);
		kfree(obj);
exit :
	return -1;
}
/*----------------------------------------------------------------------------*/

static int ctp_alsps_remove(void)
{
	APS_FUN();    
	return 0;
}

static struct alsps_init_info ctp_alsps_init_info = {
		.name = "ctp_alsps",
		.init = ctp_alsps_for_auto_local_init,
		.uninit = ctp_alsps_remove,
	
};
/*----------------------------------------------------------------------------*/
static int __init ctp_alsps_init(void)
{
	APS_FUN();
	alsps_driver_add(&ctp_alsps_init_info);
	return 0;
}
/*----------------------------------------------------------------------------*/

static void __exit ctp_alsps_exit(void)

{
	APS_FUN();
#if defined(MTK_AUTO_DETECT_ALSPS)
#else
	platform_driver_unregister(&ctp_alsps_alsps_driver);
#endif
}
/*----------------------------------------------------------------------------*/
module_init(ctp_alsps_init);
module_exit(ctp_alsps_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("xiaopu.zhu");
MODULE_DESCRIPTION("ctp simulate P-sensor driver");
MODULE_LICENSE("GPL");
