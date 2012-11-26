/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation  
*/
/*=============================================================================
                        �ύX����

 �ԍ�                ���t      �R�����g
------------------   --------  --------------------------------
G01-SWD0275-00006   11/11/24 WiFi �d�����M�����[�^�iPMIC�j���䏈���ǉ�
G01-SWD0275-00007   11/11/28 MAC�A�h���X�������ݏ����ǉ�
=============================================================================*/
#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/io.h>

#include <linux/mfd/pmic8058.h>
#include <mach/msm_iomap.h>
#include <mach/pmic.h>
#include <mach/rpc_pmapp.h>
#include <mach/vreg.h>
#include "pm.h"
#include "spm.h"

#include <linux/proc_fs.h>
#include <asm/uaccess.h>

/* For msm_batt_oem_update_dev_info_wlan function prototype. */
#include <mach/kcj_dev_info.h>

/* G01-SWD0275-00007 MAC�A�h���X�������ݏ����ǉ� */
#include <proc_comm.h>

#include <mach/kcj_wifi_bluetooth.h>

#define PM8058_GPIO_SLEEP_CLK 37 /* PMIC GPIO 38 */

/* Macros assume PMIC GPIOs start at 0 */
#define PM8058_GPIO_PM_TO_SYS(pm_gpio)     (pm_gpio + NR_GPIO_IRQS)
#define PM8058_GPIO_SYS_TO_PM(sys_gpio)    (sys_gpio - NR_GPIO_IRQS)

/* PMIC��SleepClock�ݒ�֐� */
static int wifi_sleep_clock_initialize( void );

/* /proc/kyocera_wifi/power �ւ̃��[�h�A�N�Z�X�n���h�� */
static int kcj_wifi_power_initialize_read(char *page, char **start, off_t offset,
					int count, int *eof, void *data);
/* /proc/kyocera_wifi/power �ւ̃��C�g�A�N�Z�X�n���h�� */
static int kcj_wifi_power_initialize_write(struct file *file, const char *buffer,
					unsigned long count, void *data);
/* /proc/kyocera_wifi/debug �ւ̃��[�h�A�N�Z�X�n���h�� */
static int kcj_wifi_debug_read(char *page, char **start, off_t offset,
					int count, int *eof, void *data);
/* /proc/kyocera_wifi/debug �ւ̃��[�h�A�N�Z�X�n���h�� */
static int kcj_wifi_debug_write(struct file *file, const char *buffer,
					unsigned long count, void *data);

/* G01-SWD0275-00007 MAC�A�h���X�������ݏ����ǉ� ADD-S */
static int bluepower_read_proc_bt_addr(char *page, char **start, off_t offset,
					int count, int *eof, void *data);
static int bluepower_read_proc_wlan_mac_addr(char *page, char **start, off_t offset,
					int count, int *eof, void *data);
static int read_proc_product_line(char *page, char **start, off_t offset,
					int count, int *eof, void *data);
static int bluepower_read_proc_wlan_mac_addr(char *page, char **start, off_t offset,
					int count, int *eof, void *data);
static int bluepower_read_proc_wlan_mac_addr_qcom(char *page, char **start, off_t offset,
					int count, int *eof, void *data);
static int bluepower_read_proc_wlan_mac_addr_common(char *page, char **start, off_t offset,
					int count, int *eof, void *data, int format_mode);

static int check_product_line(void);
/* G01-SWD0275-00007 MAC�A�h���X�������ݏ����ǉ� ADD-E */

/* Virtual file system read handler(/proc/kyocera_wifi/wifi_state) be add. */
static int kcj_wifi_state_read(char *page, char **start, off_t offset,
					int count, int *eof, void *data);
/* Virtual file system write handler(/proc/kyocera_wifi/wifi_state) be add. */
static int kcj_wifi_state_write(struct file *file, const char *buffer,
					unsigned long count, void *data);

/* /proc/kyocera_wifi�ւ̃t�H���_�n���h�� */
struct proc_dir_entry *kyocera_wifi_dir;

/* Wifi operation state ( WiFi_OFF , WiFi_Station_mode , WiFi_Hotspot) */
static int wifi_current_operation_state = 0;

/*
   DTV�֘A�̕s�v�����Ɏ�������Ă��郌�M�����[�^���䂪�Ȃ��ƁAWiFi���N���ł��Ȃ�����
   WiFi�̓d�����䂪���������܂ł̊ԁA�ȉ���Qualcomm�s�v������L��������B
   �֘A���� Redmine�`�P�b�g�ԍ��́A#6975 
*/

struct pm8xxx_gpio_init_info {
	unsigned			gpio;
	struct pm_gpio			config;
};

static struct pm8xxx_gpio_init_info pmic_quickvx_clk_gpio = {
	PM8058_GPIO_PM_TO_SYS(PM8058_GPIO_SLEEP_CLK),
	{
		.direction      = PM_GPIO_DIR_OUT,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.output_value   = 1,
		.pull           = PM_GPIO_PULL_NO,
		.vin_sel        = PM8058_GPIO_VIN_S3,
		.out_strength   = PM_GPIO_STRENGTH_HIGH,
		.function       = PM_GPIO_FUNC_2,
	},
};

/* Wifi operation status. */
enum KCJ_WIFI_OPERATION_TYPE {
	KCJ_WIFI_OPERATION_WiFi_OFF = 0,     /* WiFi-OFF  */
	KCJ_WIFI_OPERATION_WiFi_STATION = 1, /* WiFi-Station mode */
	KCJ_WIFI_OPERATION_WiFi_HOTSPOT = 2, /* WiFi-Hotspot mode */
};

/*@**************************************************************************
�֐���   : wifi_sleep_clock_initialize
�@�\     : board-msm7x30.c�ŁAPMIC��SleepClk��������Ă����������ڐA
           WiFION�O�ɌĂяo�����K�v������
����     : �Ȃ�
�߂�l   : 0 �Ȃ琬�� 0 �ȊO�Ȃ玸�s
****************************************************************************/
static int wifi_sleep_clock_initialize( void )
{
	int rc = 0;

	rc = pm8xxx_gpio_config(pmic_quickvx_clk_gpio.gpio,
				&pmic_quickvx_clk_gpio.config);
	if (rc) {
		pr_err("%s: pm8xxx_gpio_config(%#x)=%d\n",
			__func__, pmic_quickvx_clk_gpio.gpio,
			rc);
		return rc;
	}

	gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(
		PM8058_GPIO_SLEEP_CLK), 0);

	return rc;
}

/*@**************************************************************************
�֐���   : kyocera_wifi_init
�@�\     : ���z�t�@�C���V�X�e��(/proc)�ɁAWiFi�p�̃G���g�����쐬����
����     : �Ȃ�
�߂�l   : 0 �Ȃ琬��  -ENOMEM�Ȃ玸�s
****************************************************************************/
int kyocera_wifi_init(void)
{
	int retval = 0;
	struct proc_dir_entry *ent;

	/* Creating directory "kyocera_wifi" entry */
	kyocera_wifi_dir = proc_mkdir("kyocera_wifi", NULL);
	if (kyocera_wifi_dir == NULL) {
		pr_err("Unable to create /proc/kyocera_wifi directory");
		return -ENOMEM;
	}

	/* Creating read/write "power" entry */
	ent = create_proc_entry("power", 0666, kyocera_wifi_dir);
	if (ent == NULL) {
		pr_err("Unable to create /proc/kyocera_wifi/power entry" );
		retval = -ENOMEM;
		goto fail;
	}
	ent->read_proc = kcj_wifi_power_initialize_read;
	ent->write_proc = kcj_wifi_power_initialize_write;

	/* Creating read/write "debug" entry */
	ent = create_proc_entry("debug", 0666, kyocera_wifi_dir);
	if (ent == NULL) {
		pr_err("Unable to create /proc/kyocera_wifi/debug entry");
		retval = -ENOMEM;
		goto fail;
	}
	ent->read_proc = kcj_wifi_debug_read;
	ent->write_proc = kcj_wifi_debug_write;

/* G01-SWD0275-00007 MAC�A�h���X�������ݏ����ǉ� ADD-S */
	/* read only proc entries */
	if (create_proc_read_entry("bd_addr", 0444, kyocera_wifi_dir,
		bluepower_read_proc_bt_addr, NULL) == NULL) {
		pr_err("Unable to create /proc/kyocera_wifi/bd_addr entry");
		retval = -ENOMEM;
		goto fail;
	}

	/* read only proc entries */
	if (create_proc_read_entry("wlan_mac_addr", 0444, kyocera_wifi_dir,
		bluepower_read_proc_wlan_mac_addr, NULL) == NULL) {
		pr_err("Unable to create /proc/kyocera_wifi/wlan_mac_addr entry" );
		retval = -ENOMEM;
		goto fail;
	}

	/* read only proc entries */
	if (create_proc_read_entry("wlan_mac_addr_qcom", 0444, kyocera_wifi_dir,
		bluepower_read_proc_wlan_mac_addr_qcom, NULL) == NULL) {
		pr_err("Unable to create /proc/kyocera_wifi/wlan_mac_addr_qcom entry" );
		retval = -ENOMEM;
		goto fail;
	}

	/* read only proc entries */
	if (create_proc_read_entry("product_line", 0444, kyocera_wifi_dir,
		read_proc_product_line, NULL) == NULL) {
		pr_err("Unable to create /proc/kyocera_wifi/product_line entry" );
		retval = -ENOMEM;
		goto fail;
	}

	/* Creating read/write "wifi_state" entry */
	ent = create_proc_entry("wifi_state", 0666, kyocera_wifi_dir);
	if (ent == NULL) {
		pr_err("Unable to create /proc/kyocera_wifi/wifi_state entry" );
		retval = -ENOMEM;
		goto fail;
	}
	ent->read_proc = kcj_wifi_state_read;
	ent->write_proc = kcj_wifi_state_write;

	return 0;

fail:
	remove_proc_entry("wlan_mac_addr_qcom", kyocera_wifi_dir );
	remove_proc_entry("wifi_state", kyocera_wifi_dir );
	remove_proc_entry("product_line", kyocera_wifi_dir );
	remove_proc_entry("wlan_mac_addr", kyocera_wifi_dir );
	remove_proc_entry("bd_addr", kyocera_wifi_dir );
/* G01-SWD0275-00007 MAC�A�h���X�������ݏ����ǉ� ADD-E */

	remove_proc_entry("debug", kyocera_wifi_dir );
	remove_proc_entry("power", kyocera_wifi_dir );
	return retval;
}

/*@**************************************************************************
�֐���   : kcj_wifi_power_initialize_read
�@�\     : SleepClockSetting��������������Ă��邱�Ƃ�W���o�͂ɏo�͂���
����     : (O)   char*  page   0:OFF   1:ON ���g�p
           (I)   char** start  ���g�p
           (I)   off_t  ifset  ���g�p
           (I)   int    count  ���g�p
           (I)   int*   eof    1��ݒ�
           (I)   void*  data   ���g�p
�߂�l   : page�ɐݒ肷��T�C�Y
           0 :���s
****************************************************************************/
static int kcj_wifi_power_initialize_read(char *page, char **start, off_t offset,
					int count, int *eof, void *data)
{
	*eof = 1;
	return sprintf(page, "kcj_wifi_power_initialize_read\n");
}
/*@**************************************************************************
�֐���   : kcj_wifi_power_initialize_write
�@�\     : *buffer�̐擪��1�ł���΁AWiFi�p��SleepClock��ݒ肷��
����     : struct file *file ���g�p
           const char *buffer �������ݗv�����ꂽ�f�[�^
           unsigned long count �f�[�^�T�C�Y
           void *data ���g�p

�߂�l   : page�ɐݒ肷��T�C�Y
           0 :���s
****************************************************************************/
static int kcj_wifi_power_initialize_write(struct file *file, const char *buffer,
					unsigned long count, void *data)
{
	char *buf;

	if (count < 1)
		return -EINVAL;

	buf = kmalloc(count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	if (copy_from_user(buf, buffer, count)) {
		kfree(buf);
		return -EFAULT;
	}

	if (buf[0] == '0') {
/*		wifi_sleep_clock_initialize(); */
	} else if (buf[0] == '1') {
		wifi_sleep_clock_initialize();
		printk(KERN_WARNING "wifi_sleep_clock_initialize occured.\n");
	} else {
		kfree(buf);
		return -EINVAL;
	}

	kfree(buf);
	return count;
}

/*@**************************************************************************
�֐���   : kcj_wifi_state_read
�@�\     : Wifi status would be read.
����     : (O)   char*  page   0:OFF   1:ON
           (I)   char** start  ���g�p
           (I)   off_t  ifset  ���g�p
           (I)   int    count  ���g�p
           (I)   int*   eof    1��ݒ�
           (I)   void*  data   ���g�p
�߂�l   : page�ɐݒ肷��T�C�Y
           0 :���s
****************************************************************************/
static int kcj_wifi_state_read(char *page, char **start, off_t offset,
					int count, int *eof, void *data)
{
	/* Current WiFi status parameter would return. */
    return sprintf(page, "%d", wifi_current_operation_state );
}

/*@**************************************************************************
�֐���   : kcj_wifi_state_write
�@�\     : Wifi status would be set.
����     : (I) struct file *file ���g�p
           (I) const char *buffer �������ݗv�����ꂽ�f�[�^
           (I) unsigned long count �f�[�^�T�C�Y
           (I) void *data ���g�p

�߂�l   : page�ɐݒ肷��T�C�Y
           0 :���s
****************************************************************************/
static int kcj_wifi_state_write(struct file *file, const char *buffer,
					unsigned long count, void *data)
{
	char *buf;

	if (count < 1)
		return -EINVAL;

	buf = kmalloc(count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	if (copy_from_user(buf, buffer, count)) {
		kfree(buf);
		return -EFAULT;
	}

	if (buf[0] == '0')
	{
		printk(KERN_WARNING "Wifi status is direct changed to 0.\n");
		kcj_wifi_set_current_operation_state(0);
	}
	else if (buf[0] == '1')
	{
		printk(KERN_WARNING "Wifi status is direct changed to 1.\n");
		kcj_wifi_set_current_operation_state(1);
	}
	else if (buf[0] == '2')
	{
		printk(KERN_WARNING "Wifi status is direct changed to 2.\n");
		kcj_wifi_set_current_operation_state(2);
	}
	else
	{
		kfree(buf);
		return -EINVAL;
	}

	kfree(buf);
	return count;
}


/*@**************************************************************************
�֐���   : kcj_wifi_debug_read
�@�\     : WiFi�f�o�b�O�p�̃G���g���B ���b�Z�[�W���o�͂���
����     : (O)   char*  page   0:OFF   1:ON
           (I)   char** start  ���g�p
           (I)   off_t  ifset  ���g�p
           (I)   int    count  ���g�p
           (I)   int*   eof    1��ݒ�
           (I)   void*  data   ���g�p
�߂�l   : page�ɐݒ肷��T�C�Y
           0 :���s
****************************************************************************/
static int kcj_wifi_debug_read(char *page, char **start, off_t offset,
					int count, int *eof, void *data)
{
	*eof = 1;
	return sprintf(page, "kcj_wifi_debug_read\n");
}

/*@**************************************************************************
�֐���   : kcj_wifi_debug_write
�@�\     : WiFi�f�o�b�O�p�̃G���g���B 1���������܂��ƁA�W���o�͂Ƀ��b�Z�[�W���o��
����     : (I) struct file *file ���g�p
           (I) const char *buffer �������ݗv�����ꂽ�f�[�^
           (I) unsigned long count �f�[�^�T�C�Y
           (I) void *data ���g�p

�߂�l   : page�ɐݒ肷��T�C�Y
           0 :���s
****************************************************************************/
static int kcj_wifi_debug_write(struct file *file, const char *buffer,
					unsigned long count, void *data)
{
	char *buf;

	if (count < 1)
		return -EINVAL;

	buf = kmalloc(count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	if (copy_from_user(buf, buffer, count)) {
		kfree(buf);
		return -EFAULT;
	}

	if (buf[0] == '0') {
	} else if (buf[0] == '1') {
		printk(KERN_WARNING "kcj_wifi_debug_write occured.\n");
	} else {
		kfree(buf);
		return -EINVAL;
	}

	kfree(buf);
	return count;
}

/* G01-SWD0275-00007 MAC�A�h���X�������ݏ����ǉ� ADD-S */
/* F17-SWD0254-00007 ADD-S */
/****************************************************************************
*  �֐���   : bluepower_read_proc_bt_addr                                  
*  �@�\     : Bluetooth�A�h���X��NV���ǂݏo��                            
*  ����     : (O)   char*  page   Bluetooth�A�h���X��0x�t���Ŋi�[           
            : (I)   char** start  ���g�p                                   
            : (I)   off_t  ifset  ���g�p                                   
            : (I)   int    count  ���g�p                                   
            : (I)   int*   eof    1��ݒ�                                  
            : (I)   void*  data   ���g�p                                   
*  �߂�l   : page�ɐݒ肷��T�C�Y                                         
*           : 0 :���s                                                      
****************************************************************************/
static int bluepower_read_proc_bt_addr(char *page, char **start, off_t offset,
					int count, int *eof, void *data)
{
	unsigned int read_bd_addr[2];		  /* BD�A�h���X�i�[(�ǂݍ���)�p */
	unsigned char write_bd_addr[6];		 /* BD�A�h���X�i�[(��������)�p */
	int ret;								/* �߂�l�i�[�p               */
	unsigned int cmd;

	/* ������ */
	memset(read_bd_addr ,0 ,sizeof(read_bd_addr ));
	memset(write_bd_addr,0 ,sizeof(write_bd_addr));
	ret = -1;

	*eof = 1;

    cmd = 0;
	/* BD_ADDR�ǂݏo�� */
	ret = msm_proc_comm(PCOM_CUSTOMER_CMD3  , &cmd
											,  &read_bd_addr[0]);

    if( ret != 0)	 /* NV����ǂݏo�����s */
	{
		/* �O�̂��ߏ����� NV���擾���s���͖����I��0���i�[���� */
		memset(read_bd_addr,0,sizeof(read_bd_addr));
	}
    cmd = 1;
	ret = msm_proc_comm(PCOM_CUSTOMER_CMD3  , &cmd
											, &read_bd_addr[1]);

	if( ret != 0)	 /* NV����ǂݏo�����s */
	{
		/* �O�̂��ߏ����� NV���擾���s���͖����I��0���i�[���� */
		memset(read_bd_addr,0,sizeof(read_bd_addr));
	}

	/* �擾BT�A�h���X���o�͌`����CHAR�̔z��Ɋi�[ */
	write_bd_addr[0] =  read_bd_addr[0] & 0x000000FF ;
	write_bd_addr[1] = (read_bd_addr[0] & 0x0000FF00 ) >> 8;
	write_bd_addr[2] = (read_bd_addr[0] & 0x00FF0000 ) >> 16;
	write_bd_addr[3] = (read_bd_addr[0] & 0xFF000000 ) >> 24;
	write_bd_addr[4] =  read_bd_addr[1] & 0x000000FF ;
	write_bd_addr[5] = (read_bd_addr[1] & 0x0000FF00 ) >> 8;

	/* :��؂�ɂ���BD�A�h���X��ݒ� */
	return sprintf(page, "%02x:%02x:%02x:%02x:%02x:%02x", write_bd_addr[0], \
														  write_bd_addr[1], \
														  write_bd_addr[2], \
														  write_bd_addr[3], \
														  write_bd_addr[4], \
														  write_bd_addr[5]);
}

static int bluepower_read_proc_wlan_mac_addr_qcom(char *page, char **start, off_t offset,
					int count, int *eof, void *data)
{
	return bluepower_read_proc_wlan_mac_addr_common(page, start, offset, count, eof, data , 0);
}

static int bluepower_read_proc_wlan_mac_addr(char *page, char **start, off_t offset,
					int count, int *eof, void *data)
{
	return bluepower_read_proc_wlan_mac_addr_common(page, start, offset, count, eof, data , 1);
}

/****************************************************************************
*  �֐���   : bluepowerr_read_proc_wlan_mac_addr                           
*  �@�\     : WLAN MAC�A�h���X��NV���ǂݏo��                             
*  ����     : (O)   char*  page   MAC�A�h���X��0x�t���Ŋi�[                
            : (I)   char** start  ���g�p                                   
            : (I)   off_t  ifset  ���g�p                                   
            : (I)   int    count  ���g�p                                   
            : (I)   int*   eof    1��ݒ�                                  
            : (I)   void*  data   ���g�p                                   
*  �߂�l   : page�ɐݒ肷��T�C�Y                                         
*           : 0 :���s                                                      
****************************************************************************/
static int bluepower_read_proc_wlan_mac_addr_common(char *page, char **start, off_t offset,
					int count, int *eof, void *data , int format_mode)
{
	unsigned int read_wlan_mac_addr[2];			/* MAC�A�h���X�i�[(�ǂݍ���)�p */
	unsigned char write_wlan_mac_addr[6];		/* MAC�A�h���X�i�[(��������)�p */
	int ret;									/* �߂�l�i�[�p               */
	unsigned int cmd;

	/* ������ */
	memset(read_wlan_mac_addr ,0 ,sizeof(read_wlan_mac_addr ));
	memset(write_wlan_mac_addr,0 ,sizeof(write_wlan_mac_addr));
	ret = -1;
	*eof = 1;
    cmd = 2;
	/* MAC_ADDR�ǂݏo�� */
	ret = msm_proc_comm(PCOM_CUSTOMER_CMD3, &cmd, 
											&read_wlan_mac_addr[0]
			);
	if( ret != 0)	 /* NV����ǂݏo�����s */
	{

		/* �O�̂��ߏ����� NV���擾���s���͖����I��0���i�[���� */
		memset(read_wlan_mac_addr,0,sizeof(read_wlan_mac_addr));
	}
    cmd = 3;
	/* MAC_ADDR�ǂݏo�� */
	ret = msm_proc_comm(PCOM_CUSTOMER_CMD3, &cmd, 
											&read_wlan_mac_addr[1]
															);
/* F17-SWD0254-00009 ADD_S */
    if(( ret != 0) || 	 /* NV����ǂݏo�����s */
       ((0x00000000 == (read_wlan_mac_addr[0] & 0xFFFFFFFF)) &&  /* MAC�A�h���X��0 */
        (0x00000000 == (read_wlan_mac_addr[1] & 0x0000FFFF))
      ))
/* F17-SWD0254-00009 ADD_E */
	{

		/* �O�̂��ߏ����� NV���擾���s���͖����I��0���i�[���� */
		memset(read_wlan_mac_addr,0,sizeof(read_wlan_mac_addr));
/* F17-SWD0254-00009 ADD_S */
		/* �H�����t���O�`�F�b�N */
		if(check_product_line() != 1)
		{
			/* �H�����t���O��ON�ȊO��NV����̏ꍇ��ZZZZZZ���i�[ */
/* G01-SWD0275-00007 MAC�A�h���X�������ݏ����ǉ� MOD */
			return sprintf(page, "ZZZZZZZZ"); 
		}

	    /* �H�����t���O��ON���擾����NV�A�h���X��ALL0�ł���ꍇ�A�擪�o�C�g��1�Ƃ��� */
		/* �H�����t���O��OFF�̏ꍇ�̓h���C�o��MAC�A�h���X�ݒ菈�����ŃG���[�Ή�        */
		else{
/* G01-SWD0275-00007 MAC�A�h���X�������ݏ����ǉ� MOD */
			return sprintf(page, "100000000000"); 
		}
/* F17-SWD0254-00009 ADD_E */
	}

	/* �擾BT�A�h���X���o�͌`����CHAR�̔z��Ɋi�[ */
	write_wlan_mac_addr[0] =  read_wlan_mac_addr[0] & 0x000000FF ;
	write_wlan_mac_addr[1] = (read_wlan_mac_addr[0] & 0x0000FF00 ) >> 8;
	write_wlan_mac_addr[2] = (read_wlan_mac_addr[0] & 0x00FF0000 ) >> 16;
	write_wlan_mac_addr[3] = (read_wlan_mac_addr[0] & 0xFF000000 ) >> 24;
	write_wlan_mac_addr[4] =  read_wlan_mac_addr[1] & 0x000000FF ;
	write_wlan_mac_addr[5] = (read_wlan_mac_addr[1] & 0x0000FF00 ) >> 8;

	/* ��؂�Ȃ���MAC�A�h���X��ݒ� */
/* G01-SWD0275-00007 MAC�A�h���X�������ݏ����ǉ� MOD */
	/* If format_mode contains '1' , string will return for part of wifi station name. */
	if ( format_mode == 1 )
	{
		return sprintf(page, "macaddr=%02x:%02x:%02x:%02x:%02x:%02x", 
	                                                      write_wlan_mac_addr[0],
														  write_wlan_mac_addr[1],
														  write_wlan_mac_addr[2],
														  write_wlan_mac_addr[3],
														  write_wlan_mac_addr[4],
														  write_wlan_mac_addr[5]);
	}
	else 
	{
		return sprintf(page, "%02x%02x%02x%02x%02x%02x", 
	                                                      write_wlan_mac_addr[0],
														  write_wlan_mac_addr[1],
														  write_wlan_mac_addr[2],
														  write_wlan_mac_addr[3],
														  write_wlan_mac_addr[4],
														  write_wlan_mac_addr[5]);
	}
}
/* F17-SWD0254-00007 ADD-E */

/* SWD0312-00001 ADD-S */
/****************************************************************************
*  �֐���   : read_proc_product_line
*  �@�\     : �H�����t���OON/OFF�̏�Ԃ��t�@�C���V�X�e���ɏ����o��                            
*  ����     : (O)   char*  page   0:OFF   1:ON 
            : (I)   char** start  ���g�p                                   
            : (I)   off_t  ifset  ���g�p                                   
            : (I)   int    count  ���g�p                                   
            : (I)   int*   eof    1��ݒ�                                  
            : (I)   void*  data   ���g�p                                   
*  �߂�l   : page�ɐݒ肷��T�C�Y                                         
*           : 0 :���s                                                      
****************************************************************************/
static int read_proc_product_line(char *page, char **start, off_t offset,
					int count, int *eof, void *data)
{
    int ret = 0;
    
    ret = check_product_line();
    
    return sprintf(page, "%d", ret);    
}
/* SWD0312-00001 ADD-E*/

/* F17-SWD0254-00009 ADD_S */
/****************************************************************************
*  �֐���   : check_product_line                                           
*  �@�\     : �H�����t���O��ON���ǂ����擾����                             
*  ����     : none                                                         
*  �߂�l   : 1 :�H�����t���OON                                            
*           : 0 :�H�����t���OOFF                                           
****************************************************************************/
static int check_product_line(void)
{
    unsigned int read_product_line[2];		/* �H�����t���O(�ǂݍ���)�p */
	unsigned char check_product[8];			/* �H�����t���O(�`�F�b�N)�p */
	int ret;								/* �߂�l�i�[�p             */
	unsigned int cmd;                       /* NV�擾�p�R�}���h         */

	/* ������ */
	memset(read_product_line ,0 ,sizeof(read_product_line ));
	memset(check_product,0 ,sizeof(check_product));
	ret = 0;

    /* �H�����t���O�`�F�b�N */
    cmd = 4;
	/* �H�����t���O�ǂݏo�� */
	ret = msm_proc_comm(PCOM_CUSTOMER_CMD3, &cmd, 
											&read_product_line[0]
			);
	if( ret != 0)	 /* NV����ǂݏo�����s */
	{

		/* �O�̂��ߏ����� NV���擾���s���͖����I��0���i�[���� */
		memset(read_product_line,0,sizeof(read_product_line));
	}
    cmd = 5;
	/* �H�����t���O�ǂݏo�� */
	ret = msm_proc_comm(PCOM_CUSTOMER_CMD3, &cmd, 
											&read_product_line[1]
			);

	if( ret != 0)	 /* NV����ǂݏo�����s */
	{

		/* �O�̂��ߏ����� NV���擾���s���͖����I��0���i�[���� */
		memset(read_product_line,0,sizeof(read_product_line));
	}

	/* �擾�H�����t���O���o�͌`����CHAR�̔z��Ɋi�[ */
	check_product[0] =  read_product_line[0] & 0x000000FF ;
	check_product[1] = (read_product_line[0] & 0x0000FF00 ) >> 8;
	check_product[2] = (read_product_line[0] & 0x00FF0000 ) >> 16;
	check_product[3] = (read_product_line[0] & 0xFF000000 ) >> 24;
	check_product[4] =  read_product_line[1] & 0x000000FF ;
	check_product[5] = (read_product_line[1] & 0x0000FF00 ) >> 8;
	check_product[6] = (read_product_line[1] & 0x00FF0000 ) >> 16;
	check_product[7] = (read_product_line[1] & 0xFF000000 ) >> 24;

	/* �H�����t���O��ON���̃`�F�b�N���s��                         */
	/* �H�����t���O��ON�͉��L�̏��                               */
	/* NV_PRODUCT_LINE_I,0x37,0x37,0x32,0x37,0x34,0x35,0x32,0x37, */
	if( ( 0x37 == check_product[0])  &&
		( 0x37 == check_product[1])  &&
		( 0x32 == check_product[2])  &&
		( 0x37 == check_product[3])  &&
		( 0x34 == check_product[4])  &&
		( 0x35 == check_product[5])  &&
		( 0x32 == check_product[6])  &&
		( 0x37 == check_product[7])
	){
		ret = 1;
	}
	return ret;
}
/* F17-SWD0254-00009 ADD_E */
/* G01-SWD0275-00007 MAC�A�h���X�������ݏ����ǉ� ADD-E */

/****************************************************************************
*  �֐���   : kcj_wifi_set_current_operation_state                         
*  �@�\     : If changed wifi current state then inform to 'vbatt software driver'.
*  ����     : 0 - Wifi OFF 
*             1 - Wifi Station mode
*             2 - Wifi Hotspot mode                                        
*  �߂�l   : none                                                         
****************************************************************************/
void kcj_wifi_set_current_operation_state(int state)
{
	switch (state)
	{
		case KCJ_WIFI_OPERATION_WiFi_OFF:
			// Wifi state changing would announce to mARM.
			kcj_dev_info_update_wlan ( DEV_INFO_WLAN_OFF );
			break;

		case KCJ_WIFI_OPERATION_WiFi_STATION:
			// Wifi state changing would announce to mARM.
			kcj_dev_info_update_wlan ( DEV_INFO_WLAN_ON );
			break;

		case KCJ_WIFI_OPERATION_WiFi_HOTSPOT:
			// Wifi state changing would announce to mARM.
			kcj_dev_info_update_wlan ( DEV_INFO_WLAN_HOTSPOT_ON );
			break;

		default:
			printk(KERN_WARNING "%s:not supprted state[%d].\n",__func__,state);
			break;
	}
	
	// Current state setting is stored for debugging.
	wifi_current_operation_state = state;

	printk(KERN_WARNING "--- Wifi state change occured.[%d]\n",state);

}
EXPORT_SYMBOL(kcj_wifi_set_current_operation_state);

