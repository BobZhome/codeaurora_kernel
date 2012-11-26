/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation  
*/
/*=============================================================================
                        変更履歴

 番号                日付      コメント
------------------   --------  --------------------------------
G01-SWD0275-00006   11/11/24 WiFi 電源レギュレータ（PMIC）制御処理追加
G01-SWD0275-00007   11/11/28 MACアドレス書き込み処理追加
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

/* G01-SWD0275-00007 MACアドレス書き込み処理追加 */
#include <proc_comm.h>

#include <mach/kcj_wifi_bluetooth.h>

#define PM8058_GPIO_SLEEP_CLK 37 /* PMIC GPIO 38 */

/* Macros assume PMIC GPIOs start at 0 */
#define PM8058_GPIO_PM_TO_SYS(pm_gpio)     (pm_gpio + NR_GPIO_IRQS)
#define PM8058_GPIO_SYS_TO_PM(sys_gpio)    (sys_gpio - NR_GPIO_IRQS)

/* PMICのSleepClock設定関数 */
static int wifi_sleep_clock_initialize( void );

/* /proc/kyocera_wifi/power へのリードアクセスハンドラ */
static int kcj_wifi_power_initialize_read(char *page, char **start, off_t offset,
					int count, int *eof, void *data);
/* /proc/kyocera_wifi/power へのライトアクセスハンドラ */
static int kcj_wifi_power_initialize_write(struct file *file, const char *buffer,
					unsigned long count, void *data);
/* /proc/kyocera_wifi/debug へのリードアクセスハンドラ */
static int kcj_wifi_debug_read(char *page, char **start, off_t offset,
					int count, int *eof, void *data);
/* /proc/kyocera_wifi/debug へのリードアクセスハンドラ */
static int kcj_wifi_debug_write(struct file *file, const char *buffer,
					unsigned long count, void *data);

/* G01-SWD0275-00007 MACアドレス書き込み処理追加 ADD-S */
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
/* G01-SWD0275-00007 MACアドレス書き込み処理追加 ADD-E */

/* Virtual file system read handler(/proc/kyocera_wifi/wifi_state) be add. */
static int kcj_wifi_state_read(char *page, char **start, off_t offset,
					int count, int *eof, void *data);
/* Virtual file system write handler(/proc/kyocera_wifi/wifi_state) be add. */
static int kcj_wifi_state_write(struct file *file, const char *buffer,
					unsigned long count, void *data);

/* /proc/kyocera_wifiへのフォルダハンドル */
struct proc_dir_entry *kyocera_wifi_dir;

/* Wifi operation state ( WiFi_OFF , WiFi_Station_mode , WiFi_Hotspot) */
static int wifi_current_operation_state = 0;

/*
   DTV関連の不要処理に実装されているレギュレータ制御がないと、WiFiが起動できないため
   WiFiの電源制御が実装されるまでの間、以下のQualcomm不要処理を有効化する。
   関連する Redmineチケット番号は、#6975 
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
関数名   : wifi_sleep_clock_initialize
機能     : board-msm7x30.cで、PMICのSleepClk制御をしていた処理を移植
           WiFION前に呼び出される必要がある
引数     : なし
戻り値   : 0 なら成功 0 以外なら失敗
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
関数名   : kyocera_wifi_init
機能     : 仮想ファイルシステム(/proc)に、WiFi用のエントリを作成する
引数     : なし
戻り値   : 0 なら成功  -ENOMEMなら失敗
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

/* G01-SWD0275-00007 MACアドレス書き込み処理追加 ADD-S */
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
/* G01-SWD0275-00007 MACアドレス書き込み処理追加 ADD-E */

	remove_proc_entry("debug", kyocera_wifi_dir );
	remove_proc_entry("power", kyocera_wifi_dir );
	return retval;
}

/*@**************************************************************************
関数名   : kcj_wifi_power_initialize_read
機能     : SleepClockSetting処理が実装されていることを標準出力に出力する
引数     : (O)   char*  page   0:OFF   1:ON 未使用
           (I)   char** start  未使用
           (I)   off_t  ifset  未使用
           (I)   int    count  未使用
           (I)   int*   eof    1を設定
           (I)   void*  data   未使用
戻り値   : pageに設定するサイズ
           0 :失敗
****************************************************************************/
static int kcj_wifi_power_initialize_read(char *page, char **start, off_t offset,
					int count, int *eof, void *data)
{
	*eof = 1;
	return sprintf(page, "kcj_wifi_power_initialize_read\n");
}
/*@**************************************************************************
関数名   : kcj_wifi_power_initialize_write
機能     : *bufferの先頭が1であれば、WiFi用のSleepClockを設定する
引数     : struct file *file 未使用
           const char *buffer 書き込み要求されたデータ
           unsigned long count データサイズ
           void *data 未使用

戻り値   : pageに設定するサイズ
           0 :失敗
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
関数名   : kcj_wifi_state_read
機能     : Wifi status would be read.
引数     : (O)   char*  page   0:OFF   1:ON
           (I)   char** start  未使用
           (I)   off_t  ifset  未使用
           (I)   int    count  未使用
           (I)   int*   eof    1を設定
           (I)   void*  data   未使用
戻り値   : pageに設定するサイズ
           0 :失敗
****************************************************************************/
static int kcj_wifi_state_read(char *page, char **start, off_t offset,
					int count, int *eof, void *data)
{
	/* Current WiFi status parameter would return. */
    return sprintf(page, "%d", wifi_current_operation_state );
}

/*@**************************************************************************
関数名   : kcj_wifi_state_write
機能     : Wifi status would be set.
引数     : (I) struct file *file 未使用
           (I) const char *buffer 書き込み要求されたデータ
           (I) unsigned long count データサイズ
           (I) void *data 未使用

戻り値   : pageに設定するサイズ
           0 :失敗
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
関数名   : kcj_wifi_debug_read
機能     : WiFiデバッグ用のエントリ。 メッセージを出力する
引数     : (O)   char*  page   0:OFF   1:ON
           (I)   char** start  未使用
           (I)   off_t  ifset  未使用
           (I)   int    count  未使用
           (I)   int*   eof    1を設定
           (I)   void*  data   未使用
戻り値   : pageに設定するサイズ
           0 :失敗
****************************************************************************/
static int kcj_wifi_debug_read(char *page, char **start, off_t offset,
					int count, int *eof, void *data)
{
	*eof = 1;
	return sprintf(page, "kcj_wifi_debug_read\n");
}

/*@**************************************************************************
関数名   : kcj_wifi_debug_write
機能     : WiFiデバッグ用のエントリ。 1が書き込まれると、標準出力にメッセージを出力
引数     : (I) struct file *file 未使用
           (I) const char *buffer 書き込み要求されたデータ
           (I) unsigned long count データサイズ
           (I) void *data 未使用

戻り値   : pageに設定するサイズ
           0 :失敗
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

/* G01-SWD0275-00007 MACアドレス書き込み処理追加 ADD-S */
/* F17-SWD0254-00007 ADD-S */
/****************************************************************************
*  関数名   : bluepower_read_proc_bt_addr                                  
*  機能     : BluetoothアドレスをNVより読み出す                            
*  引数     : (O)   char*  page   Bluetoothアドレスを0x付きで格納           
            : (I)   char** start  未使用                                   
            : (I)   off_t  ifset  未使用                                   
            : (I)   int    count  未使用                                   
            : (I)   int*   eof    1を設定                                  
            : (I)   void*  data   未使用                                   
*  戻り値   : pageに設定するサイズ                                         
*           : 0 :失敗                                                      
****************************************************************************/
static int bluepower_read_proc_bt_addr(char *page, char **start, off_t offset,
					int count, int *eof, void *data)
{
	unsigned int read_bd_addr[2];		  /* BDアドレス格納(読み込み)用 */
	unsigned char write_bd_addr[6];		 /* BDアドレス格納(書き込み)用 */
	int ret;								/* 戻り値格納用               */
	unsigned int cmd;

	/* 初期化 */
	memset(read_bd_addr ,0 ,sizeof(read_bd_addr ));
	memset(write_bd_addr,0 ,sizeof(write_bd_addr));
	ret = -1;

	*eof = 1;

    cmd = 0;
	/* BD_ADDR読み出し */
	ret = msm_proc_comm(PCOM_CUSTOMER_CMD3  , &cmd
											,  &read_bd_addr[0]);

    if( ret != 0)	 /* NVから読み出し失敗 */
	{
		/* 念のため初期化 NVより取得失敗時は明示的に0を格納する */
		memset(read_bd_addr,0,sizeof(read_bd_addr));
	}
    cmd = 1;
	ret = msm_proc_comm(PCOM_CUSTOMER_CMD3  , &cmd
											, &read_bd_addr[1]);

	if( ret != 0)	 /* NVから読み出し失敗 */
	{
		/* 念のため初期化 NVより取得失敗時は明示的に0を格納する */
		memset(read_bd_addr,0,sizeof(read_bd_addr));
	}

	/* 取得BTアドレスを出力形式でCHARの配列に格納 */
	write_bd_addr[0] =  read_bd_addr[0] & 0x000000FF ;
	write_bd_addr[1] = (read_bd_addr[0] & 0x0000FF00 ) >> 8;
	write_bd_addr[2] = (read_bd_addr[0] & 0x00FF0000 ) >> 16;
	write_bd_addr[3] = (read_bd_addr[0] & 0xFF000000 ) >> 24;
	write_bd_addr[4] =  read_bd_addr[1] & 0x000000FF ;
	write_bd_addr[5] = (read_bd_addr[1] & 0x0000FF00 ) >> 8;

	/* :区切りにしてBDアドレスを設定 */
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
*  関数名   : bluepowerr_read_proc_wlan_mac_addr                           
*  機能     : WLAN MACアドレスをNVより読み出す                             
*  引数     : (O)   char*  page   MACアドレスを0x付きで格納                
            : (I)   char** start  未使用                                   
            : (I)   off_t  ifset  未使用                                   
            : (I)   int    count  未使用                                   
            : (I)   int*   eof    1を設定                                  
            : (I)   void*  data   未使用                                   
*  戻り値   : pageに設定するサイズ                                         
*           : 0 :失敗                                                      
****************************************************************************/
static int bluepower_read_proc_wlan_mac_addr_common(char *page, char **start, off_t offset,
					int count, int *eof, void *data , int format_mode)
{
	unsigned int read_wlan_mac_addr[2];			/* MACアドレス格納(読み込み)用 */
	unsigned char write_wlan_mac_addr[6];		/* MACアドレス格納(書き込み)用 */
	int ret;									/* 戻り値格納用               */
	unsigned int cmd;

	/* 初期化 */
	memset(read_wlan_mac_addr ,0 ,sizeof(read_wlan_mac_addr ));
	memset(write_wlan_mac_addr,0 ,sizeof(write_wlan_mac_addr));
	ret = -1;
	*eof = 1;
    cmd = 2;
	/* MAC_ADDR読み出し */
	ret = msm_proc_comm(PCOM_CUSTOMER_CMD3, &cmd, 
											&read_wlan_mac_addr[0]
			);
	if( ret != 0)	 /* NVから読み出し失敗 */
	{

		/* 念のため初期化 NVより取得失敗時は明示的に0を格納する */
		memset(read_wlan_mac_addr,0,sizeof(read_wlan_mac_addr));
	}
    cmd = 3;
	/* MAC_ADDR読み出し */
	ret = msm_proc_comm(PCOM_CUSTOMER_CMD3, &cmd, 
											&read_wlan_mac_addr[1]
															);
/* F17-SWD0254-00009 ADD_S */
    if(( ret != 0) || 	 /* NVから読み出し失敗 */
       ((0x00000000 == (read_wlan_mac_addr[0] & 0xFFFFFFFF)) &&  /* MACアドレスが0 */
        (0x00000000 == (read_wlan_mac_addr[1] & 0x0000FFFF))
      ))
/* F17-SWD0254-00009 ADD_E */
	{

		/* 念のため初期化 NVより取得失敗時は明示的に0を格納する */
		memset(read_wlan_mac_addr,0,sizeof(read_wlan_mac_addr));
/* F17-SWD0254-00009 ADD_S */
		/* 工程中フラグチェック */
		if(check_product_line() != 1)
		{
			/* 工程中フラグがON以外でNVが空の場合はZZZZZZを格納 */
/* G01-SWD0275-00007 MACアドレス書き込み処理追加 MOD */
			return sprintf(page, "ZZZZZZZZ"); 
		}

	    /* 工程中フラグがONかつ取得したNVアドレスがALL0である場合、先頭バイトを1とする */
		/* 工程中フラグがOFFの場合はドライバのMACアドレス設定処理内でエラー対応        */
		else{
/* G01-SWD0275-00007 MACアドレス書き込み処理追加 MOD */
			return sprintf(page, "100000000000"); 
		}
/* F17-SWD0254-00009 ADD_E */
	}

	/* 取得BTアドレスを出力形式でCHARの配列に格納 */
	write_wlan_mac_addr[0] =  read_wlan_mac_addr[0] & 0x000000FF ;
	write_wlan_mac_addr[1] = (read_wlan_mac_addr[0] & 0x0000FF00 ) >> 8;
	write_wlan_mac_addr[2] = (read_wlan_mac_addr[0] & 0x00FF0000 ) >> 16;
	write_wlan_mac_addr[3] = (read_wlan_mac_addr[0] & 0xFF000000 ) >> 24;
	write_wlan_mac_addr[4] =  read_wlan_mac_addr[1] & 0x000000FF ;
	write_wlan_mac_addr[5] = (read_wlan_mac_addr[1] & 0x0000FF00 ) >> 8;

	/* 区切りなしでMACアドレスを設定 */
/* G01-SWD0275-00007 MACアドレス書き込み処理追加 MOD */
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
*  関数名   : read_proc_product_line
*  機能     : 工程中フラグON/OFFの状態をファイルシステムに書き出す                            
*  引数     : (O)   char*  page   0:OFF   1:ON 
            : (I)   char** start  未使用                                   
            : (I)   off_t  ifset  未使用                                   
            : (I)   int    count  未使用                                   
            : (I)   int*   eof    1を設定                                  
            : (I)   void*  data   未使用                                   
*  戻り値   : pageに設定するサイズ                                         
*           : 0 :失敗                                                      
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
*  関数名   : check_product_line                                           
*  機能     : 工程中フラグがONかどうか取得する                             
*  引数     : none                                                         
*  戻り値   : 1 :工程中フラグON                                            
*           : 0 :工程中フラグOFF                                           
****************************************************************************/
static int check_product_line(void)
{
    unsigned int read_product_line[2];		/* 工程中フラグ(読み込み)用 */
	unsigned char check_product[8];			/* 工程中フラグ(チェック)用 */
	int ret;								/* 戻り値格納用             */
	unsigned int cmd;                       /* NV取得用コマンド         */

	/* 初期化 */
	memset(read_product_line ,0 ,sizeof(read_product_line ));
	memset(check_product,0 ,sizeof(check_product));
	ret = 0;

    /* 工程中フラグチェック */
    cmd = 4;
	/* 工程中フラグ読み出し */
	ret = msm_proc_comm(PCOM_CUSTOMER_CMD3, &cmd, 
											&read_product_line[0]
			);
	if( ret != 0)	 /* NVから読み出し失敗 */
	{

		/* 念のため初期化 NVより取得失敗時は明示的に0を格納する */
		memset(read_product_line,0,sizeof(read_product_line));
	}
    cmd = 5;
	/* 工程中フラグ読み出し */
	ret = msm_proc_comm(PCOM_CUSTOMER_CMD3, &cmd, 
											&read_product_line[1]
			);

	if( ret != 0)	 /* NVから読み出し失敗 */
	{

		/* 念のため初期化 NVより取得失敗時は明示的に0を格納する */
		memset(read_product_line,0,sizeof(read_product_line));
	}

	/* 取得工程中フラグを出力形式でCHARの配列に格納 */
	check_product[0] =  read_product_line[0] & 0x000000FF ;
	check_product[1] = (read_product_line[0] & 0x0000FF00 ) >> 8;
	check_product[2] = (read_product_line[0] & 0x00FF0000 ) >> 16;
	check_product[3] = (read_product_line[0] & 0xFF000000 ) >> 24;
	check_product[4] =  read_product_line[1] & 0x000000FF ;
	check_product[5] = (read_product_line[1] & 0x0000FF00 ) >> 8;
	check_product[6] = (read_product_line[1] & 0x00FF0000 ) >> 16;
	check_product[7] = (read_product_line[1] & 0xFF000000 ) >> 24;

	/* 工程中フラグがONかのチェックを行う                         */
	/* 工程中フラグがONは下記の状態                               */
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
/* G01-SWD0275-00007 MACアドレス書き込み処理追加 ADD-E */

/****************************************************************************
*  関数名   : kcj_wifi_set_current_operation_state                         
*  機能     : If changed wifi current state then inform to 'vbatt software driver'.
*  引数     : 0 - Wifi OFF 
*             1 - Wifi Station mode
*             2 - Wifi Hotspot mode                                        
*  戻り値   : none                                                         
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

