# bcmdhd
SRCROOT = drivers/net/wireless/bcmdhd/src
DHDCFLAGS = -Wall -Wstrict-prototypes -Dlinux -DBCMDRIVER                     \
        -DBCMDONGLEHOST -DUNRELEASEDCHIP -DBCMDMA32 -DWLBTAMP -DBCMFILEIMAGE  \
        -DDHDTHREAD -DDHD_GPL -DDHD_SCHED -DDHD_DEBUG -DBDC -DTOE    \
        -DDHD_BCMEVENTS -DSHOW_EVENTS -DDONGLEOVERLAYS -DBCMDBG               \
        -DMMC_SDIO_ABORT -DBCMSDIO -DBCMLXSDMMC -DBCMPLATFORM_BUS -DWLP2P     \
        -DNEW_COMPAT_WIRELESS -DWIFI_ACT_FRAME -DARP_OFFLOAD_SUPPORT          \
        -DKEEP_ALIVE -DCSCAN -DPKT_FILTER_SUPPORT                             \
        -DEMBEDDED_PLATFORM -DPNO_SUPPORT

DHDCFLAGS += -DENABLE_INSMOD_NO_FW_LOAD -DGET_CUSTOM_MAC_ENABLE

# Common Patch - Should be defined!!!!
DHDCFLAGS += -DCONFIG_COMMON_PATCH

# For LGE OTA
DHDCFLAGS += -DCONFIG_CONTROL_PM

ifeq ($(CONFIG_WIFI_CONTROL_FUNC),y)
DHDCFLAGS += -DCONFIG_WIFI_CONTROL_FUNC -DCUSTOMER_HW2
else
DHDCFLAGS += -DCUSTOM_OOB_GPIO_NUM=$(CONFIG_BCMDHD_GPIO_WL_HOSTWAKEUP)
endif

ifeq ($(CONFIG_BCMDHD_HW_OOB),y)
DHDCFLAGS += -DOOB_INTR_ONLY -DHW_OOB
else
DHDCFLAGS += -DSDIO_ISR_THREAD# -DIOCTL_RESP_TIMEOUT=2000
endif

#DHDCFLAGS += -DBCM4334_CHIP
#DHDCFLAGS += -DPROP_TXSTATUS
#DHDCFLAGS += -DVSDB -DHT40_GO
#DHDCFLAGS += -DWL_CFG80211_VSDB_PRIORITIZE_SCAN_REQUEST
#DHDCFLAGS += -DDHD_USE_IDLECOUNT
#DHDCFLAGS += -DSUPPORT_AMPDU_MPDU_CMD
#DHDCFLAGS += -DVSDB_DYNAMIC_F2_BLKSIZE -DSDIO_F2_BLKSIZE=512 -DVSDB_F2_BLKSIZE=64

DHDCFLAGS += -DBCM4334_CHIP
DHDCFLAGS += -DHT40_GO
DHDCFLAGS += -DSDIO_F2_BLKSIZE=512 
DHDCFLAGS += -DDHD_USE_IDLECOUNT
DHDCFLAGS += -DSUPPORT_AMPDU_MPDU_CMD


# For p2p connection issue
DHDCFLAGS += -DWL_CFG80211_GON_COLLISION
DHDCFLAGS += -DWL_CFG80211_SYNC_GON_TIME
#end p2p connection

# For Passing all multicast packets to host when not in suspend mode.
DHDCFLAGS += -DPASS_ALL_MCAST_PKTS

# For Scan result patch
DHDCFLAGS += -DESCAN_RESULT_PATCH

# For CCX
ifeq ($(CONFIG_BRCM_CCX),y)
DHDCFLAGS += -DBCMCCX
endif

ifeq ($(CONFIG_TARGET_LOCALE_CHN),y)
DHDCFLAGS += -DBCMWAPI_WPI -DBCMWAPI_WAI
endif

# For Debug
EXTRA_CFLAGS += $(DHDCFLAGS) -DWL_CFG80211 -DRSSI_OFFSET=0
EXTRA_CFLAGS += -DDHD_DEBUG -DSRCBASE=\"$(src)/src\"

EXTRA_CFLAGS += -I$(src)/src/include/
EXTRA_CFLAGS += -I$(src)/src/dhd/sys/
EXTRA_CFLAGS += -I$(src)/src/dongle/
EXTRA_CFLAGS += -I$(src)/src/bcmsdio/sys/
EXTRA_CFLAGS += -I$(src)/src/wl/sys/
EXTRA_CFLAGS += -I$(src)/src/shared/
EXTRA_CFLAGS += -I$(src)/src/wl/bcmwifi/src/
EXTRA_CFLAGS += -I$(src)/src/wl/bcmwifi/include/

EXTRA_LDFLAGS += --strip-debug
KBUILD_CFLAGS += -I$(LINUXDIR)/include -I$(shell pwd)

obj-$(CONFIG_BCMDHD)   += bcmdhd.o

bcmdhd-objs := src/bcmsdio/sys/bcmsdh.o	src/bcmsdio/sys/bcmsdh_linux.o \
	 src/bcmsdio/sys/bcmsdh_sdmmc.o	src/bcmsdio/sys/bcmsdh_sdmmc_linux.o \
	 src/dhd/sys/dhd_bta.o		src/dhd/sys/dhd_cdc.o \
	 src/dhd/sys/dhd_common.o	src/dhd/sys/dhd_custom_gpio.o \
	 src/dhd/sys/dhd_linux.o	src/dhd/sys/dhd_linux_sched.o \
	 src/dhd/sys/dhd_cfg80211.o	src/dhd/sys/dhd_sdio.o \
	 src/shared/aiutils.o		src/shared/bcmevent.o \
	 src/shared/bcmutils.o		src/wl/bcmwifi/src/bcmwifi_channels.o \
	 src/shared/hndpmu.o		src/shared/linux_osl.o \
	 src/shared/sbutils.o		src/shared/siutils.o \
	 src/wl/sys/wl_android.o	src/wl/sys/wl_cfg80211.o \
	 src/wl/sys/wl_cfgp2p.o		src/wl/sys/wldev_common.o \
	 src/wl/sys/wl_linux_mon.o	src/wl/sys/wl_roam.o

all:
	@echo "$(MAKE) --no-print-directory -C $(KDIR) SUBDIRS=$(CURDIR) modules"
	@$(MAKE) --no-print-directory -C $(KDIR) SUBDIRS=$(CURDIR) modules

clean:
	rm -rf *.o *.ko *.mod.c *~ .*.cmd \
	Module.symvers modules.order .tmp_versions modules.builtin \
	src/bcmsdio/sys/*.o \
	src/bcmsdio/sys/*.o.cmd \
	src/bcmsdio/sys/.*.o.cmd \
	src/dhd/sys/*.o \
	src/dhd/sys/*.o.cmd \
	src/dhd/sys/.*.o.cmd \
	src/shared/*.o \
	src/shared/*.o.cmd \
	src/shared/.*.o.cmd \
	src/wl/sys/*.o \
	src/wl/sys/*.o.cmd \
	src/wl/sys/.*.o.cmd \
	src/wl/bcmwifi/src/bcmwifi_channels.o \
	src/wl/bcmwifi/src/.*.o.cmd

install:
	@$(MAKE) --no-print-directory -C $(KDIR) \
		SUBDIRS=$(CURDIR) modules_install
