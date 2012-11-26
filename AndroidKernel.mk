#Android makefile to build kernel as a part of Android Build

ifeq ($(TARGET_PREBUILT_KERNEL),)

KERNEL_OUT := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ
KERNEL_CONFIG := $(KERNEL_OUT)/.config
TARGET_PREBUILT_INT_KERNEL := $(KERNEL_OUT)/arch/arm/boot/zImage
KERNEL_HEADERS_INSTALL := $(KERNEL_OUT)/usr
KERNEL_MODULES_INSTALL := system
KERNEL_MODULES_OUT := $(TARGET_OUT)/lib/modules

ifeq ($(TARGET_USES_UNCOMPRESSED_KERNEL),true)
$(info Using uncompressed kernel)
TARGET_PREBUILT_KERNEL := $(KERNEL_OUT)/piggy
else
TARGET_PREBUILT_KERNEL := $(TARGET_PREBUILT_INT_KERNEL)
endif

define mv-modules
mdpath=`find $(KERNEL_MODULES_OUT) -type f -name modules.dep`;\
if [ "$$mdpath" != "" ];then\
mpath=`dirname $$mdpath`;\
ko=`find $$mpath/kernel -type f -name *.ko`;\
for i in $$ko; do mv $$i $(KERNEL_MODULES_OUT)/; done;\
fi
endef

define clean-module-folder
mdpath=`find $(KERNEL_MODULES_OUT) -type f -name modules.dep`;\
if [ "$$mdpath" != "" ];then\
mpath=`dirname $$mdpath`; rm -rf $$mpath;\
fi
endef

$(KERNEL_OUT):
	mkdir -p $(KERNEL_OUT)

$(KERNEL_CONFIG): $(KERNEL_OUT)
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi- $(KERNEL_DEFCONFIG)

$(KERNEL_OUT)/piggy : $(TARGET_PREBUILT_INT_KERNEL)
	$(hide) gunzip -c $(KERNEL_OUT)/arch/arm/boot/compressed/piggy.gzip > $(KERNEL_OUT)/piggy

$(TARGET_PREBUILT_INT_KERNEL): CONFIG_TARGET_PROTOTYPE $(KERNEL_OUT) $(KERNEL_CONFIG) $(KERNEL_HEADERS_INSTALL)
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi-
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi- modules
	$(MAKE) -C kernel O=../$(KERNEL_OUT) INSTALL_MOD_PATH=../../$(KERNEL_MODULES_INSTALL) ARCH=arm CROSS_COMPILE=arm-eabi- modules_install
	$(mv-modules)
	$(clean-module-folder)

$(KERNEL_HEADERS_INSTALL): CONFIG_TARGET_PROTOTYPE $(KERNEL_OUT) $(KERNEL_CONFIG)
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi- headers_install

kerneltags: $(KERNEL_OUT) $(KERNEL_CONFIG)
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi- tags

kernelconfig: CONFIG_TARGET_PROTOTYPE $(KERNEL_OUT) $(KERNEL_CONFIG)
	env KCONFIG_NOTIMESTAMP=true \
	     $(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi- menuconfig
	env KCONFIG_NOTIMESTAMP=true \
	     $(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi- savedefconfig
	cp $(KERNEL_OUT)/defconfig kernel/arch/arm/configs/$(KERNEL_DEFCONFIG)

ifneq (, $(filter $(FILTER_PRODUCT_DEVICE), $(TARGET_PRODUCT)))
CONFIG_TARGET_PROTOTYPE: $(KERNEL_CONFIG)
	sed -i 's/CONFIG_TARGET_PROTOTYPE_WS0=y/# CONFIG_TARGET_PROTOTYPE_WS0 is not set/g' $(KERNEL_CONFIG)
	sed -i 's/CONFIG_TARGET_PROTOTYPE_WS1=y/# CONFIG_TARGET_PROTOTYPE_WS1 is not set/g' $(KERNEL_CONFIG)
	sed -i 's/CONFIG_TARGET_PROTOTYPE_WS2=y/# CONFIG_TARGET_PROTOTYPE_WS2 is not set/g' $(KERNEL_CONFIG)
	sed -i 's/CONFIG_TARGET_PROTOTYPE_C5155=y/# CONFIG_TARGET_PROTOTYPE_C5155 is not set/g' $(KERNEL_CONFIG)
	sed -i 's/CONFIG_TARGET_PROTOTYPE_C5170=y/# CONFIG_TARGET_PROTOTYPE_C5170 is not set/g' $(KERNEL_CONFIG)
	sed -i 's/# CONFIG_TARGET_PROTOTYPE_$(TARGET_PROTOTYPE) is not set/CONFIG_TARGET_PROTOTYPE_$(TARGET_PROTOTYPE)=y/g' $(KERNEL_CONFIG)
	sed -i 's/# CONFIG_TARGET_PRODUCT_$(TARGET_PRODUCT) is not set/CONFIG_TARGET_PRODUCT_$(TARGET_PRODUCT)=y/g' $(KERNEL_CONFIG)
else
CONFIG_TARGET_PROTOTYPE:
endif

endif
