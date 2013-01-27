GCC_VERSION      := $(shell $(CONFIG_SHELL) $(PWD)/scripts/gcc-version.sh $(CROSS_COMPILE)gcc)
EXTRA_CFLAGS += -Idrivers/media/video/msm
EXTRA_CFLAGS += -Idrivers/media/video/msm/io
obj-$(CONFIG_MSM_ACTUATOR) += msm_actuator.o
#obj-$(CONFIG_IMX111_ACT) += imx111_act.o
obj-$(CONFIG_MT9E013_ACT) += mt9e013_act.o
#LGE_CHANGE_S L1_SERIES (MS870, VS870) jongkwon.chae@lge.com
#obj-$(CONFIG_S5K4E1_ACT) += s5k4e1_act.o
#LGE_CHANGE_E L1_SERIES (MS870, VS870) jongkwon.chae@lge.com
