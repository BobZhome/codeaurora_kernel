
#ifndef _F200_HALL_IC_H
#define _F200_HALL_IC_H

#define GPIO_CRADLE_DETECT_N	40
#define GPIO_CRADLE_DETECT_S	39

struct f200_hall_ic_pdata_type{
	unsigned int s_gpio;
	unsigned int n_gpio;
	int s_state;
	int n_state;
	struct work_struct hall_ic_work;
	struct delayed_work hall_ic_delayed_work;
	struct input_dev *input_dev_hall_ic;
	int hall_ic_s_irq;
	int hall_ic_n_irq;
	unsigned long irqf_trigger_mode;
};

#define HALL_IC_DETECTION_TIME	150





#endif /* _F200_HALL_IC_H */

