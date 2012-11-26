/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
*/
















#ifndef _APDS9900_H_
#define _APDS9900_H_
/*****************************************************************************/
/* defines                                                                   */
/*****************************************************************************/

#define CMD_REG_REPEATED_BYTE     0x80
#define CMD_REG_AUTO_INCREMENT    0xA0
#define CMD_REG_PROX_CLEAR        0xE5


#define REG_ADDRESS_ENABLE     0x00
#define REG_ADDRESS_ATIME      0x01
#define REG_ADDRESS_PTIME      0x02
#define REG_ADDRESS_WTIME      0x03
#define REG_ADDRESS_AILTL      0x04
#define REG_ADDRESS_AILTH      0x05
#define REG_ADDRESS_AIHTL      0x06
#define REG_ADDRESS_AIHTH      0x07
#define REG_ADDRESS_PILTL      0x08
#define REG_ADDRESS_PILTH      0x09
#define REG_ADDRESS_PIHTL      0x0A
#define REG_ADDRESS_PIHTH      0x0B
#define REG_ADDRESS_PERS       0x0C
#define REG_ADDRESS_CONFIG     0x0D
#define REG_ADDRESS_PPCOUNT    0x0E
#define REG_ADDRESS_CONTROL    0x0F
#define REG_ADDRESS_REV        0x11
#define REG_ADDRESS_ID         0x12
#define REG_ADDRESS_STATUS     0x13
#define REG_ADDRESS_CDATAL     0x14
#define REG_ADDRESS_CDATAH     0x15
#define REG_ADDRESS_IRDATAL    0x16
#define REG_ADDRESS_IRDATAH    0x17
#define REG_ADDRESS_PDATAL     0x18
#define REG_ADDRESS_PDATAH     0x19


#define REG_ATIME_INIT_VALUE      0xDB
#define REG_PTIME_INIT_VALUE      0xFF
#define REG_WTIME_INIT_VALUE      0xFF
#define REG_CONFIG_INIT_VALUE     0x00


#define DELAY_CAL_BASE_TIME      (272)
#define ALS_DELAY                (278)
#define DELAY_CAL_MARGIN         (120)

#define CH0DATA_FULL_SCALE_COUNT   (65535)
#define CH0DATA_BASE_COUNT         (1023)


#define REG_ENABLE_PON_CHANGE  0x01



#ifdef CONFIG_TARGET_PROTOTYPE_WS2
  #ifdef CONFIG_TARGET_PRODUCT_C5155
#define LUX_CAL_GA             (5406000)
  #else
#define LUX_CAL_GA             (4664000)
  #endif

#else
#define LUX_CAL_GA             (4389000)
#endif

#define LUX_CAL_DF             (52)
#define LUX_CAL_BASE_TIME      (272)

#define LUX_CAL_COEFFICIENT_B    (2140)
#define LUX_CAL_COEFFICIENT_C    (344)
#define LUX_CAL_COEFFICIENT_D    (688)


#define PROXIMITY_ON           (0)
#define PROXIMITY_OFF          (1 * 1000)

#define LIGHT_SENSOR_LUX_VALUE_MIN        (0)
#define LIGHT_SENSOR_LUX_VALUE_MAX        (900)
#define LIGHT_SENSOR_LUX_VALUE_ERR        (0)


#define LIGHT_SENSOR_CHECK_WAITTIME       (10000)


#define LIGHT_SENSOR_SEND_OK       (1)
#define LIGHT_SENSOR_SEND_WAIT     (0)


#endif /* _APDS9900_H_ */

