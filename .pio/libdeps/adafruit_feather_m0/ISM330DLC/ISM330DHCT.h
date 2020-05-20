/*!
 *  @file ISM330DHCT.h
 *
 * 	I2C Driver for the Adafruit ISM330DHCT 6-DoF Accelerometer and Gyroscope
 *library
 *
 * 	This is a library for the Adafruit ISM330DHCT breakout:
 * 	https://www.adafruit.com/products/4480
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *
 *	BSD license (see license.txt)
 */

#ifndef _ISM330DHCT_H
#define _ISM330DHCT_H

#include "LSM6DSOX.h"

#define ISM330DHCT_CHIP_ID 0x6B ///< ISM330DHCT default device id from WHOAMI

/*!
 *    @brief  Class that stores state and functions for interacting with
 *            the ISM330DHCT I2C Digital Potentiometer
 */
class ISM330DHCT : public LSM6DSOX {
public:
  ISM330DHCT();
  ~ISM330DHCT(){};

private:
  bool _init(int32_t sensor_id);
};

#endif
