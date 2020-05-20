
/*!
 *  @file ISM330DHCT.cpp Adafruit ISM330DHCT 6-DoF Accelerometer
 *  and Gyroscope library
 *
 *  Bryan Siepert for Adafruit Industries
 * 	BSD (see license.txt)
 */

#include "Arduino.h"
#include <Wire.h>

#include "ISM330DHCT.h"

/*!
 *    @brief  Instantiates a new ISM330DHCT class
 */
ISM330DHCT::ISM330DHCT(void) {}

bool ISM330DHCT::_init(int32_t sensor_id) {
  // make sure we're talking to the right chip
  if (chipID() != ISM330DHCT_CHIP_ID) {
    return false;
  }
  _sensorid_accel = sensor_id;

  reset();

  // enable accelerometer by setting the data rate to non-zero
  setAccelDataRate(LSM6DS_RATE_104_HZ);

  delay(10);

  accel_sensor = new LSM6DS_Accelerometer(this);

  return true;
}
