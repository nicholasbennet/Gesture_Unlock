
/*!
 *  @file LSM6DSOX.cpp
 *  Adafruit LSM6DSOX 6-DoF Accelerometer and Gyroscope library
 *
 *  Bryan Siepert for Adafruit Industries
 * 	BSD (see license.txt)
 */

#include "Arduino.h"
#include <Wire.h>

#include "LSM6DSOX.h"

/*!
 *    @brief  Instantiates a new LSM6DSOX class
 */
LSM6DSOX::LSM6DSOX(void) {}

bool LSM6DSOX::_init(int32_t sensor_id) {
  Adafruit_BusIO_Register chip_id = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DS_WHOAMI);

  // make sure we're talking to the right chip
  if (chip_id.read() != LSM6DSOX_CHIP_ID) {
    return false;
  }
  _sensorid_accel = sensor_id;

  reset();

  Adafruit_BusIO_Register ctrl3 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSOX_CTRL3_C);
  Adafruit_BusIO_RegisterBits bdu = Adafruit_BusIO_RegisterBits(&ctrl3, 1, 6);
  bdu.write(true);

  // Disable I3C
  Adafruit_BusIO_Register ctrl_9 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSOX_CTRL9_XL);
  Adafruit_BusIO_RegisterBits i3c_disable_bit =
      Adafruit_BusIO_RegisterBits(&ctrl_9, 1, 1);

  i3c_disable_bit.write(true);

  // enable accelerometer by setting the data rate to non-zero
  setAccelDataRate(LSM6DS_RATE_104_HZ);

  delay(10);

  accel_sensor = new LSM6DS_Accelerometer(this);

  return true;
}

/**************************************************************************/
/*!
    @brief Disables and enables the SPI master bus pulllups.
    @param disable_pullups true to **disable** the I2C pullups, false to enable.
*/
void LSM6DSOX::disableSPIMasterPullups(bool disable_pullups) {

  Adafruit_BusIO_Register pin_config = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSOX_PIN_CTRL);

  Adafruit_BusIO_RegisterBits disable_ois_pu =
      Adafruit_BusIO_RegisterBits(&pin_config, 1, 7);

  disable_ois_pu.write(disable_pullups);
}

/**************************************************************************/
/*!
    @brief Enables and disables the I2C master bus pulllups.
    @param enable_pullups true to enable the I2C pullups, false to disable.
*/
void LSM6DSOX::enableI2CMasterPullups(bool enable_pullups) {

  Adafruit_BusIO_Register func_cfg_access = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSOX_FUNC_CFG_ACCESS);
  Adafruit_BusIO_RegisterBits master_cfg_enable_bit =
      Adafruit_BusIO_RegisterBits(&func_cfg_access, 1, 6);

  Adafruit_BusIO_Register master_config = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DSOX_MASTER_CONFIG);
  Adafruit_BusIO_RegisterBits i2c_master_pu_en =
      Adafruit_BusIO_RegisterBits(&master_config, 1, 3);

  master_cfg_enable_bit.write(true);
  i2c_master_pu_en.write(enable_pullups);
  master_cfg_enable_bit.write(false);
}
