
/*!
 *  @file LSM6DS.cpp Adafruit LSM6DS 6-DoF Accelerometer
 *  and Gyroscope library
 *
 *  @section intro_sec Introduction
 *
 * 	I2C Driver base for Adafruit LSM6DS 6-DoF Accelerometer
 *      and Gyroscope libraries
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *  @section dependencies Dependencies
 *  This library depends on the Adafruit BusIO library
 *
 *  This library depends on the Adafruit Unified Sensor library

 *  @section author Author
 *
 *  Bryan Siepert for Adafruit Industries
 *
 * 	@section license License
 *
 * 	BSD (see license.txt)
 *
 * 	@section  HISTORY
 *
 *     v1.0 - First release
 */

#include "Arduino.h"
#include <Wire.h>

#include "LSM6DS.h"

/*!
 *    @brief  Instantiates a new LSM6DS class
 */
LSM6DS::LSM6DS(void) {}

/*!
 *    @brief  Cleans up the LSM6DS
 */
LSM6DS::~LSM6DS(void) {
  if (accel_sensor)
    delete accel_sensor;
}

/*!  @brief  Unique subclass initializer post i2c/spi init
 *   @param sensor_id Optional unique ID for the sensor set
 *   @returns True if chip identified and initialized
 */
bool LSM6DS::_init(int32_t sensor_id) { return false; };

/*!
 *    @brief  Read chip identification register
 *    @returns 8 Bit value from WHOAMI register
 */
uint8_t LSM6DS::chipID(void) {
  Adafruit_BusIO_Register chip_id = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DS_WHOAMI);

  return chip_id.read();
}

/*!
 *    @brief  Sets up the hardware and initializes I2C
 *    @param  i2c_address
 *            The I2C address to be used.
 *    @param  wire
 *            The Wire object to be used for I2C connections.
 *    @param  sensor_id
 *            The user-defined ID to differentiate different sensors
 *    @return True if initialization was successful, otherwise false.
 */
boolean LSM6DS::begin_I2C(uint8_t i2c_address, TwoWire *wire,
                                   int32_t sensor_id) {
  if (i2c_dev) {
    delete i2c_dev; // remove old interface
  }

  i2c_dev = new Adafruit_I2CDevice(i2c_address, wire);

  if (!i2c_dev->begin()) {
    return false;
  }

  return _init(sensor_id);
}

/**************************************************************************/
/*!
    @brief Resets the sensor to its power-on state, clearing all registers and
   memory
*/
void LSM6DS::reset(void) {

  Adafruit_BusIO_Register ctrl3 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DS_CTRL3_C);

  Adafruit_BusIO_RegisterBits sw_reset =
      Adafruit_BusIO_RegisterBits(&ctrl3, 1, 0);

  Adafruit_BusIO_RegisterBits boot = Adafruit_BusIO_RegisterBits(&ctrl3, 1, 7);

  sw_reset.write(true);

  while (sw_reset.read()) {
    delay(1);
  }

  boot.write(true);
  while (boot.read()) {
    delay(1);
  }

  
}

/*!
    @brief  Gets an Adafruit Unified Sensor object for the accelerometer
    sensor component
    @return Adafruit_Sensor pointer to accelerometer sensor
 */
Adafruit_Sensor *LSM6DS::getAccelerometerSensor(void) {
  return accel_sensor;
}

bool LSM6DS::get_accel_Event(sensors_event_t *accel) {
  uint32_t t = millis();
  _read();

  // use helpers to fill in the events
  fillAccelEvent(accel, t);
  return true;
}

void LSM6DS::fillAccelEvent(sensors_event_t *accel,
                                     uint32_t timestamp) {
  memset(accel, 0, sizeof(sensors_event_t));
  accel->version = 1;
  accel->sensor_id = _sensorid_accel;
  accel->type = SENSOR_TYPE_ACCELEROMETER;
  accel->timestamp = timestamp;
  accel->acceleration.x = accX;
  accel->acceleration.y = accY;
  accel->acceleration.z = accZ;
}

/**************************************************************************/
/*!
    @brief Sets the accelerometer data rate.
    @param  data_rate
            The the accelerometer data rate. Must be a `lsm6ds_data_rate_t`.
*/
void LSM6DS::setAccelDataRate(lsm6ds_data_rate_t data_rate) {

  Adafruit_BusIO_Register ctrl1 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DS_CTRL1_XL);

  Adafruit_BusIO_RegisterBits accel_data_rate =
      Adafruit_BusIO_RegisterBits(&ctrl1, 4, 4);

  accel_data_rate.write(data_rate);
}

/**************************************************************************/
/*!
    @brief Gets the accelerometer measurement range.
    @returns The the accelerometer measurement range.
*/
lsm6ds_accel_range_t LSM6DS::getAccelRange(void) {

  Adafruit_BusIO_Register ctrl1 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DS_CTRL1_XL);

  Adafruit_BusIO_RegisterBits accel_range =
      Adafruit_BusIO_RegisterBits(&ctrl1, 2, 2);

  return (lsm6ds_accel_range_t)accel_range.read();
}
/**************************************************************************/
/*!
    @brief Sets the accelerometer measurement range.
    @param new_range The `lsm6ds_accel_range_t` range to set.
*/
void LSM6DS::setAccelRange(lsm6ds_accel_range_t new_range) {

  Adafruit_BusIO_Register ctrl1 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DS_CTRL1_XL);

  Adafruit_BusIO_RegisterBits accel_range =
      Adafruit_BusIO_RegisterBits(&ctrl1, 2, 2);

  accel_range.write(new_range);
  delay(20);
}

/******************* Adafruit_Sensor functions *****************/
/*!
 *     @brief  Updates the measurement data for all sensors simultaneously
 */
/**************************************************************************/
void LSM6DS::_read(void) {
  // get raw readings
  Adafruit_BusIO_Register data_reg = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DS_OUT_TEMP_L, 14);

  uint8_t buffer[14];
  data_reg.read(buffer, 14);

  rawAccX = buffer[9] << 8 | buffer[8];
  rawAccY = buffer[11] << 8 | buffer[10];
  rawAccZ = buffer[13] << 8 | buffer[12];
  
  lsm6ds_accel_range_t accel_range = getAccelRange();
  float accel_scale = 1; // range is in milli-g per bit!
  if (accel_range == LSM6DS_ACCEL_RANGE_16_G)
    accel_scale = 0.488;
  if (accel_range == LSM6DS_ACCEL_RANGE_8_G)
    accel_scale = 0.244;
  if (accel_range == LSM6DS_ACCEL_RANGE_4_G)
    accel_scale = 0.122;
  if (accel_range == LSM6DS_ACCEL_RANGE_2_G)
    accel_scale = 0.061;

  accX = rawAccX * accel_scale * SENSORS_GRAVITY_STANDARD / 1000;
  accY = rawAccY * accel_scale * SENSORS_GRAVITY_STANDARD / 1000;
  accZ = rawAccZ * accel_scale * SENSORS_GRAVITY_STANDARD / 1000;
}

/**************************************************************************/
/*!
    @brief Sets the INT1 and INT2 pin activation mode
    @param active_low true to set the pins  as active high, false to set the
   mode to active low
    @param open_drain true to set the pin mode as open-drain, false to set the
   mode to push-pull
*/
void LSM6DS::configIntOutputs(bool active_low, bool open_drain) {

  Adafruit_BusIO_Register ctrl3 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DS_CTRL3_C);
  Adafruit_BusIO_RegisterBits ppod_bits =
      Adafruit_BusIO_RegisterBits(&ctrl3, 2, 4);

  ppod_bits.write((active_low << 1) | open_drain);
}

/**************************************************************************/
/*!
    @brief Enables and disables the data ready interrupt on INT 1.
    @param drdy_temp true to output the data ready temperature interrupt
    @param drdy_g true to output the data ready gyro interrupt
    @param drdy_xl true to output the data ready accelerometer interrupt
    @param step_detect true to output the step detection interrupt (default off)
    @param wakeup true to output the wake up interrupt (default off)
*/
void LSM6DS::configInt1(bool drdy_temp, bool drdy_g, bool drdy_xl,
                                 bool step_detect, bool wakeup) {

  Adafruit_BusIO_Register int1_ctrl = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DS_INT1_CTRL);

  int1_ctrl.write((step_detect << 7) | (drdy_temp << 2) | (drdy_g << 1) |
                  drdy_xl);

  Adafruit_BusIO_Register md1cfg = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DS_MD1_CFG);

  Adafruit_BusIO_RegisterBits wu = Adafruit_BusIO_RegisterBits(&md1cfg, 1, 5);
  wu.write(wakeup);
}

/**************************************************************************/
/*!
    @brief  Gets the sensor_t data for the LSM6DS's accelerometer
*/
/**************************************************************************/
void LSM6DS_Accelerometer::getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, "LSM6DS_A", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_ACCELEROMETER;
  sensor->min_delay = 0;
  sensor->min_value = -156.9064F; /*  -16g = 156.9064 m/s^2  */
  sensor->max_value = 156.9064F;  /* 16g = 156.9064 m/s^2  */
  sensor->resolution = 0.061;     /* 0.061 mg/LSB at +-2g */
}

/**************************************************************************/
/*!
    @brief  Gets the accelerometer as a standard sensor event
    @param  event Sensor event object that will be populated
    @returns True
*/
/**************************************************************************/
bool LSM6DS_Accelerometer::getEvent(sensors_event_t *event) {
  _theLSM6DS->_read();
  _theLSM6DS->fillAccelEvent(event, millis());

  return true;
}