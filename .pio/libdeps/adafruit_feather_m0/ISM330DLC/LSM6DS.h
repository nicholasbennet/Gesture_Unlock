/*!
 *  @file LSM6DS.h
 *
 * 	I2C Driver base for Adafruit LSM6DSxx 6-DoF Accelerometer and Gyroscope
 *      library
 *
 * 	Adafruit invests time and resources providing this open source code,
 *      please support Adafruit and open-source hardware by purchasing products
 *from Adafruit!
 *
 *	BSD license (see license.txt)
 */

#ifndef _LSM6DS_H
#define _LSM6DS_H

#include "Arduino.h"
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define LSM6DS_I2CADDR_DEFAULT 0x6A ///< LSM6DS default i2c address

#define LSM6DS_FUNC_CFG_ACCESS 0x1 ///< Enable embedded functions register
#define LSM6DS_INT1_CTRL 0x0D      ///< Interrupt control for INT 1
#define LSM6DS_INT2_CTRL 0x0E      ///< Interrupt control for INT 2
#define LSM6DS_WHOAMI 0xF          ///< Chip ID register
#define LSM6DS_CTRL1_XL 0x10       ///< Main accelerometer config register
#define LSM6DS_CTRL2_G 0x11        ///< Main gyro config register
#define LSM6DS_CTRL3_C 0x12        ///< Main configuration register
#define LSM6DS_CTRL8_XL 0x17       ///< High and low pass for accel
#define LSM6DS_CTRL10_C 0x19       ///< Main configuration register
#define LSM6DS_WAKEUP_SRC 0x1B     ///< Why we woke up
#define LSM6DS_OUT_TEMP_L 0x20     ///< First data register (temperature low)
#define LSM6DS_OUTX_L_G 0x22       ///< First gyro data register
#define LSM6DS_OUTX_L_A 0x28       ///< First accel data register
#define LSM6DS_STEPCOUNTER 0x4B    ///< 16-bit step counter
#define LSM6DS_TAP_CFG 0x58        ///< Tap/pedometer configuration
#define LSM6DS_WAKEUP_THS                                                      \
  0x5B ///< Single and double-tap function threshold register
#define LSM6DS_WAKEUP_DUR                                                      \
  0x5C ///< Free-fall, wakeup, timestamp and sleep mode duration
#define LSM6DS_MD1_CFG 0x5E ///< Functions routing on INT1 register

/** The accelerometer data rate */
typedef enum data_rate {
  LSM6DS_RATE_SHUTDOWN,
  LSM6DS_RATE_12_5_HZ,
  LSM6DS_RATE_26_HZ,
  LSM6DS_RATE_52_HZ,
  LSM6DS_RATE_104_HZ,
  LSM6DS_RATE_208_HZ,
  LSM6DS_RATE_416_HZ,
  LSM6DS_RATE_833_HZ,
  LSM6DS_RATE_1_66K_HZ,
  LSM6DS_RATE_3_33K_HZ,
  LSM6DS_RATE_6_66K_HZ,
} lsm6ds_data_rate_t;

/** The accelerometer data range */
typedef enum accel_range {
  LSM6DS_ACCEL_RANGE_2_G,
  LSM6DS_ACCEL_RANGE_16_G,
  LSM6DS_ACCEL_RANGE_4_G,
  LSM6DS_ACCEL_RANGE_8_G
} lsm6ds_accel_range_t;

class LSM6DS;

/** Adafruit Unified Sensor interface for accelerometer component of LSM6DS */
class LSM6DS_Accelerometer : public Adafruit_Sensor {
public:
  /** @brief Create an Adafruit_Sensor compatible object for the accelerometer
     sensor
      @param parent A pointer to the LSM6DS class */
  LSM6DS_Accelerometer(LSM6DS *parent) {
    _theLSM6DS = parent;
  }
  bool getEvent(sensors_event_t *);
  void getSensor(sensor_t *);

private:
  int _sensorID = 0x6D1;
  LSM6DS *_theLSM6DS = NULL;
};

/*!
 *    @brief  Class that stores state and functions for interacting with
 *            the LSM6DS I2C Accel/Gyro
 */
class LSM6DS {
public:
  LSM6DS();
  ~LSM6DS();

  bool begin_I2C(uint8_t i2c_addr = LSM6DS_I2CADDR_DEFAULT,
                 TwoWire *wire = &Wire, int32_t sensorID = 0);
  
  bool get_accel_Event(sensors_event_t *accel);

  void setAccelDataRate(lsm6ds_data_rate_t data_rate);

  lsm6ds_accel_range_t getAccelRange(void);
  void setAccelRange(lsm6ds_accel_range_t new_range);

  void reset(void);
  void configIntOutputs(bool active_low, bool open_drain);
  void configInt1(bool drdy_temp, bool drdy_g, bool drdy_xl,
                  bool step_detect = false, bool wakeup = false);

  int16_t rawAccX, ///< Last reading's raw accelerometer X axis
      rawAccY,     ///< Last reading's raw accelerometer Y axis
      rawAccZ;     ///< Last reading's raw accelerometer Z axis

  Adafruit_Sensor *getAccelerometerSensor(void);

protected:
  float accX,          ///< Last reading's accelerometer X axis m/s^2
      accY,          ///< Last reading's accelerometer Y axis m/s^2
      accZ;          ///< Last reading's accelerometer Z axis m/s^2
  uint8_t chipID();
  void _read(void);
  virtual bool _init(int32_t sensor_id);

  uint16_t _sensorid_accel; ///< ID number for accelerometer

  Adafruit_I2CDevice *i2c_dev = NULL; ///< Pointer to I2C bus interface
  Adafruit_SPIDevice *spi_dev = NULL; ///< Pointer to SPI bus interface

  LSM6DS_Accelerometer *accel_sensor =
      NULL;                                 ///< Accelerometer data object

private:
  friend class LSM6DS_Accelerometer; ///< Gives access to private
                                              ///< members to Accelerometer data
                                              ///< object

  void fillAccelEvent(sensors_event_t *accel, uint32_t timestamp);
};

#endif