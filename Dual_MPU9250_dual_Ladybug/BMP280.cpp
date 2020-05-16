#include "BMP280.h"

/*全局变量*/
bmp280_calib_data _bmp280_calib;
int32_t t_fine;
uint8_t address = 0x77;
ctrl_meas _measReg;
config _configReg;

void bmp_write8(uint8_t reg, uint8_t val) {
    Wire.beginTransmission((uint8_t)address);
    Wire.write((uint8_t)reg);
    Wire.write((uint8_t)val);
    Wire.endTransmission();
}
uint8_t bmp_read8(uint8_t reg){
  uint8_t val;
  Wire.beginTransmission((uint8_t)address);
  Wire.write((uint8_t)reg);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)address,1);
  val = Wire.read();
  return val;
  }
uint16_t bmp_read16(uint8_t reg){
  uint16_t val;
  Wire.beginTransmission((uint8_t)address);
  Wire.write((uint8_t)reg);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)address,2);
  val = (Wire.read()<<8) | Wire.read();
  return val;
  }
uint32_t bmp_read24(uint8_t reg){
  uint32_t val;
  Wire.beginTransmission((uint8_t)address);
  Wire.write((uint8_t)reg);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)address,3);
  val = Wire.read();
  val <<= 8;
  val |= Wire.read();
  val <<= 8;
  val |= Wire.read();
  return val;
  }

uint16_t bmp_read16_LE(uint8_t reg) {
  uint16_t temp = bmp_read16(reg);
  return (temp >> 8) | (temp << 8);
}
int16_t bmp_readS16(byte reg){ 
return (int16_t)bmp_read16(reg);
}

int16_t bmp_readS16_LE(byte reg) {
  return (int16_t)bmp_read16_LE(reg);
}

void bmp_readCoefficients() {
  _bmp280_calib.dig_T1 = bmp_read16_LE(BMP280_REGISTER_DIG_T1);
  _bmp280_calib.dig_T2 = bmp_readS16_LE(BMP280_REGISTER_DIG_T2);
  _bmp280_calib.dig_T3 = bmp_readS16_LE(BMP280_REGISTER_DIG_T3);

  _bmp280_calib.dig_P1 = bmp_read16_LE(BMP280_REGISTER_DIG_P1);
  _bmp280_calib.dig_P2 = bmp_readS16_LE(BMP280_REGISTER_DIG_P2);
  _bmp280_calib.dig_P3 = bmp_readS16_LE(BMP280_REGISTER_DIG_P3);
  _bmp280_calib.dig_P4 = bmp_readS16_LE(BMP280_REGISTER_DIG_P4);
  _bmp280_calib.dig_P5 = bmp_readS16_LE(BMP280_REGISTER_DIG_P5);
  _bmp280_calib.dig_P6 = bmp_readS16_LE(BMP280_REGISTER_DIG_P6);
  _bmp280_calib.dig_P7 = bmp_readS16_LE(BMP280_REGISTER_DIG_P7);
  _bmp280_calib.dig_P8 = bmp_readS16_LE(BMP280_REGISTER_DIG_P8);
  _bmp280_calib.dig_P9 = bmp_readS16_LE(BMP280_REGISTER_DIG_P9);
}
float bmp_readTemperature() {
  int32_t var1, var2;
  int32_t adc_T = bmp_read24(BMP280_REGISTER_TEMPDATA);
  adc_T >>= 4;
  var1 = ((((adc_T >> 3) - ((int32_t)_bmp280_calib.dig_T1 << 1))) *
          ((int32_t)_bmp280_calib.dig_T2)) >>
         11;
  var2 = (((((adc_T >> 4) - ((int32_t)_bmp280_calib.dig_T1)) *
            ((adc_T >> 4) - ((int32_t)_bmp280_calib.dig_T1))) >>
           12) *
          ((int32_t)_bmp280_calib.dig_T3)) >>
         14;
  t_fine = var1 + var2;

  float T = (t_fine * 5 + 128) >> 8;
  return T / 100;
}
uint8_t bmp_getStatus(void) {
  return bmp_read8(BMP280_REGISTER_STATUS);
}
void bmp_setSampling() {
    sensor_mode mode = MODE_NORMAL;
    sensor_sampling tempSampling = SAMPLING_X16;
    sensor_sampling pressSampling = SAMPLING_X16;
    sensor_filter filter = FILTER_OFF;
    standby_duration duration = STANDBY_MS_1;
    _measReg.mode = mode;
    _measReg.osrs_t = tempSampling;
    _measReg.osrs_p = pressSampling;

    _configReg.filter = filter;
    _configReg.t_sb = duration;

    bmp_write8(BMP280_REGISTER_CONFIG, _configReg.get());
    bmp_write8(BMP280_REGISTER_CONTROL, _measReg.get());
}
void bmp_begin()
{
    bmp_readCoefficients();
    bmp_setSampling();
    delay(100);
}

float bmp_readPressure() {
  int64_t var1, var2, p;
  // Must be done first to get the t_fine variable set up
  bmp_readTemperature();
  int32_t adc_P = bmp_read24(BMP280_REGISTER_PRESSUREDATA);
  adc_P >>= 4;

  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)_bmp280_calib.dig_P6;
  var2 = var2 + ((var1 * (int64_t)_bmp280_calib.dig_P5) << 17);
  var2 = var2 + (((int64_t)_bmp280_calib.dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)_bmp280_calib.dig_P3) >> 8) +
         ((var1 * (int64_t)_bmp280_calib.dig_P2) << 12);
  var1 =
      (((((int64_t)1) << 47) + var1)) * ((int64_t)_bmp280_calib.dig_P1) >> 33;

  if (var1 == 0) {
    return 0; // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)_bmp280_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)_bmp280_calib.dig_P8) * p) >> 19;

  p = ((p + var1 + var2) >> 8) + (((int64_t)_bmp280_calib.dig_P7) << 4);
  return (float)p / 256;
}

float bmp_readAltitude(float seaLevelhPa) {
  float altitude;
  float pressure = bmp_readPressure(); // in Si units for Pascal
  pressure /= 100;

  altitude = 44330 * (1.0 - pow(pressure / seaLevelhPa, 0.1903));

  return altitude;
}