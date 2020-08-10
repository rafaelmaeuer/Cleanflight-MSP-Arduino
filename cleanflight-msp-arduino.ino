#include <MSP.h>
#include <SoftwareSerial.h>

MSP msp;
SoftwareSerial mspSerial(10, 11); // RX TX
    
void setup() {
  Serial.begin(19200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("Start");
  
  mspSerial.begin(19200);
  msp.begin(mspSerial);
}

void loop() {
  mspGPS();
  mspStatusEx();
  //mspCalibData();
  //mspImuData();
}

void mspImuData() {
  msp_raw_imu_t imu;
  if (msp.request(MSP_RAW_IMU, &imu, sizeof(imu))) {
    uint16_t acc_x    = imu.acc[0];
    uint16_t acc_y    = imu.acc[1];
    uint16_t acc_z    = imu.acc[2];
    uint16_t gyro_x   = imu.gyro[0];
    uint16_t gyro_y   = imu.gyro[1];
    uint16_t gyro_z   = imu.gyro[2];
    //uint16_t mag_x    = imu.mag[0];
    //uint16_t mag_y    = imu.mag[1];
    //uint16_t mag_z    = imu.mag[2];
  Serial.println("Accelerometer[x,y,z]: " + String(acc_x/448)+ ","+ String(acc_y/448) +","+ String(acc_z/448));
  Serial.println("Gyroscope[x,y,z]: " + String(gyro_x/2000)+ ","+ String(gyro_y/2000) +","+ String(gyro_z/2000));
  //Serial.println("Magnetometer[x,y,z]: " + String(mag_x)+ ","+ String(mag_y) +","+ String(mag_z));
  }
}

void mspStatusEx() {
  msp_status_ex_t status_ex;
  if (msp.request(MSP_STATUS_EX, &status_ex, sizeof(status_ex))) {
    
    uint32_t activeModes = status_ex.flightModeFlags;
    if (activeModes>>MSP_MODE_ARM & 1) {
      Serial.println("ARMED");
    } else {
      Serial.println("NOT ARMED");
    }
  }
}

void mspGPS() {
  msp_raw_gps_t gps;
  if (msp.request(MSP_RAW_GPS, &gps, sizeof(gps))) {
    
    uint8_t fixType = gps.fixType;
    if (fixType == MSP_GPS_FIX_3D) {
      Serial.println("GPS FIX 3D");
    } else if (fixType == MSP_GPS_FIX_2D) {
      Serial.println("GPS FIX 2D");
    } else {
      Serial.println("GPS NO FIX");
    }
  }
}

void mspCalibData() {
  msp_calibration_data_t calib_data;
  if (msp.request(MSP_CALIBRATION_DATA, &calib_data, sizeof(calib_data))) {
    
    int16_t accZeroX = calib_data.accZeroX;
    int16_t accZeroY = calib_data.accZeroY;
    int16_t accZeroZ = calib_data.accZeroZ;
    int16_t accGainX = calib_data.accGainX;
    int16_t accGainY = calib_data.accGainY;
    int16_t accGainZ = calib_data.accGainZ;
    Serial.print("accZeroX: " + String(accZeroX));
    Serial.print(" accZeroY: " + String(accZeroY));
    Serial.println(" accZeroZ: " + String(accZeroZ));
    Serial.print("accGainX: " + String(accGainX));
    Serial.print(" accGainY: " + String(accGainY));
    Serial.println(" accGainZ: " + String(accGainZ));
  }
}
