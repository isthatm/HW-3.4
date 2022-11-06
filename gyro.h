#include "mbed.h"
#include "stm32l475e_iot01_gyro.h"

#define SENSOR_PI 3141592LL
#define SENSOR_PI_DOUBLE (SENSOR_PI / 1000000.0)
#define SENSOR_DEG2RAD_DOUBLE (SENSOR_PI_DOUBLE / 180)
#define LSM6DS0_DEFAULT_GYRO_FULLSCALE 0
#define LSM6DS0_DEFAULT_GYRO_FULLSCALE_FACTOR 8.75

class Gyro {

public:
  Gyro() {
    BSP_GYRO_Init();
    for (int i = 0; i < 3; i++) {
      // initial internal data
      gyroDataXYZ[i] = 0;
      acc_gyro[i] = 0;
      avg_gyro[i] = 0;
      err_gyro[i] = 0;
    }
    calibrate();
  }

  void calibrate() {
    int count = 0;
    double gyro_sample_x;
    double gyro_sample_y;
    double gyro_sample_z;
    // printf("Measuring 10 samples.\n");
    for (int i = 0; i < 200; ++i) {

      BSP_GYRO_GetXYZ(gyroDataXYZ);

      gyro_sample_x = (double)gyroDataXYZ[0] *
                             (double)LSM6DS0_DEFAULT_GYRO_FULLSCALE_FACTOR /
                             1000.0 * SENSOR_DEG2RAD_DOUBLE;
      gyro_sample_y = (double)gyroDataXYZ[1] *
                             (double)LSM6DS0_DEFAULT_GYRO_FULLSCALE_FACTOR /
                             1000.0 * SENSOR_DEG2RAD_DOUBLE;
       gyro_sample_z = (double)gyroDataXYZ[2] *
                             (double)LSM6DS0_DEFAULT_GYRO_FULLSCALE_FACTOR /
                             1000.0 * SENSOR_DEG2RAD_DOUBLE;

      // Accumulate
      count++;

      acc_gyro[0] += gyro_sample_x;
      acc_gyro[1] += gyro_sample_y;
      acc_gyro[2] += gyro_sample_z;
      // printf("Gyro= %f, %f, %f\n", gyro_sample_x, gyro_sample_y,
      // gyro_sample_z);
    }

    for (int i = 0; i < 3; ++i) {
      avg_gyro[i] = acc_gyro[i] / count;
    }
    // printf("Average Gyro= %f, %f, %f\n", avg_gyro[0], avg_gyro[1],
    // avg_gyro[2]);

    for (int i = 0; i < 3; ++i) {
      err_gyro[i] = avg_gyro[i];
    }

    //printf("Average Gyro error= %f, %f, %f\n", err_gyro[0], err_gyro[1], err_gyro[2]);
}
  // HW3
  // Access the sensor and return scaled data
  void GetGyroSensor(double GyroData[3]) {
    // Gyro measurement
    int count2 = 0;
    for(int i = 0; i < 10; i++){
        BSP_GYRO_GetXYZ(gyroDataXYZ);

        acc_gyro[0] = (double)gyroDataXYZ[0] *
                  (double)LSM6DS0_DEFAULT_GYRO_FULLSCALE_FACTOR / 1000.0 *
                  SENSOR_DEG2RAD_DOUBLE;
        acc_gyro[1] = (double)gyroDataXYZ[1] *
                  (double)LSM6DS0_DEFAULT_GYRO_FULLSCALE_FACTOR / 1000.0 *
                  SENSOR_DEG2RAD_DOUBLE;

        acc_gyro[2] = (double)gyroDataXYZ[2] *
                  (double)LSM6DS0_DEFAULT_GYRO_FULLSCALE_FACTOR / 1000.0 *
                  SENSOR_DEG2RAD_DOUBLE;
    count2++;
    GyroData[0] += acc_gyro[0];
    GyroData[1] += acc_gyro[1];
    GyroData[2] += acc_gyro[2];
    }

    for (int i = 0; i < 3; ++i) {
          GyroData[i] = GyroData[i] / count2;
        }
  }


//Read from the current calibrated Gyro sample
void GetGyroCalibratedData(double GyroData[3]){
  /*
  GyroData[0] = GyroData[0] - err_gyro[0];
  GyroData[1] = GyroData[1] - err_gyro[1];
  GyroData[2] = GyroData[2] - err_gyro[2];
    */
  GyroData[0] = acc_gyro[0] - err_gyro[0];
  GyroData[1] = acc_gyro[1] - err_gyro[1];
  GyroData[2] = acc_gyro[2] - err_gyro[2];
  
}

// Read from the current calibrated Gyro sample
void GetGyroData(double GyroData[3]) {

  GyroData[0] = acc_gyro[0];
  GyroData[1] = acc_gyro[1];
  GyroData[2] = acc_gyro[2];
}

private:
float gyroDataXYZ[3];
double acc_gyro[3];
double avg_gyro[3];
double err_gyro[3]; // average error terms
};