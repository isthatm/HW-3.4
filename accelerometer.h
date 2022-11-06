#include "mbed.h"
#include "stm32l475e_iot01_accelero.h"

#define SENSOR_G 9806650LL
#define SENSOR_G_DOUBLE (SENSOR_G / 1000000.0)
#define ACCEL_FULLSCALE_FACTOR (2.0 * SENSOR_G_DOUBLE)
const double scale = ACCEL_FULLSCALE_FACTOR;

class Accelerometer {

public:
  Accelerometer() {

    BSP_ACCELERO_Init();
    for (int i = 0; i < 3; i++) {
      // initial internal data
      accDataXYZ[i] = 0;
      acc_accel[i] = 0;
      avg_accel[i] = 0;
      err_accel[i] = 0;
    }
    calibrate();
  }

  void calibrate() {
    int count = 0;
    // printf("Measuring 200 samples to calculate the error.\n");
    for (int i = 0; i < 200; ++i) {
      BSP_ACCELERO_AccGetXYZ(accDataXYZ);

      double accel_sample_x = (double)accDataXYZ[0] * (double)scale / 20640.0;
      double accel_sample_y = (double)accDataXYZ[1] * (double)scale / 20640.0;
      double accel_sample_z = (double)accDataXYZ[2] * (double)scale / 20640.0;

      // Accumulate
      count++;
      acc_accel[0] += accel_sample_x;
      acc_accel[1] += accel_sample_y;
      acc_accel[2] += accel_sample_z;

      // printf("ACC= %f, %f, %f\n", accel_sample_x, accel_sample_y, accel_sample_z);
    }
    // avarage the sum (accumulated acceleration) of the samples above
    for (int i = 0; i < 3; ++i) {
      avg_accel[i] = acc_accel[i] / count;
    }
    // printf("Average ACC= %f, %f, %f\n", avg_accel[0], avg_accel[1], avg_accel[2]);

    for (int i = 0; i < 3; ++i) {
      err_accel[i] = avg_accel[i];
    }
    err_accel[2] = avg_accel[2] - 1; // Assume 1G=1 m/s2
    //printf("Average ACC error= %f, %f, %f\n", err_accel[0], err_accel[1], err_accel[2]);
  }
   // HW3
  //Access the sensor and return scaled data
  void GetAcceleromterSensor(double AccData[3]){

      int count2 = 0;
      for (int i = 0; i < 10; i++){
          BSP_ACCELERO_AccGetXYZ(accDataXYZ);

         acc_accel[0] = (double)accDataXYZ[0] * (double)scale / 20640.0;
         acc_accel[1] = (double)accDataXYZ[1] * (double)scale / 20640.0;
         acc_accel[2] = (double)accDataXYZ[2] * (double)scale / 20640.0;

         count2++;
         AccData[0] += acc_accel[0];
         AccData[1] += acc_accel[1];
         AccData[2] += acc_accel[2];
      }

      for (int i = 0; i < 3; ++i) {
          AccData[i] = AccData[i] / count2;
        }

  }

  //Read from the current calibrated Acc sample
  void GetAcceleromterCalibratedData(double AccData[3]){
      /*
      AccData[0] = AccData[0]-err_accel[0];
      AccData[1] = AccData[1]-err_accel[1];
      AccData[2] = AccData[2]-err_accel[2];
      */
      AccData[0] = acc_accel[0]-err_accel[0];
      AccData[1] = acc_accel[1]-err_accel[1];
      AccData[2] = acc_accel[2]-err_accel[2];
    
  }

  //Read from the current Acc sample
  void GetAcceleromterData(double AccData[3]){

      AccData[0] = acc_accel[0];
      AccData[1] = acc_accel[1];
      AccData[2] = acc_accel[2];
  }

private:

  int16_t accDataXYZ[3];
  double acc_accel[3];  //current sample
  double avg_accel[3];
  double err_accel[3]; // average error terms
};