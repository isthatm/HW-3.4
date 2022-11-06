#include "mbed.h"
#include "accelerometer.h"
#include "gyro.h"

const int16_t waveformLength = 128;
const int16_t lookUpTableDelay = 10;

AnalogOut Aout(PA_4);
InterruptIn btnRecord(BUTTON1);
EventQueue queue(32 * EVENTS_EVENT_SIZE);
EventQueue queue_note(32 * EVENTS_EVENT_SIZE);
Thread t; 

Accelerometer acc;
Gyro gyro;
double Accel[3]={0};
double Gyro[3]={0};
double  accAngleX=0;
double  accAngleY=0;
double elapsedTime=0;
double roll, pitch, yaw;
double gyroAngleX=0;
double gyroAngleY=0;
int counter=0;
int idR[32] = {0};
int indexR = 0;

void record(void) {

  acc.GetAcceleromterSensor(Accel);
  acc.GetAcceleromterCalibratedData(Accel);

  accAngleX = (atan(Accel[1] / sqrt(Accel[0]*Accel[0] + Accel[2]*Accel[2])) * 180 / SENSOR_PI_DOUBLE);
  accAngleY = (atan(-1 * Accel[0] / sqrt(Accel[1]*Accel[1] + Accel[2]*Accel[2])) * 180 / SENSOR_PI_DOUBLE);

  gyro.GetGyroSensor(Gyro);

  elapsedTime=0.1; //100ms by thread sleep time
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + Gyro[0] * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + Gyro[1] * elapsedTime;

  yaw =  yaw + Gyro[2] * elapsedTime;
  roll = accAngleX;
  pitch = accAngleY;
  printf("%f/%f/%f\n", roll, pitch, yaw);

}

void startRecord(void) {
  idR[indexR++] = queue.call_every(100ms, record);
  indexR = indexR % 32;
}

void stopRecord(void) {
  for (auto &i : idR)
    queue.cancel(i);

}

int main() {
  t.start(callback(&queue, &EventQueue::dispatch_forever));
  btnRecord.fall(queue.event(startRecord));
  btnRecord.rise(queue.event(stopRecord));
}
