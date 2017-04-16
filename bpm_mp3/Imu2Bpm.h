/*
  Calculate beats-per-minute based on IMU accelerometer + gyroscope
  1. estimate pitch and roll using kalman filter, sensor fusion
  2. extract dominant frequency using fourier transform

  IMU code adopted from:
  https://github.com/TKJElectronics/Example-Sketch-for-IMU-including-Kalman-filter
  http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/

  FFT library adopted from:
  https://github.com/kosme/arduinoFFT
*/

// #include "StandardCplusplus.h"
// #include <algorithm>
#include <Wire.h>
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter
#include "arduinoFFT.h"
#include "I2C.h"

// **************************
// ******** IMU init ********
// **************************

class Imu2Bpm {
private:
  Kalman kalmanX, kalmanY; // Create the Kalman instances

  const uint8_t MPU6050 = 0x68; // If AD0 is logic low on the PCB the address is 0x68, otherwise set this to 0x69

  float accX, accY, accZ;
  float gyroX, gyroY, gyroZ;
  int16_t tempRaw;

  float roll, pitch; // Roll and pitch are calculated using the accelerometer while yaw is calculated using the magnetometer

  float gyroXangle, gyroYangle; // Angle calculate using the gyro only
  float compAngleX, compAngleY; // Calculated angle using a complementary filter
  float kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

  int bpm_cache;

  uint32_t timer;
  uint32_t timerFFT;
  uint32_t timer_poll;

  uint32_t index = 0; // ith loop
  uint32_t index_last = 0;
  uint8_t i2cData[14]; // Buffer for I2C data


  // **************************
  // ******** FFT init ********
  // **************************
  arduinoFFT FFT = arduinoFFT(); /* Create FFT object */

  const static uint16_t IMU_POLL_DELAY = 200;
  const static uint16_t N_SAMPLES = 32; //This value MUST ALWAYS be a power of 2
  const static uint16_t FFT_EVERY_N_SAMPLES = 16; // eval FFT every n samples

  float angles[N_SAMPLES]; // angles array for FFT
  int angles_index = 0; // ring buffer index

  #define SCL_INDEX 0x00
  #define SCL_TIME 0x01
  #define SCL_FREQUENCY 0x02
  
  void PrintVector(float *vData, uint8_t bufferSize, uint8_t scaleType, float samplingFrequency) {
    for (uint16_t i = 0; i < bufferSize; i++) {
      float abscissa;
      /* Print abscissa value */
      switch (scaleType)
      {
        case SCL_INDEX:
          abscissa = (i * 1.0);
        break;
            case SCL_TIME:
              abscissa = ((i * 1.0) / samplingFrequency);
        break;
            case SCL_FREQUENCY:
              abscissa = ((i * 1.0 * samplingFrequency) / N_SAMPLES);
        break;
      }
      Serial.print(abscissa, 6);
      Serial.print(" ");
      Serial.print(vData[i], 4);
      Serial.println();
    }
    Serial.println();
  }

  void updateMPU6050() {
    while (i2cRead(MPU6050, 0x3B, i2cData, 14)); // Get accelerometer and gyroscope values
    accX = ((i2cData[0] << 8) | i2cData[1]);
    accY = -((i2cData[2] << 8) | i2cData[3]);
    accZ = ((i2cData[4] << 8) | i2cData[5]);
    tempRaw = (i2cData[6] << 8) | i2cData[7];
    gyroX = -(i2cData[8] << 8) | i2cData[9];
    gyroY = (i2cData[10] << 8) | i2cData[11];
    gyroZ = -(i2cData[12] << 8) | i2cData[13];
  }

  void updatePitchRoll() {
    // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
    // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
    // It is then converted from radians to degrees
    roll = atan2(accY, accZ) * RAD_TO_DEG;
    pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  }

  void print_imu_data() {
      /* Print Data */
    #if 0
      Serial.print(roll); Serial.print("\t");
      Serial.print(gyroXangle); Serial.print("\t");
      Serial.print(compAngleX); Serial.print("\t");
      Serial.print(kalAngleX); Serial.print("\t");
    
      Serial.print("\t");
    
      Serial.print(pitch); Serial.print("\t");
      Serial.print(gyroYangle); Serial.print("\t");
      Serial.print(compAngleY); Serial.print("\t");
      Serial.print(kalAngleY); Serial.print("\t");
    
      Serial.print("\t");
    
      Serial.print(yaw); Serial.print("\t");
      Serial.print(gyroZangle); Serial.print("\t");
      Serial.print(compAngleZ); Serial.print("\t");
      Serial.print(kalAngleZ); Serial.print("\t");
    #endif
    #if 0 // Set to 1 to print the IMU data
      Serial.print(accX / 16384.0); Serial.print("\t"); // Converted into g's
      Serial.print(accY / 16384.0); Serial.print("\t");
      Serial.print(accZ / 16384.0); Serial.print("\t");
    
      Serial.print(gyroXrate); Serial.print("\t"); // Converted into degress per second
      Serial.print(gyroYrate); Serial.print("\t");
      Serial.print(gyroZrate); Serial.print("\t");
    
      Serial.print(magX); Serial.print("\t"); // After gain and offset compensation
      Serial.print(magY); Serial.print("\t");
      Serial.print(magZ); Serial.print("\t");
    #endif
    #if 0 // Set to 1 to print the temperature
      Serial.print("\t");
    
      double temperature = (double)tempRaw / 340.0 + 36.53;
      Serial.print(temperature); Serial.print("\t");
    #endif
  }

  // ring buffer implementation
  void append_angles(float value) {
    angles[angles_index] = value;
    angles_index++;
    angles_index %= N_SAMPLES;
  }

  // return continuous array of angles
  float * get_angles() {
    static float _angles[N_SAMPLES];
    memset(_angles, 0, sizeof(_angles));
    // std::copy(angles + angles_index, angles + N_SAMPLES, _angles);
    // std::copy(angles, angles + angles_index, _angles + N_SAMPLES - angles_index);
    memcpy(_angles, angles + angles_index, (N_SAMPLES - angles_index) * sizeof(float));
    memcpy(_angles + N_SAMPLES - angles_index, angles, angles_index * sizeof(float));
    return _angles;
  }


public:
  Imu2Bpm() {
    // no-op
  }

  void init() {
    delay(100); // Wait for sensors to get ready

    // Serial.begin(115200);
    Wire.begin();
    TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz

    i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
    i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
    i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
    i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
    while (i2cWrite(MPU6050, 0x19, i2cData, 4, false)); // Write to all four registers at once
    while (i2cWrite(MPU6050, 0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

    while (i2cRead(MPU6050, 0x75, i2cData, 1));
    if (i2cData[0] != 113) { // Read "WHO_AM_I" register
      Serial.print(F("Error reading sensor"));
      while (1);
    }

    delay(100); // Wait for sensors to stabilize

    /* Set Kalman and gyro starting angle */
    updateMPU6050();
    updatePitchRoll();

    kalmanX.setAngle(roll); // First set roll starting angle
    gyroXangle = roll;
    compAngleX = roll;

    kalmanY.setAngle(pitch); // Then pitch
    gyroYangle = pitch;
    compAngleY = pitch;

    timer = micros(); // Initialize the timer
    timerFFT = micros(); // init timer for computing sampling frequency for FFT
  }

  // return BPM
  int loop() {
    // poll every delay ms
    if (millis() - timer_poll < IMU_POLL_DELAY) {
      return bpm_cache;
    }
    timer_poll = millis();

    poll();

    if (index % FFT_EVERY_N_SAMPLES != 0) {
      return bpm_cache;
    }
    
    bpm_cache = get_bpm();
    return bpm_cache;
  }

  void poll() {
    // Serial.print("polling imu");

    // **************************
    // *** IMU calculations 
    // **************************
    /* Update all the IMU values */
    updateMPU6050();

    double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
    timer = micros();

    /* Roll and pitch estimation */
    updatePitchRoll();
    float gyroXrate = gyroX / 131.0; // Convert to deg/s
    float gyroYrate = gyroY / 131.0; // Convert to deg/s

    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
      kalmanX.setAngle(roll);
      compAngleX = roll;
      kalAngleX = roll;
      gyroXangle = roll;
    } else
      kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

    if (abs(kalAngleX) > 90)
      gyroYrate = -gyroYrate; // Invert rate, so it fits the restricted accelerometer reading
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);

    /* Estimate angles using gyro only */
    gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
    gyroYangle += gyroYrate * dt;
    
    /* Estimate angles using complimentary filter */
    compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
    compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

    // Reset the gyro angles when they has drifted too much
    if (gyroXangle < -180 || gyroXangle > 180)
      gyroXangle = kalAngleX;
    if (gyroYangle < -180 || gyroYangle > 180)
      gyroYangle = kalAngleY;

    print_imu_data();

    // append to arrays
    append_angles(kalAngleX + kalAngleY);
    index++;
  }
  
  int get_bpm() {
    // **************************
    // *** FFT calculations 
    // **************************

    double delta_sec = (micros() - timerFFT) / 1000000.0;
    int delta_samples = index - index_last;
    double samplingFrequency = delta_samples / delta_sec;
    timerFFT = micros(); // reset fft timer
    index_last = index; // reset index

    /*
    These are the input and output vectors
    Input vectors receive computed results from FFT
    */
    float * vReal = get_angles();
    float vImag[N_SAMPLES];
    memset(vImag, 0, sizeof(vImag)); // reset vImag to zero
    
    // FFT calculations
    // Serial.println("real Data:");
    // PrintVector(vReal, N_SAMPLES, SCL_TIME, samplingFrequency);
    // Serial.println("imag Data:");
    // PrintVector(vImag, N_SAMPLES, SCL_TIME, samplingFrequency);
    FFT.Windowing(vReal, N_SAMPLES, FFT_WIN_TYP_RECTANGLE, FFT_FORWARD);  /* Weigh data */
    FFT.Compute(vReal, vImag, N_SAMPLES, FFT_FORWARD); /* Compute FFT */
    FFT.ComplexToMagnitude(vReal, vImag, N_SAMPLES); /* Compute magnitudes */

    // Serial.println("Computed magnitudes:");
    // PrintVector(vReal, (N_SAMPLES >> 1), SCL_FREQUENCY, samplingFrequency);
    float major_peak = FFT.MajorPeak(vReal, N_SAMPLES, samplingFrequency);
    float bpm = 60.0 * major_peak;
    Serial.print(" - IMU BPM: ");
    Serial.println(bpm);

    return (int)bpm;
  }
};