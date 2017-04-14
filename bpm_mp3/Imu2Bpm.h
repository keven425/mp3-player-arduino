/*
IMU code adopted from:
https://github.com/TKJElectronics/Example-Sketch-for-IMU-including-Kalman-filter
http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/

FFT library adopted from:
https://github.com/kosme/arduinoFFT
*/

#include <StandardCplusplus.h>
#include <vector>
#include <Wire.h>
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter
#include "arduinoFFT.h"
#include "I2C.h"


// **************************
// ******** IMU init ********
// **************************
#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

class Imu2Bpm {
private:
  Kalman kalmanX, kalmanY, kalmanZ; // Create the Kalman instances

  const uint8_t MPU6050 = 0x68; // If AD0 is logic low on the PCB the address is 0x68, otherwise set this to 0x69
  const uint8_t HMC5883L = 0x0C; // Address of magnetometer

  double accX, accY, accZ;
  double gyroX, gyroY, gyroZ;
  double magX, magY, magZ;
  int16_t tempRaw;

  double roll, pitch, yaw; // Roll and pitch are calculated using the accelerometer while yaw is calculated using the magnetometer

  double gyroXangle, gyroYangle, gyroZangle; // Angle calculate using the gyro only
  double compAngleX, compAngleY, compAngleZ; // Calculated angle using a complementary filter
  double kalAngleX, kalAngleY, kalAngleZ; // Calculated angle using a Kalman filter

  uint32_t timer;
  uint32_t timerFFT;
  uint32_t index = 0; // ith loop
  uint32_t index_last = 0;
  uint8_t i2cData[14]; // Buffer for I2C data

  #define MAG0MAX 603
  #define MAG0MIN -578

  #define MAG1MAX 542
  #define MAG1MIN -701

  #define MAG2MAX 547
  #define MAG2MIN -556

  float magOffset[3] = { (MAG0MAX + MAG0MIN) / 2, (MAG1MAX + MAG1MIN) / 2, (MAG2MAX + MAG2MIN) / 2 };
  double magGain[3];


  // **************************
  // ******** FFT init ********
  // **************************
  arduinoFFT FFT = arduinoFFT(); /* Create FFT object */

  const uint16_t DELAY = 50;
  const uint16_t N_SAMPLES = 64; //This value MUST ALWAYS be a power of 2
  const uint16_t FFT_EVERY_N_SAMPLES = 16; // eval FFT every n samples

  //double vReal[N_SAMPLES];
  double vImag[N_SAMPLES];
  double angles[N_SAMPLES]; // angles array for FFT
  int angles_index = 0; // ring buffer index

  #define SCL_INDEX 0x00
  #define SCL_TIME 0x01
  #define SCL_FREQUENCY 0x02
  
  void PrintVector(double *vData, uint8_t bufferSize, uint8_t scaleType, double samplingFrequency) {
    for (uint16_t i = 0; i < bufferSize; i++)
    {
      double abscissa;
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

  void updateHMC5883L() {
    while (i2cRead(HMC5883L, 0x03, i2cData, 6)); // Get magnetometer values
    magX = ((i2cData[0] << 8) | i2cData[1]);
    magZ = ((i2cData[2] << 8) | i2cData[3]);
    magY = ((i2cData[4] << 8) | i2cData[5]);
  }

  void updatePitchRoll() {
    // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
    // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
    // It is then converted from radians to degrees
  #ifdef RESTRICT_PITCH // Eq. 25 and 26
    roll = atan2(accY, accZ) * RAD_TO_DEG;
    pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  #else // Eq. 28 and 29
    roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  #endif
  }

  void updateYaw() { // See: http://www.freescale.com/files/sensors/doc/app_note/AN4248.pdf
    magX *= -1; // Invert axis - this it done here, as it should be done after the calibration
    magZ *= -1;

    magX *= magGain[0];
    magY *= magGain[1];
    magZ *= magGain[2];

    magX -= magOffset[0];
    magY -= magOffset[1];
    magZ -= magOffset[2];

    double rollAngle = kalAngleX * DEG_TO_RAD;
    double pitchAngle = kalAngleY * DEG_TO_RAD;

    double Bfy = magZ * sin(rollAngle) - magY * cos(rollAngle);
    double Bfx = magX * cos(pitchAngle) + magY * sin(pitchAngle) * sin(rollAngle) + magZ * sin(pitchAngle) * cos(rollAngle);
    yaw = atan2(-Bfy, Bfx) * RAD_TO_DEG;

    yaw *= -1;
  }

  void calibrateMag() { // Inspired by: https://code.google.com/p/open-headtracker/
    i2cWrite(HMC5883L, 0x00, 0x11, true);
    delay(100); // Wait for sensor to get ready
    updateHMC5883L(); // Read positive bias values

    int16_t magPosOff[3] = { magX, magY, magZ };

    i2cWrite(HMC5883L, 0x00, 0x12, true);
    delay(100); // Wait for sensor to get ready
    updateHMC5883L(); // Read negative bias values

    int16_t magNegOff[3] = { magX, magY, magZ };

    i2cWrite(HMC5883L, 0x00, 0x10, true); // Back to normal

    magGain[0] = -2500 / float(magNegOff[0] - magPosOff[0]);
    magGain[1] = -2500 / float(magNegOff[1] - magPosOff[1]);
    magGain[2] = -2500 / float(magNegOff[2] - magPosOff[2]);

  #if 1
    Serial.print("Mag cal: ");
    Serial.print(magNegOff[0] - magPosOff[0]);
    Serial.print(",");
    Serial.print(magNegOff[1] - magPosOff[1]);
    Serial.print(",");
    Serial.println(magNegOff[2] - magPosOff[2]);

    Serial.print("Gain: ");
    Serial.print(magGain[0]);
    Serial.print(",");
    Serial.print(magGain[1]);
    Serial.print(",");
    Serial.println(magGain[2]);
  #endif
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
  void append_angles(double value) {
    angles[angles_index] = value;
    angles_index++;
    angles_index %= N_SAMPLES;
  }

  // return continuous array of angles
  double * get_angles() {
    static double _angles[N_SAMPLES];
    std::copy(angles + angles_index, angles + N_SAMPLES, _angles);
    std::copy(angles, angles + angles_index, _angles + N_SAMPLES - angles_index);
    return _angles;
  }


public:
  Imu2Bpm() {
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

    while (i2cWrite(HMC5883L, 0x02, 0x00, true)); // Configure device for continuous mode
    calibrateMag();

    delay(100); // Wait for sensors to stabilize

    /* Set Kalman and gyro starting angle */
    updateMPU6050();
    updateHMC5883L();
    updatePitchRoll();
    updateYaw();

    kalmanX.setAngle(roll); // First set roll starting angle
    gyroXangle = roll;
    compAngleX = roll;

    kalmanY.setAngle(pitch); // Then pitch
    gyroYangle = pitch;
    compAngleY = pitch;

    kalmanZ.setAngle(yaw); // And finally yaw
    gyroZangle = yaw;
    compAngleZ = yaw;

    timer = micros(); // Initialize the timer
    timerFFT = micros(); // init timer for computing sampling frequency for FFT
  }

  void loop() {
    // **************************
    // *** IMU calculations 
    // **************************
    /* Update all the IMU values */
    updateMPU6050();
    updateHMC5883L();

    double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
    timer = micros();


    /* Roll and pitch estimation */
    updatePitchRoll();
    double gyroXrate = gyroX / 131.0; // Convert to deg/s
    double gyroYrate = gyroY / 131.0; // Convert to deg/s

  #ifdef RESTRICT_PITCH
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
  #else
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
      kalmanY.setAngle(pitch);
      compAngleY = pitch;
      kalAngleY = pitch;
      gyroYangle = pitch;
    } else
      kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

    if (abs(kalAngleY) > 90)
      gyroXrate = -gyroXrate; // Invert rate, so it fits the restricted accelerometer reading
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
  #endif

    /* Yaw estimation */
    updateYaw();
    double gyroZrate = gyroZ / 131.0; // Convert to deg/s
    // This fixes the transition problem when the yaw angle jumps between -180 and 180 degrees
    if ((yaw < -90 && kalAngleZ > 90) || (yaw > 90 && kalAngleZ < -90)) {
      kalmanZ.setAngle(yaw);
      compAngleZ = yaw;
      kalAngleZ = yaw;
      gyroZangle = yaw;
    } else
      kalAngleZ = kalmanZ.getAngle(yaw, gyroZrate, dt); // Calculate the angle using a Kalman filter


    /* Estimate angles using gyro only */
    gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
    gyroYangle += gyroYrate * dt;
    gyroZangle += gyroZrate * dt;
    
    /* Estimate angles using complimentary filter */
    compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
    compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;
    compAngleZ = 0.93 * (compAngleZ + gyroZrate * dt) + 0.07 * yaw;

    // Reset the gyro angles when they has drifted too much
    if (gyroXangle < -180 || gyroXangle > 180)
      gyroXangle = kalAngleX;
    if (gyroYangle < -180 || gyroYangle > 180)
      gyroYangle = kalAngleY;
    if (gyroZangle < -180 || gyroZangle > 180)
      gyroZangle = kalAngleZ;

    print_imu_data();
  }
  
  int get_bpm() {
    // **************************
    // *** FFT calculations 
    // **************************
    // append to arrays
    append_angles(kalAngleX + kalAngleY);

    // if ((index >= N_SAMPLES) && (index % FFT_EVERY_N_SAMPLES == 0)) {
      double delta_sec = (micros() - timerFFT) / 1000000.0;
      int delta_samples = index - index_last;
      double samplingFrequency = delta_samples / delta_sec;
      timerFFT = micros(); // reset fft timer
      index_last = index; // reset index
      //    Serial.print("samplingFrequency: ");
      //    Serial.println(samplingFrequency);
      /*
      These are the input and output vectors
      Input vectors receive computed results from FFT
      */
      double * vReal = get_angles(); // access vector's array
      memset(vImag, 0, sizeof(vImag)); // reset vImag to zero
      
      // FFT calculations
      //    Serial.println("real Data:");
      //    PrintVector(vReal, N_SAMPLES, SCL_TIME, samplingFrequency);
      //    Serial.println("imag Data:");
      //    PrintVector(vImag, N_SAMPLES, SCL_TIME, samplingFrequency);
      FFT.Windowing(vReal, N_SAMPLES, FFT_WIN_TYP_RECTANGLE, FFT_FORWARD);  /* Weigh data */
      //    Serial.println("Weighed data:");
      //    PrintVector(vReal, N_SAMPLES, SCL_TIME, samplingFrequency);
      FFT.Compute(vReal, vImag, N_SAMPLES, FFT_FORWARD); /* Compute FFT */
      //    Serial.println("Computed Real values:");
      //    PrintVector(vReal, N_SAMPLES, SCL_INDEX, samplingFrequency);
      //    Serial.println("Computed Imaginary values:");
      //    PrintVector(vImag, N_SAMPLES, SCL_INDEX, samplingFrequency);
      FFT.ComplexToMagnitude(vReal, vImag, N_SAMPLES); /* Compute magnitudes */
      //    Serial.println("Computed magnitudes:");
      //    PrintVector(vReal, (N_SAMPLES >> 1), SCL_FREQUENCY, samplingFrequency);
      double major_peak = FFT.MajorPeak(vReal, N_SAMPLES, samplingFrequency);
      //    Serial.println(major_peak, 6);
      double bpm = 60.0 * major_peak;
      Serial.print("BPM: ");
      Serial.println(bpm);
      //    while(1);
    // }

    index++;
    // delay(DELAY);

    return (int)bpm;
  }
}