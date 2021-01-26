#include "IMU.h"

// ------------------ Constructors ---------------------------------
IMU::IMU() {}

/*-------------------------------------Functions----------------------------------------*/

//--------------------------------------------//
// Read the sensor values
//--------------------------------------------//
void IMU::readIMU() {
  uint8_t buffer[14];
  I2Cdev::readBytes(imuAddress, MPU6050_RA_ACCEL_XOUT_H, 14, buffer);
  accX = (((int16_t)buffer[0]) << 8) | buffer[1];
  accY = (((int16_t)buffer[2]) << 8) | buffer[3];
  accZ = (((int16_t)buffer[4]) << 8) | buffer[5];
  gyroX = (((int16_t)buffer[8]) << 8) | buffer[9];
  gyroY = (((int16_t)buffer[10]) << 8) | buffer[11];
  gyroZ = (((int16_t)buffer[12]) << 8) | buffer[13];
}

//--------------------------------------------//
// Get the mean values from the sensor 
//--------------------------------------------//
void IMU::meanSensors() {
  long i=0,buff_ax=0,buff_ay=0,buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;
  
  while (i<(buffersize+101)){
    // read raw accel/gyro measurements from device
    readIMU();
    
    if (i>100 && i<=(buffersize+100)){ //First 100 measures are discarded
      buff_ax=buff_ax+accX;
      buff_ay=buff_ay+accY;
      buff_az=buff_az+accZ;
      buff_gx=buff_gx+gyroX;
      buff_gy=buff_gy+gyroY;
      buff_gz=buff_gz+gyroZ;
    }
    if (i==(buffersize+100)){
      mean_ax=buff_ax/buffersize;
      mean_ay=buff_ay/buffersize;
      mean_az=buff_az/buffersize;
      mean_gx=buff_gx/buffersize;
      mean_gy=buff_gy/buffersize;
      mean_gz=buff_gz/buffersize;
    }
    i++;
    delay(2); //Needed so we don't get repeated measures
  }
}

//--------------------------------------------//
// Calibrate sensor
//--------------------------------------------//
void IMU::calibrate() {
  log_d("Calibrate");

  ax_offset=-mean_ax/8;
  ay_offset=-mean_ay/8;
  az_offset=(16384-mean_az)/8;

  gx_offset=-mean_gx/4;
  gy_offset=-mean_gy/4;
  gz_offset=-mean_gz/4;
  while (1){
    int ready=0;
    mpu.setXAccelOffset(ax_offset);
    mpu.setYAccelOffset(ay_offset);
    mpu.setZAccelOffset(az_offset);

    mpu.setXGyroOffset(gx_offset);
    mpu.setYGyroOffset(gy_offset);
    mpu.setZGyroOffset(gz_offset);

    meanSensors();

    if (abs(mean_ax)<=acel_deadzone) ready++;
    else ax_offset=ax_offset-mean_ax/acel_deadzone;

    if (abs(mean_ay)<=acel_deadzone) ready++;
    else ay_offset=ay_offset-mean_ay/acel_deadzone;

    if (abs(16384-mean_az)<=acel_deadzone) ready++;
    else az_offset=az_offset+(16384-mean_az)/acel_deadzone;

    if (abs(mean_gx)<=giro_deadzone) ready++;
    else gx_offset=gx_offset-mean_gx/(giro_deadzone+1);

    if (abs(mean_gy)<=giro_deadzone) ready++;
    else gy_offset=gy_offset-mean_gy/(giro_deadzone+1);

    if (abs(mean_gz)<=giro_deadzone) ready++;
    else gz_offset=gz_offset-mean_gz/(giro_deadzone+1);

    if (ready==6) break;
  }
}

//----------------------------------------------------//
// Calibrate bias of the accelerometer and gyroscope
//----------------------------------------------------//
void IMU::calibrateSensor()
{
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);

  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  
  log_d("Reading sensors for first time...");

  meanSensors();
  delay(1000);
  
  log_d("Calculating offsets...");
  
  calibrate();
  delay(1000);

  // LOGLN("Reading sensors for second time...");
  meanSensors();
  
  log_d("FINISHED!");
  log_d("Sensor readings with offsets:");
  log_d("acc %d, %d, %d", mean_ax, mean_ay, mean_az); 
  log_d("gyro %d, %d, %d", mean_gx, mean_gy, mean_gz); 
  log_d("Your offsets:"); 
  log_d("acc %d, %d, %d", ax_offset, ay_offset, az_offset); 
  log_d("gyro %d, %d, %d", gx_offset, gy_offset,gz_offset); 
  log_d("Data is printed as: acelX acelY acelZ giroX giroY giroZ");
  log_d("Check that your sensor readings are close to 0 0 16384 0 0 0");
  log_d("If calibration was succesful write down your offsets so you can set them in your projects using something similar to mpu.setXAccelOffset(youroffset)");
  

}

//--------------------------------------------//
// Read the raw IMU data from the buffer
//--------------------------------------------//
void IMU::readFifoBuffer() {
  // Clear the buffer so as we can get fresh values
  // The sensor is running a lot faster than our sample period
  mpu.resetFIFO();
  
  // get current FIFO count
  uint16_t fifoCount = mpu.getFIFOCount();
  
  // wait for correct available data length, should be a VERY short wait
  while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

  // read a packet from FIFO
  mpu.getFIFOBytes(fifoBuffer, packetSize);
}

//--------------------------------------------//
// Get the initial yaw offset and use it to
// set the Kalman and Complimentary angles
//--------------------------------------------//
void IMU::setAngles() {
  
  // Get Euler angles in radians
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetEuler(euler, &q);
  
  // You need a positive number for counter-clockwise rotations 
  // and a negative number for clockwise rotations. So multiply by -1
  // Could also reverse the mount angle of the IMU.
  const float yaw = euler[YAW] * RAD_TO_DEG; // Yaw

  // Save the yaw zero offset
  yaw_zero_offset = euler[YAW] * RAD_TO_DEG;
  const float pitch = euler[PITCH] * RAD_TO_DEG;

  compYaw = yaw; // Yaw angle in degrees
  compPitch = pitch; // Pitch angle in degrees

  log_d("Initial compYaw and yaw_zero_offset %d", compYaw);
  log_d("Initial yaw_zero_offset %d", yaw_zero_offset);  
}

//--------------------------------------------//
// Initialize the sensor
//--------------------------------------------//
void IMU::initMPU6050(void)
{
  Wire.begin();
  mpu.initialize();
  
  // Verify connection
  // log_d(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  log_d("Initializing DMP...");
  devStatus = mpu.dmpInitialize();

  log_d("Setting offsets:");

  // ----------------- Calibrate sensors -------------------------------------//
  // Calibrating needs to be calibrated at each power cycle.
  // The results can be saved into the parameter server so as we don't have to
  // run this routine everytime.
  #if CALIBRATE_SENSORS == true
    calibrateSensor();
  #else
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(gyroBiasX);
    mpu.setYGyroOffset(gyroBiasY);
    mpu.setZGyroOffset(gyroBiasZ);
    mpu.setXAccelOffset(accBiasX);
    mpu.setYAccelOffset(accBiasY);
    mpu.setZAccelOffset(accBiasZ); // 1688 factory default for my test chip

    // Log offsets to console
    log_d("accBiasX %d", accBiasX); 
    log_d("accBiasY %d", accBiasY); 
    log_d("accBiasZ %d", accBiasZ); 
    log_d("gyroBiasX %d", gyroBiasX); 
    log_d("gyroBiasY %d", gyroBiasY); 
    log_d("gyroBiasZ %d", gyroBiasZ); 
  #endif

  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    // log_dln("Enabling DMP...");
    mpu.setDMPEnabled(true);

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    // log_dln("DMP ready! Waiting for first interrupt...");
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();

    // Read the raw data from the IMU buffer
    readFifoBuffer();
  
  } else {
    // ERROR!
    log_d("DMP Initialization failed code ");
  }

}

//----------------------------------------------------------//
// Get the Euler angles from the IMU and convert to degrees
//----------------------------------------------------------//
IMU::EulerAngles IMU::getEulerAngles() {

  // Get Euler angles in radians
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetEuler(euler, &q);
  
  // You need a positive number for counter-clockwise rotations 
  // and a negative number for clockwise rotations. So multiply by -1
  float yaw = -( (euler[YAW]) - yaw_zero_offset);  // Yaw (inverted)
  const float pitch = euler[PITCH];  // Pitch
  const float roll = euler[ROLL];  // Roll
  // log_d("YAW (radians) %2.2f", yaw); 
  // log_d("YAW (degrees) %2.0f", yaw * RAD_TO_DEG);
  // log_d("LASTYAW %2.2f", lastYaw); 

  // Keep the angle sign the same
  // if ( sgn(lastYaw) != sgn(yaw) ) {
  //   log_d("ANGLE SWITCHED SIGN", lastYaw * RAD_TO_DEG, " to ", yaw * RAD_TO_DEG);
  // }

  // if (fabs(diff) > PI/2 && diff < std::round(TWO_PI)) {
  //   log_d("BAD READ sending", lastYaw * RAD_TO_DEG);
  //   yaw = lastYaw;
  // }
  lastYaw = yaw;

  return EulerAngles{yaw, pitch, roll};
}

//-----------------------------------------------//
// Get angular velocity in degrees
//-----------------------------------------------//
void IMU::getAngularVelocity() {

  float currentAngularVelocity[3];
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(currentAngularVelocity, &q, &gravity);

  angularVelocity[PITCH] = (lastAngularVelocity[PITCH] - currentAngularVelocity[PITCH]); // Pitch
  angularVelocity[YAW] = (lastAngularVelocity[YAW] - currentAngularVelocity[YAW]); // Yaw
  angularVelocity[ROLL] = 0; // Roll.  Zero in planar mode

  // Save the previous values
  lastAngularVelocity[YAW] = currentAngularVelocity[YAW];
  lastAngularVelocity[PITCH] = currentAngularVelocity[PITCH];
  lastAngularVelocity[ROLL] = currentAngularVelocity[ROLL]; 
}

//-----------------------------------------------//
// Get and report the linear acceleration values
//-----------------------------------------------//
VectorInt16 IMU::getAcceleration() {

  // display real acceleration, adjusted to remove gravity
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

  return aaReal; 
}

//--------------------------------------------//
// Run from loop()
//--------------------------------------------//
void IMU::update() {

  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // Read the raw data from the IMU buffer
  readFifoBuffer();
  
  // --- Calculate angular velocity ----- //
  // getAngularVelocity();

  getAcceleration();

  getEulerAngles();

}