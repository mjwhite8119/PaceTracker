#ifndef _IMU_H_
#define _IMU_H_

/*  MPU6050 configuration and functions.  
 *  For the Heltec the I2C port is SDA 4, AND SCL 15, which
 *  Shares I2C bus with the OLED display and the ina219 I2C address is 0x40.
 *  
 *  Power supply: 3~5V.  3 axis gyroscope, 3 axis accelerator
 *  Communication mode: standard IIC communication protocol
 *  Chip built-in 16bit AD converter, 16bit data output
 *  Gyroscopes range: +/- 250 500 1000 2000 degree/sec
 *  Acceleration range: ±2 ±4 ±8 ±16g 
 */
#include <MPU6050_6Axis.h>

// Define I2C device addresses
#define PWR_MGMT_1    0x6B
#define MPU6050ADDR   0x68  //slave address
#define ACCEL_XOUT_H  0x3B
#define MPU6050_RA_ACCEL_XOUT_H 0x3B

// Calibrate the sensors on startup
#define CALIBRATE_SENSORS false

class IMU
{
  public:

    // Constructor
    IMU();

    // Assign mpu
    MPU6050 mpu;

    const uint16_t imuAddress = 0x68;
    const uint8_t registers = 14;

    const byte YAW = 0;
    const byte PITCH = 1;
    const byte ROLL = 2;

    struct EulerAngles {
      float yaw = 0.0;
      float pitch = 0.0;
      float roll = 0.0;
    };

    float euler[3]; // [psi, theta, phi]    Euler angle container
    float angularVelocity[3];   // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
    float lastAngularVelocity[3];   // [yaw, pitch, roll]
    
    float lastYaw = 0.0;
    float previousYaw = 0.0;
    float predictedYaw = 0.0;
    float yaw_zero_offset = 0.0;
    
    float compPitch = 0.0; 
    float compYaw = 0.0; // Calculated angle using a complementary filter
    float accYaw = 0.0;
    float kalPitch = 0.0;
    float kalYaw = 0.0; // Calculated angle using a Kalman filter

    // Sensor calibration variables
    // Use these values instead of running the calibrateSensor routine.
    static const int accBiasX = -3391;
    static const int accBiasY = -5495;
    static const int accBiasZ = 946;
    static const int gyroBiasX = 0;
    static const int gyroBiasY = 35;
    static const int gyroBiasZ = 136;

    // Raw sensor data
    int16_t accX=0, accY=0, accZ=0;
    int16_t gyroX=0, gyroY=0, gyroZ=0;

    //Change these 3 variables if you want to fine tune the sketch to your needs.
    const int buffersize=1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
    const int acel_deadzone=8;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
    const int giro_deadzone=1;     //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)

    int mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz;
    int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset;

    // MPU control/status vars
    bool dmpReady = false;  // set true if DMP init was successful
    uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
    uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
    // uint16_t fifoCount;     // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64]; // FIFO storage buffer

    // orientation/motion vars
    Quaternion q;           // [w, x, y, z]         quaternion container
    VectorInt16 aa;         // [x, y, z]            accel sensor measurements
    VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
    VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
    VectorFloat gravity;    // [x, y, z]            gravity vector

    // -------- Member functions ------------------

    void readIMU();

    void meanSensors();

    void calibrate();

    void calibrateSensor();

    void readFifoBuffer();

    void setAngles();

    void initMPU6050(void);

    EulerAngles getEulerAngles();

    void getAngularVelocity();

    VectorInt16 getAcceleration();

    float getXAcceleration() {return aaReal.x;}

    float getRotationDegrees() {
      EulerAngles angles = getEulerAngles();
      return angles.yaw * RAD_TO_DEG;  // Yaw
    }

    float getRotationRadians() {
      EulerAngles angles = getEulerAngles();
      return angles.yaw; // Yaw
    }

    float getRotationVelocityRadians() {
      return angularVelocity[YAW];
    }

    float getRotationVelocityDegrees() {
      return angularVelocity[YAW] * RAD_TO_DEG;
    }

    void update();

  private:

};
#endif // _IMU_H_