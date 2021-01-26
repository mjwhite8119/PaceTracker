#ifndef _TRACKER_H_
#define _TRACKER_H_

#include "IMU.h"

#include "frc/Rotation2d.h"

class Tracker
{
  public:
    
    // Constructor
    Tracker() {}

    // Define the IMU
    IMU imu;

    /**
     * Gets the robot heading from the IMU
     * 
     * @return Rotation2d object that includes the robot heading
     */
    Rotation2d getRotation() {
      imu.update();
      float heading = imu.getRotationRadians();
      if (heading < 0.0) {heading += TWO_PI;}
      return Rotation2d(heading); 
    };

    /**
     * Set the initial heading of the robot
     *
     */
    void setGyroOffset() {
      Rotation2d offset = getRotation();
      log_d("Set Offset... %2.0f", offset.Degrees());

      gyroOffset_ += offset;
      log_d("Set Offset... %2.0f", gyroOffset_.Degrees());
    }

    /**
     * Gets the offset of the Gryo
     * 
     * @return Rotation2d object that includes the gyro offset
     */
    Rotation2d getGyroOffset() { return gyroOffset_; } 

    /**
     * Gets the acceleration from the IMU
     * 
     * @return VectorInt16 object that contains acceleration
     */
    VectorInt16 getAcceleration() {
      return imu.getAcceleration();
    }; 

  private:
    Rotation2d gyroOffset_; // The initial offset of the robot heading
};

#endif // _TRACKER_H_