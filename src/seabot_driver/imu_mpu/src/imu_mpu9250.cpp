//  Copyright (c) 2014 Justin Eskesen
//
//  This file is part of i2c_imu
//
//  i2c_imu is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  i2c_imu is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with i2c_imu.  If not, see <http://www.gnu.org/licenses/>.
//

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <angles/angles.h>
#include <imu_mpu/ImuDebug.h>

#include "RTIMULib.h"
#include "RTIMUSettings.h"

#include <unistd.h>
#include <cstring>

#define G_2_MPSS 9.80665
#define uT_2_T 1000000.

class ImuSettings: public RTIMUSettings{
public:
  ImuSettings(ros::NodeHandle *nh):settings_nh_(nh){setDefaults();}
  virtual bool loadSettings();
private:
  ros::NodeHandle *settings_nh_; // private node handle
};

class I2cImu{
public:
  I2cImu();

  void update();
  void spin();
  void init();

private:
  //ROS Stuff
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  // sensor msg topic output
  sensor_msgs::Imu imu_msg;
  imu_mpu::ImuDebug debug_msg;

  ros::Publisher imu_pub_;
  ros::Publisher magnetometer_pub_;
  ros::Publisher euler_pub_;
  ros::Publisher imu_debug_pub_;

  std::string imu_frame_id_;

  ros::Time last_update_;
  double declination_radians_;
  double reset_norm_acc = 4;

  //RTUIMULib stuff
  RTIMU *imu_;

  ImuSettings imu_settings_;
};

bool ImuSettings::loadSettings(){
  ROS_DEBUG("[IMU] %s: reading IMU parameters from param server", __FUNCTION__);

  // General
  settings_nh_->getParam("imu_type", m_imuType);
  settings_nh_->getParam("fusion_type", m_fusionType);

  if(settings_nh_->param<bool>("is_i2c", true))
    m_busIsI2C = true;
  else
    m_busIsI2C = false;

  m_I2CSlaveAddress = (unsigned int)settings_nh_->param<int>("i2c_slave_address", 0x68);
  m_I2CBus = (unsigned int)settings_nh_->param<int>("i2c_bus", 1);

  m_SPIBus = (unsigned char)settings_nh_->param<int>("spi_bus", 0);
  m_SPISelect = (unsigned char)settings_nh_->param<int>("spi_select", 0);
  m_SPISpeed = (unsigned int)settings_nh_->param<int>("spi_speed", 500000);

  settings_nh_->getParam("axis_rotation", m_axisRotation);

  //MPU9250
  settings_nh_->getParam("mpu9250/gyro_accel_sample_rate", m_MPU9250GyroAccelSampleRate);
  settings_nh_->getParam("mpu9250/compass_sample_rate", m_MPU9250CompassSampleRate);
  settings_nh_->getParam("mpu9250/accel_full_scale_range", m_MPU9250AccelFsr);
  settings_nh_->getParam("mpu9250/accel_low_pass_filter", m_MPU9250AccelLpf);
  settings_nh_->getParam("mpu9250/gyro_full_scale_range", m_MPU9250GyroFsr);
  settings_nh_->getParam("mpu9250/gyro_low_pass_filter", m_MPU9250GyroLpf);

  /// ************** SENSOR CALIBRATION ************** ///
  // Max/min Compass (diseable option)
  m_compassCalMin = RTVector3(-1., -1., -1.);
  m_compassCalMax = RTVector3(1., 1., 1.);
  m_compassCalValid = true;

  // Ellipsoid offset Compass
  m_compassCalEllipsoidValid = true;
  std::vector<double> compass_ellipsoid_offset;
  if (settings_nh_->getParam("ellipsoid_offset", compass_ellipsoid_offset)
      && compass_ellipsoid_offset.size() == 3){
    m_compassCalEllipsoidOffset = RTVector3(compass_ellipsoid_offset[0], compass_ellipsoid_offset[1], compass_ellipsoid_offset[2]);
    ROS_DEBUG("[IMU] Got Calibration Ellipsoid Offset for Compass");
  }
  else{
    m_compassCalEllipsoidValid = false;
    ROS_INFO("[IMU] No Calibration Ellipsoid Offset for Compass");
  }

  std::vector<double> ellipsoid_corr0, ellipsoid_corr1, ellipsoid_corr2;
  if (settings_nh_->getParam("ellipsoid_matrix0", ellipsoid_corr0)
      && settings_nh_->getParam("ellipsoid_matrix1", ellipsoid_corr1)
      && settings_nh_->getParam("ellipsoid_matrix2", ellipsoid_corr2)
      && ellipsoid_corr0.size() == 3 && ellipsoid_corr1.size() == 3 && ellipsoid_corr2.size() == 3){
    m_compassCalEllipsoidCorr[0][0] = ellipsoid_corr0[0];
    m_compassCalEllipsoidCorr[0][1] = ellipsoid_corr0[1];
    m_compassCalEllipsoidCorr[0][2] = ellipsoid_corr0[2];
    m_compassCalEllipsoidCorr[1][0] = ellipsoid_corr1[0];
    m_compassCalEllipsoidCorr[1][1] = ellipsoid_corr1[1];
    m_compassCalEllipsoidCorr[1][2] = ellipsoid_corr1[2];
    m_compassCalEllipsoidCorr[2][0] = ellipsoid_corr2[0];
    m_compassCalEllipsoidCorr[2][1] = ellipsoid_corr2[1];
    m_compassCalEllipsoidCorr[2][2] = ellipsoid_corr2[2];
    ROS_DEBUG("[IMU] Got Calibration Ellipsoid Matrix for Compass");
  }
  else{
    m_compassCalEllipsoidValid = false;
    ROS_INFO("[IMU] No Calibration Ellipsoid Matrix for Compass");
  }

  // Compas Biais
  std::vector<double> gyro_bias;
  if (settings_nh_->getParam("gyro_bias", gyro_bias)
      && compass_ellipsoid_offset.size() == 3){
    m_gyroBias = RTVector3(gyro_bias[0], gyro_bias[1], gyro_bias[2]);
    m_gyroBiasValid = true;
    ROS_DEBUG("[IMU] Got Calibration Gyro Bias");
  }
  else{
    ROS_INFO("[IMU] No Calibration Gyro Bias");
  }

  // Min/Max Acc
  std::vector<double> accel_max, accel_min;
  if (settings_nh_->getParam("accel_min", accel_min)
      && settings_nh_->getParam("accel_max", accel_max)
      && accel_min.size() == 3 && accel_max.size() == 3)
  {
    m_accelCalMin = RTVector3(accel_min[0], accel_min[1], accel_min[2]);
    m_accelCalMax = RTVector3(accel_max[0],accel_max[1], accel_max[2]);
    m_accelCalValid = true;
    ROS_DEBUG("[IMU] Got Calibration for Accelerometer");
  }
  else
  {
    ROS_INFO("[IMU] No Calibration for Accelerometer");
  }

  // Mag Declination
  settings_nh_->getParam("mag_declination",m_compassAdjDeclination);

  return true;
}

I2cImu::I2cImu() : nh_(), private_nh_("~"), imu_settings_(&private_nh_){
  // do all the ros parameter reading & pulbishing
  private_nh_.param<std::string>("frame_id", imu_frame_id_, "imu_link");

  imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu",1);
  imu_debug_pub_ = nh_.advertise<imu_mpu::ImuDebug>("imu_debug", 1);

  if(private_nh_.param<bool>("publish_magnetometer", false))
    magnetometer_pub_ = nh_.advertise<geometry_msgs::Vector3>("mag", 1, false);

  if(private_nh_.param<bool>("publish_euler", false))
    euler_pub_ = nh_.advertise<geometry_msgs::Vector3>("euler", 1, false);

  reset_norm_acc = private_nh_.param<double>("reset_norm_acc", 4);

  imu_settings_.loadSettings();
  imu_settings_.saveSettings();

  declination_radians_ = private_nh_.param<double>("magnetic_declination", 0.0);

  // now set up the IMU

  ROS_INFO("[IMU] imu type = %i", imu_settings_.m_imuType);
  imu_ = RTIMU::createIMU(&imu_settings_);
  //  imu_->setDebugEnable(true);
  if (imu_ == nullptr){
    ROS_FATAL("I2cImu - %s - Failed to open the i2c device", __FUNCTION__);
    ROS_BREAK();
  }
  init();
}

void I2cImu::init(){
  if (!imu_->IMUInit()){ // After loading parameters
    ROS_FATAL("I2cImu - %s - Failed to init the IMU", __FUNCTION__);
    ROS_BREAK();
  }

  imu_->setSlerpPower(0.02);
  imu_->setGyroEnable(true);
  imu_->setAccelEnable(true);
  imu_->setCompassEnable(true);

  imu_->setCompassCalibrationMode(false);
  imu_->setAccelCalibrationMode(false);
}


void I2cImu::update(){
  bool valid = imu_->IMURead();
  if(valid){
    RTIMU_DATA imuData = imu_->getIMUData();
    debug_msg.readValid = true;

    /// ********** IMU msg **********
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.frame_id = imu_frame_id_;
    imu_msg.orientation.x = imuData.fusionQPose.x();
    imu_msg.orientation.y = imuData.fusionQPose.y();
    imu_msg.orientation.z = imuData.fusionQPose.z();
    imu_msg.orientation.w = imuData.fusionQPose.scalar();

    imu_msg.angular_velocity.x = imuData.gyro.x();
    imu_msg.angular_velocity.y = imuData.gyro.y();
    imu_msg.angular_velocity.z = imuData.gyro.z();

    imu_msg.linear_acceleration.x = imuData.accel.x();
    imu_msg.linear_acceleration.y = imuData.accel.y();
    imu_msg.linear_acceleration.z = imuData.accel.z();

    imu_pub_.publish(imu_msg);

    debug_msg.accelValid = imuData.accelValid;
    debug_msg.fusionPoseValid = imuData.fusionPoseValid;
    debug_msg.fusionQPoseValid = imuData.fusionQPoseValid;
    debug_msg.gyroValid = imuData.gyroValid;
    debug_msg.compassValid = imuData.compassValid;

    /// ********** Mag msg **********
    if (magnetometer_pub_ != nullptr && imuData.compassValid){
      geometry_msgs::Vector3 msg;

      msg.x = imuData.compass.x(); // in uT
      msg.y = imuData.compass.y(); // in uT
      msg.z = imuData.compass.z(); // in uT

      magnetometer_pub_.publish(msg);
    }

    /// ********** Euler msg **********
    if (euler_pub_ != nullptr){
      geometry_msgs::Vector3 msg;

      msg.x = imuData.fusionPose.x();
      msg.y = imuData.fusionPose.y();
      msg.z = imuData.fusionPose.z();

      euler_pub_.publish(msg);
    }

    // Test reset condition (norm of acc)
    if(pow(imuData.accel.x(),2)+pow(imuData.accel.y(),2)+pow(imuData.accel.z(),2)>pow(reset_norm_acc,2)){
      init();
      debug_msg.readValid = false;
    }
    imu_debug_pub_.publish(debug_msg);
  }
}

void I2cImu::spin(){
  ros::Rate r(1.0 / (imu_->IMUGetPollInterval() / 1000.0));

  if(!imu_->getCompassCalibrationValid())
    ROS_WARN("[IMU] Compass Calibration Offset not valid");
  if(!imu_->getCompassCalibrationEllipsoidValid())
    ROS_WARN("[IMU] Compass Calibration Ellipsoid not valid");
  if(!imu_->getAccelCalibrationValid())
    ROS_WARN("[IMU] Compass Calibration Acc not valid");

  while (ros::ok()){
    update();
    r.sleep();
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "i2c_imu_node");

  I2cImu i2c_imu;
  ROS_INFO("[IMU] Start Ok");
  i2c_imu.spin();

  return (0);
}
// EOF
