#include <gtest/gtest.h>
#include <thread>

#include "gz/math/Angle.hh"
#include "gz/math/Helpers.hh"
#include "gz/math/FourwidsDriveOdometry.hh"

using namespace gz;

TEST(FourwidsDriveOdometryTest, BasicInit)
{
  math::FourwidsDriveOdometry odom;
  EXPECT_DOUBLE_EQ(0.0, *odom.Heading());
  EXPECT_DOUBLE_EQ(0.0, odom.X());
  EXPECT_DOUBLE_EQ(0.0, odom.Y());
  EXPECT_DOUBLE_EQ(0.0, odom.LinearVelocity());
  EXPECT_DOUBLE_EQ(0.0, odom.LateralVelocity());
  EXPECT_DOUBLE_EQ(0.0, *odom.AngularVelocity());
  EXPECT_FALSE(odom.Initialized());

  double wheelSeparation = 0.5;
  double wheelBase = 0.5;
  double wheelRadius = 0.12;

  // Setup the wheel parameters, and initialize
  odom.SetWheelParams(wheelSeparation, wheelBase, wheelRadius);
  auto startTime = std::chrono::steady_clock::now();
  odom.Init(startTime);
  EXPECT_TRUE(odom.Initialized());

  // TODO: 这边数值还没有确定，最好需要xyheading都有值，线速度角速度都有值
  auto time1 = startTime + std::chrono::milliseconds(100);
  odom.Update(GZ_DTOR(1.0), GZ_DTOR(1.0), GZ_DTOR(1.0), GZ_DTOR(1.0), 
              GZ_DTOR(0.0), GZ_DTOR(0.0), GZ_DTOR(0.0), GZ_DTOR(0.0),time1);
  
  // Initialize again, and odom values should be reset.
  startTime = std::chrono::steady_clock::now();
  odom.Init(startTime);
  EXPECT_DOUBLE_EQ(0.0, *odom.Heading());
  EXPECT_DOUBLE_EQ(0.0, odom.X());
  EXPECT_DOUBLE_EQ(0.0, odom.Y());
  EXPECT_DOUBLE_EQ(0.0, odom.LinearVelocity());
  EXPECT_DOUBLE_EQ(0.0, *odom.AngularVelocity());
}

TEST(FourwidsDriveOdometryTest, StraightForward)
{
  math::FourwidsDriveOdometry odom;

  double wheelSeparation = 0.5;
  double wheelBase = 0.5;
  double wheelRadius = 0.12;
  double wheelCircumference = 2.0 * GZ_PI * wheelRadius;
  
  // This is the linear distance traveled per degree of wheel rotation.
  double distPerDegree = wheelCircumference / 360.0;

  // Setup the wheel parameters, and initialize
  odom.SetWheelParams(wheelSeparation, wheelBase, wheelRadius);
  auto startTime = std::chrono::steady_clock::now();
  odom.Init(startTime);  

  // Sleep for a little while, then update the odometry with the new wheel
  // position.
  auto time1 = startTime + std::chrono::milliseconds(100);
  odom.Update(GZ_DTOR(1.0), GZ_DTOR(1.0), GZ_DTOR(1.0), GZ_DTOR(1.0), 
              GZ_DTOR(0.0), GZ_DTOR(0.0), GZ_DTOR(0.0), GZ_DTOR(0.0),time1);
  EXPECT_NEAR(distPerDegree, odom.X(), 1e-3); 
  EXPECT_NEAR(0.0, odom.Y(), 1e-3);
  EXPECT_NEAR(0.0, *odom.Heading(), 1e-3);
  // Linear velocity should be dist_traveled / time_elapsed.
  EXPECT_NEAR(distPerDegree / 0.1, odom.LinearVelocity(), 1e-3);
  EXPECT_NEAR(0.0, odom.LateralVelocity(), 1e-3);
  // Angular velocity should be zero since the "robot" is traveling in a
  // straight line.
  EXPECT_NEAR(0.0, *odom.AngularVelocity(), 1e-3);
}

TEST(FourwidsDriveOdometryTest, StraightSide)
{
  math::FourwidsDriveOdometry odom;

  double wheelSeparation = 0.5;
  double wheelBase = 0.5;
  double wheelRadius = 0.12;
  double wheelCircumference = 2.0 * GZ_PI * wheelRadius;
  
  // This is the linear distance traveled per degree of wheel rotation.
  double distPerDegree = wheelCircumference / 360.0;

  // Setup the wheel parameters, and initialize
  odom.SetWheelParams(wheelSeparation, wheelBase, wheelRadius);
  auto startTime = std::chrono::steady_clock::now();
  odom.Init(startTime);  

  // Sleep for a little while, then update the odometry with the new wheel
  // position.
  auto time1 = startTime + std::chrono::milliseconds(100);
  odom.Update(GZ_DTOR(1.0), GZ_DTOR(1.0), GZ_DTOR(1.0), GZ_DTOR(1.0), 
              GZ_DTOR(90.0), GZ_DTOR(90.0), GZ_DTOR(90.0), GZ_DTOR(90.0),time1);
  EXPECT_NEAR(0.0, odom.X(), 1e-3); 
  EXPECT_NEAR(distPerDegree, odom.Y(), 1e-3);
  EXPECT_NEAR(0.0, *odom.Heading(), 1e-3);
  EXPECT_NEAR(0.0, odom.LinearVelocity(), 1e-3);
  // Lateral velocity should be dist_traveled / time_elapsed.
  EXPECT_NEAR(distPerDegree / 0.1, odom.LateralVelocity(), 1e-3);
  // Angular velocity should be zero since the "robot" is traveling in a
  // straight line.
  EXPECT_NEAR(0.0, *odom.AngularVelocity(), 1e-3);
}

TEST(FourwidsDriveOdometryTest, StraightDiagonal)
{
  math::FourwidsDriveOdometry odom;

  double wheelSeparation = 0.5;
  double wheelBase = 0.5;
  double wheelRadius = 0.12;
  double wheelCircumference = 2.0 * GZ_PI * wheelRadius;
  
  // This is the linear distance traveled per degree of wheel rotation.
  double distPerDegree = wheelCircumference / 360.0;

  // Setup the wheel parameters, and initialize
  odom.SetWheelParams(wheelSeparation, wheelBase, wheelRadius);
  auto startTime = std::chrono::steady_clock::now();
  odom.Init(startTime);  

  // Sleep for a little while, then update the odometry with the new wheel
  // position.
  auto time1 = startTime + std::chrono::milliseconds(100);
  odom.Update(GZ_DTOR(1.0), GZ_DTOR(1.0), GZ_DTOR(1.0), GZ_DTOR(1.0), 
              GZ_DTOR(45.0), GZ_DTOR(45.0), GZ_DTOR(45.0), GZ_DTOR(45.0),time1);
  EXPECT_NEAR(distPerDegree / sqrt(2), odom.X(), 1e-3); 
  EXPECT_NEAR(distPerDegree / sqrt(2), odom.Y(), 1e-3);
  EXPECT_NEAR(0.0, *odom.Heading(), 1e-3);
  EXPECT_NEAR(distPerDegree / sqrt(2) / 0.1, odom.LinearVelocity(), 1e-3);
  EXPECT_NEAR(distPerDegree / sqrt(2) / 0.1, odom.LateralVelocity(), 1e-3);
  // Angular velocity should be zero since the "robot" is traveling in a
  // straight line.
  EXPECT_NEAR(0.0, *odom.AngularVelocity(), 1e-3);
}

TEST(FourwidsDriveOdometryTest, RotateInPlace)
{
  math::FourwidsDriveOdometry odom;

  double wheelSeparation = 0.5;
  double wheelBase = 0.5;
  double wheelRadius = 0.12;
  double wheelCircumference = 2.0 * GZ_PI * wheelRadius;
  
  // The distance between the wheel axle and the center of the vehicle
  double wheelToCenter = sqrt((wheelSeparation * wheelSeparation) / 4 + (wheelBase * wheelBase) / 4);

  // This is the linear distance traveled per degree of wheel rotation.
  double distPerDegree = wheelCircumference / 360.0;

  // Setup the wheel parameters, and initialize
  odom.SetWheelParams(wheelSeparation, wheelBase, wheelRadius);
  auto startTime = std::chrono::steady_clock::now();
  odom.Init(startTime);  

  // Sleep for a little while, then update the odometry with the new wheel
  // position.
  auto time1 = startTime + std::chrono::milliseconds(100);
  odom.Update(GZ_DTOR(1.0), GZ_DTOR(-1.0), GZ_DTOR(1.0), GZ_DTOR(-1.0), 
              GZ_DTOR(-45.0), GZ_DTOR(45.0), GZ_DTOR(45.0), GZ_DTOR(-45.0),time1);
  EXPECT_NEAR(0.0, odom.X(), 1e-3); 
  EXPECT_NEAR(0.0, odom.Y(), 1e-3);
  EXPECT_NEAR(-distPerDegree / wheelToCenter, *odom.Heading(), 1e-3);
  // Linear velocity should be zero since the "robot" is rotating in place.
  EXPECT_NEAR(0.0, odom.LinearVelocity(), 1e-3);
  EXPECT_NEAR(0.0, odom.LateralVelocity(), 1e-3);
  // Angular velocity should be angular_traveled / time_elapsed.
  EXPECT_NEAR(-distPerDegree / wheelToCenter / 0.1, *odom.AngularVelocity(), 1e-3);
}

// NOTE: 这个算法是有前提的，也就是输入的角度是需要连续的，否则和实际运行效果会有差异。同时控制的时间间隔要尽可能小