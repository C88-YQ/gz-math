#include <cmath>
#include "gz/math/FourwidsDriveOdometry.hh"
#include "gz/math/RollingMean.hh"

#include <Eigen/Dense>
#include <iostream>

namespace gz
{
namespace math
{
inline namespace GZ_MATH_VERSION_NAMESPACE
{
class FourwidsDriveOdometry::Implementation
{
  /// \brief Constructor.
  /// \param[in] _windowSize Rolling window size used to compute the
  /// velocity mean.
  public: explicit Implementation(size_t _windowSize)
    : linearMean(_windowSize), lateralMean(_windowSize),
      angularMean(_windowSize)
  {
  }

  /// \brief Integrates the pose.
  /// \param[in] _linear Linear velocity.
  /// \param[in] _lateral Lateral velocity.
  /// \param[in] _angular Angular velocity.
  public: void IntegrateExact(double _linear, double _lateral, double _angular);

  /// \brief Integrates the pose using second order Runge-Kuffa approximation.
  /// \param[in] _linear Linear velocity.
  /// \param[in] _lateral Lateral velocity.
  /// \param[in] _angular Angular velocity.
  public: void IntegrateRungeKutta2(double _linear, double _lateral,
    double _angular);

  /// \brief Current timestamp.
  public: FourwidsDriveOdometry::clock::time_point lastUpdateTime;

  /// \brief Current x position in meters.
  public: double x{0.0};

  /// \brief Current y position in meters.
  public: double y{0.0};

  /// \brief Current heading in radians.
  public: gz::math::Angle heading;

  /// \brief Current linear velocity in meter/second.
  public: double linearVel{0.0};

  /// \brief Current lateral velocity in meter/second.
  public: double lateralVel{0.0};

  /// \brief Current angular velocity in radians/second.
  public: gz::math::Angle angularVel;

  /// \brief Wheel radius in meters.
  public: double wheelRadius{0.0};

  /// \brief Wheel separation in meters.
  public: double wheelSeparation{1.0};

  /// \brief Wheel base in meters.
  public: double wheelBase{1.0};

  /// \brief Previous frontleft wheel position/state in radians.
  public: double frontLeftWheelOldPos{0.0};

  /// \brief Previous frontright wheel position/state in radians.
  public: double frontRightWheelOldPos{0.0};

  /// \brief Previous backleft wheel position/state in radians.
  public: double backLeftWheelOldPos{0.0};

  /// \brief Previous backright wheel position/state in radians.
  public: double backRightWheelOldPos{0.0};

  /// \brief Previous frontleft steering position/state in radians.
  public: double frontLeftSteeringOldPos{0.0};

  /// \brief Previous frontright steering position/state in radians.
  public: double frontRightSteeringOldPos{0.0};

  /// \brief Previous backleft steering position/state in radians.
  public: double backLeftSteeringOldPos{0.0};

  /// \brief Previous backright steering position/state in radians.
  public: double backRightSteeringOldPos{0.0};

  /// \brief Rolling mean accumulators for the linear velocity
  public: gz::math::RollingMean linearMean;

  /// \brief Rolling mean accumulators for the lateral velocity
  public: gz::math::RollingMean lateralMean;

  /// \brief Rolling mean accumulators for the angular velocity
  public: gz::math::RollingMean angularMean;

  /// \brief Initialized flag.
  public: bool initialized{false};
};
} // namespace GZ_MATH_VERSION_NAMESPACE
} // namespace math
} // namespace gz

using namespace gz;
using namespace math;

//////////////////////////////////////////////////
FourwidsDriveOdometry::FourwidsDriveOdometry(size_t _windowSize)
  : dataPtr(gz::utils::MakeImpl<Implementation>(_windowSize))
{
}

void FourwidsDriveOdometry::Init(const clock::time_point &_time)
{
  // Reset accumulators and timestamp.
  this->dataPtr->linearMean.Clear();
  this->dataPtr->lateralMean.Clear();
  this->dataPtr->angularMean.Clear();
  this->dataPtr->x = 0.0;
  this->dataPtr->y = 0.0;
  this->dataPtr->heading = 0.0;
  this->dataPtr->linearVel = 0.0;
  this->dataPtr->lateralVel = 0.0;
  this->dataPtr->angularVel = 0.0;
  this->dataPtr->frontLeftWheelOldPos = 0.0;
  this->dataPtr->frontRightWheelOldPos = 0.0;
  this->dataPtr->backLeftWheelOldPos = 0.0;
  this->dataPtr->backRightWheelOldPos = 0.0;
  this->dataPtr->frontLeftSteeringOldPos = 0.0;
  this->dataPtr->frontRightSteeringOldPos = 0.0;
  this->dataPtr->backLeftSteeringOldPos = 0.0;
  this->dataPtr->backRightSteeringOldPos = 0.0;

  this->dataPtr->lastUpdateTime = _time;
  this->dataPtr->initialized = true;
}

//////////////////////////////////////////////////
bool FourwidsDriveOdometry::Initialized() const
{
  return this->dataPtr->initialized;
}

//////////////////////////////////////////////////
bool FourwidsDriveOdometry::Update(const Angle &_frontLeftPos,
  const Angle &_frontRightPos, const Angle &_backLeftPos,
  const Angle &_backRightPos, const Angle &_frontLeftSteeringPos,
  const Angle &_frontRightSteeringPos, const Angle &_backLeftSteeringPos,
  const Angle &_backRightSteeringPos, const clock::time_point &_time)
{
  if (!this->dataPtr->initialized)
  {
    return false;
  }

  // TODO: 最小二乘求解合速度

  // Compute x, y and heading using velocity
  const std::chrono::duration<double> dt =
    _time - this->dataPtr->lastUpdateTime;

  // We cannot estimate the speed if the time interval is zero (or near
  // zero).
  if (equal(0.0, dt.count()))
    return false;
  
  // Get current wheel joint positions:
  const double frontLeftWheelCurPos =
    *_frontLeftPos * this->dataPtr->wheelRadius;
  const double frontRightWheelCurPos =
    *_frontRightPos * this->dataPtr->wheelRadius;
  const double backLeftWheelCurPos =
    *_backLeftPos * this->dataPtr->wheelRadius;
  const double backRightWheelCurPos =
    *_backRightPos * this->dataPtr->wheelRadius;
  
  // TODO: 对于过零问题，要怎么处理？？？
  // Estimate velocity of wheels using old and current position:
  const double frontLeftWheelEstVel = frontLeftWheelCurPos -
                                 this->dataPtr->frontLeftWheelOldPos;

  const double frontRightWheelEstVel = frontRightWheelCurPos -
                                  this->dataPtr->frontRightWheelOldPos;

  const double backLeftWheelEstVel = backLeftWheelCurPos -
                                 this->dataPtr->backLeftWheelOldPos;

  const double backRightWheelEstVel = backRightWheelCurPos -
                                  this->dataPtr->backRightWheelOldPos;
              
  // Update old position with current
  this->dataPtr->frontLeftWheelOldPos = frontLeftWheelCurPos;
  this->dataPtr->frontRightWheelOldPos = frontRightWheelCurPos;
  this->dataPtr->backLeftWheelOldPos = backLeftWheelCurPos;
  this->dataPtr->backRightWheelOldPos = backRightWheelCurPos;
  
  // TODO:(注释)先计算是否四个舵的方向都差不多，如果差不多，就直接平均算速度
  Eigen::Vector2d frontLeftWheelVelVec(frontLeftWheelEstVel / dt.count() * std::cos(*_frontLeftSteeringPos), 
                                       frontLeftWheelEstVel / dt.count() * std::sin(*_frontLeftSteeringPos));
  Eigen::Vector2d frontRightWheelVelVec(frontRightWheelEstVel / dt.count() * std::cos(*_frontRightSteeringPos), 
                                        frontRightWheelEstVel / dt.count() * std::sin(*_frontRightSteeringPos));
  Eigen::Vector2d backLeftWheelVelVec(backLeftWheelEstVel / dt.count() * std::cos(*_backLeftSteeringPos), 
                                      backLeftWheelEstVel / dt.count() * std::sin(*_backLeftSteeringPos));
  Eigen::Vector2d backRightWheelVelVec(backRightWheelEstVel / dt.count() * std::cos(*_backRightSteeringPos), 
                                        backRightWheelEstVel / dt.count() * std::sin(*_backRightSteeringPos));

  Eigen::Vector2d meanVec = (frontLeftWheelVelVec + frontRightWheelVelVec + backLeftWheelVelVec + backRightWheelVelVec) / 4.0;
  double maxDev = std::max({(frontLeftWheelVelVec - meanVec).norm(), 
                             (frontRightWheelVelVec - meanVec).norm(),
                             (backLeftWheelVelVec - meanVec).norm(),
                             (backRightWheelVelVec - meanVec).norm()});
  double meanNorm = meanVec.norm();

  // TODO: 魔鬼数字，最好可以配置
  const double ABS_VEL_THRESHOLD = 3e-3;
  const double REL_VEL_THRESHOLD = 0.01;
  bool vectors_identical = false;
  if (maxDev < ABS_VEL_THRESHOLD)
  {
    vectors_identical = true;
  }
  else if (meanNorm > 1e-6 && (maxDev / meanNorm) < REL_VEL_THRESHOLD)
  {
    vectors_identical = true;
  }
  
  Eigen::MatrixXd A(4, 3);
  Eigen::VectorXd s(4);

  A(0,0) = std::cos(*_frontLeftSteeringPos);
  A(0,1) = std::sin(*_frontLeftSteeringPos);
  A(0,2) = std::sin(*_frontLeftSteeringPos) * (0.5 * this->dataPtr->wheelBase) - 
            std::cos(*_frontLeftSteeringPos) * (0.5 * this->dataPtr->wheelSeparation);
  A(1,0) = std::cos(*_frontRightSteeringPos);
  A(1,1) = std::sin(*_frontRightSteeringPos);
  A(1,2) = std::sin(*_frontRightSteeringPos) * (0.5 * this->dataPtr->wheelBase) - 
            std::cos(*_frontRightSteeringPos) * (-0.5 * this->dataPtr->wheelSeparation);
  A(2,0) = std::cos(*_backLeftSteeringPos);
  A(2,1) = std::sin(*_backLeftSteeringPos);
  A(2,2) = std::sin(*_backLeftSteeringPos) * (-0.5 * this->dataPtr->wheelBase) - 
            std::cos(*_backLeftSteeringPos) * (0.5 * this->dataPtr->wheelSeparation);
  A(3,0) = std::cos(*_backRightSteeringPos);
  A(3,1) = std::sin(*_backRightSteeringPos);
  A(3,2) = std::sin(*_backRightSteeringPos) * (-0.5 * this->dataPtr->wheelBase) - 
            std::cos(*_backRightSteeringPos) * (-0.5 * this->dataPtr->wheelSeparation);
            
  s(0) = frontLeftWheelEstVel / dt.count();
  s(1) = frontRightWheelEstVel / dt.count();
  s(2) = backLeftWheelEstVel / dt.count();
  s(3) = backRightWheelEstVel / dt.count();
  
  // TODO: 确定这个求解需要多少时间，最好要在1ms以内
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::VectorXd singularValues = svd.singularValues();
  Eigen::Vector3d v;

  int rank = (singularValues.array() > 1e-6).count();
  
  if (vectors_identical)
  {
    // 可以直接根据基本一致的平均速度获得车体速度
    v << meanVec(0), meanVec(1), 0.0;
    // TODO: 仅在调试的时候使用，后续需要删除，或者换用能只在 DEBUG 模式下输出的 log
    std::cerr << "Warning: FourwidsDriveOdometry odometry calculation, wheel vectors identical, use mean velocity directly." << std::endl;
  }
  else if (rank < 3)
  {
    // 如果无解，先退化为 w=0，去求解
    Eigen::MatrixXd ADegrade = A.leftCols(2);
    Eigen::JacobiSVD<Eigen::MatrixXd> svdDegrade(ADegrade, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd singularValuesDegrade = svdDegrade.singularValues();
    int rankDegrade = (singularValuesDegrade.array() > 1e-6).count();

    std::cerr << "Warning: FourwidsDriveOdometry odometry calculation matrix rank deficient, rank[" 
              << rank << "] degrade to rank[" << rankDegrade << "]" << std::endl;
    if (rankDegrade >= 2)
    {
      Eigen::Vector2d vDegrade = svdDegrade.solve(s);
      v << vDegrade(0), vDegrade(1), 0.0;
    }
    else
    {
      // TODO: 需要log，正则化为最小范数解，确定一下这边的lambda是什么意思，设置什么值合适，是否需要能够配置
      double lambda = 1e-3;
      Eigen::MatrixXd I = Eigen::MatrixXd::Identity(3, 3);
      v = (A.transpose() * A + lambda * I).ldlt().solve(A.transpose() * s);
    }
  }
  else
  {
    v = svd.solve(s);
  }

  // Integrate the pose
  this->dataPtr->IntegrateExact(v(0) * dt.count(), v(1) * dt.count(), v(2) * dt.count());

  this->dataPtr->lastUpdateTime = _time;

  // Estimate speeds using a rolling mean to filter them out:
  this->dataPtr->linearMean.Push(v(0));
  this->dataPtr->lateralMean.Push(v(1));
  this->dataPtr->angularMean.Push(v(2));

  this->dataPtr->linearVel = this->dataPtr->linearMean.Mean();
  this->dataPtr->lateralVel = this->dataPtr->lateralMean.Mean();
  this->dataPtr->angularVel = this->dataPtr->angularMean.Mean();

  return true;
}

//////////////////////////////////////////////////
void FourwidsDriveOdometry::SetWheelParams(double _wheelSeparation,
  double _wheelBase, double _wheelRadius)
{
  this->dataPtr->wheelSeparation = _wheelSeparation;
  this->dataPtr->wheelBase = _wheelBase;
  this->dataPtr->wheelRadius = _wheelRadius;
}

//////////////////////////////////////////////////
void FourwidsDriveOdometry::SetVelocityRollingWindowSize(size_t _size)
{
  this->dataPtr->linearMean.SetWindowSize(_size);
  this->dataPtr->lateralMean.SetWindowSize(_size);
  this->dataPtr->angularMean.SetWindowSize(_size);
}

//////////////////////////////////////////////////
const Angle &FourwidsDriveOdometry::Heading() const
{
  return this->dataPtr->heading;
}

//////////////////////////////////////////////////
double FourwidsDriveOdometry::X() const
{
  return this->dataPtr->x;
}

//////////////////////////////////////////////////
double FourwidsDriveOdometry::Y() const
{
  return this->dataPtr->y;
}

//////////////////////////////////////////////////
double FourwidsDriveOdometry::LinearVelocity() const
{
  return this->dataPtr->linearVel;
}

//////////////////////////////////////////////////
double FourwidsDriveOdometry::LateralVelocity() const
{
  return this->dataPtr->lateralVel;
}

//////////////////////////////////////////////////
const Angle &FourwidsDriveOdometry::AngularVelocity() const
{
  return this->dataPtr->angularVel;
}

//////////////////////////////////////////////////
double FourwidsDriveOdometry::WheelSeparation() const
{
  return this->dataPtr->wheelSeparation;
}

//////////////////////////////////////////////////
double FourwidsDriveOdometry::WheelBase() const
{
  return this->dataPtr->wheelBase;
}

//////////////////////////////////////////////////
double FourwidsDriveOdometry::WheelRadius() const
{
  return this->dataPtr->wheelRadius;
}

//////////////////////////////////////////////////
void FourwidsDriveOdometry::Implementation::IntegrateRungeKutta2(
    double _linear, double _lateral, double _angular)
{
  const double direction = *this->heading + _angular * 0.5;

  // Runge-Kutta 2nd order integration:
  this->x += (_linear * std::cos(direction)) - (_lateral * std::sin(direction));
  this->y += _linear * std::sin(direction) + (_lateral * std::cos(direction));
  this->heading += _angular;
}

//////////////////////////////////////////////////
void FourwidsDriveOdometry::Implementation::IntegrateExact(double _linear,
  double _lateral, double _angular)
{
  if (std::fabs(_angular) < 1e-6)
  {
    this->IntegrateRungeKutta2(_linear, _lateral, _angular);
  }
  else
  {
    // 这个计算方法默认了在一个控制周期内的速度和角速度是恒定的，只要控制周期足够小，这个假设是成立的
    // Exact integration (should solve problems when angular is zero):
    const double headingOld = *this->heading;
    const double ratio = _linear / _angular;
    const double ratio2 = _lateral / _angular;
    this->heading += _angular;
    this->x += (ratio * (std::sin(*this->heading) - std::sin(headingOld)))
      - (-ratio2 * (std::cos(*this->heading) - std::cos(headingOld)));
    this->y += (-ratio * (std::cos(*this->heading) - std::cos(headingOld)))
      + (ratio2 * (std::sin(*this->heading) - std::sin(headingOld)));
  }
}

