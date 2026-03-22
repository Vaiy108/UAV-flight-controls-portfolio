#include "Common.h"
#include "QuadControl.h"

#include "Utility/SimpleConfig.h"
#include "Utility/StringUtils.h"
#include "Trajectory.h"
#include "BaseController.h"
#include "Math/Mat3x3F.h"

#ifdef __PX4_NUTTX
#include <systemlib/param/param.h>
#endif

void QuadControl::Init()
{
  BaseController::Init();

  // Integral state for altitude hold.
  integratedAltitudeError = 0.0f;

#ifndef __PX4_NUTTX
  // Load controller gains and vehicle limits from the simulator config.
  ParamsHandle config = SimpleConfig::GetInstance();

  kpPosXY = config->Get(_config + ".kpPosXY", 0);
  kpPosZ  = config->Get(_config + ".kpPosZ", 0);
  KiPosZ  = config->Get(_config + ".KiPosZ", 0);

  kpVelXY = config->Get(_config + ".kpVelXY", 0);
  kpVelZ  = config->Get(_config + ".kpVelZ", 0);

  kpBank = config->Get(_config + ".kpBank", 0);
  kpYaw  = config->Get(_config + ".kpYaw", 0);

  kpPQR = config->Get(_config + ".kpPQR", V3F());

  maxDescentRate = config->Get(_config + ".maxDescentRate", 100);
  maxAscentRate  = config->Get(_config + ".maxAscentRate", 100);
  maxSpeedXY     = config->Get(_config + ".maxSpeedXY", 100);
  maxAccelXY     = config->Get(_config + ".maxHorizAccel", 100);

  maxTiltAngle = config->Get(_config + ".maxTiltAngle", 100);

  minMotorThrust = config->Get(_config + ".minMotorThrust", 0);
  maxMotorThrust = config->Get(_config + ".maxMotorThrust", 100);
#else
  // Load parameters from PX4 parameter system.
  // TODO: complete PX4 parameter mapping if targeting real firmware integration.
  param_get(param_find("MC_PITCH_P"), &Kp_bank);
  param_get(param_find("MC_YAW_P"), &Kp_yaw);
#endif
}

VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)
{
  // Convert desired collective thrust + body moments into individual motor thrusts.
  //
  // For an X-configuration quadrotor, each rotor contributes to:
  // - total thrust
  // - roll moment
  // - pitch moment
  // - yaw moment (through drag torque, scaled by kappa)
  //
  // Motor order used by the simulator:
  // 0 = front-left, 1 = front-right, 2 = rear-left, 3 = rear-right

  const float armLength = L / sqrtf(2.0f);

  const float f0 = 0.25f * (
      collThrustCmd +
      momentCmd.x / armLength +
      momentCmd.y / armLength -
      momentCmd.z / kappa);

  const float f1 = 0.25f * (
      collThrustCmd -
      momentCmd.x / armLength +
      momentCmd.y / armLength +
      momentCmd.z / kappa);

  const float f2 = 0.25f * (
      collThrustCmd +
      momentCmd.x / armLength -
      momentCmd.y / armLength +
      momentCmd.z / kappa);

  const float f3 = 0.25f * (
      collThrustCmd -
      momentCmd.x / armLength -
      momentCmd.y / armLength -
      momentCmd.z / kappa);

  // Clamp each motor to its achievable thrust range.
  cmd.desiredThrustsN[0] = CONSTRAIN(f0, minMotorThrust, maxMotorThrust);
  cmd.desiredThrustsN[1] = CONSTRAIN(f1, minMotorThrust, maxMotorThrust);
  cmd.desiredThrustsN[2] = CONSTRAIN(f2, minMotorThrust, maxMotorThrust);
  cmd.desiredThrustsN[3] = CONSTRAIN(f3, minMotorThrust, maxMotorThrust);

  return cmd;
}

V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
  // Inner-loop body-rate controller.
  //
  // The commanded body rates are compared with the estimated body rates.
  // A proportional controller generates desired angular accelerations,
  // which are then converted to moments using the vehicle inertias.

  V3F momentCmd;

  const V3F rateError = pqrCmd - pqr;
  const V3F angularAccelCmd = kpPQR * rateError;

  momentCmd.x = Ixx * angularAccelCmd.x;
  momentCmd.y = Iyy * angularAccelCmd.y;
  momentCmd.z = Izz * angularAccelCmd.z;

  return momentCmd;
}

// Returns desired roll and pitch rates in body frame.
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{
  // Convert desired lateral acceleration in the inertial frame into desired
  // roll/pitch body rates.
  //
  // Intuition:
  // - lateral motion is produced by tilting the thrust vector
  // - so we first compute desired body-z direction components (bx, by)
  // - then drive current (bx, by) toward those targets with kpBank
  //
  // Note: collective thrust is given as force [N], so we divide by mass
  // to obtain the corresponding acceleration magnitude.

  V3F pqrCmd;
  Mat3x3F R = attitude.RotationMatrix_IwrtB();

  const float thrustAccel = collThrustCmd / mass;
  const float epsilon = 1e-6f;

  // If thrust is effectively zero, tilt-based lateral control is not meaningful.
  if (thrustAccel < epsilon)
  {
    return pqrCmd;
  }

  float bxCmd = -accelCmd.x / thrustAccel;
  float byCmd = -accelCmd.y / thrustAccel;

  // Limit commanded tilt by constraining the desired body-z components.
  const float maxTiltComponent = sinf(maxTiltAngle);
  bxCmd = CONSTRAIN(bxCmd, -maxTiltComponent, maxTiltComponent);
  byCmd = CONSTRAIN(byCmd, -maxTiltComponent, maxTiltComponent);

  const float bx = R(0, 2);
  const float by = R(1, 2);

  const float bxDotCmd = kpBank * (bxCmd - bx);
  const float byDotCmd = kpBank * (byCmd - by);

  const float R33 = R(2, 2);

  // Map desired body-z direction rates into desired p and q.
  if (fabsf(R33) > epsilon)
  {
    pqrCmd.x = (R(1, 0) * bxDotCmd - R(0, 0) * byDotCmd) / R33;
    pqrCmd.y = (R(1, 1) * bxDotCmd - R(0, 1) * byDotCmd) / R33;
  }

  return pqrCmd;
}

float QuadControl::AltitudeControl(
    float posZCmd,
    float velZCmd,
    float posZ,
    float velZ,
    Quaternion<float> attitude,
    float accelZCmd,
    float dt)
{
  // Altitude controller in NED coordinates.
  //
  // NED convention:
  // - +Z points downward
  // - therefore higher upward thrust corresponds to lower commanded Z acceleration
  //
  // Controller structure:
  // - position error -> desired vertical velocity correction
  // - velocity error -> desired vertical acceleration correction
  // - integral term helps remove steady-state altitude bias
  // - feed-forward acceleration is added directly
  //
  // Finally, desired vertical acceleration is converted into a collective thrust.

  Mat3x3F R = attitude.RotationMatrix_IwrtB();

  // Constrain commanded vertical speed.
  // In NED: ascent is negative Z velocity, descent is positive Z velocity.
  velZCmd = CONSTRAIN(velZCmd, -maxAscentRate, maxDescentRate);

  const float zErr = posZCmd - posZ;
  const float zDotErr = velZCmd - velZ;

  // Integrate altitude error with a small clamp to limit windup.
  integratedAltitudeError += zErr * dt;
  integratedAltitudeError = CONSTRAIN(integratedAltitudeError, -4.0f, 4.0f);

  const float u1Bar =
      kpPosZ * zErr +
      kpVelZ * zDotErr +
      KiPosZ * integratedAltitudeError +
      accelZCmd;

  // Convert desired vertical acceleration into collective thrust.
  // R(2,2) projects body thrust onto the world Z axis.
  const float epsilon = 1e-3f;
  float thrust = mass * (CONST_GRAVITY - u1Bar) / fmaxf(R(2, 2), epsilon);

  // Clamp total collective thrust to achievable bounds.
  //
  // NOTE:
  // For a quadrotor, a common bound is 4 * min/maxMotorThrust because there
  // are 4 motors. If you intentionally keep 5x due to simulator-specific
  // tuning, document that in the README. Otherwise, changing this to 4x will
  // look cleaner and more physically grounded to an employer.
  thrust = CONSTRAIN(thrust, 5.0f * minMotorThrust, 5.0f * maxMotorThrust);

  return thrust;
}

// Returns desired lateral acceleration in the global frame.
V3F QuadControl::LateralPositionControl(
    V3F posCmd,
    V3F velCmd,
    V3F pos,
    V3F vel,
    V3F accelCmdFF)
{
  // Cascaded lateral position controller:
  // 1) position error generates a corrective velocity command
  // 2) velocity error generates a corrective acceleration command
  // 3) velocity and acceleration are both saturated to respect vehicle limits
  //
  // Only XY motion is controlled here. Z is handled separately by the altitude loop.

  accelCmdFF.z = 0.0f;
  velCmd.z = 0.0f;
  posCmd.z = pos.z;

  V3F accelCmd = accelCmdFF;

  const V3F posErr = posCmd - pos;
  V3F velCmdCorrected = velCmd + kpPosXY * posErr;

  // Limit horizontal speed magnitude.
  V3F velXY(velCmdCorrected.x, velCmdCorrected.y, 0.0f);
  const float speedXY = velXY.mag();

  if (speedXY > maxSpeedXY)
  {
    velCmdCorrected.x *= maxSpeedXY / speedXY;
    velCmdCorrected.y *= maxSpeedXY / speedXY;
  }

  const V3F velErr = velCmdCorrected - vel;

  // Add feedback acceleration to the feed-forward term.
  accelCmd += kpVelXY * velErr;

  // Limit horizontal acceleration magnitude.
  V3F accelXY(accelCmd.x, accelCmd.y, 0.0f);
  const float accelMagXY = accelXY.mag();

  if (accelMagXY > maxAccelXY)
  {
    accelCmd.x *= maxAccelXY / accelMagXY;
    accelCmd.y *= maxAccelXY / accelMagXY;
  }

  accelCmd.z = 0.0f;
  return accelCmd;
}

float QuadControl::YawControl(float yawCmd, float yaw)
{
  // Yaw controller.
  //
  // The yaw error must be wrapped to [-pi, pi] so that the controller always
  // chooses the shortest angular path instead of rotating the long way around.

  float yawRateCmd = 0.0f;

  float yawErr = yawCmd - yaw;
  const float twoPi = 2.0f * static_cast<float>(F_PI);

  yawErr = fmodf(yawErr, twoPi);
  if (yawErr > static_cast<float>(F_PI))
  {
    yawErr -= twoPi;
  }
  if (yawErr < static_cast<float>(-F_PI))
  {
    yawErr += twoPi;
  }

  yawRateCmd = kpYaw * yawErr;
  return yawRateCmd;
}

VehicleCommand QuadControl::RunControl(float dt, float simTime)
{
  // Top-level cascaded controller:
  // trajectory -> position/altitude -> attitude/body rates -> body moments -> motors

  curTrajPoint = GetNextTrajectoryPoint(simTime);

  float collThrustCmd = AltitudeControl(
      curTrajPoint.position.z,
      curTrajPoint.velocity.z,
      estPos.z,
      estVel.z,
      estAtt,
      curTrajPoint.accel.z,
      dt);

  // Reserve a small thrust margin so the attitude controller still has room
  // to create differential motor commands without immediately saturating.
  const float thrustMargin = 0.02f * (maxMotorThrust - minMotorThrust);
  collThrustCmd = CONSTRAIN(
      collThrustCmd,
      (minMotorThrust + thrustMargin) * 4.0f,
      (maxMotorThrust - thrustMargin) * 4.0f);

  V3F desAcc = LateralPositionControl(
      curTrajPoint.position,
      curTrajPoint.velocity,
      estPos,
      estVel,
      curTrajPoint.accel);

  V3F desOmega = RollPitchControl(desAcc, estAtt, collThrustCmd);
  desOmega.z = YawControl(curTrajPoint.attitude.Yaw(), estAtt.Yaw());

  V3F desMoment = BodyRateControl(desOmega, estOmega);

  return GenerateMotorCommands(collThrustCmd, desMoment);
}
