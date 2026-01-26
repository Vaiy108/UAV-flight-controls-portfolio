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

  // variables needed for integral control
  integratedAltitudeError = 0;
    
#ifndef __PX4_NUTTX
  // Load params from simulator parameter system
  ParamsHandle config = SimpleConfig::GetInstance();
   
  // Load parameters (default to 0)
  kpPosXY = config->Get(_config+".kpPosXY", 0);
  kpPosZ = config->Get(_config + ".kpPosZ", 0);
  KiPosZ = config->Get(_config + ".KiPosZ", 0);
     
  kpVelXY = config->Get(_config + ".kpVelXY", 0);
  kpVelZ = config->Get(_config + ".kpVelZ", 0);

  kpBank = config->Get(_config + ".kpBank", 0);
  kpYaw = config->Get(_config + ".kpYaw", 0);

  kpPQR = config->Get(_config + ".kpPQR", V3F());

  maxDescentRate = config->Get(_config + ".maxDescentRate", 100);
  maxAscentRate = config->Get(_config + ".maxAscentRate", 100);
  maxSpeedXY = config->Get(_config + ".maxSpeedXY", 100);
  maxAccelXY = config->Get(_config + ".maxHorizAccel", 100);

  maxTiltAngle = config->Get(_config + ".maxTiltAngle", 100);

  minMotorThrust = config->Get(_config + ".minMotorThrust", 0);
  maxMotorThrust = config->Get(_config + ".maxMotorThrust", 100);
#else
  // load params from PX4 parameter system
  //TODO
  param_get(param_find("MC_PITCH_P"), &Kp_bank);
  param_get(param_find("MC_YAW_P"), &Kp_yaw);
#endif
}

VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)
{
  // Convert a desired 3-axis moment and collective thrust command to 
  //   individual motor thrust commands
  // INPUTS: 
  //   collThrustCmd: desired collective thrust [N]
  //   momentCmd: desired rotation moment about each axis [N m]
  // OUTPUT:
  //   set class member variable cmd (class variable for graphing) where
  //   cmd.desiredThrustsN[0..3]: motor commands, in [N]

  // HINTS: 
  // - you can access parts of momentCmd via e.g. momentCmd.x
  // You'll need the arm length parameter L, and the drag/thrust ratio kappa

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

    const float l = L / sqrtf(2.f);

    // Motor order: 0=front-left, 1=front-right, 2=rear-left, 3=rear-right
    

    float f0 = 0.25f * (collThrustCmd + momentCmd.x / l + momentCmd.y / l - momentCmd.z / kappa);
    float f1 = 0.25f * (collThrustCmd - momentCmd.x / l + momentCmd.y / l + momentCmd.z / kappa);
    float f2 = 0.25f * (collThrustCmd + momentCmd.x / l - momentCmd.y / l + momentCmd.z / kappa);
    float f3 = 0.25f * (collThrustCmd - momentCmd.x / l - momentCmd.y / l - momentCmd.z / kappa);

    cmd.desiredThrustsN[0] = fmaxf(minMotorThrust, fminf(f0, maxMotorThrust));
    cmd.desiredThrustsN[1] = fmaxf(minMotorThrust, fminf(f1, maxMotorThrust));
    cmd.desiredThrustsN[2] = fmaxf(minMotorThrust, fminf(f2, maxMotorThrust));
    cmd.desiredThrustsN[3] = fmaxf(minMotorThrust, fminf(f3, maxMotorThrust));

    


  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return cmd;
}

V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
  // Calculate a desired 3-axis moment given a desired and current body rate
  // INPUTS: 
  //   pqrCmd: desired body rates [rad/s]
  //   pqr: current or estimated body rates [rad/s]
  // OUTPUT:
  //   return a V3F containing the desired moments for each of the 3 axes

  // HINTS: 
  //  - you can use V3Fs just like scalars: V3F a(1,1,1), b(2,3,4), c; c=a-b;
  //  - you'll need parameters for moments of inertia Ixx, Iyy, Izz
  //  - you'll also need the gain parameter kpPQR (it's a V3F)

  V3F momentCmd;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

    // Rate error
  V3F rateError = pqrCmd - pqr;

  // Element-wise proportional term
  V3F uBar = kpPQR * rateError;   // kpPQR is a V3F (Kp_p, Kp_q, Kp_r)

  // Convert desired angular accelerations -> moments using moments of inertia
  momentCmd.x = Ixx * uBar.x;
  momentCmd.y = Iyy * uBar.y;
  momentCmd.z = Izz * uBar.z;

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return momentCmd;
}

// returns a desired roll and pitch rate 
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{
  // Calculate a desired pitch and roll angle rates based on a desired global
  //   lateral acceleration, the current attitude of the quad, and desired
  //   collective thrust command
  // INPUTS: 
  //   accelCmd: desired acceleration in global XY coordinates [m/s2]
  //   attitude: current or estimated attitude of the vehicle
  //   collThrustCmd: desired collective thrust of the quad [N]
  // OUTPUT:
  //   return a V3F containing the desired pitch and roll rates. The Z
  //     element of the V3F should be left at its default value (0)

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the roll/pitch gain kpBank
  //  - collThrustCmd is a force in Newtons! You'll likely want to convert it to acceleration first

  V3F pqrCmd;
  Mat3x3F R = attitude.RotationMatrix_IwrtB();

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  //if (collThrustCmd >= 1e-6f) return pqrCmd;

  //// acceleration magnitude from thrust
  float c = collThrustCmd / mass;
  if (c < 1e-6f) return pqrCmd;


  // Use thrust magnitude to avoid sign-convention issues
  //float c = fabsf(collThrustCmd) / mass;   // positive [m/s^2]
  //if (c < 1e-6f) return pqrCmd;

  float b_x_c = -accelCmd.x / c;
  float b_y_c = -accelCmd.y / c;

  // limit tilt
  float maxB = sinf(maxTiltAngle);
  b_x_c = fmaxf(-maxB, fminf(b_x_c, maxB));
  b_y_c = fmaxf(-maxB, fminf(b_y_c, maxB));

  float b_x = R(0, 2);
  float b_y = R(1, 2);

  float b_x_dot_c = kpBank * (b_x_c - b_x);
  float b_y_dot_c = kpBank * (b_y_c - b_y);

  float R33 = R(2, 2);
  if (fabsf(R33) > 1e-6f) {
      pqrCmd.x = (R(1, 0) * b_x_dot_c - R(0, 0) * b_y_dot_c) / R33;
      pqrCmd.y = (R(1, 1) * b_x_dot_c - R(0, 1) * b_y_dot_c) / R33;
  }



  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return pqrCmd;
}

float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt)
{
  // Calculate desired quad thrust based on altitude setpoint, actual altitude,
  //   vertical velocity setpoint, actual vertical velocity, and a vertical 
  //   acceleration feed-forward command
  // INPUTS: 
  //   posZCmd, velZCmd: desired vertical position and velocity in NED [m]
  //   posZ, velZ: current vertical position and velocity in NED [m]
  //   accelZCmd: feed-forward vertical acceleration in NED [m/s2]
  //   dt: the time step of the measurements [seconds]
  // OUTPUT:
  //   return a collective thrust command in [N]

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the gain parameters kpPosZ and kpVelZ
  //  - maxAscentRate and maxDescentRate are maximum vertical speeds. Note they're both >=0!
  //  - make sure to return a force, not an acceleration
  //  - remember that for an upright quad in NED, thrust should be HIGHER if the desired Z acceleration is LOWER

  Mat3x3F R = attitude.RotationMatrix_IwrtB();
  // float thrust = 0;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
    // Constrain vertical velocity command (NED: +Z is down)
  velZCmd = CONSTRAIN(velZCmd, -maxAscentRate, maxDescentRate);

  // Position / velocity errors
  float zErr = posZCmd - posZ;
  float zDotErr = velZCmd - velZ;

  // Integrate altitude error
  integratedAltitudeError += zErr * dt;
  integratedAltitudeError = CONSTRAIN(integratedAltitudeError, -4.f, 4.f);

  // Desired vertical acceleration in NED
  float u1Bar = kpPosZ * zErr + kpVelZ * zDotErr + KiPosZ * integratedAltitudeError + accelZCmd;

  // Convert desired accel to thrust. For an upright vehicle, R(2,2) ~ 1
  // Thrust must counter gravity: accel down positive in NED, so (g - u1Bar)
  float thrust = mass * (static_cast<float>(CONST_GRAVITY) - u1Bar)
      / fmaxf(R(2, 2), 1e-3f);

  // Clamp to achievable total thrust
  thrust = CONSTRAIN(thrust, 5.f * minMotorThrust, 5.f * maxMotorThrust);

  /////////////////////////////// END STUDENT CODE ////////////////////////////
  
  return thrust;
}

// returns a desired acceleration in global frame
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmdFF)
{
  // Calculate a desired horizontal acceleration based on 
  //  desired lateral position/velocity/acceleration and current pose
  // INPUTS: 
  //   posCmd: desired position, in NED [m]
  //   velCmd: desired velocity, in NED [m/s]
  //   pos: current position, NED [m]
  //   vel: current velocity, NED [m/s]
  //   accelCmdFF: feed-forward acceleration, NED [m/s2]
  // OUTPUT:
  //   return a V3F with desired horizontal accelerations. 
  //     the Z component should be 0
  // HINTS: 
  //  - use the gain parameters kpPosXY and kpVelXY
  //  - make sure you limit the maximum horizontal velocity and acceleration
  //    to maxSpeedXY and maxAccelXY

  // make sure we don't have any incoming z-component
  accelCmdFF.z = 0;
  velCmd.z = 0;
  posCmd.z = pos.z;

  // we initialize the returned desired acceleration to the feed-forward value.
  // Make sure to _add_, not simply replace, the result of your controller
  // to this variable
  V3F accelCmd = accelCmdFF;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
// Position error
  V3F posErr = posCmd - pos;

  // Use position error to refine velocity command
  V3F velCmd2 = velCmd + kpPosXY * posErr;

  // Limit horizontal velocity to maxSpeedXY
  V3F velCmd2XY(velCmd2.x, velCmd2.y, 0.f);
  float speed = velCmd2XY.mag();
  if (speed > maxSpeedXY) {
      velCmd2.x *= maxSpeedXY / speed;
      velCmd2.y *= maxSpeedXY / speed;
  }
  // Velocity error
  V3F velErr = velCmd2 - vel;

  // Acceleration command (add to feed-forward)
  accelCmd += kpVelXY * velErr;

  // Limit horizontal acceleration to maxAccelXY (by magnitude)
  V3F accelXY(accelCmd.x, accelCmd.y, 0.f);
  float a = accelXY.mag();
  if (a > maxAccelXY) {
      accelCmd.x *= maxAccelXY / a;
      accelCmd.y *= maxAccelXY / a;
  }

  accelCmd.z = 0.f;

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return accelCmd;
}

// returns desired yaw rate
float QuadControl::YawControl(float yawCmd, float yaw)
{
  // Calculate a desired yaw rate to control yaw to yawCmd
  // INPUTS: 
  //   yawCmd: commanded yaw [rad]
  //   yaw: current yaw [rad]
  // OUTPUT:
  //   return a desired yaw rate [rad/s]
  // HINTS: 
  //  - use fmodf(foo,b) to unwrap a radian angle measure float foo to range [0,b]. 
  //  - use the yaw control gain parameter kpYaw

    float yawRateCmd=0;
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
	
     // Error
    float yawErr = yawCmd - yaw;
    // Yaw error (wrap to [-pi, pi])
    // Wrap to [-pi, pi]
    const float TWO_PI = 2.f * static_cast<float>(F_PI);
    yawErr = fmodf(yawErr, 2.f * F_PI);
    if (yawErr > static_cast<float>(F_PI))  yawErr -= TWO_PI;
    if (yawErr < static_cast<float>(-F_PI)) yawErr += TWO_PI;
    
    // Proportional yaw rate command
	yawRateCmd = kpYaw * yawErr;


    

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return yawRateCmd;

}

VehicleCommand QuadControl::RunControl(float dt, float simTime)
{
  curTrajPoint = GetNextTrajectoryPoint(simTime);

  float collThrustCmd = AltitudeControl(curTrajPoint.position.z, curTrajPoint.velocity.z, estPos.z, estVel.z, estAtt, curTrajPoint.accel.z, dt);

  // reserve some thrust margin for angle control
  float thrustMargin = .02f*(maxMotorThrust - minMotorThrust);
  collThrustCmd = CONSTRAIN(collThrustCmd, (minMotorThrust+ thrustMargin)*4.f, (maxMotorThrust-thrustMargin)*4.f);
  
  V3F desAcc = LateralPositionControl(curTrajPoint.position, curTrajPoint.velocity, estPos, estVel, curTrajPoint.accel);
  
  V3F desOmega = RollPitchControl(desAcc, estAtt, collThrustCmd);
  desOmega.z = YawControl(curTrajPoint.attitude.Yaw(), estAtt.Yaw());

  V3F desMoment = BodyRateControl(desOmega, estOmega);

  return GenerateMotorCommands(collThrustCmd, desMoment);
}
