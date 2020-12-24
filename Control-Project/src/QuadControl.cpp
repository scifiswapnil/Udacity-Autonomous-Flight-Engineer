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
#include <iostream>

using std::cout;

void QuadControl::Init()
{
  BaseController::Init();

  // variables needed for integral control
  integratedAltitudeError = 0;
  alt_previous_error = 0;
    
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
    
    float a = momentCmd.x/(L*(1.414213562373095));//(L*(1.414213562373095));
    float b = momentCmd.y/(L*(1.414213562373095));//(L*(1.414213562373095));
    float c = momentCmd.z/kappa;
    float d = collThrustCmd;

    cmd.desiredThrustsN[0] = ((a+b+c+d)/(4.f));
    cmd.desiredThrustsN[1] = ((-a+b-c+d)/(4.f));
    cmd.desiredThrustsN[3] = ((-a-b+c+d)/(4.f));
    cmd.desiredThrustsN[2] = ((a-b-c+d)/(4.f));

    
    cmd.desiredThrustsN[0] = CONSTRAIN(cmd.desiredThrustsN[0],minMotorThrust,maxMotorThrust);
    cmd.desiredThrustsN[1] = CONSTRAIN(cmd.desiredThrustsN[1],minMotorThrust,maxMotorThrust);
    cmd.desiredThrustsN[2] = CONSTRAIN(cmd.desiredThrustsN[2],minMotorThrust,maxMotorThrust);
    cmd.desiredThrustsN[3] = CONSTRAIN(cmd.desiredThrustsN[3],minMotorThrust,maxMotorThrust);

  return cmd;
}

float QuadControl::clip(float n, float lower, float upper) {
    return std::max(lower, std::min(n, upper));
}

V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{

    V3F momentCmd;
    V3F error = pqrCmd - pqr;
    V3F ubar = kpPQR * error;
    V3F moments = ubar * V3F(Ixx,Iyy,Izz);
    return moments;
}

// returns a desired roll and pitch rate 
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{

    V3F pqrCmd;
    Mat3x3F R = attitude.RotationMatrix_IwrtB();
    float b_x_a = R(0,2);
    float b_y_a = R(1,2);
    float R33 = R(2,2);
    float R21 = R(1,0);
    float R22 = R(1,1);
    float R12 = R(0,1);
    float R11 = R(0,0);
    float b_x_c_target = CONSTRAIN(accelCmd[0]*mass/(collThrustCmd),-maxTiltAngle, maxTiltAngle);
    float b_y_c_target = CONSTRAIN(accelCmd[1]*mass/(collThrustCmd),-maxTiltAngle, maxTiltAngle);

    if (collThrustCmd < 0)
    {
        b_x_c_target = 0;
        b_y_c_target = 0;
    }

    float b_dot_x_c = kpBank*(b_x_c_target - b_x_a);
    float b_dot_y_c = kpBank*(b_y_c_target - b_y_a);
    
    float p_c = (1/R33)*(R21*b_dot_x_c - R11*b_dot_y_c);
    float q_c = (1/R33)*(R22*b_dot_x_c - R12*b_dot_y_c);

    pqrCmd.x = p_c;
    pqrCmd.y = q_c;
    return pqrCmd;

}

float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt)
{

    Mat3x3F R = attitude.RotationMatrix_IwrtB();
    float thrust = 0;
    float b_z = R(2,2);
    float d_term = 0;

    velZCmd = -CONSTRAIN(-velZCmd,-maxDescentRate,maxAscentRate);
    float e = posZCmd - posZ;
    integratedAltitudeError += KiPosZ*e*dt;

    float u_bar_1 = kpPosZ*(posZCmd - posZ) + kpVelZ*(velZCmd - velZ) + accelZCmd + integratedAltitudeError + d_term;
    float accelZ = (u_bar_1 - 9.81f)/b_z;
    if (accelZ > 0)
    {
        accelZ = 0;
    }
    thrust = -accelZ*mass;
    return thrust;
}

// returns a desired acceleration in global frame
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmd)
{
    accelCmd.z = 0;
    velCmd.z = 0;
    posCmd.z = pos.z;

    V3F desAccel;
    
    accelCmd[0] = CONSTRAIN(accelCmd[0], -maxAccelXY, maxAccelXY);
    accelCmd[1] = CONSTRAIN(accelCmd[1], -maxAccelXY, maxAccelXY);
    
    velCmd[0] = CONSTRAIN(velCmd[0], -maxSpeedXY,maxSpeedXY);
    velCmd[1] = CONSTRAIN(velCmd[1], -maxSpeedXY,maxSpeedXY);

    
    desAccel.x = kpPosXY*(posCmd[0] - pos[0]) + kpVelXY*(velCmd[0] - vel[0]) + accelCmd[0];
    desAccel.y = kpPosXY*(posCmd[1] - pos[1]) + kpVelXY*(velCmd[1] - vel[1]) + accelCmd[1];
    
    desAccel.x = -desAccel.x;
    desAccel.y = -desAccel.y;
    desAccel.x = CONSTRAIN(desAccel.x, -maxAccelXY, maxAccelXY);
    desAccel.y = CONSTRAIN(desAccel.y, -maxAccelXY, maxAccelXY);

    desAccel.z = 0;
    
    return desAccel;
}

// returns desired yaw rate
float QuadControl::YawControl(float yawCmd, float yaw)
{

    float yawRateCmd=0;
    float yaw_error = yawRateCmd - yaw;
    yaw_error = fmodf(yaw_error, F_PI*2.f);

    if (yaw_error >F_PI){
        yaw_error = yaw_error - 2.0f*F_PI;
    } else if (yaw_error < -M_PI){
        yaw_error = yaw_error + 2.0f*F_PI;
    }
    yawRateCmd = kpYaw*yaw_error;
    return yawRateCmd;

}

VehicleCommand QuadControl::RunControl(float dt, float simTime)
{
  curTrajPoint = GetNextTrajectoryPoint(simTime);

  float collThrustCmd = AltitudeControl(curTrajPoint.position.z, curTrajPoint.velocity.z, estPos.z, estVel.z, estAtt, curTrajPoint.accel.z, dt);

  // reserve some thrust margin for angle control
  float thrustMargin = .1f*(maxMotorThrust - minMotorThrust);
  collThrustCmd = CONSTRAIN(collThrustCmd, (minMotorThrust+ thrustMargin)*4.f, (maxMotorThrust-thrustMargin)*4.f);
  
  V3F desAcc = LateralPositionControl(curTrajPoint.position, curTrajPoint.velocity, estPos, estVel, curTrajPoint.accel);
  
  V3F desOmega = RollPitchControl(desAcc, estAtt, collThrustCmd);
  desOmega.z = YawControl(curTrajPoint.attitude.Yaw(), estAtt.Yaw());

  V3F desMoment = BodyRateControl(desOmega, estOmega);

  return GenerateMotorCommands(collThrustCmd, desMoment);
}
