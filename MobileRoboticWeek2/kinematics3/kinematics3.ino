#include <Encoder.h>

// pin numbers
const byte encL1 = 2;
const byte encL2 = 3;
const byte encR1 = 18;
const byte encR2 = 19;

// initialize encoders
Encoder encL(encL2, encL1);
Encoder encR(encR1, encR2);

struct Pose
{
  float x;
  float y;
  float theta;
};

// the desired velocities
float angular_vel, linear_vel;
float* wheel_vels;
float* real_vels;

float test[] = {0.0, 0.0};
float* prop_error_old = test;
float test2[] = {0.0, 0.0};
float* integral_error_old = test2;
float* prop_error_new;
float* integral_error_new;
float* deriv_error;

float* gain;

struct Pose pose;

// robot values
float robot_r = 1.5;
float robot_b = 9.5;
int   robot_C = 12 * 298;

long unsigned currentTime;

void poseUpdate2(int NL, int NR, Pose& pose, float deltaT, int b, int r, int C)
{
  // get the real linear and angular velocities
  float v = (2*PI*r / C) * (NR + NL) / 2 / deltaT;
  float w = (2*PI*r / C) * (NR - NL) / b / deltaT;  

  pose.theta = atan2(sin(pose.theta + w * deltaT), cos(pose.theta + w *deltaT));
  pose.x = pose.x + v * cos(pose.theta) * deltaT;
  pose.y = pose.y + v * sin(pose.theta) * deltaT;
}

float* getRealVelocities(int NL, int NR, int b, int r, int C, float deltaT)
{
  static float real_vels[2] = {0.0};
  // get the real linear and angular velocities
  real_vels[0] = (2*PI*r / C) * (NR + NL) / 2 / deltaT;
  real_vels[1] = (2*PI*r / C) * (NR - NL) / b / deltaT;
  return real_vels;   
}

// converts linear and angular velocities to wheels angular velocities (slide 38)
float* cmd_vel2wheels(float v, float w, int b, int r)
{
    static float w_d[2] = {0.0};
    w_d[0] = (v - b/2. * w)/r;
    w_d[1] = (v + b/2. * w)/r;
    return w_d;
}

// calculate proportional error
float* getPropError(float w_d_l, float w_d_r, float w_l, float w_r)
{
  static float error[] = {0.0};
  error[0] = w_d_l - w_l;
  error[1] = w_d_r - w_r;
  return error;
}

float* getDerivError(float* prop_error_old, float* prop_error_new)
{
  static float deriv_error[] = {0.0};
  deriv_error[0] = prop_error_new[0] - prop_error_old[0];
  deriv_error[1] = prop_error_new[1] - prop_error_old[1];
  return deriv_error;
}

float* getIntegralError(float* integral_error_old, float* prop_error_new)
{
  static float integral_error[] = {0.0};
  integral_error[0] = integral_error_old[0] - prop_error_new[0];
  integral_error[1] = integral_error_old[1] - prop_error_new[1];
  return integral_error;
}

float* pid(float Kp, float Ki, float Kd, float* prop_error, float* deriv_error, float* integral_error, float deltaT)
{
  static float gain[] = {};
  gain[0] = Kp * prop_error[0] + Ki * integral_error[0] * deltaT + Kd * deriv_error[0] / deltaT;
  gain[1] = Kp * prop_error[1] + Ki * integral_error[1] * deltaT + Kd * deriv_error[1] / deltaT;
  return gain;
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  currentTime = millis();
  
  // desired velocities
  linear_vel  = 0.1;
  angular_vel = 0.1;
  wheel_vels = cmd_vel2wheels(linear_vel, angular_vel, robot_b, robot_r);

  pose.x = 0.0;
  pose.y = 0.0;
  pose.theta = 0.0;
}

void loop() {
  while(millis() - currentTime > 100) // deltaT 100ms = 0.1s
  {
      long NL = encL.read();
      long NR = encR.read();
      poseUpdate2(NL, NR, pose, 0.1, robot_b, robot_r, robot_C);
      real_vels = getRealVelocities(NL, NR, robot_b, robot_r, robot_C, 0.1);
      prop_error_new = getPropError(wheel_vels[0], wheel_vels[1], real_vels[0], real_vels[1]);
      deriv_error = getDerivError(prop_error_old, prop_error_new);
      integral_error_new = getIntegralError(integral_error_old, prop_error_new);
      gain = pid(1, 0, 0, prop_error_new, deriv_error, integral_error_new, 0.1);
      Serial.print(gain[0]);
      Serial.print(", ");
      Serial.print(gain[1]);
      Serial.println();
      
      prop_error_old = prop_error_new;
      integral_error_old = integral_error_new;
      currentTime = millis();
  }
}
