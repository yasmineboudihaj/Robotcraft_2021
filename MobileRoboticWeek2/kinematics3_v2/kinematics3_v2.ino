#include <Encoder.h>

// pin numbers
const byte encL1 = 2;
const byte encL2 = 3;
const byte encR1 = 18;
const byte encR2 = 19;

// initialize encoders
Encoder encR(encL1, encL2);
Encoder encL(encR2, encR1);

struct Pose
{
  float x;
  float y;
  float theta;
    
  void print()
  {
    Serial.print("(");
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.print(", ");
    Serial.print(theta);
    Serial.print(")");
    Serial.println();
  }
  
};

class Robot
{
  private:
    float r;  // wheel radius
    float b;  // distance between wheels
    int C;    // encoder resolution
    Pose pose;
  
  public:
  Robot(float _r, float _b, int _C): r{_r}, b{_b}, C{_C}
  {
    pose = {0., 0., 0.};
  }

  // Setter and getter
  float getR() { return r; }
  float getB() { return b; }
  int   getC() { return C; }
  struct Pose getPose(){return pose;}
  void setPose(float x, float y, float theta)
  { 
    pose.x = x;
    pose.y = y;
    pose.theta = theta;
  }

  // print functions
  void printPose(){ pose.print(); }
  void print(){
    Serial.print("Radius: ");
    Serial.print(r);
    Serial.print(", wheel distance: ");
    Serial.print(b);
    Serial.print(", encoder resoultion: ");
    Serial.print(C);
    Serial.print(", pose: ");
    pose.print();
    Serial.println();
  }
  
};

// Initialize Robot
Robot robot = Robot(0.015, 0.095, 12 * 298);

// calculates new pose of the robot based on current number of encoder pulses
// and robots's physical properties
void poseUpdate2(int NL, int NR, class Robot& robot, float deltaT, float* realVels)
{
  float coefficient = 2 * PI * robot.getR()/robot.getC()/deltaT;
  float v = coefficient * (NR + NL)/2.;
  float w = coefficient * (NR - NL)/robot.getB();

  realVels[0] = v;
  realVels[1] = w;
  
  Pose pose = robot.getPose();
  float theta = atan2(sin(pose.theta + w * deltaT),
                      cos(pose.theta + w * deltaT));
  robot.setPose(pose.x + v * cos(theta) * deltaT,
                pose.y + v * sin(theta) * deltaT, 
                theta);
}

// returns the desired linear and angular velocities
void cmd_vel(float v, float w, float* result)
{
  result[0] = v;
    result[1] = w;
}

// converts lineaer and angular velocities to wheels angular velocities
void cmd_vel2wheels(float v, float w, class Robot& robot, float* result)
{
  result[0] = (v - robot.getB()/2. * w)/robot.getR();
    result[1] = (v + robot.getB()/2. * w)/robot.getR();
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

long unsigned currentTime;
// Arduino
void setup() {
  Serial.begin(9600);
  currentTime = millis();
}

float desiredVels[2]; // 0 - linear, 1 - angular
float desiredWheelVels[2]; // 0 - left, 1 - right
float realVels[2];
float realWheelVels[2];
float* propErrorOld;
float* propErrorNew;
float* derivError;
float* integralErrorOld;
float* integralErrorNew;
float* gain;

float deltaT = 0.1;
void loop() {
  int currentPulses[2] = {0, 0};
  int oldPulses[2] = {0, 0};
  propErrorOld[0] = 0;
  propErrorNew[1] = 0;
  integralErrorOld[0] = 0;
  integralErrorOld[1] = 0;
  
  cmd_vel(0.01, 0.01, desiredVels);
  cmd_vel2wheels(desiredVels[0], desiredVels[1], robot, desiredWheelVels);
  while (millis() - currentTime > 100){ // 100ms
   // output
    //robot.printPose();
    currentPulses[0] = encL.read();
    currentPulses[1] = encR.read();
    Serial.print(currentPulses[0]);
    Serial.print(" ");
    Serial.print(currentPulses[1]);
    Serial.println();
    
    // update robot pose
    poseUpdate2(currentPulses[0] - oldPulses[0],
                currentPulses[1] - oldPulses[1], 
                robot, deltaT, realVels);
    // convert to angular velocities of the wheels
    cmd_vel2wheels(realVels[0], realVels[1], robot, realWheelVels);
    propErrorNew = getPropError(desiredWheelVels[0], desiredWheelVels[1], 
                             realWheelVels[0]   , realWheelVels[1]);
    derivError = getDerivError(propErrorOld, propErrorNew);
    integralErrorNew = getIntegralError(integralErrorOld, propErrorNew);

    gain = pid(0.1, 0.1, 0.1, propErrorNew, derivError, integralErrorNew, deltaT);
    Serial.print(gain[0]);
    Serial.print(" ");
    Serial.print(gain[1]);
    Serial.println();
    // updates
    oldPulses[0] = currentPulses[0];
    oldPulses[1] = currentPulses[1];
    
    propErrorOld[0] = propErrorNew[0];
    propErrorOld[1] = propErrorNew[1];
    integralErrorOld[0] = integralErrorNew[0];
    integralErrorOld[1] = integralErrorNew[1];
    currentTime = millis();
  }
  
}
