#include <Encoder.h>

// encoder pins
const byte encL1 = 2;
const byte encL2 = 3;
const byte encR1 = 18;
const byte encR2 = 19;

// motor pins
const byte rightMotorSpeed = 4;
const byte rightMotorToggle = 5;
const byte leftMotorSpeed = 9;
const byte leftMotorToggle = 6;

// initialize encoders
Encoder encR(encL1, encL2);
Encoder encL(encR2, encR1);

float R = 0.015;
float B = 0.095;
float C = 12 * 298;

// calculates new pose of the robot based on current number of encoder pulses
// and robots's physical properties
void poseUpdate(float NL, float NR, float* pose, float deltaT, float* realVels){
    float coefficient = 2 * PI * R/(C * deltaT);
    float v = coefficient * (NR + NL)/2.;
    float w = coefficient * (NR - NL)/B;
    float theta = atan2(sin(pose[2] + w * deltaT), cos(pose[2] + w * deltaT));

    realVels[0] = v;
    realVels[1] = w;
    pose[0] = pose[0] + v * cos(theta) * deltaT;
    pose[1] = pose[1] + v * sin(theta) * deltaT;
    pose[2] = theta;

    Serial.print(pose[0]);
    Serial.print(" ");
    Serial.print(pose[1]);
    Serial.print(" ");
    Serial.print(pose[2]);
    Serial.print(" ");
    Serial.println();
  }

// converts lineaer and angular velocities to wheels angular velocities
void cmd_vel2wheels(float v, float w, float* result)
{
    result[0] = (v - B/2. * w)/R;
    result[1] = (v + B/2. * w)/R;
}

// calculate proportional error
void getPropError(float w_d_l, float w_d_r, float w_l, float w_r, float* propError)
{
  propError[0] = w_d_l - w_l;
  propError[1] = w_d_r - w_r;
}

void getDerivError(float* prop_error_old, float* prop_error_new, float* deriv_error)
{
  deriv_error[0] = prop_error_new[0] - prop_error_old[0];
  deriv_error[1] = prop_error_new[1] - prop_error_old[1];
}

void getIntegralError(float* integral_error_old, float* prop_error_new, float* integral_error)
{  
  integral_error[0] = integral_error_old[0] + prop_error_new[0];
  integral_error[1] = integral_error_old[1] + prop_error_new[1];
}

void pid(float Kp, float Ki, float Kd, float* prop_error, float* deriv_error, float* integral_error, float deltaT, float* gain)
{
  gain[0] = Kp * prop_error[0] + Ki * integral_error[0] * deltaT + Kd * deriv_error[0] / deltaT;
  gain[1] = Kp * prop_error[1] + Ki * integral_error[1] * deltaT + Kd * deriv_error[1] / deltaT;
}

long unsigned currentTime;
// Arduino
void setup() {
  Serial.begin(9600);
  currentTime = millis();
}

float currentPulses[2] = {0, 0};
float oldPulses[2] = {0, 0};
float propErrorOld[2] = {0, 0};
float propErrorNew[2] = {0, 0};
float integralErrorOld[2] = {0, 0};
float integralErrorNew[2] = {0, 0};
float derivErrorOld[2] = {0, 0};
float derivErrorNew[2] = {0, 0};
float desiredVels[2];
float desiredWheelVels[2];
float realVels[2];
float realWheelVels[2];
float gain[2];
float pose[3] = {0, 0, 0};
float deltaT = 0.1;

  
void loop() {
    cmd_vel2wheels(0., 0, desiredWheelVels);
    if (millis() - currentTime > 100){ // 100ms
      // output
      currentPulses[0] = encL.read();
      currentPulses[1] = encR.read();

      // update robot pose
      poseUpdate(currentPulses[0] - oldPulses[0],
                  currentPulses[1] - oldPulses[1], 
                  pose ,deltaT, realVels);
      // convert to angular velocities of the wheels
      cmd_vel2wheels(realVels[0], realVels[1], realWheelVels);
      getPropError(desiredWheelVels[0], desiredWheelVels[1], 
                               realWheelVels[0]   , realWheelVels[1], propErrorNew);
      getDerivError(propErrorOld, propErrorNew, derivErrorNew);
      getIntegralError(integralErrorOld, propErrorNew, integralErrorNew);
  
      pid(67, 0., 0, propErrorNew, derivErrorNew, integralErrorNew, deltaT, gain); 

      if (gain[0] > 0) digitalWrite(leftMotorToggle, LOW);
      else digitalWrite(leftMotorToggle, HIGH);
      if (gain[1] > 0) digitalWrite(rightMotorToggle, LOW);
      else digitalWrite(rightMotorToggle, HIGH);

      if (gain[0] > 255) gain[0] = 255;
      if (gain[1] > 255) gain[1] = 255;
      if (gain[0] < -255) gain[0] = -255;
      if (gain[1] < -255) gain[1] = -255;
      analogWrite(leftMotorSpeed, abs((int)gain[0]));
      analogWrite(rightMotorSpeed, abs((int)gain[1]));
      Serial.print("gain");
      Serial.print(gain[0]);
      Serial.print(" ");
      Serial.print(gain[1]);
      Serial.println();
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
