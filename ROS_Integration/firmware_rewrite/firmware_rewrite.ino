// ROS Headers
#include <ros.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

#include <Encoder.h>
#include <SharpIR.h>
#include <Average.h>

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

// sensor pin numbers
const byte sensorLeftPin  = A3;
const byte sensorFrontPin = A4;
const byte sensorRightPin = A2;

// object to get distance from sensors
SharpIR sensorLeft (SharpIR::GP2Y0A21YK0F, sensorLeftPin );
SharpIR sensorFront(SharpIR::GP2Y0A21YK0F, sensorFrontPin);
SharpIR sensorRight(SharpIR::GP2Y0A21YK0F, sensorRightPin);

// We take the average of the ten last sensor outputs
Average<float> avgLeft(10);
Average<float> avgFront(10);
Average<float> avgRight(10);

// ROS
ros::NodeHandle nh;

// Publish robot pose
geometry_msgs::Pose2D pose_for_ros;
ros::Publisher pose_pub("pose", &pose_for_ros);

// Publish sensor ranges
std_msgs::Float32 front_distance, right_distance, left_distance;
ros::Publisher front_distance_pub("front_distance", &front_distance);
ros::Publisher right_distance_pub("right_distance", &right_distance);
ros::Publisher left_distance_pub ("left_distance" , &left_distance );

// Subscribe to cmd_vel to get velocity
float linear_vel  = 0.;
float angular_vel = 0.;

void cmd_vel_callback(const geometry_msgs::Twist& msg){
  linear_vel = msg.linear.x;
  angular_vel = msg.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", cmd_vel_callback);

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
  // Output
  pinMode(rightMotorSpeed, OUTPUT);
  pinMode(rightMotorToggle, OUTPUT);
  pinMode(leftMotorSpeed, OUTPUT);
  pinMode(leftMotorToggle, OUTPUT);
   
  // Input
  pinMode(encL1,INPUT);
  pinMode(encL2,INPUT);
  pinMode(encR1,INPUT);
  pinMode(encR2,INPUT);
  pinMode(sensorLeftPin,INPUT);
  pinMode(sensorFrontPin,INPUT);
  pinMode(sensorRightPin,INPUT);

  nh.initNode();
  //nh.getHardware()->setBaud(57600);
  
  // The topics that robot publishes:
  nh.advertise(pose_pub);
  
  nh.advertise(front_distance_pub);
  nh.advertise(right_distance_pub);
  nh.advertise(left_distance_pub);

  // The topics that robot subscribes to:
  nh.subscribe(cmd_vel_sub);
  
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
    cmd_vel2wheels(linear_vel, angular_vel, desiredWheelVels); //anguar: 0 to 2, linear: 0 to 0.1
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
  
      pid(20, 100, 0, propErrorNew, derivErrorNew, integralErrorNew, deltaT, gain); 

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

      pose_for_ros.x = pose[0];
      pose_for_ros.y = pose[1];
      pose_for_ros.theta = pose[2];
      pose_pub.publish(&pose_for_ros);
  
      avgLeft.push(sensorLeft.getDistance());
      avgFront.push(sensorFront.getDistance());
      avgRight.push(sensorRight.getDistance());
      
      front_distance.data = avgLeft.mean();
      front_distance_pub.publish(&front_distance);
      
      right_distance.data = avgFront.mean();
      right_distance_pub.publish(&right_distance);
      
      left_distance.data = avgRight.mean();
      left_distance_pub.publish(&left_distance);
      
      // updates
      oldPulses[0] = currentPulses[0];
      oldPulses[1] = currentPulses[1];
      
      propErrorOld[0] = propErrorNew[0];
      propErrorOld[1] = propErrorNew[1];
      
      integralErrorOld[0] = integralErrorNew[0];
      integralErrorOld[1] = integralErrorNew[1];    
      
      currentTime = millis();
    }
    nh.spinOnce();
}
