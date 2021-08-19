#include <Encoder.h>
#include <SharpIR.h>
#include <Average.h>

// ROS headers
#include <ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>

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

// sensor pin numbers
//const byte sensorLeftPin = A2;
//const byte sensorFrontPin = A3;
//const byte sensorRightPin = A4;
const byte sensorLeftPin = A3;
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

// initialize encoders
Encoder encR(encL1, encL2);
Encoder encL(encR2, encR1);

// global variables
long unsigned current_time;
float linear_vel  = 0.;
float angular_vel = 0.;

struct Pose
{
  float x;
  float y;
  float theta;
  
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
  
};

// Initialize Robot
Robot robot = Robot(0.015, 0.095, 12 * 298);

ros::NodeHandle nh;

std_msgs::Float32 front_distance, right_distance, left_distance;
geometry_msgs::Pose2D pose;

// Initialize publishers
ros::Publisher front_distance_pub("front_distance", &front_distance);
ros::Publisher right_distance_pub("right_distance", &right_distance);
ros::Publisher left_distance_pub ("left_distance" , &left_distance );

ros::Publisher pose_pub("pose", &pose);

// converts linear and angular velocities to wheels angular velocities
void cmd_vel2wheels(float v, float w, class Robot& robot, float* result)
{
  result[0] = (v - robot.getB()/2. * w)/robot.getR();
  result[1] = (v + robot.getB()/2. * w)/robot.getR();
}
// Initialize subscribers
void cmd_vel_callback(const geometry_msgs::Twist& msg){
  linear_vel = msg.linear.x;
  angular_vel = msg.angular.z;
}

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

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", cmd_vel_callback);

void setup() {
    
  // initialize node
  nh.initNode();

  
  // The topics that robot publishes:
  nh.advertise(front_distance_pub);
  nh.advertise(right_distance_pub);
  nh.advertise(left_distance_pub);
  nh.advertise(pose_pub);
  
  // The topics that robot subscribes to:
  nh.subscribe(cmd_vel_sub);
}

// store intermediate results
float desired_wheel_vels[2];
float real_vels[2];
float real_wheel_vels[2];
float* prop_error_old;
float* prop_error_new;
float* deriv_error;
float* integral_error_old;
float* integral_error_new;
float* gain;

float deltaT = 0.1;

void loop() {
  int current_pulses[2] = {0, 0};
  int old_pulses[2] = {0, 0};
  prop_error_old[0] = 0;
  prop_error_new[1] = 0;
  integral_error_old[0] = 0;
  integral_error_old[1] = 0;
  
  // publish data at 10Hz
  if (millis() - current_time > 100){ 
    // calculations
    cmd_vel2wheels(linear_vel, angular_vel, robot, desired_wheel_vels);

    // get encoder data
    current_pulses[0] = encL.read();
    current_pulses[1] = encR.read();
    
    // update robot pose
    poseUpdate2(current_pulses[0] - old_pulses[0],
                current_pulses[1] - old_pulses[1], 
                robot, deltaT, real_vels);
    cmd_vel2wheels(real_vels[0], real_vels[1], robot, real_wheel_vels);

    // calculate errors for pid
    prop_error_new = getPropError(desired_wheel_vels[0], desired_wheel_vels[1], 
                                real_wheel_vels[0]   , real_wheel_vels[1]);
    deriv_error = getDerivError(prop_error_old, prop_error_new);
    integral_error_new = getIntegralError(integral_error_old, prop_error_new);

    gain = pid(1.5, 0.3, 0.7, prop_error_new, deriv_error, integral_error_new, deltaT);

    float gain2 [2];
    //float desired_vels2[2];
    float real_vels2[2];
    
    gain2[0] = gain[0];
    gain2[1] = gain[1];
    //desired_vels2[0] = desired_vels[0];
    //desired_vels2[1] = desired_vels[1];
    real_vels2[0] = real_vels[0];
    real_vels2[1] = real_vels[1];
    
    if (gain2[0] < 0) digitalWrite(leftMotorToggle, HIGH);
    else digitalWrite(leftMotorToggle, LOW);
    if (gain2[1] < 0) digitalWrite(rightMotorToggle, HIGH);
    else digitalWrite(rightMotorToggle, LOW);

    if (gain2[0] > 255) gain2[0] = 255;
    if (gain2[1] > 255) gain2[1] = 255;
    if (gain2[0] < -255) gain2[0] = -255;
    if (gain2[1] < -255) gain2[1] = -255;
    analogWrite(leftMotorSpeed, abs(gain2[0]));
    analogWrite(rightMotorSpeed, abs(gain2[1]));
    
    // updates
    old_pulses[0] = current_pulses[0];
    old_pulses[1] = current_pulses[1];

    prop_error_old[0] = prop_error_new[0];
    prop_error_old[1] = prop_error_new[1];
    integral_error_old[0] = integral_error_new[0];
    integral_error_old[1] = integral_error_new[1];
    
    // publish data
    avgLeft.push(sensorLeft.getDistance());
    avgFront.push(sensorFront.getDistance());
    avgRight.push(sensorRight.getDistance());
    
    front_distance.data = avgLeft.mean();
    front_distance_pub.publish(&front_distance);
    
    right_distance.data = avgFront.mean();
    right_distance_pub.publish(&right_distance);
    
    left_distance.data = avgRight.mean();
    left_distance_pub.publish(&left_distance);

    Pose robot_pose = robot.getPose();
    pose.x = robot_pose.x;
    pose.y = robot_pose.y;
    pose.theta = robot_pose.theta;
    pose_pub.publish(&pose);

 
    current_time = millis();
    nh.spinOnce();
  }   
  
}
