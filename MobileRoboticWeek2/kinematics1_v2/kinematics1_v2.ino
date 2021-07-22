// Save 'encoder' values
int encoderVals[10][2] = 
              {{   0,   0},
               { 100, 100},
               { 300, 200},
               { 500, 300},
               { 700, 400},
               { 900, 500},
               {1000, 600},
               {1100, 700}, 
               {1200, 800},
               {1200, 800}};

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
//Robot robot = Robot(1.5, 9.5, 12 * 298);
Robot robot = Robot(5, 20, 100);

// get the current number of encoder pulses
void encUpdate(int* array, int t)
{
  array[0] = encoderVals[t][0];
  array[1] = encoderVals[t][1];    
}

// calculates new pose of the robot based on current number of encoder pulses
// and robots's physical properties
void poseUpdate(int NL, int NR, class Robot& robot)
{
  float coefficient = 2 * PI * robot.getR()/robot.getC();
    float DL = coefficient * NL;
    float DR = coefficient * NR;
    float D = (DL + DR)/2.;
    
    Pose pose = robot.getPose();
    float dTheta = (DR - DL)/robot.getB();

    robot.setPose(pose.x + D * cos(dTheta),
                  pose.y + D * sin(dTheta), 
                  pose.theta + dTheta);
}

void setup() {
  Serial.begin(9600);
}

void loop() {
  int currentPulses[2] = {0, 0};
  int oldPulses[2] = {0, 0};
  
  for (int t = 0; t < 10; t++)
  {
    // output
    Serial.print("Iteration ");
    Serial.print(t);
    Serial.print(": ");
    robot.printPose();
    
    // get new encoder pulses since last iteration
    encUpdate(currentPulses, t);
    // update robot pose
    poseUpdate(currentPulses[0] - oldPulses[0],
               currentPulses[1] - oldPulses[1],
               robot);
               
    oldPulses[0] = currentPulses[0];
    oldPulses[1] = currentPulses[1];
    
  }
  // reset
  robot.setPose(0, 0, 0);
  
}
