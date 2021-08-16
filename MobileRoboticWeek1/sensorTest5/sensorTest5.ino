#include <SharpIR.h>
#include <Average.h>

// Pin numbers
const byte sensorLeftPin = A2;
const byte sensorFrontPin = A3;
const byte sensorRightPin = A4;

SharpIR sensorLeft (SharpIR::GP2Y0A21YK0F, sensorLeftPin );
SharpIR sensorFront(SharpIR::GP2Y0A21YK0F, sensorFrontPin);
SharpIR sensorRight(SharpIR::GP2Y0A21YK0F, sensorRightPin);

// We take the average of the ten last sensor outputs
Average<float> avgLeft(10);
Average<float> avgFront(10);
Average<float> avgRight(10);

void setup() {
    Serial.begin(9600);
}

void loop() {
    
    // Add a new random value to the bucket
    avgLeft.push(sensorLeft.getDistance());
    //Serial.print("Sensor 1: ");
    //Serial.println(avgLeft.mean());
    //Serial.print(" ");
    
    avgFront.push(sensorFront.getDistance());
    //Serial.print("Sensor 2: ");
    //Serial.println(avgFront.mean());
    //Serial.print(" ");
    
    avgRight.push(sensorRight.getDistance());
    //Serial.print("Sensor 3: ");
    Serial.println(avgRight.mean());

    Serial.println();
    delay(100);
}
