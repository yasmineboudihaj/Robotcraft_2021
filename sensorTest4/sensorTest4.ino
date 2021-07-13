#include <Average.h>

// Pin numbers
const byte sensorLeft = A2;
const byte sensorFront = A3;
const byte sensorRight = A4;

// We take the average of the ten last sensor outputs
Average<float> avgLeft(10);
Average<float> avgFront(10);
Average<float> avgRight(10);

//reads in analog sensor data input (0 to 1023) which is then converts to a voltage (0 to 5V),
// and finally to a distance
// formula to convert voltage to distance taken from here:
// https://electropeak.com/learn/interfacing-sharp-gp2y0a21yk0f-ir-distance-sensor-with-arduino/
float sense(byte pinNr)
{
  return 10 * 27.728 * 1.3 * pow(map(analogRead(pinNr), 0, 1023, 0, 5000) / 1000.0, -1.2045);
}

// Auxiliary functions to read in left, front and right sensor
float senseL()
{
  return sense(sensorLeft);
}
float senseR()
{
  return sense(sensorRight);  
}
float senseF()
{
  return sense(sensorFront);  
}
void setup() {
    Serial.begin(9600);
}

void loop() {
    
    // Add a new random value to the bucket
    avgLeft.push(senseL());
    Serial.print("Sensor 1: ");
    Serial.println(avgLeft.mean());
    
    avgFront.push(senseF());
    Serial.print("Sensor 2: ");
    Serial.println(avgFront.mean());
    
    avgRight.push(senseR());
    Serial.print("Sensor 3: ");
    Serial.println(avgRight.mean());

    Serial.println();
    delay(100);
}
