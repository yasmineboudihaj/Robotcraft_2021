#include <Encoder.h>

const byte encL1 = 2;
const byte encL2 = 3;
const byte encR1 = 18;
const byte encR2 = 19;

long positionL, positionR, newPositionL, newPositionR;
int startTime;

Encoder encL(encL1, encL2);
Encoder encR(encR2, encR1);

long readIn(Encoder enc)
{
  return abs(enc.read()); 
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); 
  positionL = readIn(encL);
  positionR = readIn(encR);
  startTime = millis();

}

void loop() {
  // put your main code here, to run repeatedly:
  if (millis() - startTime > 100) //10Hz -> 1/10s -> 0.1s -> 100ms
  { 
     newPositionL = readIn(encL);
     newPositionR = readIn(encR);
     Serial.print("Difference left: ");
     Serial.println(positionL - newPositionL);
     Serial.print("Position right: ");
     Serial.println(positionR - newPositionR); 
     positionL = newPositionL;
     positionR = newPositionR;
     startTime = millis();
  }
}
