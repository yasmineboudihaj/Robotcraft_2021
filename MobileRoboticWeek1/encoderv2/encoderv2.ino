#include <Encoder.h>

// pin numbers
const byte encL1 = 2;
const byte encL2 = 3;
const byte encR1 = 18;
const byte encR2 = 19;

long positionL, positionR;
unsigned long startTime;

// initialize encoders
Encoder encL(encL2, encL1);
Encoder encR(encR1, encR2);

// this function calculates the difference between the old and new
// encoder values (left + right)
void getDifferences(long& positionL, long& positionR)
{
     long newPositionL = readIn(encL);
     long newPositionR = readIn(encR);
     Serial.print("Difference left: ");
     Serial.println(positionL - newPositionL);
     Serial.print("Position right: ");
     Serial.println(positionR - newPositionR); 
     Serial.println();
     positionL = newPositionL;
     positionR = newPositionR;
}

// returns absolute value from the encoder
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
  // get difference every 0.1s (10Hz)
  if (millis() - startTime > 100) 
  { 
     getDifferences(positionL, positionR);
     startTime = millis();
  }
}
