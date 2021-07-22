#include <Average.h>

const byte sensor1 = A2;
const byte sensor2 = A3;
const byte sensor3 = A4;

// Reserve space for 10 entries in the average bucket.
// Change the type between < and > to change the entire way the library works.
Average<float> ave(10);

void setup() {
    Serial.begin(9600);
}

void loop() {
    int minat = 0;
    int maxat = 0;
    
    // Add a new random value to the bucket
    ave.push(27.728*1.3 * pow(map(analogRead(sensor1), 0, 1023, 0, 5000) / 1000.0, -1.2045));
    Serial.print("Sensor 1: ");
    Serial.println(ave.mean());
    
    ave.push(27.728*1.3 * pow(map(analogRead(sensor2), 0, 1023, 0, 5000) / 1000.0, -1.2045));
    Serial.print("Sensor 2: ");
    Serial.println(ave.mean());
    
    ave.push(27.728*1.3 * pow(map(analogRead(sensor3), 0, 1023, 0, 5000) / 1000.0, -1.2045));
    Serial.print("Sensor 3: ");
    Serial.println(ave.mean());

    Serial.println();
    delay(100);
}
