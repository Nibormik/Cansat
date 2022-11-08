// The GY91 reads the data from the GY91 module
#include <GY91.h>

// Declare all variables
double pressure;
GY91 gy91; // We need to make an instance of the GY91 library object

void setup() {
  // Start USB communication to the computer
  Serial.begin(9600);
  // Wait until communication with the computer is set up
  while(!Serial);

  // Initiate the gy91 object, and stop if it could not init
  if (!gy91.init()) {
    Serial.println("Could not initiate");
    while(1);
  }
}

void loop() {
  // Update all variables (acc, gyro, mag and pressure)
  gy91.read_acc();
  gy91.read_gyro();
  gy91.read_mag();
  // The pressure is returned in the readPressure function
  pressure = gy91.readPressure();

  // Now we can print this to screen
  Serial.print(gy91.ax);
  Serial.print("\t");
  Serial.print(gy91.ay);
  Serial.print("\t");
  Serial.print(gy91.az);
  Serial.print("\t");

  Serial.print(gy91.gx);
  Serial.print("\t");
  Serial.print(gy91.gy);
  Serial.print("\t");
  Serial.print(gy91.gz);
  Serial.print("\t");
    
  Serial.print(gy91.mx);
  Serial.print("\t");
  Serial.print(gy91.my);
  Serial.print("\t");
  Serial.print(gy91.mz);
  Serial.print("\t");

  Serial.print(pressure);
  Serial.println();

  // Delay for 100 ms, so we do not get too much data on the screen
  delay(100);
}
