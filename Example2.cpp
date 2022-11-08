// The GY91 reads the data from the GY91 module
#include <GY91.h>
#include <Cansat_RFM96.h>
#define USE_SD 0

// Declare all variables
double pressure;
unsigned int counter=0;

// We need to make an instance of the two objects
GY91 gy91;
Cansat_RFM96 rfm96(433700, USE_SD);

void setup() {
  Serial.begin(9600);
  
  // Initiate the gy91 object, and stop if it could not init
  if (!gy91.init()) {
    Serial.println("Could not initiate");
    while(1);
  }
  if (!rfm96.init()) {
    Serial.println("Init of radio failed, stopping");
    while(1);
  }
rfm96.setTxPower(5);

  
}

void loop() {
  // Update the new pressure data
  pressure = gy91.readPressure();

  // Add the data to the buffer. These functions do not initiate a transfer
  rfm96.printToBuffer(millis());
  rfm96.printToBuffer(", ");
  rfm96.printToBuffer(counter++);
  rfm96.printToBuffer(", ");
  rfm96.printToBuffer(pressure);
  rfm96.printlnToBuffer(", ");

  // When the buffer is filled up of what we want, then transfer it to send.
  // There is no need to check if Tx is ready, since this function will
  // check if Tx is ready, and if not it will only write to file (if SD
  // is enabled). If neither is ready/enabled, it will do nothing.
  rfm96.sendAndWriteToFile();
  delay(500);
}
