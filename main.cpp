// The GY91 reads the data from the GY91 module
#include <GY91.h>
#include <Cansat_RFM96.h>
#define USE_SD 0

// Declare all variables
double pressure;
unsigned int counter = 0;
double Temp_celisius = 0;

// We need to make an instance of the two objects
GY91 gy91;
Cansat_RFM96 rfm96(433700, USE_SD);

void setup() {
  Serial.begin(9600);
  analogReadResolution(12);
  
  // Initiate the gy91 object, and stop if it could not init
  if (!gy91.init()) {
    Serial.println("Could not initiate");
    while(1);
  }
  if (!rfm96.init()) {
    Serial.println("Init of radio failed, stopping");
    while(1);
  }
rfm96.setTxPower(10);
}

double read_temp() {
  double R_NTC, log_NTC;
  uint16_t ARead = analogRead(A10);
  R_NTC = 4700*ARead/(4095.0-ARead);
  log_NTC = log(R_NTC/10000);

  // The line below is the Steinhart-Hart equation
  return 1/(3.354016E-3 + 2.569850E-4*log_NTC + 2.620131E-6*log_NTC*log_NTC + 6.383091E-8*log_NTC*log_NTC*log_NTC)-273.15;
}



void loop() {
  // Update the new pressure data
  pressure = gy91.readPressure();
  Temp_celisius = read_temp();
  gy91.read_acc();
  gy91.read_gyro();
  gy91.read_mag();
  

  // Add the data to the buffer. These functions do not initiate a 
  rfm96.printToBuffer("TITIN");
  rfm96.printToBuffer(";");
  rfm96.printToBuffer(millis());
  rfm96.printToBuffer(";");
  rfm96.printToBuffer(counter++);
  rfm96.printToBuffer(";");

  // Adds Gyro Acc and Mag Data To The Buffer
  rfm96.printToBuffer(gy91.ax);
  rfm96.printToBuffer(";");
  rfm96.printToBuffer(gy91.ay);
  rfm96.printToBuffer(";");
  rfm96.printToBuffer(gy91.az);
  rfm96.printToBuffer(";");

  rfm96.printToBuffer(gy91.gx);
  rfm96.printToBuffer(";");
  rfm96.printToBuffer(gy91.gy);
  rfm96.printToBuffer(";");
  rfm96.printToBuffer(gy91.gz);
  rfm96.printToBuffer(";");

  rfm96.printToBuffer(gy91.mx);
  rfm96.printToBuffer(";");
  rfm96.printToBuffer(gy91.my);
  rfm96.printToBuffer(";");
  rfm96.printToBuffer(gy91.mz);
  rfm96.printToBuffer(";");

  // adds Pressure and temp to Buffer
  rfm96.printToBuffer(Temp_celisius);
  rfm96.printToBuffer(";");
  rfm96.printToBuffer(pressure);
  rfm96.printlnToBuffer(";");

  // Sends Stored Bufferdata
  rfm96.sendAndWriteToFile();
  delay(500);
}
