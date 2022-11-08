#include <Cansat_RFM96.h>
#define USE_SD 0

Cansat_RFM96 rfm96(433500, USE_SD);
unsigned long time_counter=0;

void setup() {
  Serial.begin(9600);
  while(!Serial);

  Serial.println("Starting setup");
  
  if (!rfm96.init()) {
    Serial.println("Init of radio failed, stopping");
    while(1);
  }

  Serial.println("Found RFM96 radio, and it is working as expected");
  
  rfm96.setTxPower(5); // +5 dBm, approx 3 mW, which is quite low

  Serial.println("End of setup");
  Serial.println();
}

void loop() {
  uint8_t read_value;

  // We check if there it something in the buffer
  while (rfm96.available()) {
    // We keep track of the time since we last received something,
    // so the user can get feedback if the radio does not receive
    // something
    time_counter = millis();
    
    // Read it into a variable. Here we could just directly use:
    // Serial.write(rfm96.read());
    read_value = rfm96.read();

    // Write it to file. We do not use Serial.print, because the
    // conversion to readable ASCII has already been done when
    // we send it
    Serial.write(read_value);
  }

  if (millis()-time_counter > 5000) {
    time_counter = millis();
    Serial.println("We have not received anything in 5 seconds");
  }
}
