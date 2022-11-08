/*
 * Teensy 3.5 test code.
 * This code is intended as a very first introduction to reading analog and digital values.
 * 
 * The code reads in two digital inputs and two analog inputs and display this on the Serial Monitor once every second.
 */

void setup() {
  Serial.begin(9600); // Baud rate doesn't matter, so just use 9600
  while(!Serial);

  pinMode(2, INPUT);
  pinMode(3, INPUT_PULLUP);
  analogReadResolution(12); // 12 bit, meaning maxium value is 4095 (2^12-1)
}

void loop() {
  Serial.print("Time: ");
  Serial.print(millis());
  Serial.print(" ms\tD2: ");
  Serial.print(digitalReadFast(2));
  Serial.print("\tD3: ");
  Serial.print(digitalReadFast(3));
  Serial.print("\tA0: ");
  Serial.print(analogRead(A0)*3.3/4095);
  Serial.print(" volts\tA1: ");
  Serial.print(analogRead(A1)*3.3/4095);
  Serial.print(" volts");
  Serial.println();

  delay(1000);
}
