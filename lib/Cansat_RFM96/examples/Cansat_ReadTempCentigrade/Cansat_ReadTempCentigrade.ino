void setup() {
  // Start USB communication to the computer
  Serial.begin(9600);
  // Wait until communication with the computer is set up
  while(!Serial);

  // To use all of the 12 bits of the ADC, we need to change
  // it from the default 10 bits, which is only to make it
  // compatible with the other Arduino boards (which are mostly
  // 10 bits maximum)
  analogReadResolution(12);
}

void loop() {
  // Declare that temp_celsius is a variable of type double
  double temp_celsius;

  // Store a new temperature reading, which is in degrees Celsius
  temp_celsius = read_temp_direct();

  // Print to the computer via the USB, and add a line shift
  Serial.println(temp_celsius);

  // Wait 100 ms
  delay(100);
}

// This function reads the Cansat temperature in centigrades. It
// uses the Steinhart-Hart equation
double read_temp_direct() {
  double R_NTC, log_NTC;
  uint16_t ARead = analogRead(A10);
  R_NTC = 4700*ARead/(4095.0-ARead);
  log_NTC = log(R_NTC/10000);

  // The line below is the Steinhart-Hart equation
  return 1/(3.354016E-3 + 2.569850E-4*log_NTC + 2.620131E-6*log_NTC*log_NTC + 6.383091E-8*log_NTC*log_NTC*log_NTC)-273.15;
}
