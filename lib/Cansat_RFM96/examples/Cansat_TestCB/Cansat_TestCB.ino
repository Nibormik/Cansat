// The GY91 reads the data from the GY91 module
#include <GY91.h>

#define USE_BUZZER            0
#define OUTPUT_TEMP           1
#define OUTPUT_VIN            1
#define OUTPUT_ACC            1
#define OUTPUT_GYRO           1
#define OUTPUT_MAG            1
#define OUTPUT_PRESSURE       1

#define BUZZER_PIN            29
#define BUZZER_TEMP           31.0

unsigned long _time=0;
double ax, ay, az, gx, gy, gz, mx, my, mz, pressure;
GY91 gy91; // We need to make an instance of the GY91 library object

void setup() {
  Serial.begin(9600);
  while(!Serial);

  analogWriteResolution(16);
  
  digitalWriteFast(BUZZER_PIN, LOW);
  pinMode(BUZZER_PIN, OUTPUT);

  if (!gy91.init()) {
    Serial.println("Could not initiate");
    while(1);
  }
}

void loop() {
#if USE_BUZZER
  if (read_temp_direct() > BUZZER_TEMP)
    analogWrite(BUZZER_PIN, (read_temp_direct()-BUZZER_TEMP)*1000);
  else
    analogWrite(BUZZER_PIN, 0);
#endif 
  
  if (millis()-_time > 100) { // Makes sure that we do not send data too often
    _time = millis();

    gy91.read_acc();
    gy91.read_gyro();
    gy91.read_mag();

    pressure = gy91.readPressure();

    ax = gy91.ax;
    ay = gy91.ay;
    az = gy91.az;

    gx = gy91.gx;
    gy = gy91.gy;
    gz = gy91.gz;

    mx = gy91.mx;
    my = gy91.my;
    mz = gy91.mz;

#if OUTPUT_VIN == 1
    Serial.print(analogRead(A11)*3.3*2/1023); // Multiply with 2, since we have a 1/2 resistor divider
    Serial.print("\t");
#endif

#if OUTPUT_TEMP == 1
    Serial.print(read_temp_direct());
    Serial.print("\t");
#endif

#if OUTPUT_ACC == 1
    Serial.print(ax);
    Serial.print("\t");
    Serial.print(ay);
    Serial.print("\t");
    Serial.print(az);
    Serial.print("\t");
#endif

#if OUTPUT_GYRO == 1
    Serial.print(gx);
    Serial.print("\t");
    Serial.print(gy);
    Serial.print("\t");
    Serial.print(gz);
    Serial.print("\t");
#endif

#if OUTPUT_MAG == 1
    Serial.print(mx);
    Serial.print("\t");
    Serial.print(my);
    Serial.print("\t");
    Serial.print(mz);
    Serial.print("\t");
#endif

#if OUTPUT_PRESSURE == 1
    Serial.print(pressure/1000,4); // it is in Pascals, but we want it in centi bar to make
                                  // it easier to plot, which is one tenth of hPa/mbar
#endif
    
    Serial.println();
  }
}

// This function reads the Cansat temperature in centigrades. It
// uses the Steinhart-Hart equation
double read_temp_direct() {
  // This should be set only once in setup, but for simplicity we do it here
  analogReadResolution(12);
  
  double R_NTC, log_NTC;
  uint16_t ARead = analogRead(A10);
  R_NTC = 4700*ARead/(4095.0-ARead);
  log_NTC = log(R_NTC/10000);

  // The line below is the Steinhart-Hart equation
  return 1/(3.354016E-3 + 2.569850E-4*log_NTC + 2.620131E-6*log_NTC*log_NTC + 6.383091E-8*log_NTC*log_NTC*log_NTC)-273.15;
}

