#include <Wire.h>
//#include "SparkFun_SCD4x_Arduino_Library.h"
#include <Adafruit_BMP280.h>
#include <Adafruit_GPS.h>
#include <DFRobot_ENS160.h>
#include "hardware/adc.h"
#include "Adafruit_INA219.h"

#define GPSSerial Serial1
#define GPSECHO false
#define Vref_value 2485.0 //max. 3300 in mV 
#define ENS160_Address 0x53

// Declare sensors
//SCD4x mySensor;
Adafruit_BMP280 bmp;
Adafruit_GPS GPS(&GPSSerial);
DFRobot_ENS160_I2C ENS160(&Wire, /*I2CAddr*/ ENS160_Address);
Adafruit_INA219 ina219;

// Define control pin for the multiplexer
const byte PSPin = 23; //  Set internal DC-DC converter mode
const byte VBusPin = 24; // USB mode detect
const byte UV_sensor1_Pin = 26;    // Analog pin connected to UV sensor1
const byte UV_sensor2_Pin = 27;    // Analog pin connected to UV sensor2
const byte NTCPin = 28; // Analog pin connected to Battery NTC sensor
const byte VsysPin = 29; // Analog pin connected to internal Vsys voltage sensor

const int Sensors_interval = 1000;   // Sensors readout interval ms
const int ADC_conversion10 = (Vref_value * 1000) / (1 << 10);; //for 10bit ADC
const int ADC_conversion12 = (Vref_value * 1000) / (1 << 12); //for 12bit ADC
const int SERIESRESISTOR = 10000;
const int NOMINAL_RESISTANCE = 10000;
const byte NOMINAL_TEMPERATURE = 25;
const int  BCOEFFICIENT = 3950;
const byte BAT_Temp_ADC_sampling = 5;
const byte CPU_Temp_ADC_sampling = 5;
const byte Vsys_ADC_sampling = 5;
const byte UV_ADC_sampling = 5;
const long Sea_level_pressure = 1032000; //bar
const byte BMP280_Address = 0x76;
const byte sensors_multiplier = 10;
const bool PFM_mode = 0;
const bool PWM_mode = 1;

unsigned long UVStartmillis = millis();
unsigned long timer = millis();
bool BMP_280 = 0;
bool ENS_160 = 0;
bool INA_219 = 0;

SerialPIO serial3(12, 13);

void setup() {
  Serial.begin(115200);
  //Serial1.begin(115200);
  //Serial2.setTX(8);
  //Serial2.setRX(9);
  //Serial2.begin(115200);
  serial3.begin(115200);
  Wire.begin();

  BMP280_init();
  //SCD4x_init();
  ENS_init();
  current_init();

  // Set GPIOs and ADC resolution
  pinMode(PSPin, OUTPUT);
  pinMode(UV_sensor1_Pin, INPUT);
  pinMode(UV_sensor2_Pin, INPUT);
  pinMode(NTCPin, INPUT);
  pinMode(VsysPin, INPUT);
  pinMode(VBusPin, INPUT);
  analogReadResolution(12);

  Serial.println("Adafruit GPS library basic parsing test!");
  serial3.println("Adafruit GPS library basic parsing test!");
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);

  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);

  //Set Pico internal voltage converter mode
  digitalWrite(PSPin, PWM_mode);
}

void loop() {
  Get_GPS_data ();
  if (millis() - UVStartmillis >= Sensors_interval)
  {
    //Read_SCD4x();
    ENS_readData();
    Serial.println();
    serial3.println();
    Read_BMP280();
    Serial.println();
    serial3.println();
    ReadUVSensors(UV_sensor1_Pin, UV_sensor2_Pin, UV_ADC_sampling);
    Serial.println();
    serial3.println();
    CPU_temp(CPU_Temp_ADC_sampling);
    Serial.println();
    serial3.println();
    System_voltage(VsysPin, Vsys_ADC_sampling);
    Serial.println();
    serial3.println();
    ReadBatTemperature(NTCPin, BAT_Temp_ADC_sampling);
    Serial.println();
    serial3.println();
    get_current();
    Serial.println();
    serial3.println();
    sensor_status();
    Serial.println();
    serial3.println();
    UVStartmillis = UVStartmillis + Sensors_interval;
  }
}

void CPU_temp(int ADC_sampling) {
  int ADCvalue = 0;
  adc_init();
  adc_set_temp_sensor_enabled (true);
  // Select ADC input 4 for internal temperature sensor
  adc_select_input(4);
  //Temperature calculation based on sensor characteristics
  for (byte i = 0; i < ADC_sampling ; i++)
  {
    ADCvalue = ADCvalue + adc_read();// sensor on analog pin
  }
  ADCvalue = ADCvalue / ADC_sampling;
  //int CPU_Temp2 = (27 - (((2.5 / 4096) * ADCvalue) - 0.706) / 0.001721) * 100;
  int CPU_Temp = (27 - ((((Vref_value / (1 << 12)) / 1000.0) * ADCvalue) - 0.706) / 0.001721) * 100;
  Serial.print(F("CPU Temperature = "));
  Serial.print(CPU_Temp);
  Serial.println("/100 °C");
  serial3.print(F("CPU Temperature = "));
  serial3.print(CPU_Temp);
  serial3.println("/100 °C");
  String CPU_data = "";
  if  (CPU_Temp < 0) {
    CPU_data += "0";
  }
  else {
    CPU_data += "1";
  }
  CPU_Temp = abs(CPU_Temp);
  if  (CPU_Temp < 10000 && CPU_Temp > 999) {
    CPU_data += "0";
  }
  if  (CPU_Temp < 1000 && CPU_Temp > 99) {
    CPU_data += "0";
    CPU_data += "0";
  }
  if  (CPU_Temp < 100 && CPU_Temp > 9) {
    CPU_data += "0";
    CPU_data += "0";
    CPU_data += "0";
  }
  if  (CPU_Temp < 10) {
    CPU_data += "0";
    CPU_data += "0";
    CPU_data += "0";
    CPU_data += "0";
  }
  CPU_data += CPU_Temp;
  Serial.print("CPU Temperature string: ");
  Serial.println(CPU_data);
  serial3.print("CPU Temperature string: ");
  serial3.println(CPU_data);
}

void System_voltage(byte ADC_pin, int ADC_sampling) {
  int ADCvalue = 0;
  for (byte i = 0; i < ADC_sampling ; i++)
  {
    ADCvalue = ADCvalue + (analogRead(ADC_pin));// sensor on analog pin
  }
  ADCvalue = ADCvalue / ADC_sampling;    // average
  int Vsys = ((ADC_conversion12 * ADCvalue) / 10000.0) * 3;
  Serial.print(F("Sytem_voltage = "));
  Serial.print(Vsys * 10);
  Serial.print(" mV");
  serial3.print(F("Sytem_voltage = "));
  serial3.print(Vsys * 10);
  serial3.print(" mV");
  bool USB_supply = 0;
  if (digitalRead(VBusPin) == HIGH) {
    Serial.println(" (USB supply)");
    serial3.println(" (USB supply)");
    USB_supply = 1;
  }
  else {
    Serial.println(" (Battery supply)");
    serial3.println(" (Battery supply)");
    USB_supply = 0;
  }
  String Vsys_data = "";
  Vsys_data += USB_supply;
  if  (Vsys < 100 && Vsys > 9) {
    Vsys_data += "0";
  }
  if  (Vsys < 10) {
    Vsys_data += "0";
    Vsys_data += "0";
  }
  Vsys_data += Vsys;
  Serial.print("Vsys voltage string: ");
  Serial.println(Vsys_data);
  serial3.print("Vsys voltage string: ");
  serial3.println(Vsys_data);
}

void ReadBatTemperature(byte ADC_pin, int ADC_sampling) {
  int ADCvalue = 0;
  //float Resistance;
  //float steinhart = 0;
  int Batt_Temp = 0;
  for (byte i = 0; i < ADC_sampling ; i++)
  {
    ADCvalue = ADCvalue + analogRead(ADC_pin);// sensor on analog pin
  }
  ADCvalue = ADCvalue / ADC_sampling;    // average
  //Serial1.println(sval);
  //convert value to resistance
  /*Resistance = ((1 << 12) / (ADCvalue / 1.0))  - 1;
    Resistance = SERIESRESISTOR / Resistance;
    steinhart = Resistance / NOMINAL_RESISTANCE; // (R/Ro)
    steinhart = log(steinhart); // ln(R/Ro)
    steinhart /= BCOEFFICIENT; // 1/B * ln(R/Ro)
    steinhart += 1.0 / (NOMINAL_TEMPERATURE + 273.15); // + (1/To)
    steinhart = 1.0 / steinhart; // Invert
    steinhart -= 273.15; // convert to C*/
  Batt_Temp = ((1.0 / (((log((SERIESRESISTOR / (((1 << 12) / (ADCvalue / 1.0)) - 1)) / NOMINAL_RESISTANCE)) / BCOEFFICIENT) +  (1.0 / (NOMINAL_TEMPERATURE + 273.15)))) - 273.15) * 10; // Steinhart formula
  Serial.print("Battery Temperature: ");
  Serial.print(Batt_Temp);
  Serial.println("/10 °C, ");
  serial3.print("Battery Temperature: ");
  serial3.print(Batt_Temp);
  serial3.println("/10 °C, ");
  String Batt_data = "";
  if  (Batt_Temp < 0) {
    Batt_data += "0";
  }
  else {
    Batt_data += "1";
  }
  Batt_Temp = abs(Batt_Temp);
  if  (Batt_Temp < 100 && Batt_Temp > 9) {
    Batt_data += "0";
  }
  if  (Batt_Temp < 10) {
    Batt_data += "0";
    Batt_data += "0";
  }
  Batt_data += Batt_Temp;
  Serial.print("Battery NTC sensor data string: ");
  Serial.println(Batt_data);
  serial3.print("Battery NTC sensor data string: ");
  serial3.println(Batt_data);
}

/*void Battery_current(byte ADC_pin, float ADC_sampling) {
  float ADCvalue = 0;
  if (digitalRead(VBusPin) == HIGH) {
    Serial.print(F("Battery_current = "));
    Serial.println(" 0 mA (USB supply)");
    serial3.print(F("Battery_current = "));
    serial3.println(" 0 mA (USB supply)");
  }
  else {
    for (byte i = 0; i < ADC_sampling ; i++)
    {
      ADCvalue = ADCvalue + analogRead(ADC_pin);// sensor on analog pin
    }
    ADCvalue = ADCvalue / ADC_sampling;    // average
    float Batt_current = ADC_conversion12 * ADCvalue;
    //float Batt_current = ((ADC_conversion12 * ADCvalue) / current_multiplier) * 1000;
    Serial.print(F("Battery_current = "));
    Serial.print(Batt_current);
    Serial.println(" mA");
    serial3.print(F("Battery_current = "));
    serial3.print(Batt_current);
    serial3.println(" mA");
    serial3.println("Hello");
  }
  }*/

void Get_GPS_data () {
  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) Serial.print(c);
  if (c) serial3.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    serial3.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }
  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer >= 2000) {
    timer = timer + 2000; // reset the timer
    Serial.print("\nTime: ");
    serial3.print("\nTime: ");
    if (GPS.hour < 10) {
      Serial.print('0');
      serial3.print('0');
    }
    Serial.print(GPS.hour, DEC); Serial.print(':');
    serial3.print(GPS.hour, DEC); serial3.print(':');
    if (GPS.minute < 10) {
      Serial.print('0');
      serial3.print('0');
    }
    Serial.print(GPS.minute, DEC); Serial.print(':');
    serial3.print(GPS.minute, DEC); serial3.print(':');
    if (GPS.seconds < 10) {
      Serial.print('0');
      serial3.print('0');
    }
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    serial3.print(GPS.seconds, DEC); serial3.print('.');
    if (GPS.milliseconds < 10) {
      Serial.print("00");
      serial3.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      Serial.print("0");
      serial3.print("0");
    }
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    serial3.println(GPS.milliseconds);
    serial3.print("Date: ");
    serial3.print(GPS.day, DEC); serial3.print('/');
    serial3.print(GPS.month, DEC); serial3.print("/20");
    serial3.println(GPS.year, DEC);
    serial3.print("Fix: "); serial3.print((int)GPS.fix);
    serial3.print(" quality: "); serial3.println((int)GPS.fixquality);
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
      Serial.print("Antenna status: "); Serial.println((int)GPS.antenna);
      serial3.print("Location: ");
      serial3.print(GPS.latitude, 4); serial3.print(GPS.lat);
      serial3.print(", ");
      serial3.print(GPS.longitude, 4); serial3.println(GPS.lon);
      serial3.print("Speed (knots): "); serial3.println(GPS.speed);
      serial3.print("Angle: "); serial3.println(GPS.angle);
      serial3.print("Altitude: "); serial3.println(GPS.altitude);
      serial3.print("Satellites: "); serial3.println((int)GPS.satellites);
      serial3.print("Antenna status: "); serial3.println((int)GPS.antenna);
    }
    Serial.println();
    serial3.println();
  }
}

void Read_BMP280() {
  BMP280_init();
  if (BMP_280 == 1 ) {
    // Read from BMP280
    Serial.println(F("BMP280 Data:"));
    Serial.print(F("Temperature (C): "));
    Serial.println(bmp.readTemperature());
    Serial.print(F("Pressure (kPa): "));
    Serial.println(bmp.readPressure() / 1000);
    Serial.print(F("Approx Altitude (m): "));
    Serial.println(bmp.readAltitude((Sea_level_pressure / 1000))); // Sea Level co2 in Budapest and we also substract 102meter because Budapest is 119 meter above sea levle
    serial3.println(F("BMP280 Data:"));
    serial3.print(F("Temperature (C): "));
    serial3.println(bmp.readTemperature());
    serial3.print(F("Pressure (kPa): "));
    serial3.println(bmp.readPressure() / 1000);
    serial3.print(F("Approx Altitude (m): "));
    serial3.println(bmp.readAltitude((Sea_level_pressure / 1000))); // Sea Level co2 in Budapes1t and we also substract 102meter because Budapest is 119 meter above sea levle
    int BMP_Temp = bmp.readTemperature() * sensors_multiplier * sensors_multiplier;
    int BMP_Pressure = bmp.readPressure() / sensors_multiplier;
    int BMP_Altitude = bmp.readAltitude((Sea_level_pressure / 1000)) * sensors_multiplier;
    String Temp_data = "";
    if  (BMP_Temp < 0) {
      Temp_data += "0";
    }
    else {
      Temp_data += "1";
    }
    BMP_Temp = abs(BMP_Temp);
    if  (BMP_Temp < 10000 && BMP_Temp > 999) {
      Temp_data += "0";
    }
    if  (BMP_Temp < 1000 && BMP_Temp > 99) {
      Temp_data += "0";
      Temp_data += "0";
    }
    if  (BMP_Temp < 100 && BMP_Temp > 9) {
      Temp_data += "0";
      Temp_data += "0";
      Temp_data += "0";
    }
    if  (BMP_Temp < 10) {
      Temp_data += "0";
      Temp_data += "0";
      Temp_data += "0";
      Temp_data += "0";
    }
    Temp_data += BMP_Temp ;
    String Pressure_data = "";
    Pressure_data += "1";
    if  (BMP_Pressure < 10000 && BMP_Pressure > 999) {
      Pressure_data += "0";
    }
    if  (BMP_Pressure < 1000 && BMP_Pressure > 99) {
      Pressure_data += "0";
      Pressure_data += "0";
    }
    if  (BMP_Pressure < 100 && BMP_Pressure > 9) {
      Pressure_data += "0";
      Pressure_data += "0";
      Pressure_data += "0";
    }
    if  (BMP_Pressure < 10) {
      Pressure_data += "0";
      Pressure_data += "0";
      Pressure_data += "0";
      Pressure_data += "0";
    }
    Pressure_data += BMP_Pressure ;
    String Altitude_data = "";
    if  (BMP_Altitude < 0) {
      Altitude_data += "0";
    }
    else {
      Altitude_data += "1";
    }
    BMP_Altitude = abs(BMP_Altitude);
    if  (BMP_Altitude < 10000 && BMP_Altitude > 999 ) {
      Altitude_data += "0";
    }
    if  (BMP_Altitude < 1000 && BMP_Altitude > 99) {
      Altitude_data += "0";
      Altitude_data += "0";
    }
    if  (BMP_Temp < 100 && BMP_Altitude > 9) {
      Altitude_data += "0";
      Altitude_data += "0";
      Altitude_data += "0";
    }
    if  (BMP_Temp < 10) {
      Altitude_data += "0";
      Altitude_data += "0";
      Altitude_data += "0";
      Altitude_data += "0";
    }
    Altitude_data += BMP_Altitude;
    String BMP_data = "";
    BMP_data += Temp_data;
    BMP_data += Pressure_data;
    BMP_data += Altitude_data;
    Serial.print("BMP sensor data string: ");
    Serial.println(BMP_data);
    serial3.print("BMP sensor data string: ");
    serial3.println(BMP_data);
  }
  else {
    Serial.println(F("BMP280: No data available."));
    serial3.println(F("BMP280: No data available."));
    String BMP_data = "000000000000000000";
    Serial.print("BMP sensor data string: ");
    Serial.println(BMP_data);
    serial3.print("BMP sensor data string: ");
    serial3.println(BMP_data);
  }
}

/*void Read_SCD4x() {
  SCD4x_init();
  if (SCD4x_sensor == 1) {
    // Read from SCD4x
    if (mySensor.readMeasurement()) {
      Serial.println(F("SCD4x Data:"));
      Serial.print(F("CO2 (ppm): "));
      Serial.println(mySensor.getCO2());
      Serial.print(F("Temperature (C): "));
      Serial.println(mySensor.getTemperature(), 1);
      Serial.print(F("Humidity (%RH): "));
      Serial.println(mySensor.getHumidity(), 1);
      serial3.println(F("SCD4x Data:"));
      serial3.print(F("CO2 (ppm): "));
      serial3.println(mySensor.getCO2());
      serial3.print(F("Temperature (C): "));
      serial3.println(mySensor.getTemperature(), 1);
      serial3.print(F("Humidity (%RH): "));
      serial3.println(mySensor.getHumidity(), 1);
    } else {
      Serial.println(F("SCD4x: No data available."));
      serial3.println(F("SCD4x: No data available."));
    }
  }
  }*/

void BMP280_init() {
  // Initialize BMP280 sensor
  Serial.print(F("Initializing BMP280..."));
  serial3.print(F("Initializing BMP280..."));
  if (!bmp.begin(BMP280_Address)) { // Try 0x77 if this fails
    Serial.println(F("BMP280 not detected. Please check wiring."));
    serial3.println(F("BMP280 not detected. Please check wiring."));
    BMP_280 = 0;
  }
  else {
    BMP_280 = 1;
  }
  if (BMP_280 == 1) {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X4, /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16, /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16, /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_125); /* Standby time. */
    Serial.println(F("OK."));
    serial3.println(F("OK."));
  }
}

/*void SCD4x_init() {
  // Initialize SCD4x sensor
  Serial.println(F("Initializing SCD4x..."));
  serial3.println(F("Initializing SCD4x..."));
  if (!mySensor.begin()) {
    Serial.println(F("SCD4x not detected. Please check wiring."));
    serial3.println(F("SCD4x not detected. Please check wiring."));
    SCD4x_sensor = 0;
  }
  else {
    SCD4x_sensor = 1;
    Serial.println(F("OK."));
    serial3.println(F("OK."));
  }
  }*/

void ReadUVSensors(byte ADC1_pin, byte ADC2_pin, int ADC_sampling) {
  int ADCvalue = 0;
  for (byte i = 0; i < ADC_sampling ; i++)
  {
    ADCvalue = ADCvalue + analogRead(ADC1_pin);// sensor on analog pin
  }
  ADCvalue = ADCvalue / ADC_sampling;    // average
  int voltage1 = (ADC_conversion12 * ADCvalue) / 1000.0;
  for (byte i = 0; i < ADC_sampling ; i++)
  {
    ADCvalue = ADCvalue + analogRead(ADC2_pin);// sensor on analog pin
  }
  ADCvalue = ADCvalue / ADC_sampling;    // average
  int voltage2 = (ADC_conversion12 * ADCvalue) / 1000.0;
  Serial.print(F("UV Voltage (Sensor 1): "));
  serial3.print(F("UV Voltage Index (Sensor 1): "));
  Serial.print(voltage1);
  serial3.print(voltage1);
  Serial.println(F(" mV"));
  serial3.println(F(" mV"));
  Serial.print(F("UV Voltage (Sensor 2): "));
  serial3.print(F("UV Voltage Index (Sensor 2): "));
  Serial.print(voltage2);
  serial3.print(voltage2);
  Serial.println(F(" mV"));
  serial3.println(F(" mV"));
  String UV_sensor1 = "";
  if  (voltage1 < 1000 && voltage1 > 99) {
    UV_sensor1 += "0";
  }
  if  (voltage1 < 100 && voltage1 > 9) {
    UV_sensor1 += "0";
    UV_sensor1 += "0";
  }
  if  (voltage1 < 10) {
    UV_sensor1 += "0";
    UV_sensor1 += "0";
    UV_sensor1 += "0";
  }
  UV_sensor1 += voltage1;
  String UV_sensor2 = "";
  if  (voltage2 < 1000 && voltage2 > 99) {
    UV_sensor2 += "0";
  }
  if  (voltage2 < 100 && voltage2 > 9) {
    UV_sensor2 += "0";
    UV_sensor2 += "0";
  }
  if  (voltage2 < 10) {
    UV_sensor2 += "0";
    UV_sensor2 += "0";
    UV_sensor2 += "0";
  }
  UV_sensor2 += voltage2;
  String UV_data = "";
  UV_data += UV_sensor1;
  UV_data += UV_sensor2;
  Serial.print("UV sensors data string: ");
  Serial.println(UV_data);
  serial3.print("UV sensors data string: ");
  serial3.println(UV_data);
}

void ENS_init() {
  Serial.print(F("Initializing ENS160..."));
  serial3.print(F("Initializing ENS160..."));
  if (NO_ERR != ENS160.begin()) {
    Serial.println("Communication with ENS160 failed, please check connection");
    serial3.println("Communication with ENS160 failed, please check connection");
    ENS_160 = 0;
  }
  else {
    Serial.println("OK!");
    serial3.println("OK!");
    ENS160.setPWRMode(ENS160_STANDARD_MODE);
    ENS160.setTempAndHum(/*temperature=*/25.0, /*humidity=*/50.0);
    ENS_160 = 1;
  }
}
void ENS_readData()
{
  ENS_init();
  if (ENS_160 == 1) {
    /**
       Get the sensor operating status
       Return value: 0-Normal operation,
               1-Warm-Up phase, first 3 minutes after power-on.
               2-Initial Start-Up phase, first full hour of operation after initial power-on. Only once in the sensor’s lifetime.
       note: Note that the status will only be stored in the non-volatile memory after an initial 24h of continuous
             operation. If unpowered before conclusion of said period, the ENS160 will resume "Initial Start-up" mode
             after re-powering.
    */
    Serial.println(F("ENS160 Data:"));
    serial3.println(F("ENS160 Data:"));
    uint8_t Status = ENS160.getENS160Status();
    Serial.print("Sensor operating status : ");
    Serial.println(Status);
    serial3.print("Sensor operating status : ");
    serial3.println(Status);

    /**
       Get the air quality index
       Return value: 1-Excellent, 2-Good, 3-Moderate, 4-Poor, 5-Unhealthy
    */
    uint8_t AQI = ENS160.getAQI();
    Serial.print("Air quality index : ");
    Serial.println(AQI);
    serial3.print("Air quality index : ");
    serial3.println(AQI);

    /**
       Get TVOC concentration
       Return value range: 0–65000, unit: ppb
    */
    uint16_t TVOC = ENS160.getTVOC();
    Serial.print("Concentration of total volatile organic compounds : ");
    Serial.print(TVOC);
    Serial.println(" ppb");
    serial3.print("Concentration of total volatile organic compounds : ");
    serial3.print(TVOC);
    serial3.println(" ppb");

    /**
       Get CO2 equivalent concentration calculated according to the detected data of VOCs and hydrogen (eCO2 – Equivalent CO2)
       Return value range: 400–65000, unit: ppm
       Five levels: Excellent(400 - 600), Good(600 - 800), Moderate(800 - 1000),
                     Poor(1000 - 1500), Unhealthy(> 1500)
    */
    uint16_t ECO2 = ENS160.getECO2();
    Serial.print("Carbon dioxide equivalent concentration : ");
    Serial.print(ECO2);
    Serial.println(" ppm");
    Serial.println();
    serial3.print("Carbon dioxide equivalent concentration : ");
    serial3.print(ECO2);
    serial3.println(" ppm");
    serial3.println();
    String ENS160_TVOC = "";
    if  (TVOC < 10000 && TVOC > 999) {
      ENS160_TVOC += "0";
    }
    if  (TVOC < 1000 && TVOC > 99) {
      ENS160_TVOC += "0";
      ENS160_TVOC += "0";
    }
    if  (TVOC < 100 && TVOC > 9) {
      ENS160_TVOC += "0";
      ENS160_TVOC += "0";
      ENS160_TVOC += "0";
    }
    if  (TVOC < 10) {
      ENS160_TVOC += "0";
      ENS160_TVOC += "0";
      ENS160_TVOC += "0";
      ENS160_TVOC += "0";
    }
    ENS160_TVOC += TVOC;
    String ENS160_ECO2 = "";
    if  (ECO2 < 100 && ECO2 > 9) {
      ENS160_ECO2 += "0";
    }
    if  (ECO2 < 10) {
      ENS160_ECO2 += "0";
      ENS160_ECO2 += "0";
    }
    ENS160_ECO2 += ECO2;
    String ENS160_data = "";
    ENS160_data += "0";
    ENS160_data += Status;
    ENS160_data += AQI;
    ENS160_data += ENS160_TVOC;
    ENS160_data += ENS160_ECO2;
    Serial.print("ENS160 sensor data string: ");
    Serial.println(ENS160_data);
    serial3.print("ENS sensor data string: ");
    serial3.println(ENS160_data);
  }
  else {
    Serial.println(F("ENS160: No data available."));
    serial3.println(F("ENS160: No data available."));
    String ENS160_data = "000000000000";
    Serial.print("ENS160 sensor data string: ");
    Serial.println(ENS160_data);
    serial3.print("ENS160 sensor data string: ");
    serial3.println(ENS160_data);
  }
}

void current_init() {
  Serial.print(F("Initializing INA219..."));
  serial3.print(F("Initializing INA219..."));
  if (! ina219.begin()) {
    Serial.println("Communication with INA219 failed, please check connection");
    serial3.println("Communication with INA219 failed, please check connection");
    INA_219 = 0;
  }
  else {
    Serial.println("OK!");
    serial3.println("OK!");
    INA_219 = 1;
    ina219.setCalibration_16V_400mA();
  }
}

void get_current() {
  current_init();
  if (INA_219 == 1) {
    int shuntvoltage = 0;
    int busvoltage = 0;
    int current_mA = 0;
    int loadvoltage = 0;
    int power_mW = 0;

    shuntvoltage = ina219.getShuntVoltage_mV();
    busvoltage = ina219.getBusVoltage_V() * 1000;
    current_mA = ina219.getCurrent_mA();
    power_mW = ina219.getPower_mW();
    loadvoltage = busvoltage + (shuntvoltage);
    Serial.println(F("INA219 Data:"));
    Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" mV");
    Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
    Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" mV");
    Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
    Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");
    Serial1.println("");
    serial3.println(F("INA219 Data:"));
    serial3.print("Bus Voltage:   "); serial3.print(busvoltage); serial3.println(" mV");
    serial3.print("Shunt Voltage: "); serial3.print(shuntvoltage); serial3.println(" mV");
    serial3.print("Load Voltage:  "); serial3.print(loadvoltage); serial3.println(" mV");
    serial3.print("Current:       "); serial3.print(current_mA); serial3.println(" mA");
    serial3.print("Power:         "); serial3.print(power_mW); serial3.println(" mW");
    serial3.println("");
    String INA219_load = "";
    if  (loadvoltage < 1000 && loadvoltage > 99) {
      INA219_load += "0";
    }
    if  (loadvoltage < 100 && loadvoltage > 9) {
      INA219_load += "0";
      INA219_load += "0";
    }
    if  (loadvoltage < 10) {
      INA219_load += "0";
      INA219_load += "0";
      INA219_load += "0";
    }
    INA219_load += loadvoltage;
    String INA219_power = "";
    if  (power_mW < 1000 && power_mW > 99) {
      INA219_power += "0";
    }
    if  (power_mW < 100 && power_mW > 9) {
      INA219_power += "0";
      INA219_power += "0";
    }
    if  (power_mW < 10) {
      INA219_power += "0";
      INA219_power += "0";
      INA219_power += "0";
    }
    INA219_power += power_mW;
    String INA219_current = "";
    INA219_current += "0";
    if  (current_mA < 100 && current_mA > 9) {
      INA219_current += "0";
    }
    if  (current_mA < 10) {
      INA219_current += "0";
      INA219_current += "0";
    }
    INA219_current += current_mA ;
    String INA219_data = "";
    INA219_data += INA219_load;
    INA219_data += INA219_current;
    INA219_data += INA219_power;
    Serial.print("INA219 sensor data string: ");
    Serial.println(INA219_data);
    serial3.print("INA219 sensor data string: ");
    serial3.println(INA219_data);
  }
  else {
    Serial.println(F("INA219: No data available."));
    serial3.println(F("INA219: No data available."));
    String INA219_data = "000000000000";
    Serial.print("INA219 sensor data string: ");
    Serial.println(INA219_data);
    serial3.print("INA219 sensor data string: ");
    serial3.println(INA219_data);
  }
}

void sensor_status() {
  serial3.print("Sensor status: ");
  serial3.print(BMP_280);
  serial3.print(ENS_160);
  serial3.println(INA_219);
  Serial.print("Sensor status: ");
  Serial.print(BMP_280);
  Serial.print(ENS_160);
  Serial.println(INA_219);
}
/*void ReadUVSensor(byte ADC_pin, byte Control_pin, float ADC_sampling, bool sensorNO) {
  float ADCvalue = 0;
  digitalWrite(Control_pin, sensorNO);
  for (byte i = 0; i < ADC_sampling ; i++)
  {
    ADCvalue = ADCvalue + analogRead(ADC_pin);// sensor on analog pin
  }
  ADCvalue = ADCvalue / ADC_sampling;    // average
  float voltage = ADC_conversion12 * ADCvalue;
  if (sensorNO == 0) {
    Serial.print(F("UV Index (Sensor 1): "));
    serial3.print(F("UV Index (Sensor 1): "));
  }
  else {
    Serial.print(F("UV Index (Sensor 2): "));
    serial3.print(F("UV Index (Sensor 2): "));
  }
  Serial.println(voltage);
  serial3.println(voltage);
  }*/
