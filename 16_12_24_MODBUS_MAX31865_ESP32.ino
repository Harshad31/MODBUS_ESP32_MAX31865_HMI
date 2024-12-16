#include <ModbusRTUSlave.h>
#include <Adafruit_MAX31865.h>

#define MAX31865_MISO 19
#define MAX31865_MOSI 23
#define MAX31865_CLK 18
#define MAX31865_CS 5


#define MAX485_DE 32 
#define MAX485_RE 33 
#define MAX485_RX 16 
#define MAX485_TX 17 

ModbusRTUSlave modbus_slave(Serial1); 
const uint8_t slaveID = 1;
const uint32_t baud = 9600;
uint16_t holdingRegisters[20] = {0};

Adafruit_MAX31865 thermo = Adafruit_MAX31865(MAX31865_CS, MAX31865_MOSI, MAX31865_MISO, MAX31865_CLK);

#define R0 100.0
#define A 3.9083e-3
#define B -5.775e-7
#define C -4.183e-12

void setup() {

  Serial.begin(115200);


  pinMode(MAX485_DE, OUTPUT);
  pinMode(MAX485_RE, OUTPUT);
  digitalWrite(MAX485_DE, LOW);  // Set DE low (Receiver mode)
  digitalWrite(MAX485_RE, HIGH); // Set RE high (Receiver mode)

  // Initialize Modbus Slave on Serial1 (TX=17, RX=16)
  Serial1.begin(baud, SERIAL_8N1, MAX485_RX, MAX485_TX);

  modbus_slave.configureHoldingRegisters(holdingRegisters, 20);
  modbus_slave.begin(slaveID, baud, SERIAL_8N1);

  thermo.begin(MAX31865_3WIRE);
}

void loop() {
  uint16_t rtd = thermo.readRTD();
  float Rt = (rtd / 32768.0 * 430.0);
  float temperature = calculateTemperature(Rt);

  holdingRegisters[0] = (uint16_t)(temperature * 10);  

  // Set to transmit mode for Modbus communication
  transmitMode();

  // Poll Modbus slave
  modbus_slave.poll();

  // Set back to receive mode after communication
  receiveMode();

  // Debug output
  Serial.print("Temperature: ");
  Serial.print(temperature, 2);
  Serial.println(" °C");

  delay(500); 
}

// Function to set MAX485 to transmit mode
void transmitMode() {
  digitalWrite(MAX485_DE, HIGH);  // Enable Driver (Transmit mode)
  digitalWrite(MAX485_RE, HIGH);  // Disable Receiver
}

// Function to set MAX485 to receive mode
void receiveMode() {
  digitalWrite(MAX485_DE, LOW);   // Disable Driver (Receive mode)
  digitalWrite(MAX485_RE, LOW);   // Enable Receiver
}

// Function to calculate the temperature from RTD
float calculateTemperature(float Rt) {
  float t;

  if (Rt >= R0) {
    // For temperatures above 0°C
    t = (-A + sqrt(A * A - 4 * B * (1 - Rt / R0))) / (2 * B);
  } else {
    // For temperatures below 0°C
    float tolerance = 0.001;  // Convergence tolerance
    int maxIterations = 100;
    int iteration = 0;
    float diff;
    t = Rt;

    do {
      float fValue = R0 * (1 + A * t + B * t * t + C * (t - 100) * t * t * t) - Rt;
      float fDerivative = R0 * (A + 2 * B * t + 3 * C * (t - 100) * t * t + C * t * t * t);
      float nextT = t - fValue / fDerivative;
      diff = abs(nextT - t);
      t = nextT;
      iteration++;
    } while (diff > tolerance && iteration < maxIterations);

    if (iteration == maxIterations) {
      Serial.println("Max iterations reached. Convergence not achieved.");
    }
  }

  // Check for faults in the MAX31865 sensor
  uint8_t fault = thermo.readFault();
  if (fault) {
    Serial.print("Fault 0x"); 
    Serial.println(fault, HEX);
    if (fault & MAX31865_FAULT_HIGHTHRESH) {
      Serial.println("RTD High Threshold");
    }
    if (fault & MAX31865_FAULT_LOWTHRESH) {
      Serial.println("RTD Low Threshold");
    }
    if (fault & MAX31865_FAULT_REFINLOW) {
      Serial.println("REFIN- > 0.85 x Bias");
    }
    if (fault & MAX31865_FAULT_REFINHIGH) {
      Serial.println("REFIN- < 0.85 x Bias - FORCE- open");
    }
    if (fault & MAX31865_FAULT_RTDINLOW) {
      Serial.println("RTDIN- < 0.85 x Bias - FORCE- open");
    }
    if (fault & MAX31865_FAULT_OVUV) {
      Serial.println("Under/Over voltage");
    }
    thermo.clearFault();
  }

  return t;
}
