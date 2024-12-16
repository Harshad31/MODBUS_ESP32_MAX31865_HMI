#include <Adafruit_MAX31865.h>
#include <WiFi.h>
#include <ModbusRTUSlave.h>

#define MAX31865_MISO 19
#define MAX31865_MOSI 23
#define MAX31865_CLK 18
#define MAX31865_CS 5

#define MAX485_DE 32
#define MAX485_RE 33
#define MAX485_RX 16
#define MAX485_TX 17

// MAX31865 temperature sensor setup
Adafruit_MAX31865 thermo = Adafruit_MAX31865(MAX31865_CS, MAX31865_MOSI, MAX31865_MISO, MAX31865_CLK);

// Modbus setup
ModbusRTUSlave modbus_slave(Serial1);
const uint8_t slaveID = 1;
const uint32_t baud = 9600;
uint16_t holdingRegisters[20] = {0};

// WiFi and NTP setup
const char* ssid = "VIRAT";
const char* password = "12345678";
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 19800;  // GMT offset for Asia/Kolkata (UTC+5:30)
const int daylightOffset_sec = 0;

// RTD calculation parameters
#define R0 100.0
#define Rref 430.0
#define A 3.9083e-3
#define B -5.775e-7
#define C -4.183e-12


#define totalReadingsMAX31865 2
double temperatureReadingsMAX31865[totalReadingsMAX31865] = {0};

// Timing variables
unsigned long totalReadingTime = 0;
unsigned long totalSleepTime = 0;
int readingCount = 0;
unsigned long lastReadingTime = 0;  // Store the last time the reading was taken
const long readingInterval = 500;   // Interval between readings (500ms)

// EMA variables
float previousEMA = 0.0;  // Store the previous EMA value
const float alpha = 0.1;  // Smoothing factor (alpha = 0.1)

void setup() {
  Serial.begin(115200);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("WiFi connected.");

  // Init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  // Check if time is synchronized
  struct tm timeinfo;
  for (int i = 0; i < 5; i++) {
    if (getLocalTime(&timeinfo)) {
      Serial.println("Time synchronized");
      break;
    } else {
      Serial.println("Failed to get time, retrying...");
      delay(1000); // Retry after 1 second
    }
  }

  // Setup MAX31865
  thermo.begin(MAX31865_3WIRE);

  // Setup Modbus RTU on Serial1 (TX=17, RX=16)
  pinMode(MAX485_DE, OUTPUT);
  pinMode(MAX485_RE, OUTPUT);
  digitalWrite(MAX485_DE, LOW);  // Set DE low (Receiver mode)
  digitalWrite(MAX485_RE, HIGH); // Set RE high (Receiver mode)
  Serial1.begin(baud, SERIAL_8N1, MAX485_RX, MAX485_TX);

  // Configure Modbus Slave
  modbus_slave.configureHoldingRegisters(holdingRegisters, 20);
  modbus_slave.begin(slaveID, baud, SERIAL_8N1);

  Serial.println("MAX31865 Temperature Sensor Initialized");
}

void loop() {
  unsigned long startTime = millis();

  // Get current time
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    unsigned long ms = millis() % 1000;  // Calculate the milliseconds
    char formattedTime[20];
    snprintf(formattedTime, sizeof(formattedTime), "%02d:%02d:%02d:%03lu", 
             timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, ms);

    Serial.print(formattedTime); // Print the formatted time
    Serial.print(" , ");

    // MAX31865 Sensor Reading (multiple readings for smoothing)
    for (int i = 0; i < totalReadingsMAX31865; i++) {
      uint16_t adcCode = thermo.readRTD();
      float Rt = (adcCode * Rref) / 32768.0;
      temperatureReadingsMAX31865[i] = Rt;
    }

    // Calculate average temperature
    double tempSumMax31865 = 0;
    for (int i = 0; i < totalReadingsMAX31865; i++) {
      tempSumMax31865 += temperatureReadingsMAX31865[i];
    }
    float averagedRtMax31865 = tempSumMax31865 / totalReadingsMAX31865;
    float temperatureMax31865 = calculateTemperature(averagedRtMax31865);

    // Apply Exponential Moving Average (EMA)
    if (readingCount == 0) {
      previousEMA = temperatureMax31865;  // Initialize with the first reading
    } else {
      previousEMA = alpha * temperatureMax31865 + (1 - alpha) * previousEMA;  // EMA calculation
    }

    // Update Modbus holding register with temperature (scaled by 10)
    holdingRegisters[0] = (uint16_t)(previousEMA * 10);  // Store the temperature in register

    // Set to transmit mode for Modbus communication
    transmitMode();

    // Poll Modbus slave for communication
    modbus_slave.poll();

    // Set back to receive mode after communication
    receiveMode();

    // Print smoothed temperature (EMA) on Serial Monitor
    Serial.print("TEMPERATURE, ");
    Serial.print(previousEMA, 2);  // Print temperature with 2 decimal places
    Serial.println("°C");

    // End of sensor reading
    unsigned long endTime = millis();
    unsigned long timeTakenForReadings = endTime - startTime;
    totalReadingTime += timeTakenForReadings;
    readingCount++;

    // Calculate remaining sleep time to maintain 500ms interval
    unsigned long remainingSleepTime = readingInterval - timeTakenForReadings;
    if (remainingSleepTime > 0) {
      delay(remainingSleepTime);
      totalSleepTime += remainingSleepTime;
    }
  } else {
    Serial.println("Failed to obtain time");
  }
}

// Function to calculate temperature from RTD resistance (using Callendar-Van Dusen equation)
float calculateTemperature(float Rt) {
  float t;

  if (Rt >= R0) {
    t = (-A + sqrt(A * A - 4 * B * (1 - Rt / R0))) / (2 * B);
  } else {
    float tolerance = 0.001;
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
      Serial.println("Warning: Temperature calculation failed to converge.");
    }
  }

  return t;
}

// Function to set MAX485 to transmit mode (for Modbus communication)
void transmitMode() {
  digitalWrite(MAX485_DE, HIGH);  // Enable Driver (Transmit mode)
  digitalWrite(MAX485_RE, HIGH);  // Disable Receiver
}

// Function to set MAX485 to receive mode (after Modbus communication)
void receiveMode() {
  digitalWrite(MAX485_DE, LOW);   // Disable Driver (Receive mode)
  digitalWrite(MAX485_RE, LOW);   // Enable Receiver
}
