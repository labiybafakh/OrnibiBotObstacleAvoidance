#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP280.h>
#include "credentials.hpp"
#include "messaging.hpp"
#include <HardwareSerial.h>


HardwareSerial SerialPort(0); // use UART1


const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;
const int serverPort = UDP_PORT;
const char* serverIP = SERVER_IP;

float knownAltitude = 105.0;
float localSeaLevelPressure;

WiFiUDP udp;

volatile ornibibot_data ornibibot_data_;

SemaphoreHandle_t dataAccessMutex;

#define BNO055_ADDRESS (0x28)
#define BMP280_ADDRESS (0x77) // This is the I2C address for the BMP280 on the BFF board

Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDRESS);
Adafruit_BMP280 bmp;

volatile int8_t turning_angle=0;
volatile char command_fly;

void calibrateAltitude(float knownAltitude) {
    float pressure = bmp.readPressure() / 100.0F; // Get current pressure in hPa
    localSeaLevelPressure = pressure / pow(1.0 - (knownAltitude / 44330.0), 5.255);

}

void wifiTask(void *pvParameters) {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    // Serial.println("Connecting to WiFi...");
  }
  // Serial.println("Connected to WiFi");
  // Serial.println(WiFi.localIP());
  udp.begin(UDP_PORT);
  vTaskDelete(NULL);
}

void sendDataTask(void *pvParameters) {
  const int packetSize = 1; // We're expecting a single character
  uint8_t incomingPacket[packetSize + 1]; // +1 for null terminator

  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(20); // 1 second interval
  while (1) {
    int packetSized = udp.parsePacket();
    if (packetSize == 1){
      int len = udp.read(incomingPacket, packetSize);
      incomingPacket[len] = 0; // Null-terminate the string
      if(WiFi.status() != WL_DISCONNECTED){
        SerialPort.print(incomingPacket[0]);
      }
      else{
        uint8_t stop = 0;
        SerialPort.print(stop);
      }
    }

    noInterrupts();
    udp.beginPacket(serverIP, serverPort);
    udp.write((uint8_t*)&ornibibot_data_, sizeof(ornibibot_data_));
    udp.endPacket();
    interrupts();

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void serialReadTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(5); // 1 second interval
  while (1) {
    if(SerialPort.available()) turning_angle = SerialPort.read();
      ornibibot_data_.turning = static_cast<int>(turning_angle);
      Serial.println(ornibibot_data_.turning);
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void sensorReadTask(void *pvParameters) {
  sensors_event_t orientationData, linearAccelData, gravityData;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(20); // 100Hz update rate
  bool readSuccess = true;

  while (1) {
  readSuccess = true;
      // Read BNO055 data
  if (!bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER) ||
      !bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL) ||
      !bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY)) {
      // Serial.println("Failed to read from BNO055");
      readSuccess = false;
  }

    // Read BMP280 data
  float temperature = bmp.readTemperature();
  float pressure = bmp.readPressure();
  float altitude = bmp.readAltitude(localSeaLevelPressure);

  if (isnan(temperature) || isnan(pressure) || isnan(altitude)) {
          // Serial.println("Failed to read from BMP280");
          readSuccess = false;
      }

      if (readSuccess) {
          // xSemaphoreTake(dataAccessMutex, portMAX_DELAY);
          ornibibot_data_.timestamp = millis();
          ornibibot_data_.roll = orientationData.orientation.y;
          ornibibot_data_.pitch = orientationData.orientation.z;
          ornibibot_data_.yaw = orientationData.orientation.x;
          ornibibot_data_.linear_accel_x = linearAccelData.acceleration.x;
          ornibibot_data_.linear_accel_y = linearAccelData.acceleration.y;
          ornibibot_data_.linear_accel_z = linearAccelData.acceleration.z;
          // ornibibot_data_.turning = turning_angle;
          ornibibot_data_.temperature = temperature;
          ornibibot_data_.pressure = pressure / 100.0F; // Convert Pa to hPa
          ornibibot_data_.altitude = altitude;

          // Serial.println(ornibibot_data_.altitude);
          // xSemaphoreGive(dataAccessMutex);
      } else {
          // If read failed, wait a bit before trying again
          vTaskDelay(pdMS_TO_TICKS(20));
      }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void setup() {
  SerialPort.begin(115200);
  ornibibot_data_.turning = 0;

  // SerialPort.setRxIntMsk(true); // Enable the receive interrupt

  Wire.begin();
  Wire.setClock(100000); // Set I2C clock to 100kHz

  delay(1000);

  if (!bno.begin()) {
    // Serial.println("Failed to initialize BNO055!");
    while (1) {
      delay(10);
    }
  }
  // Serial.println("BNO055 initialized successfully");

  // Initialize BMP280
  if (!bmp.begin(BMP280_ADDRESS)) {
    // Serial.println("Failed to initialize BMP280!");
    while (1) {
      delay(10);
    }
  }
  // Serial.println("BMP280 initialized successfully");

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);

  bno.setExtCrystalUse(true);

  calibrateAltitude(knownAltitude);


  dataAccessMutex = xSemaphoreCreateMutex();

  xTaskCreate(wifiTask, "WiFi Connection", 4096, NULL, 1, NULL);
  xTaskCreate(sendDataTask, "Send UDP Data", 4096, NULL, 1, NULL);
  // xTaskCreate(serialReadTask, "Serial Read", 2048, NULL, 1, NULL);
  xTaskCreate(sensorReadTask, "Sensor Read", 4096, NULL, 2, NULL); // Higher priority for sensor reading
  // xTaskCreate(receiveUdpTask, "Receive UDP Data", 4096, NULL, 1, NULL);

}

void loop() {
  // Empty. Tasks are handled by FreeRTOS

}

// void serialEvent1(){
//   while(SerialPort.available()){
//     int8_t data_ = SerialPort.read();

//     // ornibibot_data_.turning = data_;
//   }
// }