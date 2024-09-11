#include <Wire.h>
#include <ArduinoEigenDense.h>
#include <SparkFun_VL53L5CX_Library.h> //http://librarymanager/All#SparkFun_VL53L5CX
#include <iostream>
#include "string.h"

#include <memory>
#include <unistd.h>
#include <math.h>
#include <atomic>
#include <cstdint>
#include <vector>

#define IMAGE_WIDTH 8  // Assuming 8x8 ToF sensor array
#define SAFETY_DISTANCE 1000  // in mm
#define TURN_ANGLE 30  // degrees to turn when obstacle detected
#define INT_TOF 13

IntervalTimer tof_timer;
IntervalTimer obstacle_avoidance_timer;
IntervalTimer interpolation_pattern_timer;
IntervalTimer sbus_update_timer;

int i=0;
using namespace Eigen;

SparkFun_VL53L5CX myImager;
VL53L5CX_ResultsData measurementData; 

int imageResolution = 0; 
int imageWidth = 0; 

volatile uint8_t counter_tof=0;
// Global variables
volatile bool newDataReady = false;
Matrix<uint16_t, IMAGE_WIDTH, IMAGE_WIDTH> distanceMatrix;

// struct flapping{
//   uint8_t amplitude;
//   uint8_t offset;
//   uint8_t roll;
//   double freq;
// };

struct ornibibot_param{
  std::atomic<float> frequency;
  std::atomic<int8_t> roll;
  std::atomic<int8_t> pitch;
};

std::atomic<std::int_fast8_t> wing_position;
int amplitude = 70;
uint16_t timing = 0;

std::vector<uint16_t> corner_tof(4,0);
std::vector<uint16_t> last_corner_tof(64,0);
std::vector<uint16_t> sensor_time(64,0);

std::atomic<bool> out_of_range;
std::atomic<uint8_t> counter_out_of_range;


ornibibot_param ornibibot_parameter;

uint16_t degToSignal(int8_t pos){
    //Rotate Servo from -60 to 60 Degrees
    //Mid Servo using SBUS is 1023
    //Upstroke<1023 - Downstroke>1023
    if(pos>90)         pos=90;
    else if(pos<-90)   pos=-90;

    return (uint16_t)(1023 - (-pos*11.36)); //reversed to adjust upstroke-downstroke
}

uint16_t degToSignalTail(int8_t pos){
    //Rotate Servo from -60 to 60 Degrees
    //Mid Servo using SBUS is 1023
    //Upstroke<1023 - Downstroke>1023
    if(pos>80)         pos=80;
    else if(pos<-80)   pos=-80;

    return (uint16_t)(1023 - (-pos*12.79)); //reversed to adjust upstroke-downstroke
}

volatile bool dataReady = false;

void setPosition(uint16_t pos_left, uint16_t pos_right, uint16_t pos_tail_left, uint16_t pos_tail_right){
    const size_t SBUS_BUFFER = 25;
    uint8_t packet_sbus[SBUS_BUFFER];
    memset(packet_sbus, 0x00, SBUS_BUFFER);

    uint16_t zeroing = 0;

    packet_sbus[0] = 0x0f;
    packet_sbus[1] = (uint8_t)(pos_left & 0xff);
    packet_sbus[2] = (uint8_t)((pos_left >> 8) & 0x07 ) | ((pos_right  << 3 ) );
    packet_sbus[3] = (uint8_t)((pos_right >> 5) & 0x3f ) | (pos_tail_left  << 6);
    packet_sbus[4] = (uint8_t)((pos_tail_left >> 2) & 0xFF);
    packet_sbus[5] = (uint8_t)((pos_tail_left >> 10) & 0x01) | (pos_tail_right << 1);
    packet_sbus[6] = (uint8_t)(pos_tail_right >> 7) & 0x0f | (zeroing << 4);

    // // Fill the rest of the packet with zeros (assuming no other channels are used)
    // for (int i = 5; i < 23; i++) {
    //     packet_sbus[i] = 0x00;
    // }

    // Stop byte(s)
    packet_sbus[23] = 0x00;
    packet_sbus[24] = 0x00;

    Serial3.write(packet_sbus, sizeof(packet_sbus));
}

void interpolationPattern(){
      uint16_t periode_ = 1000 / ornibibot_parameter.frequency;
      wing_position = (amplitude * sin((2 * M_PI * (double)timing) / periode_));

      if(wing_position > 0) wing_position = amplitude;
      else wing_position = amplitude * -1;

      if (timing < periode_) {
          timing++;
      } else {
          timing = 0;
      }
}

void motorUpdate(){
    const int adjustment = 8;
    const int minimum_pitch_tail = 25;
    
    if(ornibibot_parameter.frequency < 0.5){

        // if(ornibibot_parameter.pitch < -20) ornibibot_parameter.pitch = -20;

        // if(ornibibot_parameter.roll > 25) ornibibot_parameter.roll = 25;
        // else if(ornibibot_parameter.roll < -25) ornibibot_parameter.roll = -25;

        int8_t left_tail = minimum_pitch_tail + ornibibot_parameter.pitch;
        int8_t right_tail = minimum_pitch_tail + ornibibot_parameter.pitch;

        setPosition(
          degToSignal(25),
          degToSignal((25+adjustment)*-1),
          degToSignalTail(left_tail),
          degToSignalTail(right_tail*-1)
        );
    }

    else{
        // if(ornibibot_parameter.pitch < -0) ornibibot_parameter.pitch = -20;

        // if(ornibibot_parameter.roll > 25) ornibibot_parameter.roll = 25;
        // else if(ornibibot_parameter.roll < -25) ornibibot_parameter.roll = -25;

        int8_t left_tail = minimum_pitch_tail + ornibibot_parameter.pitch;
        int8_t right_tail = minimum_pitch_tail + ornibibot_parameter.pitch;
        
        // if(payload==100){
        //   setPosition(
        //   degToSignal(wing_position),
        //   degToSignal((wing_position+adjustment)*-1),
        //   degToSignalTail(left_tail),
        //   degToSignalTail(right_tail*-1)
        // );
        // }
        // else{
          setPosition(
          degToSignal(wing_position+ornibibot_parameter.roll),
          degToSignal((wing_position+adjustment-ornibibot_parameter.roll)*-1),
          degToSignalTail(left_tail),
          degToSignalTail(right_tail*-1)
        );
        // }
    }
}

void obstacleAvoidance(){
  if(newDataReady){
    Matrix<bool, IMAGE_WIDTH, IMAGE_WIDTH> obstacleMatrix = (distanceMatrix.array() < SAFETY_DISTANCE).cast<bool>();
    
    bool obstacleDetected = obstacleMatrix.any();
    
    if (obstacleDetected) {
      // Serial.print("STOP");
      
      Vector2i obstacleCounts;
      obstacleCounts << obstacleMatrix.leftCols(IMAGE_WIDTH / 2).count(),
                        obstacleMatrix.rightCols(IMAGE_WIDTH / 2).count();
      
      if (obstacleCounts(0) > obstacleCounts(1)){
        // String data_serial = "Turn right  " + (String)TURN_ANGLE + (String)millis();
        // Serial.println(data_serial);
      } else {
        // String data_serial = "Turn left  " + (String)TURN_ANGLE +  (String)millis();
        // Serial.println(data_serial);
      }
    }
    else{
      // Serial.println("FORWARD");
    }
  }
}


void tofHandler(){
  Serial5.write(0x01);
}

void interruptRoutine()
{
  newDataReady = true;
}

void setup()
{
  Serial.begin(460800);
  Serial3.begin(100000,SERIAL_8E2);
  Serial5.begin(115200);
  delay(1000);

  Wire.begin(); //This resets to 100kHz I2C
  Wire.setClock(400000); //Sensor has max I2C freq of 400kHz 
  
  while(myImager.begin(0x29) != true)
  {
    Serial.println(F("Sensor not found - check your wiring. Freezing"));
    delay(1000);
  }
  Serial.println("OK");
  
  myImager.setResolution(8*8); //Enable all 64 pads
  myImager.setRangingFrequency(15);
  myImager.setRangingMode(SF_VL53L5CX_RANGING_MODE::CONTINUOUS); 

  imageResolution = myImager.getResolution(); 
  imageWidth = sqrt(imageResolution); 
  
  if(!myImager.startRanging()){
    Serial.println("Failed to start ranging");
  }
  Serial.println("Started ranging");

  attachInterrupt(digitalPinToInterrupt(INT_TOF), interruptRoutine, FALLING);


  interpolation_pattern_timer.begin(interpolationPattern, 1000);
  interpolation_pattern_timer.priority(0);
  sbus_update_timer.begin(motorUpdate, 5000);
  sbus_update_timer.priority(1);
  tof_timer.begin(tofHandler, 5000);
  tof_timer.priority(2);
  obstacle_avoidance_timer.begin(obstacleAvoidance, 5000);
  obstacle_avoidance_timer.priority(3);

}

void loop()
{
  ornibibot_parameter.frequency = 5;

  i=0;

      //Poll sensor for new data
  if (newDataReady == true)
  {
    counter_tof = 0;
    newDataReady = false;
    if(myImager.getRangingData(&measurementData))

      corner_tof[0] = measurementData.distance_mm[0];
      corner_tof[1] = measurementData.distance_mm[7];
      corner_tof[2] = measurementData.distance_mm[56];
      corner_tof[3] = measurementData.distance_mm[63];

      out_of_range = false;

      for(int i=0; i<corner_tof.size(); i++){
          if(corner_tof[i] == last_corner_tof[i]) {out_of_range=true; break;}
      }

      if(out_of_range) counter_out_of_range++;
      else counter_out_of_range = 0;
      
      // String data = (String) counter_out_of_range + "\t" + (String)corner_tof[0] + "\t" + (String)corner_tof[1] + "\t" + (String)corner_tof[2] + "\t" + (String)corner_tof[3];
      // Serial.println(data);
      // if(!out_of_range){
      //   for (int y = 0; y < IMAGE_WIDTH; y++) {
      //     for (int x = 0; x < IMAGE_WIDTH; x++) {
      //       distanceMatrix(y, IMAGE_WIDTH - 1 - x) = measurementData.distance_mm[y * IMAGE_WIDTH + x];
      //     }
      //   }
      // }

      for (int y = 0 ; y <= imageWidth * (imageWidth - 1) ; y += imageWidth)
      {
        for (int x = imageWidth - 1 ; x >= 0 ; x--)
        {
          Serial.print("\t");
          if(last_corner_tof[x + y] != measurementData.distance_mm[x + y]){
            sensor_time[x + y] = 0;
          }
          else{
            sensor_time[x + y] ++;
          }

          if(sensor_time[x + y] < 15)
            Serial.print(measurementData.distance_mm[x + y]);
          else
            Serial.print("XX");

          last_corner_tof[x + y] = measurementData.distance_mm[x + y];
          //i++;
        }
        Serial.println();
      }
      Serial.println();

      // for(int i=0; i < last_corner_tof.size(); i++){
      //   last_corner_tof[i] = corner_tof[i];
      // }

    }

    // Serial.println(counter_tof);
}
