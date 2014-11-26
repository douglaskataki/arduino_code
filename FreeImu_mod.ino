#define BAUDRATE 57600 //Define the BaudRate Here.
#define SIZE 20 //Size of the buffer

#include <AP_Math_freeimu.h>
#include <Filter.h>    // Filter library
#include <Butter.h>    // Butterworth filter

#include <I2Cdev.h>
#include <MPU60X0.h>
#include <AK8975.h>
#include <Adafruit_MPL115A2.h>
#include <Wire.h>
#include <SPI.h>

//#define DEBUG
#include "DebugUtils.h"
#include "CommunicationUtils.h"
#include "FreeIMU.h"
#include "FilteringScheme.h"
//#include "RunningAverage.h"

KalmanFilter kFilters[4];
int k_index = 3;

float q[4]={0,0,0,0};
float val[12];
int burst=122;
char cmd;

// Set the FreeIMU object
FreeIMU my3IMU = FreeIMU();

//The command from the PC

Adafruit_MPL115A2 mpl115a2; //Adafruit baro and temp sensor
float pressure,temperature; //variables for temperature and pressure

void setup() {
  Serial.begin(BAUDRATE);
  Wire.begin();
  mpl115a2.begin(); //Set adafruit I2C pressure and temp
  
  float qVal = 0.125; //Set Q Kalman Filter(process noise) value between 0 and 1
  float rVal = 32.; //Set K Kalman Filter (sensor noise)
  
  for(int i = 0; i <= k_index; i++) { //Initialize Kalman Filters for 10 neighbors
  //KalmanFilter(float q, float r, float p, float intial_value);
    kFilters[i].KalmanInit(qVal,rVal,5.0,0.5);
  }  
  my3IMU.init(true);
  //my3IMU.initGyros();
}

void loop(){
  if(Serial.available()){
    cmd = Serial.read();
    if(cmd=='z'){
      float val_array[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
      for(uint8_t i=0; i<burst; i++){
        my3IMU.getQ(q, val);
        //this is used for the processing 
        //val_array[15] = my3IMU.sampleFreq;
        
        //quaternions
        val_array[0] = (q[0]);
        val_array[1] = (q[1]);
        val_array[2] = (q[2]);
        val_array[3] = (q[3]);
        
        //acceleration
        val_array[4] = (val[0]);
        val_array[5] = (val[1]);
        val_array[6] = (val[2]);
        
        //gyros
        val_array[7] = (val[3] * M_PI/180);
        val_array[8] = (val[4] * M_PI/180);
        val_array[9] = (val[5] * M_PI/180);
        
        //magnectic
        val_array[10] = (val[6]);
        val_array[11] = (val[7]);
        val_array[12] = (val[8]);
        
        //pressure and temperature      
        mpl115a2.getPT(&pressure,&temperature);
        val_array[13] = temperature;  
        val_array[14] = pressure;
        
        //heading
        val_array[15] = val[9];
        
        //tmp = mpl115a2.getTemperature();
        //serialFloatPrint(tmp);
        //serialPrintFloatArr(q, 4);
        serialPrintFloatArr(val_array,16);
        Serial.print('\n');
      }
    }
  }
}

