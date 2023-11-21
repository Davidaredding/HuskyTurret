#include <Arduino.h>
#include <Servo.h>//HUSKYLENS green line >> Pin 10; blue line >> Pin 11//HUSKYLENS green line >> Pin 10; blue line >> Pin 11plat
#include <HUSKYLENS.h>

/*macros & constants*/
#define bootstrapDelay delay(5000);

const uint8_t pan_servo_pin = D1;
const uint8_t tilt_servo_pin = D0;
 
#define pan(angle) servo_ctl.write(pan_servo_pin, angle);
#define tilt(angle) servo_ctl.write(tilt_servo_pin, angle);
/*******/

Servo servo_ctl = Servo();
HUSKYLENS husky;
EspSoftwareSerial::UART huskySerial;
int16_t pidError_x = 160;
int16_t pidError_y = 120;
int32_t pidOutput_x = 0;
int32_t pidOutput_y = 0;


void setup() {
  bootstrapDelay;
  Serial.begin(115200);
  Serial.println("Serial OK!");  
  pan(90);
  tilt(90);
  delay(15);

  Serial.println("Servo OK!");
  if(!Wire.begin()){
    Serial.println("Wire Error!");
    while(1); delay(1000);
  }  
  Serial.println("Wire OK!");
  if(!husky.begin(Wire)){
    Serial.println("Husky Error!");
    while(1); delay(1000);
  }
  Serial.println("Husky OK!");
  
 

  husky.writeAlgorithm(ALGORITHM_TAG_RECOGNITION);
  
  float kp = .08, ki = 0.01, kd = 0;


}

short xpos = 90;
short ypos = 90;
int16_t integral = 0;
const float dt = 0.05;  

float kp = .05, ki = 0.01, kd = 0;

short pidControl(short setPoint,short process, float Kp, float Ki=0, float Kd = 0){
  short error = setPoint - process;
  float porportional = Kp * error;
  integral += error * dt;
  float ic = Ki * integral;
  return (short)porportional + (short)ic;
  // return (short)porportional;
}

char buffer[6];
void loop() {
  if(Serial.available() > 0){
    Serial.println("Serial available" + String(Serial.available()));
    char c = Serial.read();
    Serial.read(buffer,6);
    if(c=='i') ki = atof(buffer);
    if(c=='d') kd = atof(buffer);
    if(c=='p') kp = atof(buffer);
    Serial.println(buffer);
    
  }
  
  husky.request();
  if(husky.available()){
    HUSKYLENSResult result = husky.read();
    pidError_x = 90-map(result.xCenter - 160, -160,160, -90,90);
    pidError_y = 90-map(result.yCenter - 120, -120,120, -90,90);
    Serial.println("X: "+String(pidError_x)+" Y: "+String(pidError_y));
  
    short adj_x = pidControl(90,pidError_x,.025,.005,.001);
    short adj_y = pidControl(90,pidError_y,.0125,.005,.001);
    xpos -= adj_x;
    ypos += adj_y;

    xpos = constrain(xpos,0,180);
    ypos = constrain(ypos,20,160);
    
    Serial.println(" adj x: " + String(adj_x));
    Serial.println(" adj y: " + String(adj_y));
    Serial.println(" - x: " + String(xpos));
    Serial.println(" - y: " + String(ypos));

    pan(xpos);
    tilt(ypos);
    delay(10);
   
  }

}

