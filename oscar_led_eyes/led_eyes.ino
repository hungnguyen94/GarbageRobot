#include <math.h>
#include <Wire.h>

#define clock 9
#define reset 8

//test pin
#define v5 13

#define resetDelay 5

int happy[8] = {
  B00000000,
  B00011000,
  B00111100,
  B01100110,
  B01000010,
  B01000010,
  B00000000,
  B00000000};

int blinkOrder[] = {
  0, 1, 2, 3, 4, 4, 4, 3, 2, 1, 0};
int blinkImg[][8] = {    // Eye animation frames
  { 
    B00111100,         // Fully open eye
    B01111110,
    B11111111,
    B11111111,
    B11100111,
    B11100111,
    B01111110,
    B00111100   }
  ,
  { 
    B00000000,
    B01111110,
    B11111111,
    B11111111,
    B11100111,
    B11100111,
    B01111110,
    B00111100   }
  ,
  { 
    B00000000,
    B00000000,
    B00111100,
    B11111111,
    B11111111,
    B11100111,
    B00111100,
    B00000000   }
  ,
  { 
    B00000000,
    B00000000,
    B00000000,
    B00111100,
    B11111111,
    B01111110,
    B00011000,
    B00000000   }
  ,
  { 
    B00000000,         // Fully closed eye
    B00000000,
    B00000000,
    B00000000,
    B10000001,
    B01111110,
    B00000000,
    B00000000   } 
}; 

int wink[8] = {
  B00000000,
  B00000000,
  B00000000,
  B01111110,
  B01111110,
  B00000000,
  B00000000,
  B00000000};
int neutral[8] = {
  B00111100, 
  B01111110, 
  B11111111, 
  B11111111, 
  B11111111, 
  B11111111, 
  B01111110, 
  B00111100};


int all[8] = {
  B00000001,
  B00000011,
  B00000111,
  B00001111,
  B00011111,
  B00111111,
  B01111111,
  B11111111};


int love[8] = {
  B00000000,
  B01100110,
  B11111111,
  B11111111,
  B01111110,
  B00111100,
  B00011000,
  B00000000};


int die[8] = {
  B00000000,
  B01000010,
  B00111100,
  B00011000,
  B00011000,
  B00100100,
  B01000010,
  B00000000};

void setup(){
  Wire.begin(); // wake up I2C bus
  // set I/O pins to outputs
  Wire.beginTransmission(0x20);
  Wire.write(0x00); // IODIRA register
  Wire.write(0x00); // set all of port A to outputs
  Wire.endTransmission();

  Wire.beginTransmission(0x20);
  Wire.write(0x01); // IODIRB register
  Wire.write(0x00); // set all of port B to outputs
  Wire.endTransmission();

  pinMode(clock,OUTPUT);
  pinMode(reset,OUTPUT);
  pinMode(v5,OUTPUT);
  digitalWrite(v5,HIGH);
  digitalWrite(reset,HIGH);
  delayMicroseconds(5);
  digitalWrite(reset,LOW);
  //Serial.begin(9600);
}


int i = 0;

void loop(){
  int var = 4;


  switch (var) {
  case 0:
    // be happy
    //    eyes(happy[i]);
    drawEyes(happy);
    break;
  case 1:
    // blink
    //    eyes(wink[0]);
    drawEyes(wink);
    break;
  case 2:
    // die
    //    eyes(die[0]);
    drawEyes(die);
    break;
  case 3: 
    drawEyes(love);
    break;
  case 4:
    animateEyes(blinkImg, blinkOrder, 13);
    break;
  default:
    // neutral expression
    drawEyes(neutral);
    //    if(i == 7){
    //      trans(happy[i]);
    //      delayMicroseconds(1500);
    //      trans(happy[0]);
    //      reset_on();
    //      i = 0;
    //    }
    //    else{
    //      trans(happy[i]);
    //      delayMicroseconds(1500);
    //      trans(happy[i+1]);
    //      clock_on();
    //      i++;
    //    }
    //drawEyes(all);
    break;
  }

}

void animateEyes(int img[][8], int order[], int length) {
  for(int i = 0; i < length; i++) {
    int eyeImgIndex = order[i];
    for(int j = 0; j < 4; j++) {
      drawEyes(img[eyeImgIndex]);
    }
  }
}

void drawEyes(int eyeShape[]) {
  reset_on();
  for (int i = 0; i < 8; i++) {
    trans(eyeShape[i]);
    delayMicroseconds(1500);
    if(i >= 7) {
      trans(eyeShape[0]);
    } 
    else {
      trans(eyeShape[i+1]);
    }
    clock_on();
  }
}

void eyes2(int eye){
  trans(eye);
  //delayMicroseconds(500);
  delayMicroseconds(500);
}

void eyes(int eye){
  trans(eye);
  //delayMicroseconds(500);
  delayMicroseconds(500);
  clock_on();
}

void trans(int expr){

  //  Wire.beginTransmission(0x20);
  //  Wire.write(0x12); // IODIRA register
  //  Wire.write(expr); // set all of port A to outputs
  //  Wire.endTransmission();

  Wire.beginTransmission(0x20);
  Wire.write(0x13); 
  Wire.write(expr); // set all of port B to outputs
  Wire.endTransmission();

}

void reset_on(){
  digitalWrite(reset,HIGH);
  delayMicroseconds(5);
  digitalWrite(reset,LOW);
}

void clock_on(){
  digitalWrite(clock,HIGH);
  delayMicroseconds(5);
  digitalWrite(clock,LOW);
  //delayMicroseconds(500);
}










