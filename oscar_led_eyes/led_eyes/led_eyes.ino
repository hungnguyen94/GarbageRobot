#include <math.h>
#include <Wire.h>

#define clock 9
#define reset 8

//test pin
#define v5 13

#define resetDelay 5

typedef struct {
  int frameIndex;
  int frameDelay;
} frame;

int happy[8] = {
  B00000000,
  B00011000,
  B00111100,
  B01100110,
  B01000010,
  B01000010,
  B00000000,
  B00000000
};

int evilEyes[][8] =    
{   
  { // LeftEye1, 0  
    B00111100,
    B01000010,
    B10000001,
    B10011001,
    B10011001,
    B10000001,
    B01000010,
    B00111100
  },  
  { // RightEye1, 1 
    B00111100,
    B01000010,
    B10000001,
    B10011001,
    B10011001,
    B10000001,
    B01000010,
    B00111100
  },  
  { // LeftEye2, 2  
    B00000000,
    B00111100,
    B01000010,
    B01011010,
    B01011010,
    B01000010,
    B00111100,
    B00000000
  },  
  { // RightEye2, 3 
    B00000000,
    B00111100,
    B01000010,
    B01011010,
    B01011010,
    B01000010,
    B00111100,
    B00000000
  },  
  { // LeftEye3, 4  
    B00000000,
    B00111100,
    B00100100,
    B00110100,
    B00110100,
    B00100100,
    B00111100,
    B00000000
  },  
  { // RightEye3, 5 
    B00000000,
    B00111100,
    B00100100,
    B00110100,
    B00110100,
    B00100100,
    B00111100,
    B00000000
  },  
  { // LeftEye4, 6  
    B00011000,
    B00011000,
    B00011000,
    B00011000,
    B00011000,
    B00011000,
    B00011000,
    B00000000
  },  
  { // RightEye4, 7 
    B00000000,
    B00011000,
    B00011000,
    B00011000,
    B00011000,
    B00011000,
    B00011000,
    B00011000
  },  
  { // LeftEye5, 8  
    B01111110,
    B10000001,
    B10000001,
    B10011001,
    B10011001,
    B10000001,
    B10000010,
    B01111100
  },  
  { // RightEye5, 9 
    B01111100,
    B10000010,
    B10000001,
    B10011001,
    B10011001,
    B10000001,
    B10000001,
    B01111110
  },  
  { // LeftEye6, 10 
    B01111110,
    B10000001,
    B10000001,
    B10011001,
    B10011001,
    B10000010,
    B10000100,
    B01111000
  },  
  { // RightEye6, 11  
    B01111000,
    B10000100,
    B10000010,
    B10011001,
    B10011001,
    B10000001,
    B10000001,
    B01111110
  },  
  { // LeftEye7, 12 
    B01111110,
    B11000001,
    B10000001,
    B10011001,
    B10011010,
    B10000100,
    B10001000,
    B01110000
  },  
  { // RightEye7, 13  
    B01110000,
    B10001000,
    B10000100,
    B10011010,
    B10011001,
    B10000001,
    B11000001,
    B01111110
  },  
  { // LeftEye8, 14 
    B00111110,
    B01000001,
    B10000001,
    B10011001,
    B10011010,
    B10000100,
    B01001000,
    B00110000
  },  
  { // RightEye8, 15  
    B00110000,
    B01001000,
    B10000100,
    B10011010,
    B10011001,
    B10000001,
    B01000001,
    B00111110
  },  
  { // LeftEye9, 16 
    B01111110,
    B10000001,
    B10000001,
    B10011001,
    B10011001,
    B10000001,
    B10000001,
    B01111110
  },  
  { // RightEye9, 17  
    B01111110,
    B10000001,
    B10000001,
    B10011001,
    B10011001,
    B10000001,
    B10000001,
    B01111110
  },  
  { // LeftEye10, 18  
    B01111110,
    B10000001,
    B10000001,
    B10000001,
    B10011001,
    B10011001,
    B10000001,
    B01111110
  },  
  { // RightEye10, 19 
    B01111110,
    B10000001,
    B10000001,
    B10000001,
    B10011001,
    B10011001,
    B10000001,
    B01111110
  },  
  { // LeftEye11, 20  
    B01111110,
    B10000001,
    B10000001,
    B10000001,
    B10011001,
    B10011001,
    B10000001,
    B01111110
  },  
  { // RightEye11, 21 
    B01111110,
    B10000001,
    B10000001,
    B10000001,
    B10011001,
    B10011001,
    B10000001,
    B01111110
  },  
  { // LeftEye12, 22  
    B01111110,
    B10000001,
    B10000001,
    B10000001,
    B10000001,
    B10011001,
    B10011001,
    B01111110
  },  
  { // RightEye12, 23 
    B01111110,
    B10000001,
    B10000001,
    B10000001,
    B10000001,
    B10011001,
    B10011001,
    B01111110
  },  
  { // LeftEye13, 24  
    B01111110,
    B10000001,
    B10000001,
    B10000001,
    B10000001,
    B10000001,
    B10011001,
    B01111110
  },  
  { // RightEye13, 25 
    B01111110,
    B10000001,
    B10000001,
    B10000001,
    B10000001,
    B10000001,
    B10011001,
    B01111110
  },  
  { // LeftEye14, 26  
    B00000000,
    B00111100,
    B01000010,
    B01000010,
    B01000010,
    B01011010,
    B00111100,
    B00000000
  },  
  { // RightEye14, 27 
    B00000000,
    B00111100,
    B01000010,
    B01000010,
    B01000010,
    B01011010,
    B00111100,
    B00000000
  },  
  { // LeftEye15, 28  
    B00000000,
    B00111100,
    B00100100,
    B00100100,
    B00100100,
    B00111100,
    B00111100,
    B00000000
  },  
  { // RightEye15, 29 
    B00000000,
    B00111100,
    B00100100,
    B00100100,
    B00100100,
    B00111100,
    B00111100,
    B00000000
  },  
  { // LeftEye16, 30  
    B00011000,
    B00011000,
    B00011000,
    B00011000,
    B00011000,
    B00011000,
    B00011000,
    B00000000
  },  
  { // RightEye16, 31 
    B00000000,
    B00011000,
    B00011000,
    B00011000,
    B00011000,
    B00011000,
    B00011000,
    B00011000
  },  
  { // LeftEye17, 32  
    B00010000,
    B00010000,
    B00010000,
    B00010000,
    B00010000,
    B00010000,
    B00010000,
    B00000000
  },  
  { // RightEye17, 33 
    B00000000,
    B00010000,
    B00010000,
    B00010000,
    B00010000,
    B00010000,
    B00010000,
    B00010000
  },  
  { // LeftEye18, 34  
    B01111110,
    B10000001,
    B10000001,
    B10000001,
    B10000001,
    B10000001,
    B10001101,
    B01111110
  },  
  { // RightEye18, 35 
    B01111110,
    B10000001,
    B10000001,
    B10000001,
    B10000001,
    B10000001,
    B10001101,
    B01111110
  },  
  { // LeftEye19, 36  
    B01111110,
    B10000001,
    B10000001,
    B10000001,
    B10000001,
    B10000001,
    B10000111,
    B01111110
  },  
  { // RightEye19, 37 
    B01111110,
    B10000001,
    B10000001,
    B10000001,
    B10000001,
    B10000001,
    B10000111,
    B01111110
  },  
  { // LeftEye20, 38  
    B01111110,
    B10000001,
    B10000001,
    B10000001,
    B10000001,
    B10000011,
    B10000011,
    B01111110
  },  
  { // RightEye20, 39 
    B01111110,
    B10000001,
    B10000001,
    B10000001,
    B10000001,
    B10000011,
    B10000011,
    B01111110
  } 
};    

frame evilFrames[] = {
  {0, 1500}, {2, 5}, {4, 10}, {6, 10}, {32, 100}, {6, 10}, {4, 10}, {2, 5}, {0, 1500},
  {8, 10}, {10, 10}, {12, 10}, {14, 4000}, {12, 10}, {10, 10}, {8, 10}
};


int blinkImg[][8] = {    // Eye animation frames
  {
    B00111100,         // Fully open eye
    B01111110,
    B11111111,
    B11100111,
    B11100111,
    B11111111,
    B01111110,
    B00111100
  }
  ,
  {
    B00000000,
    B01111110,
    B11111111,
    B11100111,
    B11100111,
    B11111111,
    B01111110,
    B00111100
  }
  ,
  {
    B00000000,
    B00000000,
    B00111100,
    B11111111,
    B11100111,
    B11111111,
    B00111100,
    B00000000
  }
  ,
  {
    B00000000,
    B00000000,
    B00000000,
    B00111100,
    B11111111,
    B01111110,
    B00011000,
    B00000000
  }
  ,
  {
    B00000000,         // Fully closed eye
    B00000000,
    B00000000,
    B00000000,
    B10000001,
    B01111110,
    B00000000,
    B00000000
  }
};

frame blinkFrames[] = {
  {0, 2000},
  {1, 20},
  {2, 20},
  {3, 20},
  {4, 200},
  {3, 20},
  {2, 20},
  {1, 20},
  {0, 20}
};


int wink[8] = {
  B00000000,
  B00000000,
  B00000000,
  B01111110,
  B01111110,
  B00000000,
  B00000000,
  B00000000
};
int neutral[8] = {
  B00111100,
  B01111110,
  B11111111,
  B11111111,
  B11111111,
  B11111111,
  B01111110,
  B00111100
};


int all[8] = {
  B00000001,
  B00000011,
  B00000111,
  B00001111,
  B00011111,
  B00111111,
  B01111111,
  B11111111
};


int love[8] = {
  B00000000,
  B01100110,
  B11111111,
  B11111111,
  B01111110,
  B00111100,
  B00011000,
  B00000000
};


int die[8] = {
  B00000000,
  B01000010,
  B00111100,
  B00011000,
  B00011000,
  B00100100,
  B01000010,
  B00000000
};

void setup() {
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

  pinMode(clock, OUTPUT);
  pinMode(reset, OUTPUT);
  pinMode(v5, OUTPUT);
  digitalWrite(v5, HIGH);
  digitalWrite(reset, HIGH);
  delayMicroseconds(5);
  digitalWrite(reset, LOW);
  Serial.begin(9600);
}


int i = 0;

void loop() {
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
//      drawEyes(love);
        animateEyes2(blinkImg, blinkFrames, sizeof(blinkFrames)/sizeof(*blinkFrames));
//      animateEyes(evilEyes, evilEyesOrder, sizeof(evilEyesOrder) / sizeof(*evilEyesOrder), 5);
      break;
    case 4:
//      drawExpression(happy, 2000);
        animateEyes2(evilEyes, evilFrames, sizeof(evilFrames)/sizeof(*evilFrames));
//      animateEyes(blinkImg, blinkOrder, sizeof(blinkOrder) / sizeof(*blinkOrder), 70);
      break;
    default:
      // neutral expression
      drawEyes(neutral);
      //drawEyes(all);
      break;
  }
}

// Draw single frame for a spedified time. 
void drawExpression(int *expression, int delayTime) {
  long startTime = millis();
  while ((millis() - startTime) < delayTime) {
    drawEyes(expression);
  }
//  animateEyes(blinkImg, blinkOrder, sizeof(blinkOrder) / sizeof(*blinkOrder));
}

void animateEyes2(int (*image)[8], frame *frames, int frameLength) {
  for (int i = 0; i < frameLength; i++) {
    frame f = frames[i];
    drawExpression(image[f.frameIndex], f.frameDelay);
  }
}

// Delay in ms
void animateEyes(int (*img)[8], int *order, int orderLength, int delayTime) {
  for (int i = 0; i < orderLength; i++) {
    int eyeImgIndex = order[i];
    unsigned long startTime = millis();
    // Normal eyes
    if (i == 0) {
      delayTime = 2000;
    }
    while ((millis() - startTime) < delayTime) {
      drawEyes(img[eyeImgIndex]);
    }
  }
}

void drawEyes(int eyeShape[]) {
  reset_on();
  for (int i = 0; i < 8; i++) {
    trans(eyeShape[i]);
    delayMicroseconds(1500);
    if (i >= 7) {
      trans(eyeShape[0]);
    }
    else {
      trans(eyeShape[i + 1]);
    }
    clock_on();
  }
}


//void eyes(int eye){
//  trans(eye);
//  //delayMicroseconds(500);
//  delayMicroseconds(500);
//  clock_on();
//}

void trans(int expr) {
  Wire.beginTransmission(0x20);
  Wire.write(0x13);
  Wire.write(expr); // set all of port B to outputs
  Wire.endTransmission();

}

void reset_on() {
  digitalWrite(reset, HIGH);
  delayMicroseconds(5);
  digitalWrite(reset, LOW);
}

void clock_on() {
  digitalWrite(clock, HIGH);
  delayMicroseconds(5);
  digitalWrite(clock, LOW);
  //delayMicroseconds(500);
}










