#include <math.h>
#include <Wire.h>

#define clk 9
#define reset 8

#define pin0 10
#define pin1 11
#define pin2 12
#define pin3 13

#define resetDelay 5

typedef struct {
  int frameLeftIndex;
  int frameRightIndex;
  int frameDelay;
} frame;

int blinkEyes[][8] =    
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
    B00000000,
    B01111110,
    B01000010,
    B01011010,
    B01111110,
    B00000000,
    B00000000
  },  
  { // RightEye3, 5   
    B00000000,
    B00000000,
    B01111110,
    B01000010,
    B01011010,
    B01111110,
    B00000000,
    B00000000
  },  
  { // LeftEye4, 6  
    B00000000,
    B00000000,
    B00000000,
    B01111110,
    B01111110,
    B00000000,
    B00000000,
    B00000000
  },  
  { // RightEye4, 7  
    B00000000,
    B00000000,
    B00000000,
    B01111110,
    B01111110,
    B00000000,
    B00000000,
    B00000000
  },
  { // LeftEye5, 8
    B00000000,
    B00000000,
    B00000000,
    B00000000,
    B01111110,
    B00000000,
    B00000000,
    B00000000
  },
  { // RightEye5, 9
    B00000000,
    B00000000,
    B00000000,
    B00000000,
    B01111110,
    B00000000,
    B00000000,
    B00000000
  }
};
  
int evilEyes[][8]  { 
  {// LeftEye5, 0  
    B00111100,
    B01000010,
    B10000001,
    B10011001,
    B10011001,
    B10000001,
    B01000010,
    B00111100
  },  
  { // RightEye5, 1  
    B00111100,
    B01000010,
    B10000001,
    B10011001,
    B10011001,
    B10000001,
    B01000010,
    B00111100
  },  
  { // LeftEye6, 2  
    B00011100,
    B00100010,
    B01000001,
    B10011001,
    B10011001,
    B10000001,
    B01000010,
    B00111100
  },  
  { // RightEye6, 3 
    B01111000,
    B10000100,
    B10000010,
    B10011001,
    B10011001,
    B10000001,
    B01000010,
    B00111100
  },  
  { // LeftEye7, 4 
    B00001110,
    B00010001,
    B00100001,
    B01011001,
    B10011001,
    B10000001,
    B01000010,
    B00111100
  },  
  { // RightEye7, 5
    B01110000,
    B10001000,
    B10000100,
    B10011010,
    B10011001,
    B10000001,
    B01000010,
    B00111100
  },  
  { // LeftEye8, 6  
    B00001100,
    B00010010,
    B00100001,
    B01011001,
    B10011001,
    B10000001,
    B01000010,
    B00111100
  },  
  { // RightEye8, 7  
    B00110000,
    B01001000,
    B10000100,
    B10011010,
    B10011001,
    B10000001,
    B01000010,
    B00111100
  }
};

int lookEyes[][8] = {
    
  // Looking to the right
  
  { // LeftEye10, 0 
    B00111100,
    B01000010,
    B10000001,
    B10110001,
    B10110001,
    B10000001,
    B01000010,
    B00111100
  },
  { // LeftEye12, 1 
    B00111100,
    B01000010,
    B10000001,
    B11100001,
    B11100001,
    B10000001,
    B01000010,
    B00111100
  },
  { // LeftEye13, 2
    B00111100,
    B01000010,
    B10000001,
    B11000001,
    B11000001,
    B10000001,
    B01000010,
    B00111100
  },
  
  // Looking to the left
  
  { // LeftEye21, 3 
    B00111100,
    B01000010,
    B10000001,
    B10001101,
    B10001101,
    B10000001,
    B01000010,
    B01011100
  },
  { // LeftEye22, 4 
    B00111100,
    B01000010,
    B10000001,
    B10000111,
    B10000111,
    B10000001,
    B01000010,
    B01011100
  },
  { // LeftEye23, 5
    B00111100,
    B01000010,
    B10000001,
    B10000011,
    B10000011,
    B10000001,
    B01000010,
    B01011100
  },
  {//           , 6
    B00111100,
    B01000010,
    B10000001,
    B10011001,
    B10011001,
    B10000001,
    B01000010,
    B00111100
  }
};

int happyEyes[][8] = {
  // Looking happy

  { //happyEye1, 0  
    B00000000,
    B00111100,
    B01000010,
    B01011010,
    B01011010,
    B01000010,
    B00111100,
    B00000000
  },
  { //happyEye2, 1  
    B00000000,
    B00111100,
    B01011010,
    B01011010,
    B01000010,
    B01111110,
    B00000000,
    B00000000
  },
  { //happyEye3, 2  
    B00000000,
    B00111100,
    B01111110,
    B01000010,
    B01111110,
    B01000010,
    B00000000,
    B00000000
  },
  { //happyEye4, 3  
    B00000000,
    B00111100,
    B01111110,
    B01111110,
    B01000010,
    B01000010,
    B00000000,
    B00000000
  },
  { //happyEye5, 4
    B00000000,
    B00011000,
    B00111100,
    B01100110,
    B01000010,
    B01000010,
    B00000000,
    B00000000
  }
};

int deadEyes[][8] = {
  {// 0
    B01111110,
    B10000001,
    B10000001,
    B10011001,
    B10011001,
    B10000001,
    B10000001,
    B01111110
  },
  {// 1
    B10000001,
    B01111110,
    B01000010,
    B01011010,
    B01011010,
    B01000010,
    B01111110,
    B10000001
  },
  {// 2
    B10000001,
    B01000010,
    B00111100,
    B00111100,
    B00111100,
    B00111100,
    B01000010,
    B10000001
  },
  {// 3
    B10000001,
    B01000010,
    B00100100,
    B00011000,
    B00011000,
    B00100100,
    B01000010,
    B10000001
  }
};

  //////////////////////////////
  // All the animation frames //
  //////////////////////////////

frame blinkFrames[] = {
  {0, 1, 1500}, {2, 3, 5}, {4, 5, 10}, {6, 7, 10}, {8, 9, 100}, {6, 7, 10}, {4, 5, 10}, {2, 3, 5}, {0, 1, 1500}
};

frame evilBeginFrames[] = {
  {0, 1, 100}, {2, 3, 100}, {4, 5, 100}
};

frame evilEndFrames[] = {
  {4, 5, 100}, {2, 3, 100}, {0, 1, 100}
};

frame neutralFrames[] = {
  {0, 0, 2000}
};

frame happyBeginFrames[] = {
  {0, 0, 15}, {1, 1, 5}, {2, 2, 10}, {3, 3, 10}
};

frame happyEndFrames[] = {
  {3, 3, 10}, {2, 2, 10}, {1, 1, 5}, {0, 0, 15}
};

frame lookLeftBeginFrames[] = {
  {3, 3, 100}, {4, 4, 100}, {5, 5, 100}  
};

frame lookLeftEndFrames[] = {
  {5, 5, 100}, {4, 4, 100}, {3, 3, 100}  
};

frame lookRightBeginFrames[] = {
  {0, 0, 100}, {1, 1, 100}, {2, 2, 100}  
};

frame lookRightEndFrames[] = {
  {2, 2, 100}, {1, 1, 100}, {0, 0, 100}  
};

frame lookAroundFrames[] = {
  {3, 3, 100}, {4, 4, 100}, {5, 5, 100}, {4, 4, 100}, {3, 3, 100}, {6, 6, 100}, {0, 0, 100}, {1, 1, 100}, {2, 2, 100}, {1, 1, 100}, {0, 0, 100}
};

frame dieBeginFrames[] = {
  {0, 0, 50}, {1, 1, 50}, {2, 2, 50}, {3, 3, 50}
};

frame dieEndFrames[] = {
  {3, 3, 50}, {2, 2, 50}, {1, 1, 50}, {0, 0, 50}
};

int angryLeft[8] = { 
  // LeftEye8, 6  
  B00001100,
  B00010010,
  B00100001,
  B01011001,
  B10011001,
  B10000001,
  B01000010,
  B00111100
};

int angryRight[8] = { 
  // RightEye8, 7  
  B00110000,
  B01001000,
  B10000100,
  B10011010,
  B10011001,
  B10000001,
  B01000010,
  B00111100
};

int stayHappy[8] = { 
  //happyEye5, 4
  B00000000,
  B00011000,
  B00111100,
  B01100110,
  B01000010,
  B01000010,
  B00000000,
  B00000000
};

int stayDead[8] = {
  // 3
  B10000001,
  B01000010,
  B00100100,
  B00011000,
  B00011000,
  B00100100,
  B01000010,
  B10000001
};

int neutral[8] = {
  B00111100,
  B01000010,
  B10000001,
  B10011001,
  B10011001,
  B10000001,
  B01000010,
  B00111100
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

  pinMode(clk, OUTPUT);
  pinMode(reset, OUTPUT);
  pinMode(pin0, INPUT);
  pinMode(pin1, INPUT);
  pinMode(pin2, INPUT);
  pinMode(pin3, INPUT);
  digitalWrite(reset, HIGH);
  delayMicroseconds(5);
  digitalWrite(reset, LOW);
  Serial.begin(9600);
}

long number;
int tmp;
int var = 0;
int p = 0;
int i = 0;
int state = 0;
// state 0 = neutral 
// state 1 = angry
// state 2 = happy
// state 3 = dead
// state 4 = looking around

void animateEyes2(int (*image)[8], frame *frames, int frameLength) {
  for (int i = 0; i < frameLength; i++) {
    frame f = frames[i];
    drawExpression(f, image);
  }
}

void loop() {

  //---------------
  //-- pin0 -> 10 = bumper (angry)
  //-- pin1 -> 11 = sorting feedback (looking)
  //-- pin2 -> 12 = full bin (die)
  //-- pin3 -> 

  number = random(100);

  if(number > 90){
    if(state == 5){
      state = tmp;
    } else {
      tmp = state;
      state = 5;
    }
  } else if(state == 0) {
    if(digitalRead(pin0) == HIGH) {
      state = 1;
      angry(0);
    } else if(digitalRead(pin1) == HIGH) {
      state = 4;
    } else if(digitalRead(pin2) == LOW) {
      state = 3;
      die(0);
    } //else if(digitalRead(pin3) == HIGH) {
      //state = 4;
    //}
  }

  switch(state) {
    case 1:
      // Angry state
      if(digitalRead(pin0) == LOW) {
        angry(2);
        state = 0;
      } else {
        // stay angry
        angry(1);
      }
      break;
    case 2:
      // Happy state
      if(digitalRead(pin1) == LOW) {
        happiness(2);
        state = 0;
      } else {
        // stay happy
        happiness(1);
      }
      break;
    case 3:
      // Die state
      if(digitalRead(pin2) == HIGH) {
        die(2);
        state = 0;
      } else {
        // stay dead
        die(1);
      }
      break;
    case 4:
      // Look around State
      if( digitalRead(pin1) == LOW){
        state = 2;
        happiness(0);
      }//else if(digitalRead(pin3) == HIGH) {
        //state = 0;
      //} 
      else {
        // Look around
        looking();
      }
      break;
    case 5:
      state = tmp;
      blinking();
      break;
    default:
      normal();
      break;      
      
  }
};

// animate Emotions
void angry(int p){
  if (p == 0){
    animateEyes2(evilEyes, evilBeginFrames, sizeof(evilBeginFrames)/sizeof(*evilBeginFrames));
  }else if(p == 2){
    animateEyes2(evilEyes, evilEndFrames, sizeof(evilEndFrames)/sizeof(*evilEndFrames));
  }else{
    drawEyes(angryLeft,angryRight);
  }
};

void normal(){
  drawEyes(neutral, neutral);
};

void blinking(){
  animateEyes2(blinkEyes, blinkFrames, sizeof(blinkFrames)/sizeof(*blinkFrames));
};

void happiness(int p){
  if(p == 0){
    animateEyes2(happyEyes, happyBeginFrames, sizeof(happyBeginFrames)/sizeof(*happyBeginFrames));
  } else if(p == 2){
    animateEyes2(happyEyes, happyEndFrames, sizeof(happyEndFrames)/sizeof(*happyEndFrames));
  } else {
    drawEyes(stayHappy, stayHappy);
  }
};

void looking(){
  animateEyes2(lookEyes, lookAroundFrames, sizeof(lookAroundFrames)/sizeof(*lookAroundFrames));
};

void die(int p){
  if(p == 0){
    animateEyes2(deadEyes, dieBeginFrames, sizeof(dieBeginFrames)/sizeof(*dieBeginFrames));
  } else if(p == 2){
    animateEyes2(deadEyes, dieEndFrames, sizeof(dieEndFrames)/sizeof(*dieEndFrames));
  } else {
    drawEyes(stayDead, stayDead);
  }
};

// Draw single frame for a spedified time. 
void drawExpression(frame &f, int (*image)[8]) {
  long startTime = millis();
  while ((millis() - startTime) < f.frameDelay) {
    int *shapeLeft = image[f.frameLeftIndex];
    int *shapeRight = image[f.frameRightIndex];
    drawEyes(shapeLeft, shapeRight);
  }
};

void drawEyes(int eyeShapeLeft[], int eyeShapeRight[]) {
  reset_on();
  for (int i = 0; i < 8; i++) {
    trans(eyeShapeLeft[i],eyeShapeRight[i]);
    delayMicroseconds(1000);
    if (i >= 7) {
      trans(eyeShapeLeft[i],eyeShapeRight[0]);
      //delayMicroseconds(10);
    }
    else {
      trans(eyeShapeLeft[i],eyeShapeRight[i+1]);
      //delayMicroseconds(10);
    }
    clock_on();
  }
};

void trans(int left, int right){
  
  Wire.beginTransmission(0x20);
  Wire.write(0x12);
  Wire.write(right); // set all of port A to outputs
  Wire.endTransmission();
  
  Wire.beginTransmission(0x20);
  Wire.write(0x13);
  Wire.write(left); // set all of port B to outputs
  Wire.endTransmission();
  
};

void reset_on() {
  digitalWrite(reset, HIGH);
  delayMicroseconds(15);
  digitalWrite(reset, LOW);
};

void clock_on() {
  digitalWrite(clk, HIGH);
  delayMicroseconds(15);
  digitalWrite(clk, LOW);
};










