/* Pin Configuration of Encoders with STM32
    All the signals pins to be pulled up using 680ohm.

    Encoder X-Green to PB8
              White to PB9-

    Encoder Y-Green to PB12
              White to PB13

    Encoder Z-Green to PB5
              White to PB15
*/
#include <Wire_slave.h>

int encoderPinX1 = PB8; // GREEN
int encoderPinX2 = PB9; // WHITE
volatile int lastEncodedX = 0;
volatile long encoderValueX = 0;
long lastencoderValueX = 0;
int lastMSBX = 0;
int lastLSBX = 0;

int encoderPinY1 = PB12; // GREEN
int encoderPinY2 = PB13; // WHITE
volatile int lastEncodedY = 0;
volatile long encoderValueY = 0;
long lastencoderValueY = 0;
int lastMSBY = 0;
int lastLSBY = 0;

int encoderPinZ1 = PB5; // GREEN
int encoderPinZ2 = PB15; // WHITE
volatile int lastEncodedZ = 0;
volatile long encoderValueZ = 0;
long lastencoderValueZ = 0;
int lastMSBZ = 0;
int lastLSBZ = 0;
double roundsX = 0;  double roundsY = 0;  double roundsZ = 0;
long roundsXX; long roundsYY; long roundsZZ;


byte buf[11]; long valX = 0; long valY = 0; long valZ = 0;

void setup() {
  Serial.begin (115200);  // To see the encoder data in Serial Monitor.
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onRequest(requestEvent); // register event

  pinMode(encoderPinX1, INPUT_PULLDOWN);
  pinMode(encoderPinX2, INPUT_PULLDOWN);
  attachInterrupt(encoderPinX1, updateEncoderX, CHANGE);
  attachInterrupt(encoderPinX2, updateEncoderX, CHANGE);

  pinMode(encoderPinY1, INPUT_PULLDOWN);
  pinMode(encoderPinY2, INPUT_PULLDOWN);
  attachInterrupt(encoderPinY1, updateEncoderY, CHANGE);
  attachInterrupt(encoderPinY2, updateEncoderY, CHANGE);

  pinMode(encoderPinZ1, INPUT_PULLDOWN);
  pinMode(encoderPinZ2, INPUT_PULLDOWN);
  attachInterrupt(encoderPinZ1, updateEncoderZ, CHANGE);
  attachInterrupt(encoderPinZ2, updateEncoderZ, CHANGE);
}


void loop() {
//-------------------Serial Print------------------------------------
  roundsXX = roundsX * 100;
  roundsYY = roundsY * 100;
  roundsZZ = roundsZ * 100;
 Serial.print(roundsXX);  Serial.print("  ");  Serial.print(roundsYY); Serial.print("  ");  Serial.println(roundsZZ);
//-----------------------STORE ROUNDS X IN BUF------------------------------  
  buf[0] = (roundsXX);
  buf[1] = (roundsXX >> 8);
  buf[2] = (roundsXX >> 16);
  buf[3] = (roundsXX >> 24);
//-----------------------STORE ROUNDS Y1 IN BUF------------------------------ 
  buf[4] = (roundsYY);
  buf[5] = (roundsYY >> 8);
  buf[6] = (roundsYY >> 16);
  buf[7] = (roundsYY >> 24);
//-----------------------STORE ROUNDS Y2 IN BUF------------------------
  buf[8] = (roundsZZ);
  buf[9] = (roundsZZ >> 8);
  buf[10] = (roundsZZ >> 16);
  buf[11] = (roundsZZ >> 24);
//---------------------------------------------------------------------
  roundsX = encoderValueX * 5.0 / 1440.0;
  roundsY = encoderValueY * 5.0 / 1440.0;
  roundsZ = encoderValueZ * 5.0 / 1440.0;

delay(10); //just here to slow down the output, and show it will work  even during a delay
}

void updateEncoderX()
{
  int MSBX = digitalRead(encoderPinX1); //MSB = most significant bit
  int LSBX = digitalRead(encoderPinX2); //LSB = least significant bit
  int encodedX = (MSBX << 1) | LSBX; //converting the 2 pin value to single number
  int sumX  = (lastEncodedX << 2) | encodedX; //adding it to the previous encoded value
  if (sumX == 0b1101 || sumX == 0b0100 || sumX == 0b0010 || sumX == 0b1011) encoderValueX ++;
  if (sumX == 0b1110 || sumX == 0b0111 || sumX == 0b0001 || sumX == 0b1000) encoderValueX --;

  lastEncodedX = encodedX; //store this value for next time
}

void updateEncoderY()
{
  int MSBY = digitalRead(encoderPinY1); //MSB = most significant bit
  int LSBY = digitalRead(encoderPinY2); //LSB = least significant bit
  int encodedY = (MSBY << 1) | LSBY; //converting the 2 pin value to single number
  int sumY  = (lastEncodedY << 2) | encodedY; //adding it to the previous encoded value
  if (sumY == 0b1101 || sumY == 0b0100 || sumY == 0b0010 || sumY == 0b1011) encoderValueY ++;
  if (sumY == 0b1110 || sumY == 0b0111 || sumY == 0b0001 || sumY == 0b1000) encoderValueY --;

  lastEncodedY = encodedY; //store this value for next time
}

void updateEncoderZ()
{
  int MSBZ = digitalRead(encoderPinZ1); //MSB = most significant bit
  int LSBZ = digitalRead(encoderPinZ2); //LSB = least significant bit
  int encodedZ = (MSBZ << 1) | LSBZ; //converting the 2 pin value to single number
  int sumZ  = (lastEncodedZ << 2) | encodedZ; //adding it to the previous encoded value
  if (sumZ == 0b1101 || sumZ == 0b0100 || sumZ == 0b0010 || sumZ == 0b1011) encoderValueZ ++;
  if (sumZ == 0b1110 || sumZ == 0b0111 || sumZ == 0b0001 || sumZ == 0b1000) encoderValueZ --;

  lastEncodedZ = encodedZ; //store this value for next time
}



void requestEvent()
{
  //----------------send thru i2c---------------------------
  for (int i = 0; i <= 11; i++)  {
    Wire.write(buf[i]);
  }



}
