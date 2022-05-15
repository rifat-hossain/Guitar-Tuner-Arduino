#include <arduinoFFT.h>
#define SAMPLES 128
#define SAMPLING_FREQUENCY 2048

arduinoFFT FFT = arduinoFFT();

unsigned int samplingPeriod;
unsigned long microSeconds;

double data[SAMPLES];
double data_img[SAMPLES];
double maxval = 0;
double minval = 0;
double base = 0;
int i = 0;
double strings[6] = {82.41,110,146.83,196,246.94,329.63};
double tholds[6][6];
int sel_range = 3;
const int analogInPin = A0;
int i_s = -1;

float outdt[3] = {0,0,0};

void setup() {
  Serial.begin(250000);
  samplingPeriod = round(1000000*(1.0/SAMPLING_FREQUENCY));

  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  for(int i=0;i<6;i++){
    tholds[i][2] = strings[i] - sel_range;
    tholds[i][3] = strings[i] + sel_range;
    if(i == 0){
      tholds[0][5] = (strings[0] + strings[1])/2;
      tholds[0][0] = 2 * strings[0] - tholds[0][5];
      tholds[0][4] = (tholds[i][5] + strings[0])/2;
      tholds[0][1] = 2 * strings[0] - tholds[0][4];
    }
    else if(i == 5){
      tholds[5][0] = (strings[4] + strings[5])/2;
      tholds[5][5] = 2 * strings[5] - tholds[5][0];
      tholds[5][1] = (tholds[5][0] + strings[5])/2;
      tholds[5][4] = 2 * strings[5] - tholds[5][1];
    }
    else{
      tholds[i][0] = (strings[i-1] + strings[i])/2;
      tholds[i][1] = (tholds[i][0] + strings[i])/2;
      tholds[i][5] = (strings[i] + strings[i+1])/2;
      tholds[i][4] = (strings[i] + tholds[i][5])/2;
    }
  }
}

void loop() {
  for(i=0;i<SAMPLES;i++){
    microSeconds = micros();

    data[i] = analogRead(analogInPin);
    data_img[i] = 0;
    while(micros() < (microSeconds + samplingPeriod));
  }
  maxval = data[0];
  minval = data[0];
  for(int j = 1;j<SAMPLES;j++){
    if(data[j] > maxval){
      maxval = data[j];
    }
    if(data[j] < minval){
      minval = data[j];
    }
  }
  base = (maxval + minval)/2;
  for(int j = 0;j<SAMPLES;j++){
    data[j] = data[j] - base;
  }

  FFT.Windowing(data, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(data, data_img, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(data, data_img, SAMPLES);
  
  outdt[0] = outdt[1];
  outdt[1] = outdt[2];
  outdt[2] = (FFT.MajorPeak(data, SAMPLES, SAMPLING_FREQUENCY) + outdt[0] + outdt[1])/3;
  
  Serial.println(outdt[2]);
  if(outdt[2] < tholds[0][5]){
    if(outdt[2] > tholds[0][0]){
      digitalWrite(4,HIGH);
      digitalWrite(3,LOW);
      digitalWrite(2,LOW);
      digitalWrite(7,LOW);
      digitalWrite(6,LOW);
      digitalWrite(5,LOW);
      i_s = 0;
    }
    else{
      i_s = -1;
      digitalWrite(4,LOW);
      digitalWrite(3,LOW);
      digitalWrite(2,LOW);
      digitalWrite(7,LOW);
      digitalWrite(6,LOW);
      digitalWrite(5,LOW);
    }
  }
  else if(outdt[2] < tholds[1][5]){
    digitalWrite(4,LOW);
    digitalWrite(3,HIGH);
    digitalWrite(2,LOW);
    digitalWrite(7,LOW);
    digitalWrite(6,LOW);
    digitalWrite(5,LOW);
    i_s = 1;
  }
  else if(outdt[2] < tholds[2][5]){
    digitalWrite(4,LOW);
    digitalWrite(3,LOW);
    digitalWrite(2,HIGH);
    digitalWrite(7,LOW);
    digitalWrite(6,LOW);
    digitalWrite(5,LOW);
    i_s = 2;
  }
  else if(outdt[2] < tholds[3][5]){
    digitalWrite(4,LOW);
    digitalWrite(3,LOW);
    digitalWrite(2,LOW);
    digitalWrite(7,HIGH);
    digitalWrite(6,LOW);
    digitalWrite(5,LOW);
    i_s = 3;
  }
  else if(outdt[2] < tholds[4][5]){
    digitalWrite(4,LOW);
    digitalWrite(3,LOW);
    digitalWrite(2,LOW);
    digitalWrite(7,LOW);
    digitalWrite(6,HIGH);
    digitalWrite(5,LOW);
    i_s = 4;
  }
  else if(outdt[2] < tholds[5][5]){
    digitalWrite(4,LOW);
    digitalWrite(3,LOW);
    digitalWrite(2,LOW);
    digitalWrite(7,LOW);
    digitalWrite(6,LOW);
    digitalWrite(5,HIGH);
    i_s = 5;
  }
  else{
    digitalWrite(4,LOW);
    digitalWrite(3,LOW);
    digitalWrite(2,LOW);
    digitalWrite(7,LOW);
    digitalWrite(6,LOW);
    digitalWrite(5,LOW);
    i_s = -1;
  }
  if(i_s >= 0){
    if(outdt[2] < tholds[i_s][1]){
      digitalWrite(8,HIGH);
      digitalWrite(9,LOW);
      digitalWrite(10,LOW);
      digitalWrite(11,LOW);
      digitalWrite(12,LOW);
    }
    else if(outdt[2] < tholds[i_s][2]){
      digitalWrite(8,LOW);
      digitalWrite(9,HIGH);
      digitalWrite(10,LOW);
      digitalWrite(11,LOW);
      digitalWrite(12,LOW);
    }
    else if(outdt[2] < tholds[i_s][3]){
      digitalWrite(8,LOW);
      digitalWrite(9,LOW);
      digitalWrite(10,HIGH);
      digitalWrite(11,LOW);
      digitalWrite(12,LOW);
    }
    else if(outdt[2] < tholds[i_s][4]){
      digitalWrite(8,LOW);
      digitalWrite(9,LOW);
      digitalWrite(10,LOW);
      digitalWrite(11,HIGH);
      digitalWrite(12,LOW);
    }
    else{
      digitalWrite(8,LOW);
      digitalWrite(9,LOW);
      digitalWrite(10,LOW);
      digitalWrite(11,LOW);
      digitalWrite(12,HIGH);
    }
  }
  else{
    digitalWrite(8,LOW);
    digitalWrite(9,LOW);
    digitalWrite(10,LOW);
    digitalWrite(11,LOW);
    digitalWrite(12,LOW);
  }
}
