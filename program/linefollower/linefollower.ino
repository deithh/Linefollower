
#include "SparkFun_TB6612.h"

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// Piny do silników
  #define AIN1 7
  #define AIN2 8
  #define PWMA 5
  
  #define BIN1 2
  #define BIN2 1
  #define PWMB 3
  
  #define STBY 4

// Piny do sensorów
#define S1 A0
#define S2 A1
#define S3 A2
#define S4 A3
#define S5 A4

// Pozostałe piny
#define CN A5       //Czujnik napięcia
#define Led1 10     //Przedni led
#define Led2 6      //Tylni led
#define Button 9

// Stałe związane z silnikami 
#define max_speed 100.0 //set Max Speed Value
#define max_speed_change 350 //set Max Speed Change Value

// PID
#define set_point 2000
#define sensor_weight 1000
int minValues[5] = {0,0,0,0,0};
int maxValues[5] = {1023, 1023, 1023, 1023, 1023};
float Kp = 1; //set Kp Value  Optimized: K=0.73 I=0.0007 D=1 max_speed=165
float Ki = 0.000; //set Ki Value
float Kd = 0; //set Kd Value

//aditional const to control error value magnitude
#define silencer 1

#define max_offroad_time 1000
#define delay_time 1

int state = 0;
// Konfiguracja silników
const int offsetA = 1;
const int offsetB = -1;

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);      //prawy motor
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);      //lewy motor

const int sensorPins[5] = {S1, S2, S3, S4, S5};
long sensorMults[5] = {0,0,0,0,0};

float volts = 0;
bool robotRunning = false;
bool lastSwitchState = HIGH;

// zmienne do PID
long sensors_sum=0;
long sensors_average=0;
long sensors[5]={0,0,0,0,0};

long Position=0;
long proportional=0;
long integral=0;
long derivative=0;
long last_proportional=0;
long error_value=0;

int left_speed=0;
int right_speed=0;

int last_left_speed=0;
int last_right_speed=0;

int offroad_time=0;
bool ifLedOn = false;

void setup() {
  analogReference(DEFAULT);

  sbi(ADCSRA, ADPS2);
  sbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);

  for (int i = 0; i < 5; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  pinMode(Led1, OUTPUT);
  pinMode(Led2, OUTPUT);
  pinMode(Button, INPUT_PULLUP);

  for( int i = 0; i < 14; i++){
    if(i == 6 || i == 10) continue;
    pinMode(i , INPUT_PULLUP);
  }
  
  digitalWrite(Led1, LOW);
  digitalWrite(Led2, LOW);
  for (int i = 0; i < 5; i++){
      sensorMults[i] = i * sensor_weight;
      
  }
}

void loop() {
  volts = analogRead(CN) * 10.0 / 1023.0;
   //Poniżej minimalnego napięcia baterii
  if(volts < 7.5){
    // Czy robot jest włączony?
      if(robotRunning){
        robotRunning = false;
        
        Shutdown();
      }

    while(1){
      digitalWrite(Led2, HIGH);
      delay(1000);
      digitalWrite(Led2, LOW);
      delay(1000);
    }
  }else if(volts < 8){
    digitalWrite(Led2, HIGH);
  }else{
    digitalWrite(Led2, LOW);
  }

  bool currentSwitchState = digitalRead(Button);

  if (currentSwitchState == LOW && lastSwitchState == HIGH) {
    robotRunning = !robotRunning;

    if (robotRunning) {
      delay(300);
      currentSwitchState = digitalRead(Button);
      if(currentSwitchState == LOW){
        calibrate();
        robotRunning = !robotRunning;
        }
       else{
      Start();
       }
    } else {
      Shutdown();
    }

    delay(1000);
  }

  lastSwitchState = currentSwitchState;

  if (robotRunning){
    sensors_sum = 0;
    sensors_average = 0;
    
    for (int i = 0; i < 5; i++){
      sensors[i] = process_signal(analogRead(sensorPins[i]));
      sensors_average += sensors[i] * sensorMults[i];
      sensors_sum += sensors[i];
      
    }

    if(sensors_sum < 4500 && sensors_sum > 2500){
      ifLedOn = true;
      digitalWrite(Led1, HIGH);
      offroad_time = 0;
      
      Position = int(sensors_average / sensors_sum);
      pid_calc();
      calc_turn();

       motor_drive(right_speed,left_speed);
    } else {
      if(offroad_time % 100 == 0){
        if(ifLedOn){
          ifLedOn = false;
          digitalWrite(Led1, LOW);
        }else{
          ifLedOn = true;
          digitalWrite(Led1, HIGH);
        }
      }
      if(offroad_time >= max_offroad_time){
        ifLedOn = true;
        digitalWrite(Led1, HIGH);
        robotRunning = false;
        Shutdown();
        offroad_time = 0;
      }else{
        offroad_time += delay_time;
      }
    }



  }

  delay(delay_time);
}

void Shutdown(){
  brake(motor1, motor2);
  delay(1500);

  digitalWrite(STBY, LOW);

  delay(500);
  ifLedOn = false;
  digitalWrite(Led1, LOW);
}
void Start(){
  digitalWrite(STBY, HIGH);

  delay(1000);
  ifLedOn = true;
  digitalWrite(Led1, HIGH);
}
void pid_calc() 
{
  

    proportional=Position-set_point;
    integral = integral + proportional;
    derivative = proportional - last_proportional;
    last_proportional = proportional;
    error_value = long(proportional * Kp + integral * Ki + derivative * Kd);

}
void calc_turn()
{
  //Restricting the error value between +256.
  error_value *= silencer;
  
  if (error_value < -max_speed){
    error_value = -max_speed;
  }
  if (error_value > max_speed){
    error_value = max_speed;
  }

  // If error_value is less than zero calculate right turn speed values
  if (error_value >= 0){
    right_speed = max_speed - error_value;
    left_speed = max_speed;
  } else {
    right_speed = max_speed;
    left_speed = max_speed + error_value;
  }
}
void motor_drive (int right_speed, int left_speed)
{
  if (right_speed > max_speed)  right_speed = max_speed;
  if (right_speed < 0)          right_speed = 0;
  if (left_speed > max_speed)   left_speed = max_speed;
  if (left_speed < 0)           left_speed = 0;

  // ograniczenie przyśpieszenia silników
  double left_speed_cap = last_left_speed + max_speed_change;
  double right_speed_cap = last_right_speed + max_speed_change;

  if(max_speed < left_speed_cap)    left_speed_cap = max_speed;
  if(max_speed < right_speed_cap)   right_speed_cap = max_speed;
  
  if (left_speed_cap < left_speed)    left_speed = left_speed * (left_speed_cap/max_speed);
  if (right_speed_cap < right_speed)  right_speed = right_speed * (right_speed_cap/max_speed);
  
  if(left_speed <= 5){
    motor2.drive(5);
    last_left_speed = 5;
  }else{
    motor2.drive(left_speed);
    last_left_speed = left_speed;
  }

  if(right_speed <= 5){
    motor1.drive(5);
    last_right_speed = 5;
  }else{
    motor1.drive(right_speed);
    last_right_speed = right_speed;
  }
}

  void calibrate() {
    state = (state + 1) % 3;
    switch (state){
      case 0:
        break;
       case 1:
        break;
       case 2:
        break;
    }

}

int process_signal(int x){

  if(x < 380){
    return 0;
  }
  if (x > 680){
    return 1000;
  }
  return x;
}




