

// Arduino pin assignment
#include <Servo.h>
#define PIN_IR A0
#define PIN_LED 9
#define PIN_SERVO 10

#define _DIST_ALPHA 0.1

Servo myservo;

float alpha;

int dist_ema = 0;
int a, b; // unit: mm

void setup() {
// initialize GPIO pins
  alpha = _DIST_ALPHA;
  myservo.attach(PIN_SERVO);
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);

// initialize serial port
  Serial.begin(57600);
  

  a = 70;
  b = 280;
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
  float raw_dist = ir_distance();
  float dist_cali = 100 + 300.0 / (b - a) * (raw_dist - a);
  
  dist_ema = alpha*raw_dist+(1-alpha)*dist_ema;
  
  Serial.print(map(dist_ema,0,400,100,500));
  Serial.print("min:0,max:500,dist:");
  Serial.print(raw_dist);
  Serial.print(",dist_cali:"); 
  Serial.println(dist_cali);
  if(raw_dist > 156 && raw_dist <280) digitalWrite(PIN_LED, 0);
  else digitalWrite(PIN_LED, 255);
  delay(20);

  
  if(raw_dist > 250){
    myservo.writeMicroseconds(1500);
    delay(10);
    
  }
  else{
    myservo.writeMicroseconds(1700);
    delay(10);


  }

}
