#define PIN_LED 7
int i;
void setup() {
  // put your setup code here, to run once:
  pinMode(7, OUTPUT); //7번핀을 출력핀으로 설정
  }

void loop(){ 
  digitalWrite(7,0);
  delay(1000);
  for(i=0; i<11; i++)
  {
    digitalWrite(7,!(digitalRead(7)));
    delay(100);
  }
  i++;
  while(1) //infinite loop
  {
    if(i>12)
    {
      digitalWrite(7,0);
      break;
    }
    
  }
  
}
