
#include <Servo.h>

#define SERVO_PIN    3       //PWM pin connected to servo
Servo servo;          //create a servo object
int servoAngle = 90;  //servo angle vary from 0 - 180, (50 to 130)
int servo_adjust = 0; //servo angle to be adjusted
char tmp[8];           //temp for storing servo adjust string
const char* incomingBytes;
String message = "";




void setup() {
  Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
  servo.attach(SERVO_PIN);
}

int toString(char a[]){
  int c, sign, n;
  if (tmp[0] == '0'){
    sign = 1;
  }
  else{
    sign = -1;
  }

  n = 0;
  
  for (c = 1; a[c] != '\0'; c++){
    n = n*10 + a[c] - '0';
  }

  if (sign == -1){
    n = -n;
  }
  return n;
}

void loop() {
//  servo.write(0);
//  delay(1000);
  servo.write(servoAngle);
//  delay(1000);
//  servo.write(180);
//  delay(1000);
  
  // send data only when you receive data:
  if (Serial.available() >= 7) {
    // read the incoming byte:
    message = Serial.readString();
    
    int incomingBytes_len = message.length() + 1;
    char buf[8];
    incomingBytes = message.toCharArray(buf,8);
    
    Serial.println(incomingBytes);
    strncpy(tmp, incomingBytes +4, 3);
    servo_adjust = toString(tmp);
    Serial.println(servo_adjust);
  
    servo.write(servoAngle + servo_adjust);
    delay(1000);

    
  }
}
