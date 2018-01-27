#include <Servo.h>
#include <Wire.h>
#include <LIDARLite.h>

LIDARLite myLidarLite;

//pins 3,5,6,9,10,11 are PWM
int encDist_count = 0, cum_encDist_count = 0;
const int const_Dist = 80;
int cur_pos;
int steer_power = 80;
int drive_power = 33;
int steer_pos;
int left_steer_thresh = 555;
int right_steer_thresh = 575;
int des_steer_pos = 565, max_left = 225, max_right = 850; //290,815; 250,850; 230, 870
//210,890 is past the physical limits
//220,880 gives one click and I think stalls
#define left_bump_pin 11
#define right_bump_pin 12
#define front_servo_pin 5
#define drive_pin 9
#define steer_pin 10
#define encA 2
#define encB 3
#define steerPot 1

unsigned long serialdata;
unsigned long tServo = millis()-10000;
int inbyte;
int servoPin;
int pinNumber;
int motorNumber;
int sensorVal;
int motorSpeed;
boolean motorDir;
int digitalState;
Servo myservo, driveServo, steerServo;
int servo_pos = 80, prev_servo_pos = 80;
boolean servo_active = 0, switch_hit = 0;

void setup()
{
  pinMode(encA,INPUT_PULLUP);
  pinMode(encB,INPUT_PULLUP);
  pinMode(left_bump_pin,INPUT_PULLUP);
  //digitalWrite(left_bump_pin,HIGH); //set pull up resistor
  pinMode(right_bump_pin,INPUT_PULLUP);
  attachInterrupt(0,enctick,CHANGE);
  attachInterrupt(1,enctick,CHANGE);
  Serial.begin(115200);
  
  myLidarLite.begin(0, true); // Set configuration to default and I2C to 400 kHz
  myLidarLite.configure(0); //4 gave terrible noisy results, 5 seemed good but I think it missed too much
  
  pinMode(7,OUTPUT);
  digitalWrite(7,LOW);
  
  //set steer and drive motor to zero
  steerServo.attach(steer_pin);
  mySteerCommand(0);
  driveServo.attach(drive_pin);
  myDriveCommand(0);
  
  cur_pos = analogRead(steerPot);
  //Serial.println(cur_pos);
  if(cur_pos > right_steer_thresh)
  {
    mySteerCommand(-steer_power);
    do{steer_pos = analogRead(steerPot);}while(steer_pos > right_steer_thresh);
    mySteerCommand(0);
  }
  else if(cur_pos < left_steer_thresh)
  {
    mySteerCommand(steer_power);
    do{steer_pos = analogRead(steerPot);}while(steer_pos < left_steer_thresh);
    mySteerCommand(0);
  }
  
  cur_pos = analogRead(steerPot);
  des_steer_pos = cur_pos;
  myservo.attach(front_servo_pin);
  myservo.write(servo_pos);
  delay(500);
  //myservo.detach();
  
  pinMode(13,OUTPUT);
  digitalWrite(13,LOW);
  /*for(int kk=0;kk<10;kk++)
  {
    delay(300);
    digitalWrite(13,HIGH);
    delay(500);
    digitalWrite(13,LOW);
  }*/
}

void loop()
{
  if(digitalRead(left_bump_pin)==LOW || digitalRead(right_bump_pin)==LOW)
  {
    myDriveCommand(0);
    mySteerCommand(0);
    //digitalWrite(7,LOW);
    //switch_hit = 1;
  }
  /*else
  {
    if(switch_hit == 1)
    {
      digitalWrite(7,HIGH);
      switch_hit = 0;
    }
  }*/
  
  //Replace 1000 with a variable = K*abs(servo_pos - prev_servo_pos)
  if(servo_active && (millis()-tServo > 20*abs(servo_pos - prev_servo_pos)))
  {
    //myservo.detach();
    servo_active = 0;
  }
  
  //begin steer controller
  if(des_steer_pos > max_right)
  {
    des_steer_pos = max_right;
  }
  else if(des_steer_pos < max_left)
  {
    des_steer_pos = max_left;
  }
  steer_pos = analogRead(steerPot);
  if(abs(steer_pos - des_steer_pos) < 20)
  {
      mySteerCommand(0);
  }
  else
  {
    if(des_steer_pos > steer_pos)
    {
      mySteerCommand(steer_power);
    }
    else if(des_steer_pos < steer_pos)
    {
      mySteerCommand(-steer_power);
    }
  }
  //end steer controller
  
  if(Serial.read() == 'A'){
    getSerial();
    switch(serialdata)
    {
    case 1: // A1/
      {
        //drive motors or desired steer pos
        getSerial();
        switch(serialdata)
        {
        case 1: // A1/1/
          {
            //control drive motors
            getSerial(); // A1/1/x=0,1/
            motorDir = serialdata; //0 = neg, 1 = pos
            getSerial(); // A1/1/x=0,1/y=speed/
            motorSpeed = serialdata;
            if(motorDir == 0)
            {
              myDriveCommand(-motorSpeed);
            }
            else
            {
              myDriveCommand(motorSpeed);
            }
            break;
          }
        case 2: // A1/2/
          {
            //desired steer pos
            getSerial(); // A1/2/steer_pos/
            des_steer_pos = serialdata;
            break;
          }
          break;
        }
        break;
      }
    case 2: // A2/
      {
        //digital write, none needed currently
        getSerial(); // A2/pin_to_write/x=0,1/
        pinNumber = serialdata;
        getSerial();
        digitalState = serialdata;
        
        if (digitalState == 0)
        {
          digitalWrite(pinNumber, LOW);
          if(pinNumber == 7) // A2/7/0/
          {
            I2c.end();
          }
        }
        if (digitalState == 1)
        {
          digitalWrite(pinNumber, HIGH);
          /*if(pinNumber == 7) // A2/7/1/
          {
            I2c.begin(); // Opens & joins the irc bus as master
            //delay(10); // Waits to make sure everything is powered up before sending or receiving data  
            I2c.timeOut(50); // Sets a timeout to ensure no locking up of sketch if I2C communication fails
            I2c.setSpeed(0); //Set to high speed (input is a boolean 1 or 0)
          }*/
        }
        
        pinNumber = 0;
        break; 
      }
    case 3: // A3/
      {
        getSerial(); // A3/1,2,3,4/
        switch (serialdata)
        {
          case 1: // A3/1/
          {
            //analog read
            getSerial(); // A3/1/ADC_pin_to_read/
            pinNumber = serialdata;
            //pinMode(pinNumber, INPUT);
            sensorVal = analogRead(pinNumber);
            Serial.println(sensorVal);
            sensorVal = 0;
            pinNumber = 0;
            break;
          } 
          case 2: // A3/2/
          {
            //digital read
            getSerial(); // A3/2/digital_pin_to_read/
            pinNumber = serialdata;
            if(pinNumber == left_bump_pin || pinNumber == right_bump_pin || pinNumber == encA || pinNumber == encB)
            {
              sensorVal = digitalRead(pinNumber);
              Serial.println(sensorVal);
            }
            else
            {
              sensorVal = -1;
              Serial.println(sensorVal);
            }
            sensorVal = 0;
            pinNumber = 0;
            break;
          }
          case 3: // A3/3/
          {
            //lidar read
            //getSerial();
            //pinNumber = serialdata;
            //pinMode(pinNumber, INPUT);
            sensorVal = myLidarLite.distance();
            Serial.println(sensorVal);
            sensorVal = 0;
            pinNumber = 0;
            break;
          }
          case 4: // A3/4/
          {
            //encDist_count read and reset
            sensorVal = encDist_count;
            encDist_count = 0;
            cum_encDist_count += sensorVal;
            Serial.println(sensorVal); //const_Dist
            sensorVal = 0;
            pinNumber = 0;            
            break;
          }
          case 5: // A3/5/
          {
            //cum_encDist_count read
            sensorVal = cum_encDist_count + encDist_count;
            Serial.println(sensorVal); //const_Dist
            sensorVal = 0;
            pinNumber = 0;            
            break;
          } 
        }
        break;
      }
    case 4:
      {
        getSerial();
        switch (serialdata)
        {
          case 2:
          {
             //servo write
             
             getSerial();
             servo_pos = int(serialdata);
             //myservo.attach(front_servo_pin);
             myservo.write(servo_pos);
             servo_active = 1;
             tServo = millis();
             break;
          }
        }
        break;
      }
    }
  }
}

long getSerial()
{
  serialdata = 0;
  while (inbyte != '/')
  {
    inbyte = Serial.read(); 
    if (inbyte > 0 && inbyte != '/')
    {
      serialdata = serialdata * 10 + inbyte - '0';
    }
  }
  inbyte = 0;
  return serialdata;
}

void mySteerCommand(int out)
{
  //out ranges from -99 to 99
  if(out > 99){out = 99;}
  else if(out < -99){out = -99;}
  steerServo.writeMicroseconds(1500+5*out);
}
void myDriveCommand(int out)
{
  //out ranges from -99 to 99
  if(out > 99){out = 99;}
  else if(out < -99){out = -99;}
  driveServo.writeMicroseconds(1500+5*out);
}

void enctick()
{
  static int8_t enc_states[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
  static uint8_t old_AB = 0;
  /**/
  //encoder A on pin 6, encoder B on pin 7
  //byte reading = PIND;
  //byte AB = ((reading & 0x40)>>5) | ((reading & 0x80)>>7);
  
  //encoder A on pin 2, encoder B on pin 3
  byte reading = PIND;
  byte AB = ((reading & 0x04)>>1) | ((reading & 0x08)>>3);
  
  old_AB <<= 2;                   //remember previous state
  old_AB |= ( AB );  //add current state
  encDist_count += ( enc_states[( old_AB & 0x0f )]);
}
