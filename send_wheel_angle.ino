#include <PID_v1.h>

#define PULSES_PER_TURN (2096.0 * 4)   //  Encoder Resolution:  CPR *4  for convert to PPC
#define FOR 1  //  robot Forward test


String inString = "";    // string to hold input type 0 - 255 for test motor with arduino serial
int motorPWM = 0;



//The sample code for driving one way motor encoder
const byte encoder0pinA = 3;//A pin -> the interrupt pin 0
const byte encoder0pinB = 8;//B pin -> the digital pin 4
byte encoder0PinALast;

boolean Direction;//the rotation direction 

// The pin the encoder is connected


volatile long pulseCount = 0;


int rpm = 0;

//-----------------------------------------------------------------------------
#define encodPinA1      2                       // Quadrature encoder A pin
#define encodPinB1      8                       // Quadrature encoder B pin
#define M1              5                       // PWM outputs to L298N H-Bridge motor driver module
#define M2              6

double kp =1, ki =20 , kd =0;             // modify for optimal performance
double input = 0, output = 0, setpoint = 0;
unsigned long lastTime,now;
volatile long encoderPos = 0,last_pos=0,lastpos=0;
PID myPID(&input, &output, &setpoint, kp, ki, kd,DIRECT);  
 
void setup()
{
  Serial.begin(9600);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);

  TCCR1B = TCCR1B & 0b11111000 | 1;                   // To prevent Motor Noise
  
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetOutputLimits(-255, 255);

  
 
  EncoderInit();//Initialize the module
  
}
 
void loop()
{
     double s;
    _TEST_MOTOR_PWM_SERIAL();

     now = millis();
   int timeChange = (now - lastTime);

     if(timeChange>=500 )
   {
      input = (360.0*1000*(encoderPos-last_pos)) /( PULSES_PER_TURN *(now - lastTime));
      lastTime=now;
      last_pos=encoderPos;
   }
   
      s = (360.0*(encoderPos-lastpos))/PULSES_PER_TURN;
      lastTime=now;
      last_pos=encoderPos;

      Serial.println( s);
   
  
  myPID.Compute();                                    // calculate new output
 // pwmOut(output);                                     // drive L298N H-Bridge module
  delay(10);

}

int readSerialValue()
{
     int v= -1;
  // Read serial input:
  while (Serial.available() > 0) {
    int inChar = Serial.read();
    if (isDigit(inChar)) {
      // convert the incoming byte to a char
      // and add it to the string:
      inString += (char)inChar;
    }
    // if you get a newline, print the string,
    // then the string's value:
    if (inChar == '\n') {
      Serial.print("String: ");
      Serial.println(inString);
      v = inString.toInt();
      Serial.print("Value:");
      Serial.println(v);
      
           
      // clear the string for new input:
      inString = "";
    }
  }
   return v;
}

void EncoderInit()
{
  
  pinMode(encoder0pinB,INPUT);  
  pinMode(encoder0pinA,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder0pinA), wheelSpeed, RISING);
 //  attachInterrupt(digitalPinToInterrupt(encoder0pinB), wheelSpeed, CHANGE);
}
void wheelSpeed()
{
  
  encoderPos++;

  if( encoderPos > (2096 * 4) ) encoderPos=0;
 // Serial.println("speed");
  
}
void _TEST_MOTOR_PWM_SERIAL()
{
  int pwm = readSerialValue();

    if( pwm != -1){

       if( pwm > 255) pwm = 255;
       else if( pwm < -1) pwm = 0;

       motorPWM = pwm;
    }

    if( FOR == 1){
      analogWrite(M1, motorPWM);
      analogWrite(M2, 0);
    }else{
      analogWrite(M1, 0);
      analogWrite(M2, motorPWM);
    }
}
