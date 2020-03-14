#define Ir_L 15                                         // Ir pins
#define Ir_R 13
#define motorRa 27                                      // motor pins
#define motorRb 14
#define motorLa 25
#define motorLb 26
#define Rpwm    12
#define Lpwm    33
                    
const int channel_R      = 0;                          //PWM setup
const int channel_L      = 1;   
int Right_motor_speed    = 100;                        // Motor Base speeds
int Left_motor_speed     = 100;
int irReadings[2];                   `                  //Ir sensor array defination

void setup()
{ 
  Serial.begin(115200);
  setup_motors();
  Serial.println(" All set -> Get ready ");
  delay(3000);
  start_robot();
}

void loop()
{
    Get_Ir();
    action();
}


void Get_Ir() {
irReadings[0]=(digitalRead(Ir_L));
irReadings[1]=(digitalRead(Ir_R));
Serial.print(" \n ");Serial.print(irReadings[0]);Serial.print(" | ");Serial.print(irReadings[1]);Serial.print(" \n ");


}

void action() {
 // 1 represents to be on black line
  if ((irReadings[0] == 0) && (irReadings[1] == 1))  { // only right  sensor on black
    right();
    }
  else if ((irReadings[0] == 0) && (irReadings[1] == 0) ) { // straight
    forward();
    }
  else if ((irReadings[0] == 1) && (irReadings[1] == 0) ) { // only left sensor on black
    left();
    }
  else if  ((irReadings[0] == 1) && (irReadings[1] == 1)) {
      stop_robot(); 
  }
  
  }


void setup_motors(){
  // pwm setup variables
  const int freq = 5000;
  const int res = 8;

  // direction for motor pinout defination
  pinMode(motorLa, OUTPUT);
  pinMode(motorLb, OUTPUT);
  pinMode(motorRa, OUTPUT);
  pinMode(motorRb, OUTPUT);
  pinMode(Ir_L,INPUT);
  pinMode(Ir_R,INPUT);
  // Pwm functionality setup
  ledcSetup(channel_R ,freq , res);
  ledcSetup(channel_L ,freq , res);
  ledcAttachPin(Rpwm,channel_R);
  ledcAttachPin(Lpwm,channel_L);
}

// function to enable the motors of robot
void start_robot(){
  ledcWrite(channel_R , Right_motor_speed);
  ledcWrite(channel_L , Left_motor_speed);
  digitalWrite(motorLa,HIGH);                             // Forward direction configuration
  digitalWrite(motorRa,HIGH);
}



void forward(){
  Serial.println(" forward");
  digitalWrite(motorLa,HIGH);                             
  digitalWrite(motorRa,HIGH); 
  digitalWrite(motorLb,LOW);                             
  digitalWrite(motorRb,LOW);  

}
void left(){
  Serial.println("left ");
  digitalWrite(motorLa,LOW);                             
  digitalWrite(motorRa,HIGH); 
  digitalWrite(motorLb,LOW);                             
  digitalWrite(motorRb,LOW);  

}
void right(){
  Serial.println("right ");
  digitalWrite(motorLa,HIGH);                             
  digitalWrite(motorRa,LOW); 
  digitalWrite(motorLb,LOW);                             
  digitalWrite(motorRb,LOW);  

}


// function to disable the motors of robot
void stop_robot(){
  Serial.println(" Stop");
  ledcWrite(channel_R , 0);                               // giving each motor 0 dutycycle value
  ledcWrite(channel_L , 0); 
  digitalWrite(motorLa,LOW);                             // Stop direction because constrain function overrides it.
  digitalWrite(motorRa,LOW);  
  digitalWrite(motorLb,LOW);                             
  digitalWrite(motorRb,LOW); 
  delay(3000);
}
