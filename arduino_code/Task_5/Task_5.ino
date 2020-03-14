
//Wifi Udp setup
#include <WiFi.h>
#include <WiFiUdp.h>

const char*  ssid = "LuqmanZGeh";
const char*  password = "qwertyuiop";
unsigned int localPort = 9999;
WiFiUDP udp;
IPAddress Server(192, 168, 43, 225);
IPAddress Client(192, 168, 43, 20);
IPAddress Subnet(255, 255, 255, 0);

// motor pins
#define enc_RA  5
#define enc_RB  18
#define enc_LA  17
#define enc_LB  16
#define motorRa 27
#define motorRb 14
#define motorLa 25
#define motorLb 26
#define Rpwm    12
#define Lpwm    33


// Variables

int count_R              = 0;                          //For Encoders
int count_L              = 0;                    
const int channel_R      = 0;                          //PWM setup
const int channel_L      = 1;   
int Right_motor_speed    = 130;                        // Motor Base speeds
int Left_motor_speed     = 130;
float Kp                 = 2.1;                        // Proportional Controller
int error;
int16_t output;

char packetBuffer[255];                                 //UDP variables



void setup()
{ 
  Serial.begin(115200);
  setup_motors();

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {          // Exit only when connected
    delay(500);
    Serial.print(".");}
  WiFi.mode(WIFI_STA);                             // ESP-32 as client
  WiFi.config(Client, Server, Subnet);
  
  Serial.print("\nConnected to ");Serial.println(ssid);
  Serial.print("IP address: ");Serial.println(WiFi.localIP());
  delay(3000);
  udp.begin(localPort);                            // Begin the udp communication
} 


void loop()
{   
    if(udp.parsePacket() ){                         // If we recieve any packet
      parseUdpMessage();

    }else{
    Proportional_controller();                      // start processing the controller
    changeMotorSpeed();                             // update the motors
    }             
  
}

// here we recieve the upd message and process is 
void parseUdpMessage() 
    {
    udp.read(packetBuffer, 255);
    Serial.println(packetBuffer[0]);
    delay(2000);
    if(packetBuffer[0]=='s'){
      stop_robot();
    }else{
      
    char * strtokIndx;
    strtokIndx = strtok(packetBuffer,",");
    Kp = atof(strtokIndx); 
    strtokIndx = strtok(NULL, ",");
    Right_motor_speed = atoi(strtokIndx);
    strtokIndx = strtok(NULL, ",");
    Left_motor_speed= atoi(strtokIndx);
    Serial.print("Kp =");Serial.print(Kp);
    Serial.print("  Motor Speeds :");Serial.print(Left_motor_speed);Serial.print(" // ");Serial.println(Right_motor_speed);
    delay (3000);
    start_robot();                                // enable the motors
    reset_();
    }

}

void reset_(){                       // previous Kp values residue must not interfere with new one
  error=0;
  count_R=0;
  count_L=0;
}

void Proportional_controller() {
  error =  count_L - count_R;
  output = Kp * (error); 
}

void changeMotorSpeed() {

int rightMotorSpeed = Right_motor_speed + output  ;
int leftMotorSpeed  = Left_motor_speed  - output;
    rightMotorSpeed = constrain(rightMotorSpeed, 120, 255);  // One of the best thing here :)
    leftMotorSpeed  = constrain(leftMotorSpeed, 120, 255);
  
    Serial.print(output);Serial.print("/");Serial.print(error);Serial.print(" // ");Serial.print(leftMotorSpeed);Serial.print("/");Serial.println(rightMotorSpeed);
  
    ledcWrite(channel_R, rightMotorSpeed);  
    ledcWrite(channel_L, leftMotorSpeed);

}

// Encoders-Interrupt callback functions
void Update_encR(){
   if (digitalRead(enc_RA) == digitalRead(enc_RB)) count_R--;
    else count_R++;  
}

void Update_encL(){
 if (digitalRead(enc_LA) == digitalRead(enc_LB)) count_L--;
  else count_L++; 
}

// all motor setup is done in one function call
void setup_motors(){
  // pwm setup variables
  const int freq = 5000;
  const int res = 8;

  // direction for motor pinout defination
  pinMode(motorLa, OUTPUT);
  pinMode(motorLb, OUTPUT);
  pinMode(motorRa, OUTPUT);
  pinMode(motorRb, OUTPUT);
  // encoder pinout defination
  pinMode(enc_RA,INPUT);
  pinMode(enc_RB,INPUT);
  pinMode(enc_LA,INPUT);
  pinMode(enc_LB,INPUT);
  // Interrupt connection to gpio pins and defining interrupt case
  attachInterrupt(digitalPinToInterrupt(enc_RA),Update_encR,CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc_LA),Update_encL,CHANGE);
  // Pwm functionality setup
  ledcSetup(channel_R ,freq , res);
  ledcSetup(channel_L ,freq , res);
  ledcAttachPin(Rpwm,channel_R);
  ledcAttachPin(Lpwm,channel_L);

}


// function to enable the motors of robot
void start_robot(){
  digitalWrite(motorLa,HIGH);                             // Forward direction configuration
  digitalWrite(motorRa,HIGH);
  digitalWrite(motorLb,LOW);
  digitalWrite(motorRb,LOW);
  ledcWrite(channel_R , Right_motor_speed);                // giving each motor 0 dutycycle value
  ledcWrite(channel_L , Left_motor_speed); 
}


// function to disable the motors of robot
void stop_robot(){
  Serial.println("Stopping The Car ");
  
  ledcWrite(channel_R , 0);                               // giving each motor 0 dutycycle value
  ledcWrite(channel_L , 0); 
  digitalWrite(motorLa,LOW);                             // Stop direction because constrain function overrides it.
  digitalWrite(motorRa,LOW);  
  delay(3000);
}
