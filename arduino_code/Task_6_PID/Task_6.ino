
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
float kp                 = 150;                         //Line follow PID variables
float ki                 = 0;
float kd                 = 0;
float Integral, Derivative ,output;
int error,previousError,integrating, derivating;
unsigned long now ,time_elapsed ,previous_now;
int irReadings[2];                                     //Ir sensor array defination
char packetBuffer[255];                                //UDP variables

void setup()
{ 
  Serial.begin(115200);
  setup_motors();
  wifi_def();
  Serial.println(" All set -> Get ready ");
  delay(3000);
  start_robot();
}

void loop()
{
  if(udp.parsePacket() ){                               // If we recieve any packet
      parseUdpMessage();

 }else{
    Get_Ir();
    Error_computation();
    Pid_apply();
    Updating_motors();
}
}

// here we recieve the upd message and process is 
void parseUdpMessage() 
    {
    udp.read(packetBuffer, 255);
    stop_robot();
    char * strtokIndx;
    strtokIndx = strtok(packetBuffer,",");
    kp = atof(strtokIndx); 
    strtokIndx = strtok(NULL, ",");
    ki = atof(strtokIndx);
    strtokIndx = strtok(NULL, ",");
    kd= atof(strtokIndx);
    strtokIndx = strtok(NULL, ",");
    Right_motor_speed = atoi(strtokIndx);
    strtokIndx = strtok(NULL, ",");
    Left_motor_speed= atoi(strtokIndx);
    
    Serial.print("\n\n\nKP / Ki / KD =");Serial.print(kp);Serial.print("/");Serial.print(ki);Serial.print(" / ");Serial.print(kd);
    Serial.print("M_R / M_L ");Serial.print(Left_motor_speed);Serial.print(" // ");Serial.print(Right_motor_speed);
    Serial.println("\n\n\nGET Ready !!");
    
    delay (3000);
    start_robot();                                // enable the motors
    reset_();
    

}

void reset_(){                       // previous Kp values residue must not interfere with new one
  output=0;
  previous_now=0;
  integrating=0;
}

void Get_Ir() {
irReadings[0]=(digitalRead(Ir_L));
irReadings[1]=(digitalRead(Ir_R));
Serial.print(" \n ");Serial.print(irReadings[0]);Serial.print(" | ");Serial.print(irReadings[1]);Serial.print(" \n ");


}

void Error_computation() {
 // 1 represents to be on black line
  if ((irReadings[0] == 0) && (irReadings[1] == 1))  { // only right  sensor on black
    error = -1;}
  else if ((irReadings[0] == 0) && (irReadings[1] == 0) ) { // straight
    error = 0;}
  else if ((irReadings[0] == 1) && (irReadings[1] == 0) ) { // only left sensor on black
    error = 1;}
  else if  ((irReadings[0] == 1) && (irReadings[1] == 1)) {
      stop_robot(); 
  }
  
  }
void Pid_apply() {  
  now = millis();                                      //get current time
  time_elapsed = (double)(now - previous_now);        //compute time elapsed from previous computation    
  integrating += error * time_elapsed; // 
  //derivating = double(error - previousError) / time_elapsed;
  output = kp*error + ki*integrating;// + kd*derivating;
//  previousError = error;
  previous_now=now;
}

void Updating_motors() {
  int  rightMotorSpeed = Right_motor_speed + output ;
  int   leftMotorSpeed = Left_motor_speed -  output  ;
  rightMotorSpeed = constrain(rightMotorSpeed, 90, 200);
  leftMotorSpeed  = constrain(leftMotorSpeed, 90, 200);
  Serial.print(leftMotorSpeed);Serial.print(" | ");Serial.print(rightMotorSpeed);
  Serial.print(" ///  ");Serial.print(output);Serial.print("/");Serial.print(error);Serial.print("/");Serial.println(ki*integrating);
  //ledcWrite(channel_R, rightMotorSpeed);  
  //ledcWrite(channel_L, leftMotorSpeed);
}
void wifi_def(){
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
  digitalWrite(motorLa,HIGH);                             // Forward direction configuration
  digitalWrite(motorRa,HIGH);

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
