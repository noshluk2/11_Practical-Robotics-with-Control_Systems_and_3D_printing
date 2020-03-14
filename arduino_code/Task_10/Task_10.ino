
//  https://www.nxp.com/docs/en/application-note/AN3461.pdf      documentation link
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <PID_v1.h>
const char*  ssid = "LuqmanZGeh";
const char*  password = "qwertyuiop";
const char*  udpAddress = "192.168.43.255";
unsigned int localPort_ = 9999;
WiFiUDP udp;
IPAddress ipServer(192, 168, 43, 225); 
IPAddress ipClient(192, 168, 43, 226);
IPAddress Subnet(255, 255, 255, 0);


// motor pins
#define motorRa 16
#define motorRb 17
#define motorLa 18
#define motorLb 23
#define Rpwm    4
#define Lpwm    5
#define BASE_SPEED 120

// New Variables
double Output;                                                  //PID
double kp = 1; 
double ki = 0;
double kd = 0; 
double Setpoint = 0.5;
  //MPU variables
int16_t Acc_X, Acc_Y, Acc_Z;                                    // int16_t very important
float rad_to_deg = 57.32;                                      //   180/3.141592654
double roll_angle,pitch_angle;
int RIGHT_SPEED,LEFT_SPEED,motors_speed;
// Old Variables
const int channel_R      = 0;                                  //PWM setup
const int channel_L      = 1;   
char packetBuffer[255];                                        //UDP variables


PID SBR_PID(&pitch_angle, &Output, &Setpoint, kp, ki, kd, DIRECT);

void setup()
{ 
  Serial.begin(115200);
  setup_motors();                                      
  wifi_def();                                                   // Begin the udp communication
  SBR_PID.SetMode(AUTOMATIC);
  SBR_PID.SetOutputLimits(-200,+200);
  I2C_MPU_config();
} 


void loop()
{   
 // Udp_Packet_Check();    
  Process_MPU_values();    
 // Update_Motors_Speeds();
}

void Process_MPU_values(){
 
    
     Wire.beginTransmission(0x68);
     Wire.write(0x3B);                                                     //Ask for the 0x3B register- correspond to AcX
     Wire.endTransmission(false);
     Wire.requestFrom(0x68,6,true);                                        // Accessing next 6 register containing accelerometer values
     Acc_X=Wire.read()<<8|Wire.read();                                     //each value needs two registres
     Acc_Y=Wire.read()<<8|Wire.read();
     Acc_Z=Wire.read()<<8|Wire.read();
     
    
     pitch_angle = atan(-1*(Acc_X/16384.0)/sqrt(pow((Acc_Y/16384.0),2) + pow((Acc_Z/16384.0),2)))*rad_to_deg;
  // roll_angle = atan((Acc_Y/16384.0)/sqrt(pow((Acc_X/16384.0),2) + pow((Acc_Z/16384.0),2)))*rad_to_deg;
   Serial.println(pitch_angle);

}



void Update_Motors_Speeds() {    
  SBR_PID.Compute();
  if(Output>0){
  Reverse_Robot();
}else{
   Forward_Robot();
}

    motors_speed = BASE_SPEED + abs(Output) ;
  Serial.print("OUTPUT PID            -----> ");Serial.println(Output);
  RIGHT_SPEED = constrain(motors_speed, 90, 200);
  LEFT_SPEED  = constrain(motors_speed, 90, 200);
  

   ledcWrite(channel_R, RIGHT_SPEED);  
   ledcWrite(channel_L, LEFT_SPEED);

}


void Udp_Packet_Check(){
  if(udp.parsePacket() ){                                                  // If we recieve any packet
    udp.read(packetBuffer, 255);
    Stop_Robot();
    char * strtokIndx;
    strtokIndx = strtok(packetBuffer,",");
    kp = atof(strtokIndx); 
    strtokIndx = strtok(NULL, ",");
    ki = atof(strtokIndx);
    strtokIndx = strtok(NULL, ",");
    kd= atof(strtokIndx);
    
    Serial.print("\n\n\nKP / Ki / KD =");Serial.print(kp);Serial.print("/");Serial.print(ki);Serial.print(" / ");Serial.println(kd);
    Serial.println("\n\n\nGET Ready !!");
    
    SBR_PID.SetTunings(kp, ki, kd);
    delay (3000);
    Forward_Robot();                     
    
  
  }
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
  // Pwm functionality setup
  ledcSetup(channel_R ,freq , res);
  ledcSetup(channel_L ,freq , res);
  ledcAttachPin(Rpwm,channel_R);
  ledcAttachPin(Lpwm,channel_L);

}


void Stop_Robot(){
  digitalWrite(motorLa,LOW);
  digitalWrite(motorRa,LOW);
  digitalWrite(motorLb,LOW);
  digitalWrite(motorRb,LOW);
  ledcWrite(channel_R , 0);                             
  ledcWrite(channel_L , 0);
}

void Forward_Robot(){
  digitalWrite(motorLa,HIGH);
  digitalWrite(motorRa,HIGH);
  digitalWrite(motorLb,LOW);
  digitalWrite(motorRb,LOW);
}

void Reverse_Robot(){
  digitalWrite(motorLa,LOW);
  digitalWrite(motorRa,LOW);
  digitalWrite(motorLb,HIGH);
  digitalWrite(motorRb,HIGH);
}
void I2C_MPU_config(){
  Wire.begin(22,19); //sda,scl
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);                                                   //power managment register = 6B
  Wire.write(0);
  Wire.endTransmission(true);

}
void wifi_def(){
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {                             // Exit only when connected
    delay(500);
    Serial.print(".");}
  
  Serial.print("\nConnected to ");Serial.println(ssid);
  Serial.print("IP address: ");Serial.println(WiFi.localIP());
  delay(3000);
  udp.begin(localPort_);                                              // Begin the udp communication
}
