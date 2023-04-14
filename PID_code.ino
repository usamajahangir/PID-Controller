#include <LiquidCrystal_I2C.h> // Library for LCD
#include <time.h>
LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C address 0x27, 16 column and 2 rows

///////////////// PID Variables /////////////////////
float Kp=35;
float Kd=0.3;
float Ki=1;

float error=0, P=0, I=0, D=0, PID_value=0;//defining the intial value 0
float previous_error=0, previous_I=0;//defining initially values of previous_error and previous_I 0 
int initial_motor_speed=170;

//////////////// Parameters  //////////////////////
// For Settling Time
double Ts = 0;
double Time = 0;
double margin = 0;
int Ts_Flag = 0;
int conf = 0;
// For Overshoot
float Over = 0;
float Shoot = 0;

///////////////// Motor Pins /////////////////////
//Left 
int M1A = 13;
int M1B = 12;
int M1P = 15;

// Right
int M2A = 27;
int M2B = 26;
int M2P = 14;

///////////////// Sensor Pins /////////////////////
/// Sensor Convention ///
/// !read gives if hurdle in front
///  read gives if nothing in front
int LL = 34;
int  L = 35;
int  C = 32;
int  R = 33; 
int RR = 25;

///////////////// Other Variables /////////////////////
int LCD_SDA = 21;
int LCS_SCL = 22;

int  LS = 0;   

// Setting PWM properties for motor 1
const int freq_1 = 5000;
const int resolution_1= 8;
const int pwmChannel_1 = 0;

// Setting PWM properties for motor 2
const int freq_2 = 5000;
const int resolution_2= 8;
const int pwmChannel_2 = 1;

void setup() {
    Serial.begin(115200);
  // put your setup code here, to run once:
    pinMode(M1A, OUTPUT);
    pinMode(M1P, OUTPUT);
    pinMode(M1B, OUTPUT);

    pinMode(M2P, OUTPUT);
    pinMode(M2A, OUTPUT);
    pinMode(M2B, OUTPUT);

    pinMode(LL, INPUT);
    pinMode(L, INPUT);
    pinMode(C, INPUT);
    pinMode(R, INPUT);
    pinMode(RR, INPUT);

    ledcSetup(pwmChannel_1, freq_1, resolution_1);
    ledcSetup(pwmChannel_2, freq_2, resolution_2);

    // attach the channel to the GPIO to be controlled
    ledcAttachPin(M1P, pwmChannel_1);
    ledcAttachPin(M2P, pwmChannel_2);

    lcd.init(); // initialize the lcd
    lcd.backlight();
}

void loop() {
  Check_State();  
  if (Ts_Flag == 1){
    Time = millis();
    if(Shoot > Over) Over = Shoot;      
  }
  Serial.println(error);
  Pid();
  Control_Motors();
}

void Check_State(){
    bool MLeft = digitalRead(LL);
    bool Left = digitalRead(L);
    bool Center = digitalRead(C);
    bool Right = digitalRead(R);
    bool MRight = digitalRead(RR);
    
    if(MLeft && Left && Center && Right && !MRight){
      Shoot = 2.5;
      Ts_Flag++;
      error=0.8;
      if(LS != 1)lcd.clear();
        LS = 1;             
        lcd.setCursor(0, 0);      
        lcd.print("Error = ");              
        lcd.print(error); 
    }
    else if(MLeft && Left && Center && !Right && !MRight){
      Shoot = 4.5;
      Ts_Flag++;
      error=1.2;
      if(LS != 2)lcd.clear();
        LS = 2;             
        lcd.setCursor(0, 0);          
        lcd.print("Error = ");              
        lcd.print(error); 
    }
    else if(MLeft && Left && Center && !Right && MRight){
      Shoot = 6.5;
      Ts_Flag++;
      error=1.6;
      if(LS != 2)lcd.clear();
        LS = 2;             
        lcd.setCursor(0, 0);          
        lcd.print("Error = ");              
        lcd.print(error); 
    }
    else if(MLeft && Left && !Center && !Right && MRight){   
      Shoot = 99;         
      Ts_Flag++;
      error=2;
      if(LS != 2)lcd.clear();
        LS = 2;             
        lcd.setCursor(0, 0);         
        lcd.print("Error = ");              
        lcd.print(error); 
    }
    else if(MLeft && Left && Center && Right && MRight){
      error=0;
      if(LS != 3)lcd.clear();
      LS = 3;         
      lcd.setCursor(0, 0);          
      lcd.print("Error = ");              
      lcd.print(error); 
      if (Ts_Flag != 0){
        if (conf == 0){
          margin = millis();
          conf++;          
        }
        if (conf > 0 && ((millis()-margin)/1000) >= 1){
          Ts_Flag = 0;
          Ts = ((millis() - Time)/1000)-1; 
          lcd.setCursor(0,1);          
          lcd.print("Ts=");
          lcd.print(Ts); 
          lcd.print("s Mp=");
          lcd.print(Over);  
          lcd.print("%");   
          Over = 0; 
          Shoot= 0; 
          conf = 0;
        }      
      }       
    }
    else if(MLeft && !Left && !Center && Right && MRight){
      Shoot = 99;
      Ts_Flag++;
      error=-2;
      if(LS != 4)lcd.clear();
        LS = 4;             
        lcd.setCursor(0, 0);          
        lcd.print("Error = ");              
        lcd.print(error);   
    }
    else if(MLeft && !Left && Center && Right && MRight){
      Shoot = 6.5;
      Ts_Flag++;
      error=-1.6;
      if(LS != 4)lcd.clear();
        LS = 4;             
        lcd.setCursor(0, 0);         
        lcd.print("Error = ");              
        lcd.print(error);   
    }
    else if(!MLeft && !Left && Center && Right && MRight){
      Shoot = 4.5;
      Ts_Flag++;
      error=-1.2;
      if(LS != 4)lcd.clear();
        LS = 4;             
        lcd.setCursor(0, 0);           
        lcd.print("Error = ");              
        lcd.print(error);   
    }
    else if(!MLeft && Left && Center && Right && MRight){
      Shoot = 2.5;
      Ts_Flag++;
      error=-0.8;
      if(LS != 5)lcd.clear();
        LS = 5;             
        lcd.setCursor(0, 0);       
        lcd.print("Error = ");              
        lcd.print(error);        
    }
    else if (MLeft && !Left && !Center && !Right && MRight){
      // Stop
      ledcWrite(pwmChannel_1, 200);
      ledcWrite(pwmChannel_2, 200);        
      digitalWrite(M1A, HIGH);
      digitalWrite(M1B, LOW);
      digitalWrite(M2A, LOW);
      digitalWrite(M2B, HIGH); 
      // Display on LCD
      if(LS != 6)lcd.clear();
        LS = 6;   
      lcd.setCursor(0, 0);      
      lcd.print("No Path. Right"); 
    }    
    else if(!MLeft && !Left && !Center && !Right && !MRight){
      // Stop
      ledcWrite(pwmChannel_1, 0);
      ledcWrite(pwmChannel_2, 0);        
      digitalWrite(M1A, LOW);
      digitalWrite(M1B, LOW);
      digitalWrite(M2A, LOW);
      digitalWrite(M2B, LOW); 
      // Display on LCD
      if(LS != 7)lcd.clear();
        LS = 7;   
      lcd.setCursor(0, 0);      
      lcd.print("No Path. Stop"); 
    }
    else {
      if(LS != 8)lcd.clear();
        LS = 8;   
      lcd.setCursor(0, 0);      
      lcd.print("Unknown State"); 

      ledcWrite(pwmChannel_1, 0);
      ledcWrite(pwmChannel_2, 0);        
      digitalWrite(M1A, LOW);
      digitalWrite(M1B, LOW);
      digitalWrite(M2A, LOW);
      digitalWrite(M2B, LOW);       
    }
}

void Control_Motors(){
    // Calculating the effective motor speed:
    int left_motor_speed = 10+initial_motor_speed-PID_value; 
    int right_motor_speed = initial_motor_speed+PID_value;
    
    // The motor speed should not exceed the max PWM value
    constrain(left_motor_speed,0,255);
    constrain(right_motor_speed,0,255);

    ledcWrite(pwmChannel_1, initial_motor_speed-PID_value);
    ledcWrite(pwmChannel_2, initial_motor_speed+PID_value);

    if (error == 0){
      digitalWrite(M1A, HIGH);
      digitalWrite(M1B, LOW);
     
      
      digitalWrite(M2A, HIGH);
      digitalWrite(M2B, LOW);
    }
    else if (error > 0){
      digitalWrite(M1A, LOW);
      digitalWrite(M1B, HIGH);
      
      digitalWrite(M2A, HIGH);
      digitalWrite(M2B, LOW);
    }
    else {
      digitalWrite(M1A, HIGH);
      digitalWrite(M1B, LOW);
      
      digitalWrite(M2A, LOW);
      digitalWrite(M2B, HIGH);
    }
}

void Pid(){
    P = error;
    I = I + previous_I;
    D = error-previous_error;
    
    PID_value = (Kp*P) + (Ki*I) + (Kd*D);
    
    previous_I=I;
    previous_error=error;
}
