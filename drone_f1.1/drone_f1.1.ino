// Import all libraries
#define BLYNK_PRINT Serial
#include<ESP32Servo.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>



// Bynk Authorization
char auth[] = "amadp4ybZBdUyrqQk-xq2DYEUBA3ha0P";
char ssid[] = "Honor 9 Lite";
char pass[] = "mohan123";


const uint16_t mpuAddr = 0x68;


// All motors Initialization
Servo Top_Left;
Servo Top_Right;
Servo Bottom_Left;
Servo Bottom_Right;


// speed initialization



int input_TL_THROTTLE;
int input_TR_THROTTLE;
int input_BL_THROTTLE;
int input_BR_THROTTLE;
int input_ROLL;
int input_PITCH;



// LCD widget
WidgetLCD lcd(V9);


// Gyro Variables
float elapsedTime, time1, timePrev;        //Variables for time control
int gyro_error=0;                         //We use this variable to only calculate once the gyro data error
int16_t Gyr_rawX, Gyr_rawY, Gyr_rawZ;       //Here we store the raw data read 
float Gyro_angle_x, Gyro_angle_y;         //Here we store the angle value obtained with Gyro data
float Gyro_raw_error_x, Gyro_raw_error_y; //Here we store the initial gyro data error



//Acc Variables
int acc_error=0;                            //We use this variable to only calculate once the Acc data error
float rad_to_deg = 180/3.141592654;         //This value is for pasing from radians to degrees values
int16_t Acc_rawX, Acc_rawY, Acc_rawZ;         //Here we store the raw data read 
float Acc_angle_x, Acc_angle_y;             //Here we store the angle value obtained with Acc data
float Acc_angle_error_x, Acc_angle_error_y; //Here we store the initial Acc data error


float Total_angle_x, Total_angle_y;

//More variables for the code
int i;
int mot_activated=0;
long activate_count=0;
long des_activate_count=0;
int isCallibrated = 0;




//////////////////////////////PID FOR ROLL///////////////////////////
float roll_PID, pwm_T_L, pwm_B_L, pwm_T_R, pwm_B_R, roll_error, roll_previous_error;
float roll_pid_p=0;
float roll_pid_i=0;
float roll_pid_d=0;
///////////////////////////////ROLL PID CONSTANTS////////////////////
double roll_kp=0.7;
double roll_ki=0.006;
double roll_kd=1.2;
float roll_desired_angle = 0;    

//////////////////////////////PID FOR PITCH//////////////////////////
float pitch_PID, pitch_error, pitch_previous_error;
float pitch_pid_p=0;
float pitch_pid_i=0;
float pitch_pid_d=0;
///////////////////////////////PITCH PID CONSTANTS///////////////////
double pitch_kp=0.72;//3.55
double pitch_ki=0.006;//0.003
double pitch_kd=1.22;//2.05
float pitch_desired_angle = 0;    


int D2 = 2;
int D3 = 4;
int D4 = 5;
int D5 = 18;


int BASE_THROTTLE = 1400;



void setup() {
    
    
    Top_Left.attach(D2);
    Top_Right.attach(D3);
    Bottom_Left.attach(D4);
    Bottom_Right.attach(D5);


    Top_Left.writeMicroseconds(1000);
    Top_Right.writeMicroseconds(1000);
    Bottom_Left.writeMicroseconds(1000);
    Bottom_Right.writeMicroseconds(1000);

   

    Wire.begin();
    Wire.beginTransmission(mpuAddr);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission(true);



    Wire.beginTransmission(mpuAddr);           //begin, Send the slave adress (in this case 68) 
    Wire.write(0x1B);                       //We want to write to the GYRO_CONFIG register (1B hex)
    Wire.write(0x10);                       //Set the register bits as 00010000 (100dps full scale)
    Wire.endTransmission(true);             //End the transmission with the gyro


    Wire.beginTransmission(mpuAddr);           //Start communication with the address found during search.
    Wire.write(0x1C);                       //We want to write to the ACCEL_CONFIG register (1A hex)
    Wire.write(0x10);                       //Set the register bits as 00010000 (+/- 8g full scale range)
    Wire.endTransmission(true);

    Serial.begin(9600);

    delay(1000);
    time1 = millis();                         //Start counting time in milliseconds


    if(gyro_error==0)
    {
      for(int i=0; i<200; i++)
      {
        Wire.beginTransmission(mpuAddr);            //begin, Send the slave adress (in this case 68) 
        Wire.write(0x43);                        //First adress of the Gyro data
        Wire.endTransmission(false);
        Wire.requestFrom(mpuAddr,(uint8_t)4,true);           //We ask for just 4 registers 
         
        Gyr_rawX=Wire.read()<<8|Wire.read();     //Once again we shif and sum
        Gyr_rawY=Wire.read()<<8|Wire.read();
   
      /*---X---*/
        Gyro_raw_error_x = Gyro_raw_error_x + (Gyr_rawX/32.8); 
      /*---Y---*/
        Gyro_raw_error_y = Gyro_raw_error_y + (Gyr_rawY/32.8);
        if(i==199)
        {
          Gyro_raw_error_x = Gyro_raw_error_x/200;
          Gyro_raw_error_y = Gyro_raw_error_y/200;
          gyro_error=1;
        }
      }
    }


    if(acc_error==0)
    {
      for(int a=0; a<200; a++)
      {
          Wire.beginTransmission(mpuAddr);
          Wire.write(0x3B);                       //Ask for the 0x3B register- correspond to AcX
          Wire.endTransmission(false);
          Wire.requestFrom(mpuAddr,(uint8_t)6,true); 
      
          Acc_rawX=(Wire.read()<<8|Wire.read())/4096.0 ; //each value needs two registres
          Acc_rawY=(Wire.read()<<8|Wire.read())/4096.0 ;
          Acc_rawZ=(Wire.read()<<8|Wire.read())/4096.0 ;
    
          
          /*---X---*/
          Acc_angle_error_x = Acc_angle_error_x + ((atan((Acc_rawY)/sqrt(pow((Acc_rawX),2) + pow((Acc_rawZ),2)))*rad_to_deg));
          /*---Y---*/
          Acc_angle_error_y = Acc_angle_error_y + ((atan(-1*(Acc_rawX)/sqrt(pow((Acc_rawY),2) + pow((Acc_rawZ),2)))*rad_to_deg)); 
          
          if(a==199)
          {
              Acc_angle_error_x = Acc_angle_error_x/200;
              Acc_angle_error_y = Acc_angle_error_y/200;
              acc_error=1;
          }
      }
    }//end of acc error calculation  
    
    Blynk.begin(auth, ssid, pass);

    if(isCallibrated== 0){
        lcd.clear();
        lcd.print(0,0, " Please Callibrate first !"); 
    }else{
        lcd.clear();
        lcd.print(0,0, "Callibrated");
    }
}



BLYNK_WRITE(V1){    // Arm Drone
    
    int ab = param.asInt();


    input_ROLL = 0;
    input_PITCH = 0;

  if(ab == 1){
    input_TL_THROTTLE = BASE_THROTTLE;
    input_TR_THROTTLE = BASE_THROTTLE;
    input_BL_THROTTLE = BASE_THROTTLE;
    input_BR_THROTTLE = BASE_THROTTLE;
    lcd.clear();
    lcd.print(0,0,"Drone Armed");
  }else{
    input_TL_THROTTLE = 1000;
    input_TR_THROTTLE = 1000;
    input_BL_THROTTLE = 1000;
    input_BR_THROTTLE = 1000;
    lcd.clear();
    lcd.print(0,0,"Drone DisArmed");
  }
}


BLYNK_WRITE(V2){    // Rotate Right;
    int ab= param.asInt();


    
    input_ROLL = 0;
    input_PITCH = 0;
  

    if(ab == 1){

      input_TL_THROTTLE += 10;
      input_TR_THROTTLE -= 10;
      input_BL_THROTTLE += 10;
      input_BR_THROTTLE -= 10;
      lcd.clear();
      lcd.print(0,0,"RIGHT ROTATING");
      
    }else{
      
      input_TL_THROTTLE -= 10;
      input_TR_THROTTLE += 10;
      input_BL_THROTTLE -= 10;
      input_BR_THROTTLE += 10;
      lcd.clear();
      lcd.print(0,0,"WELCOME");
       
    }
}

BLYNK_WRITE(V3){ // Rotate left
    int ab= param.asInt();

      
    
    input_ROLL = 0;
    input_PITCH = 0;

    if(ab == 1){

        input_TL_THROTTLE -= 10;
        input_TR_THROTTLE += 10;
        input_BL_THROTTLE -= 10;
        input_BR_THROTTLE += 10;
    
        lcd.clear();
        lcd.print(0,0,"LEFT ROTATING");
      
    }else{
        input_TL_THROTTLE += 10;
        input_TR_THROTTLE -= 10;
        input_BL_THROTTLE += 10;
        input_BR_THROTTLE -= 10;
        
        lcd.clear();
        lcd.print(0,0,"WELCOME");
        
    }
}

BLYNK_WRITE(V8){  // MOVE UP OR DOWN
    int x= param[0].asInt();
    int y =param[1].asInt();

    Serial.println(y);
  
    input_TL_THROTTLE += y;
    input_TR_THROTTLE += y;
    input_BL_THROTTLE += y;
    input_BR_THROTTLE += y;


    input_ROLL = 0;
    input_PITCH = 0;
    
    if(y > BASE_THROTTLE){
      lcd.clear();
      lcd.print(0,0,"MOVING UP");
    }else if(y  < BASE_THROTTLE){
      lcd.clear();
      lcd.print(0,0,"MOVING DOWN");  
    }else{
      lcd.clear();
      lcd.print(0,0,"WELCOME");
    }
    
    
}

BLYNK_WRITE(V10){  // PITCH AND ROLL 
  
    int x = param[0].asInt();
    int y = param[1].asInt();


    
    input_ROLL = x;
    input_PITCH = y;    
  
}




void loop() {


      Blynk.run();
      /////////////////////////////I M U/////////////////////////////////////
      timePrev = time1;  // the previous time is stored before the actual time read
      time1 = millis();  // actual time read
      elapsedTime = (time1 - timePrev) / 1000;     
      /*The tiemStep is the time that elapsed since the previous loop. 
      *This is the value that we will use in the formulas as "elapsedTime" 
      *in seconds. We work in ms so we have to divide the value by 1000 
      to obtain seconds*/
      /*Reed the values that the accelerometre gives.
      * We know that the slave adress for this IMU is 0x68 in
      * hexadecimal. For that in the RequestFrom and the 
      * begin functions we have to put this value.*/   
     //////////////////////////////////////Gyro read/////////////////////////////////////
      Wire.beginTransmission(mpuAddr);            //begin, Send the slave adress (in this case 68) 
      Wire.write(0x43);                        //First adress of the Gyro data
      Wire.endTransmission(false);
      Wire.requestFrom(mpuAddr,(uint8_t)4,true);           //We ask for just 4 registers        
      Gyr_rawX=Wire.read()<<8|Wire.read();     //Once again we shif and sum
      Gyr_rawY=Wire.read()<<8|Wire.read();
      /*Now in order to obtain the gyro data in degrees/seconds we have to divide first
      the raw value by 32.8 because that's the value that the datasheet gives us for a 1000dps range*/
      /*---X---*/
      Gyr_rawX = (Gyr_rawX/32.8) - Gyro_raw_error_x; 
      /*---Y---*/
      Gyr_rawY = (Gyr_rawY/32.8) - Gyro_raw_error_y;  
      /*Now we integrate the raw value in degrees per seconds in order to obtain the angle
      * If you multiply degrees/seconds by seconds you obtain degrees */
        /*---X---*/
      Gyro_angle_x = Gyr_rawX*elapsedTime;
      /*---X---*/
      Gyro_angle_y = Gyr_rawY*elapsedTime;
    
    
        
      
      //////////////////////////////////////Acc read/////////////////////////////////////
      Wire.beginTransmission(mpuAddr);     //begin, Send the slave adress (in this case 68) 
      Wire.write(0x3B);                 //Ask for the 0x3B register- correspond to AcX
      Wire.endTransmission(false);      //keep the transmission and next
      Wire.requestFrom(mpuAddr,(uint8_t)6,true);    //We ask for next 6 registers starting withj the 3B  
      /*We have asked for the 0x3B register. The IMU will send a brust of register.
      * The amount of register to read is specify in the requestFrom function.
      * In this case we request 6 registers. Each value of acceleration is made out of
      * two 8bits registers, low values and high values. For that we request the 6 of them  
      * and just make then sum of each pair. For that we shift to the left the high values 
      * register (<<) and make an or (|) operation to add the low values.
      If we read the datasheet, for a range of+-8g, we have to divide the raw values by 4096*/    
      Acc_rawX=(Wire.read()<<8|Wire.read())/4096.0 ; //each value needs two registres
      Acc_rawY=(Wire.read()<<8|Wire.read())/4096.0 ;
      Acc_rawZ=(Wire.read()<<8|Wire.read())/4096.0 ; 
     /*Now in order to obtain the Acc angles we use euler formula with acceleration values
     after that we substract the error value found before*/  
     /*---X---*/
     Acc_angle_x = (atan((Acc_rawY)/sqrt(pow((Acc_rawX),2) + pow((Acc_rawZ),2)))*rad_to_deg) - Acc_angle_error_x;
     /*---Y---*/
     Acc_angle_y = (atan(-1*(Acc_rawX)/sqrt(pow((Acc_rawY),2) + pow((Acc_rawZ),2)))*rad_to_deg) - Acc_angle_error_y;   
    
    
     //////////////////////////////////////Total angle and filter/////////////////////////////////////
     /*---X axis angle---*/
     Total_angle_x = 0.98 *(Total_angle_x + Gyro_angle_x) + 0.02*Acc_angle_x;
     /*---Y axis angle---*/
     Total_angle_y = 0.98 *(Total_angle_y + Gyro_angle_y) + 0.02*Acc_angle_y;
    
    
    
    
    
    
    
    /*///////////////////////////P I D///////////////////////////////////*/
//    roll_desired_angle = map(input_ROLL,1000,2000,-10,10);
//    pitch_desired_angle = map(input_PITCH,1000,2000,-10,10); 
    roll_desired_angle = input_ROLL;
    pitch_desired_angle = input_PITCH;
   
    
    /*First calculate the error between the desired angle and 
    *the real measured angle*/
    roll_error = Total_angle_y - roll_desired_angle;
    pitch_error = Total_angle_x - pitch_desired_angle;    
    /*Next the proportional value of the PID is just a proportional constant
    *multiplied by the error*/
    roll_pid_p = roll_kp*roll_error;
    pitch_pid_p = pitch_kp*pitch_error;
    /*The integral part should only act if we are close to the
    desired position but we want to fine tune the error. That's
    why I've made a if operation for an error between -2 and 2 degree.
    To integrate we just sum the previous integral value with the
    error multiplied by  the integral constant. This will integrate (increase)
    the value each loop till we reach the 0 point*/
    if(-3 < roll_error <3)
    {
      roll_pid_i = roll_pid_i+(roll_ki*roll_error);  
    }
    if(-3 < pitch_error <3)
    {
      pitch_pid_i = pitch_pid_i+(pitch_ki*pitch_error);  
    }
    /*The last part is the derivate. The derivate acts upon the speed of the error.
    As we know the speed is the amount of error that produced in a certain amount of
    time divided by that time. For taht we will use a variable called previous_error.
    We substract that value from the actual error and divide all by the elapsed time. 
    Finnaly we multiply the result by the derivate constant*/
    roll_pid_d = roll_kd*((roll_error - roll_previous_error)/elapsedTime);
    pitch_pid_d = pitch_kd*((pitch_error - pitch_previous_error)/elapsedTime);
    /*The final PID values is the sum of each of this 3 parts*/
    roll_PID = roll_pid_p + roll_pid_i + roll_pid_d;
    pitch_PID = pitch_pid_p + pitch_pid_i + pitch_pid_d;
    /*We know taht the min value of PWM signal is 1000us and the max is 2000. So that
    tells us that the PID value can/s oscilate more than -1000 and 1000 because when we
    have a value of 2000us the maximum value taht we could substract is 1000 and when
    we have a value of 1000us for the PWM signal, the maximum value that we could add is 1000
    to reach the maximum 2000us. But we don't want to act over the entire range so -+400 should be enough*/
    if(roll_PID < -400){roll_PID=-400;}
    if(roll_PID > 400) {roll_PID=400; }
    if(pitch_PID < -400){pitch_PID=-400;}
    if(pitch_PID > 400) {pitch_PID=400;}
    
    /*Finnaly we calculate the PWM width. We sum the desired throttle and the PID value*/
    pwm_T_R  = 115 + input_TR_THROTTLE - roll_PID - pitch_PID;
    pwm_B_R  = 115 + input_BR_THROTTLE - roll_PID + pitch_PID;
    pwm_B_L  = 115 + input_BL_THROTTLE + roll_PID + pitch_PID;
    pwm_T_L  = 115 + input_TL_THROTTLE + roll_PID - pitch_PID;


      if(pwm_T_R < 1100)
      {
        pwm_T_R= 1100;
      }
      if(pwm_T_R > 2000)
      {
        pwm_T_R=2000;
      }

      //Left front
      if(pwm_T_L < 1100)
      {
        pwm_T_L= 1100;
      }
      if(pwm_T_L > 2000)
      {
        pwm_T_L=2000;
      }
      
      //Right back
      if(pwm_B_R < 1100)
      {
        pwm_B_R= 1100;
      }
      if(pwm_B_R > 2000)
      {
        pwm_B_R=2000;
      }
      
      //Left back
      if(pwm_B_L < 1100)
      {
        pwm_B_L= 1100;
      }
      if(pwm_B_L > 2000)
      {
        pwm_B_L=2000;
      }

      roll_previous_error = roll_error; //Remember to store the previous error.
      pitch_previous_error = pitch_error; //Remember to store the previous error.


      Serial.print(pwm_T_L);
      Serial.print(pwm_T_R);
      Serial.print(pwm_B_L);
      Serial.print(pwm_B_R);
      

      Top_Left.writeMicroseconds(pwm_T_L);
      Top_Right.writeMicroseconds(pwm_T_R);
      Bottom_Left.writeMicroseconds(pwm_B_L);
      Bottom_Right.writeMicroseconds(pwm_B_R);
}
