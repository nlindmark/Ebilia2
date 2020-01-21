#include <Arduino.h>
#include "BluetoothSerial.h"
#include "ServoEasing.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif




// Rotate forward: EN = 1, RPWM = PWM, LPWM = 1, DIS = vacant  
// Rotate reverse: EN = 1, RPWM = 1, LPWM = PWM, DIS = vacant 
// Parking and brake: EN = 1, RPWM = 1, LPWM = 1, DIS = vacant 
// Parking but not brake: EN = 0, RPWM = 1, LPWM = 1, DIS = vacant
// Prohibit the use: EN = X, RPWM = X, LPWM = X, DIS = 1 

void forward(uint32_t speed);
void forward(int time, uint32_t speed);
void backward(uint32_t speed);
void backward(int time, uint32_t speed);

void turnLeft();
void turnRight();
void stop();
void setupPwm(uint8_t channel, int in1);
long map2(long x, long in_min, long in_max, long out_min, long out_max);

void driveServo(void* pvParameter);
void update(void* pvParameter);

void deSerialize();


// Servo myservo; // create servo object to control a servo

int pos = 0; // variable to store the servo position

const int servoChan = 0;
static const int servoPin = 14;
int ledPin = 13;

const int Achan = 5;
int Aen = 26;  // A0
int Ain0 = 21;
int Ain1 = 15;

const int Bchan = 6;
int Ben = 25; // A1
int Bin0 = 32;
int Bin1 = 33;

BluetoothSerial SerialBT;

#define MIN_ANGLE 0
#define MAX_ANGLE 180
long refAngle = (MAX_ANGLE + MIN_ANGLE) / 2;
long actAngle = refAngle;

#define MIN_SPEED -255
#define MAX_SPEED 255
long refSpeed = (MAX_SPEED + MIN_SPEED) / 2;
long actSpeed = refSpeed;

ServoEasing steerServo;

char data[4];
int ind = 0;

void setup()
{

    //myservo.setPeriodHertz(330);          // 330 hz servo
    //myservo.attach(servoPin, 1000,   2000); 


    pinMode(ledPin, OUTPUT);

    // Servo
    // ledcSetup(servoChan, 330, 8);
    // ledcSetup(servoChan, 50, 16);
    // ledcAttachPin(servoPin, servoChan);

    steerServo.attach(servoPin);
    steerServo.setEasingType(EASE_LINEAR
    );

   
    // A & B Motor
    ledcSetup(Achan, 5000, 8);
    ledcSetup(Bchan, 5000, 8);
    ledcAttachPin(Aen, Achan);
    ledcAttachPin(Ben, Bchan);
    
    
    pinMode(Ain0, OUTPUT);
    pinMode(Ain1, OUTPUT);
    pinMode(Bin0, OUTPUT);
    pinMode(Bin1, OUTPUT);

    stop();

    // timer1.attach_ms(100, driveServo);
    
    Serial.begin(9600);
    SerialBT.begin("Ebilia"); //Bluetooth device name
    Serial.println("The device started, now you can pair it with bluetooth!");

    xTaskCreate(
                    driveServo,          /* Task function. */
                    "driveServo",        /* String with name of task. */
                    1000,            /* Stack size in bytes. */
                    NULL,             /* Parameter passed as input of the task */
                    1,                /* Priority of the task. */
                    NULL);            /* Task handle. */





    xTaskCreate(
                    update,          /* Task function. */
                    "update",        /* String with name of task. */
                    1000,            /* Stack size in bytes. */
                    NULL,             /* Parameter passed as input of the task */
                    1,                /* Priority of the task. */
                    NULL);            /* Task handle. */

}



void loop()
{

    
    //turnLeft();    
    //forward(500, 0x80);
    //turnRight();
    //backward(500, 0x80);

    
    
    if (Serial.available()) {

        SerialBT.write(Serial.read());

    }

    

    if (SerialBT.available()) {

        deSerialize();

    }



  

    
}



void deSerialize(){

    int token = SerialBT.read();

    

    if(token == 'A'){
        data[ind] = '\n';
        refAngle = map(atol(data),-50, 50,MIN_ANGLE, MAX_ANGLE);
        ind = 0;   

        
        
        steerServo.startEaseTo(refAngle, 180, false);
           
        

        // Serial.println(refAngle);

    } else if(token == 'S'){

        data[ind] = '\n';
        refSpeed = map(atol(data), -50, 50, MIN_SPEED, MAX_SPEED);
        ind = 0;         

        //  Serial.println(refSpeed);

    } else {
        data[ind] = token;
        
        ind++;
    }

}


// void turnLeft()
// {
//     digitalWrite(ledPin, HIGH);
//     for (pos = 45; pos <= 90; pos += 1)
//     { 
//         ledcWrite(servoChan,pos);
        
//         delay(15);          
//     }
//     digitalWrite(ledPin, LOW);
//     delay(100);
// }

// void turnRight()
// {
//     digitalWrite(ledPin, HIGH);
//     for (pos = 90; pos >= 45; pos -= 1)
//     {                       
//         ledcWrite(servoChan,pos);
//         delay(15);          
//     }
//     digitalWrite(ledPin, LOW);
//     delay(100);
// }



void forward(uint32_t speed)
{
    
    digitalWrite(Ain0, HIGH);
    digitalWrite(Ain1, LOW);
    digitalWrite(Bin0, HIGH);
    digitalWrite(Bin1, LOW);

    
    ledcWrite(Achan,speed);
    ledcWrite(Bchan,speed);
}

void backward(uint32_t speed)
{
    digitalWrite(Ain0, LOW);
    digitalWrite(Ain1, HIGH);
    digitalWrite(Bin0, LOW);
    digitalWrite(Bin1, HIGH);
    ledcWrite(Achan,speed);
    ledcWrite(Bchan,speed);
}


// void forward(int time, uint32_t speed)
// {
//     digitalWrite(ledPin, HIGH);
//     forward(speed);
//     delay(time);
//     stop();
//     digitalWrite(ledPin, LOW);
//     delay(100);
// }

// void backward(int time, uint32_t speed)
// {
//     digitalWrite(ledPin, HIGH);
//     backward(speed);
//     delay(time);
//     stop();
//     digitalWrite(ledPin, LOW);
//     delay(100);
 
// }

void stop()
{
    digitalWrite(Ain0, LOW);
    digitalWrite(Ain1, LOW);
    digitalWrite(Bin0, LOW);
    digitalWrite(Bin1, LOW);
    ledcWrite(Achan,0);
    ledcWrite(Bchan,0);
}

void driveServo(void* pvParameter){
    // uint32_t duty;

    while(1)
    {
        if(refAngle > actAngle){
            actAngle++;
        } else if(refAngle < actAngle){
            actAngle--;
        }

        

        // steerServo.update();

        // steerServo.write(actAngle);
        

        // duty = (((actAngle/180.0) * 2000)/20000.0*65536.0) + 1634;

        // ledcWrite(servoChan,duty);

        if(refSpeed > actSpeed){
            actSpeed++;
        } else if (refSpeed < actSpeed){
            actSpeed--;
        }
        if(actSpeed > 0){
            forward(actSpeed);
        } else if (actSpeed < 0){
            backward(-actSpeed);
        }

        

        Serial.println(actAngle);
        vTaskDelay(10 / portTICK_RATE_MS);
    }

}

void update(void* pvParameter){

    while(1)
    {

        do {
                // First do the delay, then check for update, since we are likely called directly after start and there is nothing to move yet
                vTaskDelay((REFRESH_INTERVAL / 1000) / portTICK_RATE_MS);  // 20 ms - REFRESH_INTERVAL is in Microseconds
            } while (!steerServo.update());  

    }
}
// long map2(long x, long in_min, long in_max, long out_min, long out_max)
// {
//   if(x < in_min){
//     x = in_min;
//   } else if (x> in_max){
//     x = in_max;
//   }

//   return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
// }
 