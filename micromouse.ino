

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
// #include <PID_v1.h>

#define IN1 D4
#define IN2 D5
#define IN3 D6
#define IN4 D7

float kp = 0.1;
float kd = 0.025;
float ki = 0.001;

double angleX;
double error_prior;
double error;
double integral;
double pid_inp;

int leftSpeed;
int rightSpeed;
int baseSpeed = 255;
int speedDifference; 

const int timeStepMs = 50;

Adafruit_MPU6050 mpu;

void setup(void) {
    Serial.begin(115200);

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    while (!Serial)
        delay(10); // will pause Zero, Leonardo, etc until serial console opens

    Serial.println("Adafruit MPU6050 test!");

    if (!mpu.begin()) {
            Serial.println("Failed to find MPU6050 chip");

            while (1) {
                delay(10);
            }
    }
    Serial.println("MPU6050 Found!");

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
    delay(100);
}

void loop() {
    // analogWrite(IN1, 255);
    // analogWrite(IN2, 0);
    // analogWrite(IN3, 0);
    // analogWrite(IN4, 255);

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    if (abs(g.gyro.x) >= 0.05) {
        angleX += (((double) (g.gyro.x * timeStepMs)) / 1000.0) * (180 / M_PI);
        
        error = 0 - angleX;
        integral += ((double) (error * timeStepMs)/1000.0) ;
        double derivative =  ((double) ((error - error_prior) / timeStepMs) * 1000.0);
        pid_inp = kp*error + ki*integral + kd*derivative;

        error_prior = error;
        

    }

    // pid
    error = 0 - angleX;
    integral += ((double) (error * timeStepMs)/1000.0) ;
    double derivative =  ((double) ((error - error_prior) / timeStepMs) * 1000.0);
    pid_inp = kp*error + ki*integral + kd*derivative;

    error_prior = error;
    int baseSpeed = 255; 
    int speedDifference = pid_inp * 10; 
    int leftSpeed = constrain(baseSpeed - speedDifference, 0, 255);
    int rightSpeed = constrain(baseSpeed + speedDifference, 0, 255);

    //Control motors based on calculated speeds
    analogWrite(IN1, leftSpeed);
    analogWrite(IN2, 0); 
    analogWrite(IN3, 0); 
    analogWrite(IN4, rightSpeed);

    Serial.print("left= ");
    Serial.println(leftSpeed);
    Serial.print("right= " );
    Serial.println(rightSpeed);
    delay(timeStepMs);

  
    
}
