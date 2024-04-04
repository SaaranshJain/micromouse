#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
// #include <PID_v1.h>

#define IN1 D4
#define IN2 D5
#define IN3 D6
#define IN4 D7

float kp = 1;
float kd = 0.005;
float ki = 0.025;

double angleX;
double error_prior;
double error;
double integral;
double pid_inp;
int zero = 0;
int state = 6;

int time_elapsed = 0;
int leftSpeed;
int rightSpeed;
int baseSpeed = 255;
// int drivef = 10;
int speedDifference; 
bool flag = false;

// enum MouseState {
//     Forward0,
//     StartTurnLeft1,
//     StartTurnRight2,
//     StartTurnAround3,
//     Turning4,
//     PostTurn5,
//     Stop6
// }



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
    // set state acc to ultrasound and floodfill
    // if (time_elapsed == 5000){
    //   state = 0;
    // }
    // else if (time_elapsed == 10000){
    //   state = 6;
    // }
    // else if (time_elapsed == 12000){
    //   state = 0;
    // }
    // else if (time_elapsed == 17000){
    //   state = 3;
    // }
    // else if (time_elapsed == 22000){
    //   state = 0;
    // }
    // else if (time_elapsed == 24000){
    //     state = 6;
    // }
    // else if (time_elapsed == 26000){
    //     state = 0;
    // }
    // else if (time_elapsed == 30000){
    //     state = 3;
    // }
    // else if (time_elapsed == 35000){
    //     state = 6;
    // }
    sensors_event_t a, g, temp;
    int leftSpeed, rightSpeed;
    mpu.getEvent(&a, &g, &temp);
    angleX += (((double) (g.gyro.x * timeStepMs)) / 1000.0) * (180 / M_PI);

    if (state == 6) {
        leftSpeed = 0;
        rightSpeed = 0;

    } 
    else if (state == 0) {
        // pid
        error = (double) zero - angleX;
        integral += ((double) (error * timeStepMs)/1000.0) ;
        double derivative =  ((double) ((error - error_prior) / timeStepMs) * 1000.0);
        pid_inp = kp*error + ki*integral + kd*derivative;

        error_prior = error;
        int baseSpeedL = 255;
        int baseSpeedR = 255; 
        int speedDifference = pid_inp * 5; 
        leftSpeed = constrain(baseSpeedL - speedDifference, 0, 255);
        rightSpeed = constrain(baseSpeedR + speedDifference, 0, 255);
        // Serial.print("inside ur mom: ");
        // Serial.println(angleX);

    } 
    else if (state == 1) {
        error_prior = 0.0;
        error = 0.0;
        integral = 0.0;
        zero = (zero + 90) % 360;
        state = 4;
    } 
    else if (state == 2) {
        error_prior = 0.0;
        error = 0.0;
        integral = 0.0;
        zero = (zero - 90) % 360;
        state = 4;
    } 
    else if (state == 3) {
        error_prior = 0.0;
        error = 0.0;
        integral = 0.0;
        zero = (zero + 180) % 360;
        state = 4;
    }

    if (state == 4) {
        // pid
        error = zero - angleX;

         if (error <= abs(0.5)) {
             state = 5;
         } else {
            integral += ((double) (error * timeStepMs)/1000.0) ;
            double derivative =  ((double) ((error - error_prior) / timeStepMs) * 1000.0);
            pid_inp = kp*error + ki*integral + kd*derivative;

            error_prior = error;
            int baseSpeedL = 255;
            int baseSpeedR = 255; 
            int speedDifference = pid_inp * 10; 
            leftSpeed = constrain(baseSpeedL - speedDifference, 0, 255);
            rightSpeed = constrain(baseSpeedR + speedDifference, 0, 255);
        }
    }

    if (state == 5) {
        // PID to realign to center of lane, use ultrasound PENDING MASSIVE
        // once equal, stop
        state = 6;
        leftSpeed = 0;
        rightSpeed = 0;
    }

    // Control motors based on calculated speeds
    // start motors after half a second
    if (!flag) {
        delay(500);
        flag=true;
    }

    analogWrite(IN1, leftSpeed);
    analogWrite(IN2, 0); 
    analogWrite(IN3, rightSpeed); 
    analogWrite(IN4, 0);

    // Serial.print("left= ");
    // Serial.println(leftSpeed);
    // Serial.print("right= " );
    // Serial.println(rightSpeed);
    // Serial.print("Main: ");
    // Serial.println(error);
    
    delay(timeStepMs);    
    time_elapsed += timeStepMs;

}
