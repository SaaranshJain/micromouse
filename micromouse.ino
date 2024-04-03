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
double zero = 0.0;

int leftSpeed;
int rightSpeed;
int baseSpeed = 255;
int speedDifference; 
bool flag = false;

enum MouseState {
    Forward,
    StartTurnLeft,
    StartTurnRight,
    StartTurnAround,
    Turning,
    PostTurn,
    Stop
}

MouseState state = MouseState.Stop;

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
    sensors_event_t a, g, temp;
    int leftSpeed, rightSpeed;
    mpu.getEvent(&a, &g, &temp);

    if (state == MouseState.Stop) {
        leftSpeed = 0;
        rightSpeed = 0;
    } else if (state == MouseState.Forward) {
        // if (abs(g.gyro.x) >= 0.05) {
        //     angleX += (((double) (g.gyro.x * timeStepMs)) / 1000.0) * (180 / M_PI);
            
        //     error = 0 - angleX;
        //     integral += ((double) (error * timeStepMs)/1000.0) ;
        //     double derivative =  ((double) ((error - error_prior) / timeStepMs) * 1000.0);
        //     pid_inp = kp*error + ki*integral + kd*derivative;

        //     error_prior = error;
            

        // }

        // pid
        error = zero - angleX;
        integral += ((double) (error * timeStepMs)/1000.0) ;
        double derivative =  ((double) ((error - error_prior) / timeStepMs) * 1000.0);
        pid_inp = kp*error + ki*integral + kd*derivative;

        error_prior = error;
        int baseSpeedL = 200;
        int baseSpeedR = 200; 
        int speedDifference = pid_inp * 5; 
        leftSpeed = constrain(baseSpeedL - speedDifference, 0, 255);
        rightSpeed = constrain(baseSpeedR + speedDifference, 0, 255);
    } else if (state == MouseState.StartTurnLeft) {
        error_prior = 0.0;
        error = 0.0;
        integral = 0.0;
        zero = (zero + 90.0) % 360.0;
        state = MouseState.Turning;
    } else if (state == MouseState.StartTurnRight) {
        error_prior = 0.0;
        error = 0.0;
        integral = 0.0;
        zero = (zero - 90.0) % 360.0;
        state = MouseState.Turning;
    } else if (state == MouseState.StartTurnAround) {
        error_prior = 0.0;
        error = 0.0;
        integral = 0.0;
        zero = (zero + 180.0) % 360.0;
        state = MouseState.Turning;
    }

    if (state == MouseState.Turning) {
        // if (abs(g.gyro.x) >= 0.05) {
        //     angleX += (((double) (g.gyro.x * timeStepMs)) / 1000.0) * (180 / M_PI);
            
        //     error = 0 - angleX;
        //     integral += ((double) (error * timeStepMs)/1000.0) ;
        //     double derivative =  ((double) ((error - error_prior) / timeStepMs) * 1000.0);
        //     pid_inp = kp*error + ki*integral + kd*derivative;

        //     error_prior = error;
            

        // }

        // pid
        error = zero - angleX;

        if (error <= 0.5) {
            state = MouseState.PostTurn;
        } else {
            integral += ((double) (error * timeStepMs)/1000.0) ;
            double derivative =  ((double) ((error - error_prior) / timeStepMs) * 1000.0);
            pid_inp = kp*error + ki*integral + kd*derivative;

            error_prior = error;
            int baseSpeedL = 200;
            int baseSpeedR = 200; 
            int speedDifference = pid_inp * 5; 
            leftSpeed = constrain(baseSpeedL - speedDifference, 0, 255);
            rightSpeed = constrain(baseSpeedR + speedDifference, 0, 255);
        }
    }

    if (state == MouseState.PostTurn) {
        // PID to realign to center of lane, use ultrasound PENDING MASSIVE
        // once equal, stop
        state = MouseState.Stop;
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
    analogWrite(IN3, 0); 
    analogWrite(IN4, rightSpeed);

    Serial.print("left= ");
    Serial.println(leftSpeed);
    Serial.print("right= " );
    Serial.println(rightSpeed);
    delay(timeStepMs);    
}
