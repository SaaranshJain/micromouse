#define TrigPin 10 //SD3
#define EchoPinL 15 //D8
#define EchoPinF 13 //D7
#define EchoPinR 12 //D6

double durationF;
double durationL;
double durationR;

double distanceF;
double distanceL;
double distanceR;

float soundvel=0.034; //in cm/us

void setup() {
  // put your setup code here, to run once:
    pinMode(TrigPin, OUTPUT); // Sets the trigPin as an Output
    pinMode(EchoPinL, INPUT); // Sets the echoPin as an Input
    pinMode(EchoPinR, INPUT); // Sets the echoPin as an Input
    pinMode(EchoPinF, INPUT); // Sets the echoPin as an Input
    Serial.begin(115200);
}

void trigger() {
    digitalWrite(TrigPin, LOW);
    delayMicroseconds(2);
    // Sets the TrigPin on HIGH state for 10 micro seconds
    digitalWrite(TrigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(TrigPin, LOW);
}

void loop() {
    // put your main code here, to run repeatedly:
  
    // Reads the EchoPin, returns the sound wave travel time in microseconds
    trigger();
    durationF = pulseIn(EchoPinF, HIGH);
    delay(100);
    trigger();
    durationR = pulseIn(EchoPinR, HIGH);
    delay(100);
    trigger();
    durationL = pulseIn(EchoPinL, HIGH);
    delay(100);
  
    // Calculate the distance
    distanceF = durationF * soundvel/2;
    distanceL = durationL * soundvel/2;
    distanceR = durationR * soundvel/2;
    // Serial.print("Sensor ");
    // Serial.print(i + 1);
    // Serial.print(": ");
    Serial.print("F: ");
    Serial.println(distanceF);
    Serial.print("L: ");
    Serial.println(distanceL);
    Serial.print("R: ");
    Serial.println(distanceR);
}
