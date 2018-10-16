#include <SparkFunLSM9DS1.h>

LSM9DS1 IMU;

//Accelerometer Data
float acc1[60]; //acceleration magnitude during BW1 firing
float acc2[60]; //acceleration magnitude during BW2 firing
float acc3[60]; //acceleration magnitude during BW3 firing
float ax = 0.0;
float ay = 0.0;
float az = 0.0;

void burnAntennaOne() {
  pinMode(BURN_RELAY_A, OUTPUT);
  digitalWrite(BURN_RELAY_A, HIGH);
  pinMode(ENAB_BURN1, OUTPUT);
  digitalWrite(ENAB_BURN1, LOW);

  int counter = millis();  
  while((millis()-counter) < 2000 ) {
    digitalWrite(ENAB_BURN1, HIGH);
    delay(3);
    digitalWrite(ENAB_BURN1, LOW);
    delay(47); 
  }
  digitalWrite(ENAB_BURN1, HIGH);
  delay(2);
  digitalWrite(BURN_RELAY_A, LOW);
  digitalWrite(ENAB_BURN1, LOW);
}

void burnAntennaTwo() {
  pinMode(BURN_RELAY_A, OUTPUT);
  digitalWrite(BURN_RELAY_A, HIGH);
  pinMode(ENAB_BURN2, OUTPUT);
  digitalWrite(ENAB_BURN2, LOW);

  int counter = millis();  
  while((millis()-counter) < 2000 ) {
    digitalWrite(ENAB_BURN2, HIGH);
    delay(3);
    digitalWrite(ENAB_BURN2, LOW);
    delay(47); 
  }
  digitalWrite(ENAB_BURN2, HIGH);
  delay(2);
  digitalWrite(BURN_RELAY_A, LOW);
  digitalWrite(ENAB_BURN2, LOW);
}

void burnSpriteOne() {
  IMU.begin(); //turn on IMU
  IMU.sleepGyro(true); //turn off gyro to save power

  pinMode(BURN_RELAY_B, OUTPUT);
  digitalWrite(BURN_RELAY_B, HIGH);
  pinMode(ENAB_BURN3, OUTPUT);
  digitalWrite(ENAB_BURN3, LOW);

  int counter = millis();
  int samp = 0;
  while((millis()-counter) < 2000 ) {
    digitalWrite(ENAB_BURN3, HIGH);
    delay(3);
    digitalWrite(ENAB_BURN3, LOW);
    delay(47);

    IMU.readAccel();
    ax = IMU.calcAccel(IMU.ax);
    ay = IMU.calcAccel(IMU.ay);
    az = IMU.calcAccel(IMU.az);
    acc1[samp++] = ax*ax + ay*ay + az*az;
  }
  digitalWrite(ENAB_BURN3, HIGH);
  delay(2);
  digitalWrite(BURN_RELAY_B, LOW);
  digitalWrite(ENAB_BURN3, LOW);

  //keep filling in
  delay(48);
  while(samp < 60) {
    IMU.readAccel();
    ax = IMU.calcAccel(IMU.ax);
    ay = IMU.calcAccel(IMU.ay);
    az = IMU.calcAccel(IMU.az);
    acc1[samp++] = ax*ax + ay*ay + az*az;
    delay(50);
  }
}

void burnSpriteTwo() {
  IMU.begin(); //turn on IMU
  IMU.sleepGyro(true); //turn off gyro to save power

  pinMode(BURN_RELAY_B, OUTPUT);
  digitalWrite(BURN_RELAY_B, HIGH);
  pinMode(ENAB_BURN4, OUTPUT);
  digitalWrite(ENAB_BURN4, LOW);
  
  int counter = millis(); 
  int samp = 0; 
  while((millis()-counter) < 2000 ) {
    digitalWrite(ENAB_BURN4, HIGH);
    delay(3);
    digitalWrite(ENAB_BURN4, LOW);
    delay(47); 

    IMU.readAccel();
    ax = IMU.calcAccel(IMU.ax);
    ay = IMU.calcAccel(IMU.ay);
    az = IMU.calcAccel(IMU.az);
    acc2[samp++] = ax*ax + ay*ay + az*az;
  }
  digitalWrite(ENAB_BURN4, HIGH);
  delay(2);
  digitalWrite(BURN_RELAY_B, LOW);
  digitalWrite(ENAB_BURN4, LOW);

  //keep filling in
  delay(48);
  while(samp < 60) {
    IMU.readAccel();
    ax = IMU.calcAccel(IMU.ax);
    ay = IMU.calcAccel(IMU.ay);
    az = IMU.calcAccel(IMU.az);
    acc3[samp++] = ax*ax + ay*ay + az*az;
    delay(50);
  }
}

void burnSpriteThree(){
  IMU.begin(); //turn on IMU
  IMU.sleepGyro(true); //turn off gyro to save power

  pinMode(BURN_RELAY_B, OUTPUT);
  digitalWrite(BURN_RELAY_B, HIGH);
  pinMode(ENAB_BURN5, OUTPUT);
  digitalWrite(ENAB_BURN5, LOW);

  int counter = millis();
  int samp = 0;
  while((millis()-counter) < 2000 ) {
    digitalWrite(ENAB_BURN5, HIGH);
    delay(3);
    digitalWrite(ENAB_BURN5, LOW);
    delay(47); 

    IMU.readAccel();
    ax = IMU.calcAccel(IMU.ax);
    ay = IMU.calcAccel(IMU.ay);
    az = IMU.calcAccel(IMU.az);
    acc3[samp++] = ax*ax + ay*ay + az*az;
  }
  digitalWrite(ENAB_BURN5, HIGH);
  delay(2);
  digitalWrite(BURN_RELAY_B, LOW);
  digitalWrite(ENAB_BURN5, LOW);

  //keep filling in
  delay(48);
  while(samp < 60) {
    IMU.readAccel();
    ax = IMU.calcAccel(IMU.ax);
    ay = IMU.calcAccel(IMU.ay);
    az = IMU.calcAccel(IMU.az);
    acc2[samp++] = ax*ax + ay*ay + az*az;
    delay(50);
  }
}
