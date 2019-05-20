#include <PID_v1.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>

//temperature sensors
#define TemperatureSensorsPIN 2
DeviceAddress mainTankSensor = {0x28, 0x54, 0xF7, 0x45, 0x92, 0x11, 0x2, 0x89};
DeviceAddress Chem1Sensor =    {0x28, 0xAA, 0x8C, 0x48, 0x1A, 0x13, 0x2, 0xBE};
DeviceAddress Chem2Sensor =    {0x28, 0xAA, 0x4F, 0xF4, 0x3C, 0x14, 0x1, 0x98};
DeviceAddress Chem3Sensor =    {0x28, 0xAA, 0x90, 0x10 , 0x3D, 0x14, 0x1, 0x45};
#define maxTemp 60

//LEDs
#define LEDHeater 8
#define LEDMainTank 9
#define LEDChem1 10
#define LEDChem2 11
#define LEDChem3 12

//Heater
#define ON true
#define OFF false
#define heaterPIN 3

//Buttons and Potentiometer
#define Potentiometer A7
#define Button 7
#define NOTPRESSEDBUTTON HIGH
#define PRESSEDBUTTON LOW

double mainTank = 0;
double Chem1, Chem2, Chem3;

double Target = 0;
double Kp = 1, Ki = 10, Kd = 1;
double PIDinput = 0;
double heaterPower = 0;
boolean heaterStatus = OFF;

double c1Error, c2Error;
double c3Temp, c3Error;

byte upArrow[] =       {B00100, B01110, B10101, B00100, B00100, B00100, B00100, B00100};
byte downArrow[] =     {B00100, B00100, B00100, B00100, B00100, B10101, B01110, B00100};
byte degreeSimbol[] =  {B00110, B01001, B01001, B00110, B00000, B00000, B00000, B00000};
byte errorSimbol[] =   {B00100, B00100, B11111, B00100, B00100, B00000, B11111, B00000};



boolean menuFinished = false;
boolean menuValueChanged = false;
boolean firstTimeMenu = true;

OneWire oneWire(TemperatureSensorsPIN);
DallasTemperature sensors(&oneWire);
LiquidCrystal_I2C lcd(0x27, 16, 4);
PID myPID(&PIDinput, &heaterPower, &Target, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(9600);
  sensors.begin();
  lcd.begin();
  myPID.SetMode(AUTOMATIC);
  myPID.SetTunings(Kp, Ki, Kd);

  lcd.setBacklight((uint8_t)1);
  lcd.createChar(0, upArrow);
  lcd.createChar(1, downArrow);
  lcd.createChar(2, degreeSimbol);
  lcd.createChar(3, errorSimbol);

  pinMode(LEDHeater, OUTPUT);
  pinMode(LEDMainTank, OUTPUT);
  pinMode(LEDChem1, OUTPUT);
  pinMode(LEDChem2, OUTPUT);
  pinMode(LEDChem3, OUTPUT);

  pinMode(heaterPIN, OUTPUT);

  pinMode(Button, INPUT_PULLUP);



  lcd.setCursor(0, 0);
  lcd.print("Welcome");

  delay(1000);
}

void loop() {
  if (menuFinished == true) {
    updateSensors();
    heater();
    printLCD();
    updateLEDs();
    //delay(50);
  } else {
    menu();
  }
  if (digitalRead(Button) == PRESSEDBUTTON) {
    menuFinished = false;
    lcd.clear();
    delay(1000);
  }

}

void turnOff(){
  analogWrite(heaterPIN, 0);

  digitalWrite(LEDHeater, LOW);
  digitalWrite(LEDMainTank, LOW);
  digitalWrite(LEDChem1, LOW);
  digitalWrite(LEDChem2, LOW);
  digitalWrite(LEDChem3, LOW);

}


void menu() {
  turnOff();

  lcd.clear();
  double potentiometerValue;
  double tempValue;
  menuValueChanged = false;
  //double potentiometerValue;

  tempValue = analogRead(Potentiometer);
  lcd.setCursor(0, 0);
  lcd.print("Set Target Temp: ");

  while (digitalRead(Button) == NOTPRESSEDBUTTON) {
    if (analogRead(Potentiometer) < tempValue + 10 && analogRead(Potentiometer) > tempValue - 10 && menuValueChanged == false && firstTimeMenu == false) {
      potentiometerValue = Target;
    } else {
      int tempPotentiometerValue = analogRead(Potentiometer);
      potentiometerValue = map(tempPotentiometerValue, 0, 1023, 0, maxTemp);
      menuValueChanged = true;
    }
    lcd.setCursor(0, 1);
    lcd.print((int)potentiometerValue);
    lcd.print(" ");
    lcd.write(2);
    lcd.print("C ");
  }

  Target = potentiometerValue;
  menuValueChanged = false;
  lcd.clear();
  delay(500);


  tempValue = analogRead(Potentiometer);
  lcd.setCursor(0, 0);
  lcd.print("Set ");
  lcd.write(3);
  lcd.print(" Error C1: ");

  while (digitalRead(Button) == NOTPRESSEDBUTTON) {
    if (analogRead(Potentiometer) < tempValue + 10 && analogRead(Potentiometer) > tempValue - 10 && menuValueChanged == false && firstTimeMenu == false) {
      potentiometerValue = c1Error;
    } else {
      menuValueChanged = true;
      int tempPotentiometerValue = analogRead(Potentiometer);
      potentiometerValue = map(tempPotentiometerValue, 0, 1023, 0, 50);
      potentiometerValue = potentiometerValue / 10;
    }

    lcd.setCursor(0, 1);
    lcd.print(potentiometerValue);
    lcd.print(" ");
    lcd.write(2);
    lcd.print("C ");
    lcd.setCursor(0, 3);
    lcd.print(Target - potentiometerValue);
    lcd.print("-");
    lcd.print(Target + potentiometerValue);
    lcd.print(" ");
    lcd.write(2);
    lcd.print("C ");

  }
  c1Error = potentiometerValue;
  menuValueChanged = false;
  lcd.clear();
  delay(500);


  tempValue = analogRead(Potentiometer);
  lcd.setCursor(0, 0);
  lcd.print("Set ");
  lcd.write(3);
  lcd.print(" Error C2: ");

  while (digitalRead(Button) == NOTPRESSEDBUTTON) {
    if (analogRead(Potentiometer) < tempValue + 10 && analogRead(Potentiometer) > tempValue - 10 && menuValueChanged == false && firstTimeMenu == false) {
      potentiometerValue = c2Error;
    } else {
      int tempPotentiometerValue = analogRead(Potentiometer);
      potentiometerValue = map(tempPotentiometerValue, 0, 1023, 0, 50);
      potentiometerValue = potentiometerValue / 10;
    }

    lcd.setCursor(0, 1);
    lcd.print(potentiometerValue);
    lcd.print(" ");
    lcd.write(2);
    lcd.print("C ");
    lcd.setCursor(0, 3);
    lcd.print(Target - potentiometerValue);
    lcd.print("-");
    lcd.print(Target + potentiometerValue);
    lcd.print(" ");
    lcd.write(2);
    lcd.print("C ");
  }


  c2Error = potentiometerValue;
  menuValueChanged = false;
  lcd.clear();
  delay(500);


  tempValue = analogRead(Potentiometer);
  lcd.setCursor(0, 0);
  lcd.print("Set C3 Temp: ");

  while (digitalRead(Button) == NOTPRESSEDBUTTON) {
    if (analogRead(Potentiometer) < tempValue + 10 && analogRead(Potentiometer) > tempValue - 10 && menuValueChanged == false && firstTimeMenu == false) {
      potentiometerValue = c3Temp;
    } else {
      int tempPotentiometerValue = analogRead(Potentiometer);
      potentiometerValue = map(tempPotentiometerValue, 0, 1023, 150, 300);
      potentiometerValue = potentiometerValue / 10;
    }

    lcd.setCursor(0, 1);
    lcd.print(potentiometerValue);
    lcd.print(" ");
    lcd.write(2);
    lcd.print("C ");
  }
  c3Temp = potentiometerValue;
  menuValueChanged = false;
  lcd.clear();
  delay(500);


  tempValue = analogRead(Potentiometer);
  lcd.setCursor(0, 0);
  lcd.print("Set ");
  lcd.write(3);
  lcd.print(" Error C3: ");

  while (digitalRead(Button) == NOTPRESSEDBUTTON) {
    if (analogRead(Potentiometer) < tempValue + 10 && analogRead(Potentiometer) > tempValue - 10 && menuValueChanged == false && firstTimeMenu == false) {
      potentiometerValue = c3Error;
    } else {
      int tempPotentiometerValue = analogRead(Potentiometer);
      potentiometerValue = map(tempPotentiometerValue, 0, 1023, 0, 50);
      potentiometerValue = potentiometerValue / 10;
    }


    lcd.setCursor(0, 1);
    lcd.print(potentiometerValue);
    lcd.print(" ");
    lcd.write(2);
    lcd.print("C ");
    lcd.setCursor(0, 3);
    lcd.print(c3Temp - potentiometerValue);
    lcd.print("-");
    lcd.print(c3Temp + potentiometerValue);
    lcd.print(" ");
    lcd.write(2);
    lcd.print("C ");
  }
  c3Error = potentiometerValue;
  menuValueChanged = false;
  lcd.clear();
  delay(500);

  lcd.setCursor(0, 0);
  lcd.print("Main: ");
  lcd.print(Target);

  lcd.setCursor(0, 1);
  lcd.print("C1: ");
  lcd.print(Target);
  lcd.print(" ");
  lcd.write(3);
  lcd.print(c1Error);

  lcd.setCursor(0, 2);
  lcd.print("C2: ");
  lcd.print(Target);
  lcd.print(" ");
  lcd.write(3);
  lcd.print(c2Error);

  lcd.setCursor(0, 3);
  lcd.print("C3: ");
  lcd.print(c3Temp);
  lcd.print(" ");
  lcd.write(3);
  lcd.print(c3Error);

  while (digitalRead(Button) == NOTPRESSEDBUTTON) {
  }
  delay(500);
  lcd.clear();
  menuFinished = true;
  firstTimeMenu = false;

}

void heater() {
  //PIDinput = map(mainTank, 0, maxTemp, 0, 255);
  PIDinput = mainTank;
  Serial.println(PIDinput);
  myPID.Compute();

  if (heaterPower <= 30) {
    heaterStatus = OFF;
  } else {
    heaterStatus = ON;
  }
  int heaterPowerScaled = map(heaterPower, 0, 255, 0, 127);
  //heaterPower = map(heaterPower, 0, 255/2,0,255);
  Serial.println(heaterPowerScaled);
  analogWrite(heaterPIN, heaterPowerScaled);
}


void printLCD() {

  //lcd.setCursor(0, 0);


  lcd.setCursor(0, 0);
  lcd.print("M: ");
  lcd.print(mainTank);
  lcd.setCursor(9, 0);
  if ((Target - mainTank) > 0) {
    lcd.write(0);
  } else if (Target - mainTank < 0) {
    lcd.write(1);
  }else{
    lcd.print(" ");
  }
  lcd.setCursor(10, 0);
  lcd.print((abs(Target - mainTank) * 10) / 10);
  lcd.print(" ");
  lcd.print("T:");
  lcd.print(int(Target));

  lcd.setCursor(0, 1);
  lcd.print("C1:");
  if(Chem1 > 0){
    lcd.print(Chem1);
    lcd.setCursor(9, 1);
    if (Chem1 < Target-c1Error) {
      lcd.write(0);
      lcd.setCursor(10, 1);
      lcd.print((abs((Target-c1Error) - Chem1) * 10) / 10);
      lcd.print(" ");
    } else if (Chem1 > Target+c1Error) {
      lcd.write(1);
      lcd.setCursor(10, 1);
      lcd.print((abs((Target+c1Error) - Chem1) * 10) / 10);
      lcd.print(" ");
    }else {
      lcd.setCursor(8, 1);
      lcd.print(" OK   ");
    }
  }else{
      lcd.print("Err         ");
  }





  lcd.setCursor(0, 2);
  lcd.print("C2:");
  if(Chem2 > 0){
    lcd.print(Chem2);
    lcd.setCursor(9, 2);
    if (Chem2 < Target-c2Error) {
      lcd.write(0);
      lcd.setCursor(10, 2);
      lcd.print((abs((Target-c2Error) - Chem2) * 10) / 10);
      lcd.print(" ");
    } else if (Chem2 > Target+c2Error) {
      lcd.write(1);
      lcd.setCursor(10, 2);
      lcd.print((abs((Target+c2Error) - Chem2) * 10) / 10);
      lcd.print(" ");
    }else {
      lcd.setCursor(8, 2);
      lcd.print(" OK   ");
    }
  }else{
      lcd.print("Err         ");
  }



  lcd.setCursor(0, 3);
  lcd.print("C3:");
  if(Chem3 > 0){
    lcd.print(Chem3);

    lcd.setCursor(9, 3);
    if (Chem3 < c3Temp-c3Error) {
      lcd.write(0);
      lcd.setCursor(10, 3);
      lcd.print((abs((c3Temp-c3Error) - Chem3) * 10) / 10);
      lcd.print(" ");
    } else if (Chem3 > c3Temp+c3Error) {
      lcd.write(1);
      lcd.setCursor(10, 3);
      lcd.print((abs((c3Temp+c3Error) - Chem3) * 10) / 10);
      lcd.print(" ");
    }else {
      lcd.setCursor(8, 3);
      lcd.print(" OK   ");
    }
  }else{
    lcd.print("Err         ");
  }


  lcd.setCursor(15, 2);
  lcd.print(" P");
  lcd.print(map(heaterPower, 0, 255, 0, 100));
  if (map(heaterPower, 0, 255, 0, 100) < 100 && map(heaterPower, 0, 255, 0, 100) > 9) {
    lcd.print("%");
  } else if(map(heaterPower, 0, 255, 0, 100) < 10){
    lcd.print("% ");
  }

}


void updateSensors() {
  sensors.requestTemperatures();
  mainTank = sensors.getTempC(mainTankSensor);
  Chem3 = sensors.getTempC(Chem3Sensor);
  Chem2 = sensors.getTempC(Chem2Sensor);
  Chem1 = sensors.getTempC(Chem1Sensor);

  if (mainTank < 0) {
    updateSensors();
    return;
  }


  //int potentiometerTargetue = analogRead(Potentiometer);
  //Target = map(potentiometerTargetue, 0, 1023, 0, maxTemp);
}

void updateLEDs() {
  if (heaterStatus == ON) {
    digitalWrite(LEDHeater, HIGH);
  } else {
    digitalWrite(LEDHeater, LOW);
  }

  if(mainTank >= Target){
    digitalWrite(LEDMainTank, HIGH);
  }else{
    digitalWrite(LEDMainTank, LOW);
  }

  if(Chem1 >= Target - c1Error && Chem1 <= Target + c1Error ){
    digitalWrite(LEDChem1, HIGH);
  } else {
    digitalWrite(LEDChem1, LOW);
  }

  if(Chem2 >= Target - c2Error && Chem2 <= Target + c2Error ){
    digitalWrite(LEDChem2, HIGH);
  } else {
    digitalWrite(LEDChem2, LOW);
  }

  if(Chem3 >= c3Temp - c3Error && Chem3 <= c3Temp + c3Error ){
    digitalWrite(LEDChem3, HIGH);
  } else {
    digitalWrite(LEDChem3, LOW);
  }

}
