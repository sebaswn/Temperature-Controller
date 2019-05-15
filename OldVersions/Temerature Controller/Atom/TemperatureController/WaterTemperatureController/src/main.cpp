#include <PID_v1.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>

//temperature sensors. All Sensors are connected to pin 2
//Addresses for each device are read and recorded using another script
#define TemperatureSensors 2
DeviceAddress mainTankSensor = {0x28, 0x54, 0xF7, 0x45, 0x92, 0x11, 0x2, 0x89};
DeviceAddress Tank1Sensor =    {0x28, 0xAA, 0x8C, 0x48, 0x1A, 0x13, 0x2, 0xBE};
DeviceAddress Tank2Sensor =    {0x28, 0xAA, 0x4F, 0xF4, 0x3C, 0x14, 0x1, 0x98};
DeviceAddress Tank3Sensor =    {0x28, 0xAA, 0x90, 0x10 ,0x3D, 0x14, 0x1, 0x45};
#define maxTemp 60

//LEDs
#define LEDHeater 8
#define LEDMainTank 9
#define LEDTank1 10
#define LEDTank2 11
#define LEDTank3 12

//Heater
#define ON true
#define OFF false
#define heaterPIN 3

//Buttons and Potentiometer
#define Potentiometer A7

//Menu Variables:
boolean menuRunning = true; //true if menu is running
boolean errorTank1Chosen = false;
boolean errorTank2Chosen = false;
boolean MaxTank3Chosen = false;
boolean MinTank3Chosen = false;

double errorTank1 = 0.2;
double errorTank2 = 2;
double MaxTank3 = 20;
double MinTank3 = 25;

double mainTank=0;
double Tank1;
double Tank2;
double Tank3;

double Target = 0;
double Kp=1, Ki=10, Kd=1;
double PIDinput = 0;
double heaterPower = 0;
boolean heaterStatus = OFF;

byte upArrow[] = { B00100,B01110,B10101,B00100,B00100,B00100,B00100,B00100};
byte downArrow[] = {B00100,B00100,B00100,B00100,B00100,B10101,B01110,B00100};



OneWire oneWire(TemperatureSensors);
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

  pinMode(LEDHeater, OUTPUT);
  pinMode(LEDMainTank, OUTPUT);
  pinMode(LEDTank1, OUTPUT);
  pinMode(LEDTank2, OUTPUT);
  pinMode(LEDTank3, OUTPUT);

  pinMode(heaterPIN, OUTPUT);
} // AllSetUp

void heater(){
  //PIDinput = map(mainTank, 0, maxTemp, 0, 255);
  PIDinput = mainTank;
  Serial.println(PIDinput);
  myPID.Compute();

  if(heaterPower <= 30){
    heaterStatus = OFF;
  }else{
    heaterStatus = ON;
  }
  Serial.println(heaterPower);
  analogWrite(heaterPIN,heaterPower);
} // Checks to turn on or off heater. Includes PID controller.

void printLCD(){
  lcd.setCursor(0,0);
  lcd.print("T:");
  lcd.print(int(Target));

  lcd.setCursor(5,0);
  lcd.print("M:");
  lcd.print(mainTank);
  lcd.setCursor(13,0);
  if((Target-mainTank) > 0){
    lcd.write(0);
  }else if(Target-mainTank <0){
    lcd.write(1);
  }
  lcd.setCursor(14,0);
  lcd.print(round(abs(Target-mainTank)*10)/10);

  lcd.setCursor(0,1);
  lcd.print("C1:");
  lcd.print(Tank1);
  lcd.setCursor(9,1);
  if((Target-Tank1) > 0){
    lcd.write(0);
  }else if(Target-Tank1 <0){
    lcd.write(1);
  }
  lcd.setCursor(10,1);
  lcd.print(round(abs(Target-Tank1)*10)/10);

  lcd.setCursor(0,2);
  lcd.print("C2:");
  lcd.print(Tank2);
  lcd.setCursor(9,2);
  if((Target-Tank2) > 0){
    lcd.write(0);
  }else if(Target-Tank2 <0){
    lcd.write(1);
  }
  lcd.setCursor(10,2);
  lcd.print(round(abs(Target-Tank2)*10)/10);

  lcd.setCursor(0,3);
  lcd.print("C3:");
  lcd.print(Tank3);
  lcd.setCursor(9,3);
  if((Target-Tank3) > 0){
    lcd.write(0);
  }else if(Target-Tank3 <0){
    lcd.write(1);
  }
  lcd.setCursor(10,3);
  lcd.print(round(abs(Target-Tank3)*10)/10);

  lcd.setCursor(13,2);
  lcd.print("P:");
  lcd.print(map(heaterPower,0,255,0,100));
  lcd.print("%");
} // Prints everything to LCD

void updateSensors(){
  sensors.requestTemperatures();
  mainTank = sensors.getTempC(mainTankSensor);
  Tank3 = sensors.getTempC(Tank1Sensor);
  Tank2 = sensors.getTempC(Tank2Sensor);
  Tank1 = sensors.getTempC(Tank3Sensor);

  if(mainTank < 0){
    updateSensors();
    return;
  }

  int potentiometerTargetue = analogRead(Potentiometer);
  Target = map(potentiometerTargetue, 0, 1023, 0, maxTemp);
} // Updates All sensor readings

void updateLEDs(){
  if (heaterStatus == ON) {
    digitalWrite(LEDHeater, HIGH);
  } else {
    digitalWrite(LEDHeater, LOW);
  }

  if (mainTank >= Target) {
    digitalWrite(LEDMainTank, HIGH);
  } else {
    digitalWrite(LEDMainTank, LOW);
  }

  if (Tank1 >= Target) {
    digitalWrite(LEDTank1, HIGH);
  } else {
    digitalWrite(LEDTank1, LOW);
  }

  if (Tank2 >= Target) {
    digitalWrite(LEDTank2, HIGH);
  } else {
    digitalWrite(LEDTank2, LOW);
  }

  if (Tank3 >= Target) {
    digitalWrite(LEDTank3, HIGH);
  } else {
    digitalWrite(LEDTank3, LOW);
  }

} // Updates LEDs

void menu(){
  if(errorTank1Chosen == false){
    errorTank1Chosen = true;
  }else if(errorTank2Chosen == false){
    errorTank2Chosen = true;
  }else if(MaxTank3Chosen == false){
    MaxTank3Chosen = true;
  }else if(MinTank3Chosen == false){
    MinTank3Chosen = true;
  }else{
    menuRunning = false;
  }
}

void loop() {
  if(menuRunning == false){
    updateSensors();
    heater();
    printLCD();
    updateLEDs();
    delay(50);
  }else{
    menu();
  }

}
