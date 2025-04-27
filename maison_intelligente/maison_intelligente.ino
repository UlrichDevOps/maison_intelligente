#include <HCSR04.h>
#include <LCD_I2C.h>
#include <U8g2lib.h>
#include <AccelStepper.h>

#define TRIGGER_PIN 11
#define ECHO_PIN 12
#define BUZZER_PIN 22
#define RED_PIN 10
#define GREEN_PIN 9
#define BLUE_PIN 8

#define MOTOR_INTERFACE_TYPE 4
#define IN_1 39
#define IN_2 41
#define IN_3 43
#define IN_4 45

#define CLK_PIN 35
#define DIN_PIN 31
#define CS_PIN 33

HCSR04 hc(TRIGGER_PIN, ECHO_PIN);
LCD_I2C lcd(0x27, 16, 2);
AccelStepper myStepper(MOTOR_INTERFACE_TYPE, IN_1, IN_3, IN_2, IN_4);
U8G2_MAX7219_8X8_F_4W_SW_SPI u8g2(
  U8G2_R3,
  CLK_PIN,
  DIN_PIN,
  CS_PIN,
  U8X8_PIN_NONE,
  U8X8_PIN_NONE);

const String DA = "2372368";
unsigned long currentTime = 0;
unsigned long doorMoveTime = 0;
float distance = 0.0;
int degree = 10;
int minAngle = 10;
int maxAngle = 170;
long minStep = (minAngle * 2038.0) / 360;
long maxStep = (maxAngle * 2038.0) / 360;

bool ledState = HIGH;
bool isRed = false;
bool isMoving = false;

const int RGB_PINS[] = { RED_PIN, GREEN_PIN, BLUE_PIN };
const int RGB_PINS_SIZE = sizeof(RGB_PINS) / sizeof(int);
const int DISPLAY_INTERVAL = 100;
const float DISTANCE_INTERVAL = 50.0;
int distanceAlert = 15;
const int DOOR_MOVE_INTERVAL = 2000;
const int DISPLAY_TIME = 3000;

String inputString = "";
bool stringCompleteFlag = false;
bool endDisplay = false;
unsigned long displayStartTime = 0;


enum Etat {
  FERMEE,
  OUVERTURE,
  OUVERTE,
  FERMETURE
} currentState = FERMEE;

enum ALERTE {
  DESACTIVEE,
  ACTIVEE
} alertStatus = DESACTIVEE;

void setup() {
  Serial.begin(115200);
  inputString.reserve(200);
  lcd.begin();
  lcd.backlight();

  u8g2.begin();
  u8g2.setContrast(5);
  u8g2.setFont(u8g2_font_4x6_tr);
  u8g2.clearBuffer();
  u8g2.sendBuffer();

  pinMode(BUZZER_PIN, OUTPUT);

  for (int i = 0; i < RGB_PINS_SIZE; i++) {
    pinMode(RGB_PINS[i], OUTPUT);
  }

  myStepper.setMaxSpeed(500);
  myStepper.setAcceleration(200);
  myStepper.setSpeed(500);
  //myStepper.moveTo(minStep);
  myStepper.enableOutputs();

  departureDisplay();
}

void loop() {
  currentTime = millis();

  if (stringCompleteFlag) {
    commandManager(inputString);
    inputString = "";
    stringCompleteFlag = false;
  }

  if (endDisplay && millis() - displayStartTime >= DISPLAY_TIME) {
  u8g2.clear();
  endDisplay = false;
}

  distance = distanceTask(currentTime);
  stateManager(distance);
  motorTask();
  alertManager(currentTime);
  screenDisplay(currentTime);
  //serialPrint(currentTime);
}

void commandManager(String command) {
  int valeur = 0;

  if (command == "g_dist\n") {
    Serial.println(distance);

  } else if (command.startsWith("cfg;alm;")) {
    valeur = command.substring(8).toInt();
    distanceAlert = valeur;

    functionalCommand();

  } else if (command.startsWith("cfg;lim_inf;")) {
    valeur = command.substring(12).toInt();

    if (valeur < maxAngle) {
      minAngle = valeur;
      minStep = (minAngle * 2038.0) / 360;

      functionalCommand();

    } else {
      error();
    }

  } else if (command.startsWith("cfg;lim_sup;")) {
    valeur = command.substring(12).toInt();

    if (valeur > minAngle) {
      maxAngle = valeur;
      maxStep = (maxAngle * 2038.0) / 360;

      functionalCommand();

    } else {
      error();
    }

  } else {
    unknownCommand();
  }
}

void error() {
  u8g2.clearBuffer();
  u8g2.drawCircle(3, 3, 3);
  u8g2.drawLine(1, 1, 5, 5);
  u8g2.sendBuffer();

  endDisplay = true;
  displayStartTime = millis();
}

void functionalCommand() {
  u8g2.clearBuffer();
  u8g2.drawLine(1, 5, 3, 7);
  u8g2.drawLine(3, 7, 7, 1);
  u8g2.sendBuffer();

  endDisplay = true;
  displayStartTime = millis();
}



void unknownCommand() {
  u8g2.clearBuffer();
  u8g2.drawLine(7, 7, 0, 0);
  u8g2.drawLine(0, 7, 7, 0);
  u8g2.sendBuffer();

  endDisplay = true;
  displayStartTime = millis();
}


float distanceTask(unsigned long ct) {
  static unsigned long previousDistance = 0;
  if (ct - previousDistance >= DISTANCE_INTERVAL) {
    distance = hc.dist();
    previousDistance = ct;
  }
  return distance;
}

void stateManager(float dist) {
  switch (currentState) {
    case FERMEE:
      if (dist < 30 && dist > 0) {
        currentState = OUVERTURE;
        doorMoveTime = currentTime;
        isMoving = true;
        myStepper.enableOutputs();
        myStepper.moveTo(maxStep);
      }
      break;

    case OUVERTURE:
      if (!isMoving) {
        currentState = OUVERTE;
        myStepper.disableOutputs();
      }
      break;

    case OUVERTE:
      if (dist > 60) {
        currentState = FERMETURE;
        doorMoveTime = currentTime;
        isMoving = true;
        myStepper.enableOutputs();
        myStepper.moveTo(-minStep);
      }
      break;

    case FERMETURE:
      if (!isMoving) {
        currentState = FERMEE;
        myStepper.disableOutputs();
      }
      break;
  }
}

void alertManager(unsigned long ct) {
  static unsigned long previousTime = 0;
  const short interval = 300;

  switch (alertStatus) {
    case DESACTIVEE:
      if (distance <= distanceAlert && distance > 0) {
        digitalWrite(BUZZER_PIN, HIGH);
        blinkRedBlue(ct);
        alertStatus = ACTIVEE;
      }
      break;

    case ACTIVEE:
      if (distance > distanceAlert && (ct - previousTime) >= interval) {
        previousTime = ct;
        digitalWrite(BUZZER_PIN, LOW);
        for (int i = 0; i < RGB_PINS_SIZE; i++) {
          digitalWrite(RGB_PINS[i], LOW);
        }
        alertStatus = DESACTIVEE;
      } else {
        blinkRedBlue(ct);
      }
      break;
  }
}

void blinkRedBlue(unsigned long ct) {
  static unsigned long lastBlinkTime = 0;
  const int blinkInterval = 150;

  if (ct - lastBlinkTime >= blinkInterval) {
    if (isRed) {
      setColor(255, 0, 0);
    } else {
      setColor(0, 0, 255);
    }
    isRed = !isRed;
    lastBlinkTime = ct;
  }
}

void setColor(int red, int green, int blue) {
  analogWrite(RED_PIN, red);
  analogWrite(GREEN_PIN, green);
  analogWrite(BLUE_PIN, blue);
}

void motorTask() {
  if (isMoving) {
    int currentPos = myStepper.currentPosition();
    degree = map(currentPos, minStep, maxStep, minAngle, maxAngle);
    myStepper.run();
    if (!myStepper.isRunning()) {
      isMoving = false;
    }
  }
}

void departureDisplay() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(DA);
  lcd.setCursor(0, 1);
  lcd.print("Labo 4A");
  delay(2000);
  lcd.clear();
}

void screenDisplay(unsigned long ct) {
  static unsigned long previousDisplay = 0;
  if (ct - previousDisplay >= DISPLAY_INTERVAL) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Dist: ");
    lcd.print(distance);
    lcd.print(" cm");

    lcd.setCursor(0, 1);
    switch (currentState) {
      case FERMEE:
        lcd.print("Porte: Fermee");
        break;
      case OUVERTE:
        lcd.print("Porte: Ouverte");
        break;
      case OUVERTURE:
      case FERMETURE:
        lcd.print("Porte: ");
        lcd.print(degree);
        lcd.print(" deg");
        break;
    }
    previousDisplay = ct;
  }
}

/*
void serialPrint(unsigned long ct) {
  static unsigned long previousSerial = 0;

  if (ct - previousSerial >= SERIAL_INTERVAL) {
    Serial.print("etd:");
    Serial.print(DA);
    Serial.print(",dist:");
    Serial.print(distance);
    Serial.print(",deg:");
    Serial.println(degree);

    previousSerial = ct;
  }
}
*/

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringCompleteFlag = true;
    }
  }
}
