#include <HCSR04.h> 
#include <LCD_I2C.h>
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

HCSR04 hc(TRIGGER_PIN, ECHO_PIN);
LCD_I2C lcd(0x27, 16, 2);
AccelStepper myStepper(MOTOR_INTERFACE_TYPE, IN_1, IN_3, IN_2, IN_4);

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
const int  DISTANCE_ALERT = 15;
const int SERIAL_INTERVAL = 100;
const int DOOR_MOVE_INTERVAL = 2000;

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
  lcd.begin();
  lcd.backlight();
  pinMode(BUZZER_PIN, OUTPUT);

  for (int i = 0; i < RGB_PINS_SIZE; i++) {
    pinMode(RGB_PINS[i], OUTPUT);
  }

  myStepper.setMaxSpeed(500);
  myStepper.setAcceleration(200);
  myStepper.setSpeed(500);
  myStepper.moveTo(minStep);
  myStepper.enableOutputs();

  departureDisplay();
}

void loop() {
  currentTime = millis();
  distance = distanceTask(currentTime);
  stateManager(distance);
  alertManager(currentTime);
  motorTask();
  screenDisplay(currentTime);
  serialPrint(currentTime);
}

float distanceTask(unsigned long ct) {
  static unsigned long previousDistance = 0;
  if (ct - previousDistance < DISTANCE_INTERVAL) return distance;
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

void setColor(int red, int green, int blue) {
  analogWrite(RED_PIN, red);
  analogWrite(GREEN_PIN, green);
  analogWrite(BLUE_PIN, blue);
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

void alertManager(unsigned long ct) {
  switch (alertStatus) {

    case DESACTIVEE:
      if (distance <= DISTANCE_ALERT && distance > 0) {
        digitalWrite(BUZZER_PIN, HIGH);
        blinkRedBlue(ct);
        alertStatus = ACTIVEE;
      }
      break;

    case ACTIVEE:
      static unsigned long previousTime = 0;
      const short interval = 300;

      if (distance > DISTANCE_ALERT && (ct - previousTime) >= interval) {
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
