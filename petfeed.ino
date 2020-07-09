#include <AccelStepper.h>
#include <GyverEncoder.h>
#include <EEPROM.h>
#include <RTClib.h>
#include <U8g2lib.h>

#define FIRMWARE_VERSION    "2.1.1"

#define FEED_DEFAULT_INTERVAL   250

#define MENU_INTERVAL         10000
#define SUMMARY_INTERVAL       5000
#define GREETING_DELAY         3000

#define STEPPER_ACCELERATION  30000
#define STEPPER_MAX_SPEED      3000
#define STEPPER_MAX_STEPS       150

#define A4988_VMOT                5
#define A4988_EN                  6
#define A4988_STEP               11
#define A4988_DIR                12

#define CLK                       2
#define DT                        3
#define SW                        4

U8X8_SSD1306_128X64_NONAME_HW_I2C lcd(U8X8_PIN_NONE);
AccelStepper stepper(1, A4988_STEP, A4988_DIR);
Encoder enc(CLK, DT, SW, TYPE2);
RTC_DS1307 rtc;

struct IDate {
  byte day;
  byte month;
  int year;
};

struct ITime {
  byte hour;
  byte minute;
  byte second;
};

struct IDateTime {
  IDate date;
  ITime time;
};

struct IActivation {
  byte id;
  bool activated;
  uint32_t interval;
  uint32_t stopAt;
  int encoderIndex;
  int pointerIndex;
};

IActivation *currentEncoderActivation;
IActivation empty = {id: 0, activated: true, interval: 0, stopAt: 0, encoderIndex: 0, pointerIndex: 0};
IActivation summary = {id: 1, activated: false, interval: SUMMARY_INTERVAL, stopAt: 0, encoderIndex: 0, pointerIndex: 0};
IActivation menu = {id: 2, activated: false, interval: MENU_INTERVAL, stopAt: 0, encoderIndex: 0, pointerIndex: 0};
IActivation scheduleAdjustment = {id: 3, activated: false, interval: MENU_INTERVAL, stopAt: 0, encoderIndex: 0, pointerIndex: 0};
IActivation scheduleElementAdjustment = {id: 4, activated: false, interval: MENU_INTERVAL, stopAt: 0, encoderIndex: 0, pointerIndex: 0};
IActivation timeAdjustment = {id: 5, activated: false, interval: MENU_INTERVAL, stopAt: 0, encoderIndex: 0, pointerIndex: 0};
IActivation timeElementAdjustment = {id: 6, activated: false, interval: MENU_INTERVAL, stopAt: 0, encoderIndex: 0, pointerIndex: 0};
IActivation *encoderActivations[] = {&empty, &summary, &menu, &scheduleAdjustment, &scheduleElementAdjustment, &timeAdjustment, &timeElementAdjustment};

IDateTime dateTime = {date: {day: 1, month: 1, year: 2020}, time: {hour: 0, minute: 0, second: 0}};

int activationsLength = sizeof(encoderActivations) / sizeof(int);

int SCHEDULE_INITIAL_TEMPLATE[3][5] = {{0, 0, 0, 1, 0}, {0, 0, 0, 1, 0}, {0, 0, 0, 1, 0}};
byte EEMEM INITIAL_RUN_ADDR;
byte INITIAL_RUN;

int EEMEM SCHEDULE_ADDR[3][5];
int SCHEDULE[3][5];

char summaryHeader[] = "Summary";
char timeSettingsHeader[] = "Time setup";
char scheduleSettingsHeader[] = "Schedule setup";

int activeFeedScheduleEntry = -1;
bool feedInProgress = false;
uint32_t feedStartAt = 0;

bool lcdClear = false;

void setup() {
  Serial.begin(9600);

  attachInterrupt(0, encoderTurnLeftInterrupt, CHANGE);
  attachInterrupt(1, encoderTurnRightInterrupt, CHANGE);

  pinMode(A4988_VMOT, OUTPUT);
  pinMode(A4988_EN, OUTPUT);
  pinMode(A4988_STEP, OUTPUT);
  pinMode(A4988_DIR, OUTPUT);

  disableDriver();

  EEPROM.get((int)&INITIAL_RUN_ADDR, INITIAL_RUN);
  if (INITIAL_RUN != 0) {
    EEPROM.put((int)&SCHEDULE_ADDR, SCHEDULE_INITIAL_TEMPLATE);
    EEPROM.put((int)&INITIAL_RUN_ADDR, 0);
  }

  EEPROM.get((int)&SCHEDULE_ADDR, SCHEDULE);

  if (!lcd.begin()) {
    Serial.println("LCD has not been detected");
    Serial.flush();
    abort();
  }

  if (!rtc.begin()) {
    Serial.println("RTC has not been detected");
    Serial.flush();
    abort();
  }

  if (!rtc.isrunning()) {
    Serial.println("RTC has been initialized");
    rtc.adjust(DateTime(dateTime.date.year, dateTime.date.month, dateTime.date.day, dateTime.time.hour, dateTime.time.minute, dateTime.time.second));
  }

  stepper.setMaxSpeed(STEPPER_MAX_SPEED);
  stepper.setAcceleration(STEPPER_ACCELERATION);
  stepper.setCurrentPosition(0);

  displayGreeting();

  delay(GREETING_DELAY);

  lcd.clear();

  activate(empty);
}

void loop() {
  enc.tick();

  if (lcdClear) {
    lcd.clear();
    lcdClear = false;
  }

  if (empty.activated) {
    displayClock(3, 2, 3, 5);
    if (enc.isClick()) {
      openGate();
      closeGate();
    }
  }

  if (summary.activated) {
    displayHeader(summaryHeader);
    displaySummary(false);
    if (enc.isClick()) {
      openGate();
      closeGate();
    }
  }

  if (menu.activated) {
    showMenu();
  }

  if (scheduleAdjustment.activated) {
    showScheduleSettings(1);
  }

  if (scheduleElementAdjustment.activated) {
    showScheduleSettings(2);
  }

  if (timeAdjustment.activated) {
    showTimeSettings(1);
  }

  if (timeElementAdjustment.activated) {
    showTimeSettings(2);
  }

  currentEncoderActivationMonitor();
  feedMonitor();
}

void encoderTurnLeftInterrupt() {
  enc.tick();
  if (enc.isTurn()) {
    activate(*currentEncoderActivation);
    switch (currentEncoderActivation->id) {
      case 0:
        activate(summary);
        break;
      case 2:
        lcdClear = true;
        currentEncoderActivation->encoderIndex--;
        currentEncoderActivation->encoderIndex = currentEncoderActivation->encoderIndex < 0 ? 1 : currentEncoderActivation->encoderIndex;
        break;
      case 3:
        currentEncoderActivation->pointerIndex--;
        currentEncoderActivation->pointerIndex = currentEncoderActivation->pointerIndex < 0 ? 11 : currentEncoderActivation->pointerIndex;
        break;
      case 4:
        currentEncoderActivation->encoderIndex--;
        constrainScheduleElementAdjustmentEncoderIndex();
        break;
      case 5:
        currentEncoderActivation->pointerIndex--;
        currentEncoderActivation->pointerIndex = currentEncoderActivation->pointerIndex < 0 ? 4 : currentEncoderActivation->pointerIndex;
        break;
      case 6:
        currentEncoderActivation->encoderIndex--;
        constrainTimeElementAdjustmentEncoderIndex();
        break;
    }
  }
}

void encoderTurnRightInterrupt() {
  enc.tick();
  if (enc.isTurn()) {
    activate(*currentEncoderActivation);
    switch (currentEncoderActivation->id) {
      case 0:
        activate(menu);
        break;
      case 2:
        lcdClear = true;
        currentEncoderActivation->encoderIndex++;
        currentEncoderActivation->encoderIndex = currentEncoderActivation->encoderIndex > 1 ? 0 : currentEncoderActivation->encoderIndex;
        break;
      case 3:
        currentEncoderActivation->pointerIndex++;
        currentEncoderActivation->pointerIndex = currentEncoderActivation->pointerIndex > 11 ? 0 : currentEncoderActivation->pointerIndex;
        break;
      case 4:
        currentEncoderActivation->encoderIndex++;
        constrainScheduleElementAdjustmentEncoderIndex();
        break;
      case 5:
        currentEncoderActivation->pointerIndex++;
        currentEncoderActivation->pointerIndex = currentEncoderActivation->pointerIndex > 4 ? 0 : currentEncoderActivation->pointerIndex;
        break;
      case 6:
        currentEncoderActivation->encoderIndex++;
        constrainTimeElementAdjustmentEncoderIndex();
        break;
    }
  }
}

void showMenu() {
  if (enc.isHolded()) {
    activate(empty);
    return;
  }
  switch (currentEncoderActivation->encoderIndex) {
    case 0:
      showScheduleSettings(0);
      if (enc.isClick()) activate(scheduleAdjustment);
      break;
    case 1:
      showTimeSettings(0);
      if (enc.isClick()) activate(timeAdjustment);
      break;
  }
}

void showScheduleSettings(byte level) {
  displayHeader(scheduleSettingsHeader);
  if (level == 0) {
    displaySummary(false);
  } else if (level == 1) {
    displaySummary(true);
    if (enc.isHolded()) {
      activate(menu);
      currentEncoderActivation->encoderIndex = 0;
      return;
    }
    if (enc.isClick()) {
      int pointerIndex = currentEncoderActivation->pointerIndex;
      activate(scheduleElementAdjustment);
      currentEncoderActivation->pointerIndex = pointerIndex;
      switch (currentEncoderActivation->pointerIndex) {
        case 0:
          currentEncoderActivation->encoderIndex = SCHEDULE[0][0];
          break;
        case 1:
          currentEncoderActivation->encoderIndex = SCHEDULE[0][1];
          break;
        case 2:
          currentEncoderActivation->encoderIndex = SCHEDULE[0][2];
          break;
        case 3:
          currentEncoderActivation->encoderIndex = SCHEDULE[0][3];
          break;
        case 4:
          currentEncoderActivation->encoderIndex = SCHEDULE[1][0];
          break;
        case 5:
          currentEncoderActivation->encoderIndex = SCHEDULE[1][1];
          break;
        case 6:
          currentEncoderActivation->encoderIndex = SCHEDULE[1][2];
          break;
        case 7:
          currentEncoderActivation->encoderIndex = SCHEDULE[1][3];
          break;
        case 8:
          currentEncoderActivation->encoderIndex = SCHEDULE[2][0];
          break;
        case 9:
          currentEncoderActivation->encoderIndex = SCHEDULE[2][1];
          break;
        case 10:
          currentEncoderActivation->encoderIndex = SCHEDULE[2][2];
          break;
        case 11:
          currentEncoderActivation->encoderIndex = SCHEDULE[2][3];
          break;
      }
    }
  } else if (level == 2) {
    if (enc.isHolded()) {
      activate(menu);
      currentEncoderActivation->encoderIndex = 0;
      EEPROM.get((int)&SCHEDULE_ADDR, SCHEDULE);
      return;
    }
    switch (currentEncoderActivation->pointerIndex) {
      case 0:
        SCHEDULE[0][0] = currentEncoderActivation->encoderIndex;
        break;
      case 1:
        if (SCHEDULE[0][0] == 0) break;
        SCHEDULE[0][1] = currentEncoderActivation->encoderIndex;
        break;
      case 2:
        if (SCHEDULE[0][0] == 0) break;
        SCHEDULE[0][2] = currentEncoderActivation->encoderIndex;
        break;
      case 3:
        if (SCHEDULE[0][0] == 0) break;
        SCHEDULE[0][3] = currentEncoderActivation->encoderIndex;
        break;
      case 4:
        SCHEDULE[1][0] = currentEncoderActivation->encoderIndex;
        break;
      case 5:
        if (SCHEDULE[1][0] == 0) break;
        SCHEDULE[1][1] = currentEncoderActivation->encoderIndex;
        break;
      case 6:
        if (SCHEDULE[1][0] == 0) break;
        SCHEDULE[1][2] = currentEncoderActivation->encoderIndex;
        break;
      case 7:
        if (SCHEDULE[1][0] == 0) break;
        SCHEDULE[1][3] = currentEncoderActivation->encoderIndex;
        break;
      case 8:
        SCHEDULE[2][0] = currentEncoderActivation->encoderIndex;
        break;
      case 9:
        if (SCHEDULE[2][0] == 0) break;
        SCHEDULE[2][1] = currentEncoderActivation->encoderIndex;
        break;
      case 10:
        if (SCHEDULE[2][0] == 0) break;
        SCHEDULE[2][2] = currentEncoderActivation->encoderIndex;
        break;
      case 11:
        if (SCHEDULE[2][0] == 0) break;
        SCHEDULE[2][3] = currentEncoderActivation->encoderIndex;
        break;
    }
    displaySummary(true);
    if (enc.isClick()) {
      int newSchedule[3][5] = {
        {SCHEDULE[0][0], SCHEDULE[0][1], SCHEDULE[0][2], SCHEDULE[0][3], SCHEDULE[0][4]},
        {SCHEDULE[1][0], SCHEDULE[1][1], SCHEDULE[1][2], SCHEDULE[1][3], SCHEDULE[1][4]},
        {SCHEDULE[2][0], SCHEDULE[2][1], SCHEDULE[2][2], SCHEDULE[2][3], SCHEDULE[2][4]}
      };
      EEPROM.put((int)&SCHEDULE_ADDR, newSchedule);
      EEPROM.get((int)&SCHEDULE_ADDR, SCHEDULE);
      int pointerIndex = currentEncoderActivation->pointerIndex;
      activate(scheduleAdjustment);
      currentEncoderActivation->pointerIndex = pointerIndex;
    }
  }
}

void showTimeSettings(byte level) {
  displayHeader(timeSettingsHeader);
  if (level == 0) {
    DateTime now = rtc.now();
    dateTime.time.hour = now.hour();
    dateTime.time.minute = now.minute();
    dateTime.date.day = now.day();
    dateTime.date.month = now.month();
    dateTime.date.year = now.year();
    displayClock(3, 3, 3, 6);
  } else if (level == 1) {
    displayDateTime();
    if (enc.isHolded()) {
      activate(menu);
      currentEncoderActivation->encoderIndex = 1;
      return;
    }
    if (enc.isClick()) {
      int pointerIndex = currentEncoderActivation->pointerIndex;
      activate(timeElementAdjustment);
      currentEncoderActivation->pointerIndex = pointerIndex;
      switch (currentEncoderActivation->pointerIndex) {
        case 0:
          currentEncoderActivation->encoderIndex = dateTime.time.hour;
          break;
        case 1:
          currentEncoderActivation->encoderIndex = dateTime.time.minute;
          break;
        case 2:
          currentEncoderActivation->encoderIndex = dateTime.date.day;
          break;
        case 3:
          currentEncoderActivation->encoderIndex = dateTime.date.month;
          break;
        case 4:
          currentEncoderActivation->encoderIndex = dateTime.date.year;
          break;
      }
    }
  } else if (level == 2) {
    displayDateTime();
    if (enc.isHolded()) {
      activate(menu);
      currentEncoderActivation->encoderIndex = 1;
      return;
    }
    if (enc.isClick()) {
      rtc.adjust(DateTime(dateTime.date.year, dateTime.date.month, dateTime.date.day, dateTime.time.hour, dateTime.time.minute, dateTime.time.second));
      int pointerIndex = currentEncoderActivation->pointerIndex;
      activate(timeAdjustment);
      currentEncoderActivation->pointerIndex = pointerIndex;
    }
  }
}

void currentEncoderActivationMonitor() {
  byte counter = 0;

  for (byte i = 0; i < activationsLength; i++) {
    IActivation *activationEntry = encoderActivations[i];

    if (!activationEntry->activated) {
      counter++;
      continue;
    }

    if (activationEntry->id != 0 && millis() >= activationEntry->stopAt) {
      activationEntry->activated = false;
      currentEncoderActivation->encoderIndex = 0;
      currentEncoderActivation->pointerIndex = 0;
      activationEntry->stopAt = 0;
    }
  }

  if (counter == activationsLength) {
    EEPROM.get((int)&SCHEDULE_ADDR, SCHEDULE);
    activate(empty);
  }
}

void feedMonitor() {
  DateTime now = rtc.now();
  for (byte i = 0; i < 3; i++ ) {
    if (SCHEDULE[i][0] == 0) continue;
    if (!feedInProgress && feedStartAt == 0) {
      if (now.hour() == SCHEDULE[i][1] && now.minute() == SCHEDULE[i][2] && now.second() == 0) {
        feedInProgress = true;
        feedStartAt = millis();
        SCHEDULE[i][4] = FEED_DEFAULT_INTERVAL * (SCHEDULE[i][3] - 1);
        activeFeedScheduleEntry = i;
        openGate();
      }
    }
  }
  if (feedInProgress && millis() >= feedStartAt + SCHEDULE[activeFeedScheduleEntry][4]) {
    feedInProgress = false;
    closeGate();
  }
  if (millis() >= (feedStartAt + (SCHEDULE[activeFeedScheduleEntry][4] > 1000 ? SCHEDULE[activeFeedScheduleEntry][4] : 1000))) {
    feedStartAt = 0;
    SCHEDULE[activeFeedScheduleEntry][4] = 0;
    activeFeedScheduleEntry = -1;
  }
}

void openGate() {
  enableDriver();
  if (stepper.distanceToGo() == 0) stepper.runToNewPosition(stepper.currentPosition() + STEPPER_MAX_STEPS);
  disableDriver();
}

void closeGate() {
  enableDriver();
  if (stepper.distanceToGo() == 0) stepper.runToNewPosition(stepper.currentPosition() - STEPPER_MAX_STEPS);
  disableDriver();
}

void enableDriver() {
  digitalWrite(A4988_EN, LOW);
  digitalWrite(A4988_VMOT, HIGH);
}

void disableDriver() {
  digitalWrite(A4988_VMOT, LOW);
  digitalWrite(A4988_EN, HIGH);
}

void activate(IActivation &activation) {
  if (currentEncoderActivation->id != activation.id) {
    lcdClear = true;
  }
  for (byte i = 0; i < activationsLength; i++) {
    if (currentEncoderActivation->id == activation.id) continue;
    IActivation *activationEntry = encoderActivations[i];
    activationEntry->activated = false;
    activationEntry->encoderIndex = 0;
    activationEntry->pointerIndex = 0;
    activationEntry->stopAt = 0;
  }
  currentEncoderActivation = &activation;
  currentEncoderActivation->activated = true;
  currentEncoderActivation->stopAt = millis() + currentEncoderActivation->interval;
}

void constrainScheduleElementAdjustmentEncoderIndex() {
  switch (currentEncoderActivation->pointerIndex) {
    case 0:
    case 4:
    case 8:
      if (currentEncoderActivation->encoderIndex < 0) currentEncoderActivation->encoderIndex = 1;
      if (currentEncoderActivation->encoderIndex > 1) currentEncoderActivation->encoderIndex = 0;
      break;
    case 1:
    case 5:
    case 9:
      if (currentEncoderActivation->encoderIndex < 0) currentEncoderActivation->encoderIndex = 23;
      if (currentEncoderActivation->encoderIndex > 23) currentEncoderActivation->encoderIndex = 0;
      break;
    case 2:
    case 6:
    case 10:
      if (currentEncoderActivation->encoderIndex < 0) currentEncoderActivation->encoderIndex = 59;
      if (currentEncoderActivation->encoderIndex > 59) currentEncoderActivation->encoderIndex = 0;
      break;
    case 3:
    case 7:
    case 11:
      if (currentEncoderActivation->encoderIndex < 1) currentEncoderActivation->encoderIndex = 5;
      if (currentEncoderActivation->encoderIndex > 5) currentEncoderActivation->encoderIndex = 1;
      break;
  }
}

void constrainTimeElementAdjustmentEncoderIndex() {
  switch (currentEncoderActivation->pointerIndex) {
    case 0:
      if (currentEncoderActivation->encoderIndex < 0) currentEncoderActivation->encoderIndex = 23;
      if (currentEncoderActivation->encoderIndex > 23) currentEncoderActivation->encoderIndex = 0;
      dateTime.time.hour = currentEncoderActivation->encoderIndex;
      break;
    case 1:
      if (currentEncoderActivation->encoderIndex < 0) currentEncoderActivation->encoderIndex = 59;
      if (currentEncoderActivation->encoderIndex > 59) currentEncoderActivation->encoderIndex = 0;
      dateTime.time.minute = currentEncoderActivation->encoderIndex;
      break;
    case 2:
      if (currentEncoderActivation->encoderIndex < 1) currentEncoderActivation->encoderIndex = 31;
      if (currentEncoderActivation->encoderIndex > 31) currentEncoderActivation->encoderIndex = 1;
      dateTime.date.day = currentEncoderActivation->encoderIndex;
      break;
    case 3:
      if (currentEncoderActivation->encoderIndex < 1) currentEncoderActivation->encoderIndex = 12;
      if (currentEncoderActivation->encoderIndex > 12) currentEncoderActivation->encoderIndex = 1;
      dateTime.date.month = currentEncoderActivation->encoderIndex;
      break;
    case 4:
      if (currentEncoderActivation->encoderIndex < 2020) currentEncoderActivation->encoderIndex = 2099;
      if (currentEncoderActivation->encoderIndex > 2099) currentEncoderActivation->encoderIndex = 2020;
      dateTime.date.year = currentEncoderActivation->encoderIndex;
      break;
  }
}

void displayGreeting() {
  lcd.setFont(u8x8_font_courB18_2x3_r);
  lcd.setCursor(1, 2);
  lcd.print("PetFeed");
  lcd.setFont(u8x8_font_8x13B_1x2_r);
  lcd.setCursor(6, 21);
  lcd.print(FIRMWARE_VERSION);
}

void displayClock(byte tx, byte ty, byte dx, byte dy) {
  DateTime now = rtc.now();
  char timeFormat[] = "hh:mm";
  char dateFormat[] = "DD.MM.YYYY";
  lcd.setFont(u8x8_font_courB18_2x3_r);
  lcd.setCursor(tx, ty);
  lcd.print(now.toString(timeFormat));
  lcd.setFont(u8x8_font_8x13B_1x2_r);
  lcd.setCursor(dx, dy);
  lcd.print(now.toString(dateFormat));
}

void displaySummary(bool setup) {
  lcd.setFont(u8x8_font_8x13B_1x2_r);

  lcd.setCursor(1, 2);
  if (setup && currentEncoderActivation->pointerIndex == 0) lcd.inverse();
  lcd.print("1.");
  lcd.noInverse();
  lcd.print(" ");
  if (SCHEDULE[0][0] == 1) {
    if (setup && currentEncoderActivation->pointerIndex == 1) lcd.inverse();
    displayDateTimeElement(SCHEDULE[0][1]);
    lcd.noInverse();
    lcd.print(":");
    if (setup && currentEncoderActivation->pointerIndex == 2) lcd.inverse();
    displayDateTimeElement(SCHEDULE[0][2]);
    lcd.noInverse();
    lcd.print(" (");
    if (setup && currentEncoderActivation->pointerIndex == 3) lcd.inverse();
    lcd.print(SCHEDULE[0][3]);
    lcd.print("x");
    lcd.noInverse();
    lcd.print(")");
  }
  else {
    if (setup && currentEncoderActivation->pointerIndex == 1) lcd.inverse();
    lcd.print("--");
    lcd.noInverse();
    lcd.print(":");
    if (setup && currentEncoderActivation->pointerIndex == 2) lcd.inverse();
    lcd.print("--");
    lcd.noInverse();
    lcd.print(" (");
    if (setup && currentEncoderActivation->pointerIndex == 3) lcd.inverse();
    lcd.print("--");
    lcd.noInverse();
    lcd.print(")");
  }

  lcd.setCursor(1, 4);
  if (setup && currentEncoderActivation->pointerIndex == 4) lcd.inverse();
  lcd.print("2.");
  lcd.noInverse();
  lcd.print(" ");
  if (SCHEDULE[1][0] == 1) {
    if (setup && currentEncoderActivation->pointerIndex == 5) lcd.inverse();
    displayDateTimeElement(SCHEDULE[1][1]);
    lcd.noInverse();
    lcd.print(":");
    if (setup && currentEncoderActivation->pointerIndex == 6) lcd.inverse();
    displayDateTimeElement(SCHEDULE[1][2]);
    lcd.noInverse();
    lcd.print(" (");
    if (setup && currentEncoderActivation->pointerIndex == 7) lcd.inverse();
    lcd.print(SCHEDULE[1][3]);
    lcd.print("x");
    lcd.noInverse();
    lcd.print(")");
  }
  else {
    if (setup && currentEncoderActivation->pointerIndex == 5) lcd.inverse();
    lcd.print("--");
    lcd.noInverse();
    lcd.print(":");
    if (setup && currentEncoderActivation->pointerIndex == 6) lcd.inverse();
    lcd.print("--");
    lcd.noInverse();
    lcd.print(" (");
    if (setup && currentEncoderActivation->pointerIndex == 7) lcd.inverse();
    lcd.print("--");
    lcd.noInverse();
    lcd.print(")");
  }

  lcd.setCursor(1, 6);
  if (setup && currentEncoderActivation->pointerIndex == 8) lcd.inverse();
  lcd.print("3.");
  lcd.noInverse();
  lcd.print(" ");
  if (SCHEDULE[2][0] == 1) {
    if (setup && currentEncoderActivation->pointerIndex == 9) lcd.inverse();
    displayDateTimeElement(SCHEDULE[2][1]);
    lcd.noInverse();
    lcd.print(":");
    if (setup && currentEncoderActivation->pointerIndex == 10) lcd.inverse();
    displayDateTimeElement(SCHEDULE[2][2]);
    lcd.noInverse();
    lcd.print(" (");
    if (setup && currentEncoderActivation->pointerIndex == 11) lcd.inverse();
    lcd.print(SCHEDULE[2][3]);
    lcd.print("x");
    lcd.noInverse();
    lcd.print(")");
  }
  else {
    if (setup && currentEncoderActivation->pointerIndex == 9) lcd.inverse();
    lcd.print("--");
    lcd.noInverse();
    lcd.print(":");
    if (setup && currentEncoderActivation->pointerIndex == 10) lcd.inverse();
    lcd.print("--");
    lcd.noInverse();
    lcd.print(" (");
    if (setup && currentEncoderActivation->pointerIndex == 11) lcd.inverse();
    lcd.print("--");
    lcd.noInverse();
    lcd.print(")");
  }
}

void displayDateTime() {
  lcd.setFont(u8x8_font_courB18_2x3_r);
  lcd.setCursor(3, 3);
  if (currentEncoderActivation->pointerIndex == 0) lcd.inverse();
  displayDateTimeElement(dateTime.time.hour);
  lcd.noInverse();
  lcd.print(":");
  if (currentEncoderActivation->pointerIndex == 1) lcd.inverse();
  displayDateTimeElement(dateTime.time.minute);
  lcd.noInverse();
  lcd.setFont(u8x8_font_8x13B_1x2_r);
  lcd.setCursor(3, 6);
  if (currentEncoderActivation->pointerIndex == 2) lcd.inverse();
  displayDateTimeElement(dateTime.date.day);
  lcd.noInverse();
  lcd.print(".");
  if (currentEncoderActivation->pointerIndex == 3) lcd.inverse();
  displayDateTimeElement(dateTime.date.month);
  lcd.noInverse();
  lcd.print(".");
  if (currentEncoderActivation->pointerIndex == 4) lcd.inverse();
  lcd.print(dateTime.date.year);
  lcd.noInverse();
}

void displayHeader(char header[]) {
  lcd.setFont(u8x8_font_8x13B_1x2_r);
  lcd.setCursor(0, 0);
  lcd.print(header);
}

void displayDateTimeElement(byte number) {
  if (number < 10) {
    lcd.print("0");
    lcd.print(number);
  } else {
    lcd.print(number);
  }
}
