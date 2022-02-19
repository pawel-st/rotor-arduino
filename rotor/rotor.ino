#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27,20,4);

// pin definitions
#define ENC_CLK_PIN  3
#define ENC_DATA_PIN 4
#define ENC_SW_PIN 5
#define BUTTON1_PIN 8
#define BUTTON2_PIN 9
#define MOTOR_LEFT_PIN 10
#define MOTOR_RIGHT_PIN 11
#define SENSOR_PIN 2

// encoder variables
volatile int enc_data;

// menu variables
# define menuSize 4
char *menu[] ={"Auto Track","Manual Track","Fast Track","Azimuth Calibrate"};
bool menu_mode = true;            // starting in menu mode
bool AutoTrackEnabled = false;    // AutoTrack mode
bool ManualTrackEnabled = false;  // ManualTrack mode
bool FastTrackEnabled = false;    // FastTrack enable/disable
int FastMultiplier = 10;          // FastTrack multiplier

// antenna variables
volatile unsigned int AzAnt = 0;  // antenna azimuth (real)
unsigned int AzAnt_old = 0;
unsigned int AzMan = 0;           // manually set azimuth (requested)
unsigned int AzMan_old = 0;
boolean RunTracking = false;      // Rotating enabled
boolean RotateRight = false;      // Rotate direction (right=true, left=false)

// additional variables
const String code_version = "1.0";   // version
char lcd_chars[4];
char lcd_line[20];
volatile unsigned long sens_last_interrupt_time = 0;
volatile unsigned long enc_last_interrupt_time = 0;


////////////////////////////////////////////////////
void setup() {
  lcd.init(); 
  lcd.begin(20,4);
  lcd.backlight();
  lcd.setBacklight(HIGH);

  pinMode(ENC_CLK_PIN, INPUT);
  pinMode(ENC_DATA_PIN, INPUT);
  pinMode(ENC_SW_PIN, INPUT_PULLUP);
  pinMode(BUTTON1_PIN, INPUT);
  pinMode(BUTTON2_PIN, INPUT);

  pinMode(MOTOR_LEFT_PIN, OUTPUT);
  pinMode(MOTOR_RIGHT_PIN, OUTPUT);

  digitalWrite(MOTOR_RIGHT_PIN, HIGH);
  while (digitalRead(SENSOR_PIN) == LOW)  // rotate motor if sensor stuck in low state
  {
    digitalWrite(MOTOR_LEFT_PIN, LOW);
    delay(40);
  }
  digitalWrite(MOTOR_RIGHT_PIN, HIGH);
  digitalWrite(MOTOR_LEFT_PIN, HIGH);

  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), sensor_blink, LOW);
  attachInterrupt(digitalPinToInterrupt(ENC_CLK_PIN), enc_inter, RISING);

//  displayStartInfo();
  lcd.clear();

  enc_data = 0;
}


void displayStartInfo() {
  lcd.setCursor(0, 0);
  lcd.print("  Sterownik rotora  ");
  delay(500);
  lcd.setCursor(0, 1);
  lcd.print("--------------------");
  delay(500);
  lcd.setCursor(0, 2);
  lcd.print("SQ9NFQ  sq9nfq@wp.pl");
  delay(500);
  lcd.setCursor(0, 3);
  lcd.print("ver: ");
  lcd.setCursor(6,3);
  lcd.print(code_version);
  delay(3500);
}

///// main loop //////
void loop() {
  // menu mode 
  if (menu_mode) {   // we are in menu mode
    if (enc_data < 0) {enc_data = 0;}
    if (enc_data > (menuSize - 1)) {enc_data = menuSize - 1;}
    displayMenu();

    // choose menu item
    if (digitalRead(ENC_SW_PIN)==LOW) {   // menu item choose when enc switch pressed
      while(digitalRead(ENC_SW_PIN)==LOW);

      switch(enc_data) {
        case 0:    // AutoTrack
          AutoTrackEnabled = true;
          menu_mode = false;
          enc_data = AzMan;
          DisplayTrackInfo(0);
          break;

        case 1:   // ManualTrack
          ManualTrackEnabled = true;
          menu_mode = false;
          enc_data = AzMan;
          DisplayTrackInfo(1);
          break;

        case 2:   // FastTrack
          FastTrackEnabled = !FastTrackEnabled;
          menu_mode = true;
          break;
      }
    }
  
  } else {          // we are not in menu mode, either AutoTracking or ManualTracking
    if (AutoTrackEnabled) {
      RunAutoTrack();
    }
    if (ManualTrackEnabled) {
      RunManualTrack();
    }
  }
}

//// display Menu ////
void displayMenu() {
  lcd.setCursor(0,0);
  lcd.print("MENU ");
  sprintf(lcd_chars, "%02d", enc_data);
  lcd.setCursor(5,0);
  lcd.print(lcd_chars);
  lcd.setCursor(7,0);
  lcd.print("           ");
  lcd.setCursor(0,1);
  sprintf(lcd_line, "%-20s", menu[enc_data]);
  lcd.print(lcd_line);
  lcd.setCursor(0,2);
  lcd.print("                    ");

  switch(enc_data) {
    case 2:
      lcd.setCursor(0,3);
      if (FastTrackEnabled) {
        lcd.print("enabled ");
      } else {
        lcd.print("disabled");
      }
      break;

    default:
      lcd.setCursor(0,3);
      lcd.print("                    ");
  }
}

//// Auto Tracking ////
void RunAutoTrack() {
  if (RunTracking) { RotateAntenna(); }
  if (AzMan != AzMan_old) {
    display_AzMan(); 
    AzMan_old = AzMan;
  }
  if (AzAnt != AzAnt_old) {
    display_AzAnt(); 
    AzAnt_old = AzAnt;
  }
  
  if (enc_data < 0) { enc_data = 0; };
  if (enc_data > 35 && FastTrackEnabled) { enc_data = 35; }
  if (enc_data > 359 && !FastTrackEnabled) { enc_data = 359; }

  if (FastTrackEnabled) {
    AzMan = enc_data * FastMultiplier;
  } else {
    AzMan = enc_data;
  }

  if (digitalRead(BUTTON1_PIN)==HIGH) {     // exit RunAutoTrack and go back to menu mode if button1 pressed and released
    while(digitalRead(BUTTON1_PIN)==HIGH);
    menu_mode = true;
    AutoTrackEnabled = false;
    enc_data = 0;
  }

  if (digitalRead(ENC_SW_PIN)==LOW) {   // Start tracking antenna
    while(digitalRead(ENC_SW_PIN)==LOW);
    RunTracking = !RunTracking;
  }
}

//// Manual Tracking   -  not implemented yet !! ////    
void RunManualTrack() {

  if (enc_data < 0) { enc_data = 0; }
  if (enc_data > 359) { enc_data = 359; }
  AzMan = enc_data;

  if (digitalRead(BUTTON1_PIN)==HIGH) {     // exit RunAutoTrack and go back to menu mode if button1 pressed and released
    while(digitalRead(BUTTON1_PIN)==HIGH);
    menu_mode = true;
    ManualTrackEnabled = false;
    enc_data = 0;
  }
}


//// display only AzMan ////
void display_AzMan() {
  sprintf(lcd_chars, "%03d", AzMan);
  lcd.setCursor(5,3);
  lcd.print(lcd_chars);
}

//// display only AzAnt ////
void display_AzAnt() {
  sprintf(lcd_chars, "%03d", AzAnt);
  lcd.setCursor(5,2);
  lcd.print(lcd_chars);
}


//// display lcd info in Tracking mode (auto and manual) ////
void DisplayTrackInfo(byte trackmode) {
  lcd.setCursor(0,0);
  switch(trackmode) {
    case 0:
      lcd.print(" Tracking : Auto    ");
      break;
    case 1:
      lcd.print(" Tracking : Manual  ");
      break;
  }
  lcd.setCursor(0,1);
  lcd.print("--------------------");
  lcd.setCursor(0,2);
  lcd.print("Ant: ");
  sprintf(lcd_chars, "%03d", AzAnt);
  lcd.setCursor(5,2);
  lcd.print(lcd_chars);
  lcd.setCursor(0,3);
  lcd.print("Set: ");
  sprintf(lcd_chars, "%03d", AzMan);
  lcd.setCursor(5,3);
  lcd.print(lcd_chars);
  lcd.setCursor(10,2);
  if (RunTracking) {
    lcd.print("Rotating");
  } else {
    lcd.print("        ");
  }
}

void RotateAntenna() {
  // Rotate left
  if (AzMan < AzAnt) {
    RotateRight = false;
    lcd.setCursor(10,3);
    lcd.print("<<<<<<<<");
    digitalWrite(MOTOR_LEFT_PIN, LOW);
  }

  // Rotate right
  if (AzMan > AzAnt) {
    RotateRight = true;
    lcd.setCursor(10,3);
    lcd.print(">>>>>>>>");
    digitalWrite(MOTOR_RIGHT_PIN, LOW);
  }

  // if we rotate left and condition reached then stop rotating 
  if (!RotateRight && (AzAnt <= AzMan)) {
    RunTracking = false;
    lcd.setCursor(10,3);
    lcd.print("        ");
    if (digitalRead(SENSOR_PIN)==LOW) { delay(20); }
    digitalWrite(MOTOR_LEFT_PIN, HIGH);
  }

  // if we rotate right and condition reached then stop rotating 
  if (RotateRight && (AzAnt >= AzMan)) {
    RunTracking = false;
    lcd.setCursor(10,3);
    lcd.print("        ");
    if (digitalRead(SENSOR_PIN)==LOW) { delay(20); }
    digitalWrite(MOTOR_RIGHT_PIN, HIGH);
  }
}

void sensor_blink() {
  unsigned long now_millis = millis();
  if ((now_millis - sens_last_interrupt_time) > 10) {
    sens_last_interrupt_time = now_millis;
    if (RotateRight) { AzAnt++; } else { AzAnt--; }
  }
}

//// interrupt call when CLK rise
void enc_inter() {
  unsigned long now_millis = millis();
  if ((now_millis - enc_last_interrupt_time) > 4) {
    enc_last_interrupt_time = now_millis;
    delayMicroseconds(500);
    if(digitalRead(ENC_DATA_PIN)) {
      enc_data++;
    } else {
      enc_data--;
    }
  }
}
  