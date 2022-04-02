// need to install external library : https://github.com/mechasolution/Mecha_QMC5883L
#include <Wire.h>
#include <MechaQMC5883.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
LiquidCrystal_I2C lcd(0x27,20,4);
MechaQMC5883 qmc;

// pin definitions
#define ENC_CLK_PIN  3
#define ENC_DATA_PIN 4
#define ENC_SW_PIN 5
#define BUTTON1_PIN 8
#define BUTTON2_PIN 9
#define MOTOR_LEFT_PIN 10
#define MOTOR_RIGHT_PIN 11
#define SENSOR_PIN 2
#define VOLT_PIN A1

// qmc refresh (ms)
#define HMC_REFRESH_INTERVAL 1000

// qmc offset
int magn_off_x = 560;
int magn_off_y = -1505;

// encoder variables
volatile int enc_data;

// menu variables
# define menuSize 5
char *menu[] ={"Ant Track","Fast Track","Reset Azimuth","Align Magn to North","A1 DC Voltage"};
bool menu_mode = true;            // starting in menu mode
bool AntTrackEnabled = false;    // AutoTrack mode
bool FastTrackEnabled = false;    // FastTrack enable/disable
int FastMultiplier = 10;          // FastTrack multiplier

// antenna variables
volatile unsigned int AzAnt = 0;  // antenna azimuth (real)
int magn_azim;           // magnetic azimuth (real)
unsigned int AzAnt_old = 0;
unsigned int AzMan = 0;           // manually set azimuth (requested)
unsigned int AzMan_old = 0;
boolean RunTracking = false;      // Rotating enabled
boolean RotateRight = false;      // Rotate direction (right=true, left=false)
int correctionAngle = 0;          // correction angle for qmc
unsigned int Volt = 0;
bool eeprom_written = false;

// additional variables
const String code_version = "1.2";   // version
char lcd_chars[4];
char lcd_line[20];
volatile unsigned long sens_last_interrupt_time = 0;
volatile unsigned long enc_last_interrupt_time = 0;
unsigned long hmc_previous_millis;


////  setup ////
void setup() {
  read_eeprom();
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
    delay(80);
  }
  digitalWrite(MOTOR_RIGHT_PIN, HIGH);
  digitalWrite(MOTOR_LEFT_PIN, HIGH);

  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), sensor_blink, LOW);
  attachInterrupt(digitalPinToInterrupt(ENC_CLK_PIN), enc_inter, RISING);

  displayStartInfo();
  lcd.clear();

  enc_data = 0;
  Wire.begin();
  qmc.init();
  qmc.setMode(Mode_Continuous,ODR_200Hz,RNG_8G,OSR_512);
}


//// read data from EEPROM ////
void read_eeprom() {
  AzAnt = EEPROM.read(0) + 256 * EEPROM.read(1);
  if (AzAnt>359) { AzAnt=359; }   // in case some out of range stuff in eeprom
  AzMan = EEPROM.read(2) + 256 * EEPROM.read(3);
  if (AzMan>359) { AzMan=359; }   // in case some out of range stuff in eeprom
  correctionAngle = EEPROM.read(4) + 256 * EEPROM.read(5);
  if (correctionAngle>359)  { correctionAngle=359;  }   // in case some out of range stuff in eeprom
}

//// display info on startup ////
void displayStartInfo() {
  lcd.setCursor(0, 0);
  lcd.print("       ROTOR        ");
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
  // write to eeprom if power goes down
  Volt = analogRead(A1);
  if ((Volt < 800) && (Volt > 0) && (!eeprom_written)) {    // Volt = 0 when powered from USB port
    write_eeprom();
    eeprom_written = true;
  }
    
  // menu mode 
  if (menu_mode) {   // we are in menu mode
    if (enc_data < 0) {enc_data = 0;}
    if (enc_data > (menuSize - 1)) {enc_data = menuSize - 1;}
    displayMenu();

    // choose menu item
    if (digitalRead(ENC_SW_PIN)==LOW) {   // menu item choose when enc switch pressed
      while(digitalRead(ENC_SW_PIN)==LOW);

      switch(enc_data) {
        case 0:                         // AntTrack
          AntTrackEnabled = true;
          menu_mode = false;
          enc_data = AzMan;
          DisplayTrackInfo();
          break;

        case 1:                         // FastTrack
          FastTrackEnabled = !FastTrackEnabled;
          menu_mode = true;
          break;

        case 2:                         // ResetAzimuth
          AzMan=0;
          AzAnt=0;
          menu_mode = true;
          break;

        case 3:                         // AlignMagnToNorth
          int x,y,z;
          qmc.read(&x,&y,&z);
          correctionAngle = int((180/PI) * atan2(x - magn_off_x, y - magn_off_y));
          menu_mode = true;
          break;

        default:
          break;
      }
    }
  } else {          // we are not in menu mode we track Antenna
    if (AntTrackEnabled) {
      RunAutoTrack();
    }
//    if (ManualTrackEnabled) {
//      RunManualTrack();
//    }
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
    case 1:                 // display if menu 1 chosen (FastTrack)
      lcd.setCursor(0,3);
      if (FastTrackEnabled) {
        lcd.print("enabled             ");
      } else {
        lcd.print("disabled            ");
      }
      break;

    case 2:                 // display if menu 2 chosen (Reset Azimuth)
      lcd.setCursor(0,3);
      lcd.print("Ant: ");
      sprintf(lcd_chars, "%03d", AzAnt);
      lcd.setCursor(5,3);
      lcd.print(lcd_chars);

      lcd.setCursor(12,3);
      lcd.print("Set: ");
      sprintf(lcd_chars, "%03d", AzMan);
      lcd.setCursor(17,3);
      lcd.print(lcd_chars);
      break;

    case 3:                 // display if menu 3 chosen (Align Magn to North)
      GetMagneticAzimuth();
      lcd.setCursor(0,3);
      lcd.print("Mag: ");
      sprintf(lcd_chars, "%03d", magn_azim);
      lcd.setCursor(5,3);
      lcd.print(lcd_chars);
      lcd.setCursor(12,3);
      lcd.print("         ");
      break;

    case 4:                 // DisplayA1DCVoltage if menu 4 chosen
      displayDCVoltage();
      break;

    default:
      lcd.setCursor(0,3);
      lcd.print("                    ");
  }
}

//// Auto Tracking ////
void RunAutoTrack() {
  Tracking();

  if (digitalRead(BUTTON1_PIN)==HIGH) {     // exit RunAutoTrack and go back to menu mode if button1 pressed and released
    while(digitalRead(BUTTON1_PIN)==HIGH);
    menu_mode = true;
    AntTrackEnabled = false;
    enc_data = 0;
  }

  if (digitalRead(ENC_SW_PIN)==LOW) {   // in AutoTrack mode start tracking antenna when sw button pressed
    while(digitalRead(ENC_SW_PIN)==LOW);
    if (RunTracking==false) {
      RunTracking=true;
    } else {
      RunTracking=false;                // stop rotation immediately
      if (digitalRead(SENSOR_PIN)==LOW) { delay(80); }
      digitalWrite(MOTOR_RIGHT_PIN, HIGH);
      digitalWrite(MOTOR_LEFT_PIN, HIGH);
      lcd.setCursor(10,3);
      lcd.print("        ");
    }
  }
}

//// Antenna tracking ////
void Tracking() {
  if (RunTracking) { RotateAntenna(); }
  
  if (enc_data < 0) { enc_data = 0; };
  if (enc_data > 35 && FastTrackEnabled) { enc_data = 35; }
  if (enc_data > 359 && !FastTrackEnabled) { enc_data = 359; }

  if (FastTrackEnabled) {
    AzMan = enc_data * FastMultiplier;
  } else {
    AzMan = enc_data;
  }

  if (AzMan != AzMan_old) {
    display_AzMan(); 
    AzMan_old = AzMan;
  }
  if (AzAnt != AzAnt_old) {
    display_AzAnt(); 
    AzAnt_old = AzAnt;
  }
  display_AzMag();
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

//// display only magnetic azimuth ////
void display_AzMag() {
  lcd.setCursor(11,2);
  lcd.print("Magn: ");
  GetMagneticAzimuth();
  sprintf(lcd_chars, "%03d", magn_azim);
  lcd.setCursor(17,2);
  lcd.print(lcd_chars);
}

//// display DC Voltage on A1 pin ////
void displayDCVoltage() {
  Volt = analogRead(A1);
  sprintf(lcd_chars, "%04d", Volt);
  lcd.setCursor(0,3);
  lcd.print("V=");
  lcd.setCursor(2,3);
  lcd.print(lcd_chars);
  lcd.setCursor(6,3);
  lcd.print("              ");
}


//// display lcd info in Tracking mode ////
void DisplayTrackInfo() {
  lcd.setCursor(0,0);
  lcd.print(" Antenna Tracking");
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
}

//// Get QMC5883L azimuth ////
void GetMagneticAzimuth() {
  unsigned long currentMillis = millis();
  if (currentMillis - hmc_previous_millis >= HMC_REFRESH_INTERVAL) {
    hmc_previous_millis = currentMillis;
    int x,y,z;
    qmc.read(&x,&y,&z);

    magn_azim = int((180/PI) * atan2(x - magn_off_x, y - magn_off_y)) - correctionAngle;
    while (magn_azim < 0) {
      magn_azim += 360;
    }
    magn_azim %= 360;
  }
}


//// Antenna rotation ////
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
  if ((RotateRight==false) && (AzAnt <= AzMan)) {
    RunTracking = false;
    lcd.setCursor(10,3);
    lcd.print("        ");
    if (digitalRead(SENSOR_PIN)==LOW) { delay(20); }
    digitalWrite(MOTOR_LEFT_PIN, HIGH);
  }

  // if we rotate right and condition reached then stop rotating 
  if ((RotateRight==true) && (AzAnt >= AzMan)) {
    RunTracking = false;
    lcd.setCursor(10,3);
    lcd.print("        ");
    if (digitalRead(SENSOR_PIN)==LOW) { delay(20); }
    digitalWrite(MOTOR_RIGHT_PIN, HIGH);
  }
}

//// write data to EEPROM ////
void write_eeprom() {
  byte msb, lsb;

  msb = AzAnt/256;
  lsb = AzAnt - (msb * 256);
  EEPROM.write(0, lsb);
  EEPROM.write(1, msb);

  msb = AzMan/256;
  lsb = AzMan - (msb * 256);
  EEPROM.write(2, lsb);
  EEPROM.write(3, msb);

  msb = correctionAngle/256;
  lsb = correctionAngle - (msb * 256);
  EEPROM.write(4, lsb);
  EEPROM.write(5, msb);
}



//// function called by sensor interrupt ////
void sensor_blink() {
  unsigned long now_millis = millis();
  if ((now_millis - sens_last_interrupt_time) > 10) {
    sens_last_interrupt_time = now_millis;
    if (RotateRight) { AzAnt++; } else { AzAnt--; }
  }
}

//// function called by encoder interrupt (CLK rise) ////
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
  
