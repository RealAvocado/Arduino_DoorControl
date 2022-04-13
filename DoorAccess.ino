#include <AccelStepper.h>
#include <MultiStepper.h>
#include <rgb_lcd.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <Wire.h>
#include <Adafruit_PN532.h>

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//size of the password
#define pin_size 7
#define password_size 7

//information
String pin[5];
String pswd[5];

//to hold password input
char inputPin[pin_size];
char inputPassword[password_size];

//the true password
char masterPassword[password_size] = "202122";

//number of input password character
int pinCount = 0;
int pswdCount = 0;

bool isLocked = false;

const byte rows = 4;
const byte columns = 3;
char hexaKeys[rows][columns] = {
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'},
  {'*', '0', '#'}
};
//pins of keypad
byte row_pins[rows] = {9, 8, 7, 6};
byte column_pins[columns] = {5, 4, 3};
Keypad keypad = Keypad( makeKeymap(hexaKeys), row_pins, column_pins, rows, columns);

rgb_lcd lcd;

const int colourR = 0;
const int colourG = 255;
const int colourB = 0;


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

// If using the breakout or shield with I2C, define just the pins connected
// to the IRQ and reset lines.  Use the values below (2, 3) for the shield!
#define PN532_IRQ   (2)
#define PN532_RESET (3)  // Not connected by default on the NFC Shield

// Uncomment just _one_ line below depending on how your breakout or shield
// is connected to the Arduino:

// Use this line for a breakout with a SPI connection:
//Adafruit_PN532 nfc(PN532_SCK, PN532_MISO, PN532_MOSI, PN532_SS);

// Use this line for a breakout with a hardware SPI connection.  Note that
// the PN532 SCK, MOSI, and MISO pins need to be connected to the Arduino's
// hardware SPI SCK, MOSI, and MISO pins.  On an Arduino Uno these are
// SCK = 13, MOSI = 11, MISO = 12.  The SS line can be any digital IO pin.
//Adafruit_PN532 nfc(PN532_SS);

// Or use this line for a breakout or shield with an I2C connection:
Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET);

#if defined(ARDUINO_ARCH_SAMD)
#define Serial SerialUSB
#endif

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

// Motor pin definitions:
#define motorPin1  10      // IN1 on the ULN2003 driver
#define motorPin2  11      // IN2 on the ULN2003 driver
#define motorPin3  12     // IN3 on the ULN2003 driver
#define motorPin4  13     // IN4 on the ULN2003 driver
// Define the AccelStepper interface type; 4 wire motor in half step mode:
#define MotorInterfaceType 8
// Initialize with pin sequence IN1-IN3-IN2-IN4 for using the AccelStepper library with 28BYJ-48 stepper motor:
AccelStepper stepper = AccelStepper(MotorInterfaceType, motorPin1, motorPin3, motorPin2, motorPin4);



void setup() {

  //initialize keypad and lcd
  //------------------------------------------------------------------------------
  //------------------------------------------------------------------------------
  // Serial monitor
  Serial.begin(9600);
  //initPinAndPassword(pin,pswd);
  pin[0] = "201600";
  pswd[0] = "123456";
  pin[1] = "201508";
  pswd[1] = "097890";
  pin[2] = "201623";
  pswd[2] = "874345";
  pin[3] = "201232";
  pswd[3] = "223412";
  pin[4] = "201309";
  pswd[4] = "988721";
  lcd.begin(16, 2);
  lcd.setRGB(colourR, colourG, colourB);
  lcd.print("Protected Door");
  loading("Loading");
  lcd.clear();

  //initialize card reader
  //------------------------------------------------------------------------------
  //------------------------------------------------------------------------------
#ifndef ESP8266
  while (!Serial); // for Leonardo/Micro/Zero
#endif
  Serial.begin(115200);
  while (!Serial) delay(10); // for Leonardo/Micro/Zero
  Serial.println("Hello!");

  nfc.begin();
  uint32_t versiondata = nfc.getFirmwareVersion();
  if (! versiondata) {
    Serial.print("Didn't find PN53x board");
    while (1); // halt
  }
  nfc.setPassiveActivationRetries(0xFF);
  // configure board to read RFID tags
  nfc.SAMConfig();

  //initialize motor
  //------------------------------------------------------------------------------
  //------------------------------------------------------------------------------
  // Set the maximum steps per second:
  stepper.setMaxSpeed(1000);
}


//main procedure
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
void loop() {

  // receive the bottom value
  int pin_index = 0;
  lcd.setCursor(0, 0);
  lcd.print("Input your pin:");

  //pin的输入(即个人ID)
  char userKey = keypad.waitForKey();
  if (int(userKey) != 0) {
    inputPin[pinCount] = userKey;
    Serial.println(userKey);
    lcd.setCursor(pinCount, 1);
    lcd.print(userKey);
    pinCount++;
  }
  if (pinCount == pin_size - 1) {
    for (int i = 0; i < 5; i++) {
      const char *temp = pin[i].c_str();
      if (!strcmp(inputPin, temp)) {//如果pin输入正确
        bool rightCard = true;
        while (true) {
          delay(1000);
          lcd.clear();
          lcd.print("Card: press *");
          lcd.setCursor(0, 1);
          lcd.print("Code: press #");
          delay(1000);
          char selectKey = keypad.waitForKey();
          if (int(selectKey) != 0) {
            if (selectKey == '*') {
              //---------------------------------------------------------------------------------------------
              //use card reader to enter 读卡器
              lcd.clear();
              lcd.print("Swipe your card");
              rightCard = verifyCard();
              if (rightCard == true) {
                delay(300);
                lcd.clear();
                lcd.println("Approved.");
                //clockWise();
                counterClockWise();
                delay(1000);
                lcd.setCursor(0, 1);
                lcd.print("Door is open now.");
                delay(2000);
                lcd.clear();
                lcd.print("Waiting...");
                //loading("waiting");
                delay(2000);
                lcd.clear();
                //counterClockWise();
                clockWise();
                lcd.print("Door is closing...");
                delay(2000);
                clearPin();
                break;
              } else {
                delay(1000);
                lcd.setCursor(0, 1);
                lcd.print("You are denied.");
                clearPin();
                break;
              }

            }
            else if (selectKey == '#') {
              //---------------------------------------------------------------------------------------------
              //use password to enter 密码
              pin_index = i;//用来标记序号，方便后续密码与之对应
              delay(1000);
              lcd.clear();
              lcd.print("Enter password:");
              //开始输密码
              bool flag = true;
              while (flag) {//循环输入 直到输入满6位
                char pswdKey = keypad.getKey();
                if (int(pswdKey) != 0) {
                  inputPassword[pswdCount] = pswdKey;
                  Serial.println(pswdKey);
                  lcd.setCursor(pswdCount, 1);
                  lcd.print(pswdKey);
                  pswdCount++;
                }
                if (pswdCount == password_size - 1) {
                  flag = false;
                }
              }
              if (pswdCount == password_size - 1) {
                //校验密码是否正确
                const char *temp2 = pswd[pin_index].c_str();
                if (!strcmp(inputPassword, temp2)) {
                  delay(300);
                  lcd.clear();
                  lcd.println("Approved.");
                  clockWise();
                  delay(1000);
                  lcd.setCursor(0, 1);
                  lcd.print("Door is open now.");
                  delay(2000);
                  lcd.clear();
                  lcd.print("Waiting...");
                  //loading("waiting");
                  delay(1000);
                  lcd.clear();
                  counterClockWise();
                  lcd.print("Door is closing...");
                  delay(2000);
                } else {
                  delay(1000);
                  Serial.println("Password wrong");
                  lcd.clear();
                  lcd.print("Password wrong");
                  delay(1000);
                  lcd.setCursor(0, 1);
                  lcd.print("You are denied.");
                  delay(3000);
                }
                lcd.clear();
                clearPin();
                clearPswd();
                break;
              }
            } else {
              delay(1000);
              lcd.clear();
              lcd.print("Invalid input");
              delay(1000);
              lcd.clear();
            }
          }
        }
        return;
      }
    }
    clearPin();
    delay(1000);
    lcd.clear();
    lcd.print("Wrong pin.");
    delay(1500);
    lcd.clear();
    lcd.print("Enter again.");
    delay(1500);
    return;
  }
}


//keypad and lcd method
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
void loading (char msg[]) {
  lcd.setCursor(0, 1);
  lcd.print(msg);

  for (int i = 0; i < 9; i++) {
    delay(1000);
    lcd.print(".");
  }
}

void clearPin() {
  while (pinCount != 0)
  {
    inputPin[pinCount--] = 0;
  }
  return;
}

void clearPswd()
{
  while (pswdCount != 0)
  {
    inputPassword[pswdCount--] = 0;
  }
  return;
}

//card verification
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

bool verifyCard() {
  boolean success;
  uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
  uint8_t uidLength;        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)

  // Wait for an ISO14443A type cards (Mifare, etc.).  When one is found
  // 'uid' will be populated with the UID, and uidLength will indicate
  // if the uid is 4 bytes (Mifare Classic) or 7 bytes (Mifare Ultralight)
  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, &uid[0], &uidLength);
  return success;
}

//motor operation
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

void clockWise() {
  // Set the current position to 0:
  stepper.setCurrentPosition(0);
  // Run the motor forward at 5000 steps/second until the motor reaches 4096 steps (1 revolution):
  while (stepper.currentPosition() != 2048) {
    stepper.setSpeed(5000);
    stepper.runSpeed();
  }
  delay(1000);
}

void counterClockWise() {
  // Reset the position to 0:
  stepper.setCurrentPosition(0);
  // Run the motor backwards at 5000 steps/second until the motor reaches -4096 steps (1 revolution):
  while (stepper.currentPosition() != -2048) {
    stepper.setSpeed(-5000);
    stepper.runSpeed();
  }
  delay(1000);
}
