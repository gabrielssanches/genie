/***************************************************
DFPlayer - A Mini MP3 Player For Arduino
 <https://www.dfrobot.com/product-1121.html>
 
 ***************************************************
 This example shows the basic function of library for DFPlayer.
 
 Created 2016-12-07
 By [Angelo qiao](Angelo.qiao@dfrobot.com)
 
 GNU Lesser General Public License.
 See <http://www.gnu.org/licenses/> for details.
 All above must be included in any redistribution
 ****************************************************/

/***********Notice and Trouble shooting***************
 1.Connection and Diagram can be found here
 <https://www.dfrobot.com/wiki/index.php/DFPlayer_Mini_SKU:DFR0299#Connection_Diagram>
 2.This code is tested on Arduino Uno, Leonardo, Mega boards.
 ****************************************************/

#include <FastLED.h>
#include "Arduino.h"
#include "DFRobotDFPlayerMini.h"

#define NUM_LEDS 1
#define PIN_BUTTON 27
#define DATA_PIN 13
CRGB leds[NUM_LEDS];
unsigned long lastBlink = 0;  // the last time the output pin was toggled
unsigned long blinkInterval = 250;    // initial length of the first blink
bool playing = false; // playing = blink the eye
int blinkCounter = 0;

DFRobotDFPlayerMini myDFPlayer;
void printDetail(uint8_t type, int value);

static int nfiles = 0;

static void dfmini_play_random(void) {
  static int lastplay = -1;
  int playfile;
  do {
    playfile  = random(1, nfiles+1);
  } while (playfile == lastplay);
  lastplay = playfile;
    Serial.print(F("Number:"));
    Serial.print(playfile);
    Serial.println(F(" Playing!"));

    myDFPlayer.stop();
    myDFPlayer.play(playfile);
}

void shine(){
  for(int i = 0; i<NUM_LEDS;i++){
        leds[i].g = 150;
        leds[i].b = 150;
  }
  FastLED.show();
}
void setup()
{
  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS); 
  Serial2.begin(9600);


  Serial.begin(115200);

  Serial.println();
  Serial.println(F("DFRobot DFPlayer Mini Demo"));
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));
  
  if (!myDFPlayer.begin(Serial2, true, true)) {  //Use serial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while(true){
      delay(0); // Code to compatible with ESP8266 watch dog.
    }
  }
  Serial.println(F("DFPlayer Mini online."));

    
  myDFPlayer.setTimeOut(500); //Set serial communictaion time out 500ms
  
  //----Set volume----
  myDFPlayer.volume(20);  //Set volume value (0~30).

  
  //----Set different EQ----
  myDFPlayer.EQ(DFPLAYER_EQ_NORMAL);
//  myDFPlayer.EQ(DFPLAYER_EQ_POP);
//  myDFPlayer.EQ(DFPLAYER_EQ_ROCK);
//  myDFPlayer.EQ(DFPLAYER_EQ_JAZZ);
//  myDFPlayer.EQ(DFPLAYER_EQ_CLASSIC);
//  myDFPlayer.EQ(DFPLAYER_EQ_BASS);
  
  //----Set device we use SD as default----
//  myDFPlayer.outputDevice(DFPLAYER_DEVICE_U_DISK);
  myDFPlayer.outputDevice(DFPLAYER_DEVICE_SD);
//  myDFPlayer.outputDevice(DFPLAYER_DEVICE_AUX);
//  myDFPlayer.outputDevice(DFPLAYER_DEVICE_SLEEP);
//  myDFPlayer.outputDevice(DFPLAYER_DEVICE_FLASH);
  
  //----Mp3 control----
//  myDFPlayer.sleep();     //sleep
//  myDFPlayer.reset();     //Reset the module
//  myDFPlayer.enableDAC();  //Enable On-chip DAC
//  myDFPlayer.disableDAC();  //Disable On-chip DAC
//  myDFPlayer.outputSetting(true, 15); //output setting, enable the output and set the gain to 15
    //----Read imformation----
  Serial.println(myDFPlayer.readState()); //read mp3 state
  Serial.println(myDFPlayer.readVolume()); //read current volume
  Serial.println(myDFPlayer.readEQ()); //read EQ setting
  Serial.println(myDFPlayer.readFileCounts()); //read all file counts in SD card
  nfiles = myDFPlayer.readCurrentFileNumber();
  Serial.println(nfiles); //read current play file number

  

  
  Serial.print(F("Number of files:"));
  Serial.print(nfiles);
  //myDFPlayer.play(1);
  

  pinMode(PIN_BUTTON, INPUT_PULLUP);
  shine();
}


int buttonState;            // the current reading from the input pin
int lastButtonState = HIGH;  // the previous reading from the input pin

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

void blink(){
  if ((millis() - lastBlink) < blinkInterval)
    return;
    
  blinkCounter++;
  lastBlink = millis();
  blinkInterval = random(30,300);
  // Turn the LED on, then pause
  for(int i = 0; i<NUM_LEDS;i++){
    if(blinkCounter%2){
      leds[i] = CRGB::Black;
      leds[i].g = random(20, 60);
      leds[i].b = random(45, 60);
      }
    else{
        leds[i] = CRGB::Aqua;
      }
    }
  FastLED.show();
}
void loop()
{
  static unsigned long timer = millis();

  if(playing == true) {
    blink();
  }
  int reading = digitalRead(PIN_BUTTON);
  // If the switch changed, due to noise or pressing:
  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != buttonState) {
      buttonState = reading;
      Serial.print(F("Button changed: "));
      Serial.print(buttonState);
      Serial.println(F(""));
      if ( buttonState == LOW) {
         playing = true;
        dfmini_play_random();
      }
    }
  }

  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastButtonState = reading;

  
  // if (millis() - timer > 20000) {
  //   int playfile = random(1, nfiles);
  //   Serial.print(F("Number:"));
  //   Serial.print(playfile);
  //   Serial.println(F(" Playing!"));
  //   timer = millis();
  //   myDFPlayer.play(playfile);
  //   //myDFPlayer.next();  //Play next mp3 every 3 second.
  //   //myDFPlayer.randomAll();
  // }
  
  if (myDFPlayer.available()) {
    printDetail(myDFPlayer.readType(), myDFPlayer.read()); //Print the detail message from DFPlayer to handle different errors and states.
  }
}

void printDetail(uint8_t type, int value){
  switch (type) {
    case TimeOut:
      Serial.println(F("Time Out!"));
      break;
    case WrongStack:
      Serial.println(F("Stack Wrong!"));
      break;
    case DFPlayerCardInserted:
      Serial.println(F("Card Inserted!"));
      break;
    case DFPlayerCardRemoved:
      Serial.println(F("Card Removed!"));
      break;
    case DFPlayerCardOnline:
      Serial.println(F("Card Online!"));
      break;
    case DFPlayerUSBInserted:
      Serial.println("USB Inserted!");
      break;
    case DFPlayerUSBRemoved:
      Serial.println("USB Removed!");
      break;
    case DFPlayerPlayFinished:
      Serial.print(F("Number:"));
      Serial.print(value);
      Serial.println(F(" Play Finished!"));
      shine();
      myDFPlayer.pause();
      playing = false;
      //dfmini_play_random();
      break;
    case DFPlayerError:
      Serial.print(F("DFPlayerError:"));
      switch (value) {
        case Busy:
          Serial.println(F("Card not found"));
          break;
        case Sleeping:
          Serial.println(F("Sleeping"));
          break;
        case SerialWrongStack:
          Serial.println(F("Get Wrong Stack"));
          break;
        case CheckSumNotMatch:
          Serial.println(F("Check Sum Not Match"));
          break;
        case FileIndexOut:
          Serial.println(F("File Index Out of Bound"));
          break;
        case FileMismatch:
          Serial.println(F("Cannot Find File"));
          break;
        case Advertise:
          Serial.println(F("In Advertise"));
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
  
}
