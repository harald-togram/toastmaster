#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <NeoPixelBus.h>
#include <NeoPixelAnimator.h>
#include <math.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>
#include <PID_v1.h>
#include "BluetoothSerial.h"
/*
  #if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
  #error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
  #endif
*/
//BluetoothSerial SerialBT;

const char* ssid = "MY_WIFI_NAME";
const char* password = "MY_WIFI_NAME";

//neopixelbus setup
const uint16_t PixelCount = 13;

const uint8_t halvPixel = PixelCount / 2;  // this example assumes 4 pixels, making it smaller will cause a failure
const uint8_t PixelPin = 15;               // make sure to set this to the correct pin, ignored for Esp8266
#define colorSaturation 50
const RgbColor CylonEyeColor(HtmlColor(0xff6fff));
NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> strip(PixelCount, PixelPin);
NeoPixelAnimator animations(2);  // only ever need 2 animations
uint16_t lastPixel = 0;          // track the eye position
int8_t moveDir = 1;              // track the direction of movement
// uncomment one of the lines below to see the effects of
// changing the ease function on the movement animation
AnimEaseFunction moveEase =
  //      NeoEase::Linear;
  //NeoEase::QuadraticInOut;
  //      NeoEase::CubicInOut;
  //      NeoEase::QuarticInOut;
  //      NeoEase::QuinticInOut;
  //      NeoEase::SinusoidalInOut;
  //      NeoEase::ExponentialInOut;
  NeoEase::CircularInOut;

//oled setup
#define SCREEN_WIDTH 128     // OLED display width, in pixels
#define SCREEN_HEIGHT 64     // OLED display height, in pixels
#define OLED_RESET -1        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#define LOGO_HEIGHT 16
#define LOGO_WIDTH 16

#define EEPROM_SIZE 6
int threshold = 40;

#define VARME_KNAP 25
#define SKIFT_KNAP 33
#define START_KNAP 32
#define encoder0PinA 27
#define encoder0PinB 26
#define encoder0Btn 14
#define SDA 13
#define CLK 21
#define MOTORPIN 23
#define VARMEPIN 22
#define PIXELPIN 15                                                                                                                                                                                                        
#define TERMOMETERPIN 35  //39
#define PIN_INPUT TERMOMETERPIN
#define PIN_OUTPUT VARMEPIN

#define DIVISION 2

//PID setup
#define antalMaalinger 100
//Define Variables we'll be connecting to
double Setpoint, Input, Output;
double Setpoint2, Input2, Output2;
//Specify the links and initial tuning parameters
double Kp = 30, Ki = 10, Kd = 10;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
// setting PWM properties
const int freq = 255;
const int ledChannel = 0;
const int resolution = 8;
// which analog pin to connect
#define THERMISTORPIN TERMOMETERPIN
// resistance at 25 degrees C
#define THERMISTORNOMINAL 100000
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25
// how many samples to take and average, more takes longer
// but is more 'smooth'
#define NUMSAMPLES 20
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3950

// the value of the 'other' resistor
#define SERIESRESISTOR 4700
#define antalMaalinger 1000
float T_approx;
float V_0 = 3.3;  // voltage reference
// first resistance value for voltage divider
float R_1 = 100000.0;
// fit coefficients
float a = 283786.2;
float b = 0.07593;
float c = 49886.0;
int avg_size = 10;  // averaging size

int encoder0Pos;

int antalLys = 1;
int antalLys2 = 1;
int tidTilbage;
int gammelTidTilbage;
long startTid;

bool varmeTilstand;
bool gammelVarmeTilstand;

long gennemsnit = 0;
int maalinger = 0;

int sted;
bool rediger = false;
bool korer = false;
bool korerGammel = false;
bool skift;
int oldBtn = HIGH;
long nyTid;
long gammelTid;
long nyTidFardigt;
long gammelTidFardigt;
bool ingenSted;

long nyTidSted;
long gammelTidSted;
long tidGaet;
bool risteKorer = false;
bool pusteKorer = false;

int vertPos;
byte values[5];
byte values2[5];
byte valuesTilbage[3];

bool redigeret = false;
bool varmeState = false;

int oldSkiftBtn;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
int valRotary, lastValRotary;
bool klik = false;
bool retning;
float realTemp;
bool programFardigt = false;
long slutTidspunkt;
bool gammelValgBlink;
bool valgBlink;
byte gammelKnapTilstande[4];

String message = "besked";

void IRAM_ATTR handleInterrupt() {
  portENTER_CRITICAL_ISR(&mux);
  if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)) {
    encoder0Pos++;
  } else {
    encoder0Pos--;
  }
  valRotary = encoder0Pos / 2;
  portEXIT_CRITICAL_ISR(&mux);
}

void setup() {
  Serial.begin(115200);
  //SerialBT.begin("cigaretRister"); //Bluetooth device name
  otaSetup();

  //load data fra EEPROM
  EEPROM.begin(11);
  for (int i = 0; i < 4; i++) {
    values[i] = EEPROM.read(i);
  }
  for (int i = 4; i < 8; i++) {
    values2[i - 4] = EEPROM.read(i);
  }
  skift = EEPROM.read(8);

  //setup oled
  Wire.begin(SDA, CLK);
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;  // Don't proceed, loop forever
  }
  display.display();

  //knapper
  pinMode(VARME_KNAP, INPUT_PULLUP);
  pinMode(SKIFT_KNAP, INPUT_PULLUP);
  pinMode(START_KNAP, INPUT_PULLUP);
  pinMode(encoder0Btn, INPUT_PULLUP);
  pinMode(encoder0PinB, INPUT_PULLUP);
  pinMode(encoder0PinA, INPUT_PULLUP);

  //outputs
  pinMode(MOTORPIN, OUTPUT);
  digitalWrite(MOTORPIN, LOW);

  //temperaturPin
  pinMode(TERMOMETERPIN, INPUT);

  //interrupt
  attachInterrupt(digitalPinToInterrupt(encoder0PinA), handleInterrupt, CHANGE);

  Setpoint = 200;

  ledcSetup(ledChannel, freq, resolution);
  ledcAttachPin(VARMEPIN, ledChannel);
  ledcWrite(ledChannel, 0);
  myPID.SetMode(AUTOMATIC);

  display.setRotation(2);
  strip.Begin();
  strip.ClearTo(RgbColor(0, 0, 0));
  strip.Show();
  SetupAnimations();
  opdaterValues();
  opdaterSkaerm();
}

void loop() {
  ArduinoOTA.handle();
  tjekKnap();

  //sluk hvis der er gået 30 sekunder siden programmet sluttede
  if (slutTidspunkt + 40000 < millis() && programFardigt == true && varmeTilstand == true) {
    varmeTilstand = false;
    programFardigt = false;
    ledcWrite(0, 0);
    digitalWrite(MOTORPIN, HIGH);
    delay(5000);
    digitalWrite(MOTORPIN, LOW);
  }

  //fade lys op eller ned når der skiftes varmetilstand
  if (varmeTilstand != gammelVarmeTilstand) {
    programFardigt = false;
    if (varmeTilstand == true) {
      maalTemp();
      for (float i = 0; i < 1; i = i + 0.002) {
        fyldMedLys(i);
      }
    } else if (varmeTilstand == false) {
      for (float i = 1; i > 0; i = i - 0.002) {
        fyldMedLys(i);
      }
      strip.ClearTo(RgbColor(0, 0, 0));
      strip.Show();
    }
    opdaterSkaerm();
    gammelVarmeTilstand = varmeTilstand;
  }

  //start program
  if (korer == true) {
    besked("START", 1, 4);
    opdaterValues();
    risteKorer = true;
    pusteKorer = false;
    if (redigeret == true) {
      gemTal();
    }
    //program loop
    while (korer == true) {
      tjekKnap();
      ArduinoOTA.handle();
      int nyTid = millis();
      if (nyTid >= gammelTid + 1000) {
        //fardig med program
        if (valuesTilbage[2] == 0) {
          Serial.println("fardig med program");
          programFardigt = true;
          korer = false;
          digitalWrite(MOTORPIN, HIGH);
          besked("Done", 2, 5);
          long animationGammelTid = millis();
          long animationNyTid = millis();
          while (animationGammelTid + 10000 > animationNyTid) {
            animations.UpdateAnimations();
            strip.Show();
            animationNyTid = millis();
          }
          strip.ClearTo(RgbColor(0, 0, 0));
          strip.Show();
          digitalWrite(MOTORPIN, LOW);
          opdaterValues();
          opdaterSkaerm();
          slutTidspunkt = millis();
          break;
        }

        //rister
        if (risteKorer == true && valuesTilbage[0] > 0) {
          valuesTilbage[0] = valuesTilbage[0] - 1;
          Serial.print("nu rister vi");
          Serial.println(valuesTilbage[0]);
          digitalWrite(MOTORPIN, LOW);
        }
        //puster
        else if (pusteKorer == true && valuesTilbage[1] > 0) {
          valuesTilbage[1] = valuesTilbage[1] - 1;
          Serial.print("nu pumper vi ");
          Serial.println(valuesTilbage[1]);
          digitalWrite(MOTORPIN, HIGH);
        }

        //skift mellem pust og rist
        if (risteKorer == true && valuesTilbage[0] == 0) {
          risteKorer = false;
          pusteKorer = true;
          //skift mellem rist og pust
        } else if (pusteKorer == true && valuesTilbage[1] == 0) {
          risteKorer = true;
          pusteKorer = false;
          valuesTilbage[2] = valuesTilbage[2] - 1;
          if (skift == 0) {
            valuesTilbage[0] = values[0];
            valuesTilbage[1] = values[1];
          }
          if (skift == 1) {
            valuesTilbage[0] = values2[0];
            valuesTilbage[1] = values2[1];
          }
        }
        gammelTid = nyTid;
        opdaterSkaerm();
      }
      if (varmeTilstand == true) {
        maalTemp();
      } else {
        ledcWrite(ledChannel, 0);
      }
    }
  }
  if (varmeTilstand == true) {
    maalTemp();
  } else {
    ledcWrite(ledChannel, 0);
  }
valgBlink = ((millis() / 500) % 2 == 0);
  if (rediger == false) {
    if (gammelValgBlink != valgBlink) {
      opdaterSkaerm();
    }
  } else {
    valgBlink = true;
    opdaterSkaerm();    
  }
  gammelValgBlink = valgBlink;
}


void tjekKnap() {
  int knapTilstande[4];
  byte knapper[] = { VARME_KNAP, SKIFT_KNAP, START_KNAP, encoder0Btn };
  for (int i = 0; i < 4; i++) {
    knapTilstande[i] = digitalRead(knapper[i]);
    if (knapTilstande[i] == LOW && gammelKnapTilstande[i] == HIGH) {
      if (i == 0) {
        varmeTilstand = !varmeTilstand;
      }

      if (i == 1) {
        skift = !skift;
        redigeret = true;
      }
      if (i == 2) {
        korer = !korer;
      }
      if (i == 3) {
        rediger = !rediger;
      }
      opdaterValues();
      opdaterSkaerm();
    }
    gammelKnapTilstande[i] = knapTilstande[i];
  }

  //plus
  if (lastValRotary < valRotary) {
    korer = false;
    if (rediger == false && vertPos < 3) {
      vertPos++;
    }
    if (rediger == true) {
      if (skift == 0) {
        values[vertPos]++;
        Setpoint = values[3];
      }
      if (skift == 1) {
        values2[vertPos]++;
        Setpoint = values2[3];
      }
      redigeret = true;
    }
    opdaterValues();
    opdaterSkaerm();
  }
  //minus
  else if (lastValRotary > valRotary) {
    korer = false;
    if (rediger == false && vertPos > 0) {
      vertPos--;
    }
    if (rediger == true) {
      if (skift == 0) {
        values[vertPos]--;
      }
      if (skift == 1) {
        values2[vertPos] = values2[vertPos] - 1;
      }
      rediger = true;
    }
    opdaterValues();
    opdaterSkaerm();
  }
  lastValRotary = valRotary;

  if (skift == 0) {
    Setpoint = values[3];
  }
  if (skift == 1) {
    Setpoint = values2[3];
  }
}

void opdaterSkaerm() {
  const String PROGMEM navne[3] = { "ristetid", "pustetid", "antalpust" };
  const PROGMEM byte positioner[] = { 0, 16, 32, 50 };

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  for (int i = 0; i < 3; i++) {
    display.setTextSize(1);
    display.setCursor(13, 2 + 16 * i);
    display.print(navne[i]);
    display.setCursor(70, 2 + 16 * i);
    display.setTextSize(2);
    if (i < 2) {
      ur(valuesTilbage[i]);
    } else {
      display.print(valuesTilbage[i]);
    }
  }

  display.setTextSize(1);
  display.setCursor(13, 48);
  if (skift == 0) {
    display.print(values[3]);
  } else if (skift == 1) {
    display.print(values2[3]);
  }
  if (varmeTilstand == true) {
    display.print(" ");
    display.print(Input);
  }
  display.setTextSize(1 + (vertPos < 3));     // Draw white text
  display.setCursor(0, positioner[vertPos]);  // Start at top-left corner
  if (valgBlink == false) {
    display.println();
  } else {
    display.println(F(">"));
  }
  display.setCursor(0, 56);  // Start at low-left corner
  display.setTextSize(1);    // Draw white text
  if (skift == 0) {
    display.print("   joint      ");
    tidTilbage = ((valuesTilbage[2] - 1) * values[0]) + valuesTilbage[0] + ((valuesTilbage[2] - 1) * values[1]) + valuesTilbage[1];
  } else if (skift == 1) {
    tidTilbage = ((valuesTilbage[2] - 1) * values2[0]) + valuesTilbage[0] + ((valuesTilbage[2] - 1) * values2[1]) + valuesTilbage[1];
    display.print("   pibe       ");
  }/*
  if (skift == 0) {
    tidTilbage = ((valuesTilbage[2] - 1) * values[0]) + valuesTilbage[0] + ((valuesTilbage[2] - 1) * values[1]) + valuesTilbage[1];
  } else if (skift == 1) {
    tidTilbage = ((valuesTilbage[2] - 1) * values2[0]) + valuesTilbage[0] + ((valuesTilbage[2] - 1) * values2[1]) + valuesTilbage[1];
  }*/
  // tidTilbage = risteTidTilFardigt[skift] + ((antalPustTilFardigt[skift] - 1) * risteTidDone[skift]) + pusteTidTilFardigt[skift] + ((antalPustTilFardigt[skift] - 1) * pusteTidDone[skift]);

  ur(tidTilbage);  
  
  display.setCursor(90, 50);  // Start at low-left corner
  display.setTextSize(1);
  display.print(message);    // Draw white text
  display.display();
}

bool urBlink;
void ur(int sekunder) {
  for (int i = 0; i < 60 * 70; i = i + 60) {
    if (sekunder >= i && sekunder < i + 60) {
      display.print(i / 60);
      if (korer == true && urBlink == true) {
        display.print(":");
      }
      if (korer == true && urBlink == false) {
        display.print(" ");
      }
      if (korer == false) {
        display.print(":");
      }
      if (sekunder < i + 10) {
        display.print(0);
      }
      display.println(sekunder - i);
    }
  }
  urBlink = (tidTilbage % 2 == 0);
}

void opdaterValues() {
  if (skift == 0) {
    valuesTilbage[0] = values[0];
    valuesTilbage[1] = values[1];
    valuesTilbage[2] = values[2];
  }
  if (skift == 1) {
    valuesTilbage[0] = values2[0];
    valuesTilbage[1] = values2[1];
    valuesTilbage[2] = values2[2];
  }
}

void besked(String besked, int gentag, int storrelse) {
  display.clearDisplay();
  for (int i = 0; i < gentag; i++) {
    display.setTextSize(storrelse);       // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE);  // Draw white text
    display.setCursor(0, 0);              // Start at top-left corner
    display.print(besked);
    display.display();
    delay(300);
    display.clearDisplay();
    display.setTextSize(storrelse);
    display.setCursor(0, 0);                             // Start at top-left corner          // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);  // Draw 'inverse' text
    display.print(besked);
    display.display();
    delay(300);
    display.clearDisplay();
  }
}

void gemTal() {
  for (int i = 0; i < 4; i++) {
    EEPROM.write(i, values[i]);
  }
  for (int i = 4; i < 8; i++) {
    EEPROM.write(i, values2[i - 4]);
  }
  EEPROM.write(8, skift);
  EEPROM.commit();
  redigeret = false;
}

double ReadVoltage(int reading) {
  if (reading < 1 || reading > 4095) return 0;
  // return -0.000000000009824 * pow(reading,3) + 0.000000016557283 * pow(reading,2) + 0.000854596860691 * reading + 0.065440348345433;
  return -0.000000000000016 * pow(reading, 4) + 0.000000000118171 * pow(reading, 3) - 0.000000301211691 * pow(reading, 2) + 0.001109019271794 * reading + 0.034143524634089;
}

void maalTemp() {
  long gennemsnit = 0;
  for (int i = 0; i < antalMaalinger; i++) {
    int sensorValue = analogRead(TERMOMETERPIN);
    gennemsnit += sensorValue;
  }
  gennemsnit = gennemsnit / antalMaalinger;

  double realVoltage = ReadVoltage(gennemsnit);

  float I = realVoltage / 2000;
  float VRx = 3.3 - realVoltage;
  float Rx = VRx / I;
  Rx = (3.3 - realVoltage) / I;

  float steinhart;
  steinhart = Rx / THERMISTORNOMINAL;                // (R/Ro)
  steinhart = log(steinhart);                        // ln(R/Ro)
  steinhart /= (BCOEFFICIENT);                       // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15);  // + (1/To)
  steinhart = 1.0 / steinhart;                       // Invert
  steinhart -= 273.15;                               // convert absolute temp to C
  Input = steinhart;
  myPID.Compute();
  int intOutput;
  intOutput = int(Output);
  ledcWrite(ledChannel, intOutput);
  opdaterSkaerm();
  fyldMedLys(1);
}

void otaSetup() {
  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    break;
    //ESP.restart();
  }

  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  ArduinoOTA.setHostname("toastmasterNeo");

  // No authentication by default
  //ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else  // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void FadeAll(uint8_t darkenBy) {
  RgbColor color;
  for (uint16_t indexPixel = 0; indexPixel < strip.PixelCount(); indexPixel++) {
    color = strip.GetPixelColor(indexPixel);
    color.Darken(darkenBy);
    strip.SetPixelColor(indexPixel, color);
  }
}

void FadeAnimUpdate(const AnimationParam& param) {
  if (param.state == AnimationState_Completed) {
    FadeAll(10);
    animations.RestartAnimation(param.index);
  }
}

void MoveAnimUpdate(const AnimationParam& param) {
  // apply the movement animation curve
  float progress = moveEase(param.progress);

  // use the curved progress to calculate the pixel to effect
  uint16_t nextPixel;
  if (moveDir > 0) {
    nextPixel = progress * PixelCount;
  } else {
    nextPixel = (1.0f - progress) * PixelCount;
  }

  // if progress moves fast enough, we may move more than
  // one pixel, so we update all between the calculated and
  // the last
  if (lastPixel != nextPixel) {
    for (uint16_t i = lastPixel + moveDir; i != nextPixel; i += moveDir) {
      strip.SetPixelColor(i, CylonEyeColor);
    }
  }
  strip.SetPixelColor(nextPixel, CylonEyeColor);

  lastPixel = nextPixel;

  if (param.state == AnimationState_Completed) {
    // reverse direction of movement
    moveDir *= -1;

    // done, time to restart this position tracking animation/timer
    animations.RestartAnimation(param.index);
  }
}

void SetupAnimations() {
  // fade all pixels providing a tail that is longer the faster
  // the pixel moves.
  animations.StartAnimation(0, 5, FadeAnimUpdate);

  // take several seconds to move eye fron one side to the other
  animations.StartAnimation(1, 1000, MoveAnimUpdate);
}

void fyldMedLys(float lysStyrke) {
  float farveForskel = Input / Setpoint;
  antalLys = farveForskel * PixelCount - 1;
  long Input100x = Input * 100;
  long farve = map(Input100x, Setpoint * 100 - 50 * 100, Setpoint * 100, 0, 255);
  int farveConstrained = constrain(farve, 0, 255);
  float hue = 1.0 / 1024.0 * farveConstrained;
  
  message = String(hue, 4);
  //Serial.println(antal);
  for (int i = 0; i < antalLys; i++) {
    strip.SetPixelColor(i, HsbColor(hue, 1, lysStyrke));
  }
  for (int i = antalLys; i < PixelCount; i++) {
    strip.SetPixelColor(i, HsbColor(hue, 0, 0));
  }
  float topFarve = (farveForskel * PixelCount) - antalLys;
  topFarve = topFarve * lysStyrke;
  strip.SetPixelColor(antalLys, HsbColor(hue, 1, topFarve));
  strip.Show();
}