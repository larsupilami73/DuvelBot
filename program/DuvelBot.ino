 /*************************************************************
 * DuvelBot
 * A simple ESP32-CAM Beer-server robot
 * larsupilami73
 * 10/05/2020
 * 
 * Based on many tutorials by Rui Santos
 * Please visit: https://randomnerdtutorials.com  
 *************************************************************/


#include "esp_camera.h"
#include "WiFi.h"
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include "fb_gfx.h"
#include "soc/soc.h" //disable brownout problems
#include "soc/rtc_cntl_reg.h"  //disable brownout problems
#include "ESPAsyncWebServer.h"
#include "SPIFFS.h"

//change credentials of your router here
const char* ssid = "yourNetworkSSIDHere";
const char* password = "yourPasswordHere";


// This project was tested with the AI Thinker Model
#define CAMERA_MODEL_AI_THINKER
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_WITHOUT_PSRAM
//#define CAMERA_MODEL_WROVER_KIT

#if defined(CAMERA_MODEL_WROVER_KIT)
  #define PWDN_GPIO_NUM    -1
  #define RESET_GPIO_NUM   -1
  #define XCLK_GPIO_NUM    21
  #define SIOD_GPIO_NUM    26
  #define SIOC_GPIO_NUM    27
  
  #define Y9_GPIO_NUM      35
  #define Y8_GPIO_NUM      34
  #define Y7_GPIO_NUM      39
  #define Y6_GPIO_NUM      36
  #define Y5_GPIO_NUM      19
  #define Y4_GPIO_NUM      18
  #define Y3_GPIO_NUM       5
  #define Y2_GPIO_NUM       4
  #define VSYNC_GPIO_NUM   25
  #define HREF_GPIO_NUM    23
  #define PCLK_GPIO_NUM    22

#elif defined(CAMERA_MODEL_M5STACK_PSRAM)
  #define PWDN_GPIO_NUM     -1
  #define RESET_GPIO_NUM    15
  #define XCLK_GPIO_NUM     27
  #define SIOD_GPIO_NUM     25
  #define SIOC_GPIO_NUM     23
  
  #define Y9_GPIO_NUM       19
  #define Y8_GPIO_NUM       36
  #define Y7_GPIO_NUM       18
  #define Y6_GPIO_NUM       39
  #define Y5_GPIO_NUM        5
  #define Y4_GPIO_NUM       34
  #define Y3_GPIO_NUM       35
  #define Y2_GPIO_NUM       32
  #define VSYNC_GPIO_NUM    22
  #define HREF_GPIO_NUM     26
  #define PCLK_GPIO_NUM     21

#elif defined(CAMERA_MODEL_M5STACK_WITHOUT_PSRAM)
  #define PWDN_GPIO_NUM     -1
  #define RESET_GPIO_NUM    15
  #define XCLK_GPIO_NUM     27
  #define SIOD_GPIO_NUM     25
  #define SIOC_GPIO_NUM     23
  
  #define Y9_GPIO_NUM       19
  #define Y8_GPIO_NUM       36
  #define Y7_GPIO_NUM       18
  #define Y6_GPIO_NUM       39
  #define Y5_GPIO_NUM        5
  #define Y4_GPIO_NUM       34
  #define Y3_GPIO_NUM       35
  #define Y2_GPIO_NUM       17
  #define VSYNC_GPIO_NUM    22
  #define HREF_GPIO_NUM     26
  #define PCLK_GPIO_NUM     21

#elif defined(CAMERA_MODEL_AI_THINKER)
  #define PWDN_GPIO_NUM     32
  #define RESET_GPIO_NUM    -1
  #define XCLK_GPIO_NUM      0
  #define SIOD_GPIO_NUM     26
  #define SIOC_GPIO_NUM     27
  
  #define Y9_GPIO_NUM       35
  #define Y8_GPIO_NUM       34
  #define Y7_GPIO_NUM       39
  #define Y6_GPIO_NUM       36
  #define Y5_GPIO_NUM       21
  #define Y4_GPIO_NUM       19
  #define Y3_GPIO_NUM       18
  #define Y2_GPIO_NUM        5
  #define VSYNC_GPIO_NUM    25
  #define HREF_GPIO_NUM     23
  #define PCLK_GPIO_NUM     22
#else
  #error "Camera model not selected"
#endif



//GPIOs connected to the motor driver and LEDs
const int motorEnable  = 12; //only 1 pwm channel needed for both left and right motors
const int leds = 4;//remove Q1 or otherwise Flash Led will light
const int left1 = 13;
const int left2 = 15;
const int right1 = 14;
const int right2 = 2; 
const int pwmChannel = 1;
const int pwmFreq = 500;
const int pwmResolution = 8;
int dutyCycleMax = 120;
int dutyCycleMin = 40;
int dutyCycleStep = 10;
int dutyCycleStepDelay = 50; //ms
const int ledpwmChannel = 3;
#define STOP 0
#define LEFT 1
#define RIGHT 2
#define FORWARD 3
#define BACKWARD 4
int actionNow = STOP;
int previousAction = STOP;
int dutyCycleNow = 0;

unsigned long currentMillis=0,previousMillis=0;

//create an AsyncWebServer object on port 80
AsyncWebServer server(80);

void setup(){
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  Serial.begin(115200);
  Serial.setDebugOutput(false);
  Serial.println("DuvelBot initializing...");
  
  //motors setup
  pinMode(motorEnable,OUTPUT);
  pinMode(left1,OUTPUT);
  pinMode(left2,OUTPUT);
  pinMode(right1,OUTPUT);
  pinMode(right2,OUTPUT);
  digitalWrite(left1,LOW);
  digitalWrite(left2,LOW);
  digitalWrite(right1,LOW);
  digitalWrite(right2,LOW);
  ledcSetup(pwmChannel,pwmFreq,pwmResolution);
  ledcAttachPin(motorEnable,pwmChannel);
  ledcWrite(pwmChannel,0);
  
   //leds setup
  pinMode(leds,OUTPUT);
  ledcSetup(ledpwmChannel,pwmFreq,pwmResolution);
  ledcAttachPin(leds,ledpwmChannel);
  ledcWrite(ledpwmChannel,0);

  //configure camera
  Serial.println("Configuring camera.");
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_2;
  config.ledc_timer = LEDC_TIMER_2;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG; 
  
  if(psramFound())
  {
    Serial.println("PSRAM found.");
    config.frame_size = FRAMESIZE_VGA; 
    config.jpeg_quality = 12;
    config.fb_count = 2; //number of framebuffers see: https://github.com/espressif/esp32-camera
  } 
  else 
  {
    Serial.println("no PSRAM found.");
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
  
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
  else{
    Serial.println("Camera configured!");
  }
 
  
  //initialize SPIFFS
  if(!SPIFFS.begin(true))
    { Serial.println("An Error has occurred while mounting SPIFFS!");
      return;
    }

  //connect to Wi-Fi
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
    {
      Serial.print('.');
      delay(500);
    }
 
  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println(" / request received!");
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });

  // Route for duvel logo
  server.on("/duvel", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println(" duvel logo request duvel received!");
    request->send(SPIFFS, "/duvel.png", "image/png");
  });

  // Route to load style.css file
  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println(" css request received");
    request->send(SPIFFS, "/style.css", "text/css");
  });

  // Route to forward
  server.on("/forward", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("received /forward");
    actionNow = FORWARD;
    request->send(200, "text/plain", "OK forward.");
  });
  
  // Route to backward
  server.on("/backward", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("received /backward");
    actionNow = BACKWARD;
    request->send(200, "text/plain", "OK backward.");
  });

  // Route to left
  server.on("/left", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("received /left");
    actionNow = LEFT;
    request->send(200, "text/plain", "OK left.");
  });

  // Route to right
  server.on("/right", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("received /right");
    actionNow = RIGHT;
    request->send(200, "text/plain", "OK right.");
  });

  // Route to stop
  server.on("/stop", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("received /stop");
    actionNow = STOP;
    request->send(200, "text/plain", "OK stop.");
  });

  // Route for LED
  server.on("/LED/*", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("led request received!");
    setLedBrightness((request->url()).substring(5).toInt());
    request->send(200, "text/plain", "OK Leds.");
  });

  // Route for normal speed
  server.on("/normalspeed", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("normalspeed request received!");
    dutyCycleMax = 120;dutyCycleMin = 40;dutyCycleStep = 10;dutyCycleStepDelay = 50; //ms
    request->send(200, "text/plain", "OK normalspeed.");
  });

  // Route for slow speed
  server.on("/slowspeed", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("slowspeed request received!");
    dutyCycleMax = 100;dutyCycleMin = 20;dutyCycleStep = 5;dutyCycleStepDelay = 50; //ms
    request->send(200, "text/plain", "OK slowspeed.");
  });

  // Route for fast speed
  server.on("/fastspeed", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("fastspeed request received!");
    dutyCycleMax = 250;dutyCycleMin = 50;dutyCycleStep = 25;dutyCycleStepDelay = 20; //ms
    request->send(200, "text/plain", "OK fastspeed.");
  });

 // Route for CAMERA
  server.on("/CAMERA", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("camera request received!");
    camera_fb_t * fb = NULL;
    //esp_err_t res = ESP_OK;
    size_t _jpg_buf_len = 0;
    uint8_t * _jpg_buf = NULL;

    //capture a frame
    fb = esp_camera_fb_get();
    if (!fb) {Serial.println("Frame buffer could not be acquired");return;}

    if(fb->format != PIXFORMAT_JPEG)//already in this format from config
    {
      bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
      esp_camera_fb_return(fb);
      fb = NULL;
      if(!jpeg_converted){Serial.println("JPEG compression failed");return;}
    }
    else
    {
      _jpg_buf_len = fb->len;
      _jpg_buf = fb->buf;
    }
     
    //Serial.println(_jpg_buf_len);
    //send the formatted image
    request->send_P(200,"image/jpg", _jpg_buf, _jpg_buf_len);

    //cleanup
    if(fb){
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if(_jpg_buf){
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
  });


  //start the server
  server.begin();

  for(int i=0;i<5;i++)
  {
  setLedBrightness(50);
  delay(50);
  setLedBrightness(0);
  delay(50);
  }

  Serial.print("\nDuvelBot ready! Go to http://");
  Serial.println(WiFi.localIP());
}


// Replaces placeholders in the html like %DATA%
// with the variables you want to show
// <p>Data: <strong>%DATA%</strong></p>
String processor(const String& var)
{
  if(var == "DATA")
  {
    //Serial.println("in processor!");
    return String(dutyCycleNow);
  }
    
  return String();
}


void setLedBrightness(char brightness)
{
  ledcWrite(ledpwmChannel,brightness);
}


void stopNow()
{
  digitalWrite(left1,LOW);
  digitalWrite(left2,LOW);
  digitalWrite(right1,LOW);
  digitalWrite(right2,LOW);
  ledcWrite(pwmChannel,0);
}


void setDir(int direction)
{
  switch(direction)
  {
    case STOP:
      digitalWrite(left1,0);
      digitalWrite(left2,0);
      digitalWrite(right1,0);
      digitalWrite(right2,0);
      break;

    case LEFT:
      digitalWrite(left1,0);
      digitalWrite(left2,1);
      digitalWrite(right1,1);
      digitalWrite(right2,0);
      break;

    case RIGHT:
      digitalWrite(left1,1);
      digitalWrite(left2,0);
      digitalWrite(right1,0);
      digitalWrite(right2,1);
      break;

    case FORWARD: 
      digitalWrite(left1,1);
      digitalWrite(left2,0);
      digitalWrite(right1,1);
      digitalWrite(right2,0);
      break;

    case BACKWARD: 
      digitalWrite(left1,0);
      digitalWrite(left2,1);
      digitalWrite(right1,0);
      digitalWrite(right2,1);
      break;

    default:
      digitalWrite(left1,0);
      digitalWrite(left2,0);
      digitalWrite(right1,0);
      digitalWrite(right2,0);
      break;
  }
}

void loop()
{
  currentMillis = millis();
  if (currentMillis - previousMillis >= dutyCycleStepDelay) 
  {
    // save the last time you executed the loop
    previousMillis = currentMillis;

    //mainloop is responsible for ramping up/down the motors
    if(actionNow != previousAction)
    {
      //ramp down, then stop, then change action and ramp up
      dutyCycleNow = dutyCycleNow-dutyCycleStep;
      if (dutyCycleNow <= 0)
      { //if after ramping down dc is 0, set to the new direction,start at min dutycycle
        setDir(actionNow);
        previousAction = actionNow;
        dutyCycleNow = dutyCycleMin;
      }
     }
    else //actionNow == previousAction --> ramp up,except when direction is STOP
    {
      if (actionNow != STOP)
      {
        dutyCycleNow = dutyCycleNow+dutyCycleStep;
        if (dutyCycleNow > dutyCycleMax) dutyCycleNow = dutyCycleMax;
      }
      else dutyCycleNow = 0;
    }
    ledcWrite(pwmChannel,dutyCycleNow); //adjust the motor dutycycle 
  }
}
