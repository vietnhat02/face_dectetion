#include "esp_camera.h"
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include <UniversalTelegramBot.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "fb_gfx.h"
#include "fd_forward.h"
#include "fr_forward.h"
#include "fr_flash.h"

#include <WiFi.h>
#include <WiFiClientSecure.h>

#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

#define ENROLL_CONFIRM_TIMES 5
#define FACE_ID_SAVE_NUMBER 7
 String wanrning = "Warning!!!!";
//#define FLASH_LED_PIN 4
#define BTN 2
unsigned long door_opened_millis;
unsigned long pressStartTime = 0;
const unsigned long PressDuration = 1500; // Adjust as needed (milliseconds)
int buttonState = 0;
int checktime=1
;
#define I2C_SDA 14 
#define I2C_SCL 15
//#define sig 0
#define sig1 13



//const char* ssid = "NO.1";
//const char* password = "01N235678";

const char* ssid = "Maiu";
const char* password = "eup11111";

String BOTtoken = "7164409410:AAG2fCyx_0i-c9C9JK9la9uS2k5aNobCyeg";  // your Bot Token (Get from Botfather)

String CHAT_ID = "5869562712";
bool sendPhoto = false;

LiquidCrystal_I2C lcd(0x27,16,2);
WiFiClientSecure clientTCP;
UniversalTelegramBot bot(BOTtoken, clientTCP);

int botRequestDelay = 1000;
unsigned long lastTimeBotRan;

int countfail=0;

#define ENROLL_CONFIRM_TIMES 5
#define FACE_ID_SAVE_NUMBER 7
#define FACE_COLOR_WHITE 0x00FFFFFF
#define FACE_COLOR_BLACK 0x00000000
#define FACE_COLOR_RED 0x000000FF
#define FACE_COLOR_GREEN 0x0000FF00
#define FACE_COLOR_BLUE 0x00FF0000
#define FACE_COLOR_YELLOW (FACE_COLOR_RED | FACE_COLOR_GREEN)
#define FACE_COLOR_CYAN (FACE_COLOR_BLUE | FACE_COLOR_GREEN)
#define FACE_COLOR_PURPLE (FACE_COLOR_BLUE | FACE_COLOR_RED)




typedef struct {
    size_t size; //number of values used for filtering
    size_t index; //current value index
    size_t count; //value count
    int sum;
    int *values; //array to be filled with values
} ra_filter_t;

static ra_filter_t ra_filter;
static mtmn_config_t mtmn_config = { 0 };
static int8_t stream_enabled = 0;
static int8_t detection_enabled = 1;
static int8_t recognition_enabled = 1;
static int8_t is_enrolling = 0;
static face_id_list id_list = { 0 };

static bool first_face_deleted = false;

static ra_filter_t *ra_filter_init(ra_filter_t *filter, size_t sample_size) {
    memset(filter, 0, sizeof(ra_filter_t));
    filter->values = (int *)malloc(sample_size * sizeof(int));
    if (!filter->values) {
        return NULL;
    }
    memset(filter->values, 0, sample_size * sizeof(int));
    filter->size = sample_size;
    return filter;
}

static int ra_filter_run(ra_filter_t *filter, int value) {
    if (!filter->values) {
        return value;
    }
    filter->sum -= filter->values[filter->index];
    filter->values[filter->index] = value;
    filter->sum += filter->values[filter->index];
    filter->index++;
    filter->index = filter->index % filter->size;
    if (filter->count < filter->size) {
        filter->count++;
    }
    return filter->sum / filter->count;
}
static void draw_face_boxes(dl_matrix3du_t *image_matrix, box_array_t *boxes, int face_id) {
  // camera_fb_t *fb = esp_camera_fb_get();
  //       if (fb) {
  //           ESP_LOGE(TAG, "Lỗi khi nhận khung hình");
  //       }
  int x, y, w, h, i;
  uint32_t color = FACE_COLOR_YELLOW;
  if (face_id < 0) {
    color = FACE_COLOR_RED;
  } else if (face_id > 0) {
    color = FACE_COLOR_GREEN;
  }
  fb_data_t fb;
  fb.width = image_matrix->w;
  fb.height = image_matrix->h;
  fb.data = image_matrix->item;
  fb.bytes_per_pixel = 3;
  fb.format = FB_BGR888;
  for (i = 0; i < boxes->len; i++) {
    // rectangle box
    x = (int)boxes->box[i].box_p[0];
    y = (int)boxes->box[i].box_p[1];
    w = (int)boxes->box[i].box_p[2] - x + 1;
    h = (int)boxes->box[i].box_p[3] - y + 1;
    fb_gfx_drawFastHLine(&fb, x, y, w, color);
    fb_gfx_drawFastHLine(&fb, x, y + h - 1, w, color);
    fb_gfx_drawFastVLine(&fb, x, y, h, color);
    fb_gfx_drawFastVLine(&fb, x + w - 1, y, h, color);
#if 0
        // landmark
        int x0, y0, j;
        for (j = 0; j < 10; j+=2) {
            x0 = (int)boxes->landmark[i].landmark_p[j];
            y0 = (int)boxes->landmark[i].landmark_p[j+1];
            fb_gfx_fillRect(&fb, x0, y0, 3, 3, color);
        }
#endif
  }
}
static int run_face_recognition(dl_matrix3du_t *image_matrix, box_array_t *net_boxes) {

  dl_matrix3du_t *aligned_face = NULL;
  int matched_id = 0;

  aligned_face = dl_matrix3du_alloc(1, FACE_WIDTH, FACE_HEIGHT, 3);
  if (!aligned_face) {
    //Serial.println("Could not allocate face recognition buffer");
    return matched_id;
  }
  if (align_face(net_boxes, image_matrix, aligned_face) == ESP_OK) {
    if (is_enrolling == 1) {
      int8_t left_sample_face = enroll_face_id_to_flash(&id_list, aligned_face);

      if (left_sample_face == (ENROLL_CONFIRM_TIMES - 1)) {
        Serial.printf("Enrolling Face ID: %d\n", id_list.tail);
      }
      Serial.printf("Enrolling Face ID: %d sample %d\n", id_list.tail, ENROLL_CONFIRM_TIMES - left_sample_face);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Enrolling ID: " + String(id_list.tail));
      lcd.setCursor(0, 1);
      lcd.print("Sample: " + String(ENROLL_CONFIRM_TIMES - left_sample_face));

      delay(10);

      if (left_sample_face == 0) {
        is_enrolling = 0;
        Serial.printf("Enrolled Face ID: %d\n", id_list.tail);
        is_enrolling = 0;
  
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Enrolled: " + String(id_list.tail));
        delay(1000);
      }
    } else {

      matched_id = recognize_face(&id_list, aligned_face);
      if (matched_id >= 0) {
        Serial.printf("Match Face ID: %u\n", matched_id);
        open_door();
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Unlock");
        lcd.setCursor(0, 1);
        lcd.print("Welcome");
        delay(500);
        
        stream_enabled = 0;
        countfail=0;
        // rgb_printf(image_matrix, FACE_COLOR_GREEN, "Hello Subject %u", matched_id);
        // Khởi tạo chân GPIO cho đèn flash và cấu hình là đầu ra
        //pinMode(4, OUTPUT)
      } else {
        Serial.println("No Match Found");
        countfail++;
        // rgb_print(image_matrix, FACE_COLOR_RED, "Intruder Alert!");
        matched_id = -1;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("No Match");
        lcd.setCursor(0, 1);
        delay(500);
      }
    }
  } else {
    Serial.println("Face Not Aligned");
  }
  dl_matrix3du_free(aligned_face);
  return matched_id;
}
void handleNewMessages(int numNewMessages) {
  Serial.print("Handle New Messages: ");
  Serial.println(numNewMessages);

  for (int i = 0; i < numNewMessages; i++) {
    String chat_id = String(bot.messages[i].chat_id);
    if (chat_id != CHAT_ID){
      bot.sendMessage(chat_id, "Unauthorized user", "");
      continue;
    }
    
    // Print the received message
    String text = bot.messages[i].text;
    Serial.println(text);
    
    String from_name = bot.messages[i].from_name;
    if (text == "/start") {
      String welcome = "Welcome , " + from_name + "\n";
      welcome += "Use the following commands to interact with the ESP32-CAM \n";
      welcome += "/photo : takes a new photo\n";
      bot.sendMessage(CHAT_ID, welcome, "");
    }
    if (text == "/photo") {
      sendPhoto = true;
      Serial.println("New photo request");
    }
  }
}
String sendPhotoTelegram() {
  const char* myDomain = "api.telegram.org";
  String getAll = "";
  String getBody = "";

  //Dispose first picture because of bad quality
  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get();
  esp_camera_fb_return(fb); // dispose the buffered image
  
  // Take a new photo
  fb = NULL;  
  fb = esp_camera_fb_get();  
  if(!fb) {
    Serial.println("Camera capture failed");
    delay(1000);
    ESP.restart();
    return "Camera capture failed";
  }  
  
  Serial.println("Connect to " + String(myDomain));


  if (clientTCP.connect(myDomain, 443)) {    
    String head = "--RandomNerdTutorials\r\nContent-Disposition: form-data; name=\"chat_id\"; \r\n\r\n" + CHAT_ID + "\r\n--RandomNerdTutorials\r\nContent-Disposition: form-data; name=\"photo\"; filename=\"esp32-cam.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
    String tail = "\r\n--RandomNerdTutorials--\r\n";

    size_t imageLen = fb->len;
    size_t extraLen = head.length() + tail.length();
    size_t totalLen = imageLen + extraLen;
  
    clientTCP.println("POST /bot"+BOTtoken+"/sendPhoto HTTP/1.1");
    clientTCP.println("Host: " + String(myDomain));
    clientTCP.println("Content-Length: " + String(totalLen));
    clientTCP.println("Content-Type: multipart/form-data; boundary=RandomNerdTutorials");
    clientTCP.println();
    clientTCP.print(head);
  
    uint8_t *fbBuf = fb->buf;
    size_t fbLen = fb->len;
    for (size_t n=0;n<fbLen;n=n+1024) {
      if (n+1024<fbLen) {
        clientTCP.write(fbBuf, 1024);
        fbBuf += 1024;
      }
      else if (fbLen%1024>0) {
        size_t remainder = fbLen%1024;
        clientTCP.write(fbBuf, remainder);
      }
    }  
    
    clientTCP.print(tail);
    
    esp_camera_fb_return(fb);
    
    int waitTime = 10000;   // timeout 10 seconds
    long startTimer = millis();
    boolean state = false;
    
    while ((startTimer + waitTime) > millis()){
      Serial.print(".");
      delay(100);      
      while (clientTCP.available()) {
        char c = clientTCP.read();
        if (state==true) getBody += String(c);        
        if (c == '\n') {
          if (getAll.length()==0) state=true; 
          getAll = "";
        } 
        else if (c != '\r')
          getAll += String(c);
        startTimer = millis();
      }
      if (getBody.length()>0) break;
    }
    clientTCP.stop();
    Serial.println(getBody);
  }
  else {
    getBody="Connected to api.telegram.org failed.";
    lcd.print("Connected to api.telegram.org failed.");
  }
  return getBody;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);
  lcd.init();
  lcd.backlight();
//  pinMode(sig, OUTPUT); 
  pinMode(sig1, OUTPUT); 

  digitalWrite(sig1, LOW);

//  pinMode(ENROLL, INPUT);  
  pinMode(BTN, INPUT_PULLUP);
  WiFi.mode(WIFI_STA);
  //Serial.println();
  //Serial.print("Connecting to ");
  lcd.print("Connecting to ");
  lcd.setCursor(0, 1);
  lcd.print("wifi");
  WiFi.begin(ssid, password);
  clientTCP.setCACert(TELEGRAM_CERTIFICATE_ROOT); // Add root certificate for api.telegram.org
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
    lcd.clear();
    lcd.print("connect ");
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
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
    if (psramFound()) {
        config.frame_size = FRAMESIZE_UXGA;
        config.jpeg_quality = 10;
        config.fb_count = 2;
    } else {
        config.frame_size = FRAMESIZE_SVGA;
        config.jpeg_quality = 12;
        config.fb_count = 1;
    }

    // Camera init
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x", err);
        return;
    }

    sensor_t *s = esp_camera_sensor_get();
    s->set_framesize(s, FRAMESIZE_QVGA);

    // Initialize face recognition configurations
    ra_filter_init(&ra_filter, 20);
    mtmn_config.type = FAST;
    mtmn_config.min_face = 80;
    mtmn_config.pyramid =0.707;
    mtmn_config.pyramid_times = 4;
    mtmn_config.p_threshold.score = 0.6;
    mtmn_config.p_threshold.nms = 0.7;
    mtmn_config.p_threshold.candidate_number = 20;
    mtmn_config.r_threshold.score = 0.7;
    mtmn_config.r_threshold.nms = 0.7;
    mtmn_config.r_threshold.candidate_number = 10;
    mtmn_config.o_threshold.score = 0.7;
    mtmn_config.o_threshold.nms = 0.7;
    mtmn_config.o_threshold.candidate_number = 1;

    
    face_id_init(&id_list, FACE_ID_SAVE_NUMBER, ENROLL_CONFIRM_TIMES);
    read_face_id_from_flash(&id_list);
}

void open_door() {
    digitalWrite(sig1, HIGH);
    lcd.setCursor(0, 1);
    lcd.print("Door Unlocked");
    door_opened_millis = millis(); // time relay closed and door opened
   checktime=0;
}
void delete_face_id(face_id_list *id_list) {
    id_list->tail--; // Decrease the count of face IDs
    id_list->count--; // Decrease the total count of face IDs
    if (id_list->tail == 1) {
        return; 
    }
}

void delete_face() {
    if (id_list.count <= 1) {
        // There is only one face registered, nothing to delete
        return;
    }
    for (int i = 1; i < id_list.count; i++) {
        // Call the function to delete the face ID from your storage mechanism
        delete_face_id(&id_list);  
    }
    lcd.clear();
    lcd.print("Deleting Face");
    delay(3000);
    lcd.clear();
}


void loop() {
  
    camera_fb_t *fb = NULL;
    fb = esp_camera_fb_get();
  if (sendPhoto||countfail==5) {
    lcd.clear();
    lcd.print("Send");
    lcd.setCursor(0,1);
    lcd.print(" Telegram");
    if(countfail==5)
    {
      bot.sendMessage(CHAT_ID, wanrning, "");
      }
    sendPhotoTelegram(); 
    lcd.clear();
    lcd.print("Send");
    lcd.setCursor(0,1);
    lcd.print(" successfully");
    sendPhoto = false; 
    countfail=0;
  }
  buttonState = digitalRead(BTN);

  if (buttonState == LOW && countfail==0)
  {
    lcd.clear();
    lcd.print("Start Press");
    pressStartTime = millis();
    char i=0;
    lcd.setCursor(0, 1);
    while (digitalRead(BTN) == LOW)
    {
      // Wait for button release
      lcd.print(i);
      delay(100);
    }
    unsigned long pressDuration = millis() - pressStartTime;

   if (pressDuration < PressDuration)
    {
       is_enrolling = 1;
       lcd.clear();
       lcd.setCursor(0, 1);
       lcd.print("Add Face");
       
    }
    if (pressDuration >= PressDuration)
    {
      lcd.clear();
      lcd.setCursor(0, 1);
      lcd.print("Delete");
      delete_face();
    }
  }
  if(checktime==0){
  if( millis()-5000 > door_opened_millis)
   {
    digitalWrite(sig1, LOW);
    door_opened_millis=0;
   checktime=1;
    }}

  if (millis() > lastTimeBotRan + botRequestDelay)  {
    int numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    while (numNewMessages) {
      handleNewMessages(numNewMessages);
      numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    }
    lastTimeBotRan = millis();
  }
    if (!fb) {
              Serial.println("Camera capture failed");
              } else {
              dl_matrix3du_t *image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);
                if (!image_matrix) {
                    Serial.println("dl_matrix3du_alloc failed");
                    } else {
                        if (!fmt2rgb888(fb->buf, fb->len, fb->format, image_matrix->item)) {
                            Serial.println("fmt2rgb888 failed");
                        } else {
                            box_array_t *net_boxes = face_detect(image_matrix, &mtmn_config);
                            if (net_boxes) {
                                if (recognition_enabled) {
                                    int face_id = run_face_recognition(image_matrix, net_boxes);
                                          // Handle face recognition result here
                                                        }
                                dl_lib_free(net_boxes->score);
                                dl_lib_free(net_boxes->box);
                                dl_lib_free(net_boxes->landmark);
                                dl_lib_free(net_boxes);
                               }
                              }
                                 dl_matrix3du_free(image_matrix);
                            }
                    esp_camera_fb_return(fb);
}
delay(300); // Adjust delay as needed
} 

    
