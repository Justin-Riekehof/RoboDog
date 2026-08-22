#include <arduino.h>
#include "WebPage.h"
// Replace with your network credentials
// WIFI_AP settings.
const char* AP_SSID = "WAVESHARE Robot";
const char* AP_PWD  = "1234567890";

// WIFI_STA settings.
const char* STA_SSID = "OnePlus 8";
const char* STA_PWD  = "40963840";

// set the default wifi mode here.
// 1 as [AP] mode, it will not connect other wifi.
// also, 1 as upper computer control mode.
// 2 as [STA] mode, it will connect to know wifi.
#define DEFAULT_WIFI_MODE 1
extern int WIFIP_MODE;

extern IPAddress IP_ADDRESS;
extern byte WIFI_MODE;
extern String MAC_ADDRESS;
extern int WIFI_RSSI;

#include "esp_wifi.h"
#include <WiFi.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#include "dl_lib_matrix3d.h"
#include <esp32-hal-ledc.h>
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "img_converters.h"

#define CAMERA_MODEL_ESP_EYE
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     4
#define SIOD_GPIO_NUM     18
#define SIOC_GPIO_NUM     23

#define Y9_GPIO_NUM       36
#define Y8_GPIO_NUM       37
#define Y7_GPIO_NUM       38
#define Y6_GPIO_NUM       39
#define Y5_GPIO_NUM       35
#define Y4_GPIO_NUM       14
#define Y3_GPIO_NUM       13
#define Y2_GPIO_NUM       34
#define VSYNC_GPIO_NUM    5
#define HREF_GPIO_NUM     27
#define PCLK_GPIO_NUM     25

extern int MiddlePosition;
extern int moveFB;
extern int moveLR;
extern int debugMode;
extern int funcMode;
extern void initPosAll();
extern void middlePosAll();
extern void servoDebug(byte servoID, int offset);
extern void servoConfigSave(byte activeServo);
extern int ServoMiddlePWM[16];
extern int CurrentPWM[16];
// RoboDog: link watchdog, defined in WAVEGO.ino.
extern unsigned long ROBODOG_WATCHDOG_MS;
extern void robodogWatchdogFeed();


extern void getMAC(){
  WiFi.mode(WIFI_MODE_STA);
  MAC_ADDRESS = WiFi.macAddress();
  // Serial.print("MAC:");
  // Serial.println(WiFi.macAddress());
}


extern void getIP(){
  IP_ADDRESS = WiFi.localIP();
}


void setAP(){
  WiFi.softAP(AP_SSID, AP_PWD);
  IPAddress myIP = WiFi.softAPIP();
  IP_ADDRESS = myIP;
  // Serial.print("AP IP address: ");
  // Serial.println(myIP);
  WIFI_MODE = 1;
}


void setSTA(){
  WIFI_MODE = 3;
  WiFi.begin(STA_SSID, STA_PWD);
}


extern void getWifiStatus(){
  if(WiFi.status() == WL_CONNECTED){
    WIFI_MODE = 2;
    getIP();
    WIFI_RSSI = WiFi.RSSI();
  }
  else if(WiFi.status() == WL_CONNECTION_LOST && DEFAULT_WIFI_MODE == 2){
    WIFI_MODE = 3;
    // WiFi.disconnect();
    WiFi.reconnect();
  }
}


void wifiInit(){
  WIFI_MODE = DEFAULT_WIFI_MODE;
  if(WIFI_MODE == 1){setAP();}
  else if(WIFI_MODE == 2){setSTA();}
}


typedef struct {
  httpd_req_t *req;
  size_t len;
} jpg_chunking_t;
 
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";
 
httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;

static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t * _jpg_buf = NULL;
  char * part_buf[64];
  dl_matrix3du_t *image_matrix = NULL;
 
  static int64_t last_frame = 0;
  if (!last_frame) {
    last_frame = esp_timer_get_time();
  }
 
  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) {
    return res;
  }
 
  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      // Serial.println("Camera capture failed");
      res = ESP_FAIL;
    } 
    else {
      {
        if (fb->format != PIXFORMAT_JPEG) {
          bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
          esp_camera_fb_return(fb);
          fb = NULL;
          if (!jpeg_converted) {
            // Serial.println("JPEG compression failed");
            res = ESP_FAIL;
          }
        } else {
          _jpg_buf_len = fb->len;
          _jpg_buf = fb->buf;
        }
      }
    }
    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if (res == ESP_OK) {
      size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }

    if (fb) {
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if (_jpg_buf) {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if (res != ESP_OK) {
      break;
    }
    int64_t fr_end = esp_timer_get_time();
    int64_t frame_time = fr_end - last_frame;
    last_frame = fr_end;
    frame_time /= 1000;

    delay(10);
  }
 
  last_frame = 0;
  return res;
}

 
// === RoboDog: a whole pose in one request ==================================
// `sconfig` moves one servo at a time, relatively, in PWM counts. This carries
// all four foot targets in a single query, so a pose costs one request instead
// of five round trips over a slow soft-AP:
//
//   /control?var=pose&val=0&cmd=0&l1x=16&l1y=95&l1z=25&l2x=-16&...&l4z=25
//
// `val` and `cmd` are still mandatory -- the handler rejects any request
// missing them before it ever looks at the variable (ASSUMPTIONS D3).
//
// The values have to be read while the query buffer is still alive: the
// handler frees it before the dispatch chain runs, so parsing happens up
// there and the dispatch only acts on what was stored here.
//
// Staging and applying are the same functions the serial path uses, and they
// touch no I2C -- which is what makes it safe to call them from the httpd task
// (see the I2C rule in the fork's README).
extern void robodogLegTarget(int leg, double x, double y, double z);
extern void robodogApply();

static double ROBODOG_POSE[4][3];
static int ROBODOG_POSE_READY = 0;

extern void robodogPoseFromQuery(const char *query, const char *variable){
  ROBODOG_POSE_READY = 0;
  if (strcmp(variable, "pose") != 0) { return; }
  const char axes[3] = {'x', 'y', 'z'};
  char key[8];
  char raw[24];
  for (int leg = 1; leg <= 4; leg++){
    for (int a = 0; a < 3; a++){
      snprintf(key, sizeof(key), "l%d%c", leg, axes[a]);
      if (httpd_query_key_value(query, key, raw, sizeof(raw)) != ESP_OK){
        Serial.print("ROBODOG: pose is missing ");Serial.println(key);
        return;
      }
      ROBODOG_POSE[leg-1][a] = atof(raw);
    }
  }
  ROBODOG_POSE_READY = 1;
}

extern int robodogPoseApplyParsed(){
  if (!ROBODOG_POSE_READY) { return 0; }
  for (int leg = 1; leg <= 4; leg++){
    robodogLegTarget(leg, ROBODOG_POSE[leg-1][0], ROBODOG_POSE[leg-1][1],
                     ROBODOG_POSE[leg-1][2]);
  }
  robodogApply();
  return 1;
}
// === end RoboDog ===========================================================


static esp_err_t cmd_handler(httpd_req_t *req){
  char*  buf;
  size_t buf_len;
  char variable[32] = {0,};
  char value[32] = {0,};
  char cmd[32] = {0,};
 
  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1) {
    buf = (char*)malloc(buf_len);
    if (!buf) {
      httpd_resp_send_500(req);
      return ESP_FAIL;
    }
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
      if (httpd_query_key_value(buf, "var", variable, sizeof(variable)) == ESP_OK &&
          httpd_query_key_value(buf, "val", value, sizeof(value)) == ESP_OK &&
          httpd_query_key_value(buf, "cmd", cmd, sizeof(cmd)) == ESP_OK) {
      } else {
        free(buf);
        httpd_resp_send_404(req);
        return ESP_FAIL;
      }
      // === RoboDog: read the pose keys before buf is freed below ============
      robodogPoseFromQuery(buf, variable);
      // === end RoboDog =====================================================
    } else {
      free(buf);
      httpd_resp_send_404(req);
      return ESP_FAIL;
    }
    free(buf);
  } else {
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }
 
  int val = atoi(value);
  int cmdint = atoi(cmd);
  sensor_t * s = esp_camera_sensor_get();
  int res = 0;
 
  // Look at values within URL to determine function
  robodogWatchdogFeed();   // RoboDog: any accepted request counts as "alive"

  if (!strcmp(variable, "framesize")){
    Serial.println("framesize");
    if (s->pixformat == PIXFORMAT_JPEG) res = s->set_framesize(s, (framesize_t)val);
  }

  // functions ctrl.
  else if (!strcmp(variable, "funcMode")){
    debugMode = 0;
    if (val == 1){
      if(funcMode == 1){funcMode = 0;Serial.println("Steady OFF");}
      else if(funcMode == 0){funcMode = 1;Serial.println("Steady ON");}
    }
    else{
      funcMode = val;
      Serial.println(val);
    }        
  }

  // servo config. debugMode
  // val as servoID, cmdint as command.
  else if (!strcmp(variable, "sconfig")){
    debugMode = 1;
    funcMode = 0;
    servoDebug(val, cmdint);
    Serial.print("servo:");Serial.print(val);Serial.print(" position:");Serial.print(CurrentPWM[val]);
    Serial.print(" MID:");Serial.print(ServoMiddlePWM[val]);Serial.print(" offset:");Serial.println(cmdint);
  }
  else if (!strcmp(variable, "sset")){
    if(debugMode){
    servoConfigSave(val);
    Serial.print("SET servo:");Serial.print(val);Serial.print(" position:");Serial.println(ServoMiddlePWM[val]);
    }
    else{
      Serial.print("DebugMode = 0, servo config could not be saved.");
    }
  }

  // RoboDog: arm the link watchdog. val = milliseconds, 0 disables it.
  else if (!strcmp(variable, "watchdog")){
    ROBODOG_WATCHDOG_MS = (val > 0) ? (unsigned long)val : 0;
    Serial.print("watchdog:");Serial.println(val);
  }

  // RoboDog: keep-alive that changes nothing else. The feed above did the work.
  else if (!strcmp(variable, "ping")){
  }

  // === RoboDog: a whole pose, staged and applied in one request ============
  // 500 when any of the twelve values was missing -- a half-parsed pose must
  // not move anything.
  else if (!strcmp(variable, "pose")){
    if (!robodogPoseApplyParsed()){
      Serial.println("ROBODOG: pose rejected, need l1x..l4z");
      res = -1;
    }
  }
  // === end RoboDog ==========================================================

  // move ctrl.
  else if (!strcmp(variable, "move")){
    debugMode = 0;
    funcMode  = 0;
    if (val == 1) {
      Serial.println("Forward");
      moveFB = 1;
    }
    else if (val == 2) {
      Serial.println("TurnLeft");
      moveLR = -1;
    }
    else if (val == 3) {
      Serial.println("FBStop");
      moveFB = 0;
    }
    else if (val == 4) {
      Serial.println("TurnRight");
      moveLR = 1;
    }
    else if (val == 5) {
      Serial.println("Backward");
      moveFB = -1;
    }
    else if (val == 6){
      Serial.println("LRStop");
      moveLR = 0;
    }
  }

  else
  {
    Serial.println("variable");
    res = -1;
  }
 
  if (res) {
    return httpd_resp_send_500(req);
  }
 
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}

 
static esp_err_t index_handler(httpd_req_t *req){
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, (const char *)INDEX_HTML, strlen(INDEX_HTML));
}


void startCameraServer(){
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
 
    httpd_uri_t index_uri = {
        .uri       = "/",
        .method    = HTTP_GET,
        .handler   = index_handler,
        .user_ctx  = NULL
    };
 
    httpd_uri_t cmd_uri = {
        .uri       = "/control",
        .method    = HTTP_GET,
        .handler   = cmd_handler,
        .user_ctx  = NULL
    };
 
   httpd_uri_t stream_uri = {
        .uri       = "/stream",
        .method    = HTTP_GET,
        .handler   = stream_handler,
        .user_ctx  = NULL
    };
    
    // Serial.printf("Starting web server on port: '%d'\n", config.server_port);
    if (httpd_start(&camera_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(camera_httpd, &index_uri);
        httpd_register_uri_handler(camera_httpd, &cmd_uri);
    }
 
    config.server_port += 1;
    config.ctrl_port += 1;
    // Serial.printf("Starting stream server on port: '%d'\n", config.server_port);
    if (httpd_start(&stream_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(stream_httpd, &stream_uri);
    }
}




// === RoboDog: grab one frame to the serial console =========================
// Diagnosis for ASSUMPTIONS F2. The stream endpoint hangs and the vendor page
// shows black, but `esp_camera_init` reports ok -- so the open question is
// whether a frame ever comes out of the sensor. This answers it over USB,
// with no Wi-Fi involved at all.
//
// val = 0: report the frame's metadata only.
// val = 1: also dump the JPEG as base64 between markers, to be decoded on the
//          host and looked at.
//
// Note it can block: if the sensor never completes a frame, esp_camera_fb_get
// waits, and the serial task waits with it. Nothing is moving at that point,
// so the cost is a robot that has to be reset.
static const char ROBODOG_B64_CHARS[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

extern void robodogSnap(int withImage){
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("ROBODOG: snap NULL -- esp_camera_fb_get returned nothing");
    return;
  }
  Serial.print("ROBODOG: snap len=");   Serial.print(fb->len);
  Serial.print(" w=");                  Serial.print(fb->width);
  Serial.print(" h=");                  Serial.print(fb->height);
  Serial.print(" fmt=");                Serial.println((int)fb->format);

  if (withImage && fb->len > 0) {
    Serial.println("ROBODOG: b64 begin");
    size_t i = 0;
    while (i + 2 < fb->len) {
      uint32_t n = ((uint32_t)fb->buf[i] << 16) | ((uint32_t)fb->buf[i+1] << 8) | fb->buf[i+2];
      Serial.print(ROBODOG_B64_CHARS[(n >> 18) & 63]);
      Serial.print(ROBODOG_B64_CHARS[(n >> 12) & 63]);
      Serial.print(ROBODOG_B64_CHARS[(n >>  6) & 63]);
      Serial.print(ROBODOG_B64_CHARS[ n        & 63]);
      i += 3;
    }
    size_t rest = fb->len - i;
    if (rest == 1) {
      uint32_t n = (uint32_t)fb->buf[i] << 16;
      Serial.print(ROBODOG_B64_CHARS[(n >> 18) & 63]);
      Serial.print(ROBODOG_B64_CHARS[(n >> 12) & 63]);
      Serial.print("==");
    } else if (rest == 2) {
      uint32_t n = ((uint32_t)fb->buf[i] << 16) | ((uint32_t)fb->buf[i+1] << 8);
      Serial.print(ROBODOG_B64_CHARS[(n >> 18) & 63]);
      Serial.print(ROBODOG_B64_CHARS[(n >> 12) & 63]);
      Serial.print(ROBODOG_B64_CHARS[(n >>  6) & 63]);
      Serial.print("=");
    }
    Serial.println();
    Serial.println("ROBODOG: b64 end");
  }
  esp_camera_fb_return(fb);
}
// === end RoboDog ===========================================================


void webServerInit(){
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // prevent brownouts by silencing them

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

  //init with high specs to pre-allocate larger buffers
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 63;
  config.fb_count = 1;

  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    // Serial.printf("Camera init failed with error 0x%x", err);
    // return;
  }
  else{
    sensor_t * s = esp_camera_sensor_get();
    s->set_saturation(s, 2);
    delay(1000); 
  }

  // === RoboDog: say whether the camera came up ===============================
  // Upstream commented out both the message and the `return`, so a failed
  // camera init is completely silent: the robot serves a /stream endpoint that
  // never produces a frame, and nothing anywhere says why (ASSUMPTIONS F2).
  // 0x105 = ESP_ERR_NOT_FOUND (no camera answered on the SCCB bus)
  // 0x101 = ESP_ERR_NO_MEM    (the frame buffer did not fit)
  if (err != ESP_OK) {
    Serial.print("ROBODOG: camera init FAILED, error 0x");
    Serial.println(err, HEX);
  }
  else {
    Serial.println("ROBODOG: camera init ok");
  }
  // === end RoboDog ===========================================================

  wifiInit();
  delay(1000); 
  startCameraServer();
}
