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
// RoboDog: camera image quality, defined further down in this file.
extern int robodogCameraSet(const char *name, int val);
extern void robodogCameraReport();
extern int robodogCameraJson(char *out, size_t n);
// RoboDog: how long the next pose should take, defined in WAVEGO.ino.
extern void robodogApplyMs(int val);
// RoboDog: the IMU ring, sampled in loop() and formatted here (InitConfig.h).
extern int robodogImuJson(char *out, size_t n, uint32_t since);


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

  // === RoboDog: the IMU, buffered =========================================
  // `val` is the last sequence number the host already has; the reply carries
  // everything newer. That is what makes 50 Hz of gyroscope survive a link
  // where a request costs 78-140 ms: the samples accumulate on the device and
  // travel in batches, instead of one per round trip.
  //
  // Reads only the ring, never the chip -- loop() is the single I2C writer
  // (see the fork's README), and breaking that rule crashes the robot.
  else if (!strcmp(variable, "imu")){
    // `static`, and that is a bug fix, not a style choice. This handler runs
    // on the httpd task, whose stack is HTTPD_DEFAULT_CONFIG()'s 4096 bytes --
    // a 2 KB buffer here, plus this frame, plus snprintf's float formatting,
    // overflows it. Measured on the robot 2026-08-25: the first HTTP `imu`
    // request ever made killed the IP stack mid-connect, twice, identically --
    // ARP went silent while the 802.11 association stayed up for another five
    // minutes, which is what a corrupted neighbour task looks like, not a
    // panic (a panic reboots, and a reboot drops the association). The serial
    // path had worked for a whole evening because it runs on a different task;
    // its buffer is now static too, since 4000 bytes of stack is the same
    // cliff. Not reentrant, and does not need to be: httpd serialises its
    // handlers on one task.
    static char json[2048];
    int len = robodogImuJson(json, sizeof(json), (uint32_t)(val < 0 ? 0 : val));
    if (len < 0) { res = -1; }
    else {
      httpd_resp_set_type(req, "application/json");
      httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
      return httpd_resp_send(req, json, len);
    }
  }
  // === end RoboDog =========================================================

  // === RoboDog: camera parameters, cam_<name> ==============================
  // The same names the serial transport takes -- robodogCameraSet has the
  // list. 404 rather than a silent success for a name that does not exist:
  // a typo in a tuning session should say so, not look like a sensor that
  // ignores the setting.
  else if (!strncmp(variable, "cam_", 4)){
    if (!robodogCameraSet(variable + 4, val)){
      httpd_resp_send_404(req);
      return ESP_FAIL;
    }
    // Answer with what the sensor holds now, so a write confirms itself in the
    // same round trip. The alternative -- set, then ask -- doubles the traffic
    // on a link that also carries the control loop, and still races anyone
    // else touching the camera in between.
    {
      char json[512];
      int len = robodogCameraJson(json, sizeof(json));
      if (len < 0) { len = 0; }
      if ((size_t)len >= sizeof(json)) { len = sizeof(json) - 1; }
      httpd_resp_set_type(req, "application/json");
      httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
      return httpd_resp_send(req, json, len);
    }
  }
  // === end RoboDog =========================================================

  // === RoboDog: a whole pose, staged and applied in one request ============
  // 500 when any of the twelve values was missing -- a half-parsed pose must
  // not move anything.
  else if (!strcmp(variable, "pose")){
    robodogApplyMs(val);
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




// === RoboDog: camera image quality =========================================
// What upstream leaves on the table. The camera is set up once, at
// FRAMESIZE_QVGA with `jpeg_quality = 63` -- the *worst* value on the 0-63
// scale, where lower is better -- and the sensor itself is then touched
// exactly once, for `set_saturation(s, 2)`. Everything else keeps the driver's
// defaults. F2's two reference frames show what that produces: 320x240 with
// visible block artefacts, oversaturated in light, green and noisy in the dark.
//
// Three different things limit that picture, and they are worth keeping apart,
// because only the first one is what "exposure" means:
//
//   Exposure  the OV2640's auto-exposure aims low and its gain ceiling is
//             conservative, so an indoor scene comes out dark. `ae_level`
//             shifts what the AEC aims for, `aec2` adds the DSP's own longer
//             integration, `gainceiling` allows more analogue gain.
//   Detail    `jpeg_quality` is compression, not exposure. At 63 the sensor's
//             output is thrown away after being exposed and before it reaches
//             the network, and no amount of exposure tuning brings it back.
//   Colour    white balance, gamma and lens correction are all off by default
//             in this driver generation, and cost nothing to switch on.
//
// None of it is free. Longer integration is motion blur on a robot that walks,
// more gain is more noise, and a bigger frame is a bigger JPEG -- which the
// serial `snap` path pays for in seconds at 115200 baud. So the values below
// aim at "clearly better indoors", not at the sensor's maximum, and every one
// of them moves at runtime: see robodogCameraSet, reachable as `cam_<name>`
// over both transports. See ASSUMPTIONS F4, F5 and F6.

// How far up `cam_size` may go. The frame buffer is allocated once, by
// esp_camera_init, for the frame size it is given -- and set_framesize only
// writes sensor registers afterwards, it never grows that buffer. Asking for
// more than we booted with is how you end up with no image at all, which would
// look exactly like the camera fault F2 turned out not to be. Recorded at
// bring-up, enforced in robodogCameraSet.
static framesize_t ROBODOG_CAM_MAX_SIZE = FRAMESIZE_QVGA;

// The sensor does not act on a register write immediately: the frame already
// in flight was exposed under the old settings, and the AEC needs a few more
// to converge. Whoever changes a value and grabs a picture right away would
// otherwise measure the setting *before* it and conclude nothing happened. So
// a change is remembered here, and the next snap drops that many frames first.
// Only after a change -- an unchanged camera costs nothing.
static int ROBODOG_CAM_SETTLE = 4;
static bool ROBODOG_CAM_CHANGED = false;

// Not every setter exists. The OV2640 driver of this core generation leaves
// some of these function pointers null -- sharpness and denoise are the known
// ones -- and calling through a null pointer reboots the ESP32. It would do it
// inside webServerInit, before Wi-Fi is up, with nothing left to report it
// with. So every sensor write in this file goes through here.
#define ROBODOG_CAM_SET(cam, setter, value)                                   \
  do {                                                                        \
    if ((cam) && (cam)->setter) { (cam)->setter((cam), (value)); }             \
  } while (0)

// Our defaults, applied after a successful init and by `cam_reset`.
static void robodogCameraTune(sensor_t *s){
  if (!s) { return; }

  // Exposure: keep it automatic -- a robot that walks from a window into a
  // corridor cannot hold one fixed exposure -- but aim it higher, and let the
  // AEC use more gain and the DSP's longer integration to get there.
  ROBODOG_CAM_SET(s, set_exposure_ctrl, 1);   // AEC on
  ROBODOG_CAM_SET(s, set_aec2,          1);   // plus the DSP's own AEC
  ROBODOG_CAM_SET(s, set_ae_level,      2);   // aim at the top of -2..+2
  ROBODOG_CAM_SET(s, set_gain_ctrl,     1);   // AGC on
  // 16X, not the driver's 2X and not the sensor's 128X: the ceiling only
  // matters in the dark, where 128X trades the noise floor for brightness.
  if (s->set_gainceiling) { s->set_gainceiling(s, GAINCEILING_16X); }

  // Colour and correction. All of these are what the vendor's own web UI
  // switches on, and none of them costs frame rate.
  ROBODOG_CAM_SET(s, set_whitebal, 1);        // AWB
  ROBODOG_CAM_SET(s, set_awb_gain, 1);        // AWB gain: the green cast
  ROBODOG_CAM_SET(s, set_wb_mode,  0);        // auto, not a lighting preset
  ROBODOG_CAM_SET(s, set_raw_gma,  1);        // gamma: the shadows
  ROBODOG_CAM_SET(s, set_lenc,     1);        // lens correction: the corners
  ROBODOG_CAM_SET(s, set_bpc,      1);        // bad pixel correction
  ROBODOG_CAM_SET(s, set_wpc,      1);        // white pixel correction
  ROBODOG_CAM_SET(s, set_dcw,      1);        // downsize with interpolation

  // Upstream's `set_saturation(s, 2)` is the top of the scale, and it is why
  // the reference frame's blues bloom. Neutral, and let the AWB do the work.
  ROBODOG_CAM_SET(s, set_saturation, 0);
  ROBODOG_CAM_SET(s, set_contrast,   0);
  ROBODOG_CAM_SET(s, set_brightness, 0);      // real exposure first, lift later

  // Compression. 63 is the worst the scale has; 10 is what Espressif's own
  // camera example uses whenever there is memory for it.
  ROBODOG_CAM_SET(s, set_quality, 10);

  ROBODOG_CAM_CHANGED = true;
}

// Called on the config *before* esp_camera_init: that is the only moment the
// frame buffer size can still be chosen. PSRAM decides how much there is to
// choose from, and whether this unit has any is not documented anywhere we
// trust -- so it is asked, not assumed. See ASSUMPTIONS F4.
extern void robodogCameraConfig(camera_config_t *config){
  if (psramFound()) {
    // Room for a second frame buffer, so a browser holding the stream no
    // longer blocks snap the way fb_count = 1 does.
    config->frame_size   = FRAMESIZE_SVGA;    // 800x600
    config->jpeg_quality = 10;
    config->fb_count     = 2;
  } else {
    // Internal DRAM only, shared with Wi-Fi, the OLED buffer and the servo
    // tables. VGA is four times the vendor's QVGA and still fits; going
    // higher here is what makes init fail with ESP_ERR_NO_MEM.
    config->frame_size   = FRAMESIZE_VGA;     // 640x480
    config->jpeg_quality = 12;
    config->fb_count     = 1;
  }
  ROBODOG_CAM_MAX_SIZE = config->frame_size;
}

// Called with whatever esp_camera_init returned. A bigger frame is a want; a
// working camera is a need -- so if the buffer did not fit, fall back to the
// vendor's own frame size rather than leave the robot blind. Returns the error
// the caller should go on reporting.
extern esp_err_t robodogCameraStart(camera_config_t *config, esp_err_t err){
  if (err != ESP_OK) {
    Serial.print("ROBODOG: camera init 0x");
    Serial.print(err, HEX);
    Serial.println(" at our frame size -- retrying at the vendor's QVGA");
    // The init failed, so the driver holds nothing; deinit answers
    // ESP_ERR_INVALID_STATE in that case and changes nothing.
    esp_camera_deinit();
    config->frame_size   = FRAMESIZE_QVGA;
    config->jpeg_quality = 12;
    config->fb_count     = 1;
    ROBODOG_CAM_MAX_SIZE = config->frame_size;
    err = esp_camera_init(config);
  }
  if (err == ESP_OK) {
    robodogCameraTune(esp_camera_sensor_get());
  }
  return err;
}

// Drop the frames that were still exposed under the previous settings.
extern void robodogCameraSettle(){
  if (!ROBODOG_CAM_CHANGED) { return; }
  ROBODOG_CAM_CHANGED = false;
  for (int i = 0; i < ROBODOG_CAM_SETTLE; i++) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) { return; }
    esp_camera_fb_return(fb);
  }
}

// One line of key=value, so a tuning session over the serial console can read
// what is in the sensor rather than what it believes it asked for.
// The same values robodogCameraReport prints, as JSON for the HTTP caller.
// Over Wi-Fi the serial console is not readable, so without this a host can
// set the camera but never learn what it holds -- and a teach UI showing what
// it *asked for* rather than what the sensor has is a UI that lies as soon as
// anything else touches the camera.
//
// `size_max` is the one value nothing else can discover: the frame buffer is
// allocated once, before esp_camera_init, and a host has no way to know
// whether this robot got VGA or fell back to QVGA (ASSUMPTIONS F4).
//
// Returns the length written. The buffer is caller-owned and the snprintf is
// bounded -- this runs on the device, where an overrun is a reboot.
extern int robodogCameraJson(char *out, size_t n){
  sensor_t *s = esp_camera_sensor_get();
  if (!s) { return snprintf(out, n, "{\"camera\":false}"); }
  camera_status_t *st = &s->status;
  return snprintf(out, n,
    "{\"camera\":true,\"size\":%d,\"size_max\":%d,\"quality\":%d,"
    "\"ae_level\":%d,\"aec\":%d,\"aec2\":%d,\"aec_value\":%d,"
    "\"agc\":%d,\"agc_gain\":%d,\"gainceiling\":%d,"
    "\"brightness\":%d,\"contrast\":%d,\"saturation\":%d,"
    "\"raw_gma\":%d,\"lenc\":%d,"
    "\"awb\":%d,\"awb_gain\":%d,\"wb_mode\":%d,"
    "\"hmirror\":%d,\"vflip\":%d,\"psram\":%d}",
    (int)st->framesize, (int)ROBODOG_CAM_MAX_SIZE, (int)st->quality,
    (int)st->ae_level, (int)st->aec, (int)st->aec2, (int)st->aec_value,
    (int)st->agc, (int)st->agc_gain, (int)st->gainceiling,
    (int)st->brightness, (int)st->contrast, (int)st->saturation,
    (int)st->raw_gma, (int)st->lenc,
    (int)st->awb, (int)st->awb_gain, (int)st->wb_mode,
    (int)st->hmirror, (int)st->vflip, psramFound() ? 1 : 0);
}

extern void robodogCameraReport(){
  sensor_t *s = esp_camera_sensor_get();
  if (!s) { Serial.println("ROBODOG: cam absent"); return; }
  camera_status_t *st = &s->status;
  Serial.print("ROBODOG: cam size=");   Serial.print((int)st->framesize);
  Serial.print("/");                    Serial.print((int)ROBODOG_CAM_MAX_SIZE);
  Serial.print(" quality=");            Serial.print((int)st->quality);
  Serial.print(" ae_level=");           Serial.print((int)st->ae_level);
  Serial.print(" aec=");                Serial.print((int)st->aec);
  Serial.print(" aec2=");               Serial.print((int)st->aec2);
  Serial.print(" aec_value=");          Serial.print((int)st->aec_value);
  Serial.print(" agc=");                Serial.print((int)st->agc);
  Serial.print(" agc_gain=");           Serial.print((int)st->agc_gain);
  Serial.print(" gainceiling=");        Serial.print((int)st->gainceiling);
  Serial.print(" awb=");                Serial.print((int)st->awb);
  Serial.print(" brightness=");         Serial.print((int)st->brightness);
  Serial.print(" contrast=");           Serial.print((int)st->contrast);
  Serial.print(" saturation=");         Serial.print((int)st->saturation);
  Serial.print(" psram=");              Serial.println(psramFound() ? 1 : 0);
}

// Runtime tuning, one integer per parameter. Reachable as `cam_<name>` from
// serial JSON and from /control alike, so a tuning session does not depend on
// which transport happens to be up. Returns 0 for a name it does not know.
extern int robodogCameraSet(const char *name, int val){
  sensor_t *s = esp_camera_sensor_get();
  if (!name) { return 0; }

  // Housekeeping first: these three touch no sensor register of their own.
  if (!strcmp(name, "report")) { robodogCameraReport(); return 1; }
  if (!strcmp(name, "settle")) {
    ROBODOG_CAM_SETTLE = (val < 0) ? 0 : ((val > 30) ? 30 : val);
    Serial.print("ROBODOG: cam settle="); Serial.println(ROBODOG_CAM_SETTLE);
    return 1;
  }
  if (!strcmp(name, "reset")) { robodogCameraTune(s); robodogCameraReport(); return 1; }

  if (!s) { Serial.println("ROBODOG: cam absent"); return 1; }
  int known = 1;

  if      (!strcmp(name, "quality"))     { ROBODOG_CAM_SET(s, set_quality,        val); }
  else if (!strcmp(name, "size")) {
    // Clamped, not rejected: see ROBODOG_CAM_MAX_SIZE above. Asking for more
    // than the frame buffer holds is the one setting that ends in no image.
    if (val < 0) { val = 0; }
    if (val > (int)ROBODOG_CAM_MAX_SIZE) { val = (int)ROBODOG_CAM_MAX_SIZE; }
    if (s->set_framesize) { s->set_framesize(s, (framesize_t)val); }
  }
  else if (!strcmp(name, "ae_level"))    { ROBODOG_CAM_SET(s, set_ae_level,       val); }
  else if (!strcmp(name, "aec"))         { ROBODOG_CAM_SET(s, set_exposure_ctrl,  val); }
  else if (!strcmp(name, "aec2"))        { ROBODOG_CAM_SET(s, set_aec2,           val); }
  else if (!strcmp(name, "aec_value"))   { ROBODOG_CAM_SET(s, set_aec_value,      val); }
  else if (!strcmp(name, "agc"))         { ROBODOG_CAM_SET(s, set_gain_ctrl,      val); }
  else if (!strcmp(name, "agc_gain"))    { ROBODOG_CAM_SET(s, set_agc_gain,       val); }
  else if (!strcmp(name, "gainceiling")) {
    if (s->set_gainceiling) { s->set_gainceiling(s, (gainceiling_t)val); }
  }
  else if (!strcmp(name, "brightness"))  { ROBODOG_CAM_SET(s, set_brightness,     val); }
  else if (!strcmp(name, "contrast"))    { ROBODOG_CAM_SET(s, set_contrast,       val); }
  else if (!strcmp(name, "saturation"))  { ROBODOG_CAM_SET(s, set_saturation,     val); }
  else if (!strcmp(name, "sharpness"))   { ROBODOG_CAM_SET(s, set_sharpness,      val); }
  else if (!strcmp(name, "denoise"))     { ROBODOG_CAM_SET(s, set_denoise,        val); }
  else if (!strcmp(name, "awb"))         { ROBODOG_CAM_SET(s, set_whitebal,       val); }
  else if (!strcmp(name, "awb_gain"))    { ROBODOG_CAM_SET(s, set_awb_gain,       val); }
  else if (!strcmp(name, "wb_mode"))     { ROBODOG_CAM_SET(s, set_wb_mode,        val); }
  else if (!strcmp(name, "raw_gma"))     { ROBODOG_CAM_SET(s, set_raw_gma,        val); }
  else if (!strcmp(name, "lenc"))        { ROBODOG_CAM_SET(s, set_lenc,           val); }
  else if (!strcmp(name, "bpc"))         { ROBODOG_CAM_SET(s, set_bpc,            val); }
  else if (!strcmp(name, "wpc"))         { ROBODOG_CAM_SET(s, set_wpc,            val); }
  else if (!strcmp(name, "dcw"))         { ROBODOG_CAM_SET(s, set_dcw,            val); }
  else if (!strcmp(name, "hmirror"))     { ROBODOG_CAM_SET(s, set_hmirror,        val); }
  else if (!strcmp(name, "vflip"))       { ROBODOG_CAM_SET(s, set_vflip,          val); }
  else if (!strcmp(name, "effect"))      { ROBODOG_CAM_SET(s, set_special_effect, val); }
  else { known = 0; }

  if (known) {
    ROBODOG_CAM_CHANGED = true;
    Serial.print("ROBODOG: cam "); Serial.print(name);
    Serial.print("="); Serial.println(val);
  } else {
    Serial.print("ROBODOG: cam unknown parameter "); Serial.println(name);
  }
  return known;
}
// === end RoboDog ===========================================================

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
  robodogCameraSettle();   // whatever was just changed, let it reach the frame
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

  // === RoboDog: choose the frame buffer before it gets allocated ============
  // The three lines above are the vendor's, and esp_camera_init below is the
  // last moment any of them can still change. See robodogCameraConfig.
  robodogCameraConfig(&config);
  // === end RoboDog ==========================================================

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

  // === RoboDog: apply our settings, and retry smaller if the buffer did not fit
  err = robodogCameraStart(&config, err);
  // === end RoboDog ===========================================================

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
