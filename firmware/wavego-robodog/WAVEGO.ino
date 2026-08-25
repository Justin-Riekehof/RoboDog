// WIFI settings in app_httpd.cpp

// OLED screen display.
// === PAGE 1 ===
// Row1: [WIFI_MODE]:IP ADDRESS
//       [WIFI_MODE]: 1 as [AP] mode, it will not connect other wifi.
//                    2 as [STA] mode, it will connect to known wifi.
// Row2: [RSSI]
// Row3: [STATUS] [A] [B] [C] [D]
//       [A]: 1 as forward. -1 as backward.
//       [B]: 1 as turn right. -1 as turn left.
//       [C]: 0 as not in the debugMode. Robot can be controled.
//            1 as in the debugMode. You can debug the servos.
//       [D]: 0 as not in any function mode. Robot can be controled to move around.
//            1 as in steady mode. keep balancing.
//            2 as stayLow action.
//            3 as handshake action.
//            4 as jump action.
//            5, 6 and 7 as ActionA, ActionB and ActionC.
//            8 as servos moving to initPos, initPos is the middle angle for servos.
//            9 as servos moving to middlePos, middlePos is the middle angle for program.
// Row4: [BATTERY]
// === PAGE 2 ===
// [SHOW] DebugMode via wire config.
// [ . . . o o ]  LED G21 G15 G12 3V3
// [ . . . . . ]  TX  RX  GND  5V  5V
//    <SWITCH>
extern IPAddress IP_ADDRESS = (0, 0, 0, 0);
extern byte WIFI_MODE = 0; // select WIFI_MODE in app_httpd.cpp
// RoboDog: loopTask handle, so the prio command can reach it from other tasks.
TaskHandle_t ROBODOG_LOOP_TASK = NULL;
extern void getWifiStatus();
extern int WIFI_RSSI = 0;

// gait type ctrl
// 0: simpleGait(DiagonalGait).
// 1: triangularGait.
extern int GAIT_TYPE = 0;

int CODE_DEBUG = 0;


// ctrl interface.
// refer to OLED screen display for more detail information.
extern int moveFB = 0;
extern int moveLR = 0;
extern int debugMode = 0;
extern int funcMode  = 0;

// === RoboDog: on-device link watchdog =======================================
// Not in the upstream firmware. The stock protocol LATCHES: one "move" walks
// until a stop arrives, so a lost link leaves the robot walking and no command
// can reach it (ASSUMPTIONS B9/D10). This is the on-device answer.
//
// OFF BY DEFAULT, deliberately. The vendor's own web UI sends nothing while the
// robot walks, so a watchdog that was always on would break it. A host that
// wants the net asks for it ("watchdog", value in ms) and then has to keep
// talking -- any command feeds it, "ping" feeds it without changing anything.
extern unsigned long ROBODOG_WATCHDOG_MS = 0;   // 0 = disabled
extern unsigned long ROBODOG_LAST_CMD_MS = 0;
extern int ROBODOG_WATCHDOG_TRIPPED = 0;

// RoboDog: defined in app_httpd.cpp, which has the camera headers.
extern void robodogSnap(int withImage);
// RoboDog: defined further down, next to the ramp itself. Declared here
// because the watchdog below has to be able to end a ramp in flight.
extern void robodogRampCancel();
extern void robodogApplyMs(int val);
// RoboDog: camera tuning, also defined in app_httpd.cpp.
extern int robodogCameraSet(const char *name, int val);

extern void robodogWatchdogFeed(){
  ROBODOG_LAST_CMD_MS = millis();
  ROBODOG_WATCHDOG_TRIPPED = 0;
}

// Called from loop(). Only ever stops something that is actually moving, so a
// parked or trimming robot is never disturbed by it.
extern void robodogWatchdogCheck(){
  if(ROBODOG_WATCHDOG_MS == 0){return;}
  if(ROBODOG_WATCHDOG_TRIPPED){return;}
  if(moveFB == 0 && moveLR == 0){return;}
  // Unsigned arithmetic, so this stays correct across the millis() rollover.
  if(millis() - ROBODOG_LAST_CMD_MS <= ROBODOG_WATCHDOG_MS){return;}
  moveFB = 0;
  moveLR = 0;
  funcMode = 2;                    // stayLow: down, low and stable
  robodogRampCancel();             // stayLow owns the servos now, not a ramp
  ROBODOG_WATCHDOG_TRIPPED = 1;
  Serial.println("WATCHDOG: link lost, stopping and crouching");
}
// === end RoboDog ============================================================
float gestureUD = 0;
float gestureLR = 0;
float gestureOffSetMax = 15;
float gestureSpeed = 2;
int STAND_STILL = 0;

const char* UPPER_IP = "";
int UPPER_TYPE = 0;
unsigned long LAST_JSON_SEND;
int JSON_SEND_INTERVAL;


// import libraries.
#include "InitConfig.h"
#include "ServoCtrl.h"
#include "PreferencesConfig.h"
#include <ArduinoJson.h>

StaticJsonDocument<200> docReceive;
StaticJsonDocument<100> docSend;
TaskHandle_t threadings;

// placeHolders.
void webServerInit();


// === RoboDog: pose commands, twelve servos at once =========================
// The gap the stock firmware leaves: `sconfig` moves ONE servo, relatively, in
// PWM counts, and it switches the control loop off (ASSUMPTIONS D5/D8). There
// is no way to say "put the feet here" -- which is what pose teach-in and
// keyframe playback need.
//
// This is that command, in two halves on purpose:
//
//   {"var":"leg","val":1,"x":16,"y":95,"z":25}   stage one leg
//   {"var":"apply"}                              move all twelve, together
//
// Two rules shape the implementation, both learned the hard way:
//
// 1. NEVER touch I2C from this task. `GoalPosAll()` writes the PCA9685 over
//    Wire, and `robotCtrl()` already calls it from loop() on every pass. Doing
//    it from the serial task too crashes the ESP32 core 1.0.x I2C driver with
//    IntegerDivideByZero in i2cProcQueue -- verified on the device 2026-08-22,
//    backtrace decoded. So `apply` only publishes values; the main loop is the
//    single writer to the bus.
//
// 2. Stage into a shadow copy, not into GoalPWM. The loop re-applies GoalPWM
//    continuously, so writing legs into it one at a time would move them one at
//    a time. The shadow is copied over in one go, which is what makes the pose
//    simultaneous rather than a servo-by-servo ripple.
//
// `singleLegCtrl` runs the firmware's own IK and writes GoalPWM[] without
// applying it. Using the firmware's IK rather than our own is deliberate:
// `robodog.kinematics` is a line-faithful port of it, so twin and robot compute
// the same pose.
//
// Frame: x forward, y down towards the ground, z outward, millimetres --
// exactly the frame `LegTarget` uses. Legs 1..4 are FL, HL, FR, HR.
//
// There is NO workspace check here. The host's safety supervisor does that
// before anything is sent; a hand-typed command over serial bypasses it, which
// is the same trade the firmware already makes everywhere else.
int ROBODOG_STAGED[16];
int ROBODOG_HAS_STAGED = 0;

extern void robodogLegTarget(int leg, double x, double y, double z){
  if(leg < 1 || leg > 4){
    Serial.println("ROBODOG: leg must be 1..4");
    return;
  }
  int live[16];
  for(int i = 0; i < 16; i++){live[i] = GoalPWM[i];}
  if(!ROBODOG_HAS_STAGED){
    for(int i = 0; i < 16; i++){ROBODOG_STAGED[i] = live[i];}
    ROBODOG_HAS_STAGED = 1;
  }
  // Compute into GoalPWM (that is where singleLegCtrl writes), harvest the
  // result into the shadow, then put the live pose back untouched.
  for(int i = 0; i < 16; i++){GoalPWM[i] = ROBODOG_STAGED[i];}
  singleLegCtrl((uint8_t)leg, x, y, z);
  for(int i = 0; i < 16; i++){ROBODOG_STAGED[i] = GoalPWM[i];}
  for(int i = 0; i < 16; i++){GoalPWM[i] = live[i];}

  Serial.print("ROBODOG: staged leg ");Serial.print(leg);
  Serial.print(" x=");Serial.print(x);
  Serial.print(" y=");Serial.print(y);
  Serial.print(" z=");Serial.println(z);
}

// --- moving to the staged pose over time, rather than in one step --------
// GoalPosAll() writes GoalPWM straight to the servo driver with no ramp of its
// own, and the loop does that every STEP_DELAY (4 ms) whether anything changed
// or not. So the robot already refreshes its servos some 250 times a second
// while a host over Wi-Fi manages ten poses -- 24 of every 25 writes repeat a
// value, and each pose that does arrive lands as a jump.
//
// Filling those writes in is what this does: `apply` may name a duration, and
// GoalPWM then travels to the staged pose across it, one step per loop pass.
// Smoothness stops depending on the link -- five poses a second from the host
// become hundreds on the robot.
//
// Interpolation is LINEAR on purpose. The host already eases whole routines
// (the teach session's cosine), and easing each segment on top of that would
// decelerate into every one of them -- a pulse at 10 Hz, which is worse than
// the steps it replaces.
// How long the next apply should take, in ms. 0 = jump, which is what every
// caller got before this existed. Clamped: a ramp nobody ends is a robot
// that ignores its own controls for as long as the number says.
unsigned long ROBODOG_APPLY_MS = 0;
const unsigned long ROBODOG_APPLY_MS_MAX = 5000;

int ROBODOG_RAMP_FROM[16];
int ROBODOG_RAMP_TO[16];
unsigned long ROBODOG_RAMP_START = 0;
unsigned long ROBODOG_RAMP_MS = 0;   // 0 = no ramp in flight

extern void robodogRampStep(){
  if(ROBODOG_RAMP_MS == 0){return;}
  unsigned long gone = millis() - ROBODOG_RAMP_START;
  if(gone >= ROBODOG_RAMP_MS){
    for(int i = 0; i < 16; i++){GoalPWM[i] = ROBODOG_RAMP_TO[i];}
    ROBODOG_RAMP_MS = 0;   // arrived; stop stepping until the next apply
    return;
  }
  // Integer maths, and the multiply before the divide: at 4 ms per pass a
  // float here would be free, but the rounding of `from + (to-from)*gone/ms`
  // is what keeps the last step landing exactly on the target above.
  for(int i = 0; i < 16; i++){
    long span = (long)ROBODOG_RAMP_TO[i] - (long)ROBODOG_RAMP_FROM[i];
    GoalPWM[i] = ROBODOG_RAMP_FROM[i] + (int)((span * (long)gone) / (long)ROBODOG_RAMP_MS);
  }
}

// Abandon a ramp in flight. Anything that takes the servos over -- a gait, a
// canned animation, the watchdog -- must, or the ramp keeps writing GoalPWM
// underneath it.
// `val` on apply/pose is how long the move should take, in ms. 0 keeps the
// old behaviour exactly -- a jump -- so nothing that predates this changes.
extern void robodogApplyMs(int val){
  if(val <= 0){ROBODOG_APPLY_MS = 0; return;}
  ROBODOG_APPLY_MS = (unsigned long)val;
  if(ROBODOG_APPLY_MS > ROBODOG_APPLY_MS_MAX){ROBODOG_APPLY_MS = ROBODOG_APPLY_MS_MAX;}
}

extern void robodogRampCancel(){
  ROBODOG_RAMP_MS = 0;
}

extern void robodogApply(){
  if(!ROBODOG_HAS_STAGED){
    Serial.println("ROBODOG: nothing staged");
    return;
  }
  // Leave the states that would overwrite the pose on the next loop pass: a
  // gait writes GoalPWM every iteration, funcMode 8/9 rewrite it forever
  // (ASSUMPTIONS D11), and debugMode suspends the loop altogether. STAND_STILL
  // matters too -- at 0 the loop calls standMassCenter() once more and the
  // pose would be gone before it was ever seen.
  moveFB = 0;
  moveLR = 0;
  funcMode = 0;
  debugMode = 0;
  STAND_STILL = 1;
  if(ROBODOG_APPLY_MS > 0){
    // Travel there instead of arriving there. From wherever GoalPWM is now,
    // which may itself be mid-ramp -- that is what makes a stream of poses
    // continuous rather than a sequence of restarts.
    for(int i = 0; i < 16; i++){
      ROBODOG_RAMP_FROM[i] = GoalPWM[i];
      ROBODOG_RAMP_TO[i]   = ROBODOG_STAGED[i];
    }
    ROBODOG_RAMP_START = millis();
    ROBODOG_RAMP_MS    = ROBODOG_APPLY_MS;
  } else {
    robodogRampCancel();
    for(int i = 0; i < 16; i++){GoalPWM[i] = ROBODOG_STAGED[i];}
  }
  ROBODOG_HAS_STAGED = 0;
  // No GoalPosAll() here, on purpose -- see rule 1. The loop applies this
  // within STEP_DELAY, and it is the only thing allowed to drive the bus.
  Serial.println("ROBODOG: pose applied");
}
// === end RoboDog ===========================================================


// var(variable), val(value).                  
// === RoboDog: the "var" key as a plain string ===============================
// Needed for prefix matching. The vendor's chain compares whole names, and the
// camera parameters are a family ("cam_ae_level", "cam_quality", ...) rather
// than one name -- listing two dozen of them here would put the same table in
// two files and let them drift apart.
static const char* robodogVar(){
  const char *v = docReceive["var"];
  return v ? v : "";
}
// === end RoboDog ===========================================================

void serialCtrl(){
  if (Serial.available()){
    // Read the JSON document from the "link" serial port
    DeserializationError err = deserializeJson(docReceive, Serial);

    if (err == DeserializationError::Ok){
      UPPER_TYPE = 1;
      docReceive["val"].as<int>();

      int val = docReceive["val"];

      robodogWatchdogFeed();   // RoboDog: any accepted command counts as "alive"

      if(docReceive["var"] == "funcMode"){
        debugMode = 0;
        gestureUD = 0;
        gestureLR = 0;
        robodogRampCancel();   // RoboDog: an animation owns the servos from here
        if(val == 1){
          if(funcMode == 1){funcMode = 0;Serial.println("Steady OFF");}
          else if(funcMode == 0){funcMode = 1;Serial.println("Steady ON");}
        }
        else{
          funcMode = val;
          Serial.println(val);
        }
      }

      else if(docReceive["var"] == "move"){
        debugMode = 0;
        funcMode  = 0;
        robodogRampCancel();   // RoboDog: a gait owns the servos from here
        digitalWrite(BUZZER, HIGH);
        switch(val){
          case 1: moveFB = 1; Serial.println("Forward");break;
          case 2: moveLR =-1; Serial.println("TurnLeft");break;
          case 3: moveFB = 0; Serial.println("FBStop");break;
          case 4: moveLR = 1; Serial.println("TurnRight");break;
          case 5: moveFB =-1; Serial.println("Backward");break;
          case 6: moveLR = 0; Serial.println("LRStop");break;
        }
      }

      else if(docReceive["var"] == "ges"){
        debugMode = 0;
        funcMode  = 0;
        switch(val){
          case 1: gestureUD += gestureSpeed;if(gestureUD > gestureOffSetMax){gestureUD = gestureOffSetMax;}break;
          case 2: gestureUD -= gestureSpeed;if(gestureUD <-gestureOffSetMax){gestureUD =-gestureOffSetMax;}break;
          case 3: break;
          case 4: gestureLR -= gestureSpeed;if(gestureLR <-gestureOffSetMax){gestureLR =-gestureOffSetMax;}break;
          case 5: gestureLR += gestureSpeed;if(gestureLR > gestureOffSetMax){gestureLR = gestureOffSetMax;}break;
          case 6: break;
        }
        pitchYawRollHeightCtrl(gestureUD, gestureLR, 0, 0);
      }

      else if(docReceive["var"] == "light"){
        switch(val){
          case 0: setSingleLED(0,matrix.Color(0, 0, 0));setSingleLED(1,matrix.Color(0, 0, 0));break;
          case 1: setSingleLED(0,matrix.Color(0, 32, 255));setSingleLED(1,matrix.Color(0, 32, 255));break;
          case 2: setSingleLED(0,matrix.Color(255, 32, 0));setSingleLED(1,matrix.Color(255, 32, 0));break;
          case 3: setSingleLED(0,matrix.Color(32, 255, 0));setSingleLED(1,matrix.Color(32, 255, 0));break;
          case 4: setSingleLED(0,matrix.Color(255, 255, 0));setSingleLED(1,matrix.Color(255, 255, 0));break;
          case 5: setSingleLED(0,matrix.Color(0, 255, 255));setSingleLED(1,matrix.Color(0, 255, 255));break;
          case 6: setSingleLED(0,matrix.Color(255, 0, 255));setSingleLED(1,matrix.Color(255, 0, 255));break;
          case 7: setSingleLED(0,matrix.Color(255, 64, 32));setSingleLED(1,matrix.Color(32, 64, 255));break;
        }
      }

      else if(docReceive["var"] == "buzzer"){
        switch(val){
          case 0: digitalWrite(BUZZER, HIGH);break;
          case 1: digitalWrite(BUZZER, LOW);break;
        }
      }

      // RoboDog: arm the link watchdog. val = milliseconds, 0 disables it.
      else if(docReceive["var"] == "watchdog"){
        ROBODOG_WATCHDOG_MS = (val > 0) ? (unsigned long)val : 0;
        Serial.print("watchdog:");Serial.println(val);
      }

      // RoboDog: keep-alive that changes nothing else.
      else if(docReceive["var"] == "ping"){
        Serial.println("ping");
      }

      // === RoboDog: grab one camera frame to this console ===================
      // val=1 also dumps the JPEG as base64. See robodogSnap() in app_httpd.cpp.
      else if(docReceive["var"] == "snap"){
        robodogSnap(val);
      }

      // Camera parameters, "cam_<name>", the same names /control takes.
      // {"var":"cam_ae_level","val":2}, {"var":"cam_report","val":0}.
      else if(strncmp(robodogVar(), "cam_", 4) == 0){
        robodogCameraSet(robodogVar() + 4, val);
      }

      // Stage one leg's foot target; see robodogLegTarget above.
      else if(docReceive["var"] == "leg"){
        robodogLegTarget(val, docReceive["x"], docReceive["y"], docReceive["z"]);
      }

      // Move every staged leg at once.
      else if(docReceive["var"] == "apply"){
        robodogApplyMs(val);
        robodogApply();
      }

      // Gait priority, live. {"var":"prio","val":1..12} -- see setup().
      else if(docReceive["var"] == "prio"){
        if(ROBODOG_LOOP_TASK != NULL && val >= 1 && val <= 12){
          vTaskPrioritySet(ROBODOG_LOOP_TASK, val);
          Serial.print("prio:");Serial.println(val);
        }
      }

      // Battery voltage and current, as one JSON line. The vendor measures
      // both every pass (INA219, allDataUpdate) and then shows them only on
      // the OLED -- jsonSend() would have exported them but is never called.
      // First asked for on 2026-08-25, chasing a POWERON_RESET mid-session:
      // whether the rail sags under load is exactly the question a battery
      // answer settles. {"var":"vol","val":0}
      else if(docReceive["var"] == "vol"){
        Serial.print("{\"vol\":");Serial.print(loadVoltage_V);
        Serial.print(",\"ma\":");Serial.print(current_mA);
        Serial.println("}");
      }

      // The IMU, as one JSON line. val = the last sequence already seen, so
      // {"var":"imu","val":0} dumps everything the ring still holds. This is
      // the bring-up answer to "is the gyroscope alive at all", which nothing
      // in the vendor firmware could ever be asked.
      else if(docReceive["var"] == "imu"){
        // static: this task has 4000 bytes of stack (xTaskCreate below), and
        // the HTTP twin of this buffer measurably overflowed httpd's 4096 --
        // see app_httpd.cpp. One task, one caller, no reentrancy.
        static char json[2048];
        int len = robodogImuJson(json, sizeof(json), (uint32_t)(val < 0 ? 0 : val));
        if(len > 0){Serial.println(json);}
        else{Serial.println("{\"imu\":false}");}
      }
      // === end RoboDog ======================================================

      // === RoboDog: drain trailing whitespace ==============================
      // Or the NEXT pass stalls the robot for a full second:
      // deserializeJson returns at the closing brace and
      // leaves a sender's newline in the buffer; the next serialCtrl() sees
      // Serial.available(), calls the parser on it, and ArduinoJson's stream
      // reader BUSY-WAITS through Serial's 1000 ms timeout hoping a document
      // follows -- on this task, which outranks loop(), so the gait and the
      // IMU freeze for that second. Measured 2026-08-25: exactly one ~1045 ms
      // sampling hole per newline-terminated command, zero without the
      // newline. Only whitespace is drained, so a second queued command
      // survives.
      while (Serial.available() > 0){
        int rdPeek = Serial.peek();
        if (rdPeek=='\n' || rdPeek=='\r' || rdPeek==' ' || rdPeek=='\t'){ Serial.read(); }
        else { break; }
      }
      // === end RoboDog ======================================================
    }


      // else if(docReceive['var'] == "ip"){
      //     UPPER_IP = docReceive['ip'];
      // }
     

    else {
      while (Serial.available() > 0)
        Serial.read();
    }
  }
}


void jsonSend(){
  if(millis() - LAST_JSON_SEND > JSON_SEND_INTERVAL || millis() < LAST_JSON_SEND){
    docSend["vol"] = loadVoltage_V;
    serializeJson(docSend, Serial);
    LAST_JSON_SEND = millis();
  }
}


void robotThreadings(void *pvParameter){
  delay(3000);
  while(1){
    serialCtrl();
    delay(25);
  }
}


void threadingsInit(){
  xTaskCreate(&robotThreadings, "RobotThreadings", 4000, NULL, 5, &threadings);
}



// === RoboDog: the whole IMU, sampled on the device ==========================
//
// Lives here rather than beside the sensor's own setup in InitConfig.h, which
// stays byte-identical to the vendored reference -- the fork adds, it does not
// edit (see tests/test_firmware_fork.py).
//
// The vendor reads three accelerometer axes into globals and then never calls
// the function: `accXYZUpdate()` is commented out of loop(). Gyroscope and
// magnetometer are configured by nobody and read by nobody, so six of the nine
// axes have never left the chip.
//
// Why this is buffered rather than polled a sample at a time: a request over
// Wi-Fi costs 78-140 ms (measured 2026-08-22), so a host polling for single
// samples would see about eight a second -- useless for integrating a gyro,
// which is the only reason to have one. The device samples at a fixed rate
// into a ring, and one request collects everything since the host's last
// sequence number. Ten requests a second then carry fifty samples a second.
//
// The rule this firmware taught us applies here too (see README): only loop()
// may touch I2C. robodogImuSample() is called from there and nowhere else; the
// HTTP handler and the serial console only format what is already in the ring.

#define ROBODOG_IMU_SLOTS 64
// This was 20 ms ("a few percent of the loop") and both halves of that
// sentence were wrong, reported by the operator on the robot's first Wi-Fi
// drive with the IMU aboard (2026-08-25): the gait was visibly slower and
// jerkier than the 08-22 firmware. Measured, the read costs ~12 ms of I2C on
// the servos' own bus -- at a 20 ms gate that is a third of the loop, not a
// few percent. And the cost lands twice: every HTTP poll formats the
// accumulated samples on the httpd task, which outranks loop() and preempts
// the gait precisely while the robot drives, since driving is when the host
// polls. Both scale with this one number. 100 ms cuts them 5x; ~10 Hz still
// oversamples a gait whose pitch swings at about 2 Hz, and the host filter
// weighs by per-sample timestamps, so it does not care about the rate.
#define ROBODOG_IMU_PERIOD_MS 100
// The magnetometer sits behind the auxiliary bus and costs its own
// transaction, and a heading drifts slowly -- it does not need the gyro's rate.
#define ROBODOG_IMU_MAG_PERIOD_MS 100

struct RobodogImuSample {
  uint32_t t;                  // device millis() when the sample was taken
  float ax, ay, az;            // g
  float gx, gy, gz;            // deg/s
};

RobodogImuSample ROBODOG_IMU_RING[ROBODOG_IMU_SLOTS];
// RoboDog: the longest gap between two loop() passes since the last imu
// request, in ms. loop() is where the gait advances, so this number IS the
// gait's health -- and it survives a session, so the next serial connection
// can read what a Wi-Fi drive did to the loop after the fact. Reset on read.
volatile uint32_t ROBODOG_LOOP_MAX_MS = 0;
uint32_t ROBODOG_LOOP_PREV_MS = 0;

void robodogLoopWatch(){
  uint32_t now = millis();
  if (ROBODOG_LOOP_PREV_MS != 0){
    uint32_t gap = now - ROBODOG_LOOP_PREV_MS;
    if (gap > ROBODOG_LOOP_MAX_MS){ ROBODOG_LOOP_MAX_MS = gap; }
  }
  ROBODOG_LOOP_PREV_MS = now;
}
// Sequence of the NEWEST sample written, counting from 1. The host sends back
// the last one it saw, so nothing has to be acknowledged and a lost reply
// simply gets the samples again on the next request.
volatile uint32_t ROBODOG_IMU_SEQ = 0;
uint32_t ROBODOG_IMU_LAST_MS = 0;
uint32_t ROBODOG_IMU_MAG_MS = 0;
float ROBODOG_MAG_X = 0, ROBODOG_MAG_Y = 0, ROBODOG_MAG_Z = 0;
uint32_t ROBODOG_MAG_T = 0;
bool ROBODOG_MAG_OK = false;
float ROBODOG_IMU_TEMP = 0;

// Configure the two sensors the vendor leaves untouched. Called from setup(),
// straight after InitICM20948().
void robodogImuInit(){
  // 500 deg/s: a walking quadruped's body turns far slower, but a footfall is
  // a jolt, and a gyro that saturates during a step corrupts the integral
  // exactly when it matters.
  myIMU.setGyrRange(ICM20948_GYRO_RANGE_500);
  myIMU.setGyrDLPF(ICM20948_DLPF_6);
  myIMU.setGyrSampleRateDivider(10);

  // The magnetometer is deliberately NOT initialised, and both halves of that
  // decision were measured on this robot on 2026-08-24/25:
  //
  //  * `myIMU.initMagnetometer()` **hangs setup() forever**, intermittently.
  //    It succeeded on the first boot of the evening and blocked on every one
  //    after, with no watchdog, no panic and no output -- the robot simply
  //    never finished starting. The AK09916 sits behind the ICM20948's own
  //    auxiliary I2C master, and the library's init spins waiting for it.
  //  * It would buy nothing if it worked. The one reading we got measured
  //    about 200 uT, against an earth field of 25-65: the robot's own magnets
  //    and motor currents dominate it, so it is not a compass here without a
  //    hard-iron calibration nobody has done (ASSUMPTIONS G9).
  //
  // Trading a guaranteed boot for a reading known to be useless is not a
  // trade. Yaw comes from the gyroscope alone and is reported as "turned since
  // the run started" rather than as a heading, which is what the behaviour
  // needs anyway. Reviving this means reading the aux bus by hand with a
  // timeout -- not calling a library function that cannot fail safely.
  ROBODOG_MAG_OK = false;
}

// Called from loop() only. Rate-gated, so it costs one I2C burst per period
// rather than one per pass.
// The read half of the profiler: hand out the worst gap and start fresh.
uint32_t robodogLoopMaxTake(){
  uint32_t worst = ROBODOG_LOOP_MAX_MS;
  ROBODOG_LOOP_MAX_MS = 0;
  return worst;
}

void robodogImuSample(){
  uint32_t now = millis();
  if(now - ROBODOG_IMU_LAST_MS < ROBODOG_IMU_PERIOD_MS){return;}
  ROBODOG_IMU_LAST_MS = now;

  myIMU.readSensor();
  xyzFloat g = myIMU.getGValues();
  xyzFloat r = myIMU.getGyrValues();

  uint32_t seq = ROBODOG_IMU_SEQ + 1;
  RobodogImuSample *slot = &ROBODOG_IMU_RING[seq % ROBODOG_IMU_SLOTS];
  slot->t = now;
  slot->ax = g.x; slot->ay = g.y; slot->az = g.z;
  slot->gx = r.x; slot->gy = r.y; slot->gz = r.z;
  // Published last, so a reader that sees this sequence sees a whole sample.
  ROBODOG_IMU_SEQ = seq;

  // The vendor's own globals, kept fed for anything that still reads them.
  ACC_X = g.x; ACC_Y = g.y; ACC_Z = g.z;

  // Temperature on its own schedule. It used to ride along with the
  // magnetometer read, which meant switching that off silently switched this
  // off too -- and the gyroscope's bias drifts with temperature, so it is the
  // one of the two actually worth having.
  if(now - ROBODOG_IMU_MAG_MS >= ROBODOG_IMU_MAG_PERIOD_MS){
    ROBODOG_IMU_MAG_MS = now;
    ROBODOG_IMU_TEMP = myIMU.getTemperature();
  }
}

// Everything newer than `since`, as JSON, into `out`. Returns the length.
//
// Stops early rather than truncating mid-value when the buffer would overflow:
// the reply says which sequence it ended at, and the host asks again. Samples
// that fell out of the ring are reported as `dropped` rather than silently
// missing -- a gap the host does not know about is a gap it integrates
// straight through.
extern int robodogImuJson(char *out, size_t n, uint32_t since){
  uint32_t newest = ROBODOG_IMU_SEQ;
  uint32_t oldest = (newest > ROBODOG_IMU_SLOTS) ? (newest - ROBODOG_IMU_SLOTS + 1) : 1;
  uint32_t from = (since + 1 > oldest) ? (since + 1) : oldest;
  uint32_t dropped = (from > since + 1) ? (from - since - 1) : 0;

  int len = snprintf(out, n,
    "{\"imu\":true,\"rate\":%d,\"lmax\":%lu,\"seq\":%lu,\"dropped\":%lu,\"mag_ok\":%d,"
    "\"mag\":[%.2f,%.2f,%.2f],\"mag_t\":%lu,\"temp\":%.1f,\"s\":[",
    (int)(1000 / ROBODOG_IMU_PERIOD_MS), (unsigned long)robodogLoopMaxTake(), (unsigned long)newest,
    (unsigned long)dropped, ROBODOG_MAG_OK ? 1 : 0,
    ROBODOG_MAG_X, ROBODOG_MAG_Y, ROBODOG_MAG_Z,
    (unsigned long)ROBODOG_MAG_T, ROBODOG_IMU_TEMP);
  if(len < 0 || (size_t)len >= n){return -1;}

  uint32_t last = since;
  for(uint32_t seq = from; seq <= newest; seq++){
    RobodogImuSample *s = &ROBODOG_IMU_RING[seq % ROBODOG_IMU_SLOTS];
    // Slack for one more sample plus the closing brackets.
    if((size_t)len + 96 >= n){break;}
    int wrote = snprintf(out + len, n - len,
      "%s[%lu,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f]",
      (seq == from) ? "" : ",", (unsigned long)s->t,
      s->ax, s->ay, s->az, s->gx, s->gy, s->gz);
    if(wrote < 0 || (size_t)(len + wrote) >= n){break;}
    len += wrote;
    last = seq;
  }
  int tail = snprintf(out + len, n - len, "],\"last\":%lu}", (unsigned long)last);
  if(tail < 0 || (size_t)(len + tail) >= n){return -1;}
  return len + tail;
}
// === end RoboDog ============================================================

void setup() {
  Wire.begin(S_SDA, S_SCL);
  Serial.begin(115200);

  // WIRE DEBUG INIT.
  wireDebugInit();
  
  // INA219 INIT.
  Serial.println("ROBODOG: setup ina219");
  InitINA219();

  // BUZZER INIT.
  InitBuzzer();

  // RGB INIT
  InitRGB();

  // PCA9685 INIT.
  Serial.println("ROBODOG: setup pca9685");
  ServoSetup();

  // SSD1306 INIT.
  Serial.println("ROBODOG: setup oled");
  InitScreen();

  // EEPROM INIT.
  preferencesSetup();

  // Standup for ICM20948 calibrating.
  delay(100);
  setSingleLED(0,matrix.Color(0, 128, 255));
  setSingleLED(1,matrix.Color(0, 128, 255));
  Serial.println("ROBODOG: setup standup");
  standMassCenter(0, 0);GoalPosAll();delay(1000);
  setSingleLED(0,matrix.Color(255, 128, 0));
  setSingleLED(1,matrix.Color(255, 128, 0));
  delay(500);

  // ICM20948 INIT.
  Serial.println("ROBODOG: setup imu (autoOffsets -- hold still)");
  InitICM20948();
  Serial.println("ROBODOG: setup imu gyro+mag");
  robodogImuInit();   // RoboDog: gyroscope and magnetometer, which the vendor configures nowhere
  Serial.println("ROBODOG: setup wifi");

  // WEBCTRL INIT. WIFI settings included.
  webServerInit();

  // === RoboDog: the gait outranks the web servers ==========================
  // loop() -- where robotCtrl() computes the gait and writes all twelve
  // servos -- runs in loopTask at priority 1. The two httpd tasks run at 5,
  // unpinned, and the MJPEG stream handler is a near-continuous worker: with
  // a client attached it loops fb_get -> chunk-send at full tilt, preempting
  // the gait at will. Measured by elimination 2026-08-25: every session that
  // held the stream walked slow and hitching, the same session without it was
  // smooth, and a serial-driven gait with no Wi-Fi client ran clean 114 ms
  // loop intervals all along.
  //
  // This IDF's httpd_config_t has no core_id yet, so the servers cannot be
  // pinned away. Raising loopTask above them inverts the preemption instead:
  // the gait computes whenever it needs to, and the servers run in the gaps
  // the loop's own I2C waits leave open -- which on a pass that spends most
  // of its time on the servo bus is most of the time -- and in core 0's
  // leftovers beside Wi-Fi. The stream loses frames under load; the walk
  // does not lose steps. That is the right direction for a robot.
  ROBODOG_LOOP_TASK = xTaskGetCurrentTaskHandle();
  vTaskPrioritySet(NULL, 7);
  //
  // 7 beat the httpd tasks and the walk stayed sluggish anyway (2026-08-25),
  // which points one level up: this core generation's camera driver runs its
  // DMA task at priority 10, PINNED to this core. So the priority is also a
  // runtime knob -- {"var":"prio"} on both transports -- because finding the
  // rung that actually clears the ladder is a dose-response experiment, and
  // an experiment per reflash is a bad afternoon. Clamped to 1..12: above 12
  // sit the system's own tasks (esp_timer, tcpip, Wi-Fi), and a gait that
  // outranked those would take the radio down with every step.
  // === end RoboDog ==========================================================

  // RGB LEDs on.
  delay(500);
  setSingleLED(0,matrix.Color(0, 32, 255));
  setSingleLED(1,matrix.Color(255, 32, 0));

  // update data on screen.
  allDataUpdate();

  // threadings start.
  threadingsInit();
}


// main loop.
void loop() {
  robotCtrl();
  allDataUpdate();
  wireDebugDetect();
  robodogRampStep();        // RoboDog: carry GoalPWM towards the staged pose
  robodogLoopWatch();       // RoboDog: how long since the last pass -- gait health
  robodogImuSample();       // RoboDog: the only place the IMU is read (I2C rule)
  robodogWatchdogCheck();   // RoboDog: stop by ourselves if the host went away
}


// <<<<<<<<<<=== Devices on Board ===>>>>>>>>>>>>

// --- --- ---   --- --- ---   --- --- ---
// ICM20948 init. --- 9-axis sensor for motion tracking.
// InitICM20948();

// read and update the pitch, raw and roll data from ICM20948.
// accXYZUpdate();


// --- --- ---   --- --- ---   --- --- ---
// INA219 init. --- DC current/voltage sensor.
// InitINA219();

// read and update the voltage and current data from INA219.
// InaDataUpdate();


// --- --- ---   --- --- ---   --- --- ---
// RGB INIT. --- the 2 RGB LEDs in front of the robot. LED_NUM = 0, 1.
// InitRGB();

// control RGB LED. 0 <= R, G, B <= 255.
// setSingleLED(LED_NUM, matrix.Color(R, G, B));


// --- --- ---   --- --- ---   --- --- ---
// SSD1306 INIT. --- OLED Screen
// InitScreen();

// show the newest data on the OLED screen.
// for more information you can refer to <OLED screen display>.
// screenDataUpdate();


// --- --- ---   --- --- ---   --- --- ---
// BUZZER INIT. --- the device that make a sound.
// InitBuzzer();

// BUZZER on.
// digitalWrite(BUZZER, HIGH);

// BUZZER off.
// digitalWrite(BUZZER, LOW);


// --- --- ---   --- --- ---   --- --- ---
// PCA9685 INIT.
// ServoSetup();

// all servos move to the middle position of the servos.
// initPosAll();


// <<<<<<<<<<<<=== Servos/Legs/Motion Ctrl ===>>>>>>>>>>>>>>>

// --- --- ---   --- --- ---   --- --- ---
// all servos move to the middle position of the program. 
// the position that you have to debug it to make it moves to.
// middlePosAll();

// control a single servo by updating the angle data in GoalPWM[].
// once it is called, call GoalPosALL() to move all of the servos.
// goalPWMSet(servoNum, angleInput);

// all servos move to goal position(GoalPWM[]).
// GoalPosAll();

// Ctrl a single leg of WAVEGO, once it is called, call GoalPosALL() to move all of the servos.
// input (x,y) position and return angle alpha and angle beta.
//     O  X  O                 O ------ [BODY]      I(1)  ^  III(3)
//    /         .              |          |               |
//   /    |        O           |          |               |
//  O     y     .              |          |         II(2) ^  IV(4)
//   \.   |  .                 |          |
//    \.  .                    |          |
//     O  |                    |          |
//  .                          |          |
//   \.   |                    |          |
//    \-x-X                    X----z-----O
// ---------------------------------------------------------------
// x, y, z > 0
// singleLegCtrl(LEG_NUM, X, Y, Z);


// --- --- ---   --- --- ---   --- --- ---
// a simple gait to ctrl the robot.
// GlobalInput changes between 0-1.
// use directionAngle to ctrl the direction.
// once it is called, call GoalPosALL() to move all of the servos.
// simpleGait(GlobalInput, directionAngle);

// a triangular gait to ctrl the robot.
// GlobalInput changes between 0-1.
// use directionAngle to ctrl the direction.
// once it is called, call GoalPosALL() to move all of the servos.
// triangularGait(GlobalInput, directionAngle);


// --- --- ---   --- --- ---   --- --- ---
// Stand and adjust mass center.
//     ^
//     a
//     |
// <-b-M
// a,b > 0
// standMassCenter(aInput, bInput);


// --- --- ---   --- --- ---   --- --- ---
// ctrl pitch yaw and roll.
// pitchInput (-, +), if > 0, look up.
// yawInput   (-, +), if > 0, look right.
// rollInput  (-, +), if > 0, lean right.
// 75 < input < 115
// pitchYawRoll(pitchInput, yawInput, rollInput);


// --- --- ---   --- --- ---   --- --- ---
// balancing function.
// once it is called, call GoalPosALL() to move all of the servos.
// balancing();


// --- --- ---   --- --- ---   --- --- ---
// the default function to control robot.
// robotCtrl();


// <<<<<<<<<<<<<<<=== Save Data Permanently ===>>>>>>>>>>>>>>>>>>

// EEPROM INIT.
// preferencesSetup();

// save the current position of the servoNum in EEPROM.
// servoConfigSave(servoNum);

// read the saved middle position data of the servos from EEPROM.
// middleUpdate();