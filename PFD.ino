#include <SPI.h>
#include <Wire.h>
#include <TFT_eSPI.h>
#include <MPU6050.h>
#include <TinyGPSPlus.h>
#include <math.h>
#include <string.h>

#include "PhoneMirror.h"

#define SCREEN_W 320
#define SCREEN_H 240
#define ATT_X 60
#define ATT_W 200

static const int GPS_RX_PIN = 16;
static const int GPS_TX_PIN = 17;
static const uint32_t GPS_BAUD = 9600;

static const bool DISPLAY_SPEED_KMH = true;

static const char* WIFI_SSID = "PFD-ESP32";
static const char* WIFI_PASS = "pfd12345";
static const uint32_t WS_BROADCAST_MS = 100;

static const uint32_t IMU_PERIOD_MS = 10;
static const uint32_t RENDER_PERIOD_MS = 33;
static const uint32_t IMU_TIMEOUT_MS = 200;

static const uint32_t GPS_TIMEOUT_MS = 2500;
static const bool GPS_WAIT_FOR_LOCK = true;
static const uint32_t GPS_WAIT_MAX_MS = 0;
static const uint32_t GPS_FIX_SPLASH_MS = 1200;

static const uint32_t BLINK_PERIOD_MS = 300;
static const uint32_t IMU_RECOVER_CHECK_MS = 250;
static const uint16_t IMU_BAD_READ_LIMIT = 20;

static const uint16_t CAL_SAMPLES = 60;
static const uint16_t CAL_SAMPLE_DELAY_MS = 10;

static const float ANGLE_LPF_ALPHA = 0.25f;
static const float SLIP_LPF_ALPHA  = 0.30f;
static const float RATE_LPF_ALPHA  = 0.35f;

static const float GPS_ALT_TAU_S = 0.70f;
static const float GPS_SPD_TAU_S = 0.15f;

static const float SPEED_ZERO_THRESH_KMH = 1.5f;
static const float SPEED_ZERO_THRESH_KT  = 0.8f;
static const uint8_t SPEED_ZERO_HOLD_UPDATES = 3;

static const uint8_t MPU_DLPF_MODE = 3;

static inline int sx(int screenX) { return screenX - ATT_X; }
static inline int iround(float v) { return (int)lroundf(v); }
static inline float lpf(float prev, float in, float alpha) { return prev + alpha * (in - prev); }
static inline float metersToFeet(float m) { return m * 3.280839895f; }

static inline float alphaFromTau(float dt_s, float tau_s) {
  if (tau_s <= 1e-3f) return 1.0f;
  if (dt_s <= 0.0f) return 0.0f;
  float a = 1.0f - expf(-dt_s / tau_s);
  if (a < 0.0f) a = 0.0f;
  if (a > 1.0f) a = 1.0f;
  return a;
}

enum SystemState : uint8_t { SYS_INIT=0, SYS_RUN, SYS_DEGRADED, SYS_FAULT };

struct FlightData {
  float pitch_deg{};
  float roll_deg{};
  float slip_px{};
  float turnRate_dps{};
  float altitude_ft{};
  float speed_disp{};
  bool imu_valid{};
  uint32_t last_imu_ok_ms{};
  bool gps_valid{};
  uint32_t last_gps_ok_ms{};
};

TFT_eSPI tft;
TFT_eSprite attSpr(&tft);
MPU6050 mpu;
TinyGPSPlus gps;
HardwareSerial GPSSerial(2);

TinyGPSCustom ggaFixQuality(gps, "GPGGA", 6);
TinyGPSCustom ggaSatsUsed  (gps, "GPGGA", 7);
TinyGPSCustom ggaHdop      (gps, "GPGGA", 8);

PhoneMirror mirror;

static uint16_t SKY_COLOR;
static const uint16_t GROUND_COLOR = TFT_BROWN;
static const uint16_t FG = TFT_WHITE;
static const uint16_t PANEL = TFT_DARKGREY;
static const uint16_t BALL = TFT_WHITE;
static const uint16_t ROLL_PTR = TFT_YELLOW;
static const uint16_t WARN_ORANGE = 0xFD20;

static SystemState g_state = SYS_INIT;
static FlightData g_fd{};

static uint16_t g_bad_read_count = 0;
static const char* g_fault_msg = "FAULT: IMU";
static uint32_t g_last_recover_check_ms = 0;

static float g_pitch_zero = 0.0f;
static float g_roll_zero  = 0.0f;
static bool g_zero_valid  = false;

static uint32_t g_last_gps_filter_ms = 0;
static uint8_t g_speed_zero_cnt = 0;

static bool initDisplay();
static bool initIMU();
static void initGPS();
static void serviceGPS();
static int getFixQuality();
static int getSatsUsed();
static float getHdop();
static const char* fixTypeString(int fixQuality);
static void gpsBootWait();
static void gpsFixSplash(uint32_t duration_ms);
static bool calibrateZeroOffsets();

static void taskIMU(uint32_t now_ms);
static void taskRender(uint32_t now_ms);

static bool readIMURaw(int16_t& ax, int16_t& ay, int16_t& az, int16_t& gx, int16_t& gy, int16_t& gz);
static bool computeAttitudeFromAccel(float ax_g, float ay_g, float az_g, float& pitch_deg, float& roll_deg);

static void drawPFD(uint32_t now_ms);
static void drawStatusOverlay(uint32_t now_ms);
static void drawBootCalibrationMessage();
static void clearBootCalibrationMessage();

static void sprDrawAttitude();
static void sprDrawPitchLadder();
static void sprDrawRollScale();
static void sprDrawAircraftSymbol();
static void sprDrawTurnRateBottom();
static void sprDrawSlipSkidBottom();
static void drawAirspeedTape();
static void drawAltitudeTape();

static void sprFillRotRect(int cx, int cy, float angRad, int w, int h, int color) {
  float c = cosf(angRad), s = sinf(angRad);
  int hw = w / 2, hh = h / 2;

  int lx0=-hw, ly0=-hh, lx1= hw, ly1=-hh, lx2= hw, ly2= hh, lx3=-hw, ly3= hh;

  int x0 = cx + iround(lx0 * c - ly0 * s);
  int y0 = cy + iround(lx0 * s + ly0 * c);
  int x1 = cx + iround(lx1 * c - ly1 * s);
  int y1 = cy + iround(lx1 * s + ly1 * c);
  int x2 = cx + iround(lx2 * c - ly2 * s);
  int y2 = cy + iround(lx2 * s + ly2 * c);
  int x3 = cx + iround(lx3 * c - ly3 * s);
  int y3 = cy + iround(lx3 * s + ly3 * c);

  attSpr.fillTriangle(x0,y0,x1,y1,x2,y2,color);
  attSpr.fillTriangle(x0,y0,x2,y2,x3,y3,color);
}

void setup() {
  Serial.begin(115200);

  if (!initDisplay()) { g_state = SYS_FAULT; g_fault_msg = "FAULT: DISP"; return; }

  mirror.beginAP(WIFI_SSID, WIFI_PASS);
  Serial.print("AP IP: "); Serial.println(mirror.apIP());

  drawBootCalibrationMessage();

  if (!initIMU()) { clearBootCalibrationMessage(); g_state = SYS_FAULT; g_fault_msg = "FAULT: IMU"; return; }
  initGPS();

  g_zero_valid = calibrateZeroOffsets();
  clearBootCalibrationMessage();

  gpsBootWait();

  g_state = g_zero_valid ? SYS_RUN : SYS_DEGRADED;
  tft.fillScreen(TFT_BLACK);
}

void loop() {
  static uint32_t next_imu_ms = 0;
  static uint32_t next_render_ms = 0;

  const uint32_t now = millis();
  if (!next_imu_ms) next_imu_ms = now + IMU_PERIOD_MS;
  if (!next_render_ms) next_render_ms = now + RENDER_PERIOD_MS;

  serviceGPS();

  if ((int32_t)(now - next_imu_ms) >= 0) { taskIMU(now); next_imu_ms = now + IMU_PERIOD_MS; }
  if ((int32_t)(now - next_render_ms) >= 0) { taskRender(now); next_render_ms = now + RENDER_PERIOD_MS; }

  MirrorPayload p {
    g_fd.pitch_deg, g_fd.roll_deg, g_fd.slip_px, g_fd.turnRate_dps,
    g_fd.speed_disp, g_fd.altitude_ft,
    (uint8_t)g_state,
    (uint8_t)(g_fd.gps_valid ? 1 : 0),
    (uint8_t)(g_fd.imu_valid ? 1 : 0),
    DISPLAY_SPEED_KMH ? "km/h" : "kt"
  };

  mirror.broadcast(now, WS_BROADCAST_MS, p);
  mirror.tick();
}

static bool initDisplay() {
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextFont(2);

  SKY_COLOR = tft.color565(0, 110, 140);

  attSpr.setColorDepth(16);
  attSpr.createSprite(ATT_W, SCREEN_H);
  attSpr.setTextFont(2);
  attSpr.setTextSize(1);
  attSpr.fillSprite(TFT_BLACK);

  if (attSpr.getPointer() == nullptr) return false;
  return true;
}

static bool initIMU() {
  Wire.begin(21, 22);
  mpu.initialize();
  if (!mpu.testConnection()) return false;
  mpu.setDLPFMode(MPU_DLPF_MODE);
  return true;
}

static void initGPS() {
  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
}

static void serviceGPS() {
  const uint32_t now = millis();
  while (GPSSerial.available() > 0) gps.encode((char)GPSSerial.read());

  const int fq = getFixQuality();
  const bool fixOk = (fq > 0) && gps.location.isValid() && (gps.location.age() < 2000);

  if (fixOk) {
    float dt_s = 1.0f;
    if (g_last_gps_filter_ms) {
      dt_s = (float)(now - g_last_gps_filter_ms) / 1000.0f;
      if (dt_s < 0.001f) dt_s = 0.001f;
      if (dt_s > 2.0f) dt_s = 2.0f;
    }
    g_last_gps_filter_ms = now;

    bool updated = false;

    if (gps.altitude.isValid() && gps.altitude.isUpdated()) {
      const float alt_ft = metersToFeet((float)gps.altitude.meters());
      if (!g_fd.gps_valid) g_fd.altitude_ft = alt_ft;
      else g_fd.altitude_ft = lpf(g_fd.altitude_ft, alt_ft, alphaFromTau(dt_s, GPS_ALT_TAU_S));
      updated = true;
    }

    if (gps.speed.isValid() && gps.speed.isUpdated()) {
      float spd = DISPLAY_SPEED_KMH ? (float)gps.speed.kmph() : (float)gps.speed.knots();
      const float zth = DISPLAY_SPEED_KMH ? SPEED_ZERO_THRESH_KMH : SPEED_ZERO_THRESH_KT;

      if (spd < zth) {
        if (g_speed_zero_cnt < 255) g_speed_zero_cnt++;
        if (g_speed_zero_cnt >= SPEED_ZERO_HOLD_UPDATES) spd = 0.0f;
      } else {
        g_speed_zero_cnt = 0;
      }

      if (!g_fd.gps_valid) g_fd.speed_disp = spd;
      else g_fd.speed_disp = lpf(g_fd.speed_disp, spd, alphaFromTau(dt_s, GPS_SPD_TAU_S));
      updated = true;
    }

    if (updated) {
      g_fd.gps_valid = true;
      g_fd.last_gps_ok_ms = now;
    }
  }

  if (g_fd.gps_valid && ((now - g_fd.last_gps_ok_ms) > GPS_TIMEOUT_MS)) {
    g_fd.gps_valid = false;
  }
}

static int getFixQuality() {
  const char* v = ggaFixQuality.value();
  if (!v || v[0] == '\0') return -1;
  return atoi(v);
}
static int getSatsUsed() {
  const char* v = ggaSatsUsed.value();
  if (!v || v[0] == '\0') return -1;
  return atoi(v);
}
static float getHdop() {
  const char* v = ggaHdop.value();
  if (!v || v[0] == '\0') return NAN;
  return (float)atof(v);
}
static const char* fixTypeString(int fixQuality) {
  switch (fixQuality) {
    case 0: return "NO FIX";
    case 1: return "GPS FIX";
    case 2: return "DGPS";
    default: return "UNKNOWN";
  }
}

static void gpsBootWait() {
  const uint32_t start = millis();
  uint32_t lastDraw = 0;

  tft.fillScreen(TFT_BLACK);
  tft.setTextFont(2);

  while (true) {
    serviceGPS();

    const int fq = getFixQuality();
    const bool fixOk = (fq > 0) && gps.location.isValid() && (gps.location.age() < 2000);
    const bool timedOut = (GPS_WAIT_MAX_MS > 0) ? ((millis() - start) > GPS_WAIT_MAX_MS) : false;

    if (GPS_WAIT_FOR_LOCK) {
      if (fixOk) { gpsFixSplash(GPS_FIX_SPLASH_MS); break; }
      if (timedOut) break;
    } else {
      if ((millis() - start) > 5000) break;
    }

    const uint32_t now = millis();
    if ((now - lastDraw) >= 200) {
      lastDraw = now;
      tft.fillScreen(TFT_BLACK);
      tft.setTextColor(FG, TFT_BLACK);

      tft.setCursor(10, 10); tft.print("GPS ACQUIRE");
      tft.setCursor(10, 30); tft.print("Fix: "); tft.print(fq >= 0 ? fixTypeString(fq) : "--");
      tft.setCursor(10, 50); tft.print("Sats: "); tft.print(getSatsUsed());
      tft.setCursor(10, 70); tft.print("HDOP: ");
      { float hd = getHdop(); if (isfinite(hd)) tft.printf("%.2f", hd); else tft.print("--"); }

      tft.setCursor(10, 105);
      tft.setTextColor(WARN_ORANGE, TFT_BLACK);
      tft.print("Phone: http://192.168.4.1");

      tft.setCursor(10, 200);
      if (fixOk) { tft.setTextColor(TFT_GREEN, TFT_BLACK); tft.print("FIX OK"); }
      else { tft.setTextColor(WARN_ORANGE, TFT_BLACK); tft.print("ACQUIRING..."); }
    }

    MirrorPayload p {
      g_fd.pitch_deg, g_fd.roll_deg, g_fd.slip_px, g_fd.turnRate_dps,
      g_fd.speed_disp, g_fd.altitude_ft,
      (uint8_t)g_state,
      (uint8_t)(g_fd.gps_valid ? 1 : 0),
      (uint8_t)(g_fd.imu_valid ? 1 : 0),
      DISPLAY_SPEED_KMH ? "km/h" : "kt"
    };
    mirror.broadcast(now, WS_BROADCAST_MS, p);
    mirror.tick();

    delay(5);
  }
}

static void gpsFixSplash(uint32_t duration_ms) {
  const uint32_t start = millis();
  uint32_t lastBlink = 0;
  bool on = true;

  while ((millis() - start) < duration_ms) {
    serviceGPS();
    const uint32_t now = millis();

    if ((now - lastBlink) >= 200) {
      lastBlink = now;
      on = !on;

      tft.fillScreen(TFT_BLACK);
      tft.setTextFont(4);
      tft.setTextColor(on ? TFT_GREEN : FG, TFT_BLACK);
      tft.setCursor(35, 90);
      tft.print("GPS FIX OK");

      tft.setTextFont(2);
      tft.setTextColor(FG, TFT_BLACK);
      tft.setCursor(55, 140);
      tft.print("Starting PFD...");
    }

    mirror.tick();
    delay(5);
  }
}

static bool calibrateZeroOffsets() {
  float sumPitch = 0.0f, sumRoll = 0.0f;
  uint16_t good = 0;

  for (uint16_t i = 0; i < CAL_SAMPLES; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    if (readIMURaw(ax, ay, az, gx, gy, gz)) {
      float ax_g = (float)ax / 16384.0f;
      float ay_g = (float)ay / 16384.0f;
      float az_g = (float)az / 16384.0f;

      float p, r;
      if (computeAttitudeFromAccel(ax_g, ay_g, az_g, p, r)) {
        sumPitch += p;
        sumRoll  += r;
        good++;
      }
    }
    delay(CAL_SAMPLE_DELAY_MS);
  }

  if (good < (CAL_SAMPLES / 2)) { g_pitch_zero = 0.0f; g_roll_zero = 0.0f; return false; }
  g_pitch_zero = sumPitch / (float)good;
  g_roll_zero  = sumRoll  / (float)good;
  return true;
}

static void taskIMU(uint32_t now_ms) {
  if (g_state == SYS_FAULT) {
    if (!g_last_recover_check_ms || (now_ms - g_last_recover_check_ms) >= IMU_RECOVER_CHECK_MS) {
      g_last_recover_check_ms = now_ms;
      if (mpu.testConnection()) {
        int16_t ax, ay, az, gx, gy, gz;
        if (readIMURaw(ax, ay, az, gx, gy, gz)) {
          g_bad_read_count = 0;
          g_fd.imu_valid = false;
          g_fd.last_imu_ok_ms = now_ms;
          g_state = SYS_RUN;
        }
      }
    }
    return;
  }

  int16_t ax, ay, az, gx, gy, gz;
  const bool ok = readIMURaw(ax, ay, az, gx, gy, gz);

  if (!ok) {
    if (g_bad_read_count < 0xFFFF) g_bad_read_count++;
    if (g_bad_read_count >= IMU_BAD_READ_LIMIT) { g_state = SYS_FAULT; g_fault_msg = "FAULT: IMU"; return; }
  } else {
    g_bad_read_count = 0;

    float ax_g = (float)ax / 16384.0f;
    float ay_g = (float)ay / 16384.0f;
    float az_g = (float)az / 16384.0f;

    float pitch_meas = g_fd.pitch_deg;
    float roll_meas  = g_fd.roll_deg;

    if (computeAttitudeFromAccel(ax_g, ay_g, az_g, pitch_meas, roll_meas)) {
      if (g_zero_valid) { pitch_meas -= g_pitch_zero; roll_meas -= g_roll_zero; }

      pitch_meas = constrain(pitch_meas, -30.0f, 30.0f);
      roll_meas  = constrain(roll_meas,  -60.0f, 60.0f);

      float slip_meas = constrain(ay_g * 35.0f, -35.0f, 35.0f);
      float turn_meas = (float)gx / 131.0f;

      if (!g_fd.imu_valid) {
        g_fd.pitch_deg = pitch_meas;
        g_fd.roll_deg  = roll_meas;
        g_fd.slip_px   = slip_meas;
        g_fd.turnRate_dps = turn_meas;
      } else {
        g_fd.pitch_deg = lpf(g_fd.pitch_deg, pitch_meas, ANGLE_LPF_ALPHA);
        g_fd.roll_deg  = lpf(g_fd.roll_deg,  roll_meas,  ANGLE_LPF_ALPHA);
        g_fd.slip_px   = lpf(g_fd.slip_px,   slip_meas,  SLIP_LPF_ALPHA);
        g_fd.turnRate_dps = lpf(g_fd.turnRate_dps, turn_meas, RATE_LPF_ALPHA);
      }

      g_fd.imu_valid = true;
      g_fd.last_imu_ok_ms = now_ms;
      if (g_state == SYS_DEGRADED) g_state = SYS_RUN;
    }
  }

  if (g_fd.imu_valid) {
    if ((now_ms - g_fd.last_imu_ok_ms) > IMU_TIMEOUT_MS) g_state = SYS_DEGRADED;
  } else {
    if (g_state != SYS_FAULT) g_state = SYS_DEGRADED;
  }
}

static void taskRender(uint32_t now_ms) { drawPFD(now_ms); }

static bool readIMURaw(int16_t& ax, int16_t& ay, int16_t& az, int16_t& gx, int16_t& gy, int16_t& gz) {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  return !((ax==0)&&(ay==0)&&(az==0)&&(gx==0)&&(gy==0)&&(gz==0));
}

static bool computeAttitudeFromAccel(float ax_g, float ay_g, float az_g, float& pitch_deg, float& roll_deg) {
  if (!isfinite(ax_g) || !isfinite(ay_g) || !isfinite(az_g)) return false;
  const float denom = sqrtf((ay_g * ay_g) + (az_g * az_g));
  if (!isfinite(denom) || denom < 1e-6f) return false;
  pitch_deg = atan2f(-ax_g, denom) * (180.0f / PI);
  roll_deg  = -atan2f(ay_g, az_g)  * (180.0f / PI);
  return isfinite(pitch_deg) && isfinite(roll_deg);
}

static void drawPFD(uint32_t now_ms) {
  drawAirspeedTape();
  drawAltitudeTape();

  sprDrawAttitude();
  sprDrawPitchLadder();
  sprDrawRollScale();
  sprDrawAircraftSymbol();
  sprDrawTurnRateBottom();
  sprDrawSlipSkidBottom();

  drawStatusOverlay(now_ms);
  attSpr.pushSprite(ATT_X, 0);
}

static void drawStatusOverlay(uint32_t now_ms) {
  attSpr.fillRect(0, 0, ATT_W, 18, TFT_BLACK);

  if (g_state == SYS_FAULT) {
    const bool on = ((now_ms / BLINK_PERIOD_MS) & 1U) == 0U;
    if (on) {
      attSpr.setTextColor(WARN_ORANGE, TFT_BLACK);
      attSpr.setCursor(4, 4);
      attSpr.print(g_fault_msg);
    }
    return;
  }

  if (g_state == SYS_DEGRADED) {
    attSpr.setTextColor(WARN_ORANGE, TFT_BLACK);
    attSpr.setCursor(4, 4);
    attSpr.print("DEGRADED: IMU");
    return;
  }

  if (!g_fd.gps_valid) {
    const bool on = ((now_ms / BLINK_PERIOD_MS) & 1U) == 0U;
    if (on) {
      attSpr.setTextColor(WARN_ORANGE, TFT_BLACK);
      attSpr.setCursor(4, 4);
      attSpr.print("DEGRADED: GPS");
    }
  }
}

static void sprDrawAttitude() {
  const int cx = sx(SCREEN_W / 2);
  const int cy = SCREEN_H / 2;

  int horizonY = cy + (int)iround(g_fd.pitch_deg * 4.0f);
  attSpr.fillRect(0, 0, ATT_W, SCREEN_H, SKY_COLOR);

  if (horizonY < 0) horizonY = 0;
  if (horizonY > SCREEN_H) horizonY = SCREEN_H;
  attSpr.fillRect(0, horizonY, ATT_W, SCREEN_H - horizonY, GROUND_COLOR);

  const float r = g_fd.roll_deg * DEG_TO_RAD;
  const int len = 260;

  const int x1 = cx - (int)iround(cosf(r) * (float)len);
  const int y1 = horizonY - (int)iround(sinf(r) * (float)len);
  const int x2 = cx + (int)iround(cosf(r) * (float)len);
  const int y2 = horizonY + (int)iround(sinf(r) * (float)len);

  attSpr.drawLine(x1, y1, x2, y2, FG);
}

static void sprDrawPitchLadder() {
  const int cx = sx(SCREEN_W / 2);
  const int cy = SCREEN_H / 2;
  const float r = g_fd.roll_deg * DEG_TO_RAD;

  attSpr.setTextColor(FG, TFT_TRANSPARENT);

  for (int p = -30; p <= 30; p += 5) {
    if (p == 0) continue;

    const int y = cy + (int)iround((g_fd.pitch_deg - (float)p) * 4.0f);
    if (y < -40 || y > SCREEN_H + 40) continue;

    const int len = (p % 10 == 0) ? 40 : 20;
    const int x1 = cx - len;
    const int x2 = cx + len;

    const int xr1 = cx + (int)iround((float)(x1 - cx) * cosf(r) - (float)(y - cy) * sinf(r));
    const int yr1 = cy + (int)iround((float)(x1 - cx) * sinf(r) + (float)(y - cy) * cosf(r));
    const int xr2 = cx + (int)iround((float)(x2 - cx) * cosf(r) - (float)(y - cy) * sinf(r));
    const int yr2 = cy + (int)iround((float)(x2 - cx) * sinf(r) + (float)(y - cy) * cosf(r));

    if (max(xr1, xr2) < 0 || min(xr1, xr2) > ATT_W) continue;

    attSpr.drawLine(xr1, yr1, xr2, yr2, FG);

    if (p % 10 == 0) {
      const int txR = xr2 + 4, tyR = yr2 - 4;
      if (txR >= 0 && txR < ATT_W - 12 && tyR >= 0 && tyR < SCREEN_H - 8) {
        attSpr.setCursor(txR, tyR); attSpr.printf("%d", abs(p));
      }
      const int txL = xr1 - 16, tyL = yr1 - 4;
      if (txL >= 0 && txL < ATT_W - 12 && tyL >= 0 && tyL < SCREEN_H - 8) {
        attSpr.setCursor(txL, tyL); attSpr.printf("%d", abs(p));
      }
    }
  }
}

static void sprDrawRollScale() {
  const int cx = sx(SCREEN_W / 2);
  const int cy = 110;
  const int radius = 110;

  for (int dx = -1; dx <= 1; dx++) attSpr.drawLine(cx+dx, cy-radius, cx+dx, cy-radius+14, FG);

  for (int a = -60; a <= 60; a += 10) {
    const float ar = a * DEG_TO_RAD;
    const bool major = (a % 30 == 0);
    const int w = major ? 2 : 1;
    const int h = major ? 15 : 9;

    const int xI = cx + (int)iround(sinf(ar) * (float)(radius - 6));
    const int yI = cy - (int)iround(cosf(ar) * (float)(radius - 6));
    attSpr.fillRect(xI - w / 2, yI - h / 2, w, h, FG);
  }

  const float rr = constrain(g_fd.roll_deg, -60.0f, 60.0f) * DEG_TO_RAD;

  const int tipX = cx + (int)iround(sinf(rr) * (float)(radius - 18));
  const int tipY = cy - (int)iround(cosf(rr) * (float)(radius - 18));

  const float perpX = cosf(rr);
  const float perpY = sinf(rr);

  const int baseHalf = 8;
  const int baseR = radius - 30;

  const int baseCx = cx + (int)iround(sinf(rr) * (float)baseR);
  const int baseCy = cy - (int)iround(cosf(rr) * (float)baseR);

  const int x2 = baseCx + (int)iround(perpX * (float)baseHalf);
  const int y2 = baseCy + (int)iround(perpY * (float)baseHalf);
  const int x3 = baseCx - (int)iround(perpX * (float)baseHalf);
  const int y3 = baseCy - (int)iround(perpY * (float)baseHalf);

  attSpr.fillTriangle(tipX, tipY, x2, y2, x3, y3, ROLL_PTR);
}

static void sprDrawTurnRateBottom() {
  const int cx = sx(SCREEN_W / 2);
  const int y = SCREEN_H - 40;

  int len = (int)iround(g_fd.turnRate_dps * 3.0f);
  len = constrain(len, -40, 40);

  attSpr.drawFastHLine(cx - 40, y, 80, FG);
  attSpr.drawFastVLine(cx + len, y - 5, 10, FG);
}

static void sprDrawSlipSkidBottom() {
  const int cx = sx(SCREEN_W / 2);
  const int y = SCREEN_H - 22;
  const int w = 90;

  attSpr.drawRect(cx - w/2, y - 8, w, 16, FG);
  attSpr.drawFastVLine(cx, y - 6, 12, FG);

  attSpr.fillCircle(cx + (int)iround(g_fd.slip_px), y, 5, BALL);
}

static void sprDrawAircraftSymbol() {
  const int cx = sx(SCREEN_W / 2);
  const int cy = SCREEN_H / 2;
  const float r = g_fd.roll_deg * DEG_TO_RAD;

  sprFillRotRect(cx, cy, r, 70, 5, FG);
  sprFillRotRect(cx, cy, r, 6, 30, FG);
}

static void drawAirspeedTape() {
  tft.fillRect(0, 0, 60, SCREEN_H, PANEL);
  tft.setTextColor(FG, PANEL);

  tft.setCursor(6, 6);
  tft.print(DISPLAY_SPEED_KMH ? "km/h" : "kt");

  const int centerY = SCREEN_H / 2;
  const int centerSpeed = (int)lroundf(g_fd.speed_disp);

  for (int i = -5; i <= 5; i++) {
    const int y = centerY + i * 20;
    const int val = centerSpeed - i;

    if (y > 0 && y < SCREEN_H) {
      tft.drawFastHLine(40, y, 15, FG);
      if ((val % 5) == 0) { tft.setCursor(4, y - 4); tft.printf("%d", val); }
    }
  }

  tft.fillRect(0, centerY - 20, 60, 40, TFT_BLACK);
  tft.drawRect(0, centerY - 20, 60, 40, FG);
  tft.setCursor(10, centerY - 8);
  tft.printf("%3.0f", g_fd.speed_disp);
}

static void drawAltitudeTape() {
  const int x = SCREEN_W - 60;
  tft.fillRect(x, 0, 60, SCREEN_H, PANEL);
  tft.setTextColor(FG, PANEL);

  const int centerY = SCREEN_H / 2;
  const int centerAlt = (int)lroundf(g_fd.altitude_ft);

  for (int i = -5; i <= 5; i++) {
    const int y = centerY + i * 20;
    const int val = centerAlt - i * 100;

    if (y > 0 && y < SCREEN_H) {
      tft.drawFastHLine(x + 5, y, 15, FG);
      tft.setCursor(x + 5, y - 4);
      tft.printf("%d", val);
    }
  }

  tft.fillRect(x, centerY - 20, 60, 40, TFT_BLACK);
  tft.drawRect(x, centerY - 20, 60, 40, FG);
  tft.setCursor(x + 5, centerY - 8);
  tft.printf("%4.0f", g_fd.altitude_ft);
}

static void drawBootCalibrationMessage() {
  const int boxW = 220, boxH = 40;
  const int bx = (SCREEN_W - boxW) / 2;
  const int by = (SCREEN_H - boxH) / 2;

  tft.fillRect(bx, by, boxW, boxH, TFT_BLACK);
  tft.drawRect(bx, by, boxW, boxH, FG);
  tft.setTextColor(FG, TFT_BLACK);
  tft.setTextFont(2);
  tft.setCursor(bx + 35, by + 14);
  tft.print("CALIBRATING...");
}

static void clearBootCalibrationMessage() {
  const int boxW = 220, boxH = 40;
  const int bx = (SCREEN_W - boxW) / 2;
  const int by = (SCREEN_H - boxH) / 2;
  tft.fillRect(bx, by, boxW, boxH, TFT_BLACK);
}
