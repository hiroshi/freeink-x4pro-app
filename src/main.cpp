// FreeInk SDK — elapsed-time + battery status screen for the Xteink X4 Pro.
//
// Top-center : "Nd HH:MM"  — time since the last boot (setup() resets the RTC
//              to 2000-01-01 00:00:00 on every power-on / reset; light-sleep
//              wakes resume in loop() and keep counting).
// Top-right  : "87%  3.92V" — CW2017 fuel-gauge percent + battery millivolts.
//
// Portrait layout (480 wide x 800 tall).
//
// Redraws once a minute. Between updates the ESP32 light-sleeps: the panel
// controller stays powered (GPIO1 rail held), so its RAM survives as the
// differential baseline and each minute's repaint is a FAST refresh with NO
// black/white flash. One FULL refresh runs at startup; there is no periodic
// scrub (drop in a HALF refresh here later if ghosting proves to accumulate).
//
// Option D per the plan: no deep sleep, no rail cut — trades higher average
// sleep current for a flicker-free clock. The point is to measure that current.

#include <Arduino.h>
#include <time.h>

#include <esp_sleep.h>

#include <BoardConfig.h>
#include <EInkDisplay.h>
#include <FreeInkUIDisplayTarget.h>
#include <XteinkDetect.h>
#include <Rtc.h>
#include <BatteryMonitor.h>

using freeink::ui::DisplayTarget;
using freeink::ui::Orientation;
using freeink::ui::Rect;
using freeink::ui::TextAlign;
using freeink::ui::TextStyle;

// ---- Config --------------------------------------------------------------
static constexpr uint32_t kUpdatePeriodSec = 60;   // redraw cadence
static constexpr int16_t  kTopMargin       = 8;     // px from the top edge
static constexpr int16_t  kEdgeInset       = 7;     // X4 Pro bezel.right overlap
static constexpr int16_t  kBattBoxW        = 220;   // fits "100%  4.15V" at 24 px

// ---- Persistent across light sleep (and a reset, via RTC memory) ---------
RTC_DATA_ATTR uint16_t g_lastBattPct = 0xFFFF;      // 0xFFFF = no good read yet

// ---- Hardware ----------------------------------------------------------
static EInkDisplay display(BoardConfig::ACTIVE.display.sclk, BoardConfig::ACTIVE.display.mosi,
                           BoardConfig::ACTIVE.display.cs, BoardConfig::ACTIVE.display.dc,
                           BoardConfig::ACTIVE.display.rst, BoardConfig::ACTIVE.display.busy);
static Rtc rtc;
static BatteryMonitor battery;

// The elapsed-time display counts from here; setup() re-sets the RTC to this on
// every boot, so "Nd HH:MM" is always "time since the last reset".
static const Rtc::DateTime kEpoch{2000, 1, 1, 0, 0, 0, 6};  // 2000-01-01 = Saturday

// ---- Helpers ---------------------------------------------------------

// Whole days between the RTC epoch and `dt`. mktime() on both endpoints in the
// same (implicit) timezone, so the offset cancels in the difference.
static long elapsedDays(const Rtc::DateTime& dt) {
  struct tm cur = {};
  cur.tm_year = dt.year - 1900;
  cur.tm_mon = dt.month - 1;
  cur.tm_mday = dt.day;
  cur.tm_hour = dt.hour;
  cur.tm_min = dt.minute;
  cur.tm_sec = dt.second;
  cur.tm_isdst = 0;

  struct tm epoch = {};
  epoch.tm_year = kEpoch.year - 1900;
  epoch.tm_mon = kEpoch.month - 1;
  epoch.tm_mday = kEpoch.day;
  epoch.tm_isdst = 0;

  const time_t a = mktime(&cur);
  const time_t b = mktime(&epoch);
  if (a == (time_t)-1 || b == (time_t)-1 || a < b) return 0;
  return static_cast<long>((a - b) / 86400);
}

// Reads gauge percent; on a transient I2C failure falls back to the last good
// value (or 0 if there has never been one). Returns whether this read succeeded.
static bool readBattery(uint16_t& outPct) {
  uint16_t p = 0;
  if (battery.readPercentageChecked(p)) {
    g_lastBattPct = p;
    outPct = p;
    return true;
  }
  outPct = (g_lastBattPct == 0xFFFF) ? 0 : g_lastBattPct;
  return false;
}

// Probe which panel controller this unit carries (SSD1677 / UC8179 / UC8279
// vary by production batch) and link the matching driver, then bring the panel
// up. Same sequence as the original Hello World.
static void bringUpPanel() {
  const bool promoted = freeink::applyXteinkDisplayController();
  const freeink::XteinkDisplayProbeDiag& diag = freeink::getXteinkDisplayProbeDiag();
  Serial.printf("Xteink display probe: promoted=%d verdict=%u ver=%02X %02X %02X %02X %02X flg=%02X\n",
                promoted, diag.verdict, diag.ver[0], diag.ver[1], diag.ver[2], diag.ver[3], diag.ver[4],
                diag.flg);
  display.begin();
}

static void renderStatus(const Rtc::DateTime& dt, bool haveTime, uint16_t pct, bool battOk, uint16_t mv) {
  // Portrait: 90 deg CW from panel-native, logical frame 480 wide x 800 tall,
  // "top" = short edge. If the panel reads upside-down on hardware, switch to
  // Orientation::PortraitInverted.
  DisplayTarget target(display.getFrameBuffer(), display.getDisplayWidth(), display.getDisplayHeight(),
                       display.getDisplayWidthBytes(), Orientation::Portrait);
  display.clearScreen(0xFF);  // white page (set bit = white)

  char clk[24];
  if (haveTime) {
    snprintf(clk, sizeof clk, "%ldd %02u:%02u", elapsedDays(dt), dt.hour, dt.minute);
  } else {
    snprintf(clk, sizeof clk, "--d --:--");
  }
  TextStyle clockStyle;
  clockStyle.align = TextAlign::Center;
  target.text(Rect{0, kTopMargin, target.logicalWidth(), target.lineHeight(clockStyle.font)}, clk,
              clockStyle);

  char bat[24];
  if (battOk) {
    snprintf(bat, sizeof bat, "%u%%  %u.%02uV", pct, mv / 1000u, (mv % 1000u) / 10u);
  } else {
    snprintf(bat, sizeof bat, "--%%  --.--V");
  }
  TextStyle battStyle;
  battStyle.align = TextAlign::Right;
  const int16_t battX = static_cast<int16_t>(target.logicalWidth() - kBattBoxW - kEdgeInset);
  target.text(Rect{battX, kTopMargin, kBattBoxW, target.lineHeight(battStyle.font)}, bat, battStyle);
}

// ---- Arduino entry points ------------------------------------------

void setup() {
  Serial.begin(115200);

  // GPIO1 master peripheral rail (panel + shared I2C bus). Held HIGH for the
  // whole session — Option D keeps the controller powered so FAST refresh never
  // has to rebuild a clean baseline. Deliberately NOT gpio_hold_en'd and NOT
  // cut before sleep.
  BoardConfig::holdPowerRails();

  const bool rtcBegun = rtc.begin();  // brings up Wire on SDA39/SCL38 @ 400k, addr 0x51
  Rtc::DateTime dt = kEpoch;
  // Every boot restarts the elapsed clock at 0d 00:00. Light-sleep wakes never
  // reach setup(), so the count only resets on a real power-on / reset.
  if (rtcBegun) rtc.set(kEpoch);
  Serial.printf("rtc: begun=%d reset to %04u-%02u-%02u %02u:%02u:%02u\n", rtcBegun, dt.year, dt.month,
                dt.day, dt.hour, dt.minute, dt.second);

  bringUpPanel();

  uint16_t pct = 0;
  const bool battOk = readBattery(pct);        // first read runs the one-time CW2017 BATINFO upload
  const uint16_t mv = battery.readMillivolts();
  Serial.printf("batt: ok=%d pct=%u mv=%u charging=%d\n", battOk, pct, mv, battery.isCharging());

  renderStatus(dt, rtcBegun, pct, battOk, mv);
  // One intentional flash at startup to establish a clean baseline for the
  // differential FAST refreshes that follow.
  display.displayBuffer(EInkDisplay::FULL_REFRESH, /*turnOffScreen=*/true);
  Serial.println("startup frame painted");
}

void loop() {
  Rtc::DateTime dt{};
  bool rtcOk = rtc.now(dt);

  // Sleep until the next wall-clock minute boundary.
  uint32_t toNext = rtcOk ? (kUpdatePeriodSec - (dt.second % kUpdatePeriodSec)) : kUpdatePeriodSec;
  if (toNext < 2) toNext += kUpdatePeriodSec;                 // never an immediate re-wake
  if (toNext > kUpdatePeriodSec) toNext = kUpdatePeriodSec;   // clamp against an I2C glitch

  // ESP32-S3 USB-Serial/JTAG and esp_light_sleep_start() do not coexist: light
  // sleep drops the USB link permanently (setup() never re-runs to re-init it).
  // While a host is attached, stay awake with delay() so serial logs and
  // reflashing keep working; on battery, light-sleep for the power saving this
  // firmware exists to measure.
  if (Serial) {
    delay(toNext * 1000UL);
  } else {
    esp_sleep_enable_timer_wakeup(static_cast<uint64_t>(toNext) * 1000000ULL);
    esp_light_sleep_start();  // RAM + panel controller retained; execution resumes here
  }

  rtcOk = rtc.now(dt);
  uint16_t pct = 0;
  const bool battOk = readBattery(pct);
  const uint16_t mv = battery.readMillivolts();

  renderStatus(dt, rtcOk, pct, battOk, mv);
  display.displayBuffer(EInkDisplay::FAST_REFRESH, /*turnOffScreen=*/true);

  Serial.printf("tick: %s %ldd %02u:%02u  batt=%u%% %umV  awake=%d\n", rtcOk ? "ok" : "RTC?",
                rtcOk ? elapsedDays(dt) : 0L, dt.hour, dt.minute, pct, mv, (bool)Serial);
}
