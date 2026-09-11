// FreeInk SDK — elapsed-time + battery status screen for the Xteink X4 Pro.
//
// Top-left   : "USB AWAKE" / "BAT SLEEP" — power + link state (see below).
// Top-center : "Nd HH:MM"  — time since the last boot (setup() resets the RTC
//              to 2000-01-01 00:00:00 on every power-on / reset; light-sleep
//              wakes resume in loop() and keep counting).
// Top-right  : "87%  3.92V" — CW2017 fuel-gauge percent + battery millivolts.
//
// Portrait layout (480 wide x 800 tall).
//
// Redraws once a minute. Between updates the ESP32 light-sleeps ONLY on battery:
// the panel controller stays powered (GPIO1 rail held), so its RAM survives as
// the differential baseline and each minute's repaint is a FAST refresh with NO
// black/white flash. One FULL refresh runs at startup; there is no periodic
// scrub (drop in a HALF refresh here later if ghosting proves to accumulate).
//
// Reflash friendliness (firmware can only be flashed while the ESP32 is awake —
// light sleep drops the USB-Serial/JTAG link and setup() never re-runs to
// re-init it):
//   * Every boot holds awake for kBootAwakeGraceSec so a reflash always has a
//     window without needing the RESET-mash trick.
//   * While a USB host is attached (`Serial` truthy) or the charger is running
//     (charger /STAT on GPIO21 HIGH) the loop stays awake with delay().
//   * On battery it light-sleeps, but arms a wake on GPIO21 going HIGH, so
//     plugging in USB pulls it back out of sleep; that wake re-opens the awake
//     window (kUsbWakeAwakeSec) and bounces the CDC link.
//   * While awake the minute wait is interruptible, so unplug/replug repaints
//     the "USB/BAT AWAKE/SLEEP" field within ~0.2 s instead of at the next tick.
//   (esp_sleep_enable_usb_serial_jtag_wakeup() would be the direct signal but
//   it is not in the pioarduino prebuilt libs — the charge-status GPIO is the
//   available proxy for "USB just connected".)
//
// Option D per the plan: no deep sleep, no rail cut — trades higher average
// sleep current for a flicker-free clock. The point is to measure that current.
//
// Reader feature: pressing Power fetches plain text from kFetchUrl over WiFi
// and shows it full-screen, word-wrapped and paginated; Left/Right (the two
// physical nav keys) turn pages. Entering reader mode suspends the plain clock
// screen — it never comes back on its own, since the e-ink page holds at zero
// power regardless of what the ESP32 is doing — but the SAME status header
// (USB/battery/clock, drawStatusHeaderNow()) keeps showing at the top, and
// once idle a little while the board light-sleeps between input polls just
// like the clock screen does: waking on any nav/power button, a charger
// connect (kLeftButtonGpio/kRightButtonGpio/kPowerButtonGpio/kChargeStatGpio),
// or once a minute on a timer purely to refresh that header in place (page
// position untouched) so it doesn't read stale forever between button presses.
// See
// enterReaderModeAndFetch() / dispatchReaderInput() / g_readerMode below.

#include <Arduino.h>
#include <time.h>
#include <string>

#include <driver/gpio.h>
#include <esp_sleep.h>

#include <WiFi.h>

#include <BoardConfig.h>
#include <EInkDisplay.h>
#include <FreeInkUIDisplayTarget.h>
#include <InputManager.h>
#include <SecureHttpClient.h>
#include <XteinkDetect.h>
#include <Rtc.h>
#include <BatteryMonitor.h>

#if defined(FREEINK_NET_WOLFSSL)
// Pulls in the Arduino-wolfSSL port's own wolfssl/wolfcrypt/settings.h +
// wolfssl/ssl.h, plus (non-inline, so include this exactly once across the
// project) its definition of wolfSSL_Arduino_Serial_Print — the debug-log
// sink wolfSSL's logging.c links against by symbol name whenever DEBUG_WOLFSSL
// ends up enabled. SecureClient.cpp includes <wolfssl/ssl.h> directly (not
// this wrapper), so without this include here the symbol is never emitted and
// the link fails.
#include <wolfssl.h>
#endif

using freeink::ui::DisplayTarget;
using freeink::ui::Orientation;
using freeink::ui::Rect;
using freeink::ui::TextAlign;
using freeink::ui::TextAreaLine;
using freeink::ui::TextStyle;
using freeink::ui::textAreaVisibleLines;
using freeink::ui::textAreaWalk;

// ---- Config --------------------------------------------------------------
static constexpr uint32_t kUpdatePeriodSec   = 60;   // redraw cadence
static constexpr uint32_t kBootAwakeGraceSec  = 20;  // stay awake after every boot so a USB reflash always has a window
static constexpr uint32_t kUsbWakeAwakeSec    = 30;  // re-open that window whenever USB is (re)connected mid-run
static constexpr int16_t  kTopMargin       = 8;     // px from the top edge
static constexpr int16_t  kEdgeInset       = 7;     // X4 Pro bezel.right overlap
static constexpr int16_t  kBattBoxW        = 220;   // fits "100%  4.15V" at 24 px
static constexpr int16_t  kStatBoxW        = 240;   // fits "USB AWAKE" at 24 px
// X4 Pro charger /STAT (BoardConfig batteryChargeStatus), driven HIGH while
// charging — used both to display "USB attached" and to wake from light sleep.
static constexpr gpio_num_t kChargeStatGpio = GPIO_NUM_21;

// ---- Reader feature config (fetch-on-button) ------------------------------
// WiFi credentials and the fetch URL come from build-time -D defines (see
// platformio.ini's build_flags, fed from either real shell env vars or a
// gitignored .env file — see .env.example) rather than source, since this repo
// is public. Empty when unset; enterReaderModeAndFetch() refuses to run and
// shows a "not configured" screen instead of connecting with an empty SSID.
#ifndef WIFI_SSID
#define WIFI_SSID ""
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD ""
#endif
#ifndef FETCH_URL
#define FETCH_URL ""
#endif
static constexpr char kWifiSsid[]     = WIFI_SSID;
static constexpr char kWifiPassword[] = WIFI_PASSWORD;
// http:// or https:// — TLS 1.3 via wolfSSL is on (see platformio.ini);
// fetchArticleText() below skips certificate verification (setInsecure()).
static constexpr char kFetchUrl[]     = FETCH_URL;
static constexpr uint32_t kWifiConnectTimeoutMs = 15000;
static constexpr uint32_t kFetchTimeoutMs       = 15000;
// Reader mode never falls back to the clock screen on its own — e-ink holds
// whatever was last drawn at zero power, so there is nothing to gain (and a
// mid-read page turning into the clock header is exactly the bug this avoids)
// from redrawing over it just because the reader has been idle a while.
// Instead, after this short a grace period since the last button press (with
// no USB/charger attached) loop() light-sleeps BETWEEN polls, armed to wake on
// any nav/power button or a charger connect — see the g_readerMode branch.
// Kept short so a rapid run of page turns stays on the cheap busy-poll path
// instead of paying a sleep/wake round trip between each one.
static constexpr uint32_t kReaderSleepGraceMs   = 3000;
static constexpr int16_t  kReaderBodyGap        = 8;  // px between the page-indicator row and body text

// Power button (BoardConfig XTEINK_X4_PRO input.power = GPIO3, active-LOW) —
// also armed as a light-sleep wakeup source alongside kChargeStatGpio so a
// press lands even while the board is asleep on battery.
static constexpr gpio_num_t kPowerButtonGpio = static_cast<gpio_num_t>(InputManager::POWER_BUTTON_PIN);
// Left/Right nav keys (BoardConfig XTEINK_X4_PRO input.up=GPIO0/input.down=GPIO7,
// active-LOW) — armed as light-sleep wakeup sources too so paging still works
// after reader mode's own idle light sleep kicks in (kReaderSleepGraceMs).
static constexpr gpio_num_t kLeftButtonGpio  = GPIO_NUM_0;
static constexpr gpio_num_t kRightButtonGpio = GPIO_NUM_7;

// ---- Persistent across light sleep (and a reset, via RTC memory) ---------
RTC_DATA_ATTR uint16_t g_lastBattPct = 0xFFFF;      // 0xFFFF = no good read yet

// millis() deadline until which loop() refuses to light-sleep. Seeded for the
// boot grace window in setup(); bumped again on every USB-connect wake.
static uint32_t g_stayAwakeUntilMs = 0;

// ---- Hardware ----------------------------------------------------------
static EInkDisplay display(BoardConfig::ACTIVE.display.sclk, BoardConfig::ACTIVE.display.mosi,
                           BoardConfig::ACTIVE.display.cs, BoardConfig::ACTIVE.display.dc,
                           BoardConfig::ACTIVE.display.rst, BoardConfig::ACTIVE.display.busy);
static Rtc rtc;
static BatteryMonitor battery;
static InputManager input;

// ---- Reader state ---------------------------------------------------------
static bool g_readerMode = false;             // true while showing fetched text instead of the clock
static uint32_t g_readerLastActivityMs = 0;   // last button press in reader mode, for the idle timeout
static std::string g_articleText;             // last-fetched body
static uint32_t g_readerTopLine = 0;          // first visual line shown on the current page
static uint32_t g_readerLineCount = 0;        // total word-wrapped visual lines in g_articleText
// Radio only comes up for the connect+fetch attempt itself: true from just
// before WiFi.begin() until the fetch finishes (success or failure), then
// wifiPowerOff() clears it before light sleep is allowed again. Drives the
// "WiFi ON/OFF" header badge on every reader screen.
static bool g_wifiOn = false;

// The elapsed-time display counts from here; setup() re-sets the RTC to this on
// every boot, so "Nd HH:MM" is always "time since the last reset".
static const Rtc::DateTime kEpoch{2000, 1, 1, 0, 0, 0, 6};  // 2000-01-01 = Saturday

// ---- Helpers ---------------------------------------------------------

// True while the no-light-sleep deadline is still in the future. Signed diff so
// it stays correct across the millis() wrap.
static bool inStayAwakeWindow() {
  return static_cast<int32_t>(g_stayAwakeUntilMs - millis()) > 0;
}

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

// The standard top-of-screen status row — link/power state (left), elapsed
// clock (center), battery (right) — shared by the clock screen and every
// reader screen so it reads as one consistent header throughout the app.
static void drawStatusHeader(DisplayTarget& target, const Rtc::DateTime& dt, bool haveTime, uint16_t pct,
                             bool battOk, uint16_t mv, bool awake, bool usb) {
  // Top-left: link + power state. "USB"/"BAT" = host attached or charger running
  // vs. on battery; "AWAKE"/"SLEEP" = whether the next gap light-sleeps (i.e.
  // whether a reflash can land right now).
  char st[24];
  snprintf(st, sizeof st, "%s %s", usb ? "USB" : "BAT", awake ? "AWAKE" : "SLEEP");
  TextStyle statStyle;
  statStyle.align = TextAlign::Left;
  target.text(Rect{kEdgeInset, kTopMargin, kStatBoxW, target.lineHeight(statStyle.font)}, st, statStyle);

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

static void renderStatus(const Rtc::DateTime& dt, bool haveTime, uint16_t pct, bool battOk, uint16_t mv,
                         bool awake, bool usb) {
  // Portrait: 90 deg CW from panel-native, logical frame 480 wide x 800 tall,
  // "top" = short edge. If the panel reads upside-down on hardware, switch to
  // Orientation::PortraitInverted.
  DisplayTarget target(display.getFrameBuffer(), display.getDisplayWidth(), display.getDisplayHeight(),
                       display.getDisplayWidthBytes(), Orientation::Portrait);
  display.clearScreen(0xFF);  // white page (set bit = white)
  drawStatusHeader(target, dt, haveTime, pct, battOk, mv, awake, usb);
}

// Reads the same status quantities loop() gathers for the clock tick and
// draws the standard header with them — the reader screens' equivalent of
// renderStatus(), so they show the same USB/battery/clock header instead of
// going without one.
static void drawStatusHeaderNow(DisplayTarget& target) {
  Rtc::DateTime dt{};
  const bool rtcOk = rtc.now(dt);
  uint16_t pct = 0;
  const bool battOk = readBattery(pct);
  const uint16_t mv = battery.readMillivolts();
  const bool usbNow = (bool)Serial || battery.isCharging();
  const bool awakeNow = usbNow || inStayAwakeWindow();
  drawStatusHeader(target, dt, rtcOk, pct, battOk, mv, awakeNow, usbNow);
}

// Wait up to `ms` while staying awake, but return early the moment the USB /
// stay-awake state changes (so the header can be repainted on unplug/replug
// instead of at the next minute tick) or the Power button is pressed (so the
// reader can launch immediately). Polls at 50 ms — still well under the
// e-ink FAST refresh time, tight enough for snappy button response.
// Returns true iff it returned because of a Power-button press.
static bool waitWhileAwake(uint32_t ms) {
  const bool usbEntry = (bool)Serial || battery.isCharging();
  const uint32_t deadline = millis() + ms;
  while (static_cast<int32_t>(deadline - millis()) > 0) {
    input.update();
    if (input.wasPressed(InputManager::BTN_POWER)) return true;
    if (((bool)Serial || battery.isCharging()) != usbEntry) return false;  // plugged / unplugged
    if (!(bool)Serial && !battery.isCharging() && !inStayAwakeWindow()) return false;  // window lapsed → let loop() sleep
    delay(50);
  }
  return false;
}

// ---- Reader (fetch-on-button) ---------------------------------------------

// y of the reader sub-header row (WiFi badge + page indicator): directly
// below the standard status header (drawStatusHeaderNow, same row every
// screen uses — USB/battery/clock), so the reader-specific info never
// overlaps or replaces it.
static int16_t readerSubHeaderRowY(int16_t lineHeight) {
  return static_cast<int16_t>(kTopMargin + lineHeight);
}

// Body text area: the status header, the WiFi+page sub-header row below it,
// then the wrapped article text filling the rest of the 480x800 portrait screen.
static Rect readerBodyRect(const DisplayTarget& target, int16_t lineHeight) {
  const int16_t y = static_cast<int16_t>(readerSubHeaderRowY(lineHeight) + lineHeight + kReaderBodyGap);
  return Rect{kEdgeInset, y, static_cast<int16_t>(target.logicalWidth() - 2 * kEdgeInset),
             static_cast<int16_t>(target.logicalHeight() - y - kEdgeInset)};
}

static bool connectWifiIfNeeded() {
  if (WiFi.status() == WL_CONNECTED) return true;
  WiFi.mode(WIFI_STA);
  WiFi.begin(kWifiSsid, kWifiPassword);
  const uint32_t deadline = millis() + kWifiConnectTimeoutMs;
  while (WiFi.status() != WL_CONNECTED && static_cast<int32_t>(deadline - millis()) > 0) {
    delay(200);
  }
  return WiFi.status() == WL_CONNECTED;
}

// Fully powers the radio down (not just disconnects the association) so the
// board draws idle current again before it's allowed back into light sleep.
// Called right after the fetch attempt finishes, success or failure.
static void wifiPowerOff() {
  WiFi.disconnect(/*wifioff=*/true);
  WiFi.mode(WIFI_OFF);
  g_wifiOn = false;
}

// Small top-left "WiFi ON/OFF" badge, drawn on the reader sub-header row
// (readerSubHeaderRowY) — one line below the standard status header, so it
// never overlaps either that header or the centered "PAGE X/Y" indicator
// sharing its row. Reflects g_wifiOn, not a live WiFi.status() poll, since
// the point is to show the ON-only-while-fetching intent rather than radio
// minutiae.
static void drawWifiBadge(DisplayTarget& target) {
  TextStyle style;
  style.align = TextAlign::Left;
  const int16_t lh = target.lineHeight(style.font);
  target.text(Rect{kEdgeInset, readerSubHeaderRowY(lh), kStatBoxW, lh}, g_wifiOn ? "WiFi ON" : "WiFi OFF", style);
}

// GET kFetchUrl and store the response body. Returns the HTTP status (or a
// negative freeink::SecureHttpClient transport error).
static int fetchArticleText(std::string& outBody) {
  freeink::SecureHttpClient http;
  http.setTimeout(kFetchTimeoutMs);
  http.setUserAgent("FreeInk-X4Pro/1.0");
  // No CA bundle is wired up on-device (see SecureHttpClient.h), so https://
  // connects but doesn't verify the server's certificate.
  http.setInsecure();
  if (!http.begin(kFetchUrl)) return -1;
  const int status = http.GET();
  if (status == 200) outBody = http.getString();
  return status;
}

// One centered full-screen line — used for the "Connecting..." / error states
// that bracket a fetch.
static void renderReaderMessage(const char* msg) {
  DisplayTarget target(display.getFrameBuffer(), display.getDisplayWidth(), display.getDisplayHeight(),
                       display.getDisplayWidthBytes(), Orientation::Portrait);
  display.clearScreen(0xFF);
  drawStatusHeaderNow(target);
  drawWifiBadge(target);
  TextStyle style;
  style.align = TextAlign::Center;
  const int16_t lh = target.lineHeight(style.font);
  target.text(Rect{0, static_cast<int16_t>(target.logicalHeight() / 2 - lh / 2), target.logicalWidth(), lh}, msg,
             style);
  display.displayBuffer(EInkDisplay::FAST_REFRESH, /*turnOffScreen=*/true);
}

// Draws the page-indicator row + the visual lines of g_articleText starting at
// g_readerTopLine.
static void renderReaderPage() {
  DisplayTarget target(display.getFrameBuffer(), display.getDisplayWidth(), display.getDisplayHeight(),
                       display.getDisplayWidthBytes(), Orientation::Portrait);
  display.clearScreen(0xFF);
  drawStatusHeaderNow(target);
  drawWifiBadge(target);

  TextStyle bodyStyle;
  bodyStyle.align = TextAlign::Left;
  const int16_t lh = target.lineHeight(bodyStyle.font);
  const Rect bodyRect = readerBodyRect(target, lh);
  const uint16_t visible = textAreaVisibleLines(bodyRect, lh);

  char pageIndicator[32];
  if (g_readerLineCount == 0 || visible == 0) {
    snprintf(pageIndicator, sizeof pageIndicator, "(empty)");
  } else {
    const uint32_t lastPage = (g_readerLineCount - 1) / visible;
    const uint32_t curPage = g_readerTopLine / visible;
    snprintf(pageIndicator, sizeof pageIndicator, "PAGE %lu/%lu", static_cast<unsigned long>(curPage + 1),
             static_cast<unsigned long>(lastPage + 1));
  }
  TextStyle pageStyle;
  pageStyle.align = TextAlign::Center;
  target.text(Rect{0, readerSubHeaderRowY(lh), target.logicalWidth(), lh}, pageIndicator, pageStyle);

  if (visible > 0) {
    textAreaWalk(target, bodyRect.width, g_articleText.c_str(), bodyStyle,
                [&](uint32_t idx, const TextAreaLine& ln) {
                  if (idx < g_readerTopLine || idx >= g_readerTopLine + visible) return;
                  char buf[224];
                  const uint16_t n = ln.len < 220 ? ln.len : 220;
                  memcpy(buf, g_articleText.c_str() + ln.start, n);
                  buf[n] = '\0';
                  const int16_t y = static_cast<int16_t>(bodyRect.y + (idx - g_readerTopLine) * lh);
                  target.text(Rect{bodyRect.x, y, bodyRect.width, lh}, buf, bodyStyle);
                });
  }

  display.displayBuffer(EInkDisplay::FAST_REFRESH, /*turnOffScreen=*/true);
}

// Left/Right page turn. direction<0 = previous page, >0 = next page. No-op
// (redraws the same page) past either end.
static void readerTurnPage(int direction) {
  DisplayTarget target(display.getFrameBuffer(), display.getDisplayWidth(), display.getDisplayHeight(),
                       display.getDisplayWidthBytes(), Orientation::Portrait);
  TextStyle bodyStyle;
  const int16_t lh = target.lineHeight(bodyStyle.font);
  const Rect bodyRect = readerBodyRect(target, lh);
  const uint16_t visible = textAreaVisibleLines(bodyRect, lh);
  if (visible == 0) return;

  if (direction < 0) {
    g_readerTopLine = g_readerTopLine >= visible ? g_readerTopLine - visible : 0;
  } else if (direction > 0 && g_readerLineCount > visible) {
    const uint32_t maxTop = ((g_readerLineCount - 1) / visible) * visible;
    g_readerTopLine = g_readerTopLine + visible <= maxTop ? g_readerTopLine + visible : maxTop;
  }
  renderReaderPage();
}

// Power-button entry point: connect WiFi if needed, GET kFetchUrl, and show
// page 1. Leaves g_readerMode set so loop() switches to polling
// Left/Right/Power instead of the clock tick.
//
// A failure (not configured / WiFi / HTTP) falls back to whatever article is
// already loaded — g_readerLineCount/g_articleText are left untouched by every
// failure path below (fetchArticleText only writes g_articleText on a 200) —
// and just re-renders that page instead of clobbering it with an error
// screen. An accidental Power press while reading shouldn't blank the page.
// Only a genuinely fresh attempt (nothing loaded yet) shows the error text.
static void enterReaderModeAndFetch() {
  g_readerMode = true;
  g_readerLastActivityMs = millis();
  const bool hadArticle = g_readerLineCount > 0;

  if (kWifiSsid[0] == '\0' || kFetchUrl[0] == '\0') {
    Serial.println("reader: not configured — set WIFI_SSID/WIFI_PASSWORD/FETCH_URL (see .env.example)");
    if (hadArticle) {
      renderReaderPage();
    } else {
      renderReaderMessage("Not configured (see .env.example)");
    }
    return;
  }

  // WiFi comes on only for this connect+fetch attempt; wifiPowerOff() below
  // always runs before the next render, whichever way this ends.
  g_wifiOn = true;
  renderReaderMessage("Connecting WiFi...");
  if (!connectWifiIfNeeded()) {
    wifiPowerOff();
    Serial.println("reader: WiFi connect failed");
    if (hadArticle) {
      renderReaderPage();
    } else {
      renderReaderMessage("WiFi connect failed");
    }
    return;
  }

  renderReaderMessage("Fetching...");
  const int status = fetchArticleText(g_articleText);
  wifiPowerOff();
  if (status != 200) {
    Serial.printf("reader: fetch failed (HTTP %d)\n", status);
    if (hadArticle) {
      renderReaderPage();
    } else {
      char msg[32];
      snprintf(msg, sizeof msg, "Fetch failed (HTTP %d)", status);
      renderReaderMessage(msg);
    }
    return;
  }

  // Success: recompute pagination for the new article and show page 1.
  g_readerTopLine = 0;
  g_readerLineCount = 0;
  DisplayTarget target(display.getFrameBuffer(), display.getDisplayWidth(), display.getDisplayHeight(),
                       display.getDisplayWidthBytes(), Orientation::Portrait);
  TextStyle bodyStyle;
  const int16_t lh = target.lineHeight(bodyStyle.font);
  const Rect bodyRect = readerBodyRect(target, lh);
  textAreaWalk(target, bodyRect.width, g_articleText.c_str(), bodyStyle,
              [&](uint32_t, const TextAreaLine&) { ++g_readerLineCount; });
  Serial.printf("reader: fetched %u bytes -> %lu wrapped lines (bodyRect %d,%d %dx%d, lineHeight=%d)\n",
                (unsigned)g_articleText.size(), (unsigned long)g_readerLineCount, bodyRect.x, bodyRect.y,
                bodyRect.width, bodyRect.height, lh);

  renderReaderPage();
}

// Acts on a button-press edge already latched by the most recent input.update()
// — Left/Right page, Power re-fetch. Shared by the normal per-tick poll in
// loop() and the wake-from-idle-sleep path below, so a press caught by either
// is handled identically. Returns whether it found (and handled) one.
static bool dispatchReaderInput() {
  if (input.wasPressed(InputManager::BTN_UP)) {  // Left = previous page
    readerTurnPage(-1);
    g_readerLastActivityMs = millis();
    return true;
  }
  if (input.wasPressed(InputManager::BTN_DOWN)) {  // Right = next page
    readerTurnPage(+1);
    g_readerLastActivityMs = millis();
    return true;
  }
  if (input.wasPressed(InputManager::BTN_POWER)) {  // re-fetch
    enterReaderModeAndFetch();
    return true;
  }
  return false;
}

// ---- Arduino entry points ------------------------------------------

void setup() {
  Serial.begin(115200);

  // Hold awake for a fixed window after every boot: light sleep kills the
  // USB-Serial/JTAG link for good, so this is the guaranteed slot to catch the
  // board with `pio run -t upload` without the RESET-mash workaround.
  g_stayAwakeUntilMs = millis() + kBootAwakeGraceSec * 1000UL;

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
  input.begin();

  uint16_t pct = 0;
  const bool battOk = readBattery(pct);        // first read runs the one-time CW2017 BATINFO upload
  const uint16_t mv = battery.readMillivolts();
  const bool charging = battery.isCharging();
  Serial.printf("batt: ok=%d pct=%u mv=%u charging=%d\n", battOk, pct, mv, charging);

  renderStatus(dt, rtcBegun, pct, battOk, mv, /*awake=*/true, /*usb=*/(bool)Serial || charging);
  // One intentional flash at startup to establish a clean baseline for the
  // differential FAST refreshes that follow.
  display.displayBuffer(EInkDisplay::FULL_REFRESH, /*turnOffScreen=*/true);
  Serial.println("startup frame painted");
}

void loop() {
  input.update();

  // Reader mode owns the buttons while active: Left/Right page, Power
  // re-fetches. It never falls back to the clock screen on its own — the
  // e-ink page stays on screen at zero power regardless of what the ESP32 is
  // doing, so there is nothing to gain from redrawing over it. Once idle a
  // little while (and not on USB/charging) it light-sleeps between polls
  // instead of busy-waiting, waking on any nav/power button or a charger
  // connect, so battery is still conserved without ever touching the display.
  if (g_readerMode) {
    if (!dispatchReaderInput()) {
      const bool serialUp = (bool)Serial;
      const bool charging = battery.isCharging();
      if (!serialUp && !charging && !inStayAwakeWindow() &&
          millis() - g_readerLastActivityMs > kReaderSleepGraceMs) {
        // Also wake on a timer so the header (clock/battery/AWAKE-SLEEP) keeps
        // ticking once a minute like the plain clock screen does, even with no
        // button press — otherwise it reads whatever it was at the last page
        // turn forever. Page position is untouched; only the header changes.
        esp_sleep_enable_timer_wakeup(static_cast<uint64_t>(kUpdatePeriodSec) * 1000000ULL);
        gpio_wakeup_enable(kLeftButtonGpio, GPIO_INTR_LOW_LEVEL);
        gpio_wakeup_enable(kRightButtonGpio, GPIO_INTR_LOW_LEVEL);
        gpio_wakeup_enable(kPowerButtonGpio, GPIO_INTR_LOW_LEVEL);
        gpio_wakeup_enable(kChargeStatGpio, GPIO_INTR_HIGH_LEVEL);
        esp_sleep_enable_gpio_wakeup();
        esp_light_sleep_start();  // RAM + panel controller retained; execution resumes here

        if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_GPIO) {
          // Light sleep always drops the USB-Serial/JTAG link; bounce it so a
          // charger/USB (re)connect or a fresh `pio run -t upload` both work.
          Serial.end();
          delay(20);
          Serial.begin(115200);
          g_stayAwakeUntilMs = millis() + kUsbWakeAwakeSec * 1000UL;
          // Confirm with a few debounced samples (level wake fires the
          // instant a pin reads its wake level; wasPressed() needs a settled
          // edge to commit) and act on it now rather than waiting for next
          // loop()'s update() to (maybe) miss the edge.
          for (int i = 0; i < 5; ++i) {
            input.update();
            delay(10);
          }
          dispatchReaderInput();
        } else {
          // Timer wake, no button involved: just refresh the header in place
          // (FAST_REFRESH only touches pixels that actually changed, so this
          // costs nothing extra for the unchanged body text below it).
          renderReaderPage();
        }
      } else {
        delay(30);
      }
    }
    return;
  }

  if (input.wasPressed(InputManager::BTN_POWER)) {
    enterReaderModeAndFetch();
    return;
  }

  Rtc::DateTime dt{};
  bool rtcOk = rtc.now(dt);

  // Sleep until the next wall-clock minute boundary.
  uint32_t toNext = rtcOk ? (kUpdatePeriodSec - (dt.second % kUpdatePeriodSec)) : kUpdatePeriodSec;
  if (toNext < 2) toNext += kUpdatePeriodSec;                 // never an immediate re-wake
  if (toNext > kUpdatePeriodSec) toNext = kUpdatePeriodSec;   // clamp against an I2C glitch

  // Stay awake while a USB host is attached, while the charger is running, or
  // during a boot / USB-connect grace window — so serial logs and reflashing
  // keep working. Only light-sleep on battery, and even then arm a wake on the
  // charger /STAT line so plugging in USB pulls us back out.
  const bool serialUp = (bool)Serial;
  const bool charging = battery.isCharging();
  const bool stayAwake = serialUp || charging || inStayAwakeWindow();

  if (stayAwake) {
    if (waitWhileAwake(toNext * 1000UL)) {
      enterReaderModeAndFetch();
      return;
    }
  } else {
    esp_sleep_enable_timer_wakeup(static_cast<uint64_t>(toNext) * 1000000ULL);
    gpio_wakeup_enable(kChargeStatGpio, GPIO_INTR_HIGH_LEVEL);
    gpio_wakeup_enable(kPowerButtonGpio, GPIO_INTR_LOW_LEVEL);  // Power press, active-low
    esp_sleep_enable_gpio_wakeup();
    esp_light_sleep_start();  // RAM + panel controller retained; execution resumes here

    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_GPIO) {
      // Charger /STAT went HIGH (USB reconnected) and/or the Power button is
      // down — level-triggered wake doesn't tell us which, so check both.
      // Bounce the CDC link (setup() won't re-run to do it) and hold awake so
      // a reflash can land either way.
      Serial.end();
      delay(20);
      Serial.begin(115200);
      g_stayAwakeUntilMs = millis() + kUsbWakeAwakeSec * 1000UL;

      if (digitalRead(kPowerButtonGpio) == LOW) {
        // Confirm with a few debounced samples (level wake fires the instant
        // the pin reads low; wasPressed() needs a settled edge to commit).
        for (int i = 0; i < 5; ++i) {
          input.update();
          delay(10);
        }
        if (input.isPressed(InputManager::BTN_POWER)) {
          enterReaderModeAndFetch();
          return;
        }
      }
    }
  }

  rtcOk = rtc.now(dt);
  uint16_t pct = 0;
  const bool battOk = readBattery(pct);
  const uint16_t mv = battery.readMillivolts();
  const bool usbNow = (bool)Serial || battery.isCharging();
  const bool awakeNow = usbNow || inStayAwakeWindow();

  renderStatus(dt, rtcOk, pct, battOk, mv, awakeNow, usbNow);
  display.displayBuffer(EInkDisplay::FAST_REFRESH, /*turnOffScreen=*/true);

  Serial.printf("tick: %s %ldd %02u:%02u  batt=%u%% %umV chg=%d  awake=%d usb=%d\n", rtcOk ? "ok" : "RTC?",
                rtcOk ? elapsedDays(dt) : 0L, dt.hour, dt.minute, pct, mv, (int)battery.isCharging(),
                (int)awakeNow, (int)usbNow);
}
