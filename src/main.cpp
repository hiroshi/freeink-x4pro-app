// Minimal FreeInk SDK "Hello World" for the Xteink X4 Pro.
//
// Draws one line of centered text and pushes a single full refresh. No input,
// battery, SD, or FreeInkApp scaffolding — just EInkDisplay + FreeInkUI's
// DisplayTarget::text().

#include <Arduino.h>
#include <BoardConfig.h>
#include <EInkDisplay.h>
#include <FreeInkUIDisplayTarget.h>
#include <XteinkDetect.h>

using freeink::ui::DisplayTarget;
using freeink::ui::Rect;
using freeink::ui::TextAlign;
using freeink::ui::TextStyle;

static EInkDisplay display(BoardConfig::ACTIVE.display.sclk, BoardConfig::ACTIVE.display.mosi,
                            BoardConfig::ACTIVE.display.cs, BoardConfig::ACTIVE.display.dc,
                            BoardConfig::ACTIVE.display.rst, BoardConfig::ACTIVE.display.busy);

void setup() {
  Serial.begin(115200);

  // Asserts the board's power-latch pins (X4 Pro: GPIO1, required for the
  // touch rail). Harmless/no-op for a display-only sketch, but matches the
  // SDK's standard bring-up sequence.
  BoardConfig::holdPowerRails();

  // X4 Pro production batches ship one of three panel controllers (SSD1677,
  // UC8179, or UC8279) on identical glass/pinout. Probe before begin() so the
  // facade links the matching driver instead of assuming the profile default
  // (SSD1677) — a UC81xx panel getting SSD1677 commands stays blank.
  const bool promoted = freeink::applyXteinkDisplayController();
  const freeink::XteinkDisplayProbeDiag& diag = freeink::getXteinkDisplayProbeDiag();
  Serial.printf("Xteink display probe: promoted=%d verdict=%u ver=%02X %02X %02X %02X %02X flg=%02X\n", promoted,
                diag.verdict, diag.ver[0], diag.ver[1], diag.ver[2], diag.ver[3], diag.ver[4], diag.flg);

  display.begin();
  display.clearScreen();

  DisplayTarget target(display.getFrameBuffer(), display.getDisplayWidth(), display.getDisplayHeight(),
                        display.getDisplayWidthBytes());

  TextStyle style;
  style.align = TextAlign::Center;

  const Rect fullScreen{0, 0, target.logicalWidth(), target.logicalHeight()};
  target.text(fullScreen, "Hello, World!", style);

  display.displayBuffer(EInkDisplay::FULL_REFRESH);

  Serial.println("Hello World painted.");
}

void loop() {
  // Nothing to do — this is a one-shot screen.
}
