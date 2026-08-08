#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFiManager.h>
#include <time.h>

#include "LGFX.h"
#include "WiFiManagerHelpers.h"
#include "ConfigurationWebServer.h"
#include "HttpRequestManager.h"
#include "OpenSkyAuthTokenHandler.h"
#include "AircraftManager.h"
#include "DrawHelpers.h"
#include "models/Aircraft.h"
#include "models/TrackedAircraft.h"

constexpr int SCREEN_SIZE = 240;
constexpr int SCREEN_SIZE_DIV_2 = (SCREEN_SIZE / 2);

LGFX tft;
LGFX_Sprite backbuffer(&tft);

WiFiManager wm;
ConfigurationWebServer configServer;
HttpRequestManager http;
OpenSkyAuthTokenHandler authHandler(http);

AircraftManager aircraftManager(configServer, authHandler, http, tft);

void setup()
{
  Serial.begin(115200);
  // delay(1000); // avoids immediate serial output being cut off - uncomment if needed

  // initialise LGFX + screen
  tft.init();
  tft.invertDisplay(true);
  pinMode(3, OUTPUT);
  digitalWrite(3, HIGH);

  backbuffer.setColorDepth(8);
  backbuffer.createSprite(SCREEN_SIZE, SCREEN_SIZE);

  // establish WiFi connection
  tft.fillScreen(lgfx::color888(0, 0, 0));
  tft.setTextColor(lgfx::color888(0, 255, 0));
  tft.drawCentreString("Connecting to WiFi...", SCREEN_SIZE / 2, SCREEN_SIZE / 2);

  WiFiManagerHelpers::ConfigureWiFiManager(wm, tft);
  wm.autoConnect(WiFiManagerHelpers::WiFiManagerName);

  // show allocated IP address briefly
  tft.fillScreen(lgfx::color888(0, 0, 0));
  tft.setTextColor(lgfx::color888(0, 255, 0));
  tft.drawCentreString(WiFi.localIP().toString(), SCREEN_SIZE / 2, SCREEN_SIZE / 2);
  delay(3000);

  // sync time via NTP — estimate timezone from longitude
  double cfgLon = configServer.GetStoredString("longitude").toDouble();
  int utcOffset = (int)round(cfgLon / 15.0) * 3600;
  String tz = "UTC" + String(utcOffset >= 0 ? "-" : "+") + String(abs(utcOffset) / 3600);
  configTzTime(tz.c_str(), "pool.ntp.org", "time.google.com");

  // begin background server for configuration
  configServer.statsProvider = [](const String& var) -> String {
    const auto& s = aircraftManager.GetStats();
    if (var == "STAT_TOTAL") return String(s.totalPlanesTracked);
    if (var == "STAT_PEAK") return String(s.peakPlanes);
    if (var == "STAT_PEAK_TIME") {
        if (s.peakTime == 0) return "n/a";
        unsigned long elapsed = millis() - s.peakTime;
        time_t peakEpoch = time(nullptr) - (elapsed / 1000);
        struct tm t;
        localtime_r(&peakEpoch, &t);
        char buf[6];
        snprintf(buf, sizeof(buf), "%02d:%02d", t.tm_hour, t.tm_min);
        return String(buf);
    }
    if (var == "STAT_MILITARY") return String(s.militaryCount);
    if (var == "STAT_LAST_MIL") return s.lastMilitaryCallsign.length() > 0 ? ("(last: " + s.lastMilitaryCallsign + ")") : "";
    if (var == "STAT_EMERGENCY") return String(s.emergencyCount);
    if (var == "STAT_LAST_EMER") return s.lastEmergencyCallsign.length() > 0 ? ("(last: " + s.lastEmergencyCallsign + " sqwk " + s.lastEmergencySquawk + ")") : "";
    if (var == "STAT_CLOSEST") {
        if (s.closestProximity > 900000.0f) return "none yet";
        return s.closestPair1 + " / " + s.closestPair2 + " (" + String((int)s.closestProximity) + "m)";
    }
    return "";
  };
  configServer.Initialise();

  // initialise aircraft manager
  aircraftManager.Initialise();
}

void loop()
{
  // reboot if heap is critically low (prevents silent crashes from fragmentation)
  if (ESP.getFreeHeap() < 20000) {
    ESP.restart();
  }

  aircraftManager.Update();

  // draw cycle
  backbuffer.fillScreen(lgfx::color888(0, 0, 0));

  float sweepAngle = fmod(millis() / 3000.0f, 2.0f * PI);

  String renderScanlines = configServer.GetStoredString("scanline");
  if (renderScanlines.isEmpty() || renderScanlines == "true") {
    DrawScanLines(backbuffer,
      SCREEN_SIZE_DIV_2 - 1,
      SCREEN_SIZE_DIV_2 - 1,
      SCREEN_SIZE_DIV_2 - 1 + (std::cos(sweepAngle) * SCREEN_SIZE_DIV_2),
      SCREEN_SIZE_DIV_2 - 1 + (std::sin(sweepAngle) * SCREEN_SIZE_DIV_2),
      20, 128, 5
    );
  }

  aircraftManager.Draw(backbuffer, sweepAngle);
  backbuffer.pushSprite(0, 0);
}

