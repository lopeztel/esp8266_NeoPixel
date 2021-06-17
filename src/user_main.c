#include "driver/uart.h"
#include "mqtt.h"
#include "user_config.h"
#include <c_types.h>
#include <eagle_soc.h>
#include <mem.h>
#include <os_type.h>
#include <osapi.h>
#include <string.h>
#include <user_interface.h>

// typedef void (*WifiCallback)(uint8_t);

extern int ets_uart_printf(const char *fmt, ...);
int (*console_printf)(const char *fmt, ...) = ets_uart_printf;

// Debug output.
#ifdef INFO_MSG
#undef INFO_MSG
#define INFO_MSG(...) console_printf(__VA_ARGS__);
#else
#define INFO_MSG(...)
#endif

typedef uint8_t neoPixelType;

static uint32_t _getCycleCount(void) __attribute__((always_inline));
static inline uint32_t _getCycleCount(void) {
  uint32_t ccount;
  __asm__ __volatile__("rsr %0,ccount" : "=a"(ccount));
  return ccount;
}

struct Adafruit_NeoPixel {
  bool begun;       // true if begin() previously called
  uint16_t numLEDs, // Number of RGB LEDs in strip
      numBytes;     // Size of 'pixels' buffer below (3 or 4 bytes/pixel)
  int8_t pin;       // Output pin number (-1 if not yet set)
  uint8_t brightness,
      *pixels, // Holds LED color values (3 or 4 bytes each)
      rOffset, // Index of red byte within each 3- or 4-byte pixel
      gOffset, // Index of green byte
      bOffset, // Index of blue byte
      wOffset; // Index of white byte (same as rOffset if no white)
  uint32_t endTime;
};

bool neopixel_canShow(struct Adafruit_NeoPixel *an) {
  return (system_get_time() - an->endTime) >= 300L;
}

LOCAL void espShow(uint8_t pin, uint8_t *pixels, uint32_t numBytes) {

#define CYCLES_800_T0H                                                         \
  (APB_CLK_FREQ / 2500000) // 0.4us			//F_CPU changed for the equivalent in
                           // eagle_soc.h : CPU_CLK_FREQ
#define CYCLES_800_T1H (APB_CLK_FREQ / 1250000) // 0.8us
#define CYCLES_800 (APB_CLK_FREQ / 800000) // 1.25us per bit
#define CYCLES_400_T0H (APB_CLK_FREQ / 2000000) // 0.5uS
#define CYCLES_400_T1H (APB_CLK_FREQ / 833333) // 1.2us
#define CYCLES_400 (APB_CLK_FREQ / 400000) // 2.5us per bit

  uint8_t *p, *end, pix, mask;
  uint32_t t, time0, time1, period, c, startTime, pinMask;

  pinMask = _BV(pin); //_BV is in arduino library had to include the define here
  p = pixels;
  end = p + numBytes;
  pix = *p++;
  mask = 0x80;
  startTime = 0;

  time0 = CYCLES_800_T0H;
  time1 = CYCLES_800_T1H;
  period = CYCLES_800;

  for (t = time0;; t = time0) {
    if (pix & mask)
      t = time1; // Bit high duration
    while (((c = _getCycleCount()) - startTime) < period)
      ;                                             // Wait for bit start
    GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, pinMask); // Set high
    startTime = c;                                  // Save start time
    while (((c = _getCycleCount()) - startTime) < t)
      ;                                             // Wait high duration
    GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, pinMask); // Set low
    if (!(mask >>= 1)) {                            // Next bit/byte
      if (p >= end)
        break;
      pix = *p++;
      mask = 0x80;
    }
  }
  while ((_getCycleCount() - startTime) < period)
    ; // Wait for last bit
}

void begin(struct Adafruit_NeoPixel *an) {
  if (an->pin >= 0) { // these functions depend on Arduino, declared using
                      // equivalents from <eagle_soc.h>
    // pinMode(pin, OUTPUT);
    // digitalWrite(pin, LOW);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, FUNC_GPIO2); // hardcoded GPIO 2
    GPIO_OUTPUT_SET(GPIO_ID_PIN(an->pin),
                    0); // assuming GPIO2 = pin2 and the only argument passed is
                        // pin 2, otherwise this won't work
  }
  an->begun = true;
}

void setPin(struct Adafruit_NeoPixel *an, uint8_t p) {
  if (an->begun && (an->pin >= 0))
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U,
                    FUNC_GPIO2); // pinMode(pin, INPUT); //again hardcoded, this
                                 // function may be useless
  an->pin = p;
  if (an->begun) {
    // pinMode(p, OUTPUT);
    // digitalWrite(p, LOW);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, FUNC_GPIO2); // hardcoded GPIO 2
    GPIO_OUTPUT_SET(GPIO_ID_PIN(an->pin), 0);
  }
}

void updateLength(struct Adafruit_NeoPixel *an, uint16_t n) {
  if (an->pixels) {
    os_free(an->pixels);
  } // Free existing data (if any)
  // Allocate new data -- note: ALL PIXELS ARE CLEARED
  an->numBytes = n * ((an->wOffset == an->rOffset) ? 3 : 4);
  if ((an->pixels = (uint8_t *)os_malloc(an->numBytes))) {
    os_memset(an->pixels, 0, an->numBytes);
    an->numLEDs = n;
  } else {
    an->numLEDs = an->numBytes = 0;
  }
}

void updateType(struct Adafruit_NeoPixel *an, neoPixelType t) {
  bool oldThreeBytesPerPixel = (an->wOffset == an->rOffset); // false if RGBW
  an->wOffset = (t >> 6) & 0b11; // See notes in header file
  an->rOffset = (t >> 4) & 0b11; // regarding R/G/B/W offsets
  an->gOffset = (t >> 2) & 0b11;
  an->bOffset = t & 0b11;

  // If bytes-per-pixel has changed (and pixel data was previously
  // allocated), re-allocate to new size.  Will clear any data.
  if (an->pixels) {
    bool newThreeBytesPerPixel = (an->wOffset == an->rOffset);
    if (newThreeBytesPerPixel != oldThreeBytesPerPixel)
      updateLength(an, an->numLEDs);
  }
}

uint32_t color_rgb(uint8_t r, uint8_t g, uint8_t b) {
  return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}

// Set pixel color from 'packed' 32-bit RGB color:
void setPixelColor(struct Adafruit_NeoPixel *an, uint16_t n, uint32_t c) {
  if (n < an->numLEDs) {
    uint8_t *p, r = (uint8_t)(c >> 16), g = (uint8_t)(c >> 8), b = (uint8_t)c;
    if (an->brightness) { // See notes in setBrightness()
      r = (r * an->brightness) >> 8;
      g = (g * an->brightness) >> 8;
      b = (b * an->brightness) >> 8;
    }
    if (an->wOffset == an->rOffset) {
      p = &(an->pixels[n * 3]);
    } else {
      p = &(an->pixels[n * 4]);
      uint8_t w = (uint8_t)(c >> 24);
      p[an->wOffset] = an->brightness ? ((w * an->brightness) >> 8) : w;
    }
    p[an->rOffset] = r;
    p[an->gOffset] = g;
    p[an->bOffset] = b;
  }
}

void show(struct Adafruit_NeoPixel *an) {
  if (!an->pixels)
    return;
  while (!neopixel_canShow(an))
    ;
  espShow(an->pin, an->pixels, an->numBytes);
  an->endTime = system_get_time();
}

void constructor(struct Adafruit_NeoPixel *an, uint16_t n, uint8_t p,
                 neoPixelType t) {
  an->begun = false;
  an->brightness = 50;
  an->pixels = NULL;
  an->endTime = 0;

  updateType(an, t);
  updateLength(an, n);
  setPin(an, p);
}

void clear_led_strip(struct Adafruit_NeoPixel *an) {
  for (uint8_t i = 0; i < NUMPIXELS; i++) { // Turn off everything
    setPixelColor(an, i, color_rgb(0, 0, 0));
    system_soft_wdt_feed();
    show(an);
  }
}

void set_led_strip(struct Adafruit_NeoPixel *an) {
  for (uint8_t i = 0; i < NUMPIXELS; i++) { // Turn off everything
    setPixelColor(an, i, color_rgb(255, 255, 255));
    system_soft_wdt_feed();
    show(an);
  }
}

struct Adafruit_NeoPixel ledstrip;

MQTT_Client mqttClient;
LOCAL bool ICACHE_FLASH_ATTR setup_wifi_st_mode();
static void ICACHE_FLASH_ATTR wifi_check_ip(void *arg);
static struct ip_info ipConfig;
static ETSTimer WiFiLinker;
static ETSTimer fadeTimer;
static void ICACHE_FLASH_ATTR fadeFunction(void *arg);
static tConnState connState = WIFI_CONNECTING;
bool firstTime = true;
uint8_t received_red;
uint8_t received_green;
uint8_t received_blue;
uint8_t fade_red = 255;
uint8_t fade_green = 255;
uint8_t fade_blue = 255;
bool fade = false;
bool solid = false;

enum fade_state{
  to_purple,
  to_red,
  to_orange,
  to_green,
  to_teal,
  to_blue
};

enum fade_state FADESTATE = to_purple;

const char *statusMapping[] = {"OK", "FAIL", "PENDING", "BUSY", "CANCEL"};

const char *authMapping[] = {"AUTH_OPEN",         "AUTH_WEP",
                             "AUTH_WPA_PSK",      "AUTH_WPA2_PSK",
                             "AUTH_WPA_WPA2_PSK", "AUTH_MAX"};

void mqttConnectedCb(uint32_t *args) {
  MQTT_Client *client = (MQTT_Client *)args;
  INFO_MSG("MQTT: Connected\r\n");
  MQTT_Subscribe(client, "/mqtt/lights/red", 2);
  MQTT_Subscribe(client, "/mqtt/lights/green", 2);
  MQTT_Subscribe(client, "/mqtt/lights/blue", 2);
  MQTT_Subscribe(client, "/mqtt/lights/color", 2);
  MQTT_Subscribe(client, "/mqtt/lights/fade", 2);
  MQTT_Subscribe(client, "/mqtt/lights/solid", 2);
}

void mqttDisconnectedCb(uint32_t *args) {
  MQTT_Client *client = (MQTT_Client *)args;
  INFO_MSG("MQTT: Disconnected\r\n");
}

void mqttPublishedCb(uint32_t *args) {
  MQTT_Client *client = (MQTT_Client *)args;
  INFO_MSG("MQTT: Published\r\n");
}

void mqttDataCb(uint32_t *args, const char *topic, uint32_t topic_len,
                const char *data, uint32_t data_len) {
  char *topicBuf = (char *)os_zalloc(topic_len + 1),
       *dataBuf = (char *)os_zalloc(data_len + 1);

  MQTT_Client *client = (MQTT_Client *)args;

  os_memcpy(topicBuf, topic, topic_len);
  topicBuf[topic_len] = 0;

  os_memcpy(dataBuf, data, data_len);
  dataBuf[data_len] = 0;

  INFO_MSG("Receive topic: %s, data: %s length: %d \r\n", topicBuf, dataBuf,
           data_len);
  int isRed = strcmp(topicBuf, "/mqtt/lights/red");
  int isGreen = strcmp(topicBuf, "/mqtt/lights/green");
  int isBlue = strcmp(topicBuf, "/mqtt/lights/blue");
  int isFade = strcmp(topicBuf, "/mqtt/lights/fade");
  int isSolid = strcmp(topicBuf, "/mqtt/lights/solid");
  INFO_MSG("topic comparison result: R %d, G %d, B %d, fade %d, solid%d\r\n",
           isRed, isGreen, isBlue, isFade, isSolid);
  if (isRed == 0) {
    received_red = dataBuf[0];
  } else if (isGreen == 0) {
    received_green = dataBuf[0];
  } else if (isBlue == 0) {
    received_blue = dataBuf[0];
  } else if (isFade == 0) {
    fade = dataBuf[0];
    solid = false;
  } else if (isSolid == 0) {
    solid = dataBuf[0];
    fade = false;
  }

  os_free(topicBuf);
  os_free(dataBuf);

  if (solid) {
    os_timer_disarm(&fadeTimer);

    INFO_MSG("Updated color is R:%d G:%d B:%d \r\n", received_red,
             received_green, received_blue);
    int i;
    for (i = 0; i < NUMPIXELS; i++) {
      // Color takes RGB values, from 0,0,0 up to 255,255,255
      setPixelColor(&ledstrip, i,
                    color_rgb(received_red, received_green, received_blue));
      system_soft_wdt_feed();
    }
    show(&ledstrip);
  } else if (fade) {
    fade_red = 0; //reset fade values
    fade_green = 0;
    fade_blue = 255;
    FADESTATE = to_purple;
    // MQTT_Publish(client,"/mqtt/lights/solid",0,1,2,0);
    for (uint8_t i = 0; i < NUMPIXELS; i++) {
      // Color takes RGB values, from 0,0,0 up to 255,255,255
      setPixelColor(&ledstrip, i, color_rgb(fade_red, fade_green, fade_blue));
      system_soft_wdt_feed();
    }
    show(&ledstrip);

    //initialize timer
    os_timer_disarm(&fadeTimer);
    os_timer_setfn(&fadeTimer, (os_timer_func_t *)fadeFunction, NULL);
    os_timer_arm(&fadeTimer, FADE_DELAY, 0);
  }
}

static void ICACHE_FLASH_ATTR wifi_check_ip(void *arg) {
  os_timer_disarm(&WiFiLinker);
  switch (wifi_station_get_connect_status()) {
  case STATION_GOT_IP:
    wifi_get_ip_info(STATION_IF, &ipConfig);
    if (ipConfig.ip.addr != 0) {
      connState = WIFI_CONNECTED;
      INFO_MSG("WiFi connected, wait MQTT message...\r\n");
      if (firstTime) {
        MQTT_Connect(&mqttClient);
        firstTime = false;
      }
    } else {
      connState = WIFI_CONNECTING_ERROR;
      INFO_MSG("WiFi connected, ip.addr is null\r\n");
    }
    break;
  case STATION_WRONG_PASSWORD:
    connState = WIFI_CONNECTING_ERROR;
    INFO_MSG("WiFi connecting error, wrong password\r\n");
    break;
  case STATION_NO_AP_FOUND:
    connState = WIFI_CONNECTING_ERROR;
    INFO_MSG("WiFi connecting error, ap not found\r\n");
    break;
  case STATION_CONNECT_FAIL:
    connState = WIFI_CONNECTING_ERROR;
    INFO_MSG("WiFi connecting fail\r\n");
    break;
  default:
    connState = WIFI_CONNECTING;
    INFO_MSG("WiFi connecting...\r\n");
  }
  os_timer_setfn(&WiFiLinker, (os_timer_func_t *)wifi_check_ip, NULL);
  os_timer_arm(&WiFiLinker, WIFI_CHECK_DELAY, 0);
}

static void ICACHE_FLASH_ATTR fadeFunction(void *arg) {
  os_timer_disarm(&fadeTimer);
  if (fade) {
    switch (FADESTATE) {
    case to_purple:
      if (fade_red < 255) { // purple
        fade_red += 51;
      } else {
        FADESTATE = to_red;
      }
      break;
    case to_red:
      if (fade_blue > 0) { // red
        fade_blue -= 51;
      } else {
        FADESTATE = to_orange;
      }
      break;
    case to_orange:
      if (fade_green < 255) { // orange
        fade_green += 51;
      } else {
        FADESTATE = to_green;
      }
      break;
    case to_green:
      if (fade_red > 0) { // green
        fade_red -= 51;
      } else {
        FADESTATE = to_teal;
      }
      break;
    case to_teal:
      if (fade_blue < 255) { // teal
        fade_blue += 51;
      } else {
        FADESTATE = to_blue;
      }
      break;
    case to_blue:
      if (fade_green > 0) { // blue
        fade_green -= 51;
      } else {
        FADESTATE = to_purple;
      }
      break;
    default:
      break;
    }

    for (uint8_t i = 0; i < NUMPIXELS; i++) {
      // Color takes RGB values, from 0,0,0 up to 255,255,255
      setPixelColor(&ledstrip, i, color_rgb(fade_red, fade_green, fade_blue));
      system_soft_wdt_feed();
    }
    show(&ledstrip);
  }
  os_timer_setfn(&fadeTimer, (os_timer_func_t *)fadeFunction, NULL);
  os_timer_arm(&fadeTimer, FADE_DELAY, 0);
}

LOCAL void ICACHE_FLASH_ATTR wifi_show_scan_info(void *arg, STATUS status) {
  INFO_MSG("\n==== Avaliable Networks: ====\n");
  if (status == OK) {
    struct bss_info *bssInfo;
    bssInfo = (struct bss_info *)arg;
    // skip first in chain as it is invalid
    bssInfo = STAILQ_NEXT(bssInfo, next);
    while (bssInfo != NULL) {
      INFO_MSG("SSID: %s\r\n", bssInfo->ssid);
      INFO_MSG("SECURITY: %s\r\n", authMapping[bssInfo->authmode]);
      INFO_MSG("RSSI: %d dB\r\n\n", bssInfo->rssi);
      bssInfo = STAILQ_NEXT(bssInfo, next);
    }

    INFO_MSG("Scan done, setting WiFi check timer...\r\n");
    // Wait for Wi-Fi connection
    os_timer_disarm(&WiFiLinker);
    os_timer_setfn(&WiFiLinker, (os_timer_func_t *)wifi_check_ip, NULL);
    os_timer_arm(&WiFiLinker, WIFI_CHECK_DELAY, 0);
  } else {
    INFO_MSG("There is a problem scanning nearby networks \r\n");
    INFO_MSG("Status is: %s \r\n", statusMapping[status]);
  }
}

LOCAL void ICACHE_FLASH_ATTR to_scan(void) {
  wifi_station_scan(NULL, wifi_show_scan_info);
}

LOCAL bool ICACHE_FLASH_ATTR setup_wifi_st_mode() {
  struct station_config stconfig;
  wifi_station_disconnect();
  wifi_station_dhcpc_stop();
  if (wifi_station_get_config(&stconfig)) {
    os_memset(stconfig.ssid, 0, sizeof(stconfig.ssid));
    os_memset(stconfig.password, 0, sizeof(stconfig.password));
    os_sprintf(stconfig.ssid, "%s", WIFI_CLIENTSSID);
    os_sprintf(stconfig.password, "%s", WIFI_CLIENTPASSWORD);
    if (!wifi_station_set_config(&stconfig)) {
      INFO_MSG("ESP8266 not set station config!\r\n");
      return false;
    }
  }
  wifi_station_connect();
  wifi_station_dhcpc_start();
  INFO_MSG("ESP8266 in STA mode configured.\r\n");
  return true;
}

/******************************************************************************
 * FunctionName : user_rf_cal_sector_set
 * Description  : SDK just reversed 4 sectors, used for rf init data and
 *paramters. We add this function to force users to set rf cal sector, since we
 *don't know which sector is free in user's application. sector map for last
 *several sectors : ABBBCDDD A : rf cal B : at parameters C : rf init data D :
 *sdk parameters Parameters   : none Returns      : rf cal sector
 *******************************************************************************/
uint32 ICACHE_FLASH_ATTR user_rf_cal_sector_set(void) {
  enum flash_size_map size_map = system_get_flash_size_map();
  uint32 rf_cal_sec = 0;

  switch (size_map) {
  case FLASH_SIZE_4M_MAP_256_256:
    rf_cal_sec = 128 - 8;
    break;

  case FLASH_SIZE_8M_MAP_512_512:
    rf_cal_sec = 256 - 5;
    break;

  case FLASH_SIZE_16M_MAP_512_512:
  case FLASH_SIZE_16M_MAP_1024_1024:
    rf_cal_sec = 512 - 5;
    break;

  case FLASH_SIZE_32M_MAP_512_512:
  case FLASH_SIZE_32M_MAP_1024_1024:
    rf_cal_sec = 1024 - 5;
    break;

  default:
    rf_cal_sec = 0;
    break;
  }

  return rf_cal_sec;
}

void ICACHE_FLASH_ATTR user_rf_pre_init(void) {}

void ICACHE_FLASH_ATTR user_init(void) {
  // Configure the UART
  uart_init(BIT_RATE_115200, BIT_RATE_115200);
  // Enable system messages
  system_set_os_print(1);

  // To print available networks
  wifi_set_opmode(STATION_MODE);
  system_init_done_cb(to_scan);

  INFO_MSG("\n==== System info: ====\n");
  INFO_MSG("SDK version:%s rom %d\n", system_get_sdk_version(),
           system_upgrade_userbin_check());
  INFO_MSG("Time = %ld\n", system_get_time());
  INFO_MSG("Chip id = 0x%x\n", system_get_chip_id());
  INFO_MSG("CPU freq = %d MHz\n", system_get_cpu_freq());
  // INFO_MSG("Flash size map = %s\n", system_get_flash_size_map()); //doesn't
  // work for some reason
  INFO_MSG("Free heap size = %d\n", system_get_free_heap_size());
  INFO_MSG("==== End System info ====\n");
  INFO_MSG("==== MQTT client setup ====\n");
  MQTT_InitConnection(&mqttClient, MQTT_HOST, MQTT_PORT, DEFAULT_SECURITY);
  INFO_MSG("MQTT settings:\r\n Host: %s\r\n Port: %d\r\n Security: %d\r\n",
           MQTT_HOST, MQTT_PORT, DEFAULT_SECURITY);
  MQTT_InitClient(&mqttClient, MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS,
                  MQTT_KEEPALIVE, 1);
  INFO_MSG(
      "MQTT client settings:\r\n Device ID: %s\r\n MQTT_User: %s\r\n "
      "MQTT_Password: %s\r\n MQTT_Keepalive: %d\r\n Uses clean session\r\n",
      MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS, MQTT_KEEPALIVE);
  MQTT_InitLWT(&mqttClient, "lwt", "offline", 0, 0); // last will topic
  MQTT_OnConnected(&mqttClient, mqttConnectedCb);
  MQTT_OnDisconnected(&mqttClient, mqttDisconnectedCb);
  MQTT_OnPublished(&mqttClient, mqttPublishedCb);
  MQTT_OnData(&mqttClient, mqttDataCb);
  INFO_MSG("==== End MQTT client setup ====\n");
  os_delay_us(10000);
  INFO_MSG("System init...\r\n");
  if (setup_wifi_st_mode()) {
    if (wifi_get_phy_mode() != PHY_MODE_11N)
      wifi_set_phy_mode(PHY_MODE_11N);
    if (wifi_station_get_auto_connect() == 0)
      wifi_station_set_auto_connect(1);
    wifi_station_set_reconnect_policy(TRUE);
  }
  // Init NeoPixel ledstrip
  INFO_MSG("Initializing NeoPixel led Strip ...\r\n");
  constructor(&ledstrip, NUMPIXELS, PIN, NEO_GRB);
  INFO_MSG("Updated NeoPixel led Strip struct...\r\n");
  os_delay_us(50000);
  begin(&ledstrip); // pin initialization
  uint8_t i;
  uint8_t j;

  for (uint8_t n = 0; n < 4; n++) {
    clear_led_strip(&ledstrip);
    for (j = 0; j < 255; j += 17) {
      for (i = 0; i < NUMPIXELS; i++) {
        // Color takes RGB values, from 0,0,0 up to 255,255,255
        setPixelColor(&ledstrip, i, color_rgb(j, j, j)); // All LEDs on.
        os_delay_us(250);                                // Delay
        system_soft_wdt_feed();
        /* show(&ledstrip); // This sends the updated pixel color to the */
      }
      show(&ledstrip); // This sends the updated pixel color to the
    }
    for (j = 255; j > 0; j -= 17) {
      for (i = 0; i < NUMPIXELS; i++) {
        // Color takes RGB values, from 0,0,0 up to 255,255,255
        setPixelColor(&ledstrip, i, color_rgb(j, j, j)); // use j value as color
        os_delay_us(250);                                // Delay
        system_soft_wdt_feed();
        /* show(&ledstrip); // This sends the updated pixel color to the */
      }
      show(&ledstrip); // This sends the updated pixel color to the
    }
  }

  set_led_strip(&ledstrip);
  INFO_MSG("System init done.\n");
}
