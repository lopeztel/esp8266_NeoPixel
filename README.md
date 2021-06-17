# esp8266_NeoPixel

## About

MQTT NeoPixel control using a esp8266 based board (ESP-01S). Platformio project
file for convenience.

## Requirements

- ESP-01S board
- ESP-01 ESP-01S RGB LED Controller Adapter
- WS2812 WS2812B n Bits Light Ring (I used 16 Bits)
- 3-5V supply or battery

![HW](HW.jpg)

- [Visual Studio Code](https://code.visualstudio.com/) (or Codium) with:
  - [PlatformIO extension](https://platformio.org/) with platform Espressif
    8266 installed (configuration for the board in
    [platformio.ini](platformio.ini))

## Demo

(Tested with MQTT_Dashboard)

![Demo](Demo.gif)

## References and links

- Libraries and code based on examples from the [Unofficial SDK](https://programs74.ru/udkew-en.html)
  for Espressif ESP8266
- [Adafruit NeoPixel Arduino Library](https://github.com/adafruit/Adafruit_NeoPixel)

## Important notes

It is important that the user modifies the corresponding fields between
```< >``` in the file ```src/user_config.h```

## TODO

- Implement initial SSID and Password configuration by setting the ESP as an
  Access Point, displaying a welcome site and have the user input stored in the
  device
- ~~Standalone light effects~~

## Contributing

Feel free to drop a line/contact me if interested in this project

