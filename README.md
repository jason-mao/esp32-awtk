# esp32-awtk

[ESP32](https://github.com/espressif/esp-idf) is a very popular IOT chips in the world.

[AWTK](https://github.com/zlgopen/awtk) is an open source GUI. It's easy to support embedded system, Wechat, WEB, Mobile phone and PC.

# Quick Start

Clone the `esp32-awtk` repository, set up `IDF_PATH`, and then compile, flash and monitor applications in the same way as when working with ESP-IDF.

```
git clone --recursive https://github.com/jason-mao/esp32-awtk.git
cd esp32-awtk/example
export IDF_PATH=$PWD/../esp-idf
make flash monitor
```

# Todo list
- [x] Porting awtk to support ESP32
- [x] Running first demo on ESP-WROVER-KIT with WROVER module
- [ ] Support touch input events
- [ ] Implement framebuffer to improve speed
