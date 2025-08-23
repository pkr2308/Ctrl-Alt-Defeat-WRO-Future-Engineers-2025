#include <hwrev2_rgbled.hpp>

const RGBColor hwrev2_RGBLED::RED      = {255, 0, 0};
const RGBColor hwrev2_RGBLED::GREEN    = {0, 255, 0};
const RGBColor hwrev2_RGBLED::BLUE     = {0, 0, 255};
const RGBColor hwrev2_RGBLED::AMBER    = {255, 255, 0};

hwrev2_RGBLED::hwrev2_RGBLED(VehicleConfig cfg){

  _config = cfg;

}

void hwrev2_RGBLED::init(ILogger *logger){

  _logger = logger;
  _led = new Adafruit_NeoPixel(1, _config.pinConfig.rgbLed, NEO_RGB + NEO_KHZ800);
  _led->begin();

}

void hwrev2_RGBLED::setStaticColor(RGBColor col){

  _led->setPixelColor(0, _led->Color(col.red, col.green, col.blue));
  _led->show();

}

void hwrev2_RGBLED::limitBrightness(uint8_t lim){

  _led->setBrightness(lim);
  
}