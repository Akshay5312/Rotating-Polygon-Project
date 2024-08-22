#include "PixelManifold.h"
#include "RotatingPolygon.h"

#include <FastLED.h>

#define LED_PIN     7
#define LED_LENGTH  16
#define NUM_LEDS    256  // 16 x 16

CRGB leds[NUM_LEDS];


template <int N>
class RotatingPolygonManifold : public Manifold{
    private:
        RotatingPolygon<N> polygon_;
    public:

        RotatingPolygonManifold() : Manifold() { }
        RotatingPolygonManifold(PolygonVerts<N> verts) : Manifold(), polygon_(verts) { }

        void step(double dt){
            polygon_.step(dt);
        }

        // The manifold value is the squared distance to the closest vertex.
        BLA::Matrix<1,1, float> operator()(const BLA::Matrix<2,1, float>& x) const override {
            BLA::Matrix<3> p = {x(0), x(1), 0};

            BLA::Matrix<3> closest_vert = polygon_.get_closest_vert( p );

            BLA::Matrix<3> dist = closest_vert - p;

            return BLA::MatrixTranspose<BLA::Matrix<3>>(dist) * dist;
        }

};

RotatingPolygonManifold<4> poly_manifold(get_pyramid());

PixelManifold<LED_LENGTH> pixel_manifold = PixelManifold<LED_LENGTH>(&poly_manifold);

long last_time = 0;

// RGB_grid display_grid;
uint8_t*** rgb_grid;

void setup() {
    rgb_grid = new uint8_t**[LED_LENGTH];
    for(int i = 0; i < LED_LENGTH; i++){
      rgb_grid[i] = new uint8_t*[LED_LENGTH];
      for(int j = 0; j < LED_LENGTH; j++){
        rgb_grid[i][j] = new uint8_t[3];
        for(int k = 0; k < 3; k++){
          rgb_grid[i][j][k] = 0;
        }
      }
    }

    FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(50);

    last_time = millis();
}

void display(uint8_t*** rgb_grid){
    for(int i = 0; i < LED_LENGTH; i++){
        for(int j = 0; j < LED_LENGTH; j++){
            leds[i*LED_LENGTH + j].r = rgb_grid[i][j][0];
            leds[i*LED_LENGTH + j].g = rgb_grid[i][j][0];
            leds[i*LED_LENGTH + j].b = rgb_grid[i][j][0];
        }
    }
    FastLED.show();
}

void loop() {
    // Add your main code here
    poly_manifold.step(millis() - last_time);
    last_time = millis();
    
    get_grid(pixel_manifold, rgb_grid);

    display(rgb_grid);
}