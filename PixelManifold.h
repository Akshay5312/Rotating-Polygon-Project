#pragma once
#include <Arduino.h>
#include <BasicLinearAlgebra.h>

template<class T>
using Pixel = BLA::Matrix<3,1, T>; // RGB

typedef  BLA::MatrixFunctor<2, 1, float> Manifold; // a function that maps R2 to R1, pixels are colored according to this manifold.

template<int LED_SIDE_LENGTH>
class PixelManifold : BLA::MatrixFunctor<2, 3, float>{
    public:
        /**
         * @brief Construct a new PixelManifold object
         * 
         * @param manifold The manifold to color the pixels with
         * @param base_color The color of the pixel at x \in R^2 when manifold(x) = 0
         * @param color_grad color_grad*manifold(x) + base_color is the color of the pixel at position x \in R^2
         */
        PixelManifold(Manifold* manifold, Pixel<float> base_color = {0,0,0}, Pixel<float> color_grad = {1,1,1})
            : BLA::MatrixFunctor<2, 3, float>(), base_color_(base_color), color_grad_(color_grad), manifold_(manifold) { }
        

        /**
         * @brief Get the color of a pixel at position {x \in R^2 | (-1 < x_i < 1)}
         */
        Pixel<float> operator()(const BLA::Matrix<2,1,float>& x) const {
            BLA::Matrix<1,1> evaled_manifold = manifold_->operator()(x);

            return (color_grad_ * evaled_manifold(0)) + base_color_;
        }

    private:
        Pixel<float> base_color_;
        Pixel<float> color_grad_;

        Manifold* manifold_;
};


// Fill the rgb grid given the pixel value functor.
template<int LED_SIDE_LENGTH>
void get_grid(const PixelManifold<LED_SIDE_LENGTH>& manifold, uint8_t*** grid){
    for(int i = 0; i < LED_SIDE_LENGTH; i++){
        for(int j = 0; j < LED_SIDE_LENGTH; j++){
            BLA::Matrix<2, 1> x = {(2.0*i)/(LED_SIDE_LENGTH-1), (2.0*j)/(LED_SIDE_LENGTH-1)};
            x = x - (BLA::Matrix<2,1>)(BLA::Ones<2,1>());

            // if any value > 1, it will be clipped to 1.
            Pixel<float> p = manifold(x);

            for(int k = 0; k < 3; k++){
                grid[i][j][k] = (uint8_t)(256 * max(0.0, min(1-(1e-6), p(k))));
            }

        }
    }
}