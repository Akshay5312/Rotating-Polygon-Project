# Rotating Polygon Project for Arduino

This project visualizes a rotating polygon on a 2D LED grid.

It's an abstract representation, where each pixel is colored based on their relative distance from the nearest vertex.

The rotation dynamics can be extended/implemented. The polygon can be specified (as a set of vertices). A custom color gradient can be set. Byt default, the gradient is black to white (the closer a pixel location is to a vertex, the more white).

I used a 16 x 16 LED light grid, and an Arduino Uno.