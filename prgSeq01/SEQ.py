# Example showing how functions, that accept tuples of rgb values,
# simplify working with gradients

import time
from NeoPixel.neopixel import Neopixel

numpix = 32
strip = Neopixel(numpix, 0, 29, "RGB")
# strip = Neopixel(numpsix, 0, 0, "GRBW")

red = (255, 0, 0)
orange = (255, 50, 0)
yellow = (255, 100, 0)
green = (0, 255, 0)
blue = (0, 0, 255)
indigo = (100, 0, 90)
violet = (200, 0, 100)
colors_rgb = [red, orange, yellow, green, blue, indigo, violet]

# same colors as normaln rgb, just 0 added at the end
colors_rgbw = [color+tuple([0]) for color in colors_rgb]
colors_rgbw.append((0, 0, 0, 255))

# uncomment colors_rgbw if you have RGBW strip
#colors = colors_rgb
colors = colors_rgbw


step = round(numpix / len(colors))
current_pixel = 0
strip.brightness(255)

for color1, color2 in zip(colors, colors[1:]):
    strip.set_pixel_line_gradient(current_pixel, current_pixel + step, color1, color2)
    current_pixel += step

strip.set_pixel_line_gradient(current_pixel, numpix - 1, violet, red)
print("Ready")

iStartDelay=0.03
iFinishDelay=0.06
while True:
    #strip.rotate_right(1)
    #strip.fill(colors[2])
    #time.sleep(0.001)
    #strip.show()    

    strip.circular_bounce_fill(yellow,iStartDelay,iFinishDelay,255)
    
    #for Step in colors:
     #   strip.set_pixel(0,Step)
      #  time.sleep(0.1)
      #  strip.show()
        
    
    
    
