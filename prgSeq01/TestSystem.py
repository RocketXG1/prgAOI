# Example showing how functions, that accept tuples of rgb values,
# simplify working with gradients

import time
import _thread
from  machine import Pin
#from NeoPixel.neopixel import Neopixel
from lib.Neopixel.neopixel import Neopixel
from lib.BlackLight.BlackLightControl import BlackLightControl

DATA_PIN = 29
STATE_MACHINE = 0
SECCIONES = 5
LEDS_POR_SECCION = [20, 20, 20,20,20]
TOTAL_LEDS = sum(LEDS_POR_SECCION)  # 370

numpix = TOTAL_LEDS
#strip = Neopixel(numpix, 0, 29, "RGB")
Ready = Neopixel(1, 1, 16, "GRB")

bSensor=Pin(27,Pin.IN)

BlackLight=BlackLightControl(28,1000)
BlackLight.set_percent(100)



# Colores RGB
amarillo = (255, 220, 0)
blanco   = (255, 255, 255)
cian     = (0, 255, 255)
purpura  = (160, 0, 200)

# "special" = mezcla/gradiente simple entre amarillo y purpura
# (promedio de ambos colores)
special = (
    (amarillo[0] + purpura[0]) // 2,
    (amarillo[1] + purpura[1]) // 2,
    (amarillo[2] + purpura[2]) // 2,
)




red = (255,0,0)
green = (0, 255,0)
blue = (0, 0, 255)

orange = (255, 50, 0)
yellow = (255, 150, 0)
cian = (0, 255, 255)
violet = (200, 0, 100)
wite= (120,120,120)
Off=(0,0,0)
colors_rgb = [red, orange, yellow, green, blue, cian, violet]

# same colors as normaln rgb, just 0 added at the end
colors_rgbw = [color+tuple([0]) for color in colors_rgb]
colors_rgbw.append((0, 0, 0, 255))

# uncomment colors_rgbw if you have RGBW strip
#colors = colors_rgb
colors = colors_rgbw

section_colors = [blue, blanco, blanco, cian, special]
triggers = [10, 10, 10, 10]

strip = Neopixel(num_leds=numpix, state_machine=STATE_MACHINE, pin=DATA_PIN, mode="RGB")
strip.brightness(255)

Ready.brightness(255)
Ready.fill(red)
Ready.show()
print("Ready")

iStartDelay=0.04
iFinishDelay=0.06
#strip.brightness(50)
#strip.fill(cian)
#strip.show() 
print("Ready2")

iStartDelay=0.05
iFinishDelay=0.06

#strip.circular_bounce_fill(wite,iStartDelay,iFinishDelay,150)
#strip.animate_color_range(25,34,red,0.5,False)
#strip.animate_color_range(0,35,Off,1,True) 

def MonStart():
    strip.circular_bounce_fill(wite,iStartDelay,iFinishDelay,100)
    strip.animate_color_range(0,TOTAL_LEDS,blue,0.5,False)    
    #strip.animate_color_range(0,24,yellow,1.5,False)

def MonSleep():
    #strip.animate_color_range(25,34,wite,0.5,True)
    strip.animate_color_range(0,TOTAL_LEDS,Off,1,True) 



Ready.brightness(50)
Ready.fill(green)
Ready.show()
while True: 
      
    #strip.animate_color_range(25,34,wite,0.5,True)
    #print("done")   
    #strip.SecuencialCascade(
    #data_pin=29,
    #total_leds=150,
    #sections_count=5,
    #leds_per_section=LEDS_POR_SECCION,
    #section_colors=[purpura,red,green, cian, blue],
    #step_ms=30,
    #trigger_position=triggers,
    #trail=True,
    #repeat=False,
    #state_machine=0,
    #brightness=80
#)
    strip._GradientTransition(section_colors_now=[yellow,purpura,cian],section_colors_mid=[red,blue,(255,0,255)],section_colors_final=[purpura,green,red],leds_per_section=[33,33,33],section_count=3,total_leds=100,step_ms=16,start_brightness=80,repeat=False,phase_steps=80)
    
    MonStart()      
    #BlackLight.ramp_percent(1,0.5,100,90)
    MonSleep() 
    #BlackLight.ramp_percent(1,0.5,90,100)
    
    
   # if bSensor.value():
    #    Ready.fill(cian)
     #   MonStart()
        
    #else:        
     #   Ready.fill(green)
      #  Ready.show()
       # MonSleep()

    
        
    
    
   
    
    
    
    

    
    
    
