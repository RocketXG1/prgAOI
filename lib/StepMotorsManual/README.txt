StepMotorsManual
================

Carpeta reservada para ejemplos y documentacion de uso de motores paso a paso.

Ejemplo rapido:

from lib.StepMotors.objStepMotors import objStepMotor

motor = objStepMotor(step_pin=2, direction_pin=3, enable_pin=4)
motor.enable()
motor.step(200, clockwise=True)
motor.rotate_degrees(90)
motor.disable()
