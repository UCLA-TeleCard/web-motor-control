#!/usr/bin/env python3

USB_PORT = "/dev/ttyACM0"  # Arduino Uno R3 Compatible

# import all the required libraries:
# flask for web server hosting
# GPIO communicates with motors / servos
from flask import Flask
from flask import request

import time
import atexit
import serial
import RPi.GPIO as GPIO
from time import sleep


# Connect to USB serial port
try:
  
  usb = serial.Serial(USB_PORT, 115200, timeout=2)
  usb.baudrate = 115200
  print("Arduino connected")
   
except:
   print("ERROR - Could not open USB serial port.  Please check your port name and permissions.")
   print("Exiting program.")
   exit()




# initialize the GPIO pins, using the following Pin#'s:
# https://www.windtopik.fr/wp-content/uploads/2014/11/RPI-GPIO-N-.png
# https://www.youtube.com/watch?v=xHDT4CwjUQE

# servo setup
GPIO.setmode(GPIO.BOARD)
GPIO.setup(11, GPIO.OUT)

# servo1 = TBD
servo1=GPIO.PWM(11, 50)
servo1.start(0)


def turnOffMotors(motor):
  motor.stop()
  GPIO.cleanup()

def setAngle(motor, angle):
  motor.ChangeDutyCycle(2+(angle/18))
  time.sleep(0.5)
  motor.ChangeDutyCycle(0)

atexit.register(turnOffMotors(servo1))


# WEB SERVER CODE ----------------------------------------------------------------------------------------------------

# initialize the web server
app = Flask(__name__)

# defines the home page 
@app.route("/")
def web_interface():
  html = open("web_interface.html")
  response = html.read().replace('\n', '')
  html.close()
  #pwm.ChangeDutyCycle(7.5) # neutral position
  sleep(1)
  return response


# different "pages" for each individual function
@app.route("/set_speed")
def set_speed():
  speed = int(request.args.get("speed"))
  print ("Received " + str(speed))

  #pwm.ChangeDutyCycle(speed)
  sleep(1)

  return ("Received " + str(speed))



@app.route("/set_toggle")
def set_toggle():
  state = request.args.get("state")
  print ("Received " + str(state))

  # pwm.ChangeDutyCycle(state)
  sleep(1)

  return ("Received " + str(state))


@app.route("/set_button")
def set_button():
  butt = request.args.get("state")
  print ("Received " + str(butt))

  # pwm.ChangeDutyCycle(state)
  sleep(1)

  return ("Received " + str(butt))


@app.route("/turn_wheel")
def turn_wheel():
  butt = request.args.get("state")
  print ("Received " + str(butt))
  usb.write(str(butt).encode(encoding="utf-8"))
  # sleep(1)
  
  return ("Received " + str(butt))



# allows connections from the local network
def main():
  app.run(host= '0.0.0.0')

main()
