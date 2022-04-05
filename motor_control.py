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


# initialize the web server
app = Flask(__name__)

# initialize the GPIO pins, using the following Pin#'s:
# http://www.rpi-spy.co.uk/wp-content/uploads/2012/06/Raspberry-Pi-GPIO-Layout-Model-B-Plus-rotated-2700x900.png 
# GPIO.setmode(GPIO.BOARD)
# GPIO.setup(11, GPIO.OUT)

# pwm=GPIO.PWM(11, 50)
# pwm.start(0)

# def turnOffMotors():
#   pwm.stop()
#   GPIO.cleanup()

# def setAngle(angle):
#     duty = angle / 18 + 3
#     GPIO.output(11, True)
#     pwm.ChangeDutyCycle(duty)
#     sleep(1)
#     GPIO.output(11, False)
#     pwm.ChangeDutyCycle(duty)

# atexit.register(turnOffMotors)


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
