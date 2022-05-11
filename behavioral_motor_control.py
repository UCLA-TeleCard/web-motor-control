#!/usr/bin/env python3

USB_PORT = "/dev/ttyACM0"  # Arduino Uno R3 Compatible

# import all the required libraries:
# flask for web server hosting
# GPIO communicates with motors / servos
from asyncio.windows_events import NULL
from email.charset import SHORTEST
from fileinput import close
from functools import update_wrapper
from os import system
from pickle import FALSE
from shutil import move
from turtle import update
from urllib.request import CacheFTPHandler
from flask_socketio import SocketIO, emit
from flask import Flask, render_template, url_for, copy_current_request_context, request
from random import random
from threading import Thread, Event
import time
import atexit
import serial
import RPi.GPIO as GPIO
from time import sleep
from datetime import datetime 
import numpy as np

pip
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

GPIO.setup(7, GPIO.OUT)
GPIO.setup(11, GPIO.OUT)
GPIO.setup(13, GPIO.OUT)
GPIO.setup(15, GPIO.OUT)

# servo1 = TBD
flipper=GPIO.PWM(7, 50)
flipper.start(0)

grabber=GPIO.PWM(11, 50)
grabber.start(0)

dealerUp=GPIO.PWM(13, 50)
dealerUp.start(0)

dealerDown=GPIO.PWM(15, 50)
dealerDown.start(0)

#create the internal record of which cards are in the wheel 
wheel = np.zeros(13)
currentPosition = 0

# GLOBAL CONST VARIABLES ----------------------------------------------------------------------------------------------------------
#System States are coded as ints
STANDBY_EMPTY = 0
STANDBY_FULL = 1
CARD_HELD = 2
systemState = STANDBY_EMPTY
# Other random variables are coded as ints
DEALER_GATE = 0
WHEEL_GATE = 1
LEFT = 0
RIGHT = 1
CENTER = 2
EMPTY = False
FULL = True
# Experimentally determine the ideal servo posiitons
CLAW_OPEN = 140
CLAW_CLOSED = 145
STRAIGHT_UP = 100
DISCARD_POSITION = 120
FLIPPED_POSITION = -120
# Experimentally determine the ideal stepper w lead screw posiitons
TOP = 100
MIDDLE = 50
BOTTOM = 0
# Experimentally determine the number of steps 
# that the wheel motor should turn to swap between cards
CARD_STEP = 100


# HELPER FUNCTIONS ----------------------------------------------------------------------------------------------------------

def turnOffMotors(motor):
  motor.stop()
  GPIO.cleanup()

# input = angle in degrees, between 0 and 180
def setAngle(motor, angle):
  motor.ChangeDutyCycle(2+(angle/18))
  time.sleep(0.5)
  motor.ChangeDutyCycle(0)

# placeholder dynamic update function
def randomNumberGenerator():
  """
  Generate a random number every 2 seconds and emit to a socketio instance (broadcast)
  Ideally to be run in a separate thread?
  """
  #infinite loop of magical random numbers
  print("Making random numbers")
  while not thread_stop_event.is_set():
      number = round(random()*10, 3)
      print(number)
      socketio.emit('newnumber', {'number': number}, namespace='/steps')
      socketio.sleep(2)

# count stepper positions
def stepperCounter():
  global stepsLead
  stepsLead = 0
  global stepsLeadOld
  stepsLeadOld = 1
  while not thread_stop_event.is_set():
    if stepsLead != stepsLeadOld:
      socketio.emit('newnumber', {'number': int(stepsLead)}, namespace='/steps')
      stepsLeadOld = stepsLead
    socketio.sleep(0.1)

def updatePosition(direction):
  #called as part of turnWheel() only. 
  # Moving the wheel left increases the number, 
  # Moving the wheel right decreases it
  if direction == LEFT:
    if currentPosition ==12:
      currentPosition = 0
    else:
      currentPosition += 1 
  elif direction == RIGHT:
    if currentPosition ==0:
      currentPosition = 12
    else: 
      currentPosition -= 1 
  # if direction = CENTER, do nothing

def getWheelStatus():
  # use photogate to update record of which slots are full 
  status = cardDetected(WHEEL_GATE)
  # automatically update the internal record with the new data
  wheel[currentPosition] = status
  return status 

def cardDetected(gate):
  #use the photogate-sensing libraries to get a bool value for the specified photogate
  status = True
  return status

def findClosestEmpty():
  shortestDist = 100
  closestEmpty = -1
  # calculate the closest empty position in the wheel, return the direction to turn (right, left, or go straight up) 
  # run a calculation based on the existing record to find the shortest distance to an empty slot. 
  for i in range(0,12):
    if wheel[i] == EMPTY:
      if np.abs(i-currentPosition) < np.abs(shortestDist):
        shortestDist = i-currentPosition
        closestEmpty = i
   # return null if the wheel is full
    if closestEmpty == -1:
      return NULL
   # otherwise, return the direction to turn 
    else:
      if shortestDist < 0:
        return RIGHT 
      if shortestDist > 0: 
        return LEFT
      else:
        return CENTER

def openClaw():
  setAngle(grabber, CLAW_OPEN)

def closeClaw():
  setAngle(grabber, CLAW_CLOSED)

def moveGrabber(position):
  # input a variable representing the vertical position of the card grabber
  # move stepper the right number of steps 
  global stepsLead
  usb.write(str(position).encode(encoding="utf-8"))
  sleep(0.1)
  stepsLead = int(usb.readline().decode('utf-8').rstrip())

def turnWheel(direction):
  # code the direction and distance to travel into a serial message
  message = str(direction) + str(CARD_STEP)
  # update this code to actually move the stepper motor in a given direction
  usb.write(message.encode(encoding="utf-8"))
  updatePosition(direction)
  getWheelStatus()

def dealCardUp():
  # Alter this code to run the continuous servo for the right amount of time
  setAngle(dealerUp, 100)

def dealCardDown():
  # Alter this code to run the continuous servo for the right amount of time
  setAngle(dealerUp, -100)

# MAIN SYSTEM PROCESSES ----------------------------------------------------------------------------------------------------
# Imported from the behavioral structure detailed in the Behavioral FLowchart. 

def takeCardFromWheel():
  openClaw()
  moveGrabber(TOP)
  time.sleep(50)
  closeClaw()
  time.sleep(50)
  moveGrabber(MIDDLE)
  # We can choose to do something with this information (i.e. attempt process again) or not. 
  getWheelStatus()
  return "success"

def placeCardIntoWheel():
  if getWheelStatus() == FULL:
    direction = findClosestEmpty()
    if direction == NULL:
      # Wheel is full (notify higher-level processes)
      return "wheel full"
    while getWheelStatus() == FULL:
      turnWheel(direction)
  moveGrabber(TOP)
  openClaw()
  moveGrabber(MIDDLE)
  closeClaw()
  getWheelStatus()
  return "success"

def drawCardFromDeck():
  for i in range(3):
    dealCardUp()
    if cardDetected(DEALER_GATE) :
      break
    # Dealer Box is Empty or Jammed
    return "check dealer box"
  openClaw()
  moveGrabber(BOTTOM)
  closeClaw()
  moveGrabber(MIDDLE)
  return "success"
  
def discardFaceUp():
  setAngle(flipper, FLIPPED_POSITION)
  openClaw()
  time.sleep(100)
  setAngle(flipper, STRAIGHT_UP)
  closeClaw()
  return "success"

def discardFaceDown():
  setAngle(flipper, DISCARD_POSITION)
  openClaw()
  time.sleep(100)
  setAngle(flipper, STRAIGHT_UP)
  closeClaw()
  return "success"

def dealerBoxButton(): 
  dealCardDown()
# atexit.register(turnOffMotors(servo1))


# WEB SERVER CODE ----------------------------------------------------------------------------------------------------

# initialize the web server
app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
app.config['DEBUG'] = True

#turn the flask app into a socketio app
socketio = SocketIO(app, async_mode=None, logger=True, engineio_logger=True)

#async updater Thread
thread = Thread()
thread_stop_event = Event()




# defines the home page 
@app.route("/")
def web_interface():
  # html = open("web_interface.html")
  # response = html.read().replace('\n', '')
  # html.close()

  # setAngle(servo1, 90)
  # setAngle(servo2, 90)

  return render_template('web_interface.html')



# different "pages" for each individual button on the webpage 
# buttons: 
#   Draw Card Into Wheel    (DCIW)
#   Place Card Into Wheel   (PCIW)
#   Draw Card From Deck     (DCFD)
#   Take Card From Wheel    (TCFW)
#   Discard Face Up         (DFU)
#   Discard Face Down       (DFD)
#   Wheel Left              (WL)
#   Wheel Right             (WR)

# Draw Card Into Wheel 
@app.route("/DCIW")
def DCFW():
    print ("Received DCIW")
    if systemState == CARD_HELD:
        if placeCardIntoWheel():
          systemState = STANDBY_FULL
        else:
          return("Error: Wheel Full")
    else:
        if drawCardFromDeck() == "success":
          if placeCardIntoWheel() == "success":
            systemState = STANDBY_FULL
          else:
            return("Error: Wheel Full")
        else:
          return("Error: Dealer box is empty or jammed")
    return ("Received DCIW")

# Place Card Into Wheel
@app.route("/PCIW")
def PCIW():
    print ("Received PCIW")
    if systemState == CARD_HELD:
      # Evaluate if operation was performed or if wheel was full 
        if placeCardIntoWheel():
          systemState = STANDBY_FULL
        else:
          return("Error: Wheel Full")
    else:
        return("Error- cannot perform action")
    return ("Received DCFW")

# Draw Card From Deck
@app.route("/DCFD")
def DCFD():
    print ("Received DCFD")
    if systemState == CARD_HELD:
        return("Error- cannot perform action")
    else:
      if not drawCardFromDeck():
        return("Error: Dealer box is empty or jammed")
    return ("Received DCFD")

# Take Card From Wheel 
@app.route("/TCFW")
def TCFW():
    print ("Received TCFW")
    if systemState == STANDBY_FULL:
        takeCardFromWheel()
    else:
      return("Error- cannot perform action")
    return ("Received TCFW")

# Discard Face Down
@app.route("/DFD")
def DFD():
    print ("Received DFD")
    if systemState == STANDBY_EMPTY:
      return("Error- cannot perform action")
    elif systemState == STANDBY_FULL:
      takeCardFromWheel()
      discardFaceDown()
    elif systemState == CARD_HELD:
      discardFaceDown()
    # evalutate what the new system state should be 
    if getWheelStatus() == FULL:
      systemState = STANDBY_FULL
    else:
      systemState = STANDBY_EMPTY
    return ("Received DFD")

# Discard Face Up
@app.route("/DFU")
def DFU():
    print ("Received DFU")
    if systemState == STANDBY_EMPTY:
      return("Error- cannot perform action")
    elif systemState == STANDBY_FULL:
      takeCardFromWheel()
      discardFaceUp()
    elif systemState == CARD_HELD:
      discardFaceUp()
    # evalutate what the new system state should be 
    if getWheelStatus() == FULL:
      systemState = STANDBY_FULL
    else:
      systemState = STANDBY_EMPTY
    return ("Received DFU")

# Wheel Left
@app.route("/WL")
def WL():
    print ("Received WL")
    turnWheel(LEFT)
    # update the system state only if current state is not card_held
    if systemState != CARD_HELD:
      if getWheelStatus() == FULL:
        systemState = STANDBY_FULL
      else:
        systemState = STANDBY_EMPTY
    return ("Received WL")

# Wheel Right
@app.route("/WR")
def WR():
    print ("Received WR")
    turnWheel(RIGHT)
    if systemState != CARD_HELD:
      if getWheelStatus() == FULL:
        systemState = STANDBY_FULL
      else:
        systemState = STANDBY_EMPTY
    return ("Received WR")

# Other or outdated website functions *********************************************************************************

@app.route("/set_servo1")
def set_angle1():
  angle = int(request.args.get("angle"))
  print ("Received " + str(angle))

  setAngle(servo1, angle)

  return ("Received " + str(angle))

@app.route("/set_servo3")
def set_angle3():
  speed = int(request.args.get("speed"))
  print ("Received " + str(speed))
# 3.5 to 11.5 duty cycle
# 7.0? = stop position
  servo3.ChangeDutyCycle(speed/10)

  return ("Received " + str(speed))


@app.route("/set_servo4")
def set_angle4():
  speed = int(request.args.get("speed"))
  print ("Received " + str(speed))
# 3.5 to 11.5 duty cycle
# 7.0? = stop position
  servo4.ChangeDutyCycle(speed/10)

  return ("Received " + str(speed))


@app.route("/turn_wheel")
def turn_wheel():
  global stepsLead
  butt = request.args.get("state")
  print ("Received " + str(butt))
  usb.write(str(butt).encode(encoding="utf-8"))
  sleep(0.1)
  stepsLead = int(usb.readline().decode('utf-8').rstrip())
  print (stepsLead)
  return ("Received " + str(butt))



# DYNAMIC CODE -------------------------------------------------------------------------

@socketio.on('connect', namespace='/steps')
def test_connect():
    # need visibility of the global thread object
    global thread
    print('Client connected')

    #Start the stepper ounter thread only if the thread has not been started before.
    if not thread.is_alive():
        print("Starting Thread")
        # thread = socketio.start_background_task(randomNumberGenerator)
        thread = socketio.start_background_task(stepperCounter)

@socketio.on('disconnect', namespace='/steps')
def test_disconnect():
    print('Client disconnected')




# MAIN LOOP ----------------------------------------------------------------------------

# allows connections from the local network
def main():
  socketio.run(app, host='0.0.0.0')
  # app.run(host="0.0.0.0")






try:
  main()

finally:
  GPIO.cleanup()
  print("Goodbye!")