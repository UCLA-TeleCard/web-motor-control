#!/usr/bin/env python3

USB_PORT = "/dev/ttyACM0"  # Arduino Uno R3 Compatible

# import all the required libraries:
# flask for web server hosting
# flask_socketio for websockets stuff
# pigpio communicates with motors / servos using hardware timed PWM
# GPIO for photogates
from flask_socketio import SocketIO, emit
from flask import Flask, render_template, url_for, copy_current_request_context, request
from threading import Thread, Event
import time
import atexit
import serial
import pigpio
import RPi.GPIO as GPIO
from time import sleep
import numpy as np


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

# NEW GPIO pin numbering
# http://abyz.me.uk/rpi/pigpio/

# https://www.youtube.com/watch?v=xHDT4CwjUQE

# servo setup
# initialzie hardware GPIO
pi = pigpio.pi()
if not pi.connected:
   exit()
# initialize software GPIO
GPIO.setmode(GPIO.BCM)

# Use Broadcom pin numbering
servo1 = 4 # flipper
servo2 = 17 # grabber
servo3 = 27 # dealer box top
servo4 = 22 # dealer box bottom
servo5 = 10 # tripod pan
servo6 = 9 # tripod tilt

extButt = 13

PGate = 26
PGate2 = 19

# start servos at 50Hz (standard for servos)
pi.set_PWM_frequency(servo1, 50)
pi.set_PWM_frequency(servo2, 50)
pi.set_PWM_frequency(servo3, 50)
pi.set_PWM_frequency(servo4, 50)
pi.set_PWM_frequency(servo5, 50)
pi.set_PWM_frequency(servo6, 50)

# make the servo pins pwm output
pi.set_mode(servo1, pigpio.OUTPUT)
pi.set_mode(servo2, pigpio.OUTPUT)
pi.set_mode(servo3, pigpio.OUTPUT)
pi.set_mode(servo4, pigpio.OUTPUT)
pi.set_mode(servo5, pigpio.OUTPUT)
pi.set_mode(servo6, pigpio.OUTPUT)

# initialize photogate pins
GPIO.setup(PGate, GPIO.IN)
GPIO.setup(PGate2, GPIO.IN)

# set up pin to look for button press from other player
GPIO.setup(extButt, GPIO.IN)



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
# Experimentally determine the ideal servo posiitons (in us)
CLAW_OPEN = 2400
CLAW_CLOSED = 2175
GRABBER_VERT = 2200
GRABBER_FLIP = 150

# direction and speed of dealer box servos (in us)
DEALER_GO_UP = 1300
DEALER_GO_DOWN = 1700
DEALER_RETRIES = 3
# Experimentally determine the ideal stepper w lead screw posiitons
TOP = 2650
MIDDLE = 1350
BOTTOM = 0
# Experimentally determine the number of steps 
# that the wheel motor should turn to swap between cards
CARD_STEP = 31

isZeroed = False
isCardHeld = False
currentCard = 0
cardArray = np.zeros(13)

# FUNCTIONS ----------------------------------------------------------------------------------------------------------

# input = pulsewidth in microseconds
def setPulseWidth(motor, pw):
  pi.set_servo_pulsewidth(motor, pw)
  time.sleep(0.5)
  pi.set_servo_pulsewidth(motor, 0)

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

# control the card claw
def openClaw():
  print("open claw")
  setPulseWidth(servo2, CLAW_OPEN)
def closeClaw():
  print("close claw")
  setPulseWidth(servo2, CLAW_CLOSED)
# and the card grabber
def grabberVertical():
  print("grabber vertical")
  setPulseWidth(servo1, GRABBER_VERT)

def moveGrabber(position):
  # input a variable representing the vertical position of the card grabber
  # move stepper the right number of steps 
  global stepsLead
  global isZeroed
  if not isZeroed:
    error_message = "error: not zeroed"
    print (error_message)
    return error_message
  stepsLeadOld = stepsLead
  command = "X" + str(position)
  usb.write(command.encode(encoding="utf-8"))
  sleep(0.1)
  stepsLead = int(usb.readline().decode('utf-8').rstrip())
  print(stepsLead)
  return abs(stepsLead-stepsLeadOld)

def dealCardUp():
  pi.set_servo_pulsewidth(servo3, DEALER_GO_UP)
  pi.set_servo_pulsewidth(servo4, DEALER_GO_UP)
  sleep(0.5)
  pi.set_servo_pulsewidth(servo4, 0)
  sleep(2)
  pi.set_servo_pulsewidth(servo3, 0)
  sleep(0.5)

def dealCardDown():
  pi.set_servo_pulsewidth(servo3, DEALER_GO_DOWN)
  pi.set_servo_pulsewidth(servo4, DEALER_GO_DOWN)
  sleep(0.5)
  pi.set_servo_pulsewidth(servo3, 0)
  sleep(2)
  pi.set_servo_pulsewidth(servo4, 0)
  sleep(0.5)

def findClosestEmpty():
  global isZeroed
  global currentCard
  global cardArray
  # index of empty slots
  index = np.argwhere(cardArray == 0)
  print("index")
  print(index)
  # find empty that is closest
  indexDiff = np.subtract(index,currentCard)
  print("index diff")
  print(indexDiff)
  # returns index difference to get to closest empty slot
  closestEmpty = int(indexDiff[np.argwhere(abs(indexDiff) == min(abs(indexDiff)))[0][0]])
  print("closest empty = " + str(closestEmpty))
  return closestEmpty

def wheelGoTo(indexDiff):
  global isZeroed
  if not isZeroed:
    status = "error: not zeroed"
  else:
    if indexDiff >= 0:
      command = "L" + str(abs(indexDiff) * CARD_STEP)
    elif indexDiff < 0:
      command = "R" + str(abs(indexDiff) * CARD_STEP)
    usb.write(command.encode(encoding="utf-8"))
    sleep(0.1)
    status = "success"
  return True

def flipCard():
  global isZeroed
  if not isZeroed:
    status = "error: not zeroed"
  else:
    stepsDiff = moveGrabber(MIDDLE)
    sleep(1 + int(stepsDiff)*8/TOP)
    setPulseWidth(servo1, GRABBER_FLIP)

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


# different "pages" for each individual function

## CLIENT SIDE --------------------------------------

# adds function to the button press
# 200 ms debounce time
# https://sourceforge.net/p/raspberry-gpio-python/wiki/Inputs/
GPIO.add_event_detect(extButt, GPIO.RISING, callback=dealCardDown, bouncetime=200)

# defines the home page 
@app.route("/")
def web_interface():
  return render_template('web_dashboard.html')


# buttons: 
#   Draw Card Into Wheel    (DCIW)
#   Place Card Into Wheel   (PCIW)
#   Draw Card From Deck     (DCFD)
#   Play Card from Deck     (PCFD)
#   Take Card From Wheel    (TCFW)
#   Discard Face Up         (DFU)
#   Discard Face Down       (DFD)
#   Wheel Left              (WL)
#   Wheel Right             (WR)


# Draw Card into Wheel
@app.route("/DCFD")
def DCFD():
  global isCardHeld
  # butt = request.args.get("state")
  # if butt == "TRUE":
  dealCardUp()
  # for i in range(DEALER_RETRIES):
    # dealCardUp()
    # pi.set_servo_pulsewidth(servo3, DEALER_GO_UP)
    # sleep(1.2)
    # pi.set_servo_pulsewidth(servo3, 0)
    # sleep(0.5)
    # if not GPIO.input(PGate):
    #   error_message = "success"
    #   break
    # gets here if Dealer Box is Empty or Jammed
  error_message = "check dealer box"
  sleep(0.1)
    # return error_message
  grabberVertical()
  openClaw()
  stepsDiff = moveGrabber(BOTTOM)
  sleep(1 + int(stepsDiff)*8/TOP)
  closeClaw()
  moveGrabber(MIDDLE)
  isCardHeld = True
  print(error_message)
  return error_message


# Place Card into Wheel
@app.route("/PCIW")
def PCIW():
  global currentCard
  global cardArray
  global isCardHeld
  print("Recieved PCIW")
  if not isCardHeld:
    status = "no card in grabber"
  else:
    grabberVertical()
    stepsDiff = moveGrabber(TOP)
    print(stepsDiff)
    sleep(1 + int(stepsDiff)*8/TOP)
    openClaw()
    stepsDiff = moveGrabber(MIDDLE)
    sleep(1 + int(stepsDiff)*8/TOP)
    isCardHeld = False

    cardArray[currentCard] = 1
    print(cardArray)
    status = "success"
  print(status)
  return status


# PLay Card from Deck
@app.route("/PCFD")
def PCFD():
  dealCardDown()
  # for i in range(DEALER_RETRIES):
    # dealCardDown()
    # if not GPIO.input(PGate):
    #   error_message = "success"
    #   break
    # gets here if Dealer Box is Empty or Jammed
  error_message = "check dealer box"
  sleep(0.1)
  print(error_message)
  return error_message


# Take Card From Wheel 
@app.route("/TCFW")
def TCFW():
  global isCardHeld
  print ("Received TCFW")
  if isCardHeld:
    status = "error: grabber full"
  else:
    grabberVertical()
    openClaw()
    stepsDiff = moveGrabber(TOP)
    print(stepsDiff)
    sleep(1 + int(stepsDiff)*8/TOP)
    closeClaw()
    moveGrabber(MIDDLE)
    isCardHeld = True
    status = "success"
  print(status)
  return status

# go to closest empty
@app.route("/GTCE")
def GTCE():
  global currentCard
  closest = findClosestEmpty()
  indexDiff = currentCard - closest
  wheelGoTo(indexDiff)
  currentCard += closest
  status = "success"
  print(status)
  return status


# discard face up
# discard face down
@app.route("/DFU")
def DFU():
  global isCardHeld
  if not isCardHeld:
    status = "error: no card to drop"
  else:
    flipCard()
    stepsDiff = moveGrabber(BOTTOM)
    sleep(1 + int(stepsDiff)*8/TOP)
    sleep(0.2)
    openClaw()
    status = "success"
  print(status)
  return status

# discard face down
@app.route("/DFD")
def DFD():
  global isCardHeld
  if not isCardHeld:
    status = "error: no card to drop"
  else:
    stepsDiff = moveGrabber(BOTTOM)
    sleep(1 + int(stepsDiff)*8/TOP)
    setPulseWidth(servo1, 2400)
    sleep(0.2)
    openClaw()
    status = "success"
  print(status)
  return status

# Turn wheel left/right
@app.route("/turn_wheel")
def turn_wheel():
  global currentCard
  indexDiff = int(request.args.get("indexDiff"))
  wheelGoTo(indexDiff)
  currentCard += indexDiff
  if currentCard < 0:
    currentCard = 12
  elif currentCard > 12:
    currentCard = 0
  status = "sucesss"
  print("current card = " + str(currentCard))
  return status


## DEBUG PAGE ----------------------------------------
# debug webpage
@app.route("/debug")
def debug_interface():
  return render_template('debug_interface.html')


@app.route("/set_servo1")
def set_angle1():
  angle = int(request.args.get("angle"))
  setPulseWidth(servo1, angle)
  return ("Received " + str(angle))


@app.route("/set_servo2")
def set_angle2():
  angle = int(request.args.get("angle"))
  setPulseWidth(servo2, angle)
  return ("Received " + str(angle))


@app.route("/set_servo3")
def set_angle3():
  speed = int(request.args.get("speed"))
  pi.set_servo_pulsewidth(servo3, speed)
  return ("Received " + str(speed))


@app.route("/set_servo4")
def set_angle4():
  speed = int(request.args.get("speed"))
  pi.set_servo_pulsewidth(servo4, speed)
  return ("Received " + str(speed))


@app.route("/set_servo5")
def set_angle5():
  angle = int(request.args.get("angle"))
  setPulseWidth(servo5, angle)
  return ("Received " + str(angle))


@app.route("/set_servo6")
def set_angle6():
  angle = int(request.args.get("angle"))
  setPulseWidth(servo6, angle)
  return ("Received " + str(angle))


@app.route("/turn_stepper")
def turn_stepper():
  global stepsLead
  global isZeroed
  setPulseWidth(servo1, 2400)
  butt = request.args.get("state")
  usb.write(str(butt).encode(encoding="utf-8"))
  sleep(0.1)
  stepsLead = int(usb.readline().decode('utf-8').rstrip())
  if butt == "Z0" and stepsLead == 0:
    isZeroed = True
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
  pi.stop()
  print("Goodbye!")