#!/usr/bin/env python3

USB_PORT = "/dev/ttyACM0"  # Arduino Uno R3 Compatible

# import all the required libraries:
# flask for web server hosting
# GPIO communicates with motors / servos
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
servo1=GPIO.PWM(7, 50)
servo1.start(0)

servo2=GPIO.PWM(11, 50)
servo2.start(0)

servo3=GPIO.PWM(13, 50)
servo3.start(0)

servo4=GPIO.PWM(15, 50)
servo4.start(0)


# FUNCTIONS ----------------------------------------------------------------------------------------------------------

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



# different "pages" for each individual function
@app.route("/set_servo1")
def set_angle1():
  angle = int(request.args.get("angle"))
  print ("Received " + str(angle))

  setAngle(servo1, angle)

  return ("Received " + str(angle))


@app.route("/set_servo2")
def set_angle2():
  angle = int(request.args.get("angle"))
  print ("Received " + str(angle))

  setAngle(servo2, angle)

  return ("Received " + str(angle))


@app.route("/set_servo3")
def set_angle3():
  speed = int(request.args.get("speed"))
  print ("Received " + str(speed))
# 3.5 to 11.5 duty cycle
# 7.0? = stop position
  servo3.ChangeDutyCycle(speed/10)
  sleep(0.5)

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