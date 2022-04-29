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
# import RPi.GPIO as GPIO
import pigpio
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

# NEW GPIO pin numbering
# http://abyz.me.uk/rpi/pigpio/

# https://www.youtube.com/watch?v=xHDT4CwjUQE

# servo setup
pi = pigpio.pi()
if not pi.connected:
   exit()

# Use Broadcom pin numbering
servo1 = 4
servo2 = 17
servo3 = 27
servo4 = 22

# start servos at 50Hz (standard for servos)
pi.set_PWM_frequency(servo1, 50)
pi.set_PWM_frequency(servo2, 50)
pi.set_PWM_frequency(servo3, 50)
pi.set_PWM_frequency(servo4, 50)

# make the servo pins pwm output
pi.set_mode(servo1, pigpio.OUTPUT)
pi.set_mode(servo2, pigpio.OUTPUT)
pi.set_mode(servo3, pigpio.OUTPUT)
pi.set_mode(servo4, pigpio.OUTPUT)



# FUNCTIONS ----------------------------------------------------------------------------------------------------------

# def turnOffMotors(motor):
#   motor.stop()
  # GPIO.cleanup()

# input = angle in degrees, between 0 and 180
def setPulseWidth(motor, pw):
  pi.set_servo_pulsewidth(motor, pw)
  time.sleep(0.5)
  pi.set_servo_pulsewidth(motor, 0)

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
  return render_template('web_dashboard.html')


@app.route("/debug")
def debug_interface():
  return render_template('debug_interface_hgpio.html')



# different "pages" for each individual function
@app.route("/set_servo1")
def set_angle1():
  angle = int(request.args.get("angle"))
  print ("Received " + str(angle))

  setPulseWidth(servo1, angle)

  return ("Received " + str(angle))


@app.route("/set_servo2")
def set_angle2():
  angle = int(request.args.get("angle"))
  print ("Received " + str(angle))

  setPulseWidth(servo2, angle)

  return ("Received " + str(angle))


@app.route("/set_servo3")
def set_angle3():
  speed = int(request.args.get("speed"))
  print ("Received " + str(speed))

  pi.set_servo_pulsewidth(servo3, speed)

  return ("Received " + str(speed))


@app.route("/set_servo4")
def set_angle4():
  speed = int(request.args.get("speed"))
  print ("Received " + str(speed))

  pi.set_servo_pulsewidth(servo4, speed)

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
  # GPIO.cleanup()
  pi.stop()
  print("Goodbye!")