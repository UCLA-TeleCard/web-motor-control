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
servo1 = 4
servo2 = 17
servo3 = 27
servo4 = 22
servo5 = 10
servo6 = 9

PGate = 26

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

# defines the home page 
@app.route("/")
def web_interface():
  return render_template('web_dashboard.html')


@app.route("/DCFD")
def DCFD():
  butt = request.args.get("state")
  if butt == "TRUE":
    pi.set_servo_pulsewidth(servo3, 1300)
    pi.set_servo_pulsewidth(servo4, 1300)
    sleep(0.5)
    pi.set_servo_pulsewidth(servo4, 0)
    sleep(1.5)
    pi.set_servo_pulsewidth(servo3, 0)
  return True


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
  butt = request.args.get("state")
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
  pi.stop()
  print("Goodbye!")