from flask import Flask
from flask import request

import time
import atexit
# import RPi.GPIO as GPIO
from time import sleep

app = Flask(__name__)

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

@app.route("/")
def web_interface():
  html = open("web_interface.html")
  response = html.read().replace('\n', '')
  html.close()
  #pwm.ChangeDutyCycle(7.5) # neutral position
  sleep(1)
  return response

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
  butt = request.args.get("butt")
  print ("Received " + str(butt))

  # pwm.ChangeDutyCycle(state)
  sleep(1)

  return ("Received " + str(butt))




def main():
  app.run(host= '0.0.0.0')

main()
