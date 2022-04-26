# web-motor-control

This code provides a web dashboard that allows for granular control of servos (both continuous and not) and steppers.

## Installation

Clone the repo by running `git clone https://github.com/UCLA-TeleCard/web-motor-control.git`.

Copy the code to the Raspberry Pi using ![SFTP](https://linux.die.net/man/1/sftp). 

Install the required libraries:
```
RPi.GPIO

flask

flask_socketio
```

## Usage
Get the RPi IP address by running `ip a`. Ensure this matches the `ip_a` field in `./templates/web_interface.html`.

Start the program from within the folder using `python motor_control.py`.

Navigate in a web browser to `<ip address>:5000`.


## License

This work is dual-licensed under GPL 3.0 and BSD-3.
You must use both of them if you use this work.

`SPDX-License-Identifier: GPL-3.0-or-later AND BSD-3-Clause`

## References

[1] https://github.com/david0429/web-motor-control

[2] https://github.com/shanealynn/async_flask

[3] https://www.woolseyworkshop.com/2020/02/05/controlling-an-arduino-from-a-raspberry-pi/

[4] https://www.youtube.com/watch?v=xHDT4CwjUQE
