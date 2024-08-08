# Telemetrix4RpiPico

![](images/tmx.png)

The Pico server code may be viewed [here.](https://github.com/mirte-robot/Telemetrix4RpiPico)

# Interface

Data transfers and how to communicate with telemetrix. The protocol is [length_of_msg, msg_id, msg....]. No error correction, no crc and no recovery if byte count is off. All the following messages have the length removed. Send and receive are from the computers perspective.
## Main

### Pico ID
Get a 8 byte unique id of the microcontroller.
`msg_id = 6`, both receive and send.

Send:
```py
[REPORT_PICO_UNIQUE_ID = 6]
```
Receive:
```py
[REPORT_PICO_UNIQUE_ID = 6, id[0], id[1], id[2], id[3],id[4],id[5],id[6],id[7] ]
```

### Watchdog
At first watchdog message, the watchdog is started. This message and any subsequent message will be acknowledged with the same COUNTER as the computer sent + a constant `boot_id`, that is randomly generated when the watchdog is started. The computer can check the counter to make sure no messages are lost. The constant `boot_id` is for the computer to check that the mcu did not restart. Watchdog timeout is 5s, but modules get a message after 4s to disable( we don't want rogue robots that are uncontrollable).

`msg_id = 32`. Both send and receive.

Send:
```py
[PING_ID = 32, COUNTER]
```

Receive:
```py
[PONG_ID = 32, COUNTER_FROM_COMPUTER, CONST_boot_id]
```


## Sensors

### Sonar
The sonars(hc-sr04) are time-triggered round-robin at a rate of 10Hz to not interfere with eachother. 10Hz per sonar, 3 sonars, timer at 30Hz. The sonar system uses a mutex to make sure writing of a new reading will not mess up reading from it. This might lead to missing sonar readings/messages.

Max distance is 4m. Distance is reported in cm, in 2 bytes: ```distance_m = int(distance/100); distance_cm = distance%100```

max sonars is 4.

Add sonar:
```py

[SONAR_NEW = 13, trigger_pin, echo_pin]
```

Receive:
```py
[trigger_pin, distance_m, distance_cm]
```

### Encoders
2 types are supported, quadrature and single pin. Every edge is counted (up and down, A & B). No interrupts are used, but the encoders are checked at 10kHz. Max of 4 encoders.

Encoder type is 1 for single pin encoder, 2 for quadrature. pin_B is always sent, but must be 0 if not used. The steps are reset on every report, you need to add them to keep a incremental value. The steps need to be interpreted as ```int8_t```, as a quadrature encoder can count down. 

Add encoder:
```py
[ENCODER_NEW = 30, encoder_type, pin_A, pin_B]
```

Receive:
```py
[ENCODER_REPORT = 14, pin_A, steps]
```

### Read pin
To read a digital/analog pin, you need to send a command to set the pin mode and then, on-change, the data will be sent. 

#### Analog
The pin must be in range of [0..5] (pico: 26+adc_pin == gpio_pin, 4==temp_sensor). differential is the minimal change for the value or it is not reported.

Differential is ```uint16_t```, with ```diff_high = diff>>8``` and ```diff_low = diff&0xFF```. 

Set:
```py
[SET_PIN_MODE = 1, adc_pin, ANALOG_IN = 5, diff_high, diff_low ]
```

Receive:
```py
[ANALOG_REPORT = 3, adc_pin, value_high, value_low]
```

#### Digital
Only on change, the data is sent. digital_in_type can be:
- DIGITAL_INPUT = 0
- DIGITAL_INPUT_PULLUP = 3
- DIGITAL_INPUT_PULLDOWN = 4

```report_enable``` can be used to enable or disable reporting. ```1``` for enable.
Set:
```py
[SET_PIN_MODE = 1, pin, digital_in_type, report_enable]
```

Receive:
```py
[DIGITAL_REPORT = 2, pin, value]
```
## Actuators

### Digital output
First set pin once as output. Afterwards send updates to set new values.

Set:
```py
[SET_PIN_MODE = 1, pin, DIGITAL_OUTPUT = 1]
```
Update:
```py
[DIGITAL_WRITE = 2, pin, value]
```

### PWM output / Servo
The PWM mode can be used for servos, as the PWM frequency is set to 50Hz. The value must be in range ```[0, 20000]```. ```value_high = value>>8```, ```value_low = value&0xFF```.

Setup:
```py
[SET_PIN_MODE = 1, pin, PWM_OUTPUT = 2]
```
Update:
```py
[PWM_WRITE = 3, pin, value_high, value_low]
```
## Modules
### Hiwonder:
Servos are identified based on an id. The actual servo id is only transmitted at init, after that the index of the servo in the init list is used to offload id searching from the pico to the computer.
#### Init:
```py
[ uart_id, rx_pin, tx_pin, len(servos), servo_1_id, servo_2_id, ..., servo_X_id ]
```

#### Set angle:
Message subtype 1

time in ms.
```py
 [ 1, len(servos), servo_1_num, servo_1_angle_high_byte, servo_1_angle_low_byte, servo_1_time_high_byte, servo_1_time_low_byte, ...,  servo_X_angle_high_byte, servo_X_angle_low_byte, servo_X_time_high_byte, servo_X_time_low_byte, ...]
 ```

#### Set enabled:
 ```py
[ 2, len(servos), enabled(1 or 0) servo_1_num, servo_X_num]
```
#### Set all enabled:
```py
[ 2, 0, enabled]
```
