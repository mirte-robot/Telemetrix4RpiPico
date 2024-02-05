# Telemetrix4RpiPico

![](images/tmx.png)

The Pico server code may be viewed [here.](https://github.com/mirte-robot/Telemetrix4RpiPico)

# Modules interface

## Hiwonder:
Servos are identified based on an id. The actual servo id is only transmitted at init, after that the index of the servo in the init list is used to offload id searching from the pico to the computer.
### Init:
```py
[ uart_id, rx_pin, tx_pin, len(servos), servo_1_id, servo_2_id, ..., servo_X_id ]
```

### Set angle:
Message subtype 1

time in ms.
```py
 [ 1, len(servos), servo_1_num, servo_1_angle_high_byte, servo_1_angle_low_byte, servo_1_time_high_byte, servo_1_time_low_byte, ...,  servo_X_angle_high_byte, servo_X_angle_low_byte, servo_X_time_high_byte, servo_X_time_low_byte, ...]
 ```

### Set enabled:
 ```py
[ 2, len(servos), enabled(1 or 0) servo_1_num, servo_X_num]
```
### Set all enabled:
```py
[ 2, 0, enabled]
```