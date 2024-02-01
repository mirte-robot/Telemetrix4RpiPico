# Telemetrix4RpiPico

![](images/tmx.png)

The Pico server code may be viewed [here.](https://github.com/mirte-robot/Telemetrix4RpiPico)

# Modules interface

## Hiwonder:
### Set angle:
Message subtype 1
```py
 [ 1, len(servos), servo_1_num, servo_1_angle_high_byte, servo_1_angle_low_byte, servo_1_time_high_byte, servo_1_time_low_byte, servo_X_angle_high_byte, servo_X_angle_low_byte, servo_X_time_high_byte, servo_X_time_low_byte, ...]
 ```

### Set enabled:
 ```py
[ 2, len(servos), enabled(1 or 0) servo_1_num, servo_X_num]
```
### Set all enabled:
```py
[ 2, 0, enabled]
```