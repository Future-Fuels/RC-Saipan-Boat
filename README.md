# RC Boat

## Flash

```sh
make rx-upload tx-upload
```

## Controls

- J1 vertical: motor.
- J2 horizontal: servo.
- Rotary encoder: motor speed limit. Starts at zero.
- Servo center defaults to 60° with ±30° deflection. Change both from the monitor
  or send `servo-config 60 30` to TX over serial.

Pins and settings are in `include/shared.h`.
