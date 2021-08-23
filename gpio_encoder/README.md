# gpio_encoder

Gpio Encoder Driver Ros Package

This Package is for RaspberryPi

### Yotube Video

### Dependency package
- [wiringPi](http://wiringpi.com/download-and-install/)

- [encoder_msgs](https://github.com/PigeonSensei/pigeon_encoder_driver/tree/master/encoder_msgs)

### Run

```bash
roslaunch gpio_encoder gpio_encoder.launch
```

### Published Topics

- encoder_count ([encoder_msgs/EncoderCount](https://github.com/PigeonSensei/pigeon_encoder_driver/blob/master/encoder_msgs/msg/EncoderCount.msg))

### Services
- resetEncoderCount ([encoder_msgs/ResetEncoderCount](https://github.com/PigeonSensei/pigeon_encoder_driver/blob/master/encoder_msgs/srv/ResetEncoderCount.srv))

  Reset the encoder counter

### Parameters

- ~ Encoder0_A (int, default: 0)

  Gpio Pin number corresponding to Encoder of 0A
  
- ~ Encoder0_B (int, default: 0)

  Gpio Pin number corresponding to Encoder of 0B

- ~ Encoder1_A (int, default: 0)

  Gpio Pin number corresponding to Encoder of 1A
  
- ~ Encoder1_B (int, default: 0)

  Gpio Pin number corresponding to Encoder of 1B
  
  
