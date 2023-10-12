# AudioDSP-v3.0.0c
## Description
Audio DSP - 3.0.0c - First commercial version
### Board Code
- 3: Based on AudioDSP-v3.0 hardware
- 0: Audio Input Interface
   - 2.0 Analog RCA
- 0: Audio Output Interface
   - 2.0 Analog RCA
- c: Commercial Version

## Features
- DC Input +6V/1A
- Audio Channels: 2.0
- SigmaDSP ADAU1452
   - I2C control for DSP
- Sampling frequency (default): 192KHz
- 24 bit ADC PCM1862 x1
- 24 bit DAC PCM1681 x1
- ST uC STM32F401RxT6 with Serial Wire programming
- Synchronous Clock Distribution
- LED indicators:
   - +3V3
   - +5V0
   - DSP