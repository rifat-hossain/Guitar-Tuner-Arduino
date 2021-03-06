# Guitar Tuner Device using Arduino
Every string in guitar ideally by default plays in a specific frequency. We can adjust the frequency by tightening or loosening the wires. If those frequencies match the ideal ones then it is called tuned guitar and if it is not then it is called off-tuned. To tune a guitar, each string should be played one by one and the signal it creates should be compared with the ideal signal of that string and adjust accordingly. The goal of the project is to detect which string is playing by capturing the signal by a regular microphone and suggest the person to tighten or loose the string to meet the ideal frequency.

## Equipment List
- Condenser Microphone
- Arduino
- Jumperwires
- Breadboard
- TDA2030 Audio Amplifier
- (1x) 10 kOhm Resistor
- (2x) 220 Ohm Resistors
- (6x) Greed LEDs
- (4x) Red LEDs
- (1x) Yellow LED
- 10uF Capacitor

## Working Principle
It stores n number of audio data from a condenser microphone and process FFT operation on it. Then the peak frequency found from the FFT is compared with all the string frequencies.
Finally, it indicates the string and tells you whether you have to tighten or loosen the string to get the right frequency by lighting up LEDs.

## Schematic Diagram
![Schematic Image](https://github.com/rifat-hossain/Guitar-Tuner/blob/main/img/schematic.jpg?raw=true)

## Test Run
[<img src="https://github.com/rifat-hossain/Guitar-Tuner-Arduino/blob/main/img/thumb.jpg?raw=true" width="480" height="270" />](https://youtu.be/kf_4T2DdHaA)
