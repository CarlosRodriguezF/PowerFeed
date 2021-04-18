# PowerFeed
PowerFeed for Workshop Machines based on Arduino

# PowerFeed
PowerFeed for Workshop Machines based on Arduino

**DESCRIPTION**

Project based on Arduino to be able to control PAP motor driver (EN,DIR,PULSE) and set the speed in mm/min of the axis of your machines, like lathe, milling machine...

This project is being developed in VisualStudioCode with PlatformIO libraries. You can find the CPP file and migrate it to Arduino IDE if do you prefer

Firware.hex and .helf you can use these files to upload the code directly to your target with a USBasp programer for example. 

Basically this project consist in an Arduino Nano 328P, Rotary Encoder, 3Pos (Latched) Switch and I2C LCD2*16. You can control easily PAP motor driver like for example DM545 and select which direction do you want to feed and with which speed in mm/min do you want to go. It is possible as well to modify the speed when the motor is in movement. 


**CONFIGURATION**

You can adapt the control to your Driver/Motor changing this parameters in the main code. For the moment I have not checked all of the combinations, please if you find a BUG let me know. 

const int ENCODER_STEPS_PER_NOTCH = 4;  // Change this depending on which encoder is used

long Arduino_clk = 16000000;  //Oscillator frequency of arduino board (Nano 16MHz)

float Motor_stepangle = 1.8;  //This value is defined in the specs of the motor
int Driver_stepsrev = 6400;   //This value define the microstteping which you have selected in your driver
int Machine_lead  = 2;        //mm/rev this is the mm which your axis moves each turn of your leadscrew (In my case 2mm each rev)

bool DIR_Inv  = 1;    //Use this value to Invert or not the DIRection Pin function. (In case the direction of the power feed is inverted)
bool EN_Inv  = 0;     //Use this value to Invert or not the ENable Pin function.   

**PICTURES**



NOTES:
- Dont forget to use debouncing circuitry on the ENCODER in case it doesnt have it, is is recomended
- Dont supply the 5V for the LCD directly from the ARDUINO use external power source
- Block Diagram just as reference
