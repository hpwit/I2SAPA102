# Driver to control APA102 strips in parallel output for esp32


## The APA102
The APA102 leds are expensives but really high quality and bring more control than their WS281X counterpart. These leds are clock driven, which means that their refresh rate can be really high ( couple of Mhz for APA102 vs 0.8Mhz for WS281X).The direct result is a higher fps which could be important when driving a lot of leds. For the 'slow' WS281X two main drivers have been written for esp32 to drive them in parallel output (automatically implemented in FastLED). This library is a take on driving APA102 strips in full parallel output. 

## What do I mean by parallel output ?
The FastLED library allows you to put strips on several pins. But when the `FastLED.show()` is executed, the strips are display one after the other. As a consequence if you declare 8 strips of 256 leds the time needed will be `8 x (time for 256 leds)`. This result in low fps espcially  when using WS2581X. The parallel output method allow to push all the strips at the same time hence in the later example the time needed will only be the time to display 256 leds

### What about different length strips
If you have strips of different lengths. For instance (200,10,256,900,800) then the time will be the one of the longuest strip i.e in our exmaple the time to display 900 leds

## What about esp32 ?
In the FastLED library, for the esp32 and the ws281X leds, the parallel output is automatically enabled. Two methods are available (RMT or I2S) I will not go into too much details into their difference here. But there is not yet a possiblity to drive clock driven leds like APA102 in parallel output.

## Goal of this library
Like has been done for the I2S parallel output, I have first developped the driver 'out of' the FastLED framework to have more control during the development phase. This library uses the CRBG object hence is fully compatible to be used with the FastLED library. 

### I do  not want to rewrite all my code FastLED based
Do not worry in a long term when testing is done, the code will be integrated within the FastlED framework.


## How to use it ?

### Install
Download this library into your favorite IDE library folder and restart it to be abel to access the examples

### Usage

You can find the full code in examples/testI2S

```C
#include "FastLED.h" //needed to have the CRGB and other fastled functions


#define NUM_LEDS_PER_STRIP 256
#define NUM_STRIPS 16 
#define NUM_LEDS NUM_STRIPS * NUM_LEDS_PER_STRIP
#define CLOCK_PIN 17 //needs to be higher than 16
#include "I2SAPA102.h"

CRGB leds[NUM_LEDS];
I2SAPA102 controller(0);

    
int Pins[16]={12,2,4,5,0,13,14,15,16,32,18,19,21,22,23,25}; //list of the pins used

void setup() {


        controller.initled(leds,Pins,CLOCK_PIN,NUM_STRIPS,NUM_LEDS_PER_STRIP); //default Speed 4MHz default clock pin 17 not changeable for now

        //controller.initled(leds,Pins,NUM_STRIPS,NUM_LEDS_PER_STRIP,freq in MHZ); i.e. controller.initled(leds,Pins,NUM_STRIPS,NUM_LEDS_PER_STRIP,2); //for 2MHZ


        fill_solid(leds,100,CRGB::Yellow);
        controller.showPixels(); //instead of FastLED.show()
}

...
```

## It doesn't work, what do I do ?
Go to reddit community r/FastLED drop a message u/Yves-bazin

