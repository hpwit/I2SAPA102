


#include "FastLED.h"


#define NUM_LEDS_PER_STRIP 256
#define NUM_STRIPS 16 
#define NUM_LEDS NUM_STRIPS * NUM_LEDS_PER_STRIP
#define CLOCK_PIN 17 //needs to be higher than 16
#include "I2SAPA102.h"
    
int Pins[16]={12,2,4,5,0,13,14,15,16,32,18,19,21,22,23,25};

CRGB leds[NUM_LEDS];
I2SAPA102 controller(0);

void setup() {
  // put your setup code here, to run once:
     Serial.begin(115200);
  

controller.initled(leds,Pins,CLOCK_PIN,NUM_STRIPS,NUM_LEDS_PER_STRIP); //default Speed 4MHz 

//controller.initled(leds,Pins,CLOCK_PIN,NUM_STRIPS,NUM_LEDS_PER_STRIP,freq in MHZ);
//controller.initled(leds,Pins,CLOCK_PIN,NUM_STRIPS,NUM_LEDS_PER_STRIP,2); //for 2MHZ


fill_solid(leds,100,CRGB::Yellow);
controller.showPixels();
controller.setBrightness(20); //to be used instead of fastled.setbritghness value 0--> 255 
delay(200);

 controller.showPixels();
}


   


int offset=0;


void loop() {
memset(leds,0,NUM_STRIPS*(NUM_LEDS_PER_STRIP)*3);

for(int i=0;i<NUM_STRIPS;i++)
{
  for (int j=0;j<(i+1);j++)
  {

    leds[i*(NUM_LEDS_PER_STRIP)+(j+offset)%NUM_LEDS_PER_STRIP]=CRGB::Blue;
  }

}
long ts2=__clock_cycles();
 controller.showPixels();
  Serial.printf("fps:%f\n",(float)240000000/(__clock_cycles()-ts2));
 delay(50);
 offset++;

}