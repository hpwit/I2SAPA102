/*
 Author Yves BAZIN
 change the Speed to adapt to 3.2 Mhz and 32 bits and all the functions to push the leds
 based on the work of bitluni 2019

*/
#pragma once

#include "esp_heap_caps.h"
#include "soc/soc.h"
#include "soc/gpio_sig_map.h"
#include "soc/i2s_reg.h"
#include "soc/i2s_struct.h"
#include "soc/io_mux_reg.h"
#include "driver/gpio.h"
#include "driver/periph_ctrl.h"
#include "rom/lldesc.h"
//#include "DMABuffer.h"
#include "FastLED.h"

#ifndef NUM_LEDS_PER_STRIP
#define NUM_LEDS_PER_STRIP 21
#endif

//#ifndef NUM_STRIPS
//#define NUM_STRIPS 16
//#endif
#define AA (0x00AA00AAL)
#define CC (0x0000CCCCL)
#define FF (0xF0F0F0F0L)
#define FF2 (0x0F0F0F0FL)

struct DMABufferClockI2SAPA102 {
    lldesc_t descriptor;
    uint8_t * buffer;
};
//static DMABuffer * dmaBuffers[NUM_DMA_BUFFERS];
class I2SAPA102
{
  public:
    int i2sIndex;
    intr_handle_t interruptHandle;
    volatile long timme[200];
    volatile int ledbuff[50];
    volatile long t1,t2;
    int dmaBufferActive;
    uint8_t cN,cA,cB;
    DMABufferClockI2SAPA102 * dmaBuffers[4];
    volatile bool stopSignal;
 volatile bool runningPixel=false;
 volatile float timmer;
   volatile int ledToDisplay;
   volatile int oo=0;
CRGB *leds;
  int dmaBufferCount=2; //we use two buffers
    typedef union {
        uint8_t bytes[16];
        uint32_t shorts[8];
        uint32_t raw[2];
    } Lines;
  volatile  int num_strips;
  volatile  int nun_led_per_strip;
    int *Pins;
    int brigthness;
    int ledType;

    DMABufferClockI2SAPA102 * allocateDMABuffer(int bytes)
        {
            DMABufferClockI2SAPA102 * b = (DMABufferClockI2SAPA102 *)heap_caps_malloc(sizeof(DMABufferClockI2SAPA102), MALLOC_CAP_DMA);
            
            b->buffer = (uint8_t *)heap_caps_malloc(bytes, MALLOC_CAP_DMA);
            if(!b->buffer)
            {
                Serial.println("en,nnojnoo");
            }
            memset(b->buffer, 0, bytes);
            
            b->descriptor.length = bytes;
            b->descriptor.size = bytes;
            b->descriptor.owner = 1;
            b->descriptor.sosf = 1;
            b->descriptor.buf = b->buffer;
            b->descriptor.offset = 0;
            b->descriptor.empty = 0;
            b->descriptor.eof = 1;
            b->descriptor.qe.stqe_next = 0;
            
            return b;
        }    /// hardware index [0, 1]
    I2SAPA102(const int i2sIndex = 0);
void setBrightness(uint8_t b)
    {
        fillbuffer((uint16_t*)dmaBuffers[0]->buffer,b);
        fillbuffer((uint16_t*)dmaBuffers[1]->buffer,b);
    }

 void initled(CRGB *leds,int * Pins,int num_strips,int nun_led_per_strip,uint8_t clockMHz=4,int ledType=1)
    {
        //initialization of the pins
        cA=clockMHz;
        cN=(uint8_t)80/cA;
        cB=80%clockMHz;
        
        dmaBufferCount=2;
        this->leds=leds;
        this->nun_led_per_strip=nun_led_per_strip;
        this->num_strips=num_strips;
        this->Pins=Pins;
        this->brigthness=2;
        this->runningPixel=false;
        this->ledType=ledType;
        int pinMap[24];
        for(int i=0;i<24;i++)
        {
            if(i>=this->num_strips)
            {
                pinMap[i]=-1;
            }
            else
            {
                if(this->Pins[i]>=0 && this->Pins[i]<=33)
                    pinMap[i]=this->Pins[i]; //we could add remove 6,7,8,9,10,11,12
                else
                    pinMap[i]=-1;
            }
        }
        
        this->dmaBufferCount=dmaBufferCount;
        /*
        this->dmaBuffers = (DMABufferI2S **)malloc(sizeof(DMABufferI2S *) * (dmaBufferCount+1));
        if(!this->dmaBuffers)
        {
            Serial.println("Not enough memory soory...");
            return;
        }*/
        this->initParallelOutputMode(pinMap);
        /*
        for (int i = 0; i < this->dmaBufferCount; i++)
        {
            //this->dmaBuffers[i] = DMABuffer::allocate(96); //we need 24 bit * 4 pulses per bit
            this->dmaBuffers[i] = DMABufferI2S::allocate(32*2); //we need 24 bit * 4 pulses per bit
            if (i)
                this->dmaBuffers[i - 1]->next(this->dmaBuffers[i]);
            empty((uint16_t*)this->dmaBuffers[i]->buffer); //we do get the buffer prefilled with the 0 at the end and the 1
        }
        this->dmaBuffers[dmaBufferCount - 1]->next(this->dmaBuffers[0]);
        dmaBuffers[2] = DMABufferI2S::allocate(32*2);
          //ep( this->dmaBuffers[2],0);
          dmaBuffers[2]->next(dmaBuffers[0]);
         */
                dmaBuffers[0] = allocateDMABuffer(32*2);
                dmaBuffers[1] = allocateDMABuffer(32*2);
                dmaBuffers[2] = allocateDMABuffer(32*2);
                // dmaBuffers[3] = allocateDMABuffer(64*2);
                uint8_t d=nun_led_per_strip/2;
                //if(d%2==1)
                  //  d++;
                dmaBuffers[3] = allocateDMABuffer((64*2));
                memset(dmaBuffers[3]->buffer,0xff,(32*2));
        dmaBuffers[0]->descriptor.qe.stqe_next = &(dmaBuffers[1]->descriptor);
        dmaBuffers[1]->descriptor.qe.stqe_next = &(dmaBuffers[0]->descriptor);
        dmaBuffers[2]->descriptor.qe.stqe_next = &(dmaBuffers[0]->descriptor);
        // dmaBuffers[3]->descriptor.qe.stqe_next = &(dmaBuffers[0]->descriptor);
        empty((uint16_t*)dmaBuffers[0]->buffer);
        empty((uint16_t*)dmaBuffers[1]->buffer);
       //empty((uint16_t*)dmaBuffers[3]->buffer);
       //empty((uint16_t*)(dmaBuffers[3]->buffer+64);
       //  empty((uint16_t*)dmaBuffers[2]->buffer);
        Serial.println("Controller ready");
    }

    void empty ( uint16_t *buf)
     {
         for(int i=0;i<24;i++)
         {
             //4pulses
             //buf[4*i]=0xffffffff;
             //buf[4*i+3]=0;
            
             
             //3 pulses
             //buf[3*i]=0xffffffff;
             //buf[3*i+2]=0;
             
         }
         
         //APA102
         for(int i=0;i<8;i++)
         
         {
             buf[i]=0xffff;
         }
     }
     
     void fillbuffer(uint16_t *buf,uint8_t val)
     {
            buf[0]=0xffff;
            buf[1]=0xffff;
            buf[2]=(val & 0b10000) ? 0xffff : 0x0;
            buf[3]=0xffff;
            buf[4]=(val & 0b100) ? 0xffff : 0x0;
            buf[5]=(val & 0b1000) ? 0xffff : 0x0;
            buf[6]=(val & 0b1) ? 0xffff : 0x0;
            buf[7]=(val & 0b10) ? 0xffff : 0x0;
            

                     
     }
    
    void  transpose24x1_noinline(unsigned char *A, uint32_t *B) {
      uint32_t  x, y, x1,y1,t,x2,y2;
      
      y = *(unsigned int*)(A);
      x = *(unsigned int*)(A+4);
      y1 = *(unsigned int*)(A+8);
      x1 = *(unsigned int*)(A+12);
      
     // y2 = *(unsigned int*)(A+16);
      //x2 = *(unsigned int*)(A+20);
      
      
      // pre-transform x
      t = (x ^ (x >> 7)) & 0x00AA00AA;  x = x ^ t ^ (t << 7);
      t = (x ^ (x >>14)) & 0x0000CCCC;  x = x ^ t ^ (t <<14);
      
      t = (x1 ^ (x1 >> 7)) & 0x00AA00AA;  x1 = x1 ^ t ^ (t << 7);
      t = (x1 ^ (x1 >>14)) & 0x0000CCCC;  x1 = x1 ^ t ^ (t <<14);
      
      //t = (x2 ^ (x2 >> 7)) & 0x00AA00AA;  x2 = x2 ^ t ^ (t << 7);
      //t = (x2 ^ (x2 >>14)) & 0x0000CCCC;  x2 = x2 ^ t ^ (t <<14);
      
      // pre-transform y
      t = (y ^ (y >> 7)) & 0x00AA00AA;  y = y ^ t ^ (t << 7);
      t = (y ^ (y >>14)) & 0x0000CCCC;  y = y ^ t ^ (t <<14);
      
      t = (y1 ^ (y1 >> 7)) & 0x00AA00AA;  y1 = y1 ^ t ^ (t << 7);
      t = (y1 ^ (y1 >>14)) & 0x0000CCCC;  y1 = y1 ^ t ^ (t <<14);
      
      //t = (y2 ^ (y2 >> 7)) & 0x00AA00AA;  y2 = y2 ^ t ^ (t << 7);
      //t = (y2 ^ (y2 >>14)) & 0x0000CCCC;  y2 = y2 ^ t ^ (t <<14);
      
      // final transform
      t = (x & 0xF0F0F0F0) | ((y >> 4) & 0x0F0F0F0F);
      y = ((x << 4) & 0xF0F0F0F0) | (y & 0x0F0F0F0F);
      x = t;
      
      t = (x1 & 0xF0F0F0F0) | ((y1 >> 4) & 0x0F0F0F0F);
      y1 = ((x1 << 4) & 0xF0F0F0F0) | (y1 & 0x0F0F0F0F);
      x1 = t;
      
      //t = (x2 & 0xF0F0F0F0) | ((y2 >> 4) & 0x0F0F0F0F);
      //y2 = ((x2 << 4) & 0xF0F0F0F0) | (y2 & 0x0F0F0F0F);
      //x2 = t;
      
      
      
       *((uint32_t*)(B+7)) = (uint32_t)((y & 0xff) |  (  (y1 & 0xff) << 8 ) ); // |  (  (y2 & 0xff) << 16 ) )   ;
       *((uint32_t*)(B+6)) = (uint32_t)(((y & 0xff00) |((y1&0xff00) <<8))); //  |((y2&0xff00) <<16)  )>>8    );
      *((uint32_t*)(B+5)) =(uint32_t)(  (  (y & 0xff0000) >>16)|((y1&0xff0000) >>8));  // |((y2&0xff0000))     );
        *((uint32_t*)(B+4)) = (uint32_t)(((y & 0xff000000) >>16 |((y1&0xff000000)>>8 )));   // |((y2&0xff000000) )  )>>8);
      
       *((uint32_t*)B+3) =(uint32_t)( (x & 0xff) |((x1&0xff) <<8));  //|((x2&0xff) <<16) );
        *((uint32_t*)(B+2)) = (uint32_t)(((x & 0xff00) |((x1&0xff00) <<8)));//    |((x2&0xff00) <<16)    )>>8);
       *((uint32_t*)(B+1)) = (uint32_t)(  (  (x & 0xff0000) >>16)|((x1&0xff0000) >>8)  );// |((x2&0xff0000))     );
          *((uint32_t*)(B)) = (uint32_t)(((x & 0xff000000) >>16 |((x1&0xff000000)>>8 ) ) );//  |((x2&0xff000000) )    )>>8);
      
  }
  
    static    void transpose16x1_noinline2(unsigned char *A, uint8_t *B) {
           uint32_t  x, y, x1,y1,t;
           
           
           
           y = *(unsigned int*)(A);
           x = *(unsigned int*)(A+4);
           y1 = *(unsigned int*)(A+8);
           //x1=0;
           x1 = *(unsigned int*)(A+12);
           
           
           
           
           // pre-transform x
           t = (x ^ (x >> 7)) & AA;  x = x ^ t ^ (t << 7);
           t = (x ^ (x >>14)) & CC;  x = x ^ t ^ (t <<14);
           t = (x1 ^ (x1 >> 7)) & AA;  x1 = x1 ^ t ^ (t << 7);
           t = (x1 ^ (x1 >>14)) & CC;  x1 = x1 ^ t ^ (t <<14);
           // pre-transform y
           t = (y ^ (y >> 7)) & AA;  y = y ^ t ^ (t << 7);
           t = (y ^ (y >>14)) & CC;  y = y ^ t ^ (t <<14);
           t = (y1 ^ (y1 >> 7)) & AA;  y1 = y1 ^ t ^ (t << 7);
           t = (y1 ^ (y1 >>14)) & CC;  y1 = y1 ^ t ^ (t <<14);
           
           
           // final transform
           t = (x & FF) | ((y >> 4) & FF2);
           y = ((x << 4) & FF) | (y & FF2);
           x = t;
           
           t= (x1 & FF) | ((y1 >> 4) & FF2);
           y1 = ((x1 << 4) & FF) | (y1 & FF2);
           x1 = t;
           
           
           /*
            *((uint16_t*)B) = (uint16_t)((y & 0xff) |  (  (y1 & 0xff) << 8 ) )   ;
            B-=offset;
            //B-=offset;
            *((uint16_t*)(B)) = (uint16_t)(((y & 0xff00) |((y1&0xff00) <<8))>>8);
            B-=offset;
            *((uint16_t*)(B)) = (uint16_t)(((y & 0xff0000) |((y1&0xff0000) <<8))>>16);
            B-=offset;
            *((uint16_t*)(B)) = (uint16_t)(((y & 0xff000000) >>8 |((y1&0xff000000) ))>>16);
            B-=offset;
            *((uint16_t*)B) =(uint16_t)( (x & 0xff) |((x1&0xff) <<8));
            B-=offset;
            *((uint16_t*)(B)) = (uint16_t)(((x & 0xff00) |((x1&0xff00) <<8))>>8);
            B-=offset;
            *((uint16_t*)(B)) = (uint16_t)(((x & 0xff0000) |((x1&0xff0000) <<8))>>16);
            B-=offset;
            *((uint16_t*)(B)) = (uint16_t)(((x & 0xff000000) >>8 |((x1&0xff000000) ))>>16);
            */
           
           // *((uint16_t*)(B)) = (uint16_t)(   (   (x & 0xff000000) >>24 |  (  (x1 & 0xff000000) >>16)   )  );
           // *((uint16_t*)(B+4*48)) = (uint16_t)(((x & 0xff000000) >>8 |((x1&0xff000000) ))>>16);
           *((uint16_t*)(B+2)) = (uint16_t)(((x & 0xff000000) >>8 |((x1&0xff000000) ))>>16);
           //*((uint8_t*)(B))=*((uint8_t*)(&x)+3);
           //*((uint8_t*)(B+1))=*((uint8_t*)(&x1)+2);
           
           
           
           // B[0]= (uint16_t)(   (   (x & 0xff000000) >>24 |  (  (x1&0xff000000) >>16)   )  );
           //B+=48;//offset;
           *((uint16_t*)(B)) = (uint16_t)( ((x & 0xff0000) >>16|((x1&0xff0000) >>8)));
           // B[48] = (uint16_t)( ((x & 0xff0000) >>16|((x1&0xff0000) >>8)));
           //+=48;//offset;
           *((uint16_t*)(B+6)) = (uint16_t)(((x & 0xff00) |((x1&0xff00) <<8))>>8);
           // B[2*48]  = (uint16_t)(((x & 0xff00) |((x1&0xff00) <<8))>>8);
           // B+=48;//offset;
           *((uint16_t*)(B+4)) =(uint16_t)( (x & 0xff) |((x1&0xff) <<8));
           // B[3*48] =(uint16_t)( (x & 0xff) |((x1&0xff) <<8));
           //B+=48;//offset;
           
           
           *((uint16_t*)(B+10)) = (uint16_t)(((y & 0xff000000) >>8 |((y1&0xff000000) ))>>16);
           // B[4*48]= (uint16_t)(((y & 0xff000000) >>8 |((y1&0xff000000) ))>>16);
           
           *((uint16_t*)(B+8)) = (uint16_t)(((y & 0xff0000) |((y1&0xff0000) <<8))>>16);
           
           
           *((uint16_t*)(B+14)) = (uint16_t)(((y & 0xff00) |((y1&0xff00) <<8))>>8);
           //  *((uint16_t*)(B+6*48)) = (uint16_t)(( ((y & 0xff00)>>8) |((y1&0xff00) )));
           
           // B[6*48]=    (uint16_t)(((y & 0xff00) |((y1&0xff00) <<8))>>8);
           
           
           *((uint16_t*)(B+12)) = (uint16_t)((y & 0xff) |  (  (y1 & 0xff) << 8 ) )   ;
           //  B[7*48]  = (uint16_t)((y & 0xff) |  (  (y1 & 0xff) << 8 ) )   ;
           
       }
       
  
  void   putPixelinBuffer(Lines *pixel,uint16_t *buf)
  {
      Lines b2,b;
      //AP102
   /*  for (int i=0;i<8;i++)
      {
          buf[i]=0xffffffff;
      }
     */
      
      //transpose24x1_noinline(pixel[1].bytes,buf+16);
      //transpose24x1_noinline(pixel[2].bytes,buf+24);
      for (int color=0;color<3;color++)
      {
          
         // b=pixel[color];
         // b2=b;
          //transpose24x1_noinline(b.bytes,buf);
          transpose16x1_noinline2(pixel[color].bytes,(uint8_t*)(buf+8*(color+1)));
         
          /*for(int i=0;i<8;i++)
          {
              //buf[color*32+4*i+1]=(b2.shorts[7-i] << 8); //the <<8 is to free up the first byte
             // buf[color*32+4*i+2]=ledType*(b2.shorts[7-i] << 8);
               
              //3 pulses
              //buf[color*24+3*i+1]=(b2.shorts[7-i] << 8);
              
              //APA102
              buf[color*8+i+8]=(b2.shorts[7-i] << 8);
              
          }*/
      }
  }
    void showPixels() {
        
        oo=0;
        dmaBuffers[0]->descriptor.qe.stqe_next = &(dmaBuffers[1]->descriptor);
        dmaBuffers[1]->descriptor.qe.stqe_next = &(dmaBuffers[0]->descriptor);
       dmaBuffers[2]->descriptor.qe.stqe_next = &(dmaBuffers[0]->descriptor);
//         //dmaBuffers[3]->descriptor.qe.stqe_next = &(dmaBuffers[0]->descriptor);
       // empty((uint16_t*)dmaBuffers[0]->buffer);
    //    empty((uint16_t*)dmaBuffers[1]->buffer);
        //empty((uint16_t*)dmaBuffers[3]->buffer);
        //Serial.println(NUM_LEDS_PER_STRIP);
        //Serial.println("start");
       ledToDisplay=0;
       stopSignal=false;
       //pixelsToDisplay(allpixels);
       Lines firstPixel[3];
       Lines secondPixel[3];
        //Serial.println((uint32_t) & (dmaBuffers[2]->descriptor));
        //for ws28012
        /*
        
       for(int i = 0; i < num_strips; i++) {
           firstPixel[0].bytes[i] = leds[ledToDisplay+nun_led_per_strip*i].g/brigthness;
           firstPixel[1].bytes[i] = leds[ledToDisplay+nun_led_per_strip*i].r/brigthness;
           firstPixel[2].bytes[i] = leds[ledToDisplay+nun_led_per_strip*i].b/brigthness;
       }
       
       
       ledToDisplay++;
         
       putPixelinBuffer(firstPixel,(uint32_t*)dmaBuffers[0]->buffer);
         */
        //for APA102
       // ep((uint32_t*)this->dmaBuffers[0]->buffer,0);
      // t2=__clock_cycles();
       for(int i = 0; i < num_strips; i++) {
           
           secondPixel[0].bytes[i] = leds[ledToDisplay+nun_led_per_strip*i].b;
           secondPixel[1].bytes[i] = leds[ledToDisplay+nun_led_per_strip*i].g;
           secondPixel[2].bytes[i] = leds[ledToDisplay+nun_led_per_strip*i].r;
           
       }
        //Serial.println((uint32_t)int_leds);
       
      // ledToDisplay++;
       transpose16x1_noinline2(secondPixel[0].bytes,(uint8_t*)(dmaBuffers[0]->buffer+16*(0+1)));
        transpose16x1_noinline2(secondPixel[1].bytes,(uint8_t*)(dmaBuffers[0]->buffer+16*(1+1)));
        transpose16x1_noinline2(secondPixel[2].bytes,(uint8_t*)(dmaBuffers[0]->buffer+16*(2+1)));
      //putPixelinBuffer(secondPixel,(uint16_t*)dmaBuffers[0]->buffer);
       dmaBufferActive=1;
       ledToDisplay++;
       runningPixel=true;
        //Serial.println("staddd");
        
//        dmaBuffers[3] = allocateDMABuffer((nun_led_per_strip+1)*32*2);
//        ledToDisplay=0;
//        
//        uint8_t *buff=dmaBuffers[3]->buffer+64;
//        CRGB *poli=leds+ledToDisplay;
//        long time3=ESP.getCycleCount();
//        for(int j=0;j<nun_led_per_strip;j++)
//        {
//            CRGB *poli=leds+ledToDisplay;
//            for(int i = 0; i < num_strips; i++) {
//                
//                secondPixel[0].bytes[i] = (*poli).b;
//                secondPixel[1].bytes[i] = (*poli).g;
//                secondPixel[2].bytes[i] = (*poli).r;
//                poli+=nun_led_per_strip;
//                
//            }
//             //Serial.println((uint32_t)int_leds);
//            
//            ledToDisplay++;
//            empty((uint16_t*)buff);
//            transpose16x1_noinline2(secondPixel[0].bytes,(uint8_t*)(buff+16*(0+1)));
//             transpose16x1_noinline2(secondPixel[1].bytes,(uint8_t*)(buff+16*(1+1)));
//             transpose16x1_noinline2(secondPixel[2].bytes,(uint8_t*)(buff+16*(2+1)));
//            buff+=64;
//        }
//        long time2=ESP.getCycleCount()-time3;
//        Serial.printf("tiùs:%f us or %f \n",(float)time2/240,(float)time2/240/nun_led_per_strip); 

//long time4=ESP.getCycleCount();
       startTX();
        //Serial.println("starqsdqst");
       while(runningPixel==true);
       
// controller.showPixels();
 //long time2 = ESP.getCycleCount()-t1;
 //Serial.printf("fps:%f\n",(float)240000000/(__clock_cycles()-t2));
        //delay(0);
       //Serial.println("pixel done");
       
       
   }

//    void IRAM_ATTR  callback()
//    {
//       
//        Lines pixel[3];
//       
//        
//        //i2sStop();
//        //runningPixel=false;
//        //return;
//        
//        
//        if(stopSignal)
//        {
//            //delay(0);
//            i2sStop();
//            runningPixel=false;
//            return;
//        }
//        CRGB *poli=leds+ledToDisplay;
//        
//        if(ledToDisplay<=nun_led_per_strip)
//        {
//            
//            for(int i = 0; i <num_strips; i++) {
//                //Serial.println((uint32_t)int_leds);
//                pixel[0].bytes[i] = (*poli).b;
//                pixel[1].bytes[i] = (*poli).g;
//                pixel[2].bytes[i] = (*poli).r;
//                poli+=nun_led_per_strip;
//            }
//            ledToDisplay++;
//            
//            transpose16x1_noinline2(pixel[0].bytes,(uint8_t*)(dmaBuffers[dmaBufferActive]->buffer+16*(0+1)));
//            transpose16x1_noinline2(pixel[1].bytes,(uint8_t*)(dmaBuffers[dmaBufferActive]->buffer+16*(1+1)));
//            transpose16x1_noinline2(pixel[2].bytes,(uint8_t*)(dmaBuffers[dmaBufferActive]->buffer+16*(2+1)));
//            
//           // putPixelinBuffer(pixel,(uint16_t*)dmaBuffers[dmaBufferActive]->buffer);
//            dmaBufferActive = (dmaBufferActive + 1)% dmaBufferCount;
//        }
//        else
//        {
//            //if no more pixels then we will read the other buffer and stop
//           // if(ledToDisplay==nun_led_per_strip)
//             //   ledToDisplay++;
//            //if(ledToDisplay==nun_led_per_strip+1)
//                stopSignal=true;
//        }
//        
//    }
    
//static void IRAM_ATTR  callback( I2S *cont)
//    {
//       
//        Lines pixel[3];
//       //I2S *cont=(I2S *)arg;
//        
////        cont->i2sStop();
////        cont->runningPixel=false;
////        return;
//        
//        
//        if(cont->stopSignal)
//        {
//            //delay(0);
//            cont->i2sStop();
//            cont->runningPixel=false;
//            return;
//        }
//        
//        
//        if(cont->ledToDisplay<=cont->nun_led_per_strip)
//        {
//            CRGB *poli=cont->leds+cont->ledToDisplay;
//            for(int i = 0; i <cont->num_strips; i++) {
//                //Serial.println((uint32_t)int_leds);
//                pixel[0].bytes[i] = (*poli).b;
//                pixel[1].bytes[i] = (*poli).g;
//                pixel[2].bytes[i] = (*poli).r;
//                poli+=cont->nun_led_per_strip;
//            }
//            cont->ledToDisplay++;
//            
//            cont->transpose16x1_noinline2(pixel[0].bytes,(uint8_t*)(cont->dmaBuffers[cont->dmaBufferActive]->buffer+16*(0+1)));
//            cont->transpose16x1_noinline2(pixel[1].bytes,(uint8_t*)(cont->dmaBuffers[cont->dmaBufferActive]->buffer+16*(1+1)));
//            cont->transpose16x1_noinline2(pixel[2].bytes,(uint8_t*)(cont->dmaBuffers[cont->dmaBufferActive]->buffer+16*(2+1)));
//            
//           // putPixelinBuffer(pixel,(uint16_t*)dmaBuffers[dmaBufferActive]->buffer);
//            cont->dmaBufferActive = (cont->dmaBufferActive + 1)% cont->dmaBufferCount;
////             if(cont->ledToDisplay==cont->nun_led_per_strip)
////             {
////                cont->stopSignal=true;
////             }
//        }
//        else
//        {
//            //if no more pixels then we will read the other buffer and stop
//           // if(ledToDisplay==nun_led_per_strip)
//             //   ledToDisplay++;
//            //if(ledToDisplay==nun_led_per_strip+1)
//                cont->stopSignal=true;
//        }
//        
//    }
    
    

    void reset();

    void stop();

    void i2sStop();
    void startTX();
    void startRX();
  
    void resetDMA();
    void resetFIFO();
    bool initParallelOutputMode(const int *pinMap, long APLLFreq = 100000, int baseClock = -1, int wordSelect = -1);
    bool initParallelInputMode(const int *pinMap, long sampleRate = 100000, int baseClock = -1, int wordSelect = -1);

    void allocateDMABuffers(int count, int bytes);
    void deleteDMABuffers();
  
    
  protected:
    virtual void interrupt();
    
  private:
    static void IRAM_ATTR interrupt(void *arg);
};
