#define F_CPU 16000000UL \

#define ON 1
#define OFF 0
#define SET 1
#define CLEAR 0
#define TRUE 1
#define FALSE 0
#define FORWARD 1
#define REVERSE 0
#define CLOSED 1
#define OPEN 0
#define STOPPED 0 \

#define CH2RISE 0
#define CH2FALL 1
#define CH1FALL 2
#define MAX_DUTYCYCLE 98
#define OFF 0
#define REMOTE 1
#define DIVING 2
#define SUBMERGED 3 \

/*2:*/
#line 113 "./piruett.w"

/*6:*/
#line 145 "./piruett.w"

# include <avr/io.h>  
# include <avr/interrupt.h>  
# include <avr/sleep.h>  
# include <avr/wdt.h>  
# include <stdlib.h> 
# include <stdint.h> 

/*:6*/
#line 114 "./piruett.w"

/*7:*/
#line 157 "./piruett.w"

typedef struct{
uint16_t ch2rise;
uint16_t ch2fall;
uint16_t ch1fall;
uint16_t ch1duration;
uint16_t ch2duration;
uint8_t edge;
uint8_t controlMode;
uint16_t pressure;
const uint16_t minIn;
const uint16_t maxIn;
}inputStruct;

/*:7*//*8:*/
#line 172 "./piruett.w"

typedef struct{
int16_t thrust;
int16_t radius;
int16_t track;
int16_t starboardOut;
int16_t larboardOut;
const int8_t deadBand;
}transStruct;

/*:8*//*9:*/
#line 184 "./piruett.w"

typedef struct{
uint16_t diveTime;
uint16_t submergeTime;
int16_t starboardOut;
int16_t larboardOut;
}diveStruct;


/*:9*/
#line 115 "./piruett.w"

/*10:*/
#line 193 "./piruett.w"

void relayCntl(int8_t state);
void ledCntl(int8_t state);
void larboardDirection(int8_t state);
void starboardDirection(int8_t state);
void pressureCalc(inputStruct*);
void diveTick(inputStruct*);
void pwcCalc(inputStruct*);
void edgeSelect(inputStruct*);
void translate(transStruct*);
void setPwm(int16_t,int16_t);
void lostSignal(inputStruct*);
int16_t scaler(uint16_t input,uint16_t minIn,uint16_t maxIn,
int16_t minOut,int16_t maxOut);
int16_t int16clamp(int16_t value,int16_t min,int16_t max);

/*:10*/
#line 116 "./piruett.w"

/*11:*/
#line 217 "./piruett.w"

void(*handleIrq)(inputStruct*)= NULL;



int main(void)
{

/*:11*/
#line 117 "./piruett.w"


/*:2*//*12:*/
#line 243 "./piruett.w"



const uint16_t minIn= 14970;
const uint16_t maxIn= 27530;
const int16_t minOut= -255;
const int16_t maxOut= 255;

inputStruct input_s= {
.edge= CH2RISE,
.controlMode= OFF
};


/*:12*//*13:*/
#line 260 "./piruett.w"

transStruct translation_s= {
.deadBand= 10
};

/*:13*//*14:*/
#line 267 "./piruett.w"

cli();

/*54:*/
#line 835 "./piruett.w"

{

ADCSRA&= ~(1<<ADEN);
ADCSRA&= ~((1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0));
ADCSRA&= ~(1<<ADIE);


ADCSRB|= (1<<ACME);


DIDR0|= ((1<<ADC2D)|(1<<ADC1D)|(1<<ADC0D));


ACSR|= (1<<ACBG);
ACSR|= (1<<ACIC);
ACSR|= (1<<ACIS1);


TIMSK1|= (1<<ICIE1);


TCCR1B|= (1<<ICNC1);
TCCR1B|= (1<<CS10);


ADMUX= (ADMUX&0xf0)|0U;
ADMUX&= ~(1<<REFS0);

}

/*:54*/
#line 270 "./piruett.w"

/*56:*/
#line 890 "./piruett.w"

{
TCCR2B|= (1<<CS22)|(1<<CS21)|(1<<CS20);
TCCR2A|= (1<<WGM21);
OCR2A= 243U;
TIMSK2|= (1<<OCIE2A);
}


/*:56*/
#line 271 "./piruett.w"

/*51:*/
#line 806 "./piruett.w"


DDRB|= (1<<DDB5);



DDRB|= (1<<DDB0);




DDRD|= ((1<<DDD5)|(1<<DDD6));


DDRD|= ((1<<DDD3)|(1<<DDD4));

/*:51*/
#line 272 "./piruett.w"

/*58:*/
#line 905 "./piruett.w"

{

WDTCSR|= (1<<WDCE)|(1<<WDE);
WDTCSR= (1<<WDIE)|(1<<WDP2)|(1<<WDP0);

}

/*:58*/
#line 273 "./piruett.w"


/*:14*//*15:*/
#line 278 "./piruett.w"

sei();


/*:15*//*16:*/
#line 286 "./piruett.w"

/*60:*/
#line 921 "./piruett.w"

{

TCCR0A|= (1<<WGM00);
TCCR0A|= (1<<COM0A1);
TCCR0A|= (1<<COM0B1);
TCCR0A|= (1<<COM0A0);
TCCR0A|= (1<<COM0B0);


TCCR0B|= (1<<CS01);
}

/*:60*/
#line 287 "./piruett.w"



/*:16*//*17:*/
#line 304 "./piruett.w"


/*52:*/
#line 822 "./piruett.w"

{
SMCR&= ~((1<<SM2)|(1<<SM1)|(1<<SM0));
}

/*:52*/
#line 306 "./piruett.w"



ledCntl(OFF);

/*:17*//*18:*/
#line 315 "./piruett.w"

edgeSelect(&input_s);

/*:18*//*19:*/
#line 322 "./piruett.w"



for(;;)
{

/*:19*//*20:*/
#line 334 "./piruett.w"


sleep_mode();

/*:20*//*21:*/
#line 347 "./piruett.w"

if(handleIrq!=NULL)
{
handleIrq(&input_s);
handleIrq= NULL;
}

/*:21*//*22:*/
#line 357 "./piruett.w"


{
int16_t outputCh1;
int16_t outputCh2;

if(input_s.controlMode!=OFF)
{
outputCh1= scaler(input_s.ch1duration,minIn,maxIn,minOut,maxOut);
outputCh2= scaler(input_s.ch2duration,minIn,maxIn,minOut,maxOut);
}
else
{
outputCh1= 0;
outputCh2= 0;
}

outputCh1= (abs(outputCh1)> translation_s.deadBand)?outputCh1:0;
outputCh2= (abs(outputCh2)> translation_s.deadBand)?outputCh2:0;

translation_s.radius= outputCh1;
translation_s.thrust= outputCh2;

translation_s.track= 100;
}

translate(&translation_s);

if(input_s.controlMode==REMOTE)
setPwm(translation_s.larboardOut,translation_s.starboardOut);
else
setPwm(translation_s.larboardOut,translation_s.starboardOut);


/*:22*//*23:*/
#line 393 "./piruett.w"

if(translation_s.larboardOut||translation_s.starboardOut)
ledCntl(OFF);
else
ledCntl(ON);


}



return 0;

}


/*:23*//*25:*/
#line 416 "./piruett.w"


ISR(TIMER1_CAPT_vect)
{
handleIrq= &pwcCalc;
}

/*:25*//*26:*/
#line 426 "./piruett.w"


ISR(TIMER2_COMPA_vect)
{
handleIrq= &diveTick;
}

/*:26*//*27:*/
#line 436 "./piruett.w"


ISR(ADC_vect)
{
handleIrq= &pressureCalc;
}


/*:27*//*28:*/
#line 448 "./piruett.w"

ISR(WDT_vect)
{
handleIrq= &lostSignal;
}

/*:28*//*29:*/
#line 463 "./piruett.w"

void pwcCalc(inputStruct*input_s)
{
/*:29*//*30:*/
#line 473 "./piruett.w"



switch(input_s->edge)
{
case CH2RISE:
input_s->ch2rise= ICR1;
input_s->edge= CH2FALL;
break;
case CH2FALL:
input_s->ch2fall= ICR1;
input_s->ch2duration= input_s->ch2fall-input_s->ch2rise;
input_s->edge= CH1FALL;
break;
case CH1FALL:
input_s->ch1fall= ICR1;
input_s->ch1duration= input_s->ch1fall-input_s->ch2fall;
input_s->edge= CH2RISE;
if(input_s->controlMode==OFF)input_s->controlMode= REMOTE;
}

edgeSelect(input_s);
}

/*:30*//*31:*/
#line 499 "./piruett.w"

void lostSignal(inputStruct*input_s)
{
input_s->controlMode= OFF;
input_s->edge= CH2RISE;

edgeSelect(input_s);
}

/*:31*//*32:*/
#line 512 "./piruett.w"

void diveTick(inputStruct*input_s)
{
static uint8_t tickCount= 0;



if(input_s->edge==CH2RISE)
{
ADCSRA|= (1<<ADEN);
ADMUX= (ADMUX&0xf0)|2U;
}

if(!(++tickCount))
{
if(input_s->controlMode>=DIVING)




;

wdt_reset();
}

}


/*:32*//*33:*/
#line 550 "./piruett.w"

void pressureCalc(inputStruct*input_s)
{
static uint16_t buffStart[33];
const uint16_t*buffEnd= buffStart+33;
static uint16_t*buffIndex= buffStart;
static uint16_t sum;

ADCSRA&= ~(1<<ADEN);

*buffIndex= ADCL&((uint16_t)ADCH)<<8;
sum+= *buffIndex;
buffIndex= (buffIndex!=buffEnd)?buffIndex+1:buffStart;
sum-= *buffIndex;

input_s->pressure= (sum>>5);

}


/*:33*//*34:*/
#line 574 "./piruett.w"

void edgeSelect(inputStruct*input_s)
{

switch(input_s->edge)
{
case CH2RISE:
ADMUX= (ADMUX&0xf0)|1U;
TCCR1B|= (1<<ICES1);
break;
case CH2FALL:
ADMUX= (ADMUX&0xf0)|1U;
TCCR1B&= ~(1<<ICES1);
break;
case CH1FALL:
ADMUX= (ADMUX&0xf0)|0U;
TCCR1B&= ~(1<<ICES1);
}
/*:34*//*35:*/
#line 595 "./piruett.w"


TIFR1|= (1<<ICF1);
}


/*:35*//*37:*/
#line 607 "./piruett.w"

int16_t scaler(uint16_t input,
uint16_t minIn,
uint16_t maxIn,
int16_t minOut,
int16_t maxOut)
{
/*:37*//*38:*/
#line 617 "./piruett.w"


if(input> maxIn)
return maxOut;

if(input<minIn)
return minOut;


/*:38*//*39:*/
#line 636 "./piruett.w"

const int32_t ampFact= 128L;

int32_t gain= (ampFact*(int32_t)(maxIn-minIn))/
(int32_t)(maxOut-minOut);

int32_t offset= ((ampFact*(int32_t)minIn)/gain)
-(int32_t)minOut;

return(ampFact*(int32_t)input/gain)-offset;

}

/*:39*//*40:*/
#line 664 "./piruett.w"


void translate(transStruct*trans_s)
{
int16_t speed= trans_s->thrust;
int16_t rotation;
int16_t difference;
int16_t piruett;
static int8_t lock= OFF;
const int8_t PirLockLevel= 15;
const int16_t max= (MAX_DUTYCYCLE*UINT8_MAX)/100;
const int16_t ampFact= 128;


/*:40*//*41:*/
#line 683 "./piruett.w"

difference= (speed*((ampFact*trans_s->radius)/UINT8_MAX))/ampFact;
rotation= (trans_s->track*((ampFact*difference)/UINT8_MAX))/ampFact;
piruett= trans_s->radius;

/*:41*//*42:*/
#line 699 "./piruett.w"

if(trans_s->thrust!=STOPPED&&lock==OFF)
{
trans_s->larboardOut= int16clamp(speed-rotation,-max,max);
trans_s->starboardOut= int16clamp(speed+rotation,-max,max);
}
else
{
lock= (abs(piruett)> PirLockLevel)?ON:OFF;

trans_s->larboardOut= int16clamp(piruett,-max,max);
/*:42*//*43:*/
#line 712 "./piruett.w"

piruett= -piruett;
trans_s->starboardOut= int16clamp(piruett,-max,max);
}
}



/*:43*//*44:*/
#line 723 "./piruett.w"

void setPwm(int16_t larboardOut,int16_t starboardOut)
{

if(larboardOut>=0)
{
larboardDirection(FORWARD);
OCR0A= abs(larboardOut);
}
else
{
larboardDirection(REVERSE);
OCR0A= abs(larboardOut);
}

if(starboardOut>=0)
{
starboardDirection(FORWARD);
OCR0B= abs(starboardOut);
}
else
{
starboardDirection(REVERSE);
OCR0B= abs(starboardOut);
}

/*:44*//*45:*/
#line 751 "./piruett.w"

if(larboardOut||starboardOut)
relayCntl(CLOSED);
else
relayCntl(OPEN);

}

/*:45*//*46:*/
#line 761 "./piruett.w"

void ledCntl(int8_t state)
{
PORTB= state?PORTB|(1<<PORTB5):PORTB&~(1<<PORTB5);
}

/*:46*//*47:*/
#line 769 "./piruett.w"

void relayCntl(int8_t state)
{
PORTB= state?PORTB|(1<<PORTB0):PORTB&~(1<<PORTB0);
}

/*:47*//*48:*/
#line 777 "./piruett.w"

void larboardDirection(int8_t state)
{
if(state)
PORTD&= ~(1<<PORTD3);
else
PORTD|= (1<<PORTD3);

}

/*:48*//*49:*/
#line 789 "./piruett.w"

void starboardDirection(int8_t state)
{
if(state)
PORTD&= ~(1<<PORTD4);
else
PORTD|= (1<<PORTD4);
}

/*:49*//*50:*/
#line 800 "./piruett.w"

int16_t int16clamp(int16_t value,int16_t min,int16_t max)
{
return(value> max)?max:(value<min)?min:value;
}

/*:50*/
