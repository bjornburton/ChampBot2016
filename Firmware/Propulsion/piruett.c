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
const int16_t minOut;
const int16_t maxOut;
const int8_t deadBand;
}transStruct;

/*:8*//*9:*/
#line 186 "./piruett.w"

typedef struct{
uint16_t diveTime;
uint16_t submergeTime;
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
void setPwm(transStruct*);
void lostSignal(inputStruct*);
int16_t scaler(inputStruct*,transStruct*,uint16_t input);
int16_t int16clamp(int16_t value,int16_t min,int16_t max);

/*:10*/
#line 116 "./piruett.w"

/*11:*/
#line 216 "./piruett.w"

void(*handleIrq)(inputStruct*)= NULL;



int main(void)
{

/*:11*/
#line 117 "./piruett.w"


/*:2*//*12:*/
#line 242 "./piruett.w"


inputStruct input_s= {
.edge= CH2RISE,
.minIn= 14970,
.maxIn= 27530,
.controlMode= OFF
};


/*:12*//*13:*/
#line 255 "./piruett.w"

transStruct translation_s= {
.minOut= -255,
.maxOut= 255,
.deadBand= 10
};

/*:13*//*14:*/
#line 264 "./piruett.w"

cli();

/*53:*/
#line 809 "./piruett.w"

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

/*:53*/
#line 267 "./piruett.w"

/*55:*/
#line 864 "./piruett.w"

{
TCCR2B|= (1<<CS22)|(1<<CS21)|(1<<CS20);
TCCR2A|= (1<<WGM21);
OCR2A= 243U;
TIMSK2|= (1<<OCIE2A);
}


/*:55*/
#line 268 "./piruett.w"

/*50:*/
#line 780 "./piruett.w"


DDRB|= (1<<DDB5);



DDRB|= (1<<DDB0);




DDRD|= ((1<<DDD5)|(1<<DDD6));


DDRD|= ((1<<DDD3)|(1<<DDD4));

/*:50*/
#line 269 "./piruett.w"

/*57:*/
#line 879 "./piruett.w"

{

WDTCSR|= (1<<WDCE)|(1<<WDE);
WDTCSR= (1<<WDIE)|(1<<WDP2)|(1<<WDP0);

}

/*:57*/
#line 270 "./piruett.w"


/*:14*//*15:*/
#line 275 "./piruett.w"

sei();


/*:15*//*16:*/
#line 283 "./piruett.w"

/*59:*/
#line 895 "./piruett.w"

{

TCCR0A|= (1<<WGM00);
TCCR0A|= (1<<COM0A1);
TCCR0A|= (1<<COM0B1);
TCCR0A|= (1<<COM0A0);
TCCR0A|= (1<<COM0B0);


TCCR0B|= (1<<CS01);
}

/*:59*/
#line 284 "./piruett.w"



/*:16*//*17:*/
#line 301 "./piruett.w"


/*51:*/
#line 796 "./piruett.w"

{
SMCR&= ~((1<<SM2)|(1<<SM1)|(1<<SM0));
}

/*:51*/
#line 303 "./piruett.w"



ledCntl(OFF);

/*:17*//*18:*/
#line 312 "./piruett.w"

edgeSelect(&input_s);

/*:18*//*19:*/
#line 319 "./piruett.w"



for(;;)
{

/*:19*//*20:*/
#line 331 "./piruett.w"

setPwm(&translation_s);

sleep_mode();

/*:20*//*21:*/
#line 345 "./piruett.w"

if(handleIrq!=NULL)
{
handleIrq(&input_s);
handleIrq= NULL;
}


translation_s.radius= scaler(&input_s,&translation_s,input_s.ch1duration);
translation_s.thrust= scaler(&input_s,&translation_s,input_s.ch2duration);
translation_s.track= 100;

if(input_s.controlMode==REMOTE)
translate(&translation_s);

/*:21*//*22:*/
#line 362 "./piruett.w"

if(translation_s.larboardOut||translation_s.starboardOut)
ledCntl(OFF);
else
ledCntl(ON);


}



return 0;

}


/*:22*//*24:*/
#line 385 "./piruett.w"


ISR(TIMER1_CAPT_vect)
{
handleIrq= &pwcCalc;
}

/*:24*//*25:*/
#line 395 "./piruett.w"


ISR(TIMER2_COMPA_vect)
{
handleIrq= &diveTick;
}

/*:25*//*26:*/
#line 405 "./piruett.w"


ISR(ADC_vect)
{
handleIrq= &pressureCalc;
}


/*:26*//*27:*/
#line 417 "./piruett.w"

ISR(WDT_vect)
{
handleIrq= &lostSignal;
}

/*:27*//*28:*/
#line 432 "./piruett.w"

void pwcCalc(inputStruct*input_s)
{
/*:28*//*29:*/
#line 442 "./piruett.w"



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

/*:29*//*30:*/
#line 468 "./piruett.w"

void lostSignal(inputStruct*input_s)
{
input_s->controlMode= OFF;
input_s->edge= CH2RISE;

edgeSelect(input_s);
}

/*:30*//*31:*/
#line 481 "./piruett.w"

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


/*:31*//*32:*/
#line 519 "./piruett.w"

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


/*:32*//*33:*/
#line 543 "./piruett.w"

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
/*:33*//*34:*/
#line 564 "./piruett.w"


TIFR1|= (1<<ICF1);
}


/*:34*//*36:*/
#line 576 "./piruett.w"

int16_t scaler(inputStruct*input_s,transStruct*trans_s,uint16_t input)
{
uint16_t solution;
/*:36*//*37:*/
#line 585 "./piruett.w"

if(input_s->controlMode==OFF)
return 0;

if(input> input_s->maxIn)
return trans_s->maxOut;

if(input<input_s->minIn)
return trans_s->minOut;


/*:37*//*38:*/
#line 607 "./piruett.w"

const int32_t ampFact= 128L;

int32_t gain= (ampFact*(int32_t)(input_s->maxIn-input_s->minIn))/
(int32_t)(trans_s->maxOut-trans_s->minOut);

int32_t offset= ((ampFact*(int32_t)input_s->minIn)/gain)
-(int32_t)trans_s->minOut;

solution= (ampFact*(int32_t)input/gain)-offset;


return(abs(solution)> trans_s->deadBand)?solution:0;

}

/*:38*//*39:*/
#line 638 "./piruett.w"


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


/*:39*//*40:*/
#line 657 "./piruett.w"

difference= (speed*((ampFact*trans_s->radius)/UINT8_MAX))/ampFact;
rotation= (trans_s->track*((ampFact*difference)/UINT8_MAX))/ampFact;
piruett= trans_s->radius;

/*:40*//*41:*/
#line 673 "./piruett.w"

if(trans_s->thrust!=STOPPED&&lock==OFF)
{
trans_s->larboardOut= int16clamp(speed-rotation,-max,max);
trans_s->starboardOut= int16clamp(speed+rotation,-max,max);
}
else
{
lock= (abs(piruett)> PirLockLevel)?ON:OFF;

trans_s->larboardOut= int16clamp(piruett,-max,max);
/*:41*//*42:*/
#line 686 "./piruett.w"

piruett= -piruett;
trans_s->starboardOut= int16clamp(piruett,-max,max);
}
}



/*:42*//*43:*/
#line 697 "./piruett.w"

void setPwm(transStruct*trans_s)
{

if(trans_s->larboardOut>=0)
{
larboardDirection(FORWARD);
OCR0A= abs(trans_s->larboardOut);
}
else
{
larboardDirection(REVERSE);
OCR0A= abs(trans_s->larboardOut);
}

if(trans_s->starboardOut>=0)
{
starboardDirection(FORWARD);
OCR0B= abs(trans_s->starboardOut);
}
else
{
starboardDirection(REVERSE);
OCR0B= abs(trans_s->starboardOut);
}

/*:43*//*44:*/
#line 725 "./piruett.w"

if(trans_s->larboardOut||trans_s->starboardOut)
relayCntl(CLOSED);
else
relayCntl(OPEN);

}

/*:44*//*45:*/
#line 735 "./piruett.w"

void ledCntl(int8_t state)
{
PORTB= state?PORTB|(1<<PORTB5):PORTB&~(1<<PORTB5);
}

/*:45*//*46:*/
#line 743 "./piruett.w"

void relayCntl(int8_t state)
{
PORTB= state?PORTB|(1<<PORTB0):PORTB&~(1<<PORTB0);
}

/*:46*//*47:*/
#line 751 "./piruett.w"

void larboardDirection(int8_t state)
{
if(state)
PORTD&= ~(1<<PORTD3);
else
PORTD|= (1<<PORTD3);

}

/*:47*//*48:*/
#line 763 "./piruett.w"

void starboardDirection(int8_t state)
{
if(state)
PORTD&= ~(1<<PORTD4);
else
PORTD|= (1<<PORTD4);
}

/*:48*//*49:*/
#line 774 "./piruett.w"

int16_t int16clamp(int16_t value,int16_t min,int16_t max)
{
return(value> max)?max:(value<min)?min:value;
}

/*:49*/
