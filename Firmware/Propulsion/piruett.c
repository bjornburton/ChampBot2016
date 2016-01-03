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
const uint16_t minIn;
const uint16_t maxIn;
}inputStruct;

/*:7*//*8:*/
#line 171 "./piruett.w"

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
#line 185 "./piruett.w"

typedef struct{
uint16_t diveTime;
uint16_t submergeTime;
}diveStruct;


/*:9*/
#line 115 "./piruett.w"

/*10:*/
#line 192 "./piruett.w"

void relayCntl(int8_t state);
void ledCntl(int8_t state);
void larboardDirection(int8_t state);
void starboardDirection(int8_t state);
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
#line 214 "./piruett.w"

void(*handleIrq)(inputStruct*)= NULL;



int main(void)
{

/*:11*/
#line 117 "./piruett.w"


/*:2*//*12:*/
#line 240 "./piruett.w"


inputStruct input_s= {
.edge= CH2RISE,
.minIn= 14970,
.maxIn= 27530,
.controlMode= OFF
};


/*:12*//*13:*/
#line 253 "./piruett.w"

transStruct translation_s= {
.minOut= -255,
.maxOut= 255,
.deadBand= 10
};

/*:13*//*14:*/
#line 262 "./piruett.w"

cli();

/*51:*/
#line 747 "./piruett.w"

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


ADMUX&= ~((1<<MUX2)|(1<<MUX1)|(1<<MUX0));
ADMUX&= ~(1<<REFS0);

}

/*:51*/
#line 265 "./piruett.w"

/*53:*/
#line 801 "./piruett.w"

{
TCCR2B|= (1<<CS22)|(1<<CS21)|(1<<CS20);
TCCR2A|= (1<<WGM21);
OCR2A= 243U;
TIMSK2|= (1<<OCIE2A);
}


/*:53*/
#line 266 "./piruett.w"

/*48:*/
#line 719 "./piruett.w"


DDRB|= (1<<DDB5);



DDRB|= (1<<DDB0);




DDRD|= ((1<<DDD5)|(1<<DDD6));


DDRD|= ((1<<DDD3)|(1<<DDD4));

/*:48*/
#line 267 "./piruett.w"

/*55:*/
#line 816 "./piruett.w"

{

WDTCSR|= (1<<WDCE)|(1<<WDE);
WDTCSR= (1<<WDIE)|(1<<WDP2)|(1<<WDP0);

}

/*:55*/
#line 268 "./piruett.w"


/*:14*//*15:*/
#line 273 "./piruett.w"

sei();


/*:15*//*16:*/
#line 281 "./piruett.w"

/*57:*/
#line 832 "./piruett.w"

{

TCCR0A|= (1<<WGM00);
TCCR0A|= (1<<COM0A1);
TCCR0A|= (1<<COM0B1);
TCCR0A|= (1<<COM0A0);
TCCR0A|= (1<<COM0B0);


TCCR0B|= (1<<CS01);
}

/*:57*/
#line 282 "./piruett.w"



/*:16*//*17:*/
#line 296 "./piruett.w"


/*49:*/
#line 735 "./piruett.w"

{
SMCR&= ~((1<<SM2)|(1<<SM1)|(1<<SM0));
}

/*:49*/
#line 298 "./piruett.w"



ledCntl(OFF);

/*:17*//*18:*/
#line 307 "./piruett.w"

edgeSelect(&input_s);

/*:18*//*19:*/
#line 314 "./piruett.w"



for(;;)
{

/*:19*//*20:*/
#line 326 "./piruett.w"

setPwm(&translation_s);

sleep_mode();

/*:20*//*21:*/
#line 340 "./piruett.w"

if(handleIrq!=NULL)
{
handleIrq(&input_s);
handleIrq= NULL;
}


translation_s.radius= scaler(&input_s,&translation_s,input_s.ch1duration);
translation_s.thrust= scaler(&input_s,&translation_s,input_s.ch2duration);
translation_s.track= 100;

translate(&translation_s);

/*:21*//*22:*/
#line 356 "./piruett.w"

if(translation_s.larboardOut||translation_s.starboardOut)
ledCntl(OFF);
else
ledCntl(ON);


}



return 0;

}


/*:22*//*24:*/
#line 379 "./piruett.w"


ISR(TIMER1_CAPT_vect)
{
handleIrq= &pwcCalc;
}

/*:24*//*25:*/
#line 389 "./piruett.w"


ISR(TIMER2_COMPA_vect)
{
handleIrq= &diveTick;
}


/*:25*//*26:*/
#line 401 "./piruett.w"

ISR(WDT_vect)
{
handleIrq= &lostSignal;
}

/*:26*//*27:*/
#line 416 "./piruett.w"

void pwcCalc(inputStruct*input_s)
{
/*:27*//*28:*/
#line 426 "./piruett.w"



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
wdt_reset();
}

edgeSelect(input_s);
}

/*:28*//*29:*/
#line 453 "./piruett.w"

void lostSignal(inputStruct*input_s)
{
input_s->controlMode= OFF;
input_s->edge= CH2RISE;

edgeSelect(input_s);
}

/*:29*//*30:*/
#line 467 "./piruett.w"

void diveTick(inputStruct*input_s)
{
static uint8_t tickCount= 0;

if(!(++tickCount&~128U))
{
wdt_reset();
}

}


/*:30*//*31:*/
#line 484 "./piruett.w"

void edgeSelect(inputStruct*input_s)
{

switch(input_s->edge)
{
case CH2RISE:
ADMUX|= (1<<MUX0);
TCCR1B|= (1<<ICES1);
break;
case CH2FALL:
ADMUX|= (1<<MUX0);
TCCR1B&= ~(1<<ICES1);
break;
case CH1FALL:
ADMUX&= ~(1<<MUX0);
TCCR1B&= ~(1<<ICES1);
}
/*:31*//*32:*/
#line 505 "./piruett.w"


TIFR1|= (1<<ICF1);
}


/*:32*//*34:*/
#line 517 "./piruett.w"

int16_t scaler(inputStruct*input_s,transStruct*trans_s,uint16_t input)
{
uint16_t solution;
/*:34*//*35:*/
#line 526 "./piruett.w"

if(input_s->controlMode==OFF)
return 0;

if(input> input_s->maxIn)
return trans_s->maxOut;

if(input<input_s->minIn)
return trans_s->minOut;


/*:35*//*36:*/
#line 548 "./piruett.w"

const int32_t ampFact= 128L;

int32_t gain= (ampFact*(int32_t)(input_s->maxIn-input_s->minIn))/
(int32_t)(trans_s->maxOut-trans_s->minOut);

int32_t offset= ((ampFact*(int32_t)input_s->minIn)/gain)
-(int32_t)trans_s->minOut;

solution= (ampFact*(int32_t)input/gain)-offset;


return(abs(solution)> trans_s->deadBand)?solution:0;

}

/*:36*//*37:*/
#line 579 "./piruett.w"


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


/*:37*//*38:*/
#line 598 "./piruett.w"

difference= (speed*((ampFact*trans_s->radius)/UINT8_MAX))/ampFact;
rotation= (trans_s->track*((ampFact*difference)/UINT8_MAX))/ampFact;
piruett= trans_s->radius;

/*:38*//*39:*/
#line 614 "./piruett.w"

if(trans_s->thrust!=STOPPED&&lock==OFF)
{
trans_s->larboardOut= int16clamp(speed-rotation,-max,max);
trans_s->starboardOut= int16clamp(speed+rotation,-max,max);
}
else
{
lock= (abs(piruett)> PirLockLevel)?ON:OFF;

trans_s->larboardOut= int16clamp(piruett,-max,max);
/*:39*//*40:*/
#line 627 "./piruett.w"

piruett= -piruett;
trans_s->starboardOut= int16clamp(piruett,-max,max);
}
}

/*:40*//*41:*/
#line 636 "./piruett.w"

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

/*:41*//*42:*/
#line 664 "./piruett.w"

if(trans_s->larboardOut||trans_s->starboardOut)
relayCntl(CLOSED);
else
relayCntl(OPEN);

}

/*:42*//*43:*/
#line 674 "./piruett.w"

void ledCntl(int8_t state)
{
PORTB= state?PORTB|(1<<PORTB5):PORTB&~(1<<PORTB5);
}

/*:43*//*44:*/
#line 682 "./piruett.w"

void relayCntl(int8_t state)
{
PORTB= state?PORTB|(1<<PORTB0):PORTB&~(1<<PORTB0);
}

/*:44*//*45:*/
#line 690 "./piruett.w"

void larboardDirection(int8_t state)
{
if(state)
PORTD&= ~(1<<PORTD3);
else
PORTD|= (1<<PORTD3);

}

/*:45*//*46:*/
#line 702 "./piruett.w"

void starboardDirection(int8_t state)
{
if(state)
PORTD&= ~(1<<PORTD4);
else
PORTD|= (1<<PORTD4);
}

/*:46*//*47:*/
#line 713 "./piruett.w"

int16_t int16clamp(int16_t value,int16_t min,int16_t max)
{
return(value> max)?max:(value<min)?min:value;
}

/*:47*/
