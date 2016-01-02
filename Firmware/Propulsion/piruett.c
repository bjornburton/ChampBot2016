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
#define MAX_DUTYCYCLE 98 \
 \

/*2:*/
#line 113 "./piruett.w"

/*6:*/
#line 142 "./piruett.w"

# include <avr/io.h>  
# include <avr/interrupt.h>  
# include <avr/sleep.h>  
# include <avr/wdt.h>  
# include <stdlib.h> 
# include <stdint.h> 

/*:6*/
#line 114 "./piruett.w"

/*7:*/
#line 154 "./piruett.w"

typedef struct{
uint16_t ch2rise;
uint16_t ch2fall;
uint16_t ch1fall;
uint16_t ch1duration;
uint16_t ch2duration;
uint8_t edge;
uint8_t lostSignal;
const uint16_t minIn;
const uint16_t maxIn;
}inputStruct;

/*:7*//*8:*/
#line 168 "./piruett.w"

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


/*:8*/
#line 115 "./piruett.w"

/*9:*/
#line 181 "./piruett.w"

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

/*:9*/
#line 116 "./piruett.w"

/*10:*/
#line 203 "./piruett.w"

void(*handleIrq)(inputStruct*)= NULL;



int main(void)
{

/*:10*/
#line 117 "./piruett.w"


/*:2*//*11:*/
#line 229 "./piruett.w"


inputStruct input_s= {
.edge= CH2RISE,
.minIn= 14970,
.maxIn= 27530,
.lostSignal= TRUE
};


/*:11*//*12:*/
#line 242 "./piruett.w"

transStruct translation_s= {
.minOut= -255,
.maxOut= 255,
.deadBand= 10
};

/*:12*//*13:*/
#line 251 "./piruett.w"

cli();

/*50:*/
#line 731 "./piruett.w"

{

ADCSRA&= ~(1<<ADEN);


ADCSRB|= (1<<ACME);


DIDR0|= ((1<<AIN1D)|(1<<AIN0D));


ACSR|= (1<<ACBG);
ACSR|= (1<<ACIC);
ACSR|= (1<<ACIS1);


TIMSK1|= (1<<ICIE1);


TCCR1B|= (1<<ICNC1);
TCCR1B|= (1<<CS10);


ADMUX&= ~((1<<MUX2)|(1<<MUX1)|(1<<MUX0));
}

/*:50*/
#line 254 "./piruett.w"

/*52:*/
#line 781 "./piruett.w"

{
TCCR2B|= (1<<CS22)|(1<<CS21)|(1<<CS20);
TCCR2A|= (1<<WGM21);
OCR2A= 0xf3;
TIMSK2|= (1<<OCIE2A);
}


/*:52*/
#line 255 "./piruett.w"

/*47:*/
#line 708 "./piruett.w"


DDRB|= (1<<DDB5);



DDRB|= (1<<DDB0);




DDRD|= ((1<<DDD5)|(1<<DDD6));


DDRD|= ((1<<DDD3)|(1<<DDD4));

/*:47*/
#line 256 "./piruett.w"

/*54:*/
#line 796 "./piruett.w"

{

WDTCSR|= (1<<WDCE)|(1<<WDE);
WDTCSR= (1<<WDIE)|(1<<WDP2)|(1<<WDP0);

}

/*:54*/
#line 257 "./piruett.w"


/*:13*//*14:*/
#line 262 "./piruett.w"

sei();


/*:14*//*15:*/
#line 270 "./piruett.w"

/*56:*/
#line 812 "./piruett.w"

{

TCCR0A|= (1<<WGM00);
TCCR0A|= (1<<COM0A1);
TCCR0A|= (1<<COM0B1);
TCCR0A|= (1<<COM0A0);
TCCR0A|= (1<<COM0B0);


TCCR0B|= (1<<CS01);
}

/*:56*/
#line 271 "./piruett.w"



/*:15*//*16:*/
#line 285 "./piruett.w"


/*48:*/
#line 724 "./piruett.w"

{
SMCR&= ~((1<<SM2)|(1<<SM1)|(1<<SM0));
}

/*:48*/
#line 287 "./piruett.w"



ledCntl(OFF);

/*:16*//*17:*/
#line 296 "./piruett.w"

edgeSelect(&input_s);

/*:17*//*18:*/
#line 303 "./piruett.w"



for(;;)
{

/*:18*//*19:*/
#line 315 "./piruett.w"

setPwm(&translation_s);

sleep_mode();

/*:19*//*20:*/
#line 329 "./piruett.w"

if(handleIrq!=NULL)
{
handleIrq(&input_s);
handleIrq= NULL;
}


translation_s.radius= scaler(&input_s,&translation_s,input_s.ch1duration);
translation_s.thrust= scaler(&input_s,&translation_s,input_s.ch2duration);
translation_s.track= 100;

translate(&translation_s);

/*:20*//*21:*/
#line 345 "./piruett.w"

if(translation_s.larboardOut||translation_s.starboardOut)
ledCntl(OFF);
else
ledCntl(ON);


}



return 0;

}


/*:21*//*23:*/
#line 368 "./piruett.w"


ISR(TIMER1_CAPT_vect)
{
handleIrq= &pwcCalc;
}

/*:23*//*24:*/
#line 378 "./piruett.w"


ISR(TIMER2_COMPA_vect)
{
handleIrq= &diveTick;
}


/*:24*//*25:*/
#line 390 "./piruett.w"

ISR(WDT_vect)
{
handleIrq= &lostSignal;
}

/*:25*//*26:*/
#line 405 "./piruett.w"

void pwcCalc(inputStruct*input_s)
{
/*:26*//*27:*/
#line 415 "./piruett.w"



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
input_s->lostSignal= FALSE;
wdt_reset();
}

edgeSelect(input_s);
}

/*:27*//*28:*/
#line 442 "./piruett.w"

void lostSignal(inputStruct*input_s)
{
input_s->lostSignal= TRUE;
input_s->edge= CH2RISE;

edgeSelect(input_s);
}

/*:28*//*29:*/
#line 456 "./piruett.w"

void diveTick(inputStruct*input_s)
{
static uint8_t tickCount= 0;

if(!(++tickCount&~0x80))
{

}

}


/*:29*//*30:*/
#line 473 "./piruett.w"

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
/*:30*//*31:*/
#line 494 "./piruett.w"


TIFR1|= (1<<ICF1);
}


/*:31*//*33:*/
#line 506 "./piruett.w"

int16_t scaler(inputStruct*input_s,transStruct*trans_s,uint16_t input)
{
uint16_t solution;
/*:33*//*34:*/
#line 515 "./piruett.w"

if(input_s->lostSignal==TRUE)
return 0;

if(input> input_s->maxIn)
return trans_s->maxOut;

if(input<input_s->minIn)
return trans_s->minOut;


/*:34*//*35:*/
#line 537 "./piruett.w"

const int32_t ampFact= 128L;

int32_t gain= (ampFact*(int32_t)(input_s->maxIn-input_s->minIn))/
(int32_t)(trans_s->maxOut-trans_s->minOut);

int32_t offset= ((ampFact*(int32_t)input_s->minIn)/gain)
-(int32_t)trans_s->minOut;

solution= (ampFact*(int32_t)input/gain)-offset;


return(abs(solution)> trans_s->deadBand)?solution:0;

}

/*:35*//*36:*/
#line 568 "./piruett.w"


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


/*:36*//*37:*/
#line 587 "./piruett.w"

difference= (speed*((ampFact*trans_s->radius)/UINT8_MAX))/ampFact;
rotation= (trans_s->track*((ampFact*difference)/UINT8_MAX))/ampFact;
piruett= trans_s->radius;

/*:37*//*38:*/
#line 603 "./piruett.w"

if(trans_s->thrust!=STOPPED&&lock==OFF)
{
trans_s->larboardOut= int16clamp(speed-rotation,-max,max);
trans_s->starboardOut= int16clamp(speed+rotation,-max,max);
}
else
{
lock= (abs(piruett)> PirLockLevel)?ON:OFF;

trans_s->larboardOut= int16clamp(piruett,-max,max);
/*:38*//*39:*/
#line 616 "./piruett.w"

piruett= -piruett;
trans_s->starboardOut= int16clamp(piruett,-max,max);
}
}

/*:39*//*40:*/
#line 625 "./piruett.w"

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

/*:40*//*41:*/
#line 653 "./piruett.w"

if(trans_s->larboardOut||trans_s->starboardOut)
relayCntl(CLOSED);
else
relayCntl(OPEN);

}

/*:41*//*42:*/
#line 663 "./piruett.w"

void ledCntl(int8_t state)
{
PORTB= state?PORTB|(1<<PORTB5):PORTB&~(1<<PORTB5);
}

/*:42*//*43:*/
#line 671 "./piruett.w"

void relayCntl(int8_t state)
{
PORTB= state?PORTB|(1<<PORTB0):PORTB&~(1<<PORTB0);
}

/*:43*//*44:*/
#line 679 "./piruett.w"

void larboardDirection(int8_t state)
{
if(state)
PORTD&= ~(1<<PORTD3);
else
PORTD|= (1<<PORTD3);

}

/*:44*//*45:*/
#line 691 "./piruett.w"

void starboardDirection(int8_t state)
{
if(state)
PORTD&= ~(1<<PORTD4);
else
PORTD|= (1<<PORTD4);
}

/*:45*//*46:*/
#line 702 "./piruett.w"

int16_t int16clamp(int16_t value,int16_t min,int16_t max)
{
return(value> max)?max:(value<min)?min:value;
}

/*:46*/
