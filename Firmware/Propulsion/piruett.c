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
#define AUTOMATIC 1
#define MANUAL 0
#define STOPPED 0 \

#define CH2RISE 0
#define CH2FALL 1
#define CH1FALL 2
#define MAX_DUTYCYCLE 98
#define OFF 0
#define REMOTE 1
#define DIVING 2
#define SUBMERGED 3
#define PIDSAMPCT 4 \

/*2:*/
#line 114 "./piruett.w"

/*6:*/
#line 150 "./piruett.w"

# include <avr/io.h>  
# include <avr/interrupt.h>  
# include <avr/sleep.h>  
# include <avr/wdt.h>  
# include <stdlib.h> 
# include <stdint.h> 
# include <assert.h> 
/*:6*/
#line 115 "./piruett.w"

/*7:*/
#line 169 "./piruett.w"

typedef struct{
int16_t k_p;
int16_t k_i;
int16_t k_d;
int16_t t;
int16_t setpoint;
int16_t pPvN[PIDSAMPCT];
int16_t*pPvLast;
int16_t m;
int16_t mMin;
int16_t mMax;
int8_t mode;
}ddcParameters;

/*:7*//*8:*/
#line 189 "./piruett.w"

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
ddcParameters*pPid_s;
}inputStruct;

/*:8*//*9:*/
#line 205 "./piruett.w"

typedef struct{
int16_t thrust;
int16_t radius;
int16_t track;
int16_t starboardOut;
int16_t larboardOut;
const int8_t deadBand;
}transStruct;

/*:9*//*10:*/
#line 217 "./piruett.w"

typedef struct{
uint16_t diveTime;
uint16_t submergeTime;
int16_t starboardOut;
int16_t larboardOut;
}diveStruct;


/*:10*/
#line 116 "./piruett.w"

/*11:*/
#line 226 "./piruett.w"

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
void takDdcSetPid(ddcParameters*,int16_t p,int16_t i,int16_t d,int16_t t);
int16_t takDdc(ddcParameters*);

/*:11*/
#line 117 "./piruett.w"

/*12:*/
#line 252 "./piruett.w"

void(*handleIrq)(inputStruct*)= NULL;



int main(void)
{

/*:12*/
#line 118 "./piruett.w"


/*:2*//*13:*/
#line 278 "./piruett.w"



const uint16_t minIn= 14970;
const uint16_t maxIn= 27530;
const int16_t minOut= INT16_MIN;
const int16_t maxOut= INT16_MAX;

/*:13*//*14:*/
#line 291 "./piruett.w"

inputStruct*pInput_s= &(inputStruct){
.edge= CH2RISE,
.controlMode= OFF,
.pPid_s= &(ddcParameters){
.k_p= 1,
.k_i= 1,
.k_d= 1,
.t= 1,
.m= 0,
.mMin= INT16_MIN,
.mMax= INT16_MAX,
.mode= AUTOMATIC
}
};


/*:14*//*15:*/
#line 310 "./piruett.w"

transStruct*pTranslation_s= &(transStruct){
.deadBand= 10
};


/*:15*//*16:*/
#line 318 "./piruett.w"

cli();

/*58:*/
#line 1005 "./piruett.w"

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

/*:58*/
#line 321 "./piruett.w"

/*60:*/
#line 1060 "./piruett.w"

{
TCCR2B|= (1<<CS22)|(1<<CS21)|(1<<CS20);
TCCR2A|= (1<<WGM21);
OCR2A= 243U;
TIMSK2|= (1<<OCIE2A);
}


/*:60*/
#line 322 "./piruett.w"

/*55:*/
#line 978 "./piruett.w"


DDRB|= (1<<DDB5);


DDRB|= (1<<DDB0);



DDRD|= ((1<<DDD5)|(1<<DDD6));


DDRD|= ((1<<DDD3)|(1<<DDD4));

/*:55*/
#line 323 "./piruett.w"

/*62:*/
#line 1075 "./piruett.w"

{

WDTCSR|= (1<<WDCE)|(1<<WDE);
WDTCSR= (1<<WDIE)|(1<<WDP2)|(1<<WDP0);

}

/*:62*/
#line 324 "./piruett.w"


/*:16*//*17:*/
#line 329 "./piruett.w"

sei();


/*:17*//*18:*/
#line 337 "./piruett.w"

/*64:*/
#line 1091 "./piruett.w"

{

TCCR0A|= (1<<WGM00);
TCCR0A|= (1<<COM0A1);
TCCR0A|= (1<<COM0B1);
TCCR0A|= (1<<COM0A0);
TCCR0A|= (1<<COM0B0);


TCCR0B|= (1<<CS01);
}


/*:64*/
#line 338 "./piruett.w"



/*:18*//*19:*/
#line 355 "./piruett.w"


/*56:*/
#line 992 "./piruett.w"

{
SMCR&= ~((1<<SM2)|(1<<SM1)|(1<<SM0));
}

/*:56*/
#line 357 "./piruett.w"



ledCntl(OFF);

/*:19*//*20:*/
#line 366 "./piruett.w"

edgeSelect(pInput_s);

/*:20*//*21:*/
#line 373 "./piruett.w"



for(;;)
{

/*:21*//*22:*/
#line 385 "./piruett.w"


sleep_mode();

/*:22*//*23:*/
#line 398 "./piruett.w"

if(handleIrq!=NULL)
{
handleIrq(pInput_s);
handleIrq= NULL;
}

/*:23*//*24:*/
#line 407 "./piruett.w"


{
int16_t outputCh1;
int16_t outputCh2;

if(pInput_s->controlMode!=OFF)
{
outputCh1= scaler(pInput_s->ch1duration,minIn,maxIn,minOut,maxOut);
outputCh2= scaler(pInput_s->ch2duration,minIn,maxIn,minOut,maxOut);
}
else
{
outputCh1= 0;
outputCh2= 0;
}

outputCh1= (abs(outputCh1)> pTranslation_s->deadBand)?outputCh1:0;
outputCh2= (abs(outputCh2)> pTranslation_s->deadBand)?outputCh2:0;

pTranslation_s->radius= outputCh1;
pTranslation_s->thrust= outputCh2;

pTranslation_s->track= 100;
}

translate(pTranslation_s);

if(pInput_s->controlMode==REMOTE)
setPwm(pTranslation_s->larboardOut,pTranslation_s->starboardOut);
else
setPwm(pTranslation_s->larboardOut,pTranslation_s->starboardOut);

/*:24*//*25:*/
#line 442 "./piruett.w"

if(pTranslation_s->larboardOut||pTranslation_s->starboardOut)
ledCntl(OFF);
else
ledCntl(ON);


}



return 0;

}


/*:25*//*27:*/
#line 464 "./piruett.w"


ISR(TIMER1_CAPT_vect)
{
handleIrq= &pwcCalc;
}

/*:27*//*28:*/
#line 474 "./piruett.w"


ISR(TIMER2_COMPA_vect)
{
handleIrq= &diveTick;
}

/*:28*//*29:*/
#line 484 "./piruett.w"


ISR(ADC_vect)
{
handleIrq= &pressureCalc;
}


/*:29*//*30:*/
#line 496 "./piruett.w"

ISR(WDT_vect)
{
handleIrq= &lostSignal;
}

/*:30*//*31:*/
#line 511 "./piruett.w"

void pwcCalc(inputStruct*pInput_s)
{
/*:31*//*32:*/
#line 521 "./piruett.w"



switch(pInput_s->edge)
{
case CH2RISE:
pInput_s->ch2rise= ICR1;
pInput_s->edge= CH2FALL;
break;
case CH2FALL:
pInput_s->ch2fall= ICR1;
pInput_s->ch2duration= pInput_s->ch2fall-pInput_s->ch2rise;
pInput_s->edge= CH1FALL;
break;
case CH1FALL:
pInput_s->ch1fall= ICR1;
pInput_s->ch1duration= pInput_s->ch1fall-pInput_s->ch2fall;
pInput_s->edge= CH2RISE;
if(pInput_s->controlMode==OFF)pInput_s->controlMode= REMOTE;
}

edgeSelect(pInput_s);
}

/*:32*//*33:*/
#line 547 "./piruett.w"

void lostSignal(inputStruct*pInput_s)
{
pInput_s->controlMode= OFF;
pInput_s->edge= CH2RISE;

edgeSelect(pInput_s);
}

/*:33*//*34:*/
#line 560 "./piruett.w"

void diveTick(inputStruct*pInput_s)
{
static uint8_t tickCount= 0;



if(pInput_s->edge==CH2RISE)
{
ADCSRA|= (1<<ADEN);
ADMUX= (ADMUX&0xf0)|2U;
}

if(!(++tickCount))
{
if(pInput_s->controlMode>=DIVING)
{

takDdc(pInput_s->pPid_s);
}

wdt_reset();
}

}


/*:34*//*35:*/
#line 597 "./piruett.w"

void pressureCalc(inputStruct*pInput_s)
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

pInput_s->pressure= (sum>>5);

}


/*:35*//*36:*/
#line 621 "./piruett.w"

void edgeSelect(inputStruct*pInput_s)
{

switch(pInput_s->edge)
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
/*:36*//*37:*/
#line 642 "./piruett.w"


TIFR1|= (1<<ICF1);
}


/*:37*//*39:*/
#line 654 "./piruett.w"

int16_t scaler(uint16_t input,
uint16_t minIn,
uint16_t maxIn,
int16_t minOut,
int16_t maxOut)
{
/*:39*//*40:*/
#line 664 "./piruett.w"


if(input> maxIn)
return maxOut;

if(input<minIn)
return minOut;


/*:40*//*41:*/
#line 684 "./piruett.w"

const int32_t ampFact= 128L;

int32_t gain= (ampFact*(int32_t)(maxIn-minIn))/(int32_t)(maxOut-minOut);

int32_t offset= ((ampFact*(int32_t)minIn)/gain)-(int32_t)minOut;

return(ampFact*(int32_t)input/gain)-offset;

}

/*:41*//*42:*/
#line 711 "./piruett.w"


void translate(transStruct*trans_s)
{
int16_t speed= trans_s->thrust;
int16_t rotation;
int16_t difference;
int16_t piruett;
static int8_t lock= OFF;
const int8_t pirLockLevel= 15;
const int16_t max= (MAX_DUTYCYCLE*UINT8_MAX)/100;
const int16_t ampFact= 128;


/*:42*//*43:*/
#line 730 "./piruett.w"

difference= (speed*((ampFact*trans_s->radius)/UINT8_MAX))/ampFact;
rotation= (trans_s->track*((ampFact*difference)/UINT8_MAX))/ampFact;
piruett= trans_s->radius;

/*:43*//*44:*/
#line 746 "./piruett.w"

if(trans_s->thrust!=STOPPED&&lock==OFF)
{
trans_s->larboardOut= int16clamp(speed-rotation,-max,max);
trans_s->starboardOut= int16clamp(speed+rotation,-max,max);
}
else
{
lock= (abs(piruett)> pirLockLevel)?ON:OFF;

trans_s->larboardOut= int16clamp(piruett,-max,max);
/*:44*//*45:*/
#line 759 "./piruett.w"

piruett= -piruett;
trans_s->starboardOut= int16clamp(piruett,-max,max);
}
}



/*:45*//*46:*/
#line 770 "./piruett.w"

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

/*:46*//*47:*/
#line 798 "./piruett.w"

if(larboardOut||starboardOut)
relayCntl(CLOSED);
else
relayCntl(OPEN);

}

/*:47*//*48:*/
#line 808 "./piruett.w"

void ledCntl(int8_t state)
{
PORTB= state?PORTB|(1<<PORTB5):PORTB&~(1<<PORTB5);
}

/*:48*//*49:*/
#line 816 "./piruett.w"

void relayCntl(int8_t state)
{
PORTB= state?PORTB|(1<<PORTB0):PORTB&~(1<<PORTB0);
}

/*:49*//*50:*/
#line 824 "./piruett.w"

void larboardDirection(int8_t state)
{
if(state)
PORTD&= ~(1<<PORTD3);
else
PORTD|= (1<<PORTD3);

}



/*:50*//*51:*/
#line 838 "./piruett.w"

void starboardDirection(int8_t state)
{
if(state)
PORTD&= ~(1<<PORTD4);
else
PORTD|= (1<<PORTD4);
}

/*:51*//*52:*/
#line 849 "./piruett.w"

int16_t int16clamp(int16_t value,int16_t min,int16_t max)
{
return(value> max)?max:(value<min)?min:value;
}

/*:52*//*53:*/
#line 916 "./piruett.w"


int16_t takDdc(ddcParameters*pPar_s)
{
const int8_t derCoef[]= {2,-9,18,-11};


const int8_t secDerCoef[]= {2,-5,4,-1};


_Static_assert(sizeof(derCoef)/sizeof(derCoef[0])==PIDSAMPCT,
"PID sample mismatch");
_Static_assert(sizeof(secDerCoef)/sizeof(secDerCoef[0])==PIDSAMPCT,
"PID sample mismatch");

uint8_t offset= pPar_s->pPvLast-pPar_s->pPvN;

pPar_s->pPvLast= pPar_s->pPvN+(++offset%PIDSAMPCT);




if(pPar_s->mode==AUTOMATIC)
{
int16_t dDer= 0,dSecDer= 0;

for(int8_t coIdx= 0;coIdx<PIDSAMPCT;coIdx++)
{
dDer+= derCoef[coIdx]*pPar_s->pPvN[offset%PIDSAMPCT];
dSecDer+= secDerCoef[coIdx]*pPar_s->pPvN[offset%PIDSAMPCT];
offset++;
}
dDer/= 6;


int16_t err= pPar_s->setpoint-*pPar_s->pPvLast;

pPar_s->m+= pPar_s->k_p*(dDer+pPar_s->k_i*err-pPar_s->k_d*dSecDer);

pPar_s->m= int16clamp(pPar_s->m,pPar_s->mMin,pPar_s->mMax);
}


return pPar_s->m;
}

/*:53*//*54:*/
#line 965 "./piruett.w"

void takDdcSetPid(ddcParameters*pPar_s,int16_t p,int16_t i,int16_t d,
int16_t t)
{
pPar_s->t= t;
pPar_s->k_p= (int16_t)p;
pPar_s->k_i= (int16_t)i/pPar_s->t;
pPar_s->k_d= (int16_t)d/pPar_s->t;


pPar_s->pPvLast= pPar_s->pPvN;
}

/*:54*/