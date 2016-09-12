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

#define CH1RISE 0
#define CH1FALL 1
#define CH2RISE 2
#define CH2FALL 3
#define ALLOWPRESSURE 4 \
 \

#define MAX_DUTYCYCLE 98 \

#define OFF 0
#define REMOTE 1
#define DIVING 2
#define SUBMERGED 3
#define PIDSAMPCT 4 \
 \

#define WATCHDOG ON
#define ANALOG ON
#define TICK ON
#define CAPTURE ON \
 \
 \

/*2:*/
#line 123 "./piruett.w"

/*9:*/
#line 175 "./piruett.w"

# include <avr/io.h>  
# include <avr/interrupt.h>  
# include <avr/sleep.h>  
# include <avr/wdt.h>  
# include <stdlib.h> 
# include <stdint.h> 
# include <assert.h> 
/*:9*/
#line 124 "./piruett.w"

/*10:*/
#line 194 "./piruett.w"

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

/*:10*//*11:*/
#line 214 "./piruett.w"

typedef struct{
uint16_t ch1rise;
uint16_t ch1fall;
uint16_t ch2rise;
uint16_t ch2fall;
uint16_t ch1duration;
uint16_t ch2duration;
uint8_t edge;
uint8_t controlMode;
int16_t depth;
const uint16_t minIn;
const uint16_t maxIn;
ddcParameters*pPid_s;
}inputStruct;

/*:11*//*12:*/
#line 231 "./piruett.w"

typedef struct{
int16_t thrust;
int16_t radius;
const int16_t track;
int16_t starboardOut;
int16_t larboardOut;
const int8_t deadBand;
}transStruct;

/*:12*//*13:*/
#line 243 "./piruett.w"

typedef struct{
uint16_t diveTime;
uint16_t submergeTime;
int16_t starboardOut;
int16_t larboardOut;
}diveStruct;


/*:13*/
#line 125 "./piruett.w"

/*14:*/
#line 252 "./piruett.w"

void relayCntl(int8_t state);
void ledCntl(int8_t state);
void larboardDirection(int8_t state);
void starboardDirection(int8_t state);
void depthCalc(inputStruct*);
void diveTick(inputStruct*);
void pwcCalc(inputStruct*);
void edgeSelect(inputStruct*);
void translate(transStruct*);
void setPwm(int16_t,int16_t);
void lostSignal(inputStruct*);
void thrustCalc(inputStruct*);
int16_t scaler(uint16_t input,uint16_t minIn,uint16_t maxIn,
int16_t minOut,int16_t maxOut);
int16_t int16clamp(int16_t value,int16_t min,int16_t max);
void takDdcSetPid(ddcParameters*,int16_t p,int16_t i,int16_t d,int16_t t);
int16_t takDdc(ddcParameters*);

/*:14*/
#line 126 "./piruett.w"

/*15:*/
#line 279 "./piruett.w"

void(*handleIrq)(inputStruct*)= NULL;




int main(void)
{


/*:15*/
#line 127 "./piruett.w"


/*:2*//*16:*/
#line 295 "./piruett.w"

inputStruct*pInput_s= &(inputStruct){
.edge= CH1RISE,
.controlMode= OFF,
.pPid_s= &(ddcParameters){
.k_p= 1,
.k_i= 1,
.k_d= 1,
.t= 1,
.m= 0,
.setpoint= 100,
.mMin= INT16_MIN,
.mMax= INT16_MAX,
.mode= AUTOMATIC
}
};



/*:16*//*17:*/
#line 316 "./piruett.w"

cli();

/*65:*/
#line 1184 "./piruett.w"

{
wdt_reset();
MCUSR&= ~(1<<WDRF);
WDTCSR|= (1<<WDCE)|(1<<WDE);
WDTCSR= (1<<WDE)|(1<<WDP2)|(1<<WDP0);
WDTCSR|= (1<<WDIE);

}

/*:65*/
#line 319 "./piruett.w"

/*61:*/
#line 1106 "./piruett.w"

{

ADCSRA&= ~(1<<ADEN);
ADCSRA|= ((1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0));

#if ANALOG
ADCSRA|= (1<<ADIE);
#endif


ADCSRB|= (1<<ACME);


DIDR0|= ((1<<ADC2D)|(1<<ADC1D)|(1<<ADC0D));


ACSR|= (1<<ACBG);
ACSR|= (1<<ACIC);
ACSR|= (1<<ACIS1);


#if CAPTURE
TIMSK1|= (1<<ICIE1);
#endif


TCCR1B|= (1<<ICNC1);
TCCR1B|= (1<<CS10);


ADMUX= (ADMUX&0xf0)|0U;
ADMUX|= (1<<REFS0)|(1<<REFS1);

}

/*:61*/
#line 320 "./piruett.w"

/*58:*/
#line 1079 "./piruett.w"


DDRB|= (1<<DDB5);


DDRB|= (1<<DDB0);



DDRD|= ((1<<DDD5)|(1<<DDD6));


DDRD|= ((1<<DDD3)|(1<<DDD4));

/*:58*/
#line 321 "./piruett.w"

/*63:*/
#line 1167 "./piruett.w"

{
TCCR2B|= (1<<CS22)|(1<<CS21)|(1<<CS20);
TCCR2A|= (1<<WGM21);
OCR2A= 243U;
# if TICK
TIMSK2|= (1<<OCIE2A);
# endif
}


/*:63*/
#line 322 "./piruett.w"


/*:17*//*18:*/
#line 327 "./piruett.w"

sei();


/*:18*//*19:*/
#line 335 "./piruett.w"

/*67:*/
#line 1203 "./piruett.w"

{

TCCR0A|= (1<<WGM00);
TCCR0A|= (1<<COM0A1);
TCCR0A|= (1<<COM0B1);
TCCR0A|= (1<<COM0A0);
TCCR0A|= (1<<COM0B0);


TCCR0B|= (1<<CS01);
}


/*:67*/
#line 336 "./piruett.w"



/*:19*//*20:*/
#line 353 "./piruett.w"


/*59:*/
#line 1093 "./piruett.w"

{
SMCR&= ~((1<<SM2)|(1<<SM1)|(1<<SM0));
}

/*:59*/
#line 355 "./piruett.w"




/*:20*//*21:*/
#line 363 "./piruett.w"

edgeSelect(pInput_s);

/*:21*//*22:*/
#line 370 "./piruett.w"


for(;;)
{

/*:22*//*23:*/
#line 383 "./piruett.w"

#if WATCHDOG
WDTCSR|= (1<<WDIE);
#else
WDTCSR&= ~(1<<WDIE);
#endif

sleep_mode();

/*:23*//*24:*/
#line 399 "./piruett.w"


if(handleIrq!=NULL)
{
handleIrq(pInput_s);
handleIrq= NULL;
}



}



return 0;

}


/*:24*//*26:*/
#line 424 "./piruett.w"


ISR(TIMER1_CAPT_vect)
{
handleIrq= &pwcCalc;
}

/*:26*//*27:*/
#line 434 "./piruett.w"

ISR(TIMER2_COMPA_vect)
{
handleIrq= &diveTick;
}

/*:27*//*28:*/
#line 443 "./piruett.w"


ISR(ADC_vect)
{
handleIrq= &depthCalc;
}

/*:28*//*29:*/
#line 454 "./piruett.w"


ISR(WDT_vect)
{
handleIrq= &lostSignal;
}

/*:29*//*30:*/
#line 470 "./piruett.w"

void pwcCalc(inputStruct*pInput_s)
{
/*:30*//*31:*/
#line 483 "./piruett.w"



switch(pInput_s->edge)

{
case CH1RISE:
pInput_s->ch1rise= ICR1;
pInput_s->edge= CH1FALL;
wdt_reset();
break;
case CH1FALL:
pInput_s->ch1fall= ICR1;
pInput_s->ch1duration= pInput_s->ch1fall-pInput_s->ch1rise;
pInput_s->edge= CH2RISE;
wdt_reset();
break;
case CH2RISE:
pInput_s->ch2rise= ICR1;
pInput_s->edge= CH2FALL;
wdt_reset();
break;
case CH2FALL:
pInput_s->ch2fall= ICR1;
pInput_s->ch2duration= pInput_s->ch2fall-pInput_s->ch2rise;
pInput_s->edge= ALLOWPRESSURE;
if(pInput_s->controlMode==OFF)pInput_s->controlMode= REMOTE;
wdt_reset();
break;
case ALLOWPRESSURE:
pInput_s->edge= CH1RISE;
}
edgeSelect(pInput_s);

thrustCalc(pInput_s);

}

void thrustCalc(inputStruct*pInput_s)
{

/*:31*//*32:*/
#line 541 "./piruett.w"



const uint16_t minIn= 25990U;
const uint16_t maxIn= 41850U;
const int16_t minOut= INT8_MIN;
const int16_t maxOut= INT8_MAX;
/*:32*//*33:*/
#line 552 "./piruett.w"

transStruct*pTranslation_s= &(transStruct){
.deadBand= 10,
.track= 520
};

/*:33*//*34:*/
#line 560 "./piruett.w"


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

}

translate(pTranslation_s);

if(pInput_s->controlMode==REMOTE)
setPwm(pTranslation_s->larboardOut,pTranslation_s->starboardOut);
else
setPwm(pTranslation_s->larboardOut,pTranslation_s->starboardOut);

/*:34*//*35:*/
#line 595 "./piruett.w"


#if 0
if(pTranslation_s->larboardOut||pTranslation_s->starboardOut)
ledCntl(OFF);
else
ledCntl(ON);
#endif

}

/*:35*//*36:*/
#line 609 "./piruett.w"

void lostSignal(inputStruct*pInput_s)
{

pInput_s->controlMode= OFF;
pInput_s->edge= ALLOWPRESSURE;
wdt_reset();
}

/*:36*//*37:*/
#line 627 "./piruett.w"

void diveTick(inputStruct*pInput_s)
{
static uint8_t tickCount= 0;


if(pInput_s->edge==ALLOWPRESSURE)
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

}

}


/*:37*//*38:*/
#line 686 "./piruett.w"

void depthCalc(inputStruct*pInput_s)
{
const int16_t offset= 118;
const int16_t gain= 11;
static uint16_t buffStart[1<<5]= {0};
const uint16_t*buffEnd= buffStart+(1<<5)-1;
static uint16_t*buffIndex= buffStart;
static uint16_t sum= 0;

ADCSRA&= ~(1<<ADEN);
ADMUX= (ADMUX&0xf0)|1U;

sum-= *buffIndex;
*buffIndex= (uint16_t)ADCW;
sum+= *buffIndex;
buffIndex= (buffIndex!=buffEnd)?buffIndex+1:buffStart;

pInput_s->depth= (int16_t)(((sum>>5)-offset)*gain)/32;

#if 1 
if(pInput_s->depth> 100)
ledCntl(OFF);
else
ledCntl(ON);
#endif
}

/*:38*//*39:*/
#line 718 "./piruett.w"

void edgeSelect(inputStruct*pInput_s)
{
switch(pInput_s->edge)
{
case CH1RISE:
ADMUX= (ADMUX&0xf0)|0U;
TCCR1B|= (1<<ICES1);
break;
case CH1FALL:
ADMUX= (ADMUX&0xf0)|0U;
TCCR1B&= ~(1<<ICES1);
break;
case CH2RISE:
ADMUX= (ADMUX&0xf0)|1U;
TCCR1B|= (1<<ICES1);
break;
case CH2FALL:
ADMUX= (ADMUX&0xf0)|1U;
TCCR1B&= ~(1<<ICES1);
}
/*:39*//*40:*/
#line 742 "./piruett.w"


TIFR1|= (1<<ICF1);
}


/*:40*//*42:*/
#line 755 "./piruett.w"

int16_t scaler(uint16_t input,
uint16_t minIn,
uint16_t maxIn,
int16_t minOut,
int16_t maxOut)
{
/*:42*//*43:*/
#line 765 "./piruett.w"


if(input> maxIn)
return maxOut;

if(input<minIn)
return minOut;


/*:43*//*44:*/
#line 785 "./piruett.w"

const int32_t ampFact= 128L;

int32_t gain= (ampFact*(int32_t)(maxIn-minIn))/(int32_t)(maxOut-minOut);

int32_t offset= ((ampFact*(int32_t)minIn)/gain)-(int32_t)minOut;

return(ampFact*(int32_t)input/gain)-offset;

}

/*:44*//*45:*/
#line 811 "./piruett.w"


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


/*:45*//*46:*/
#line 831 "./piruett.w"

difference= (speed*((ampFact*trans_s->radius)/UINT8_MAX))/ampFact;
rotation= (trans_s->track*((ampFact*difference)/UINT8_MAX))/ampFact;
piruett= trans_s->radius;

/*:46*//*47:*/
#line 847 "./piruett.w"

if(trans_s->thrust!=STOPPED&&lock==OFF)
{
trans_s->larboardOut= int16clamp(speed-rotation,-max,max);
trans_s->starboardOut= int16clamp(speed+rotation,-max,max);
}
else
{
lock= (abs(piruett)> pirLockLevel)?ON:OFF;

trans_s->larboardOut= int16clamp(piruett,-max,max);
/*:47*//*48:*/
#line 860 "./piruett.w"

piruett= -piruett;
trans_s->starboardOut= int16clamp(piruett,-max,max);
}
}



/*:48*//*49:*/
#line 871 "./piruett.w"

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

/*:49*//*50:*/
#line 899 "./piruett.w"

if(larboardOut||starboardOut)
relayCntl(CLOSED);
else
relayCntl(OPEN);

}

/*:50*//*51:*/
#line 909 "./piruett.w"

void ledCntl(int8_t state)
{
PORTB= state?PORTB|(1<<PORTB5):PORTB&~(1<<PORTB5);
}

/*:51*//*52:*/
#line 917 "./piruett.w"

void relayCntl(int8_t state)
{
PORTB= state?PORTB|(1<<PORTB0):PORTB&~(1<<PORTB0);
}

/*:52*//*53:*/
#line 925 "./piruett.w"

void larboardDirection(int8_t state)
{
if(state)
PORTD&= ~(1<<PORTD3);
else
PORTD|= (1<<PORTD3);

}



/*:53*//*54:*/
#line 939 "./piruett.w"

void starboardDirection(int8_t state)
{
if(state)
PORTD&= ~(1<<PORTD4);
else
PORTD|= (1<<PORTD4);
}

/*:54*//*55:*/
#line 950 "./piruett.w"

int16_t int16clamp(int16_t value,int16_t min,int16_t max)
{
return(value> max)?max:(value<min)?min:value;
}

/*:55*//*56:*/
#line 1017 "./piruett.w"


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

/*:56*//*57:*/
#line 1066 "./piruett.w"

void takDdcSetPid(ddcParameters*pPar_s,int16_t p,int16_t i,int16_t d,
int16_t t)
{
pPar_s->t= t;
pPar_s->k_p= (int16_t)p;
pPar_s->k_i= (int16_t)i/pPar_s->t;
pPar_s->k_d= (int16_t)d/pPar_s->t;


pPar_s->pPvLast= pPar_s->pPvN;
}

/*:57*/
