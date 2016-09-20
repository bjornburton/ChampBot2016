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

#define MAX_DUTYCYCLE 98 \
 \

#define WATCHDOG ON
#define ANALOG ON
#define TICK ON
#define CAPTURE ON \
 \

#define PIDSAMPCT 4
/*2:*/
#line 126 "./piruett.w"

/*7:*/
#line 162 "./piruett.w"

# include <avr/io.h>  
# include <avr/interrupt.h>  
# include <avr/sleep.h>  
# include <avr/wdt.h>  
# include <stdlib.h> 
# include <stdint.h> 
# include <assert.h> 
/*:7*/
#line 127 "./piruett.w"

/*8:*/
#line 182 "./piruett.w"

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


/*:8*//*9:*/
#line 199 "./piruett.w"

typedef enum{
CH1RISE,
CH1FALL,
CH2RISE,
CH2FALL,
ALLOWPRESSURE
}edges_t;


/*:9*//*10:*/
#line 210 "./piruett.w"

typedef enum{
IDLE,
REMOTE,
DIVING,
SUBMERGED
}controlModes_t;

/*:10*//*11:*/
#line 222 "./piruett.w"

typedef struct{
uint16_t ch1rise;
uint16_t ch1fall;
uint16_t ch2rise;
uint16_t ch2fall;
uint16_t ch1duration;
uint16_t ch2duration;
uint8_t stopped;
edges_t edge;
controlModes_t controlMode;
int16_t depth;
const uint16_t minIn;
const uint16_t maxIn;
ddcParameters*pPid_s;
}inputStruct;

/*:11*//*12:*/
#line 240 "./piruett.w"

typedef struct{
int16_t thrust;
int16_t radius;
const int16_t track;
int16_t starboardOut;
int16_t larboardOut;
const int8_t deadBand;
}transStruct;

/*:12*/
#line 128 "./piruett.w"

/*14:*/
#line 253 "./piruett.w"

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
int32_t scaler(int32_t input,int32_t minIn,int32_t maxIn,
int32_t minOut,int32_t maxOut);
int32_t int32clamp(int32_t value,int32_t min,int32_t max);
void takDdcSetPid(ddcParameters*,int16_t p,int16_t i,int16_t d,int16_t t);
int16_t takDdc(ddcParameters*);

/*:14*/
#line 129 "./piruett.w"

/*15:*/
#line 280 "./piruett.w"

void(*handleIrq)(inputStruct*)= NULL;




int main(void)
{


/*:15*/
#line 130 "./piruett.w"


/*:2*//*16:*/
#line 297 "./piruett.w"

inputStruct*pInput_s= &(inputStruct){
.edge= CH1RISE,
.controlMode= IDLE,
.pPid_s= &(ddcParameters){
.k_p= 1,
.k_i= 1,
.k_d= 1,
.t= 1,
.m= 0,
.setpoint= 100,
.mMin= -10000L,
.mMax= 10000L,
.mode= AUTOMATIC
}
};


takDdcSetPid(pInput_s->pPid_s,20,2,2,1);


/*:16*//*17:*/
#line 320 "./piruett.w"

cli();

/*70:*/
#line 1297 "./piruett.w"

{
wdt_reset();
MCUSR&= ~(1<<WDRF);
WDTCSR|= (1<<WDCE)|(1<<WDE);
WDTCSR= (1<<WDE)|(1<<WDP2)|(1<<WDP0);
WDTCSR|= (1<<WDIE);

}

/*:70*/
#line 323 "./piruett.w"

/*66:*/
#line 1221 "./piruett.w"

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

/*:66*/
#line 324 "./piruett.w"

/*63:*/
#line 1201 "./piruett.w"


DDRB&= ~(1<<DDD0);

PORTD|= (1<<PORTD0);


/*:63*/
#line 325 "./piruett.w"

/*62:*/
#line 1184 "./piruett.w"


DDRB|= (1<<DDB5);


DDRB|= (1<<DDB0);



DDRD|= ((1<<DDD5)|(1<<DDD6));


DDRD|= ((1<<DDD3)|(1<<DDD4));


DDRB|= (1<<DDD0);

/*:62*/
#line 326 "./piruett.w"

/*68:*/
#line 1281 "./piruett.w"

{
TCCR2B|= (1<<CS22)|(1<<CS21)|(1<<CS20);
TCCR2A|= (1<<WGM21);
OCR2A= 243U;
# if TICK
TIMSK2|= (1<<OCIE2A);
# endif
}


/*:68*/
#line 327 "./piruett.w"


/*:17*//*18:*/
#line 332 "./piruett.w"

sei();


/*:18*//*19:*/
#line 340 "./piruett.w"

/*72:*/
#line 1316 "./piruett.w"

{

TCCR0A|= (1<<WGM00);
TCCR0A|= (1<<COM0A1);
TCCR0A|= (1<<COM0B1);
TCCR0A|= (1<<COM0A0);
TCCR0A|= (1<<COM0B0);


TCCR0B|= (1<<CS01);
}


/*:72*/
#line 341 "./piruett.w"



/*:19*//*20:*/
#line 358 "./piruett.w"


/*64:*/
#line 1208 "./piruett.w"

{
SMCR&= ~((1<<SM2)|(1<<SM1)|(1<<SM0));
}

/*:64*/
#line 360 "./piruett.w"




/*:20*//*21:*/
#line 368 "./piruett.w"

edgeSelect(pInput_s);

/*:21*//*22:*/
#line 375 "./piruett.w"


for(;;)
{

/*:22*//*23:*/
#line 388 "./piruett.w"

#if WATCHDOG
WDTCSR|= (1<<WDIE);
#else
WDTCSR&= ~(1<<WDIE);
#endif

sleep_mode();

/*:23*//*24:*/
#line 404 "./piruett.w"


if(handleIrq!=NULL)
{
handleIrq(pInput_s);
handleIrq= NULL;
}



}



return 0;

}


/*:24*//*26:*/
#line 429 "./piruett.w"


ISR(TIMER1_CAPT_vect)
{
handleIrq= &pwcCalc;
}

/*:26*//*27:*/
#line 439 "./piruett.w"

ISR(TIMER2_COMPA_vect)
{
handleIrq= &diveTick;
}

/*:27*//*28:*/
#line 448 "./piruett.w"


ISR(ADC_vect)
{
handleIrq= &depthCalc;
}

/*:28*//*29:*/
#line 459 "./piruett.w"


ISR(WDT_vect)
{
handleIrq= &lostSignal;
}

/*:29*//*30:*/
#line 475 "./piruett.w"

void pwcCalc(inputStruct*pInput_s)
{
/*:30*//*31:*/
#line 488 "./piruett.w"


if(pInput_s->controlMode>=DIVING)return;

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
if(pInput_s->controlMode==IDLE)pInput_s->controlMode= REMOTE;
wdt_reset();
break;
case ALLOWPRESSURE:
pInput_s->edge= CH1RISE;
}

edgeSelect(pInput_s);


thrustCalc(pInput_s);

}

/*:31*//*32:*/
#line 530 "./piruett.w"

void thrustCalc(inputStruct*pInput_s)
{

/*:32*//*33:*/
#line 551 "./piruett.w"



const uint16_t minIn= 25990U;
const uint16_t maxIn= 41850U;
const int16_t minOut= INT8_MIN;
const int16_t maxOut= INT8_MAX;
/*:33*//*34:*/
#line 562 "./piruett.w"

transStruct*pTranslation_s= &(transStruct){
.deadBand= 10,
.track= 520
};

/*:34*//*35:*/
#line 570 "./piruett.w"


{
int16_t outputCh1;
int16_t outputCh2;

if(pInput_s->controlMode!=IDLE)
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


/*:35*//*36:*/
#line 606 "./piruett.w"


if(pTranslation_s->larboardOut||pTranslation_s->starboardOut)
{
pInput_s->stopped= FALSE;
#if 0
ledCntl(OFF);
#endif
}
else
{
pInput_s->stopped= TRUE;
#if 0
ledCntl(ON);
#endif
}



}

/*:36*//*37:*/
#line 630 "./piruett.w"

void lostSignal(inputStruct*pInput_s)
{
if(pInput_s->controlMode<=REMOTE)pInput_s->controlMode= IDLE;
if(pInput_s->controlMode==IDLE)setPwm(0,0);

pInput_s->edge= ALLOWPRESSURE;
wdt_reset();
}

/*:37*//*38:*/
#line 644 "./piruett.w"

void diveTick(inputStruct*pInput_s)
{
const uint8_t oneSecond= 64;
static uint8_t tickCount= 0;

const uint16_t divingSeconds= 20*oneSecond;
static uint16_t divingCount= divingSeconds;

const uint16_t submersedSeconds= 12*oneSecond;
static uint16_t submersedCount= submersedSeconds;


/*:38*//*39:*/
#line 663 "./piruett.w"


if(pInput_s->edge==ALLOWPRESSURE)edgeSelect(pInput_s);

/*:39*//*40:*/
#line 674 "./piruett.w"


if(!(tickCount+= oneSecond/4))
{
*pInput_s->pPid_s->pPvLast= pInput_s->depth;
takDdc(pInput_s->pPid_s);

if(pInput_s->controlMode>=DIVING)
{
int16_t output= scaler(-(pInput_s->pPid_s->m),
pInput_s->pPid_s->mMin,
pInput_s->pPid_s->mMax,
-(MAX_DUTYCYCLE*UINT8_MAX)/100L,
0L);

setPwm(output,output);

wdt_reset();

if(*pInput_s->pPid_s->pPvLast>=95L)
pInput_s->controlMode= SUBMERGED;
}
else
{
pInput_s->pPid_s->m= 0L;
}
}



if(pInput_s->stopped==TRUE)
{
const uint8_t debticks= oneSecond;
static uint8_t debcount= debticks;
if((PIND&(1<<PD0))&&debcount<debticks)debcount++;
if((~PIND&(1<<PD0))&&debcount> 0)debcount--;
if(!debcount)
pInput_s->controlMode= DIVING;
}

/*:40*//*41:*/
#line 716 "./piruett.w"

divingCount= (pInput_s->controlMode==DIVING)
?divingCount-1:divingSeconds;

if(divingCount==0)
pInput_s->controlMode= IDLE;

submersedCount= (pInput_s->controlMode==SUBMERGED)
?submersedCount-1:submersedSeconds;

if(submersedCount==0)
pInput_s->controlMode= IDLE;


#if 0
if(pInput_s->controlMode>=DIVING)
ledCntl(ON);
else
ledCntl(OFF);
#endif

}


/*:41*//*42:*/
#line 773 "./piruett.w"

void depthCalc(inputStruct*pInput_s)
{
const int16_t offset= 118;
const int16_t gain= 11;
static uint16_t buffStart[1<<4]= {0};
const uint16_t*buffEnd= buffStart+(1<<4)-1;
static uint16_t*buffIndex= buffStart;
static uint16_t sum= 0;

/*:42*//*43:*/
#line 785 "./piruett.w"

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

/*:43*//*44:*/
#line 809 "./piruett.w"

void edgeSelect(inputStruct*pInput_s)
{
switch(pInput_s->edge)
{
case ALLOWPRESSURE:
ADCSRA|= (1<<ADEN);
ADMUX= (ADMUX&0xf0)|2U;
return;
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
ADCSRA&= ~(1<<ADEN);
TCCR1B&= ~(1<<ICES1);
}
/*:44*//*45:*/
#line 838 "./piruett.w"


TIFR1|= (1<<ICF1);
}


/*:45*//*46:*/
#line 849 "./piruett.w"

int32_t scaler(int32_t input,
int32_t minIn,
int32_t maxIn,
int32_t minOut,
int32_t maxOut)
{
/*:46*//*47:*/
#line 859 "./piruett.w"


if(input> maxIn)
return maxOut;

if(input<minIn)
return minOut;


/*:47*//*48:*/
#line 879 "./piruett.w"

const int32_t ampFact= 128L;

int32_t gain= (ampFact*(maxIn-minIn))/(maxOut-minOut);

int32_t offset= ((ampFact*minIn)/gain)-minOut;

return(ampFact*input/gain)-offset;

}

/*:48*//*49:*/
#line 905 "./piruett.w"


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


/*:49*//*50:*/
#line 925 "./piruett.w"

difference= (speed*((ampFact*trans_s->radius)/UINT8_MAX))/ampFact;
rotation= (trans_s->track*((ampFact*difference)/UINT8_MAX))/ampFact;
piruett= trans_s->radius;

/*:50*//*51:*/
#line 941 "./piruett.w"

if(trans_s->thrust!=STOPPED&&lock==OFF)
{
trans_s->larboardOut= int32clamp(speed-rotation,-max,max);
trans_s->starboardOut= int32clamp(speed+rotation,-max,max);
}
else
{
lock= (abs(piruett)> pirLockLevel)?ON:OFF;

trans_s->larboardOut= int32clamp(piruett,-max,max);
/*:51*//*52:*/
#line 954 "./piruett.w"

piruett= -piruett;
trans_s->starboardOut= int32clamp(piruett,-max,max);
}
}



/*:52*//*53:*/
#line 967 "./piruett.w"

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

/*:53*//*54:*/
#line 995 "./piruett.w"

if(larboardOut||starboardOut)
relayCntl(CLOSED);
else
relayCntl(OPEN);

}

/*:54*//*55:*/
#line 1005 "./piruett.w"

void ledCntl(int8_t state)
{
PORTB= state?PORTB|(1<<PORTB5):PORTB&~(1<<PORTB5);
}

/*:55*//*56:*/
#line 1013 "./piruett.w"

void relayCntl(int8_t state)
{
PORTB= state?PORTB|(1<<PORTB0):PORTB&~(1<<PORTB0);
}

/*:56*//*57:*/
#line 1021 "./piruett.w"

void larboardDirection(int8_t state)
{
if(state)
PORTD&= ~(1<<PORTD3);
else
PORTD|= (1<<PORTD3);

}



/*:57*//*58:*/
#line 1035 "./piruett.w"

void starboardDirection(int8_t state)
{
if(state)
PORTD&= ~(1<<PORTD4);
else
PORTD|= (1<<PORTD4);
}

/*:58*//*59:*/
#line 1046 "./piruett.w"

int32_t int32clamp(int32_t value,int32_t min,int32_t max)
{
return(value> max)?max:(value<min)?min:value;
}

/*:59*//*60:*/
#line 1115 "./piruett.w"


int16_t takDdc(ddcParameters*pPar_s)
{
const int8_t derCoef[]= {2,-9,18,-11};


const int8_t secDerCoef[]= {2,-5,4,-1};


_Static_assert(sizeof(derCoef)/sizeof(derCoef[0])==PIDSAMPCT,
"PID sample mismatch");
_Static_assert(sizeof(secDerCoef)/sizeof(secDerCoef[0])==PIDSAMPCT,
"PID sample mismatch");

int32_t total;
uint8_t offset= pPar_s->pPvLast-pPar_s->pPvN;

pPar_s->pPvLast= pPar_s->pPvN+(++offset%PIDSAMPCT);




if(pPar_s->mode==AUTOMATIC)
{
int32_t dDer= 0,dSecDer= 0;

for(int8_t coefIdx= 0;coefIdx<PIDSAMPCT;coefIdx++)
{
dDer+= (int32_t)derCoef[coefIdx]*pPar_s->pPvN[offset%PIDSAMPCT];
dSecDer+= (int32_t)secDerCoef[coefIdx]*pPar_s->pPvN[offset%PIDSAMPCT];
offset++;
}


dDer/= 6;

int16_t err= pPar_s->setpoint-*pPar_s->pPvLast;

total= (int32_t)pPar_s->k_p*
(dDer+(int32_t)pPar_s->k_i*err-(int32_t)pPar_s->k_d*dSecDer);

pPar_s->m= int32clamp((pPar_s->m+total),pPar_s->mMin,pPar_s->mMax);
}

return pPar_s->m;
}



/*:60*//*61:*/
#line 1171 "./piruett.w"

void takDdcSetPid(ddcParameters*pPar_s,int16_t p,int16_t i,int16_t d,
int16_t t)
{
pPar_s->t= t;
pPar_s->k_p= (int16_t)p;
pPar_s->k_i= (int16_t)i*pPar_s->t;
pPar_s->k_d= (int16_t)d*pPar_s->t;


pPar_s->pPvLast= pPar_s->pPvN;
}

/*:61*/
