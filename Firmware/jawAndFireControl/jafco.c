#define F_CPU 8000000UL \
 \

#define ON 1
#define OFF 0
#define OPEN 1
#define CLOSE 0
#define SET 1
#define CLEAR 0 \

/*1:*/
#line 26 "./jafco.w"

/*4:*/
#line 44 "./jafco.w"

# include <avr/io.h>  
# include <util/delay.h>  
# include <avr/interrupt.h>  
# include <avr/sleep.h>  
# include <avr/wdt.h>  
# include <stdlib.h> 
# include <stdint.h> 


/*:4*/
#line 27 "./jafco.w"

/*5:*/
#line 54 "./jafco.w"

void jawCntl(uint8_t state);
void fuelCntl(uint8_t state);
void ignCntl(uint8_t state);
void releaseSeq(void);
void fireSeq(void);

/*:5*/
#line 28 "./jafco.w"

/*6:*/
#line 67 "./jafco.w"

void(*handleIrq)()= NULL;



int main(void)
{


/*28:*/
#line 314 "./jafco.w"

{

PCMSK|= (1<<PCINT3);

PCMSK|= (1<<PCINT4);

GIMSK|= (1<<PCIE);
}

/*:28*/
#line 76 "./jafco.w"

/*27:*/
#line 306 "./jafco.w"

{

PORTB|= (1<<PORTB3);

PORTB|= (1<<PORTB4);
}

/*:27*/
#line 77 "./jafco.w"

/*24:*/
#line 273 "./jafco.w"

{


DDRB|= (1<<DDB0);

DDRB|= (1<<DDB1);

DDRB|= (1<<DDB2);
}


/*:24*/
#line 78 "./jafco.w"


/*:6*/
#line 29 "./jafco.w"



/*:1*//*7:*/
#line 84 "./jafco.w"

sei();
/*:7*//*8:*/
#line 92 "./jafco.w"

/*34:*/
#line 374 "./jafco.w"

{
MCUCR&= ~(1<<SM1);
MCUCR&= ~(1<<SM0);
}


/*:34*/
#line 93 "./jafco.w"


/*:8*//*9:*/
#line 100 "./jafco.w"

for(;;)
{

/*:9*//*10:*/
#line 106 "./jafco.w"


ignCntl(OFF);
fuelCntl(OFF);
jawCntl(CLOSE);

/*:10*//*11:*/
#line 114 "./jafco.w"

sleep_mode();
/*:11*//*12:*/
#line 118 "./jafco.w"


if(handleIrq!=NULL)
{
handleIrq();
handleIrq= NULL;
}
}

return 0;
}

/*:12*//*13:*/
#line 132 "./jafco.w"

void releaseSeq()
{

/*:13*//*14:*/
#line 138 "./jafco.w"


jawCntl(OPEN);

while(!(PINB&(1<<PB3)))
_delay_ms(10);

jawCntl(CLOSE);

}
/*:14*//*15:*/
#line 151 "./jafco.w"

void fireSeq()
{

uint8_t firingState;
enum firingStates
{
ready,
opened,
igniting,
burning,
cooling
};


firingState= ready;

/*:15*//*16:*/
#line 172 "./jafco.w"


while(!(PINB&(1<<PB4)))
{

/*:16*//*17:*/
#line 179 "./jafco.w"

if(firingState==ready)
{

jawCntl(OPEN);
firingState= opened;
continue;
}
/*:17*//*18:*/
#line 189 "./jafco.w"

if(firingState==opened)
{

ignCntl(ON);
firingState= igniting;
continue;
}
/*:18*//*19:*/
#line 199 "./jafco.w"


if(firingState==igniting)
{

fuelCntl(ON);
firingState= burning;
continue;
}
_delay_ms(10);
}

/*:19*//*20:*/
#line 213 "./jafco.w"


ignCntl(OFF);
fuelCntl(OFF);
_delay_ms(5000);
jawCntl(CLOSE);

}


/*:20*//*22:*/
#line 232 "./jafco.w"

ISR(PCINT0_vect)
{
const int8_t high= 32;
const int8_t low= -high;
int8_t dbp3= 0;
int8_t dbp4= 0;


while(abs(dbp3)<high)
{
if(!(PINB&(1<<PB3))&&dbp3> low)
dbp3--;
else
if((PINB&(1<<PB3))&&dbp3<high)
dbp3++;
_delay_ms(1);
}

while(abs(dbp4)<high)
{
if(!(PINB&(1<<PB4))&&dbp4> low)
dbp4--;
else
if((PINB&(1<<PB4))&&dbp4<high)
dbp4++;
_delay_ms(1);
}

if(dbp3==low)
handleIrq= &releaseSeq;
else
if(dbp4==low)
handleIrq= &fireSeq;
}


/*:22*//*25:*/
#line 290 "./jafco.w"


/*:25*//*29:*/
#line 326 "./jafco.w"

void jawCntl(uint8_t state)
{
if(state)
{
OCR0A= 0xff;
_delay_ms(200);
OCR0A= 0xff>>1;
}
else{OCR0A= 0x00;}
}

/*:29*//*30:*/
#line 340 "./jafco.w"

void fuelCntl(uint8_t state)
{
if(state)
{
OCR0B= 0xff;
_delay_ms(200);
OCR0B= 0xff>>1;
}
else{OCR0A= 0x00;}

}

/*:30*//*31:*/
#line 355 "./jafco.w"

void ignCntl(uint8_t state)
{
PORTB= state?PORTB|(1<<PORTB2):PORTB&~(1<<PORTB2);
}

/*:31*/
