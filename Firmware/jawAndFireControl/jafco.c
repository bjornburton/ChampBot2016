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

void jawcntl(uint8_t state);
void fuelcntl(uint8_t state);
void igncntl(uint8_t state);
void releaseseq(void);
void fireseq(void);

/*:5*/
#line 28 "./jafco.w"

/*6:*/
#line 67 "./jafco.w"

void(*handleirq)()= NULL;



int main(void)
{


/*26:*/
#line 291 "./jafco.w"

{

PCMSK|= (1<<PCINT3);

PCMSK|= (1<<PCINT4);

GIMSK|= (1<<PCIE);
}

/*:26*/
#line 76 "./jafco.w"

/*25:*/
#line 283 "./jafco.w"

{

PORTB|= (1<<PORTB3);

PORTB|= (1<<PORTB4);
}

/*:25*/
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

/*32:*/
#line 339 "./jafco.w"

{
MCUCR&= ~(1<<SM1);
MCUCR&= ~(1<<SM0);
}


/*:32*/
#line 93 "./jafco.w"


/*:8*//*9:*/
#line 100 "./jafco.w"

for(;;)
{

/*:9*//*10:*/
#line 106 "./jafco.w"


igncntl(OFF);
fuelcntl(OFF);
jawcntl(CLOSE);

/*:10*//*11:*/
#line 114 "./jafco.w"

sleep_mode();
/*:11*//*12:*/
#line 118 "./jafco.w"


if(handleirq!=NULL)
{
handleirq();
handleirq= NULL;
}
}

return 0;
}

/*:12*//*13:*/
#line 132 "./jafco.w"

void releaseseq()
{

/*:13*//*14:*/
#line 138 "./jafco.w"


jawcntl(OPEN);

while(!(PINB&(1<<PB3)))
_delay_ms(10);

jawcntl(CLOSE);

}
/*:14*//*15:*/
#line 151 "./jafco.w"

void fireseq()
{

uint8_t firingstate;
enum firingstates
{
ready,
opened,
igniting,
burning,
cooling
};


firingstate= ready;

/*:15*//*16:*/
#line 172 "./jafco.w"


while(!(PINB&(1<<PB4)))
{

/*:16*//*17:*/
#line 179 "./jafco.w"

if(firingstate==ready)
{

jawcntl(OPEN);
firingstate= opened;
continue;
}
/*:17*//*18:*/
#line 189 "./jafco.w"

if(firingstate==opened)
{

igncntl(ON);
firingstate= igniting;
continue;
}
/*:18*//*19:*/
#line 199 "./jafco.w"


if(firingstate==igniting)
{

fuelcntl(ON);
firingstate= burning;
continue;
}
_delay_ms(10);
}

/*:19*//*20:*/
#line 213 "./jafco.w"


igncntl(OFF);
fuelcntl(OFF);
_delay_ms(5000);
jawcntl(CLOSE);

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
handleirq= &releaseseq;
else
if(dbp4==low)
handleirq= &fireseq;
}


/*:22*//*27:*/
#line 303 "./jafco.w"

void jawcntl(uint8_t state)
{
PORTB= state?PORTB|(1<<PORTB0):PORTB&~(1<<PORTB0);
}

/*:27*//*28:*/
#line 311 "./jafco.w"

void fuelcntl(uint8_t state)
{

PORTB= state?PORTB|(1<<PORTB1):PORTB&~(1<<PORTB1);
}

/*:28*//*29:*/
#line 320 "./jafco.w"

void igncntl(uint8_t state)
{
PORTB= state?PORTB|(1<<PORTB2):PORTB&~(1<<PORTB2);
}

/*:29*/
