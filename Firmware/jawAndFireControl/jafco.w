
% jafco
\input miniltx
\input graphicx


\nocon % omit table of contents
\datethis % print date on listing

@* Introduction. This is the firmware portion of Jaw and Fire control.

This will facilitate two actions: opening the jaw to release the floating
object and light the target on fire.

The jaw will close by return-spring so the action will to open it.

Fire is a  sequence of opening the jaw, releasing the butane and firing the
ignitor.

\vskip 4 pc
\includegraphics[width=35 pc]{jafco.png}

Extensive use was made of the datasheet, Atmel ``Atmel ATtiny25, ATtiny45,
 ATtiny85 Datasheet'' Rev. 2586Q–AVR–08/2013 (Tue 06 Aug 2013 03:19:12 PM EDT)
and ``AVR130: Setup and Use the AVR Timers'' Rev. 2505A–AVR–02/02.
@c
@< Include @>@;
@< Prototypes @>@;
@< Global variables @>@;


@ |"F_CPU"| is used to convey the Trinket clock rate.
@d F_CPU 8000000UL


@ Here are some Boolean definitions that are used.
@d ON 1
@d OFF 0
@d OPEN 1
@d CLOSE 0
@d SET 1
@d CLEAR 0

@ @<Include...@>=
# include <avr/io.h> // need some port access
# include <util/delay.h> // need to delay
# include <avr/interrupt.h> // have need of an interrupt
# include <avr/sleep.h> // have need of sleep
# include <avr/wdt.h> // have need of watchdog
# include <stdlib.h>
# include <stdint.h>


@ @<Prototypes@>=
void jawCntl(uint8_t state); // Jaw open and close
void fuelCntl(uint8_t state); // Fuel on and off
void ignCntl(uint8_t state); // on and off
void releaseSeq(void);
void fireSeq(void);

@
My lone global variable is a function pointer.
This lets me pass arguments to the actual interrupt handlers, should I need to.
This pointer gets the appropriate function attached by one of the |"ISR()"|
functions.

@<Global var...@>=
void @[@] (*handleIrq)() = NULL;


@/
int main(void)@/
{@/


@<Initialize interrupts@>@/
@<Initialize pin inputs@>@/
@<Initialize pin outputs@>@/

@
Of course, any interrupt function requires that bit ``Global Interrupt Enable''
is set; usually done through calling sei(). Doing this after the pin setup is
the best time.
@c
  sei();
@
Rather than burning loops, waiting for something to happen,
the ``sleep'' mode is used.
The specific type of sleep is `idle'.
In idle, execution stops but timers continue.
Interrupts are used to wake it.
@c
@<Configure to wake upon interrupt...@>

@
This is the loop that does the work.
It should spend most of its time in |sleep_mode|,
comming out at each interrupt event.

@c
 for (;;)@/
  {@/

@
We don't want anything cooking while we are asleap.
@c

 ignCntl(OFF);
 fuelCntl(OFF);
 jawCntl(CLOSE);

@
Now we wait in ``idle'' for any interrupt event.
@c
  sleep_mode();
@
If execution arrives here, some interrupt has been detected.
@c

if (handleIrq != NULL)  // not sure why it would be, but to be safe
   @/{@/
    handleIrq();
    handleIrq = NULL; // reset so that the action cannot be repeated
    }// end if handleIrq
  } // end for

return 0; // it's the right thing to do!
} // end main()

@* Interrupt Handling.

@c
void releaseSeq()@/
{@/

@
This sequence will proceed only while the button is held.
@c

jawCntl(OPEN);

   while(!(PINB & (1<<PB3)))
         _delay_ms(10);

jawCntl(CLOSE);

}
@


@c
void fireSeq()@/
{@/

uint8_t firingState;
enum firingStates
  {
   ready,
   opened,
   igniting,
   burning,
   cooling
  };


firingState = ready;

@
This sequence will proceed only while the button is held.
It can terminate after any state.
|"_delay_ms()"| is a handy macro good for $2^{16}$ milliseconds of delay.
@c

while( !(PINB & (1<<PB4)) )
     {@/

      @
      The jaw opens here for fire.
      @c
      if(firingState == ready)
        {@/

         jawCntl(OPEN);
         firingState = opened;
         continue;
        }
      @
      Ignitor is on.
      @c
      if(firingState == opened)
        {@/

         ignCntl(ON);
         firingState = igniting;
         continue;
        }
      @
      Fuel opens.
      @c

      if(firingState == igniting)
        {@/

         fuelCntl(ON);
         firingState = burning;
         continue;
        }
      _delay_ms(10);
     }

@
Once the loop fails we set fuel and ignitor off and close the jaw.
@c

 ignCntl(OFF);
 fuelCntl(OFF);
 _delay_ms(5000);
 jawCntl(CLOSE);

}


@*The ISRs.

The ISRs are pretty skimpy as they mostly used to point |handleIrq()| to
the correct function.
The need for global variables is minimized.

@
This vector responds to the jaw input at pin PB3 or fire input at PB4.
A simple debounce is included.
@c
ISR(PCINT0_vect)@/
{@/
const int8_t high = 32;
const int8_t low = -high;
int8_t dbp3 = 0;
int8_t dbp4 = 0;


while(abs(dbp3) < high)
     {
      if(!(PINB & (1<<PB3)) && dbp3 > low)
         dbp3--;
          else
          if((PINB & (1<<PB3)) && dbp3 < high)
         dbp3++;
     _delay_ms(1);
     }

while(abs(dbp4) < high)
     {
      if(!(PINB & (1<<PB4)) && dbp4 > low)
         dbp4--;
          else
          if((PINB & (1<<PB4)) && dbp4 < high)
         dbp4++;
     _delay_ms(1);
     }

if(dbp3 == low)
   handleIrq = &releaseSeq;
 else
 if(dbp4 == low)
   handleIrq = &fireSeq;
}


@* These are the supporting routines, procedures and configuration blocks.


Here is the block that sets-up the digital I/O pins.
@ @<Initialize pin outputs...@>=@/
{@/
 // 14.4.9 DDRD – The Port D Data Direction Register
 /* set the jaw port direction */
  DDRB |= (1<<DDB0);
 /* set the fuel port direction */
  DDRB |= (1<<DDB1);
 /* set the ignition port direction */
  DDRB |= (1<<DDB2);
}


@
Timer Counter 0 is configured for ``Phase Correct'' PWM which, according to the
datasheet, is preferred for motor control.
OC0A and OC0B are set to clear on a match which creates a
non-inverting PWM.
@c

@ @<Initialize Timer@>=
{@/

 // 15.9.1 TCCR0A – Timer/Counter Control Register A
 TCCR0A |= (1<<WGM00);   // Phase correct, mode 1 of PWM (table 15-9)
 TCCR0A |= (1<<COM0A1);  // Set/Clear on Comparator A match (table 15-4)
 TCCR0A &= ~(1<<COM0A0); // Set  on Comparator A match (table 15-4)
 TCCR0A |= (1<<COM0B1);  // Set/Clear on Comparator B match (table 15-7)
 TCCR0A &= ~(1<<COM0B0); // Set on Comparator B match (table 15-7)

 // 15.9.2 TCCR0B – Timer/Counter Control Register B
 TCCR0B |= (1<<CS01);   // Prescaler set to clk/8 (table 15-9)
}

@ @<Initialize pin inputs...@>=@/
{@/
 /* set the jaw input pull-up */
  PORTB |= (1<<PORTB3);
 /* set the fire input pull-up */
  PORTB |= (1<<PORTB4);
}

@ @<Initialize interrupts...@>=@/
{@/
 /* enable  change interrupt for jaw input */
  PCMSK |= (1<<PCINT3);
 /* enable  change interrupt for fire input */
  PCMSK |= (1<<PCINT4);
 /* General interrupt Mask register */
  GIMSK |= (1<<PCIE);
}

@
Here is a simple procedure to operate the jaw.
@c
void jawCntl(uint8_t state)@/
{@/
  if (state)
   {
    OCR0A = 0xff;
    _delay_ms(200);
    OCR0A = 0xff >> 1;
   }
   else {OCR0A = 0x00;}
}

@
Here is a simple procedure to operate the fuel.
@c
void fuelCntl(uint8_t state)@/
{@/
  if (state)
   {
    OCR0B = 0xff;
    _delay_ms(200);
    OCR0B = 0xff >> 1;
   }
   else {OCR0A = 0x00;}

}

@
Here is a simple procedure to operate the ignition.
@c
void ignCntl(uint8_t state)@/
{@/
  PORTB = state ? PORTB | (1<<PORTB2) : PORTB & ~(1<<PORTB2);
}

@
See section the datasheet for details on the Watchdog Timer.
We are not using it right now.
@ @<Initialize watchdog timer...@>=@/
{@/
 WDTCR |= (1<<WDCE) | (1<<WDE);
 WDTCR = (1<<WDIE) | (1<<WDP2); // reset after about 0.25 seconds
}

@
Setting these bits configure sleep\_mode() to go to ``idle''.
Idle allows the counters and comparator to continue during sleep.

@<Configure to wake upon interrupt...@>=@/
{@/
  MCUCR &= ~(1<<SM1);
  MCUCR &= ~(1<<SM0);
}



