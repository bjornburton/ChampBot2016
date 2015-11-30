
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
void jawcntl(uint8_t state); // Jaw open and close
void fuelcntl(uint8_t state); // Fuel on and off
void igncntl(uint8_t state); // on and off
void releaseseq(void);
void fireseq(void);

@
My lone global variable is a function pointer.
This lets me pass arguments to the actual interrupt handlers, should I need to.
This pointer gets the appropriate function attached by one of the |"ISR()"|
functions.

@<Global var...@>=
void @[@] (*handleirq)() = NULL;


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

 igncntl(OFF);
 fuelcntl(OFF);
 jawcntl(CLOSE);

@
Now we wait in ``idle'' for any interrupt event.
@c
  sleep_mode();
@
If execution arrives here, some interrupt has been detected.
@c

if (handleirq != NULL)  // not sure why it would be, but to be safe
   @/{@/
    handleirq();
    handleirq = NULL; // reset so that the action cannot be repeated
    }// end if handleirq
  } // end for

return 0; // it's the right thing to do!
} // end main()

@* Interrupt Handling.

@c
void releaseseq()@/
{@/

@
This sequence will proceed only while the button is held.
@c

jawcntl(OPEN);

   while(!(PINB & (1<<PB3)))
         _delay_ms(10);

jawcntl(CLOSE);

}
@


@c
void fireseq()@/
{@/

uint8_t firingstate;
enum firingstates
  {
   ready,
   opened,
   warned,
   igniting,
   burning
  };


firingstate = ready;

@
This sequence will proceed only while the button is held.
It can terminate after any state.
|"_delay_ms()"| is a handy macro good for $2^{16}$ milliseconds of delay.
@c

while( !(PINB & (1<<PB4)) )
     {@/

      @
      The jaw opens here for fire, but also partly as a warning.
      @c
      if(firingstate == ready)
        {@/


         jawcntl(OPEN);
         firingstate = opened;
         continue;
        }
      @
      Three warning bursts from the ignitor and then a 1000 ms duck delay.
      @c
      if(firingstate == opened)
        {@/

         for(int8_t count = 0;count < 3;count++)
            {
             igncntl(ON);
             _delay_ms(100);
             igncntl(OFF);
             _delay_ms(200);
            }

         _delay_ms(1000); /* human duck time */
         firingstate = warned;
         continue;
        }
      @
      Fuel opens for precharge, then some delay.
      @c

      if(firingstate == warned)
        {@/

         fuelcntl(ON);
         _delay_ms(500);
         firingstate = igniting;
         continue;
        }
      @
      Ignitor on, delay a short time.
      @c

      if(firingstate == igniting)
        {@/

         igncntl(ON);
         _delay_ms(250);
		 igncntl(OFF);
         firingstate = burning;
         continue;
        }

      @
      We should have fire now, but will tend it in case of wind.
      @c
      if(firingstate == burning)
        {@/
          _delay_ms(1000);
          igncntl(ON);
          _delay_ms(200);
          igncntl(OFF);
          continue;
		 }
     }

@
Once the loop fails we set fuel and ignitor off and close the jaw.
@c

 igncntl(OFF);
 fuelcntl(OFF);
 jawcntl(CLOSE);

}


@*The ISRs.

The ISRs are pretty skimpy as they mostly used to point |handleirq()| to
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
   handleirq = &releaseseq;
 else
 if(dbp4 == low)
   handleirq = &fireseq;
}


@* These are the supporting routines, procedures and configuration blocks.


Here is the block that sets-up the digital I/O pins.
@ @<Initialize pin outputs...@>=@/
{@/
 /* set the jaw port direction */
  DDRB |= (1<<DDB0);
 /* set the fuel port direction */
  DDRB |= (1<<DDB1);
 /* set the ignition port direction */
  DDRB |= (1<<DDB2);
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
void jawcntl(uint8_t state)@/
{@/
  PORTB = state ? PORTB | (1<<PORTB0) : PORTB & ~(1<<PORTB0);
}

@
Here is a simple procedure to operate the fuel.
@c
void fuelcntl(uint8_t state)@/
{@/

  PORTB = state ? PORTB | (1<<PORTB1) : PORTB & ~(1<<PORTB1);
}

@
Here is a simple procedure to operate the ignition.
@c
void igncntl(uint8_t state)@/
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



