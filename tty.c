/*********************************************************************
*
*       file:               tty.c
*       author:             cole maxwell
*       original author:    betty o'neil
*
*       tty driver--device-specific routines for ttys 
*
*/
#include <stdio.h>  /* for kprintf prototype */
#include <serial.h>
#include <cpu.h>
#include <pic.h>
#include "ioconf.h"
#include "tty_public.h"
#include "tty.h"
#include "queue/queue.h"

struct tty ttytab[NTTYS]; /* software params/data for each SLU dev */

Queue RX_Queue, TX_Queue; /* set up queue memory objs */
Queue *rxq = &RX_Queue; /* set up ptrs to RX/TX_QUEUE  */
Queue *txq = &TX_Queue; /* these ptrs represent their objs */

/* Record debug info in otherwise free memory between program and stack */
/* 0x300000 = 3M, the start of the last M of user memory on the SAPC */
#define DEBUG_AREA 0x300000
#define BUFLEN 20

char *debug_log_area = (char *)DEBUG_AREA;
char *debug_record;  /* current pointer into log area */ 

/* tell C about the assembler shell routines */
extern void irq3inthand(void), irq4inthand(void);

/* C part of interrupt handlers--specific names called by the assembler code */
extern void irq3inthandc(void), irq4inthandc(void); 

/* the common code for the two interrupt handlers */                           
static void irqinthandc(int dev);

/* prototype for debug_log */ 
void debug_log(char *);

/*====================================================================
*
*       tty specific initialization routine for COM devices
*
*/

void ttyinit(int dev)
{
  int baseport;
  struct tty *tty;  /* ptr to tty software params/data block */
  
  init_queue(rxq, MAXBUF);  /* create RX_QUEUE with depth 6 */
  init_queue(txq, MAXBUF);  /* create TX_QUEUE with depth 6 */

  debug_record = debug_log_area; /* clear debug log */
  baseport = devtab[dev].dvbaseport; /* pick up hardware addr */
  tty = (struct tty *)devtab[dev].dvdata; /* and software params struct */

  if (baseport == COM1_BASE) {
      /* arm interrupts by installing int vec */
      set_intr_gate(COM1_IRQ+IRQ_TO_INT_N_SHIFT, &irq4inthand);
      pic_enable_irq(COM1_IRQ);
  } else if (baseport == COM2_BASE) {
      /* arm interrupts by installing int vec */
      set_intr_gate(COM2_IRQ+IRQ_TO_INT_N_SHIFT, &irq3inthand);
      pic_enable_irq(COM2_IRQ);
  } else {
      kprintf("Bad TTY device table entry, dev %d\n", dev);
      return;			/* give up */
  }

  // members of the tty struct
  tty->echoflag = 1;		/* default to echoing */
  tty->rin = 0;               /* initialize indices */
  tty->rout = 0;
  tty->rnum = 0;              /* initialize counter */
  tty->tin = 0;               /* initialize indices */
  tty->tout = 0;
  tty->tnum = 0;              /* initialize counter */

  /* always enable interrupts on receiver */
  outpt(baseport+UART_IER, UART_IER_RDI); /* RDI = receiver data int */
}


/*====================================================================
*
*       Useful function when emptying/filling the read/write buffers
*
*/

#define min(x,y) (x < y ? x : y)


/*====================================================================
*
*       tty-specific read routine for TTY devices
*
*/

int ttyread(int dev, char *buf, int nchar)
{
  int charcount = 0;

  while (nchar != 0) {  /* loop: have nchar chars been read? */
    if (queuecount(rxq) > 0) {/* is there anything in the RX_Queue? */
      buf[charcount++] = dequeue(rxq); /* dequeue from RX, add to buf */
      nchar--;  /* decrement char count */
    }
  }
  return charcount; /* wait for all nchar chars to be read */
}


/*====================================================================
*
*       tty-specific write routine for SAPC devices
*
*/

int ttywrite(int dev, char *buf, int nchar)
{
  int baseport;
  int charcount = 0;  
  baseport = devtab[dev].dvbaseport; /* hardware addr from devtab */

  while (nchar != 0) {  /* Loop: have nchar chars been written? */
    if (queuecount(txq) == MAXBUF) { /* is TX_Queue full? */
      outpt(baseport+UART_IER, UART_IER_THRI); /* kickstart TX interrupt */
      outpt(baseport+UART_IER, !UART_IER_THRI); /* disable TX interrupts */
      outpt(baseport+UART_IER, UART_IER_RDI); /* reenable receiver after TX */
    } else {  /* no, enqueue to TX_Queue */
      char ch = buf[charcount++];
      enqueue(txq, ch); /* enqueue byte from buf into TX_Queue */
      nchar--;  /* decrement char count */
    }
  }
  outpt(baseport+UART_IER, UART_IER_THRI); /* kickstart TX interrupt */
  outpt(baseport+UART_IER, !UART_IER_THRI); /* disable TX interrupts */
  outpt(baseport+UART_IER, UART_IER_RDI); /* reenable receiver after TX */
  return charcount;       /* waits for all nchar chars to be read */
}

/*====================================================================
*
*       tty-specific control routine for TTY devices
*
*/

int ttycontrol(int dev, int fncode, int val)
{
  struct tty *this_tty = (struct tty *)(devtab[dev].dvdata);

  if (fncode == ECHOCONTROL)
    this_tty->echoflag = val;
  else return -1;
  return 0;
}

/*====================================================================
*
*       tty-specific interrupt routine for COM ports
*
*   Since interrupt handlers don't have parameters, we have two different
*   handlers.  However, all the common code has been placed in a helper 
*   function.
*/
  
void irq4inthandc()
{
  irqinthandc(TTY0);
}                              
  
void irq3inthandc()
{
  irqinthandc(TTY1);
}                              

void irqinthandc(int dev) { 

  int ch, iir;
  struct tty *tty = (struct tty *)(devtab[dev].dvdata);
  int baseport = devtab[dev].dvbaseport; /* hardware i/o port */;

  iir = inpt(baseport + UART_IIR);
  
  switch (iir & UART_IIR_ID) {  /* mask the 2-bit ID field */

    case UART_IIR_RDI:  /* ISR for receiver */
      pic_end_int();                /* notify PIC that its part is done */
      ch = inpt(baseport+UART_RX);	/* read char, ack the device */
      enqueue(rxq, ch); /* enqueue into RX_Queue */

    case UART_IIR_THRI: /* ISR for transmitter */
      pic_end_int();  /* notify PIC that its part is done */
      while (queuecount(txq) != 0) {
        ch = dequeue(txq); /* dequeue from TX_Queue and send to UART_TX */
        outpt(baseport+UART_TX, ch);  /* output the character */
      }

    default:
      return;
}

if (tty->echoflag)  /* if echoing wanted */
  outpt(baseport+UART_TX,ch); /* echo char: see note above */
}

/* append msg to memory log */
void debug_log(char *msg)
{
    strcpy(debug_record, msg);
    debug_record +=strlen(msg);
}
