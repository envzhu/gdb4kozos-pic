/*
 * pic-uart.c
 */

#include <stdint.h>
#include <unistd.h>
#include <sys/select.h>
#include <sys/types.h>

#include "sim-main.h"
#include "sim-utils.h"

#include "hw-main.h"
#include "dv-sockser.h"
#include "sim-assert.h"

#define CPU cpu
#define SD sd

#define SIM_STDIN   0
#define SIM_STDOUT  1

#define SERIAL_UART_NUM 2

#define PIC_UART1 ((volatile struct pic_uart *)0xBF806000)
#define PIC_UART2 ((volatile struct pic_uart *)0xBF806200)

typedef struct _reg_t{
  volatile uint32_t body;
  volatile uint32_t CLR;
  volatile uint32_t SET;
  volatile uint32_t INV;
}reg_t;

struct pic_uart {
  volatile uint32_t UxMODE;
  volatile uint32_t UxMODECLR;
  volatile uint32_t UxMODESET;
  volatile uint32_t UxMODEINV;
  volatile uint32_t UxSTA;
  volatile uint32_t UxSTACLR;
  volatile uint32_t UxSTASET;
  volatile uint32_t UxSTAINV;
  volatile uint32_t UxTXREG;
  volatile uint32_t UxTXREGCLR;
  volatile uint32_t UxTXREGSET;
  volatile uint32_t UxTXREGINV;
  volatile uint32_t UxRXREG;
  volatile uint32_t UxRXREGCLR;
  volatile uint32_t UxRXREGSET;
  volatile uint32_t UxRXREGINV;
};

volatile reg_t IEC[2];
volatile reg_t IFS[2];

#define IEC1 IEC[1].body
#define IFS1 IFS[1].body


//UxMODE register
#define PIC_UART_UxMODE_SIDL   (1<<13)
//skip 14bit  read as 0
#define PIC_UART_UxMODE_ON     (1<<15)

//UxSTA register
#define PIC_UART_UxSTA_URXDA   (1<<0)
#define PIC_UART_UxSTA_OERR    (1<<1)
#define PIC_UART_UxSTA_FERR    (1<<2)
#define PIC_UART_UxSTA_PERR    (1<<3)
#define PIC_UART_UxSTA_RIDLE   (1<<4)
#define PIC_UART_UxSTA_ADDEN   (1<<5)
#define PIC_UART_UxSTA_URXTISEL_EVERY  (0<<6)
#define PIC_UART_UxSTA_URXTISEL_THREE  (2<<6)
#define PIC_UART_UxSTA_URXTISEL_FULL   (3<<6)
#define PIC_UART_UxSTA_TRMT_TXBUF_is_EMPTY  (1<<8) //ignore
#define PIC_UART_UxSTA_UTXBF   (1<<9)   //ignore
#define PIC_UART_UxSTA_UTXEN   (1<<10)
#define PIC_UART_UxSTA_UTXBRK  (1<<11)
#define PIC_UART_UxSTA_URXEN   (1<<12)
#define PIC_UART_UxSTA_UTXINV  (1<<13)
#define PIC_UART_UxSTA_UTXISEL_BUFFER_SOME_FREE  (0<<14)
#define PIC_UART_UxSTA_UTXISEL_EVERY_DATA        (1<<14)
#define PIC_UART_UxSTA_UTXISEL_BUFFER_ALL_FREE   (2<<14)
#define PIC_UART_UxSTA_UTXISEL_DONOT_USE         (3<<14)
// bit 23-16 is read only
#define PIC_UART_UxSTA_ADM_EN  (1<<24)

//UART Interrupts Registers
#define PIC_UART1_RECV_INTTERUPT_FLAG    (1<<8)
#define PIC_UART1_RECV_INTTERUPT_ENABLE  (1<<8)
#define PIC_UART1_SEND_INTTERUPT_FLAG    (1<<9)
#define PIC_UART1_SEND_INTTERUPT_ENABLE  (1<<9)

#define PIC_UART2_RECV_INTTERUPT_FLAG    (1<<22)
#define PIC_UART2_RECV_INTTERUPT_ENABLE  (1<<22)
#define PIC_UART2_SEND_INTTERUPT_FLAG    (1<<23)
#define PIC_UART2_SEND_INTTERUPT_ENABLE  (1<<23)


static volatile struct pic_uart uart[SERIAL_UART_NUM];
static int interrupt_pending;

static void serial_tickle(SIM_DESC sd);

static fd_set stdin_fds;
static struct timeval stdin_tm;

void serial_init(SIM_DESC sd){
  int i;
  
  for(i=0; i<SERIAL_UART_NUM; i++){
    uart[i].UxSTA |= PIC_UART_UxSTA_TRMT_TXBUF_is_EMPTY;
  }
  IFS1=0;

  sim_write(sd, PIC_UART1, &uart[0], sizeof(struct pic_uart));
  sim_write(sd, PIC_UART2, &uart[1], sizeof(struct pic_uart));
  sim_write(sd, 0xBF881040, &IFS1, sizeof(reg_t));  


  int ret;
  char c;

  FD_ZERO(&stdin_fds);
  FD_SET(0, &stdin_fds);
  stdin_tm.tv_sec  = 0;
  stdin_tm.tv_usec = 0;

  //clear stdin buf
  ret = select(SIM_STDIN + 1, &stdin_fds, NULL, NULL, &stdin_tm);
  if(ret>0)
    sim_io_read(sd, SIM_STDIN, &c, 1);

  sim_events_schedule (sd, 0, serial_tickle, NULL);
}

// calculate *CLR, *SET, *INV register
void reg_calc(void *p, int len){
  if((len%sizeof(reg_t))!=0){
    printf("reg_calc() invaild length\n");
    return;
  }
  
  reg_t *reg = p;
  int i;
  for(i=0; i < len/sizeof(reg_t); i++){
    if(reg[i].CLR){
      reg[i].body &= ~reg[i].CLR;
      reg[i].CLR = 0;
    }
    if(reg[i].SET){
      reg[i].body |= reg[i].SET;
      reg[i].SET = 0;
    }
    if(reg[i].INV){
      printf("Not Supported Register Operation\n");
      reg[i].INV = 0;
    }
  }
}

static void serial_tickle(SIM_DESC sd){

  sim_cpu *cpu = STATE_CPU (sd, 0); /* FIXME */
  address_word cia = CPU_PC_GET (cpu);

  //Sync registers
  sim_read(sd, PIC_UART1, &uart[0], sizeof(struct pic_uart));
  sim_read(sd, PIC_UART2, &uart[1], sizeof(struct pic_uart));
  sim_read(sd, 0xBF881070, &IEC1, sizeof(reg_t));
  sim_read(sd, 0xBF881040, &IFS1, sizeof(reg_t));

  reg_calc(&uart[0], sizeof(struct pic_uart));
  reg_calc(&uart[1], sizeof(struct pic_uart));

  reg_calc(&IEC1, sizeof(reg_t));
  reg_calc(&IFS1, sizeof(reg_t));

  int i;
  for(i=0; i<SERIAL_UART_NUM; i++){
    if(uart[i].UxMODE&PIC_UART_UxMODE_ON){

      //UART send
      if((uart[i].UxSTA&PIC_UART_UxSTA_UTXEN)&&
      (uart[i].UxTXREG!=0)){
        if(uart[i].UxTXREG != '\r')
          sim_io_write(sd, SIM_STDOUT, &uart[i].UxTXREG, 1);  
        uart[i].UxTXREG = 0;
        uart[i].UxSTA |= PIC_UART_UxSTA_TRMT_TXBUF_is_EMPTY;

          //occur serial interrupt
          if(i==0)
            IFS1|=PIC_UART1_SEND_INTTERUPT_FLAG;
          if(i==1)
            IFS1|=PIC_UART2_SEND_INTTERUPT_FLAG;
      }

      //UART recieve
      if(uart[i].UxSTA&PIC_UART_UxSTA_URXEN){

        if(i==0){
          if(!(IFS1&PIC_UART1_RECV_INTTERUPT_FLAG)){
            int ret;
            char c;
            
            uart[i].UxSTA &= ~PIC_UART_UxSTA_URXDA;
            FD_ZERO(&stdin_fds);  
            FD_SET(0, &stdin_fds);
            ret = select(SIM_STDIN + 1, &stdin_fds, NULL, NULL, &stdin_tm);

            if(ret>0){
              
              sim_io_read(sd, SIM_STDIN, &c, 1);
              c = (c == '\n') ? '\r' : c;
              uart[i].UxRXREG = c;
              uart[i].UxSTA |= PIC_UART_UxSTA_URXDA;

              //Set IFS(interrpt flag)
              IFS1 |= PIC_UART1_RECV_INTTERUPT_FLAG;
          }
        }else if(i==1){
          if((IFS1&PIC_UART2_RECV_INTTERUPT_FLAG)?0:1){
            int ret;
            char c;
            
            uart[i].UxSTA &= ~PIC_UART_UxSTA_URXDA;
            FD_ZERO(&stdin_fds);  
            FD_SET(0, &stdin_fds);
            ret = select(SIM_STDIN + 1, &stdin_fds, NULL, NULL, &stdin_tm);

            if(ret>0){
              
              sim_io_read(sd, SIM_STDIN, &c, 1);
              c = (c == '\n') ? '\r' : c;
              uart[i].UxRXREG = c;
              uart[i].UxSTA |= PIC_UART_UxSTA_URXDA;

              //Set IFS(interrpt flag)
              IFS1 |= PIC_UART2_RECV_INTTERUPT_FLAG;
            }
          }
        }
      }
    }


  }

  sim_write(sd, PIC_UART1, &uart[0], sizeof(struct pic_uart));
  sim_write(sd, PIC_UART2, &uart[1], sizeof(struct pic_uart));

  sim_write(sd, 0xBF881040, &IFS1, sizeof(reg_t));  
  sim_write(sd, 0xBF881070, &IEC1, sizeof(reg_t));  

  /* If interrupt flag is set and interrupt is enabled,
     call seriat_intr() */  
  void serial_intr (SIM_DESC sd, void *data);
  if((IFS1!=0)&&
    (IEC1!=0)){
    sim_events_schedule (sd, 0, serial_intr, NULL);
  }
 sim_events_schedule (sd, 1, serial_tickle, NULL);
}


void serial_intr(SIM_DESC sd, void *data)
{
  sim_cpu *cpu = STATE_CPU (sd, 0); /* FIXME */
  address_word cia = CPU_PC_GET (cpu);

  if ((SR & status_IE) && !(SR & status_EXL)){

     /* if this exception is serial interrupt,
        let's occur interrupt.*/
    if((IFS1&PIC_UART1_SEND_INTTERUPT_FLAG)&&
    serial_intr_is_send_enable(0)){
      SignalExceptionInterrupt (37); /* interrupt "37" */
    }else if((IFS1&PIC_UART2_SEND_INTTERUPT_FLAG)&&
    serial_intr_is_send_enable(1)){
      SignalExceptionInterrupt (37); /* interrupt "37" */
    }else if((IFS1&PIC_UART1_RECV_INTTERUPT_FLAG)&&
    serial_intr_is_recv_enable(0)){
      SignalExceptionInterrupt (37); /* interrupt "37" */
    }else if((IFS1&PIC_UART2_RECV_INTTERUPT_FLAG)&&
    serial_intr_is_recv_enable(1)){
      SignalExceptionInterrupt (37); /* interrupt "37" */
    }

  }
}


/* Can we send a data ? */
int serial_is_send_enable(int index)
{
  return (!(uart[index].UxSTA&PIC_UART_UxSTA_UTXBF));
}

/* Has UART device recieved any data? */
static inline int serial_is_recv_enable(int index)
{
  return (uart[index].UxSTA & PIC_UART_UxSTA_URXDA);
}

/*Is send interrupt enabled? */
int serial_intr_is_send_enable(int index)
{
  if(index==0)
    return (IEC1 & PIC_UART1_SEND_INTTERUPT_ENABLE) ? 1 : 0;;
  
  if(index==1)
    return (IEC1 & PIC_UART2_SEND_INTTERUPT_ENABLE) ? 1 : 0;

  return -1;
}

/*Is recieve interrupt enabled? */
int serial_intr_is_recv_enable(int index)
{
  if(index==0)
    return (IEC1 & PIC_UART1_RECV_INTTERUPT_ENABLE) ? 1 : 0;;
  
  if(index==1)
    return (IEC1 & PIC_UART2_RECV_INTTERUPT_ENABLE) ? 1 : 0;

  return -1;
}
