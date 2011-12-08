#include "sysbus.h"
#include "arm-misc.h"
#include "devices.h"
#include "boards.h"
#include "qemu-common.h"
#include "qemu-timer.h"
#include "qemu-char.h"
#include "pc.h"
#include "hw/hw.h"
#include "hw/irq.h"
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include "sysemu.h"

//-----defines-----
#define PERIPHERAL_AREA_SIZE 0x3fff //16KB

/* TIMER MODULE */
typedef struct timer_state {
	SysBusDevice busdev;
	uint32_t intr_reg;
	uint32_t intr_mask;
	uint32_t timer_cntrl_reg;
	uint32_t timer_counter;
	uint32_t prescale_reg;
	uint32_t prescale_counter;
	uint32_t match_cntrl_reg;
	uint32_t match_reg[3];
	uint32_t cap_cntrl_reg;
	uint32_t capture_reg[2];
	uint32_t external_match_reg;
	uint32_t count_cntrl_reg;
	int64_t tick;
	QEMUTimer *timer;
	qemu_irq irq;
}timer_state;

static void timer_update_irq(timer_state *s)
{
	//int level;
	//level = (s->intr_reg & s->intr_mask) != 0;
	qemu_set_irq(s->irq, 0);
}

static void timer_reload(timer_state* s, int reset)
{
	int64_t tick;
	//if(reset)
		tick = qemu_get_clock_ns(vm_clock);
	//else 
	//	tick = s->tick;
	if ((s->count_cntrl_reg & 0x3) == 0)
	{
		uint32_t count;
		count = 1;
		if(s->prescale_reg > 0)
		{
			tick += (int64_t)count *  +  10 * s->prescale_reg ;//* system_clock_scale;
		} else {
			tick += (int64_t)count * 10;
		}
		s->tick = tick;
		qemu_mod_timer(s->timer, tick);
	} else {
		hw_error("Counter Mode Not Supported\n");
	}
}

static void timer_stop(timer_state *s)
{
	if(!s->timer)
		return;
	qemu_del_timer(s->timer);
}

static uint32_t timer_read(void *opaque, target_phys_addr_t offset)
{
	timer_state *s = (timer_state*) opaque;
	switch (offset)
	{
		case 0x00:
					return s->intr_reg;
		case 0x04:
					return s->timer_cntrl_reg;
		case 0x08:
					//printf("Timer Counter Read");
					return s->timer_counter;
		case 0x0c:
					return s->prescale_reg;
		case 0x10:
					return s->prescale_counter;
		case 0x14:
					return s->match_cntrl_reg;
		case 0x18:
					return s->match_reg[0];
		case 0x1c:
					return s->match_reg[1];
		case 0x20:
					return s->match_reg[2];
		case 0x24:
					return s->match_reg[3];
		case 0x28:
					return s->cap_cntrl_reg;
		case 0x2c:
					return s->capture_reg[0];
		case 0x30:
					return s->capture_reg[1];
		case 0x3c:
					return s->external_match_reg;
		case 0x70:
					return s->count_cntrl_reg;
		default:
				return 0;
	}
	return 0;
}

static void timer_write(void *opaque, target_phys_addr_t offset, uint32_t value)
{
	timer_state *s = (timer_state*)opaque;
	printf("\nTimer Write");
	switch(offset)
	{
		case 0x00:
				s->intr_mask |= value;
				break;
		case 0x04:
				printf("Timer Clock Enable update%d\n",value);
				s->timer_cntrl_reg = value;
				if((s->timer_cntrl_reg & 0x3) == 0x1) {
					timer_reload(s, 1);
				}
				if((s->timer_cntrl_reg & 0x1) == 0) {
					timer_stop(s);
				}
				printf("Timer Clock Enable update%d\n",s->timer_cntrl_reg);
				/*
				if(value & 0x1)
				{
					s->timer_cntrl_reg |= value & 0x1; 
				} else if(!(value & 0x1)) 
						{
							s->timer_cntrl_reg &= ~((uint32_t)1);
						}
				if(value & 0x2)
				{
					s->timer_cntrl_reg |= value & 0x2;		
				} else if(!(value & 0x2))
						{
							s->timer_cntrl_reg &= ~((uint32_t)(1<<1));
							// Starting Clock Ticks
							timer_reload(s, 1);
						}
				*/
				break;
		case 0x08:
				s->timer_counter = value;
				break;
		case 0x0c:
				s->prescale_reg = value;
				break;
		case 0x10:
				s->prescale_counter = value;
				break;
		case 0x14:
				s->match_cntrl_reg = value;
				break;
		case 0x18:
				s->match_reg[0] = value;
				break;
		case 0x1c:
				s->match_reg[1] = value;
				break;
		case 0x20:
				s->match_reg[2] = value;
				break;
		case 0x24:
				s->match_reg[3] = value;
				break;
		case 0x28:
				s->cap_cntrl_reg |= value & ((1<<6) - 1);	
				break;
		case 0x2c:
				printf("Cannot write to capture register 0\n");
				break;
		case 0x30:
				printf("Cannot write to capture register 1\n");
				break;
		case 0x3c:
				hw_error("External Match Not Emulated\n");
				// Not emulated
				return;
		case 0x70:
				//if (value && ~0x3)
				//	return;
				s->count_cntrl_reg |= (value & 0x3);	
				s->count_cntrl_reg |= (value & (0x3 << 2));
				break;
	}
	timer_update_irq(s);
}

static CPUReadMemoryFunc * const timer_readfn[] = {
	timer_read,
	timer_read,
	timer_read
};

static CPUWriteMemoryFunc * const timer_writefn[] = {
	timer_write,
	timer_write,
	timer_write
};

static void mbed_timer_tick(void *opaque)
{
	//printf("One Clock Tick\n");
	timer_state *s = (timer_state*) opaque;
	// Check if timer/counter is enabled
	if (!(s->timer_cntrl_reg & 0x1)) {
		printf("Counter not enabled:0x%x\n",s->timer_cntrl_reg);
		return;
	}
	// Reset Timer Counter and Prescale Counter till TCR[1] is not zero
	if (s->timer_cntrl_reg & 0x2) {
		s->timer_counter = 0;
		s->prescale_counter = 0;
		printf("Reset Timersn\n");
		return;
	}
	if((s->count_cntrl_reg & 0x3) == 0) {
	// printf("Incrementing prescale counter\n");
		// Timer Mode
		// Increment Prescale counter
		//s->prescale_counter++;
		// Check whether it's value reached prescale register
		//if(s->prescale_counter > s->prescale_reg) {
			// Increment Timer Counter by on
			s->timer_counter +=	300;
			s->prescale_counter = 0;
			if(s->timer_counter >= s->match_reg[0]) {
				if(s->match_cntrl_reg & 0x2) {
					s->timer_counter = 0;
				}
				if(s->match_cntrl_reg & 0x1) {
					qemu_irq_pulse(s->irq);
				}
				if(s->match_cntrl_reg & 0x4)
				{
					timer_stop(s);
					return;
				}
				// TODO(gdrane) Stopping pc and tc and setting tcr[0] to 0 
				// call timer_stop
			} else if(s->timer_counter >= s->match_reg[1]) {	
				if(s->match_cntrl_reg & (1<<4))
				{
					s->timer_counter = 0;
				}
				if(s->match_cntrl_reg & (1<<3))
				{
					qemu_irq_pulse(s->irq);
				}
				if(s->match_cntrl_reg & (1<<5))
				{
					timer_stop(s);
					return;
				}
			} else if(s->timer_counter >= s->match_reg[2]) {
				if(s->match_cntrl_reg & (1<<7))
				{
					s->timer_counter = 0;
				}
				if(s->match_cntrl_reg & (1<<6))
				{
					qemu_irq_pulse(s->irq);
				}
				if(s->match_cntrl_reg & (1<<8))
				{
					timer_stop(s);
					return;
				}
			} else if(s->timer_counter >= s->match_reg[3]) {
				if(s->match_cntrl_reg & (1<<10))
				{
					s->timer_counter = 0;
				}
				if(s->match_cntrl_reg & (1<<9))
				{
					qemu_irq_pulse(s->irq);
				}
				if(s->match_cntrl_reg & (1<<11))
				{
					timer_stop(s);
					return;
				}
			}
		//}
		timer_reload(s, 0);
	} else {
		// Counter Mode
		// Not Implemented
	}
	timer_update_irq(s);
}

static void mbed_timer_reset(timer_state *s)
{
	s->intr_reg = 0;
	s->intr_mask = 0;
 	s->timer_cntrl_reg = 0;
	s->timer_counter = 0;
 	s->prescale_reg = 0;
 	s->prescale_counter = 0;
 	s->match_cntrl_reg = 0;
 	s->match_reg[0] = 0;
 	s->match_reg[1] = 0;
 	s->match_reg[2] = 0;
 	s->match_reg[3] = 0;
 	s->cap_cntrl_reg = 0;
 	s->capture_reg[0] = 0;
 	s->capture_reg[1] = 0;
 	s->external_match_reg = 0;
 	s->count_cntrl_reg = 0;
	s->tick = 0;
}

static int mbed_timer_init(SysBusDevice *dev)
{
	int iomemtype;
	timer_state *s = FROM_SYSBUS(timer_state, dev);
	sysbus_init_irq(dev, &s->irq);
	mbed_timer_reset(s);
	// wrong For sending interrupts on match
	 //qdev_init_gpio_out(&dev->qdev, &s->match_trigger, 1);
	// TODO(gdrane): Add incoming interrupt for capture register
	iomemtype = cpu_register_io_memory(timer_readfn,
										timer_writefn, s,
										DEVICE_NATIVE_ENDIAN);
	sysbus_init_mmio(dev, 0x4000, iomemtype);
	s->timer = qemu_new_timer_ns(vm_clock, mbed_timer_tick, s);
	return 0;
}

/*================UART========================*/

struct mbed_uart_state {
	/*uint32_t rbr;
	uint32_t thr;
	uint32_t dll;
	uint32_t dlm;
	uint32_t ier;
	uint32_t iir;
	uint32_t fcr;
	uint32_t lcr;
	uint32_t lsr;
	uint32_t scr;
	uint32_t acr;
	uint32_t icr;
	uint32_t fdr;
	uint32_t ter;
	uint32_t dlab;
	uint8_t rxfifo[2];
	uint16_t txfifo[2];
	uint8_t rx_head, rx_tail, rx_count;
	uint8_t tx_head, tx_tail, tx_count;
	uint8_t trigger_level;
	CharDriverState *chr;
	*/
	SerialState *serial;
	qemu_irq irq;
};

static void mbed_uart_reset(struct mbed_uart_state *s)
{
	/*s->dll = 1;
	s->dlm = 0;
	s->ier= 0;
	s->iir = 1;
	s->lcr = 0;
	s->lsr = 0x60;
	s->acr = 0;
	s->icr = 0;
	s->fdr = 0x10;
	s->ter = 0x80;
	s->rx_tail = s->rx_tail = 0;
	s->rx_count = s->tx_count = 0;
	s->rx_head = s->tx_head = 0;
	s->trigger_level = 1;
*/
}

/*static void reset_rx_fifo(struct mbed_uart_state *s)
{
	s->rx_head = s->rx_tail = 0;
	s->rx_count = 0;
}

static void reset_tx_fifo(struct mbed_uart_state *s)
{
	s->tx_head = s->tx_tail = 0;
	s->tx_count = 0;
}

static uint32_t mbed_uart_read(void *opaque, target_phys_addr_t offset)
{
	struct mbed_uart_state *s = (struct mbed_uart_state *) opaque;

	switch(offset)
	{
		case 0x00:
				if (!(s->lcr & (1 << 7)))
					return s->rbr;
				return s->dll;
		case 0x04:
				if (s->lcr & (1 << 7))
						return s->dlm;
				return s->ier;
		case 0x08:
				return s->iir;
	 	case 0x0c:
				return s->lcr;
		case 0x14:
				return s->lsr;
		case 0x1c:
				return s->scr;
		case 0x20:
				return s->acr;
		case 0x24:
				return s->icr;
		case 0x28:
				return s->fdr;
		case 0x30:
				return s->ter;
	}
	return 0;
}

static void mbed_uart_write(void *opaque, target_phys_addr_t offset, uint32_t value)
{
	struct mbed_uart_state *s = (struct mbed_uart_state*) opaque;

	switch(offset)
	{
		case 0x00:
				if(!(s->lcr & (1 << 7)))
					s->thr = value;
				else
					s->dll = value;
				break;
		case 0x04:
				if(!(s->lcr & (1 << 7)))
					s->ier = value;
				else 
					s->dlm = value;
				break;
		case 0x08:
				uint32_t trig_bits;
				s->fcr = value;
				trig_bits = s->fcr >> 6;
				if(trig_bits & 0x1)
					s->trigger_level = 4;
				else if(trig_bits & 0x2)
						s->trigger_level = 8;
					else if(trig_bits & 0x3)
							s->trigger_level = 14;
						else s->trigger_level = 1;
				if((s->fcr >> 1) & 0x1) {
					reset_rx_fifo(s);
					s->fcr &= ~((uint32_t)1 << 0x1);
				}
				if((s->fcr >> 2) & 0x1) {
					reset_tx_fifo(s);
					s->fcr &= ~((uint32_t)1 << 2);
				}
				break;
		case 0xc:
				s->lcr = value;
				break;
		case 0x1c:
				s->scr = value;
				break;
		case 0x20:
				s->acr = value;
				break;
		case 0x24:
				s->icr = value;
				break;
		case 0x28:
				s->fdr = value;
				break;
		case 0x30:
				s->ter = value;
				break;
	}


}

static int mbed_serial_can_receive(void * opaque)
{
	struct mbed_uart_state *s = (struct uart_state *) opaque;
	if(s->fcr & 0x1) {
		return s->rx_count < 16;
	}
}

static void mbed_serial_receive(void *opaque, const uint8_t *buf, int size)
{
	struct mbed_uart_state *s = (struct mbed_uart_state*)opaque;
	int i;
	if(!(s->fcr & 0x1))
		return ;
	if(s->rx_count > 0) {
		s->rbr  = s->rx_fifo[s->head];
		s->head = (s->head + 1)%16;
		s->rx_count -- ;
	}
	for (i = 0;i < size; ++i) {
		if(rx_count < 16) {
			s->rx_fifo[s->tail] = buf[i];
			s->rx_count ++;
			s->tail = (s->tail + 1)%16;
			if (s->rx_count >= s->trigger_level) {
				s->lsr |= 0x1;
				if((s->ier & (1<<0)) || (s->ier & (1<<2))) {
					qemu_irq_raise(s->irq);
				}
			}
		}
	}
}

uint32_t CPUMemoryReadFunc* const mbed_uart_readfn[] = {
	mbed_uart_read,
	mbed_uart_read,
	mbed_uart_read
}

uint32_t CPUMemoryWriteFunc* const mbed_uart_writefn[] = {
	mbed_uart_write,
	mbed_uart_write,
	mbed_uart_write
}

static void mbed_uart_init(SysBusDevice *dev)
{
	printf("MBED UART initialized\n");
	int iomemtype;
	struct mbed_uart_state *s = FROM_SYSBUS(struct uart_state, dev);
	s->
	sysbus_init_irq(dev, &s->irq);
	mbed_uart_reset(s);
	// wrong For sending interrupts on match
	 //qdev_init_gpio_out(&dev->qdev, &s->match_trigger, 1);
	// TODO(gdrane): Add incoming interrupt for capture register
	iomemtype = cpu_register_io_memory(mbed_uart_readfn,
									   mbed_uart_writefn, s,
									   DEVICE_NATIVE_ENDIAN);
	sysbus_init_mmio(dev, 0x4000, iomemtype);
	
	return 0;
}*/

static void mbed_uart_init(target_phys_addr_t base, qemu_irq irq, CharDriverState* chr)
{
	struct mbed_uart_state *s;
	int iomemtype;
	s = qemu_mallocz(sizeof(struct mbed_uart_state));
	s->irq = irq;
	// mbed_uart_reset(s);
	//iomemtype = cpu_register_io_memory(mbed_uart_readfn,
	//								   mbed_uart_writefn, s,
	//								   DEVICE_NATIVE_ENDIAN);
#ifdef TARGET_WORDS_BIGENDIAN
	s->serial = serial_mm_init(base, 2, irq, 9600, qemu_chr_open("mbed_uart", "file:/Users/gaureshrane/abc.txt", NULL), 1, 0);
#else
	if(chr)
		s->serial = serial_mm_init(base , 2, irq, 9600, chr, 1, 0);
#endif
}

/*------------Pin Function And Mode-----------*/
struct pin_connect_block {
	uint32_t pinsel[11];
	uint32_t pinmode[10];
	uint32_t pinmode_od[5];	
	uint32_t i2cpadcfg;
};

static uint32_t pin_connect_block_read(void* opaque, target_phys_addr_t offset)
{
	struct pin_connect_block *s = (struct pin_connect_block*)opaque;
	switch(offset) {
		case 0x00:
				return s->pinsel[0];
		case 0x04:
				return s->pinsel[1];
		case 0x08:
				return s->pinsel[2];
		case 0x0c:
				return s->pinsel[3];
		case 0x10:
				return s->pinsel[4];
		case 0x1c:
				return s->pinsel[7];
		case 0x24:
				return s->pinsel[9];
		case 0x28:
				return s->pinsel[10];
		case 0x40:
				return s->pinmode[0];
		case 0x44:
				return s->pinmode[1];
		case 0x48:
				return s->pinmode[2];
		case 0x4c:
				return s->pinmode[3];
		case 0x50:
				return s->pinmode[4];
		case 0x54:
				return s->pinmode[5];
		case 0x58:
				return s->pinmode[6];
		case 0x5c:
				return s->pinmode[7];
		case 0x64:
				return s->pinmode[9];
		case 0x68:
				return s->pinmode_od[0];
		case 0x6c:
				return s->pinmode_od[1];
		case 0x70:
				return s->pinmode_od[2];
		case 0x74:
				return s->pinmode_od[3];
		case 0x78:
				return s->pinmode_od[4];
		case 0x7c:
				return s->i2cpadcfg;
	}
	return 0;
}

static void pin_connect_block_write(void* opaque, target_phys_addr_t offset, uint32_t value)
{
	struct pin_connect_block *s = (struct pin_connect_block*) opaque;
	switch(offset) {
		case 0x00:
			s->pinsel[0] = value;
			break;
		case 0x04:
			s->pinsel[1] = value;
			break;
		case 0x08:
			s->pinsel[2] = value;
			break;
		case 0x0c:
			s->pinsel[3] = value;
			break;
		case 0x10:
			s->pinsel[4] = value;
			break;
		case 0x1c:
			s->pinsel[7] = value;
			break;
		case 0x24:
			s->pinsel[9] = value;
			break;
		case 0x28:
			s->pinsel[10] = value;
			break;
		case 0x40:
			s->pinmode[0] = value;
			break;
		case 0x44:
			s->pinmode[1] = value;
			break;
		case 0x48:
			s->pinmode[2] = value;
			break;
		case 0x4c:
			s->pinmode[3] = value;
			break;
		case 0x50:
			s->pinmode[4] = value;
			break;
		case 0x54:
			s->pinmode[5] = value;
			break;
		case 0x58:
			s->pinmode[6] = value;
			break;
		case 0x5c:
			s->pinmode[7] = value;
			break;
		case 0x64:
			s->pinmode[9] = value;
			break;
		case 0x68:
			s->pinmode_od[0] = value;
			break;
		case 0x6c:
			s->pinmode_od[1] = value;
			break;
		case 0x70:
			s->pinmode_od[2] = value;
			break;
		case 0x74:
			s->pinmode_od[3] = value;
			break;
		case 0x78:
			s->pinmode_od[4] = value;
			break;
		case 0x7c:
			s->i2cpadcfg = value;
			break;
	}
}

static CPUReadMemoryFunc* const pin_connect_readfn[] = {
	pin_connect_block_read,
	pin_connect_block_read,
	pin_connect_block_read
};

// Pin Connect Block Global
struct pin_connect_block *pconnect = NULL;
// Creating a shared memory for communication of digital out with the outside world
char* pin_shm;
#define SHM_KEY 123456789 // Can be any number
#define SHM_PIN_SIZE 128 //To store the output and provide input to 128 pins 
static CPUWriteMemoryFunc* const pin_connect_writefn[] = {
	pin_connect_block_write,
	pin_connect_block_write,
	pin_connect_block_write
};

static void pin_connect_block_reset(struct pin_connect_block *s)
{
	pconnect->pinmode[0] = pconnect->pinmode[1] = pconnect->pinmode[2] = 0;
	pconnect->pinmode[3] = pconnect->pinmode[4] = pconnect->pinmode[5] = 0;
	pconnect->pinmode[5] = pconnect->pinmode[6] = pconnect->pinmode[7] = 0;
	pconnect->pinmode[9] = pconnect->pinsel[0] = pconnect->pinsel[1] = 0;
	pconnect->pinsel[2] = pconnect->pinsel[3] = pconnect->pinsel[4] = 0;
	pconnect->pinsel[7] = pconnect->pinsel[8] = pconnect->pinsel[9] = 0;
	pconnect->pinsel[10] = 0;
	pconnect->pinmode_od[0] = pconnect->pinmode_od[1] = pconnect->pinmode_od[2] = 0;
	pconnect->pinmode_od[3] = 0;
	pconnect->i2cpadcfg = 0;
}

static void mbed_pin_connect_init(uint32_t base, qemu_irq irq)
{
	int iomemtype;
	pconnect = (struct pin_connect_block*) qemu_mallocz(sizeof(struct pin_connect_block));
	iomemtype = cpu_register_io_memory(pin_connect_readfn,
									   pin_connect_writefn, pconnect,
									   DEVICE_NATIVE_ENDIAN);
	cpu_register_physical_memory(base, 0x00003fff, iomemtype);
	pin_connect_block_reset(pconnect);
}

/*----------GPIO--------------------*/
struct gpio_state {
	SysBusDevice busdev;
	uint8_t fiodir[5][4];
	uint8_t fiopin[5][4];
	uint8_t fiopinpreval[5][4];
	uint8_t fiomask[5][4];
	int lines;
	void* intr_ref;
	qemu_irq irq;
	qemu_irq handler[64];
};

struct gpio_interrupts {
	SysBusDevice busdev;
	uint32_t iointstatus;
	uint32_t iointenr[2];
	uint32_t iointenf[2];
	uint32_t iointstatr[2];
	uint32_t iointstatf[2];
	uint32_t iointclr[2];	
};

static void mbed_gpio_irq_update(struct gpio_state *s)
{
	qemu_irq_raise(s->irq);
}

static void mbed_gpio_handler_update(struct gpio_state *s)
{
	uint8_t diff;
	int i, j;
	for(i = 0; i < 5; ++i) 
	{
		for(j = 0; j < 4; ++j) 
		{
			if ( i != 0 || i != 2)
			{
				continue;
			}

			diff = s->fiopinpreval[i][j] ^ s->fiopin[i][j];
			while(ffs(diff))
			{
				uint8_t setbit = ffs(diff) - 1;
				if(i == 0)
				{
					qemu_set_irq(s->handler[setbit], (s->fiopin[i][j] >> setbit ) & 1);
				} else if(i == 2)
					   {
							qemu_set_irq(s->handler[setbit + 32], (s->fiopin[i][j] >> setbit) & 1);
					   }
				diff &= ~(1 << setbit);
			}
		}
	}
}

static uint32_t mbed_gpio_read_halfw(void *opaque, target_phys_addr_t offset)
{
	struct gpio_state * s = (struct gpio_state*) opaque;
	switch(offset) {
		case 0x00:
			return s->fiodir[0][0] | ((uint16_t)s->fiodir[0][1] << 8);
		case 0x02:
			return s->fiodir[0][2] | ((uint16_t)s->fiodir[0][3] << 8);
		case 0x10:
			return s->fiomask[0][0] | ((uint16_t)s->fiomask[0][1] << 8);
		case 0x12:
			return s->fiomask[0][2] | ((uint16_t)s->fiomask[0][3] << 8);
		case 0x14:
			return s->fiopin[0][0] | ((uint16_t)s->fiopin[0][1] << 8);
		case 0x16:
			return s->fiopin[0][2] | ((uint16_t)s->fiodir[0][3] << 8);
		case 0x18:
			return s->fiopin[0][0] | ((uint16_t)s->fiopin[0][1] << 8);
		case 0x1a:
			return s->fiopin[0][2] | ((uint16_t)s->fiopin[0][3] << 8);
		case 0x1c:
			return s->fiopin[0][0] | ((uint16_t)s->fiopin[0][1] << 8);
		case 0x1e:
			return s->fiopin[0][2] | ((uint16_t)s->fiopin[0][3] << 8);
		case 0x20:
			return s->fiodir[1][0] | ((uint16_t)s->fiodir[1][1] << 8);
		case 0x22:
			return s->fiodir[1][2] | ((uint16_t)s->fiodir[1][3] << 8);
		case 0x30:
			return s->fiomask[1][0] | ((uint16_t)s->fiomask[1][1] << 8);
		case 0x32:
			return s->fiomask[1][2] | ((uint16_t)s->fiomask[1][3] << 8);
		case 0x34:
			return s->fiopin[1][0] | ((uint16_t)s->fiopin[1][1] << 8);
		case 0x36:
			return s->fiopin[1][2] | ((uint16_t)s->fiodir[1][3] << 8);
		case 0x38:
			return s->fiopin[1][0] | ((uint16_t)s->fiopin[1][1] << 8);
		case 0x3a:
			return s->fiopin[1][2] | ((uint16_t)s->fiopin[1][3] << 8);
		case 0x3c:
			return s->fiopin[1][0] | ((uint16_t)s->fiopin[1][1] << 8);
		case 0x3e:
			return s->fiopin[1][2] | ((uint16_t)s->fiopin[1][3] << 8);
		case 0x40:
			return s->fiodir[2][0] | ((uint16_t)s->fiodir[2][1] << 8);
		case 0x42:
			return s->fiodir[2][2] | ((uint16_t)s->fiodir[2][3] << 8);
		case 0x50:
			return s->fiomask[2][0] | ((uint16_t)s->fiomask[2][1] << 8);
		case 0x52:
			return s->fiomask[2][2] | ((uint16_t)s->fiomask[2][3] << 8);
		case 0x54:
			return s->fiopin[2][0] | ((uint16_t)s->fiopin[2][1] << 8);
		case 0x56:
			return s->fiopin[2][2] | ((uint16_t)s->fiodir[2][3] << 8);
		case 0x58:
			return s->fiopin[2][0] | ((uint16_t)s->fiopin[2][1] << 8);
		case 0x5a:
			return s->fiopin[2][2] | ((uint16_t)s->fiopin[2][3] << 8);
		case 0x5c:
			return s->fiopin[2][0] | ((uint16_t)s->fiopin[2][1] << 8);
		case 0x5e:
			return s->fiopin[2][2] | ((uint16_t)s->fiopin[2][3] << 8);
		case 0x60:
			return s->fiodir[3][0] | ((uint16_t)s->fiodir[3][1] << 8);
		case 0x62:
			return s->fiodir[3][2] | ((uint16_t)s->fiodir[3][3] << 8);
		case 0x70:
			return s->fiomask[3][0] | ((uint16_t)s->fiomask[3][1] << 8);
		case 0x72:
			return s->fiomask[3][2] | ((uint16_t)s->fiomask[3][3] << 8);
		case 0x74:
			return s->fiopin[3][0] | ((uint16_t)s->fiopin[3][1] << 8);
		case 0x76:
			return s->fiopin[3][2] | ((uint16_t)s->fiodir[3][3] << 8);
		case 0x78:
			return s->fiopin[3][0] | ((uint16_t)s->fiopin[3][1] << 8);
		case 0x7a:
			return s->fiopin[3][2] | ((uint16_t)s->fiopin[3][3] << 8);
		case 0x7c:
			return s->fiopin[3][0] | ((uint16_t)s->fiopin[3][1] << 8);
		case 0x7e:
			return s->fiopin[3][2] | ((uint16_t)s->fiopin[3][3] << 8);
		case 0x80:
			return s->fiodir[4][0] | ((uint16_t)s->fiodir[4][1] << 8);
		case 0x82:
			return s->fiodir[4][2] | ((uint16_t)s->fiodir[4][3] << 8);
		case 0x90:
			return s->fiomask[4][0] | ((uint16_t)s->fiomask[4][1] << 8);
		case 0x92:
			return s->fiomask[4][2] | ((uint16_t)s->fiomask[4][3] << 8);
		case 0x94:
			return s->fiopin[4][0] | ((uint16_t)s->fiopin[4][1] << 8);
		case 0x96:
			return s->fiopin[4][2] | ((uint16_t)s->fiodir[4][3] << 8);
		case 0x98:
			return s->fiopin[4][0] | ((uint16_t)s->fiopin[4][1] << 8);
		case 0x9a:
			return s->fiopin[4][2] | ((uint16_t)s->fiopin[4][3] << 8);
		case 0x9c:
			return s->fiopin[4][0] | ((uint16_t)s->fiopin[4][1] << 8);
		case 0x9e:
			return s->fiopin[4][2] | ((uint16_t)s->fiopin[4][3] << 8);
	}
	return 0;
}
static uint32_t mbed_gpio_read_byte(void *opaque, target_phys_addr_t offset) {

	struct gpio_state *s = (struct gpio_state*) opaque;
	
	switch(offset) {
		case 0x00:
			return s->fiodir[0][0];
		case 0x01:
			return s->fiodir[0][1];
		case 0x02:
			return s->fiodir[0][2];
		case 0x03:
			return s->fiodir[0][3];
		case 0x10:
			return s->fiomask[0][0];
		case 0x11:
			return s->fiomask[0][1];
		case 0x12:
			return s->fiomask[0][2];
		case 0x13:
			return s->fiomask[0][3];
		case 0x14:
			return s->fiopin[0][0];
		case 0x15:
			return s->fiopin[0][1];
		case 0x16:
			return s->fiopin[0][2];
		case 0x17:
			return s->fiopin[0][3];
		case 0x18:
			return s->fiopin[0][0];
		case 0x19:
			return s->fiopin[0][1];
		case 0x1a:
			return s->fiopin[0][2];
		case 0x1b:
			return s->fiopin[0][3];
		case 0x1c:
			return s->fiopin[0][0];
		case 0x1d:
			return s->fiopin[0][1];
		case 0x1e:
			return s->fiopin[0][2];
		case 0x1f:
			return s->fiopin[0][3];
		case 0x20:
			return s->fiodir[1][0];
		case 0x21:
			return s->fiodir[1][1];
		case 0x22:
			return s->fiodir[1][2];
		case 0x23:
			return s->fiodir[1][3];
		case 0x30:
			return s->fiomask[1][0];
		case 0x31:
			return s->fiomask[1][1];
		case 0x32:
			return s->fiomask[1][2];
		case 0x33:
			return s->fiomask[1][3];
		case 0x34:
			return s->fiopin[1][0];
		case 0x35:
			return s->fiopin[1][1];
		case 0x36:
			return s->fiopin[1][2];
		case 0x37:
			return s->fiopin[1][3];
		case 0x38:
			return s->fiopin[1][0];
		case 0x39:
			return s->fiopin[1][1];
		case 0x3a:
			return s->fiopin[1][2];
		case 0x3b:
			return s->fiopin[1][3];
		case 0x3c:
			return s->fiopin[1][0];
		case 0x3d:
			return s->fiopin[1][1];
		case 0x3e:
			return s->fiopin[1][2];
		case 0x3f:
			return s->fiopin[1][3];
		case 0x40:
			return s->fiodir[2][0];
		case 0x41:
			return s->fiodir[2][1];
		case 0x42:
			return s->fiodir[2][2];
		case 0x43:
			return s->fiodir[2][3];
		case 0x50:
			return s->fiomask[2][0];
		case 0x51:
			return s->fiomask[2][1];
		case 0x52:
			return s->fiomask[2][2];
		case 0x53:
			return s->fiomask[2][3];
		case 0x54:
			return s->fiopin[2][0];
		case 0x55:
			return s->fiopin[2][1];
		case 0x56:
			return s->fiopin[2][2];
		case 0x57:
			return s->fiopin[2][3];
		case 0x58:
			return s->fiopin[2][0];
		case 0x59:
			return s->fiopin[2][1];
		case 0x5a:
			return s->fiopin[2][2];
		case 0x5b:
			return s->fiopin[2][3];
		case 0x5c:
			return s->fiopin[2][0];
		case 0x5d:
			return s->fiopin[2][1];
		case 0x5e:
			return s->fiopin[2][2];
		case 0x5f:
			return s->fiopin[2][3];
		case 0x60:
			return s->fiodir[3][0];
		case 0x61:
			return s->fiodir[3][1];
		case 0x62:
			return s->fiodir[3][2];
		case 0x63:
			return s->fiodir[3][3];
		case 0x70:
			return s->fiomask[3][0];
		case 0x71:
			return s->fiomask[3][1];
		case 0x72:
			return s->fiomask[3][2];
		case 0x73:
			return s->fiomask[3][3];
		case 0x74:
			return s->fiopin[3][0];
		case 0x75:
			return s->fiopin[3][1];
		case 0x76:
			return s->fiopin[3][2];
		case 0x77:
			return s->fiopin[3][3];
		case 0x78:
			return s->fiopin[3][0];
		case 0x79:
			return s->fiopin[3][1];
		case 0x7a:
			return s->fiopin[3][2];
		case 0x7b:
			return s->fiopin[3][3];
		case 0x7c:
			return s->fiopin[3][0];
		case 0x7d:
			return s->fiopin[3][1];
		case 0x7e:
			return s->fiopin[3][2];
		case 0x7f:
			return s->fiopin[3][3];
		case 0x80:
			return s->fiodir[4][0];
		case 0x81:
			return s->fiodir[4][1];
		case 0x82:
			return s->fiodir[4][2];
		case 0x83:
			return s->fiodir[4][3];
		case 0x90:
			return s->fiomask[4][0];
		case 0x91:
			return s->fiomask[4][1];
		case 0x92:
			return s->fiomask[4][2];
		case 0x93:
			return s->fiomask[4][3];
		case 0x94:
			return s->fiopin[4][0];
		case 0x95:
			return s->fiopin[4][1];
		case 0x96:
			return s->fiopin[4][2];
		case 0x97:
			return s->fiopin[4][3];
		case 0x98:
			return s->fiopin[4][0];
		case 0x99:
			return s->fiopin[4][1];
		case 0x9a:
			return s->fiopin[4][2];
		case 0x9b:
			return s->fiopin[4][3];
		case 0x9c:
			return s->fiopin[4][0];
		case 0x9d:
			return s->fiopin[4][1];
		case 0x9e:
			return s->fiopin[4][2];
		case 0x9f:
			return s->fiopin[4][3];
	}
	return 0;
}

static uint32_t mbed_gpio_read(void *opaque, target_phys_addr_t offset)
{
	struct gpio_state *s = (struct gpio_state*) opaque;
	uint32_t temp;
	uint8_t i;
	printf("\nReading gpio\n");
	switch(offset)
	{
		case 0x00:
			temp = 0;
			for(i = 0; i < 4; ++i)
			{
				temp |= s->fiodir[0][i]  << (8 * i);
			}
			return temp;
		case 0x20: 
			temp = 0;
			for(i = 0; i < 4; ++i)
			{
				temp |= s->fiodir[1][i] << (8 * i);
			}
			return temp;
		case 0x40:
			temp = 0;
			for(i = 0; i < 4; ++i)
			{
				temp |= s->fiodir[2][i] << (8 * i);
			}
			return temp;
		case 0x60:
			temp = 0;
			for(i = 0; i < 4; ++i)
			{
				temp |= s->fiodir[3][i] << (8 * i);
			}
			return temp;
		case 0x80:
			temp = 0;
			for(i = 0; i < 4; ++i)
			{
				temp |= s->fiodir[4][i] << (8 * i);
			}
			return temp;
		case 0x18:
			temp = 0;
			for(i = 0;i < 4; ++i)
			{
				temp |= s->fiopin[0][i] << (8 * i);
			}
			return temp;
		case 0x38:
			temp = 0;
			for(i = 0; i < 4; ++i)
			{
				temp |= s->fiopin[1][i] << (8*i);
			}
			return temp;
		case 0x58:
			temp = 0;
			for(i = 0; i < 4; ++i)
			{
				temp |= s->fiopin[2][i] << (8*i);
			}
			return temp;
		case 0x78:
			temp = 0;
			for(i = 0; i < 4; ++i)
			{
				temp |= s->fiopin[3][i] << (8*i);
			}
			return temp;
		case 0x98:
			temp = 0;
			for(i = 0; i < 4; ++i)
			{
				temp |= s->fiopin[4][i] << (8*i);
			}
			return temp;
		case 0x1c:
		case 0x3c:
		case 0x5c:
		case 0x7c:
		case 0x9c:
			printf("Cannot Write to clear registers\n");
			break;
		case 0x10:
			temp = 0;
			for(i = 0; i < 4; ++i)
			{
				temp |= s->fiomask[0][i] << (8*i);
			}
			return temp;
		case 0x30:
			temp = 0;
			for(i = 0; i < 4; ++i)
			{
				temp |= s->fiomask[1][i] << (8*i);
			}
			return temp;
		case 0x50:
			temp = 0;
			for(i = 0; i < 4; ++i)
			{
				temp |= s->fiomask[2][i] << (8*i);
			}
			return temp;
		case 0x70:
			temp = 0;
			for(i = 0; i < 4; ++i)
			{
				temp |= s->fiomask[3][i] << (8*i);
			}
			return temp;
		case 0x90:
			temp = 0;
			for(i = 0; i < 4; ++i)
			{
				temp |= s->fiomask[4][i] << (8*i);
			}
			return temp;
		case 0x14:
			temp = 0;
			for(i = 0; i < 4; ++i)
			{
				temp |= s->fiopin[0][i] << (8*i);
			}
			return temp;
		case 0x34:
			temp = 0;
			for(i = 0; i < 4; ++i)
			{
				temp |= s->fiopin[1][i] << (8*i);
			}
			return temp;
		case 0x54:
			temp = 0;
			for(i = 0; i < 4; ++i)
			{
				temp |= s->fiopin[2][i] << (8*i);
			}
			return temp;
		case 0x74:
			temp = 0;
			for(i = 0; i < 4; ++i)
			{
				temp |= s->fiopin[3][i] << (8*i);
			}
			return temp;
		case 0x94:
			temp = 0;
			for(i = 0; i < 4; ++i)
			{
				temp |= s->fiopin[4][i] << (8*i);
			}
			return temp;
	}		
	return 0;
}

static uint8_t chkmask(struct gpio_state* s, uint8_t port, uint8_t byteno, uint8_t isbyte, uint32_t value)
{
	if(isbyte == 1)
	{
		return !(s->fiomask[port][byteno] & (uint8_t)value);
	} else if(isbyte == 0) {
		return !((s->fiomask[port][byteno] & (value & 0xff)) | (s->fiomask[port][byteno + 1] & (uint8_t)(value >> 8)));
	} else if(isbyte == 2) {
		int i;
		for(i = 0; i < 4; ++i)
		{
			if(s->fiomask[port][i] & value & 0xff) {
				printf("\nMASKED\n");
				return 0;
			}
			value >>= 8;
		}
	}
	printf("\nNOT MASKED\n");
	return 1;
}

static uint8_t chkdir(struct gpio_state *s, uint8_t port, uint8_t byteno, uint8_t isbyte, uint32_t value)
{
	if(isbyte == 1)
	{
		return s->fiodir[port][byteno] & (uint8_t) value;
	} else if(isbyte == 0){
		return ((s->fiodir[port][byteno] & (value & 0xff)) | (s->fiodir[port][byteno + 1] & (value >> 8)));
	} else if(isbyte == 2) {
		int i;
		for(i = 0; i < 4; ++i)
		{
			printf("\n%d %d", s->fiodir[port][i], value); 
			if(s->fiodir[port][i] & (value & 0xff)) {
				printf("OUTPUT DIRECTION");
				return 1;
			}
			value >>= 8;
		}
	}
	printf("INPUT DIRECTION");
	return 0;
}


static void mbed_gpio_write(void *opaque, target_phys_addr_t offset, uint32_t value)
{
	struct gpio_state * s= (struct gpio_state*) opaque;
	int  i;
	printf("\nWrite GPIO \n");
	switch(offset)
	{
	case 0x00:
			for(i = 0; i < 4; ++i)
			{
				s->fiodir[0][i] = value  & ((1<<8) -1);
				value >>= 8;
			}
			break;
	case 0x20:
			for(i = 0; i < 4; ++i)
			{
				s->fiodir[1][i] = value  & ((1<<8) -1);
				value >>= 8;
			}
			break;
	case 0x40:
			for(i = 0; i < 4; ++i)
			{
				s->fiodir[2][i] = value  & ((1<<8) -1);
				value >>= 8;
			}
			break;
	case 0x60:
			for(i = 0; i < 4; ++i)
			{
				s->fiodir[3][i] = value  & ((1<<8) -1);
				value >>= 8;
			}
			break;
	case 0x80:
			for(i = 0; i < 4; ++i)
			{
				s->fiodir[4][i] = value  & ((1<<8) -1);
				value >>= 8;
			}
			break;
 	case 0x18:	
		if(chkmask(s, 0, 0, 2, value) && chkdir(s, 0, 0, 2, value))
		{
			for(i = 0;i < 4; ++i)
			{
				s->fiopin[0][i] |= (value & ((1<<8) - 1));
				value >>= 8;
			}
			mbed_gpio_handler_update(s);
		}
		break;
	case 0x38:
		printf("Set Value:%d ", value);
		if(chkmask(s, 1, 0, 2, value) && chkdir(s, 1, 0, 2, value))
		{
			for(i = 0;i < 4; ++i)
			{
				s->fiopin[1][i] |= (value & ((1<<8) - 1));
				value >>= 8;
			}
			mbed_gpio_handler_update(s);
		}
		break;
	case 0x58:
		if(chkmask(s, 2, 0, 2, value) && chkdir(s, 2, 0, 2, value))
		{
			for(i = 0;i < 4; ++i)
			{
				s->fiopin[2][i] |= (value & ((1<<8) - 1));
				value >>= 8;
			}
			mbed_gpio_handler_update(s);
		}
		break;
	case 0x78:
		if(chkmask(s, 3, 0, 2, value) && chkdir(s, 3, 0, 2, value))
		{
			for(i = 0;i < 4; ++i)
			{
				s->fiopin[3][i] |= (value & ((1<<8) - 1));
				value >>= 8;
			}
			mbed_gpio_handler_update(s);
		}
		break;
	case 0x98:
		if(chkmask(s, 4, 0, 2, value) && chkdir(s, 4, 0, 2, value))
		{
			for(i = 0;i < 4; ++i)
			{
				s->fiopin[4][i] |= (value & ((1<<8) - 1));
				value >>= 8;
			}
			mbed_gpio_handler_update(s);
		}
		break;
	case 0x1c:
		if(chkmask(s, 0, 0, 2, value) && chkdir(s, 0, 0, 2, value))
		{
			for(i = 0;i < 4; ++i)
			{
				s->fiopin[0][i] &= ~(value & ((1<<8) - 1));
				value >>= 8;
			}
			mbed_gpio_handler_update(s);
		}
		break;
	case 0x3c:
		printf("Clear Value:%d ", value);
		if(chkmask(s, 1, 0, 2, value) && chkdir(s, 1, 0, 2, value))
		{
			for(i = 0;i < 4; ++i)
			{
				s->fiopin[1][i] &= ~(value & ((1<<8) - 1));
				value >>= 8;
			}
			mbed_gpio_handler_update(s);
		}
		break;
	case 0x5c:
		if(chkmask(s, 2, 0, 2, value) && chkdir(s, 2, 0, 2, value))
		{
			for(i = 0;i < 4; ++i)
			{
				s->fiopin[2][i] &= ~(value & ((1<<8) - 1));
				value >>= 8;
			}
			mbed_gpio_handler_update(s);
		}
		break;
	case 0x7c:
		if(chkmask(s, 3, 0, 2, value) && chkdir(s, 3, 0, 2, value))
		{
			for(i = 0;i < 4; ++i)
			{
				s->fiopin[3][i] &= ~(value & ((1<<8) - 1));
				value >>= 8;
			}
			mbed_gpio_handler_update(s);
		}
		break;
	case 0x9c:
		if(chkmask(s, 4, 0, 2, value) && chkdir(s, 4, 0, 2, value))
		{
			for(i = 0;i < 4; ++i)
			{
				s->fiopin[4][i] &= (value & ((1<<8) - 1));
				value >>= 8;
			}
			mbed_gpio_handler_update(s);
		}
		break;
	case 0x10:
		for(i = 0;i < 4; ++i)
		{
			s->fiomask[0][i] = (value & ((1<<8) - 1));
			value >>= 8;
		}
		break;
	case 0x30:
		for(i = 0;i < 4; ++i)
		{
			s->fiomask[1][i] |= (value & ((1<<8) - 1));
			value >>= 8;
		}
		break;
	case 0x50:
		for(i = 0;i < 4; ++i)
		{
			s->fiomask[2][i] |= (value & ((1<<8) - 1));
			value >>= 8;
		}
		break;
	case 0x70:
		for(i = 0;i < 4; ++i)
		{
			s->fiomask[3][i] |= (value & ((1<<8) - 1));
			value >>= 8;
		}
		break;
	case 0x90:
		for(i = 0;i < 4; ++i)
		{
			s->fiomask[4][i] |= (value & ((1<<8) - 1));
			value >>= 8;
		}
		break;
	case 0x14:
		for(i = 0;i < 4; ++i)
		{
			s->fiopin[0][i] |= (value & ((1<<8) - 1));
			value >>= 8;
		}
		break;
	case 0x34:
		for(i = 0;i < 4; ++i)
		{
			s->fiopin[1][i] |= (value & ((1<<8) - 1));
			value >>= 8;
		}
		break;
	case 0x54:
		for(i = 0;i < 4; ++i)
		{
			s->fiopin[2][i] |= (value & ((1<<8) - 1));
			value >>= 8;
		}
		break;
	case 0x74:
		for(i = 0;i < 4; ++i)
		{
			s->fiopin[3][i] |= (value & ((1<<8) - 1));
			value >>= 8;
		}
		break;
	case 0x94:
		for(i = 0;i < 4; ++i)
		{
			s->fiopin[4][i] |= (value & ((1<<8) - 1));
			value >>= 8;
		}
		break;
	}
}

static void mbed_gpio_write_halfw(void * opaque, target_phys_addr_t offset, uint32_t value)
{
	printf("Writing GPIO half word\n");
	struct gpio_state *s = (struct gpio_state*) opaque;
	switch(offset)
	{
		case 0x00:
			s->fiodir[0][0] = value & ((uint16_t)0xff);
		case 0x01:
			s->fiodir[0][1] = value >> 8;
			break;
		case 0x02:
			s->fiodir[0][2] = value & ((uint16_t)0xff);
		case 0x03:
			s->fiodir[0][3] = value >> 8;
			break;
		case 0x10:
			s->fiomask[0][0] = value & ((uint16_t)0xff);
		case 0x11:
			s->fiomask[0][1] = value >> 8;
			break;
		case 0x12:
			s->fiomask[0][2] = value & ((uint16_t)0xff);
		case 0x13:
			s->fiomask[0][3] = value >> 8;
			break;
		case 0x14:
			{
				s->fiopinpreval[0][0] = s->fiopin[0][0];
				s->fiopin[0][0] = value & ((uint16_t)0xff);
				s->fiopinpreval[0][1] = s->fiopin[0][1];
				s->fiopin[0][1] = value >> 8;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x16:
			{
				s->fiopinpreval[0][2] = s->fiopin[0][2];
				s->fiopin[0][2] = value & ((uint16_t)0xff);
				s->fiopinpreval[0][3] = s->fiopin[0][3];
				s->fiopin[0][3] = value >> 8;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x18:
			if(chkmask(s, 0, 0, 0, value) && chkdir(s, 0, 0, 0, value))
			{
				s->fiopinpreval[0][0] = s->fiopin[0][0];
				s->fiopin[0][0] |= value & ((uint16_t)0xff);
				s->fiopinpreval[0][1] = s->fiopin[0][1];
				s->fiopin[0][1] |= value >> 8;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x1a:
			if(chkmask(s, 0, 2, 0, value) && chkdir(s, 0, 2, 0, value))
			{
				s->fiopinpreval[0][2] = s->fiopin[0][2];
				s->fiopin[0][2] |= value & ((uint16_t)0xff);
				s->fiopinpreval[0][3] = s->fiopin[0][3];
				s->fiopin[0][3] |= value >> 8;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x1c:
			if(chkmask(s, 0, 0, 0, value) && chkdir(s, 0, 0, 0, value))
			{
				s->fiopinpreval[0][0] = s->fiopin[0][0];
				s->fiopin[0][0] &= ~(value & ((uint16_t)0xff));
				s->fiopinpreval[0][1] = s->fiopin[0][1];
				s->fiopin[0][1] &= ~(value >> 8);
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x1e:
			if(chkmask(s, 0, 2, 0, value) && chkdir(s, 0, 2, 0, value))
			{
				s->fiopinpreval[0][2] = s->fiopin[0][2];
				s->fiopin[0][2] = ~(value & ((uint16_t)0xff));
				s->fiopinpreval[0][3] = s->fiopin[0][3];
				s->fiopin[0][3] = ~(value >> 8);
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x20:
			s->fiodir[1][0] = value & ((uint16_t)0xff);
		case 0x21:
			s->fiodir[1][1] = value >> 8;
			break;
		case 0x22:
			s->fiodir[1][2] = value & ((uint16_t)0xff);
		case 0x23:
			s->fiodir[1][3] = value >> 8;
			break;
		case 0x30:
			s->fiomask[1][0] = value & ((uint16_t)0xff);
		case 0x31:
			s->fiomask[1][1] = value >> 8;
			break;
		case 0x32:
			s->fiomask[1][2] = value & ((uint16_t)0xff);
		case 0x33:
			s->fiomask[1][3] = value >> 8;
			break;
		case 0x34:
			if(chkmask(s, 1, 0, 0, value) && chkdir(s, 1, 0, 0, value))
			{
				s->fiopinpreval[1][0] = s->fiopin[1][0];
				s->fiopin[1][0] = value & ((uint16_t)0xff);
				s->fiopinpreval[1][1] = s->fiopin[1][1];
				s->fiopin[1][1] = value >> 8;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x36:
			if(chkmask(s, 1, 2, 0, value) && chkdir(s, 1, 2, 0, value))
			{
				s->fiopinpreval[1][2] = s->fiopin[1][2];
				s->fiopin[1][2] = value & ((uint16_t)0xff);
				s->fiopinpreval[1][3] = s->fiopin[1][3];
				s->fiopin[1][3] = value >> 8;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x38:
			if(chkmask(s, 1, 0, 0, value) && chkdir(s, 1, 0, 0, value))
			{	
				s->fiopinpreval[1][0] = s->fiopin[1][0];
				s->fiopin[1][0] |= value & ((uint16_t)0xff);
				s->fiopinpreval[1][1] = s->fiopin[1][1];
				s->fiopin[1][1] |= value >> 8;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x3a:
			if(chkmask(s, 1, 2, 0, value) && chkdir(s, 1, 2, 0, value))
			{	
				s->fiopinpreval[1][2] = s->fiopin[1][2];
				s->fiopin[1][2] |= value & ((uint16_t)0xff);
				s->fiopinpreval[1][3] = s->fiopin[1][3];
				s->fiopin[1][3] |= value >> 8;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x3c:
			if(chkmask(s, 1, 0, 0, value) && chkdir(s, 1, 0, 0, value))
			{
				s->fiopinpreval[1][0] = s->fiopin[1][0];
				s->fiopin[1][0] &= ~(value & ((uint16_t)0xff));
				s->fiopinpreval[1][1] = s->fiopin[1][1];
				s->fiopin[1][1] &= ~(value >> 8);
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x3e:
			if(chkmask(s, 1, 2, 0, value) && chkdir(s, 1, 2, 0, value))
			{
				s->fiopinpreval[1][2] = s->fiopin[1][2];
				s->fiopin[1][2] &= ~(value & ((uint16_t)0xff));
				s->fiopinpreval[1][3] = s->fiopin[1][3];
				s->fiopin[1][3] &= ~(value >> 8);
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x40:
			s->fiodir[2][0] = value & ((uint16_t)0xff);
		case 0x41:
			s->fiodir[2][1] = value >> 8;
			break;
		case 0x42:
			s->fiodir[2][2] = value & ((uint16_t)0xff);
		case 0x43:
			s->fiodir[2][3] = value >> 8;
			break;
		case 0x50:
			s->fiomask[2][0] = value & ((uint16_t)0xff);
		case 0x51:
			s->fiomask[2][1] = value >> 8;
			break;
		case 0x52:
			s->fiomask[2][2] = value & ((uint16_t)0xff);
		case 0x53:
			s->fiomask[2][3] = value >> 8;
			break;
		case 0x54:
			{
				s->fiopinpreval[2][0] = s->fiopin[2][0];
				s->fiopin[2][0] = value & ((uint16_t)0xff);
				s->fiopinpreval[2][1] = s->fiopin[2][1];
				s->fiopin[2][1] = value >> 8;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x56:
			{
				s->fiopinpreval[2][2] = s->fiopin[2][2];
				s->fiopin[2][2] = value & ((uint16_t)0xff);
				s->fiopinpreval[2][3] = s->fiopin[2][3];
				s->fiopin[2][3] = value >> 8;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x58:
			if(chkmask(s, 2, 0, 0, value) && chkdir(s, 2, 0, 0, value))
			{
				s->fiopinpreval[2][0] = s->fiopin[2][0];
				s->fiopin[2][0] |= value & ((uint16_t)0xff);
				s->fiopinpreval[2][1] = s->fiopin[2][1];
				s->fiopin[2][1] |= value >> 8;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x5a:
			if(chkmask(s, 2, 2, 0, value) && chkdir(s, 2, 2, 0, value))
			{
				s->fiopinpreval[2][2] = s->fiopin[2][2];
				s->fiopin[2][2] &= ~(value & ((uint16_t)0xff));
				s->fiopinpreval[2][3] = s->fiopin[2][3];
				s->fiopin[2][3] &= ~(value >> 8);
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x5c:
			if(chkmask(s, 2, 0, 0, value) && chkdir(s, 2, 0, 0, value))
			{
				s->fiopinpreval[2][0] = s->fiopin[2][0];
				s->fiopin[2][0] &= ~(value & ((uint16_t)0xff));
				s->fiopinpreval[2][1] = s->fiopin[2][1];
				s->fiopin[2][1] &= ~(value >> 8);
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x5e:
			if(chkmask(s, 2, 2, 0, value) && chkdir(s, 2, 2, 0, value))
			{
				s->fiopinpreval[2][2] = s->fiopin[2][2];
				s->fiopin[2][2] &= ~(value & ((uint16_t)0xff));
				s->fiopinpreval[2][3] = s->fiopin[2][3];
				s->fiopin[2][3] &= ~(value >> 8);
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x60:
			s->fiodir[3][0] = value & ((uint16_t)0xff);
		case 0x61:
			s->fiodir[3][1] = value >> 8;
			break;
		case 0x62:
			s->fiodir[3][2] = value & ((uint16_t)0xff);
		case 0x63:
			s->fiodir[3][3] = value >> 8;
			break;
		case 0x70:
			s->fiomask[3][0] = value & ((uint16_t)0xff);
		case 0x71:
			s->fiomask[3][1] = value >> 8;
			break;
		case 0x72:
			s->fiomask[3][2] = value & ((uint16_t)0xff);
		case 0x73:
			s->fiomask[3][3] = value >> 8;
			break;
		case 0x74:
			{
				s->fiopinpreval[3][0] = s->fiopin[3][0];
				s->fiopin[3][0] = value & ((uint16_t)0xff);
				s->fiopinpreval[3][1] = s->fiopin[3][1];
				s->fiopin[3][1] = value >> 8;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x76:
			{
				s->fiopinpreval[3][2] = s->fiopin[3][2];
				s->fiopin[3][2] = value & ((uint16_t)0xff);
				s->fiopinpreval[3][3] = s->fiopin[3][3];
				s->fiopin[3][3] = value >> 8;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x78:
			if(chkmask(s, 3, 0, 0, value) && chkdir(s, 3, 0, 0, value))
			{		
				s->fiopinpreval[3][0] = s->fiopin[3][0];
				s->fiopin[3][0] |= value & ((uint16_t)0xff);
				s->fiopinpreval[3][1] = s->fiopin[3][1];
				s->fiopin[3][1] |= value >> 8;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x7a:
			if(chkmask(s, 3, 2, 0, value) && chkdir(s, 3, 2, 0, value))
			{
				s->fiopinpreval[3][2] = s->fiopin[3][2];
				s->fiopin[3][2] |= value & ((uint16_t)0xff);
				s->fiopinpreval[3][3] = s->fiopin[3][3];
				s->fiopin[3][3] |= value >> 8;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x7c:
			if(chkmask(s, 3, 0, 0, value) && chkdir(s, 3, 0, 0, value))
			{
				s->fiopinpreval[3][0] = s->fiopin[3][0];
				s->fiopin[3][0] &= ~(value & ((uint16_t)0xff));
				s->fiopinpreval[3][1] = s->fiopin[3][1];
				s->fiopin[3][1] &= ~(value >> 8);
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x7e:
			if(chkmask(s, 3, 2, 0, value) && chkdir(s, 3, 2, 0, value))
			{
				s->fiopinpreval[3][2] = s->fiopin[3][2];
				s->fiopin[3][2] &= ~(value & ((uint16_t)0xff));
				s->fiopinpreval[3][3] = s->fiopin[3][3];
				s->fiopin[3][3] &= ~(value >> 8);
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x80:
			s->fiodir[4][0] = value & ((uint16_t)0xff);
		case 0X81:
			s->fiodir[4][1] = value >> 8;
			break;
		case 0x82:
			s->fiodir[4][2] = value & ((uint16_t)0xff);
		case 0x83:
			s->fiodir[4][3] = value >> 8;
			break;
		case 0x90:
			s->fiomask[4][0] = value & ((uint16_t)0xff);
		case 0x91:
			s->fiomask[4][1] = value >> 8;
			break;
		case 0x92:
			s->fiomask[4][2] = value & ((uint16_t)0xff);
		case 0x93:
			s->fiomask[4][3] = value >> 8;
			break;
		case 0x94:
			{
				s->fiopinpreval[4][0] = s->fiopin[4][0];
				s->fiopin[4][0] = value & ((uint16_t)0xff);
				s->fiopinpreval[4][1] = s->fiopin[4][1];
				s->fiopin[4][1] = value >> 8;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x96:
			{
				s->fiopinpreval[4][2] = s->fiopin[4][2];
				s->fiopin[4][2] = value & ((uint16_t)0xff);
				s->fiopinpreval[4][3] = s->fiopin[4][3];
				s->fiopin[4][3] = value >> 8;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x98:
			if(chkmask(s, 4, 0, 0, value) && chkdir(s, 4, 0, 0, value))
			{
				s->fiopinpreval[4][0] = s->fiopin[4][0];
				s->fiopin[4][0] |= value & ((uint16_t)0xff);
				s->fiopinpreval[4][1] = s->fiopin[4][1];
				s->fiopin[4][1] |= value >> 8;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x9a:
			if(chkmask(s, 4, 2, 0, value) && chkdir(s, 4, 2, 0, value))
			{
				s->fiopinpreval[4][2] = s->fiopin[4][2];
				s->fiopin[4][2] |= value & ((uint16_t)0xff);
				s->fiopinpreval[4][3] = s->fiopin[4][3];
				s->fiopin[4][3] |= value >> 8;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x9c:
			if(chkmask(s, 4, 0, 0, value) && chkdir(s, 4, 0, 0, value))
			{
				s->fiopinpreval[4][0] = s->fiopin[4][0];
				s->fiopin[4][0] &= ~(value & ((uint16_t)0xff));
				s->fiopinpreval[4][1] = s->fiopin[4][1];
				s->fiopin[4][1] &= ~(value >> 8);
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x9e:
			if(chkmask(s, 4, 2, 0, value) && chkdir(s, 4, 2, 0, value))
			{
				s->fiopinpreval[4][2] = s->fiopin[4][2];
				s->fiopin[4][2] &= ~(value & ((uint16_t)0xff));
				s->fiopinpreval[4][3] = s->fiopin[4][3];
				s->fiopin[4][3] = ~(value >> 8);
				mbed_gpio_handler_update(s);
			}
			break;
	}
}

static void mbed_gpio_write_byte(void* opaque, target_phys_addr_t offset, uint32_t value) {
	struct gpio_state *s = (struct gpio_state*) opaque;
	int i, j;
	printf("Writing gpio byte\n");
	// Just storing value them even if it does not change it's fine
	for(i = 0; i < 5; ++i) {
		for(j = 0; j < 4 ; ++j) {
			s->fiopinpreval[i][j] = s->fiopin[i][j];
		}
	}
	switch(offset) {
		case 0x00:
				s->fiodir[0][0] = value;
			break;
		case 0x01:
			if(chkmask(s, 0, 1, 1, value) && chkdir(s, 0, 1, 1, value))
				s->fiodir[0][1] = value;
			break;
		case 0x02:
			s->fiodir[0][2] = value;
			break;
		case 0x03:
			s->fiodir[0][3] = value;
			break;
		case 0x10:
			s->fiomask[0][0] = value;
			break;
		case 0x11:
			s->fiomask[0][1] = value;
			break;
		case 0x12:
			s->fiomask[0][2] = value;
			break;
		case 0x13:
			s->fiomask[0][3] = value;
			break;
		case 0x14:
			{
				s->fiopin[0][0] = value;
				mbed_gpio_handler_update(s);
			}
			break;
			if(chkmask(s, 0, 1, 1, value) && chkdir(s, 0, 1, 1, value))
			{
				s->fiopin[0][1] = value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x16:
			{
				s->fiopin[0][2] = value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x17:
			{
				s->fiopin[0][3] = value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x18:
			if(chkmask(s, 0, 0, 1, value) && chkdir(s, 0, 0, 1, value))
			{
				s->fiopin[0][0] |= value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x19:
			if(chkmask(s, 0, 1, 1, value) && chkdir(s, 0, 1, 1, value))
			{
				s->fiopin[0][1] |= value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x1a:
			if(chkmask(s, 0, 2, 1, value) && chkdir(s, 0, 2, 1, value))
			{
				s->fiopin[0][2] |= value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x1b:
			if(chkmask(s, 0, 3, 1, value) && chkdir(s, 0, 3, 1, value))
			{
				s->fiopin[0][3] |= value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x1c:
			if(chkmask(s, 0, 0, 1, value) && chkdir(s, 0, 0, 1, value))
			{
				s->fiopin[0][0] &= ~value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x1d:
			if(chkmask(s, 0, 1, 1, value) && chkdir(s, 0, 1, 1, value))
			{
				s->fiopin[0][1] &= ~value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x1e:
			if(chkmask(s, 0, 2, 1, value) && chkdir(s, 0, 2, 1, value))
			{
				s->fiopin[0][2] &= ~value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x1f:
			if(chkmask(s, 0, 3, 1, value) && chkdir(s, 0, 3, 1, value))
			{
				s->fiopin[0][3] &= ~value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x20:
			s->fiodir[1][0] = value;
			break;
		case 0x21:
			s->fiodir[1][1] = value;
			break;
		case 0x22:
			s->fiodir[1][2] = value;
			break;
		case 0x23:
			s->fiodir[1][3] = value;
			break;
		case 0x30:
			s->fiomask[1][0] = value;
			break;
		case 0x31:
			s->fiomask[1][1] = value;
			break;
		case 0x32:
			s->fiomask[1][2] = value;
			break;
		case 0x33:
			s->fiomask[1][3] = value;
			break;
		case 0x34:
			if(chkmask(s, 1, 0, 1, value) && chkdir(s, 1, 0, 1, value))
			{
				s->fiopin[1][0] = value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x35:
			if(chkmask(s, 1, 1, 1, value) && chkdir(s, 1, 1, 1, value))
			{
				s->fiopin[1][1] = value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x36:
			if(chkmask(s, 1, 2, 1, value) && chkdir(s, 1, 2, 1, value))
			{
				s->fiopin[1][2] = value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x37:
			if(chkmask(s, 1, 3, 1, value) && chkdir(s, 1, 3, 1, value))
			{
				s->fiopin[1][3] = value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x38:
			if(chkmask(s, 1, 0, 1, value) && chkdir(s, 1, 0, 1, value))
			{
				s->fiopin[1][0] |= value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x39:
			if(chkmask(s, 1, 1, 1, value) && chkdir(s, 1, 1, 1, value))
			{
				s->fiopin[1][1] |= value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x3a:
			if(chkmask(s, 1, 2, 1, value) && chkdir(s, 1, 2, 1, value))
			{
				s->fiopin[1][2] |= value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x3b:
			if(chkmask(s, 1, 3, 1, value) && chkdir(s, 1, 3, 1, value))
			{
				s->fiopin[1][3] |= value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x3c:
			if(chkmask(s, 1, 0, 1, value) && chkdir(s, 1, 0, 1, value))
			{
				s->fiopin[1][0] &= ~value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x3d:
			if(chkmask(s, 1, 1, 1, value) && chkdir(s, 1, 1, 1, value))
			{
				s->fiopin[1][1] &= ~value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x3e:
			if(chkmask(s, 1, 2, 1, value) && chkdir(s, 1, 2, 1, value))
			{	
				s->fiopin[1][2] &= ~value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x3f:
			if(chkmask(s, 1, 3, 1, value) && chkdir(s, 1, 3, 1, value))
			{	
				s->fiopin[1][3] &= ~value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x40:
			s->fiodir[2][0] &= value;
			break;
		case 0x41:
			s->fiodir[2][1] &= value;
			break;
		case 0x42:
			s->fiodir[2][2] &= value;
			break;
		case 0x43:
			s->fiodir[2][3] = value;
			break;
		case 0x50:
			s->fiomask[2][0] = value;
			break;
		case 0x51:
			s->fiomask[2][1] = value;
			break;
		case 0x52:
			s->fiomask[2][2] = value;
			break;
		case 0x53:
			s->fiomask[2][3] = value;
			break;
		case 0x54:
			if(chkmask(s, 2, 0, 1, value) && chkdir(s, 2, 0, 1, value))
			{
				s->fiopin[2][0] = value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x55:
			if(chkmask(s, 2, 1, 1, value) && chkdir(s, 2, 1, 1, value))
			{
				s->fiopin[2][1] = value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x56:
			if(chkmask(s, 2, 2, 1, value) && chkdir(s, 2, 2, 1, value))
			{
				s->fiopin[2][2] = value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x57:
			if(chkmask(s, 2, 3, 1, value) && chkdir(s, 2, 3, 1, value))
			{	
				s->fiopin[2][3] = value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x58:
			if(chkmask(s, 2, 0, 1, value) && chkdir(s, 2, 0, 1, value))
			{	
				s->fiopin[2][0] |= value;
			
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x59:
			if(chkmask(s, 2, 1, 1, value) && chkdir(s, 2, 1, 1, value))
			{
				s->fiopin[2][1] |= value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x5a:
			if(chkmask(s, 2, 2, 1, value) && chkdir(s, 2, 2, 1, value))
			{
				s->fiopin[2][2] |= value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x5b:
			if(chkmask(s, 2, 3, 1, value) && chkdir(s, 2, 3, 1, value))
			{	
				s->fiopin[2][3] |= value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x5c:
			if(chkmask(s, 2, 0, 1, value) && chkdir(s, 2, 0, 1, value))
			{	
				s->fiopin[2][0] &= ~value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x5d:
			if(chkmask(s, 2, 1, 1, value) && chkdir(s, 2, 1, 1, value))
			{
				s->fiopin[2][1] &= ~value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x5e:
			if(chkmask(s, 2, 2, 1, value) && chkdir(s, 2, 2, 1, value))
			{	
				s->fiopin[2][2] &= ~value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x5f:
			if(chkmask(s, 2, 3, 1, value) && chkdir(s, 2, 3, 1, value))
			{
				s->fiopin[2][3] &= ~value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x60:
			s->fiodir[3][0] = value;
			break;
		case 0x61:
			s->fiodir[3][1] = value;
			break;
		case 0x62:
			s->fiodir[3][2] = value;
			break;
		case 0x63:
			s->fiodir[3][3] = value;
			break;
		case 0x70:
			s->fiomask[3][0] = value;
			break;
		case 0x71:
			s->fiomask[3][1] = value;
			break;
		case 0x72:
			s->fiomask[3][2] = value;
			break;
		case 0x73:
			s->fiomask[3][3] = value;
			break;
		case 0x74:
			if(chkmask(s, 3, 0, 1, value) && chkdir(s, 3, 0, 1, value))
			{
				s->fiopin[3][0] = value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x75:
			if(chkmask(s, 3, 1, 1, value) && chkdir(s, 3, 1, 1, value))
			{
				s->fiopin[3][1] = value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x76:
			if(chkmask(s, 3, 2, 1, value) && chkdir(s, 3, 2, 1, value))
			{
				s->fiopin[3][2] = value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x77:
			if(chkmask(s, 3, 3, 1, value) && chkdir(s, 3, 3, 1, value))
			{
				s->fiopin[3][3] = value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x78:
			if(chkmask(s, 3, 0, 1, value) && chkdir(s, 3, 0, 1, value))
			{
				s->fiopin[3][0] |= value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x79:
			if(chkmask(s, 3, 1, 1, value) && chkdir(s, 3, 1, 1, value))
			{
				s->fiopin[3][1] |= value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x7a:
			if(chkmask(s, 3, 2, 1, value) && chkdir(s, 3, 2, 1, value))
			{
				s->fiopin[3][2] |= value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x7b:
			if(chkmask(s, 3, 3, 1, value) && chkdir(s, 3, 3, 1, value))
			{
				s->fiopin[3][3] |= value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x7c:
			if(chkmask(s, 3, 0, 1, value) && chkdir(s, 3, 0, 1, value))
			{
				s->fiopin[3][0] &= ~value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x7d:
			if(chkmask(s, 3, 1, 1, value) && chkdir(s, 3, 1, 1, value))
			{
				s->fiopin[3][1] &= ~value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x7e:
			if(chkmask(s, 3, 2, 1, value) && chkdir(s, 3, 2, 1, value))
			{
				s->fiopin[3][2] &= ~value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x7f:
			if(chkmask(s, 3, 3, 1, value) && chkdir(s, 3, 3, 1, value))
			{
				s->fiopin[3][3] &= ~value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x80:
			s->fiodir[4][0] = value;
			break;
		case 0X81:
			s->fiodir[4][1] = value;
			break;
		case 0x82:
			s->fiodir[4][2] = value;
			break;
		case 0x83:
			s->fiodir[4][3] = value;
			break;
		case 0x90:
			s->fiomask[4][0] = value;
			break;
		case 0x91:
			s->fiomask[4][1] = value;
			break;
		case 0x92:
			s->fiomask[4][2] = value;
			break;
		case 0x93:
			s->fiomask[4][3] = value;
			break;
		case 0x94:
			if(chkmask(s, 4, 0, 1, value) && chkdir(s, 4, 0, 1, value))
			{
				s->fiopin[4][0] = value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x95:
			if(chkmask(s, 4, 1, 1, value) && chkdir(s, 4, 1, 1, value))
			{
				s->fiopin[4][1] = value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x96:
			if(chkmask(s, 4, 2, 1, value) && chkdir(s, 4, 2, 1, value))
			{
				s->fiopin[4][2] = value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x97:
			if(chkmask(s, 4, 3, 1, value) && chkdir(s, 4, 3, 1, value))
			{
				s->fiopin[4][3] = value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x98:
			if(chkmask(s, 4, 0, 1, value) && chkdir(s, 4, 0, 1, value))
			{
				s->fiopin[4][0] |= value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x99:
			if(chkmask(s, 4, 1, 1, value) && chkdir(s, 4, 1, 1, value))
			{
				s->fiopin[4][1] |= value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x9a:
			if(chkmask(s, 4, 2, 1, value) && chkdir(s, 4, 2, 1, value))
			{
				s->fiopin[4][2] |= value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x9b:
			if(chkmask(s, 4, 3, 1, value) && chkdir(s, 4, 3, 1, value))
			{
				s->fiopin[4][3] |= value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x9c:
			if(chkmask(s, 4, 0, 1, value) && chkdir(s, 4, 0, 1, value))
			{
				s->fiopin[4][0] &= ~value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x9d:
			if(chkmask(s, 4, 1, 1, value) && chkdir(s, 4, 1, 1, value))
			{
				s->fiopin[4][1] &= ~value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x9e:
			if(chkmask(s, 4, 2, 1, value) && chkdir(s, 4, 2, 1, value))
			{
				s->fiopin[4][2] &= ~value;
				mbed_gpio_handler_update(s);
			}
			break;
		case 0x9f:
			if(chkmask(s, 4, 3, 1, value) && chkdir(s, 4, 3, 1, value))
			{
				s->fiopin[4][3] &= ~value;
				mbed_gpio_handler_update(s);
			}
			break;
	}
}

static uint32_t mbed_gpio_intr_read(void * opaque, target_phys_addr_t offset)
{
	struct gpio_interrupts *s = (struct gpio_interrupts*)opaque;
	switch(offset) {
		case 0x90:
			return s->iointenr[0];
		case 0xb0:
			return s->iointenr[1];
		case 0x94:
			return s->iointenf[0];
		case 0xb4:
			return s->iointenf[1];
		case 0x84:
			return s->iointstatr[0];
		case 0xa4:
			return s->iointstatr[1];
		case 0x88:
			return s->iointstatf[0];
		case 0xa8:
			return s->iointstatf[1];
		case 0x8c:
			printf("Cannot read interrupt clear\n");
			break;
		case 0xac:
			printf("Cannot read interrupt clear\n");
			break;
		case 0x80:
			return s->iointstatus;
	}

	return 0;
}


static void mbed_gpio_intr_write(void * opaque, target_phys_addr_t offset, uint32_t value)
{
	struct gpio_interrupts *s = (struct gpio_interrupts*) opaque;
	switch(offset) {
		case 0x90:
			s->iointenr[0] = value;
			break;
		case 0xb0:
			s->iointenr[1] = value;
			break;
		case 0x94:
			s->iointenf[0] = value;
			break;
		case 0xb4:
			s->iointenf[1] = value;
			break;
		case 0x84:
			printf("Cannot write interrupt status rise 0 \n");
			break;
		case 0xa4:
			printf("Cannot write interrupt status rise 1\n");
			break;
		case 0x88:
			printf("Cannot write interrupt status fall 0 reg\n");
			break;
		case 0xa8:
			printf("Cannot write interrupt status fall 1 reg\n");
			break;
		case 0x8c:
			s->iointclr[0] = value;
			break;
		case 0xac:
			s->iointclr[1] = value;
			break;
		case 0x80:
			printf("Cannot write interrupt status register\n");

	}
}

static void mbed_gpio_reset(struct gpio_state *s) {
	int i,j;
	for(i = 0; i < 5; ++i) {
		for(j = 0 ; j < 4; j++)
		{
			s->fiodir[i][j] = 0;
			s->fiomask[i][j] = 0;
			s->fiopin[i][j] = 0;
			s->fiopinpreval[i][j] = 0;
		}
	}
}

static void mbed_gpio_set(void * opaque, int line, int level)
{
	struct gpio_state *s = (struct gpio_state *) opaque;
	struct gpio_interrupts *ptr = (struct gpio_interrupts *)ptr;
	uint32_t portno, mask;
	if(line < 32) {
		portno = 0;
	} else {
		portno = 2;
	}
	mask = (1 << line);
	if (line >= s->lines) {
		printf("%s: No GPIO pin %i\n", __FUNCTION__, line);
		return;
	}
	if(level) {
		if(portno == 0) {
			uint16_t fiodir = (s->fiodir[0][1] << 8) | s->fiodir[0][0];
			ptr->iointstatr[0] |= mask & ptr->iointenr[0] & ~fiodir; 
			if(ptr->iointstatr[0] & mask)
				qemu_irq_raise(s->irq);
		} else {
			uint16_t fiodir = (s->fiodir[2][1] << 8) | s->fiodir[2][1];
			ptr->iointstatr[1] |= ptr->iointenr[1] & mask & ~fiodir;
			if(ptr->iointstatr[1] & mask)
				qemu_irq_raise(s->irq);
		}
	} else {
		if(portno == 0) {
			uint16_t fiodir = (s->fiodir[0][1] << 8) | s->fiodir[0][0];
			ptr->iointstatf[0] |= mask & ptr->iointenf[0] & ~fiodir; 
				if(ptr->iointstatf[0] & mask)
					qemu_irq_raise(s->irq);
		} else {
			uint16_t fiodir = (s->fiodir[0][1] << 8) | s->fiodir[0][0];
			ptr->iointstatf[1] |= mask & ptr->iointenf[1] & ~fiodir; 
			if(ptr->iointstatf[1] & mask)
				qemu_irq_raise(s->irq);
		}
	}	
}

static void mbed_gpio_intr_reset(struct gpio_interrupts* s)
{
	s->iointenr[0] = s->iointenr[1] = s->iointenf[0] = s->iointenf[1] = 0;
	s->iointstatr[0] = s->iointstatr[1] = s->iointstatf[0] = s->iointstatf[1] = 0;
	s->iointclr[0] = s->iointclr[1] = s->iointstatus = 0;

}

static CPUReadMemoryFunc * const mbed_gpio_intr_readfn[] = {
	mbed_gpio_intr_read,
	mbed_gpio_intr_read,
	mbed_gpio_intr_read
};

static CPUWriteMemoryFunc * const mbed_gpio_intr_writefn[] = {
	mbed_gpio_intr_write,
	mbed_gpio_intr_write,
	mbed_gpio_intr_write
};

static CPUReadMemoryFunc * const mbed_gpio_readfn[] = {
	mbed_gpio_read_byte,
	mbed_gpio_read_halfw,
	mbed_gpio_read
};

static CPUWriteMemoryFunc * const mbed_gpio_writefn[] = {
	mbed_gpio_write_byte,
	mbed_gpio_write_halfw,
	mbed_gpio_write
};

static int mbed_gpio_init(SysBusDevice *dev)
{
	struct gpio_state *s = FROM_SYSBUS(struct gpio_state, dev);
	int iomemtype;
	qdev_init_gpio_in(&dev->qdev, mbed_gpio_set, s->lines);
	qdev_init_gpio_out(&dev->qdev, s->handler, s->lines);
	iomemtype = cpu_register_io_memory(mbed_gpio_readfn,
									   mbed_gpio_writefn, s,
									   DEVICE_NATIVE_ENDIAN);
	sysbus_init_mmio(dev, 0x4000, iomemtype);
	sysbus_init_irq(dev, &s->irq);
	mbed_gpio_reset(s);
	return 0;
}

/* Main Oscillator Of the MBED */
// Main oscillator is like the vm_clock in case of qemu because it is the one that
// provides all the timings to qemu
QEMUClock* main_oscillator;
bool main_oscillator_enabled = false;

static void enable_main_oscillator(void) 
{
	// Does nothing except gets a reference to the vm_clock maintained by
	// qemu
	if(!main_oscillator_enabled) 
	{
		main_oscillator = vm_clock;
		main_oscillator_enabled = true;
	}
}

static void disable_main_oscillator(void) 
{
	main_oscillator = NULL;
	main_oscillator_enabled = false;
}

static void change_oscillator_operating_range(int range) 
{
	printf("Changes in oscillator operating range not yet implemented\n");
}

/* PLL0 */


/* System Control */

typedef struct {
	/* Phase Locked Loop State */
} phase_locked_loop_state;

typedef struct {

} clock_power_state;

typedef struct {
	uint32_t flashcfg; /* Flash Accelerator State */
	/* Clock And Power Control State*/
	uint32_t pll0con; /* PLL Control Register */
	uint32_t pll0cfg; /* PLL Configuration Register */
	uint32_t pll0stat; /* PLL Status Register */
	uint32_t pll0feed; /* PLL Feed Register */
	uint32_t pll1con; /* PLL Control Register */
	uint32_t pll1cfg; /* PLL Configuration Register */
	uint32_t pll1stat; /* PLL Status Register */
	uint32_t pll1feed; /* PLL Feed Register */
	/* Power Control */
 	uint32_t pcon; /* Power Control Register */
	uint32_t pconp; /* Power Control for Peripherals Register */
	/* Clock Dividers */
	uint32_t cclkcfg; /* CPU Clock Configuration Register */
	uint32_t usbclkcfg; /* USB Clock Configurator Register */
	uint32_t clksrcsel; /* Clock Source Select Register */
	uint32_t extint; /* External Interrupt Flag Register */
	uint32_t extmode; /* External Interrupt Mode Register */
	uint32_t extpolar; /* External Interrupt Polarity Register */
	uint32_t rsid; /* Reset Source Indentification Register */
	uint32_t scs; /* System Control and Status */
	uint32_t pclksel0; /* Peripheral Clock Selection Register 0 */
	uint32_t pclksel1; /* Peripheral Clock Selection Register 1*/
	/* Utility */
	uint32_t clkoutcfg; /* Clock Output Configuration Register */
	qemu_irq irq;
} ssys_state;

static void ssys_update(ssys_state *s)
{
	qemu_set_irq(s->irq, (s->extint != 0));
}

static uint32_t ssys_read(void *opaque, target_phys_addr_t offset)
{
	ssys_state *s = (ssys_state *) opaque;
	switch (offset) {
		case 0x000: /* flashcfg */
			return s->flashcfg;
		case 0x080: /* PLL0 Control Register */
			return s->pll0con;
		case 0x084: /* PLL0 Config Register */
			return s->pll0cfg;
		case 0x088: /* PLL0 Status Register */
			return s->pll0stat;
		case 0x08C: /* PLL0 Feed Register */
			return s->pll0feed;
		case 0x0A0: /* PLL1 Control Register */
			return s->pll1con;
		case 0x0A4: /* PLL1 Config Register */
			return s->pll1cfg;
		case 0x0A8: /* PLL1 Status Register */
			return s->pll1stat;
		case 0x0AC: /* PLL1 Feed Register */
			return s->pll1feed;
		case 0x0C0: /* Power Control Register */
			return s->pcon;
		case 0x0C4: /* Power Control For Peripherals Register */
			return s->pconp;
		case 0x104: /* CPU Clock Configuration Register */
			return s->cclkcfg;
		case 0x108: /* USB Clock Configurator Register */
			return s->usbclkcfg;
		case 0x10C: /* Clock Source Select Register */
			return s->clksrcsel;
		case 0x140: /* External Interrupt Flag Register */
			return s->extint;
		case 0x148: /* External Interrupt Mode Register */
			return s->extmode;
		case 0x14C: /* External Interrupt Polarity Register */
			return s->extpolar;
		case 0x180: /* Reset Source Indentification Register */
			return s->rsid;
		case 0x1A0: /* System Control and Status */
			return s->scs;
		case 0x1A8: /* Peripheral Clock Selection Register 0 */
			return s->pclksel0;
		case 0x1AC: /* Peripheral Clock Selection Register 1 */
			return s->pclksel1;
		case 0x1C8: /* Clock Output Configuration Register */
			return s->clkoutcfg;
		default:
			hw_error("ssys_read : Bad Offset 0x%x\n", (int)offset);
			return 0;
	}
}

static void ssys_write(void *opaque, target_phys_addr_t offset, uint32_t value)
{
	ssys_state *s = (ssys_state *) opaque;
	printf("Trying to write to memory\n");
	switch (offset) {
		case 0x000: /* flashcfg */
			s->flashcfg = value;
			break;
		case 0x080: /* PLL0 Control Register */
			if(value & 0x1)
			{
				// Enabling and activating PLL0
				s->pll0con |= 0x1;
				// Changing the status of the pll0 in pll0stat
				s->pll0stat |= (uint32_t)(1 << 24);
				// Assume that you get a instant lock in absence of actual
				// hardware
				s->pll0stat |= (uint32_t)(1 << 26);
			}
			else if((s->pll0con & 0x1) /*TODO(gdrane) Enter the pll0 locked condition */ && (value & 0x2))
			{
				// Updating the PLL0 Connect bit  
				s->pll0con |= 0x2;
				// Updating status in pll0stat
				s->pll0stat |= (uint32_t)(1 << 25);
				//TODO(gdrane) Code to start PLL0
			}
			break;
		case 0x084: /* PLL0 Config Register */
			{
				if(s->pll0con & 0x1)
				{
					printf("PLL0 not enabled please enable PLL0 first");
					return;
				}
				// Multiplier select 
				uint32_t msel0 = value &	((uint32_t)(1 << 15) - 1);
				// Divider select
				uint32_t nsel0 = value >> 16;
				if (msel0) 
				{
					s->pll0cfg &= ~ ((uint32_t)(1 << 15) - 1);
					s->pll0stat &= ~ ((uint32_t)(1 << 15) - 1);
					s->pll0cfg |= msel0 - 1;
					s->pll0stat |= msel0 - 1;
				}
				if(nsel0) 
				{
					int prevpll0stat = s->pll0stat;
					prevpll0stat &= ~ ((uint32_t)(1 << 24) - 1);
					s->pll0cfg &= ((uint32_t)(1 << 15) - 1);
					s->pll0stat &= ((uint32_t)(1 << 15) - 1); 
					nsel0 --;
					nsel0 <<= 16;
					s->pll0cfg |= nsel0;
					s->pll0stat |=nsel0;
					s->pll0stat |= prevpll0stat;
				}
				if (msel0 && value >> 16 /* I destroy nsel thus this*/)
				{
					// Wait for plock0 to become active
					// In case of software emulation we need not wait for
					// pll0 to get the lock
				}
			}
			break;
		case 0x088: /* PLL0 Status Register */
			printf("\nYou cannot write the PLL0 status register\n");
			break;
		case 0x08C: /* PLL0 Feed Register */
			printf("Updating feed register\n");
			// This is of no importance to a software emulator as we can go
			// ahead without the feed sequence
			if(s->pll0feed == 0 && value == 0xAA)
			{
				s->pll0feed = 0xAA;
			}
			else if(s->pll0feed == 0xAA && value == 0x55)
				 {
					s->pll0feed = value;
					// Feed Sequence completed
				 }
			break;
		case 0x0A0: /* PLL1 Control Register */
			s->pll1con = value;
			break;
		case 0x0A4: /* PLL1 Config Register */
			s->pll1cfg = value;
			break;
		case 0x0A8: /* PLL1 Status Register */
			s->pll1stat = value;
			break;
		case 0x0AC: /* PLL1 Feed Register */
			s->pll1feed = value;
			break;
		case 0x0C0: /* Power Control Register */
			s->pcon = value;
			break;
		case 0x0C4: /* Power Control For Peripherals Register */
			s->pconp = value;
			break;
		case 0x104: /* CPU Clock Configuration Register */
			// Used to divide PLL0 output before plugging it to CPU
			// For cclkcfg = 0, 1 this change cannot happen when PLL0 is 
			// connected
			if (s->cclkcfg  == 0 && s->cclkcfg == 1) 
			{
				if(!(s->pll0stat & (uint32_t)(1 << 25)))
				{
					printf("PLL0 enabled cannot change configuration\n");
					break;
				}
			}
			s->cclkcfg = 0;
			s->cclkcfg |= value & (((uint32_t) 1 << 8) - 1);
			break;
		case 0x108: /* USB Clock Configurator Register */
			s->usbclkcfg = value;
			break;
		case 0x10C: /* Clock Source Select Register */
			{
				// int pll0clock = 0;
				switch(value & 0x3) {
					case 0: 
						printf("\nInternal RC Oscillator cannot be selected as Pll0 clock: NOT IMPLEMENTED\n");
					 	break;
				case 1:
						printf("\nMain Oscillator Selected as clock\n");
						break;
				case 2:
						printf("\nRTC Oscillator cannot be used as clock source:NOT IMPLEMENTED\n");
						break;
				}
				s->clksrcsel = (uint32_t)0x01;
			}
			break;
		case 0x140: /* External Interrupt Flag Register */
			s->extint = value;
			break;
		case 0x148: /* External Interrupt Mode Register */
			s->extmode = value;
			break;
		case 0x14C: /* External Interrupt Polarity Register */
			s->extpolar = value;
			break;
		case 0x180: /* Reset Source Indentification Register */
			s->rsid = value;
			break;
		case 0x1A0: /* System Control and Status */
			{
				// int oscrange = ((s->scs >> 4) & 0x1);
				uint32_t oscen = (value >> 5) & 0x1;
				uint32_t oscrange = (value >> 6) & 0x1;
				if(oscrange != ((s->scs >> 6) & 0x1))
				{
					if(oscrange) 
					{
						s->scs |= ((uint32_t)1 << 6);
					}
					else
					{
						s->scs &= ~((uint32_t)1 << 6);
					}
				}
				if(oscen)
				{
					s->scs |= (1 << 5);
				}
				if ((s->scs >> 5) & 0x1) {
					enable_main_oscillator();
					
					// Setting the main oscillator started status
					s->scs |= 0x40;
				} else {
					disable_main_oscillator();
				}
				/*if(oscrange  != ((s->scs >> 4) & 0x1))
					change_oscillator_operating_range(((s->scs >> 4) & 0x1));
				*/
			}
			break;
		case 0x1A8: /* Peripheral Clock Selection Register 0 */
			// Controls the input clock to peripherals. Clock rate given to
			// peripheral is a derivative of cpuclock CCLK as per arm user 
			// manual
			// Each peripheral has 1bits reserved
			// Btw, only peripheral that is not controlled by this configuration	
			// is RTC, it's input is fixed at 1khz and or CCLK/8 (ambiguous???
			// )
			// 2 bit representation
			// 00 = CCLK/4
			// 01 = CCLK
			// 10 = CCLK/2
			// 11 = CCLK / 8 except for CAN1, CAN2 and CAN filtering when 11 selects
			// =CCLK/6 (I don't know what they are saying)
			{
				// int prevpclksel = s->pclksel0;
				printf("PCLK register 0 written\n");
				// Not interested in simulating any of these peripherals
				// you guys are free to do whatever you want to
				s->pclksel0 = value;
			}
			break;
		case 0x1AC: /* Peripheral Clock Selection Register 1 */
			// Check out the comments for pclk register 0
			{
				// Checking whether there are timer updates to GPIO interrupts
				//
				if(value & 0xC)
				{
					//int prevpclksel1 = s->pclksel1;
					// Change clock configuration of GPIO interrupts
					// TODO(gdrane)
				}
				s->pclksel1 = value;

			}
			break;
		case 0x1C8: /* Clock Output Configuration Register */
			s->clkoutcfg = value;
			break;
		default:
			hw_error("ssys_write : Bad Offset 0x%x\n", (int)offset);
	}
	// ssys_update(s);
}

static CPUWriteMemoryFunc * const ssys_writefn[] = {
	ssys_write,
	ssys_write,
	ssys_write,
};

static CPUReadMemoryFunc * const ssys_readfn[] = {
	ssys_read,
	ssys_read,
	ssys_read,
};

static void ssys_reset(void *opaque)
{
	ssys_state *s = (ssys_state *)opaque;
	s->extint = 0;
	s->extmode = 0;
	s->extpolar = 0;
	// TODO(gdrane):  Figure out where to implement s->rsid complex logic
	s->scs = 0;
	s->clksrcsel = 0;
	s->pll0con = 0;
	s->pll0cfg = 0;
	s->pll0stat = 0;
	s->pll0feed = 0;
	s->pll1con = 0;
	s->pll1cfg = 0;
	s->pll1stat = 0;
	s->pll1feed = 0;
	s->cclkcfg = 0;
	s->usbclkcfg = 0;
	s->pclksel0 = 0;
	s->pclksel1 = 0;
	s->pcon = 0;
	s->flashcfg = 0x303A;
	s->pconp = 0x03BE;
	disable_main_oscillator();
}

static void ssys_calculate_system_clock(ssys_state *s)
{
	printf("In calculate system clock\n");
	// Derived From SystemInit in system_LPC17xx.c
	// TODO(gdrane): Calculate actual value everytime using PLL mathematics
	system_clock_scale = 18;
}

static int mbed_sys_post_load(void *opaque, int version_id)
{
	printf("In mbed post load\n");
	ssys_state *s = opaque;

	ssys_calculate_system_clock(s);

	return 0;
}

static const VMStateDescription vmstate_mbed_sys = {
	.name = "mbed_sys",
	.version_id = 1,
	.minimum_version_id = 1,
	.post_load = mbed_sys_post_load,
	.fields = (VMStateField[]) {
		VMSTATE_UINT32(flashcfg, ssys_state), 
 		VMSTATE_UINT32(pll0con, ssys_state),
 		VMSTATE_UINT32(pll0cfg, ssys_state),
 		VMSTATE_UINT32(pll0stat, ssys_state),
 		VMSTATE_UINT32(pll0feed, ssys_state),
 		VMSTATE_UINT32(pll1con, ssys_state),
 		VMSTATE_UINT32(pll1cfg, ssys_state),
		VMSTATE_UINT32(pll1stat, ssys_state),
		VMSTATE_UINT32(pll1feed, ssys_state),
		VMSTATE_UINT32(cclkcfg, ssys_state),
		VMSTATE_UINT32(usbclkcfg, ssys_state),
		VMSTATE_UINT32(clksrcsel, ssys_state),
		VMSTATE_UINT32(extint, ssys_state),
		VMSTATE_UINT32(extmode, ssys_state),
		VMSTATE_UINT32(extpolar, ssys_state),
		VMSTATE_UINT32(rsid, ssys_state),
		VMSTATE_UINT32(scs, ssys_state),
		VMSTATE_UINT32(pclksel0, ssys_state),
		VMSTATE_UINT32(pclksel1, ssys_state),
		VMSTATE_UINT32(clkoutcfg, ssys_state),
		VMSTATE_END_OF_LIST()
	}
};

static int mbed_sys_init(uint32_t base, qemu_irq irq)
{
	int iomemtype;
	ssys_state *s;
	s = (ssys_state *)qemu_mallocz(sizeof(ssys_state));
	s->irq = irq;
	iomemtype = cpu_register_io_memory(ssys_readfn, 
									   ssys_writefn, (void*)s,
									   DEVICE_NATIVE_ENDIAN);
	system_clock_scale = 18;
	cpu_register_physical_memory(base, PERIPHERAL_AREA_SIZE, iomemtype);
	ssys_reset(s);
	vmstate_register(NULL, -1, &vmstate_mbed_sys, s);
	return 0;
}

// Machine initialization routine
static void mbed_init(ram_addr_t ram_size,
				const char *boot_device,
				const char *kernel_filename, const char *kernel_cmdline,
				const char *initrd_filename, const char *cpu_model)
{
	// CPUState *env;
	// ram_addr_t ram_offset;
	qemu_irq *cpu_pic;
	static const int timer_irq[] = { 1, 2, 3, 4};
	// TODO(gdrane): Figure out gpio and pic on mbed
	// qemu_irq pic[32];
	// qemu_irq sic[32];
	// qemu_irq adc;
	DeviceState *dev;
	// flash size = 512kb = 0x200	
	int flash_size = 0x200;
	// sram_size = 32kb  = 0x20
	int sram_size = 0x20;
	cpu_pic = armv7m_init(flash_size, sram_size, kernel_filename, cpu_model);
	// Adding a ADC
	// dev = sysbus_create_varargs("mbed-adc", 0x40034000, 
	//							/* TODO(gdrane): find pic that should be connected*/);
	// qdev_connect_gpio_in(dev, 0);

	// Adding DAC
	// dev = sysbus_create_varargs("mbed-dac", 0x4008C000,
	//							/* TODO(gdrane): Find the dac's to connect to*/);
	// qdev_connect_gpio_in(dev, 0);
	
	// MBED Timer 0
		dev = sysbus_create_simple("mbed-timer0", 0x40004000, cpu_pic[timer_irq[0]]);
	// MBED Timer 1
		sysbus_create_simple("mbed-timer1", 0x40008000, cpu_pic[timer_irq[1]]);
	// MBED Timer 2
		sysbus_create_simple("mbed-timer2", 0x40090000, cpu_pic[timer_irq[2]]);
	// MBED Timer 3
		sysbus_create_simple("mbed-timer3", 0x40094000, cpu_pic[timer_irq[4]]);
	
	#define SYS_CNTRL_INTRPT_NO 28	
	mbed_sys_init(0x400fc000, cpu_pic[SYS_CNTRL_INTRPT_NO]);
	// Initializing GPIO's
	// Creating space for interrupts
	struct gpio_interrupts *s = (struct gpio_interrupts *)qemu_mallocz(sizeof(struct gpio_interrupts));
	int iomemtype = cpu_register_io_memory(mbed_gpio_intr_readfn,
										   mbed_gpio_intr_writefn, s,
										   DEVICE_NATIVE_ENDIAN);
	cpu_register_physical_memory(0x40028000, 0x4000, iomemtype);
	mbed_gpio_intr_reset(s);
	dev = qdev_create(NULL, "mbed-gpio");
	qdev_prop_set_int32(dev, "lines", 128);
	qdev_prop_set_ptr(dev, "intr_ref",(void*) s);
	qdev_init_nofail(dev);
	sysbus_mmio_map(sysbus_from_qdev(dev), 0, 0x2009c000);
	sysbus_connect_irq(sysbus_from_qdev(dev), 0,
						cpu_pic[21]);
	// UART - Serial Communication using MBED
	// sysbus_create_simple("mbed-uart0", 0x4000c000, cpu_pic[5]);
	// sysbus_create_simple("mbed-uart1", 0x40010000, cpu_pic[6]);
	// sysbus_create_simple("mbed-uart2", 0x40098000, cpu_pic[7]);
	// sysbus_create_simple("mbed-uart3", 0x4009c000, cpu_pic[8]);
	if(serial_hds[0]) {
		mbed_uart_init(0x4000c000, cpu_pic[5], /*NULL*/serial_hds[0]);
		mbed_uart_init(0x40010000, cpu_pic[6], /*NULL*/serial_hds[0]);
		mbed_uart_init(0x40098000, cpu_pic[7], /*NULL*/serial_hds[0]);
		mbed_uart_init(0x4009c000, cpu_pic[8], /*NULL*/serial_hds[0]);
	}
}

static SysBusDeviceInfo mbed_gpio_info = {
	.init 		= mbed_gpio_init,
	.qdev.name 	= "mbed-gpio",
	.qdev.desc 	= "MBED GPIO Controller",
	.qdev.size 	= sizeof(struct gpio_state),
	.qdev.props = (Property []) {
		DEFINE_PROP_INT32("lines", struct gpio_state, lines, 0),
		DEFINE_PROP_PTR("intr_ref", struct gpio_state, intr_ref),
		DEFINE_PROP_END_OF_LIST(),
	}
};

static void mbed_register_devices(void) {
	sysbus_register_dev("mbed-timer0", sizeof(timer_state),
						 mbed_timer_init);
	sysbus_register_dev("mbed-timer1", sizeof(timer_state),
						mbed_timer_init);
	sysbus_register_dev("mbed-timer2", sizeof(timer_state),
						mbed_timer_init);
	sysbus_register_dev("mbed-timer3", sizeof(timer_state),
						mbed_timer_init);
	sysbus_register_withprop(&mbed_gpio_info);
/*	sysbus_register_dev("mbed-uart0", sizeof(mbed_uart_state),
						mbed_uart_init);
	sysbus_register_dev("mbed-uart1", sizeof(mbed_uart_state),
						mbed_uart_init);
	sysbus_register_dev("mbed-uart2", sizeof(mbed_uart_state),
						mbed_uart_init);
	sysbus_register_dev("mbed-uart3", sizeof(mbed_uart_state),
						mbed_uart_init);
*/
}

device_init(mbed_register_devices);

static QEMUMachine mbed_machine = {
	.name = "mbed",
	.desc = "MBED LPC 1768",
	.init = mbed_init,
};

static void mbed_machine_init(void)
{
	qemu_register_machine(&mbed_machine);
}

machine_init(mbed_machine_init);

