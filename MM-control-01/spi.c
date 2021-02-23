


#include <avr/io.h>
#include "config.h"
#include "spi.h"



//inline void spi_init()
void spi_init(void)
{
	DDRB &= ~((1 << DD_SCK) | (1 << DD_MOSI) | (1 << DD_MISO));
	DDRB |= (1 << DD_SCK) | (1 << DD_MOSI);
}

//inline void spi_setup(uint8_t spcr, uint8_t spsr)
void spi_setup(uint8_t spcr, uint8_t spsr)
{
	SPCR = spcr;
	SPSR = spsr;
}

//inline uint8_t spi_txrx(uint8_t tx)
uint8_t spi_txrx(uint8_t tx)
{
	SPDR = tx;
	while (!(SPSR & (1 << SPIF)));
	return SPDR;
}
