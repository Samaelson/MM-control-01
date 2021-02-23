//spi.h - hardware SPI
#ifndef SPI_H
#define SPI_H

#include <inttypes.h>
#include <avr/io.h>
#include "config.h"

#define SPI_SPCR(rat, pha, pol, mst, dor) ((rat & 3) | (pha?(1<<CPHA):0) | (pol?(1<<CPOL):0) | (mst?(1<<MSTR):0) | (dor?(1<<DORD):0) | (1<<SPE))
#define SPI_SPSR(rat) ((rat & 4)?(1<<SPI2X):0)

#define DD_SCK  1
#define DD_MOSI 2
#define DD_MISO 3

#if defined(__cplusplus)
extern "C" {
#endif //defined(__cplusplus)
//inline void spi_init()
extern void spi_init(void);

extern void spi_setup(uint8_t spcr, uint8_t spsr);

extern uint8_t spi_txrx(uint8_t tx);

#if defined(__cplusplus)
}
#endif //defined(__cplusplus)
#endif //SPI_H
