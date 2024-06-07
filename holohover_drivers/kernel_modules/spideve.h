#ifndef SPIDEVE_H
#define SPIDEVE_H

#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

// This extended SPI_IOC_MESSAGE increases the maximum number of
// messages that can be sent from 511 to 16383 (by sizeof(struct spi_ioc_transfer) == 32)
#define SPI_MSGSIZE_E(N) (((N) < (1 << _IOC_SIZEBITS)) ? (N) : 0)
#define SPI_IOC_MESSAGE_E(N) _IOW(SPI_IOC_MAGIC, 6, char[SPI_MSGSIZE_E(N)])

#endif //SPIDEVE_H
