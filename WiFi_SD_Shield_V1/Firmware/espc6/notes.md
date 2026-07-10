## Improving commuication speed

Bottleneck :when ESP acts as a slave, the transmission interval (time between two consecutive SPI NRF -> ESP trasnaction) is very low.

Original impl<--<ementation:
a) stick to the current BIOGAP FW implementation --> one EXG packet is made of 211 bytes, with 4 samples per packet aggregated
b) this means that one sample must be sent (TI) every 4 * sampling rate. E.G. sampling at 500 Hz, means sending every 8 ms. 


Basic implementation -->
Max sustainable T.I measured is 200 ms, clealry not compatible with any of our application.

T.I. 200 ms. Bytes x packet = 211



Pre-queue transactions

Solution -> pre-queuing multiple SPI transactions 

|pre-queue|--> allocate some space where results of SPI transaction can fit in.
So the queue gives you:

- a place for the SPI driver to store incoming data immediately
- time to process the previous packet without making the slave “unarmed”
- protection against short delays in your task, logging, or other work


spi_slave_queue_trans() hands a transaction descriptor to the SPI driver and says, “this buffer is ready to be used when the master clocks data.”

spi_slave_get_trans_result() waits until one queued transaction finishes and gives that descriptor back to you.


Improvements: 

Managed to send 211 bytes at 4 ms transaction interval --> corresponds to ExG sampling rate of 1 KHz

