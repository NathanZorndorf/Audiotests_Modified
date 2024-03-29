#ifndef REGISTER_I2S_H_
#define REGISTER_I2S_H_

/* For more information see SPRUFX4A - Section 1.3 - Table 1-13 */
/* I2S0 Register Mapping */
#define I2S0_SCTRL   *(ioport volatile unsigned *)0x2800 //I2S0 Serializer Control Register
#define I2S0_SRATE   *(ioport volatile unsigned *)0x2804 //I2S Sample Rate Generator Register
#define I2S0_INTFL   *(ioport volatile unsigned *)0x2810 //I2S0 Interrupt Flag Register
#define I2S0_INTMASK *(ioport volatile unsigned *)0x2814 //I2S0 Interrupt Mask Register

#define I2S0_TXLT0 *(ioport volatile unsigned *)0x2808 //I2S0 Transmit Left Data 0 Register
#define I2S0_TXLT1 *(ioport volatile unsigned *)0x2809 //I2S0 Transmit Left Data 1 Register
#define I2S0_TXRT0 *(ioport volatile unsigned *)0x280C //I2S0 Transmit Right Data 0 Register
#define I2S0_TXRT1 *(ioport volatile unsigned *)0x280D //I2S0 Transmit Right Data 1 Register

#define I2S0_RXLT0 *(ioport volatile unsigned *)0x2828 //I2S0 Receive Left Data 0 Register
#define I2S0_RXLT1 *(ioport volatile unsigned *)0x2829 //I2S0 Receive Left Data 1 Register
#define I2S0_RXRT0 *(ioport volatile unsigned *)0x282C //I2S0 Receive Right Data 0 Register
#define I2S0_RXRT1 *(ioport volatile unsigned *)0x282D //I2S0 Receive Right Data 1 Register

/* I2S2 Register Mapping */
#define I2S2_SCTRL   *(ioport volatile unsigned *)0x2A00 //I2S2 Serializer Control Register
#define I2S2_SRATE   *(ioport volatile unsigned *)0x2A04 //I2S2 Sample Rate Generator Register
#define I2S2_INTFL   *(ioport volatile unsigned *)0x2A10 //I2S2 Interrupt Flag Register
#define I2S2_INTMASK *(ioport volatile unsigned *)0x2A14 //I2S2 Interrupt Mask Register

#define I2S2_TXLT0 *(ioport volatile unsigned *)0x2A08 //I2S2 Transmit Left Data 0 Register
#define I2S2_TXLT1 *(ioport volatile unsigned *)0x2A09 //I2S2 Transmit Left Data 1 Register
#define I2S2_TXRT0 *(ioport volatile unsigned *)0x2A0C //I2S2 Transmit Right Data 0 Register
#define I2S2_TXRT1 *(ioport volatile unsigned *)0x2A0D //I2S2 Transmit Right Data 1 Register

#define I2S2_RXLT0 *(ioport volatile unsigned *)0x2A28 //I2S2 Receive Left Data 0 Register
#define I2S2_RXLT1 *(ioport volatile unsigned *)0x2A29 //I2S2 Receive Left Data 1 Register
#define I2S2_RXRT0 *(ioport volatile unsigned *)0x2A2C //I2S2 Receive Right Data 0 Register
#define I2S2_RXRT1 *(ioport volatile unsigned *)0x2A2D //I2S2 Receive Right Data 1 Register

#endif /*REGISTER_I2S_H_*/
