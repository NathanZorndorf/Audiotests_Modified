#include "data_types.h"
#include "audio_setup.h"
#include "dma_setup.h"
#include "i2s_setup.h"
#include "register_system.h"
#include "register_cpu.h"
#include "aic3204.h"
#include "sar.h"
#include "hwafft.h"

#define PLL_100M 1 /* Set PLL system clock to 100MHz */
#define PLL_98M  0
#define PLL_40M  0
#define PLL_12M  0

/* Buffers for DMA in/out and overlap-add */
Int16 PingPongFlagInL;
Int16 PingPongFlagInR;
Int16 PingPongFlagOutL;
Int16 PingPongFlagOutR;

Int16 DMA_InpL[PING_PONG_SIZE];
Int16 DMA_InpR[PING_PONG_SIZE];
Int16 DMA_OutL[PING_PONG_SIZE];
Int16 DMA_OutR[PING_PONG_SIZE];

/* The input overlap buffers are considered to have the following
 * structure (assuming 50% overlap):
 *
 * +----------+----------+
 * |   OLD    |    NEW   |
 * +----------+----------+
 *  (hop_size) (hop_size)
 *
 * With each new data, the previous NEW block is copied to the OLD block
 * and the current audio data are transfered to the NEW block. */
Int16 OverlapInL[WND_LEN];
Int16 OverlapInR[WND_LEN];

/* The output overlap buffers contain the last HOP_SIZE samples of the previous
 * output buffer. These samples are added to the first HOP_SIZE samples of the current
 * output buffer and result in the current output audio data. The second half of the
 * current output buffer is then saved to these buffers. */
Int16 OverlapOutL[OVERLAP_LENGTH];
Int16 OverlapOutR[OVERLAP_LENGTH];

/* Buffers for processing */
#pragma DATA_SECTION(BufferL,"BufL");
Int16 BufferL[FFT_LENGTH];
#pragma DATA_SECTION(BufferR,"BufR");
Int16 BufferR[FFT_LENGTH];
#pragma DATA_SECTION(realL, "rfftL");
Int16 realL[FFT_LENGTH];
#pragma DATA_SECTION(realR, "rfftR");
Int16 realR[FFT_LENGTH];
#pragma DATA_SECTION(imagL, "ifftL");
Int16 imagL[FFT_LENGTH];
#pragma DATA_SECTION(imagR, "ifftR");
Int16 imagR[FFT_LENGTH];

/* --- Special buffers required for HWAFFT ---*/
#pragma DATA_SECTION(complex_buffer, "cmplxBuf");
Int32 complex_buffer[WND_LEN];

#pragma DATA_SECTION(bitreversed_buffer, "brBuf");
#pragma DATA_ALIGN(bitreversed_buffer, 2*FFT_LENGTH);
Int32 bitreversed_buffer[FFT_LENGTH];

#pragma DATA_SECTION(temporary_buffer,"tmpBuf");
Int32 temporary_buffer[FFT_LENGTH];
/* -------------------------------------------*/

/* 256 HaMMing window in Q0.15 */
/* From MATLAB:
   >> wnd = hamming(256,'periodic');
   >> fixwnd = fi(wnd,1,16,15); */
#pragma DATA_SECTION(window,"wnd1");
Int16 window[WND_LEN]={
0x0a3d, 0x0a42, 0x0a50, 0x0a66, 0x0a86, 0x0aaf, 0x0ae1, 0x0b1b, 0x0b5f, 0x0bac, 0x0c01, 0x0c5f, 0x0cc6, 0x0d36, 0x0daf, 0x0e2f,
0x0eb9, 0x0f4b, 0x0fe5, 0x1087, 0x1131, 0x11e4, 0x129e, 0x1360, 0x142a, 0x14fb, 0x15d4, 0x16b4, 0x179b, 0x1889, 0x197e, 0x1a7a,
0x1b7c, 0x1c85, 0x1d94, 0x1ea9, 0x1fc4, 0x20e5, 0x220c, 0x2337, 0x2468, 0x259f, 0x26da, 0x2819, 0x295d, 0x2aa6, 0x2bf2, 0x2d42,
0x2e96, 0x2fee, 0x3149, 0x32a7, 0x3407, 0x356a, 0x36d0, 0x3838, 0x39a2, 0x3b0e, 0x3c7b, 0x3dea, 0x3f59, 0x40ca, 0x423b, 0x43ad,
0x451f, 0x4691, 0x4802, 0x4974, 0x4ae4, 0x4c54, 0x4dc2, 0x4f30, 0x509b, 0x5205, 0x536d, 0x54d3, 0x5636, 0x5797, 0x58f5, 0x5a50,
0x5ba7, 0x5cfb, 0x5e4b, 0x5f98, 0x60e0, 0x6224, 0x6364, 0x649f, 0x65d5, 0x6706, 0x6832, 0x6958, 0x6a79, 0x6b94, 0x6ca9, 0x6db8,
0x6ec1, 0x6fc3, 0x70bf, 0x71b4, 0x72a3, 0x738a, 0x746a, 0x7542, 0x7614, 0x76dd, 0x77a0, 0x785a, 0x790c, 0x79b7, 0x7a59, 0x7af3,
0x7b85, 0x7c0e, 0x7c8f, 0x7d07, 0x7d77, 0x7dde, 0x7e3c, 0x7e92, 0x7ede, 0x7f22, 0x7f5d, 0x7f8f, 0x7fb7, 0x7fd7, 0x7fee, 0x7ffb,
0x7fff, 0x7ffb, 0x7fee, 0x7fd7, 0x7fb7, 0x7f8f, 0x7f5d, 0x7f22, 0x7ede, 0x7e92, 0x7e3c, 0x7dde, 0x7d77, 0x7d07, 0x7c8f, 0x7c0e,
0x7b85, 0x7af3, 0x7a59, 0x79b7, 0x790c, 0x785a, 0x77a0, 0x76dd, 0x7614, 0x7542, 0x746a, 0x738a, 0x72a3, 0x71b4, 0x70bf, 0x6fc3,
0x6ec1, 0x6db8, 0x6ca9, 0x6b94, 0x6a79, 0x6958, 0x6832, 0x6706, 0x65d5, 0x649f, 0x6364, 0x6224, 0x60e0, 0x5f98, 0x5e4b, 0x5cfb,
0x5ba7, 0x5a50, 0x58f5, 0x5797, 0x5636, 0x54d3, 0x536d, 0x5205, 0x509b, 0x4f30, 0x4dc2, 0x4c54, 0x4ae4, 0x4974, 0x4802, 0x4691,
0x451f, 0x43ad, 0x423b, 0x40ca, 0x3f59, 0x3dea, 0x3c7b, 0x3b0e, 0x39a2, 0x3838, 0x36d0, 0x356a, 0x3407, 0x32a7, 0x3149, 0x2fee,
0x2e96, 0x2d42, 0x2bf2, 0x2aa6, 0x295d, 0x2819, 0x26da, 0x259f, 0x2468, 0x2337, 0x220c, 0x20e5, 0x1fc4, 0x1ea9, 0x1d94, 0x1c85,
0x1b7c, 0x1a7a, 0x197e, 0x1889, 0x179b, 0x16b4, 0x15d4, 0x14fb, 0x142a, 0x1360, 0x129e, 0x11e4, 0x1131, 0x1087, 0x0fe5, 0x0f4b,
0x0eb9, 0x0e2f, 0x0daf, 0x0d36, 0x0cc6, 0x0c5f, 0x0c01, 0x0bac, 0x0b5f, 0x0b1b, 0x0ae1, 0x0aaf, 0x0a86, 0x0a66, 0x0a50, 0x0a42};

/* 256 Hann/HaMMing window in Q0.15 */
/* From MATLAB:
   >> wnd = hann(256,'periodic')./hamming(256,'periodic');
   >> fixwnd=fi(wnd,1,16,15); */
#pragma DATA_SECTION(recwin,"wnd2");
Int16 recwin[WND_LEN]={
0x0000, 0x003e, 0x00f5, 0x0222, 0x03c0, 0x05c4, 0x0827, 0x0adc, 0x0dd8, 0x110e, 0x1473, 0x17fc, 0x1b9d, 0x1f4c, 0x2300, 0x26b2,
0x2a5b, 0x2df6, 0x317d, 0x34ed, 0x3843, 0x3b7e, 0x3e9b, 0x4199, 0x4479, 0x473a, 0x49dc, 0x4c60, 0x4ec6, 0x5110, 0x533f, 0x5552,
0x574c, 0x592d, 0x5af7, 0x5caa, 0x5e48, 0x5fd2, 0x6149, 0x62ad, 0x6400, 0x6542, 0x6675, 0x679a, 0x68b0, 0x69b9, 0x6ab6, 0x6ba7,
0x6c8d, 0x6d68, 0x6e39, 0x6f01, 0x6fbf, 0x7075, 0x7123, 0x71ca, 0x7269, 0x7301, 0x7393, 0x741e, 0x74a4, 0x7524, 0x759f, 0x7614,
0x7685, 0x76f1, 0x7758, 0x77bc, 0x781b, 0x7877, 0x78cf, 0x7924, 0x7975, 0x79c3, 0x7a0e, 0x7a56, 0x7a9b, 0x7add, 0x7b1d, 0x7b5b,
0x7b96, 0x7bcf, 0x7c05, 0x7c3a, 0x7c6d, 0x7c9d, 0x7ccc, 0x7cf9, 0x7d24, 0x7d4d, 0x7d75, 0x7d9b, 0x7dc0, 0x7de3, 0x7e05, 0x7e25,
0x7e44, 0x7e62, 0x7e7f, 0x7e9a, 0x7eb4, 0x7ecd, 0x7ee4, 0x7efb, 0x7f11, 0x7f25, 0x7f38, 0x7f4b, 0x7f5c, 0x7f6d, 0x7f7c, 0x7f8b,
0x7f99, 0x7fa5, 0x7fb1, 0x7fbc, 0x7fc6, 0x7fd0, 0x7fd8, 0x7fe0, 0x7fe7, 0x7fed, 0x7ff2, 0x7ff6, 0x7ffa, 0x7ffc, 0x7ffe, 0x7fff,
0x7fff, 0x7fff, 0x7ffe, 0x7ffc, 0x7ffa, 0x7ff6, 0x7ff2, 0x7fed, 0x7fe7, 0x7fe0, 0x7fd8, 0x7fd0, 0x7fc6, 0x7fbc, 0x7fb1, 0x7fa5,
0x7f99, 0x7f8b, 0x7f7c, 0x7f6d, 0x7f5c, 0x7f4b, 0x7f38, 0x7f25, 0x7f11, 0x7efb, 0x7ee4, 0x7ecd, 0x7eb4, 0x7e9a, 0x7e7f, 0x7e62,
0x7e44, 0x7e25, 0x7e05, 0x7de3, 0x7dc0, 0x7d9b, 0x7d75, 0x7d4d, 0x7d24, 0x7cf9, 0x7ccc, 0x7c9d, 0x7c6d, 0x7c3a, 0x7c05, 0x7bcf,
0x7b96, 0x7b5b, 0x7b1d, 0x7add, 0x7a9b, 0x7a56, 0x7a0e, 0x79c3, 0x7975, 0x7924, 0x78cf, 0x7877, 0x781b, 0x77bc, 0x7758, 0x76f1,
0x7685, 0x7614, 0x759f, 0x7524, 0x74a4, 0x741e, 0x7393, 0x7301, 0x7269, 0x71ca, 0x7123, 0x7075, 0x6fbf, 0x6f01, 0x6e39, 0x6d68,
0x6c8d, 0x6ba7, 0x6ab6, 0x69b9, 0x68b0, 0x679a, 0x6675, 0x6542, 0x6400, 0x62ad, 0x6149, 0x5fd2, 0x5e48, 0x5caa, 0x5af7, 0x592d,
0x574c, 0x5552, 0x533f, 0x5110, 0x4ec6, 0x4c60, 0x49dc, 0x473a, 0x4479, 0x4199, 0x3e9b, 0x3b7e, 0x3843, 0x34ed, 0x317d, 0x2df6,
0x2a5b, 0x26b2, 0x2300, 0x1f4c, 0x1b9d, 0x17fc, 0x1473, 0x110e, 0x0dd8, 0x0adc, 0x0827, 0x05c4, 0x03c0, 0x0222, 0x00f5, 0x003e};

void InitSystem(void);
void ConfigPort(void);
void SYS_GlobalIntEnable(void);
void SYS_GlobalIntDisable(void);
void do_fft(Int16 *real_data, Int16 *fft_real, Int16 *fft_imag, Uint16 scale);
void do_ifft(Int16 *fft_real, Int16 *fft_imag, Int16 *ifft_data, Uint16 scale);

/* Choose AIC3204 sampling rate */
short fs = AIC3204_FS_48KHZ;

void main(void)
{
	Uint16 i;
	Uint16 pressed_key, processing;

	InitSystem();
	ConfigPort();

	SYS_GlobalIntEnable(); /* Enable global interrupts */
	IER0 = 0x6080;         /* Enable DMA0 interrupt */

    I2S0_setup();          /* Setup I2S0 */

	AIC3204_Init();        /* Initialize AIC3204 */

	DMA0_setup();          /* Setup DMA0 CH0-3 */

   	I2S0_enable();         /* Enable I2S0 */
   	DMA0_enable_CH(0);	   /* Enable DMA0CH0 */

   	I2S0_enable();		   /* Enable I2S0 */
   	DMA0_enable_CH(1);	   /* Enable DMA0CH1 */

   	I2S0_enable();		   /* Enable I2S0 */
   	DMA0_enable_CH(2);	   /* Enable DMA0CH2 */

   	I2S0_enable();		   /* Enable I2S0 */
   	DMA0_enable_CH(3);	   /* Enable DMA0CH3 */

	Init_SAR(); /* Initialize SAR for switch functionality */

	/* Initialize buffers */
	for (i = 0; i < WND_LEN; i++) {
		OverlapInL[i] = 0;
		OverlapInR[i] = 0;
	}
	for (i = 0; i < OVERLAP_LENGTH; i++) {
		OverlapOutL[i] = 0;
		OverlapOutR[i] = 0;
	}
	processing = 0; /* Begin without processing */

	/* Begin infinite loop */
	while (1) {

        /* Check for switch activity */
        pressed_key = Get_Sar_Key();
        if (pressed_key == SW1) {
        	processing = 0;
        }
        if (pressed_key == SW2) {
        	processing = 1;
        }

        /* Get new input audio block */
		if (PingPongFlagInL && PingPongFlagInR) {
			for (i = 0; i < HOP_SIZE; i++) {
		       	/* Copy previous NEW data to current OLD data */
		       	OverlapInL[i] = OverlapInL[i + HOP_SIZE];
		       	OverlapInR[i] = OverlapInR[i + HOP_SIZE];

		       	/* Update NEW data with current audio in */
		       	OverlapInL[i + HOP_SIZE] = DMA_InpL[i + AUDIO_IO_SIZE];
		       	OverlapInR[i + HOP_SIZE] = DMA_InpR[i + AUDIO_IO_SIZE];
		    }
		}
		else {
			for (i = 0; i < HOP_SIZE; i++) {
		       	/* Copy previous NEW data to current OLD data */
		       	OverlapInL[i] = OverlapInL[i + HOP_SIZE];
		       	OverlapInR[i] = OverlapInR[i + HOP_SIZE];

		       	/* Update NEW data with current audio in */
		       	OverlapInL[i + HOP_SIZE] = DMA_InpL[i];
		       	OverlapInR[i + HOP_SIZE] = DMA_InpR[i];
		    }
		}
		/* Create windowed buffer for processing */
        for (i = 0; i < WND_LEN; i++) {
        	BufferL[i] = _smpy(OverlapInL[i], window[i]);
        	BufferR[i] = _smpy(OverlapInR[i], window[i]);
        }

		/* Perform FFT on windowed buffer */
   		do_fft(BufferL, realL, imagL, 1);
   		do_fft(BufferR, realR, imagR, 1);

		/* Process freq. bins from 0Hz to Nyquist frequency */
		for (i = 0; i < NUM_BINS; i++) {

			if (processing) {
				/* Perform spectral processing here */
				/* ... */
			}

		}

		/* Complete symmetric frequencies (since audio data are real) */
		for (i = 1; i < FFT_LENGTH/2; i++) {
			realL[FFT_LENGTH - i] = realL[i];
			imagL[FFT_LENGTH - i] = -imagL[i]; /* conjugation */
			realR[FFT_LENGTH - i] = realR[i];
			imagR[FFT_LENGTH - i] = -imagR[i]; /* conjugation */
		}

		/* Perform IFFT */
   		do_ifft(realL, imagL, BufferL, 0);
   		do_ifft(realR, imagR, BufferR, 0);

        if (PingPongFlagOutL && PingPongFlagOutR) {
        	for (i = 0; i < OVERLAP_LENGTH; i++) {
        		/* Current output block is previous overlapped block + current processed block */
        		DMA_OutL[i + AUDIO_IO_SIZE] = OverlapOutL[i] + _smpy(BufferL[i], recwin[i]);
        		DMA_OutR[i + AUDIO_IO_SIZE] = OverlapOutR[i] + _smpy(BufferR[i], recwin[i]);

				/* Update overlap buffer */
        		OverlapOutL[i] = _smpy(BufferL[i + HOP_SIZE], recwin[i + HOP_SIZE]);
        		OverlapOutR[i] = _smpy(BufferR[i + HOP_SIZE], recwin[i + HOP_SIZE]);
        	}
        } else {
        	for (i = 0; i < OVERLAP_LENGTH; i++) {
        		/* Current output block is previous overlapped block + current processed block */
        		DMA_OutL[i] = OverlapOutL[i] + _smpy(BufferL[i], recwin[i]);
        		DMA_OutR[i] = OverlapOutR[i] + _smpy(BufferR[i], recwin[i]);

        		/* Update overlap buffer */
        		OverlapOutL[i] = _smpy(BufferL[i + HOP_SIZE], recwin[i + HOP_SIZE]);
        		OverlapOutR[i] = _smpy(BufferR[i + HOP_SIZE], recwin[i + HOP_SIZE]);
        	}
        }
	//}

}

/* ======================== Initialize DSP System Clock =========================  */
/* ------------------------------------------------------------------------------  */
/* For more info see SPRUFX5D - Section 1.4.3.2.6 (copying below for reference:)   */
/* You can follow the steps below to program the PLL of the DSP clock generator.   */
/* The recommendation is to stop all peripheral operation before changing the PLL  */
/* frequency, with the exception of the device CPU and USB. The device CPU must be */
/* operational to program the PLL controller. Software is responsible for ensuring */
/* the PLL remains in BYPASS_MODE for at least 4 ms before switching to PLL_MODE.  */
/* 1. Make sure the clock generator is in BYPASS MODE by setting SYSCLKSEL = 0.    */
/* 2. Program RDRATIO, M, and RDBYPASS in CGCR1 and CGCR2 according to your        */
/*    required settings.														   */
/* 3. Program ODRATIO and OUTDIVEN in CGCR4 according to your required settings.   */
/* 4. Write 0806h to the INIT field of CGCR3.									   */
/* 5. Set PLL_PWRDN = 0.														   */
/* 6. Wait 4 ms for the PLL to complete its phase-locking sequence.				   */
/* 7. Place the clock generator in its PLL MODE by setting SYSCLKSEL = 1.		   */
/* ------------------------------------------------------------------------------  */
/* Note: This is a suggested sequence. It is most important to have all 		   */
/* programming done before the last step to place the clock generator in PLL MODE. */
/* =============================================================================== */
void InitSystem(void)
{
	Uint16 i;

	/*Clock Configuration Register 2 (CCR2) - Section 1.4.4.6 */
    CONFIG_MSW = 0x0; /* System Clock Generator is in Bypass Mode */

#if  (PLL_100M == 1)
	/* CGCR2 - Section 1.4.4.2 */
    PLL_CNTL2 = 0x8000; /* Bypass reference divider */
    /* CGCR4 - Section 1.4.4.4 */
    PLL_CNTL4 = 0x0000; /* Bypass output divider */
    /* CGCR3 - Section 1.4.4.3 */
    PLL_CNTL3 = 0x0806; /* initialization bits */
    /* CGCR1 - Section 1.4.4.1 */
    /* PLL power up. PLL Multiplier M = 1000 */
    PLL_CNTL1 = 0x8BE8; //PG1.4: 0x82FA;

#elif (PLL_12M == 1)
    PLL_CNTL2 = 0x8000;
    PLL_CNTL4 = 0x0200;
    PLL_CNTL3 = 0x0806;
    PLL_CNTL1 = 0x82ED;

#elif (PLL_98M == 1)
    PLL_CNTL2 = 0x8000;
    PLL_CNTL4 = 0x0000;
    PLL_CNTL3 = 0x0806;
    PLL_CNTL1 = 0x82ED;

#elif (PLL_40M == 1)
    PLL_CNTL2 = 0x8000;
    PLL_CNTL4 = 0x0300;
    PLL_CNTL3 = 0x0806;
    PLL_CNTL1 = 0x8262;
#endif

    while ( (PLL_CNTL3 & 0x0008) == 0);

	/*Clock Configuration Register 2 (CCR2) - Section 1.4.4.6 */
    CONFIG_MSW = 0x1; /* System Clock Generator is in PLL Mode */

    /* Peripheral Clock Gating Configuration Register 1 (PCGCR1) - Section 1.5.3.2.1 */
    IDLE_PCGCR = 0x0020; /* System clock and other clocks active */
    					 /* According to Table 1-24 bit 5 should be always set to 1 */
    /* Peripheral Clock Gating Configuration Register 2 (PCGCR2) - Section 1.5.3.2.1 */
    IDLE_PCGCR_MSW = 0x0000; /* Enable SAR clock */

	/* Peripheral Software Reset Counter Register (PSRCR) - Section 1.7.5.1 */
    PER_RSTCOUNT = 0x08; /* Software reset signals asserted after 2 clock cycles */
    					 /* NOTE: p.75 states this value must be at least 0x08   */
    /* Peripheral Reset Control Register (PRCR) - Section 1.7.5.2 */
    PER_RESET = 0x00FF; /* Reset ALL peripherals */

    for (i=0; i< 0xFFFF; i++);
}

/* ========================= Configure External Busses ==========================  */
/* ------------------------------------------------------------------------------  */
/* For more info see SPRUFX5D - Section 1.7.3.1									   */
/* =============================================================================== */
void ConfigPort(void)
{
    Int16 i;

    /* External Bus Selection Register (EBSR) - Section 1.7.3.1 */
    PERIPHSEL0 = 0x1000; /* PPMODE = 110 - Mode 6 */
    					 /* SP1MODE = 10 - Mode 2 */
						 /* SP0MODE = 01 - Mode 1 */

    for (i=0; i< 0xFFF; i++);
}

void SYS_GlobalIntEnable(void)
{
    asm(" BIT (ST1, #ST1_INTM) = #0");
}

void SYS_GlobalIntDisable(void)
{
    asm(" BIT (ST1, #ST1_INTM) = #1");
}

/* ---------------- Wrapper function to implement HWAFFT ---------------- */
/* For more information see: Application Report - SPRABB6A, June 2010     */
/* Note that there are also several discussions in e2e.ti.com forums.     */
/* The appropriate functions are linked via c5515.cmd. See also:          */
/* http://e2e.ti.com/support/dsp/c5000/f/109/p/49635/176118.aspx#176118   */
/*																		  */
/* The function requires pre-allocated arrays for input data (16-bit Q15) */
/* and real and imaginray parts of the FFT.                               */
/* ---------------------------------------------------------------------- */
void do_fft(Int16 *real_data, Int16 *fft_real, Int16 *fft_imag, Uint16 scale)
{
	Uint16 i, data_selection;
	Int32 *complex_data, *bitrev_data, *temp_data, *fft_data;

	/* Initialize relevant pointers */
	bitrev_data  = bitreversed_buffer;
	temp_data    = temporary_buffer;
	complex_data = complex_buffer;

	/* Convert real data to "pseudo"-complex data (real, 0) */
	/* Int32 complex = Int16 real (MSBs) + Int16 imag (LSBs) */
	for (i = 0; i < FFT_LENGTH; i++) {
		*(complex_data + i) = ( (Int32) (*(real_data + i)) ) << 16;
	}

	/* Perform bit-reversing */
	hwafft_br(complex_data, bitrev_data, FFT_LENGTH);

	/* Perform FFT */
	if (scale) {
		data_selection = hwafft_256pts(bitrev_data, temp_data, FFT_FLAG, SCALE_FLAG);
	} else {
		data_selection = hwafft_256pts(bitrev_data, temp_data, FFT_FLAG, NOSCALE_FLAG);
	}

	/* Return appropriate data pointer */
	if (data_selection) {
		fft_data = temp_data;
	} else {
		fft_data = bitrev_data;
	}

	/* Extract real and imaginary parts */
	for (i = 0; i < FFT_LENGTH; i++) {
		*(fft_real + i) = (Int16)((*(fft_data + i)) >> 16);
		*(fft_imag + i) = (Int16)((*(fft_data + i)) & 0x0000FFFF);
	}

}

/* ---------------- Wrapper function to implement HWAIFFT --------------- */
/* For more information see: Application Report - SPRABB6A, June 2010     */
/* Note that there are also several discussions in e2e.ti.com forums.     */
/* The appropriate functions are linked via c5515.cmd. See also:          */
/* http://e2e.ti.com/support/dsp/c5000/f/109/p/49635/176118.aspx#176118   */
/*																		  */
/* The function requires pre-allocated arrays for output data (16-bit Q15)*/
/* of the IFFT.							                                  */
/* ---------------------------------------------------------------------- */
void do_ifft(Int16 *fft_real, Int16 *fft_imag, Int16 *ifft_data, Uint16 scale)
{
	Uint16 i, data_selection;
	Int32 *complex_data, *bitrev_data, *temp_data, *ifft_tmp;

	/* Initialize relevant pointers */
	complex_data = complex_buffer;
	bitrev_data  = bitreversed_buffer;
	temp_data    = temporary_buffer;

	/* Reconstruct complex data from real and imaginary parts */
	for (i = 0; i < FFT_LENGTH; i++) {
		*(complex_data + i) = ((Int32)(*(fft_real + i)) << 16);
		*(complex_data + i) = *(complex_data + i) | ((Int32)(*(fft_imag + i)) & 0x0000FFFF);
	}

	/* Perform bit-reversing */
	hwafft_br(complex_data, bitrev_data, FFT_LENGTH);

	/* Perform IFFT */
	if (scale) {
		data_selection = hwafft_256pts(bitrev_data, temp_data, IFFT_FLAG, SCALE_FLAG);
	} else {
		data_selection = hwafft_256pts(bitrev_data, temp_data, IFFT_FLAG, NOSCALE_FLAG);
	}

	/* Return appropriate data pointer */
	if (data_selection) {
		ifft_tmp = temp_data;
	} else {
		ifft_tmp = bitrev_data;
	}

	/* Return real part of IFFT */
	for (i = 0; i < FFT_LENGTH; i++) {
		*(ifft_data + i) = (Int16)((*(ifft_tmp + i)) >> 16);
	}
}
