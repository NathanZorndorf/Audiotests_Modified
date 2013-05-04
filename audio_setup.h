#ifndef AUDIO_SETUP_H_
#define AUDIO_SETUP_H_
/* NOTE: The following definitions assume 50% overlap */

#define HOP_SIZE 128            /* Hop size of overlap-add */
#define AUDIO_IO_SIZE  HOP_SIZE /* DMA transfer size */

#define PINGPONG 1 /* DMA Ping-Pong mode (1) OR single buffer transfers (0) */
/* Define size of ping-pong or single transfer buffers */
#if PINGPONG
	#define PING_PONG_SIZE (2 * AUDIO_IO_SIZE)
#else
	#define PING_PONG_SIZE AUDIO_IO_SIZE
#endif

#define WND_LEN     (2 * HOP_SIZE) /* Window length */
#define FFT_LENGTH  WND_LEN        /* FFT length equal to WND_LEN */
								   /* CAUTION: if you change HOP_SIZE and hence
								    * WND_LEN and FFT_LENGTH, you must also
								    * _manually_ change the corresponding 
								    * hwafft functions in do_fft(), do_ifft() */

#define NUM_BINS	(FFT_LENGTH/2 + 1) /* Useful freq. bins, there rest are symmetric */ 

#define OVERLAP_LENGTH (WND_LEN - HOP_SIZE)

#endif /*AUDIO_SETUP_H_*/
