#ifndef GN3S_DEFINES_H_
#define GN3S_DEFINES_H_

typedef struct GN3S_CPX
{
	short int i;	//!< Real value
	short int q;	//!< Imaginary value
} GN3S_CPX;

//#define GN3S_SAMPS_MS				(2048)						//!< All incoming signals are resampled to this sampling frequency
#define GN3S_SAMPS_5MS				(40919)						// 5MS at fs=8.1838e6
//!< FIFO structure for linked list?
/*----------------------------------------------------------------------------------------------*/
/*! \ingroup STRUCTS
 *  @brief linked list structure for circular FIFO buffer */
typedef struct gn3s_ms_packet {

	gn3s_ms_packet *next;
	int count;					//!< Number of packets
	GN3S_CPX data[GN3S_SAMPS_5MS];				//!< Payload size

} gn3s_ms_packet;
/*----------------------------------------------------------------------------------------------*/

#endif //GN3S_DEFINES_H_
