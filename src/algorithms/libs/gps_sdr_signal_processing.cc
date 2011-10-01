#include "gps_sdr_signal_processing.h"

#include <math.h>
#include <stdlib.h>
#include <cmath>

/*!
 * The SV ID is _prn=ID -1
 */
void code_gen_conplex(std::complex<float>* _dest, int32 _prn, unsigned int _chip_shift) {

	uint32 G1[1023];
	uint32 G2[1023];
	uint32 G1_register[10], G2_register[10];
	uint32 feedback1, feedback2;
	uint32 lcv, lcv2;
	uint32 delay;
	int32 prn = _prn-1; //Move the PRN code to fit an array indices

	/* G2 Delays as defined in GPS-ISD-200D */
	int32 delays[51] = {5, 6, 7, 8, 17, 18, 139, 140, 141, 251, 252, 254 ,255, 256, 257, 258, 469, 470, 471, 472,
		473, 474, 509, 512, 513, 514, 515, 516, 859, 860, 861, 862, 145, 175, 52, 21, 237, 235, 886, 657, 634, 762,
		355, 1012, 176, 603, 130, 359, 595, 68, 386};

	/* A simple error check */
	if((prn < 0) || (prn > 51))
		return;

	for(lcv = 0; lcv < 10; lcv++)
	{
		G1_register[lcv] = 1;
		G2_register[lcv] = 1;
	}

	/* Generate G1 & G2 Register */
	for(lcv = 0; lcv < 1023; lcv++)
	{
		G1[lcv] = G1_register[0];
		G2[lcv] = G2_register[0];

		feedback1 = G1_register[7]^G1_register[0];
		feedback2 = (G2_register[8] + G2_register[7] + G2_register[4] + G2_register[2] + G2_register[1] + G2_register[0]) & 0x1;

		for(lcv2 = 0; lcv2 < 9; lcv2++)
		{
			G1_register[lcv2] = G1_register[lcv2+1];
			G2_register[lcv2] = G2_register[lcv2+1];
		}

		G1_register[9] = feedback1;
		G2_register[9] = feedback2;
	}

	/* Set the delay */
	delay = 1023 - delays[prn];
	delay+=_chip_shift;
	delay %= 1023;
	/* Generate PRN from G1 and G2 Registers */
	for(lcv = 0; lcv < 1023; lcv++)
	{
		_dest[lcv] = std::complex<float>(G1[(lcv+_chip_shift)%1023]^G2[delay], 0);
		if(_dest[lcv].real()==0.0) //javi
		{
			_dest[lcv].real(-1.0);
		}
		delay++;
		delay %= 1023;
		//std::cout<<_dest[lcv].real(); //OK
	}

}


/*----------------------------------------------------------------------------------------------*/
/*!
 * code_gen, generate the given prn code
 * */
int32 code_gen(CPX *_dest, int32 _prn)
{

	uint32 G1[1023];
	uint32 G2[1023];
	uint32 G1_register[10], G2_register[10];
	uint32 feedback1, feedback2;
	uint32 lcv, lcv2;
	uint32 delay;
    int32 prn = _prn-1; //Move the PRN code to fit an array indices

	/* G2 Delays as defined in GPS-ISD-200D */
	int32 delays[51] = {5, 6, 7, 8, 17, 18, 139, 140, 141, 251, 252, 254 ,255, 256, 257, 258, 469, 470, 471, 472,
		473, 474, 509, 512, 513, 514, 515, 516, 859, 860, 861, 862, 145, 175, 52, 21, 237, 235, 886, 657, 634, 762,
		355, 1012, 176, 603, 130, 359, 595, 68, 386};

	/* A simple error check */
	if((prn < 0) || (prn > 51))
		return(0);

	for(lcv = 0; lcv < 10; lcv++)
	{
		G1_register[lcv] = 1;
		G2_register[lcv] = 1;
	}

	/* Generate G1 & G2 Register */
	for(lcv = 0; lcv < 1023; lcv++)
	{
		G1[lcv] = G1_register[0];
		G2[lcv] = G2_register[0];

		feedback1 = G1_register[7]^G1_register[0];
		feedback2 = (G2_register[8] + G2_register[7] + G2_register[4] + G2_register[2] + G2_register[1] + G2_register[0]) & 0x1;

		for(lcv2 = 0; lcv2 < 9; lcv2++)
		{
			G1_register[lcv2] = G1_register[lcv2+1];
			G2_register[lcv2] = G2_register[lcv2+1];
		}

		G1_register[9] = feedback1;
		G2_register[9] = feedback2;
	}

	/* Set the delay */
	delay = 1023 - delays[prn];

	/* Generate PRN from G1 and G2 Registers */
	for(lcv = 0; lcv < 1023; lcv++)
	{
		_dest[lcv].i = G1[lcv]^G2[delay];
		_dest[lcv].q = 0;

		delay++;
		delay %= 1023;
	}

	return(1);

}
/*----------------------------------------------------------------------------------------------*/



/*!
 * code_gen_complex_sampled, generate GPS L1 C/A code complex for the desired SV ID and sampled to specific sampling frequency
 */
void code_gen_complex_sampled(std::complex<float>* _dest, unsigned int _prn, int32 _fs, unsigned int _chip_shift)
{
	// This function is based on the GNU software GPS for MATLAB in the Kay Borre book
	std::complex<float> _code[1023];
	int32 _samplesPerCode,_codeValueIndex;
	float _ts;
	float _tc;
	const int32 _codeFreqBasis=1023000; //Hz
	const int32 _codeLength=1023;

	//--- Find number of samples per spreading code ----------------------------
	_samplesPerCode = round(_fs / (_codeFreqBasis / _codeLength));

	//--- Find time constants --------------------------------------------------
	_ts = 1/(float)_fs;   // Sampling period in sec
	_tc = 1/(float)_codeFreqBasis;  // C/A chip period in sec
	code_gen_conplex(_code,_prn,_chip_shift); //generate C/A code 1 sample per chip
	//std::cout<<"ts="<<_ts<<std::endl;
	//std::cout<<"tc="<<_tc<<std::endl;
	//std::cout<<"sv="<<_prn<<std::endl;
	for (int32 i=0;i<_samplesPerCode;i++)
	{
	    //=== Digitizing =======================================================

	    //--- Make index array to read C/A code values -------------------------
	    // The length of the index array depends on the sampling frequency -
	    // number of samples per millisecond (because one C/A code period is one
	    // millisecond).


	    _codeValueIndex = ceil((_ts * ((float)i+1)) / _tc)-1;

	    //--- Make the digitized version of the C/A code -----------------------
	    // The "upsampled" code is made by selecting values form the CA code
	    // chip array (caCode) for the time instances of each sample.
	    if (i==_samplesPerCode-1){
	        //--- Correct the last index (due to number rounding issues) -----------
	    	_dest[i] = _code[_codeLength-1];

	    }else{
	    	_dest[i] = _code[_codeValueIndex]; //repeat the chip -> upsample
	    }
	    //std::cout<<_codeValueIndex;

	}

}



/*----------------------------------------------------------------------------------------------*/
/*!
 * sine_gen, generate a full scale sinusoid of frequency f with sampling frequency fs for _samps samps and put it into _dest
 * */
void sine_gen(CPX *_dest, double _f, double _fs, signed int _samps) {

	signed int lcv;
	signed short c, s;
	float phase, phase_step;

	phase = 0;
	phase_step = (float)TWO_PI*_f/_fs;

	for(lcv = 0; lcv < _samps; lcv++) {
		c =	(signed short)floor(16383.0*cos(phase));
		s =	(signed short)floor(16383.0*sin(phase));
		_dest[lcv].i = c;
		_dest[lcv].q = s;

		phase += phase_step;
	}
}
/*----------------------------------------------------------------------------------------------*/

void sine_gen_complex(std::complex<float>* _dest, double _f, double _fs, unsigned int _samps) {

	double phase, phase_step;

	phase_step = ((double)TWO_PI*_f)/_fs;

	for(unsigned int i = 0; i < _samps; i++)	{

		//_dest[i] = std::complex<float>(16383.0*cos(phase), 16383.0*sin(phase));
		_dest[i] = std::complex<float>(cos(phase),sin(phase));

		phase += phase_step;
	}

}


/*----------------------------------------------------------------------------------------------*/
void sine_gen(CPX *_dest, double _f, double _fs, int32 _samps, double _p)
{

	int32 lcv;
	int16 c, s;
	double phase, phase_step;

	phase = _p;
	phase_step = (double)TWO_PI*_f/_fs;

	for(lcv = 0; lcv < _samps; lcv++)
	{
		c =	(int16)floor(16383.0*cos(phase));
		s =	(int16)floor(16383.0*sin(phase));
		_dest[lcv].i = c;
		_dest[lcv].q = s;

		phase += phase_step;
	}

}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
/*!
 * wipeoff_gen, generate a full scale sinusoid of frequency f with sampling frequency fs for _samps samps and put it into _dest
 * */
void wipeoff_gen(MIX *_dest, double _f, double _fs, int32 _samps)
{

	int32 lcv;
	int16 c, s;
	double phase, phase_step;

	phase = 0;
	phase_step = (double)TWO_PI*_f/_fs;

	for(lcv = 0; lcv < _samps; lcv++)
	{
		c =	(int16)floor(16383.0*cos(phase));
		s =	(int16)floor(16383.0*sin(phase));
		_dest[lcv].i = _dest[lcv].ni = c;
		_dest[lcv].q = s;
		_dest[lcv].nq = -s;

		phase += phase_step;
	}

}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void downsample(CPX *_dest, CPX *_source, double _fdest, double _fsource, int32 _samps)
{

	int32 lcv, k;
	uint32 phase_step;
	uint32 lphase, phase;

	phase_step = (uint32)floor((double)4294967296.0*_fdest/_fsource);

	k = lphase = phase = 0;

	for(lcv = 0; lcv < _samps; lcv++)
	{
		if(phase <= lphase)
		{
			_dest[k] = _source[lcv];
			k++;
		}

		lphase = phase;
		phase += phase_step;
	}

}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
/*!
 * Gather statistics and run AGC
 * */
int32 run_agc(CPX *_buff, int32 _samps, int32 _bits, int32 _scale)
{
	int32 lcv, num;
	int16 max, *p;
	int16 val;

	p = (int16 *)&_buff[0];

	val = (1 << _scale - 1);
	max = 1 << _bits;
	num = 0;

	if(_scale)
	{
		for(lcv = 0; lcv < 2*_samps; lcv++)
		{
			p[lcv] += val;
			p[lcv] >>= _scale;
			if(abs(p[lcv]) > max)
				num++;
		}
	}
	else
	{
		for(lcv = 0; lcv < 2*_samps; lcv++)
		{
			if(abs(p[lcv]) > max)
				num++;
		}
	}

	return(num);

}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
/*!
 * Get a rough first guess of scale value to quickly initialize agc
 * */
void init_agc(CPX *_buff, int32 _samps, int32 bits, int32 *scale)
{
	int32 lcv;
	int16 *p;
	int32 max;

	p = (int16 *)&_buff[0];

	max = 0;
	for(lcv = 0; lcv < 2*_samps; lcv++)
	{
		if(p[lcv] > max)
			max = p[lcv];
	}

	scale[0] = (1 << 14) / max;

}
/*----------------------------------------------------------------------------------------------*/
