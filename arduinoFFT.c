/*

	FFT libray
	Copyright (C) 2010 Didier Longueville
	Copyright (C) 2014 Enrique Condes

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#include "arduinoFFT.h"
//
//arduinoFFT(void)
//{ // Constructor
//	#warning("This method is deprecated and may be removed on future revisions.")
//}
//
//arduinoFFT(double *vReal, double *vImag, uint16_t samples, double samplingFrequency)
//{// Constructor
//	this->_vReal = vReal;
//	this->_vImag = vImag;
//	this->_samples = samples;
//	this->_samplingFrequency = samplingFrequency;
//	this->_power = Exponent(samples);
//}
//
//~arduinoFFT(void)
//{
//// Destructor
//}
//
//uint8_t Revision(void)
//{
//	return(FFT_LIB_REV);
//}
//
#include "debug.h"
float_t WF[32] = {0.079999, 0.082285,   0.08912,    0.100436,   0.11612,    0.136018,   0.15993,
         0.187619,   0.218811,   0.253194,   0.290428,   0.330143,   0.371943,   0.415413,   0.460121,
         0.505624,   0.551468,   0.597198,   0.642359,   0.686503,   0.729192,   0.77,   0.808522,
         0.844375,   0.877203,   0.90668,    0.932513,   0.954445,   0.972258,   0.985775,   0.994862,   0.999428};

float C1[6] = {0, 0.707106,0.923879, 0.923879, 0.995184,0.995184};
float C2[6] = {-1.0, -0.707106,-0.382683, -0.19509, -0.098017,-0.049067};
//float U2[63] = {0 ,-1.0 ,0,-0.7071 ,-1.0 ,-0.7071 ,0 ,-0.3827 ,-0.7071 ,-0.9239 ,-1.0 ,-0.9239 ,-0.7071,
//        -0.3827 ,0.0 ,-0.1951 ,-0.3605 ,-0.4921 ,-0.5879 ,-0.6476 ,-0.6723 ,-0.6649 ,-0.6292 ,-0.5697,
//        -0.4917 ,-0.4006 ,-0.3018 ,-0.2004 ,-0.1013 ,-0.0085 ,0.0747 ,-0.0980 ,-0.1951 ,-0.2903 ,-0.3827 ,
//        -0.4714 ,-0.5556 ,-0.6344 ,-0.7071 ,-0.7730 ,-0.8315 ,-0.8819 ,-0.9239 ,-0.9569 ,-0.9808 ,-0.9952 ,
//        -1.0 ,-0.9952 ,-0.9808 ,-0.9569 ,-0.9239 ,-0.8819 ,-0.8315 ,-0.7730 ,-0.7071 ,-0.6344 ,-0.5556 ,
//        -0.4714 ,-0.3827 ,-0.2903 ,-0.1951 ,-0.0980 ,0 };
//float U1[63] = { -1.0 , 0 , -1.0, 0.7071 , 0  , -0.7071 , -1.0, 0.9239 , 0.7071 , 0.3827 , 0 , -0.3827 , -0.7071 , -0.9239 ,
//        -1.0 , 0.9239 , 0.8155 , 0.6831 , 0.5351 , 0.3797 , 0.2244 , 0.0762 , -0.0594 , -0.1776 , -0.2752 , -0.3502 ,
//        -0.4017 , -0.4300 , -0.4364 , -0.4229 , -0.3924 , 0.9952 , 0.9808 , 0.9569 , 0.9239 , 0.8819 , 0.8315 , 0.7730 ,
//        0.7071 , 0.6344 , 0.5556 , 0.4714 , 0.3827 , 0.2903 , 0.1951 , 0.0980 , 0 , -0.0980 , -0.1951 , -0.2903 , -0.3827 ,
//        -0.4714 , -0.5556 , -0.6344 , -0.7071 , -0.7730 , -0.8315 , -0.8819 , -0.9239 , -0.9569 , -0.9808 , -0.9952 , -1.0};

void FFT_Compute(float_t *vReal, float_t *vImag, uint16_t samples, uint8_t dir)
{
//	#warning("This method is deprecated and may be removed on future revisions.")
	FFT_Compute_f(vReal, vImag, samples, FFT_Exponent(samples), dir);
}
//
//void Compute(uint8_t dir)
//{// Computes in-place complex-to-complex FFT /
//	// Reverse bits /
//	uint16_t j = 0;
//	for (uint16_t i = 0; i < (this->_samples - 1); i++) {
//		if (i < j) {
//			Swap(&this->_vReal[i], &this->_vReal[j]);
//			if(dir==FFT_REVERSE)
//				Swap(&this->_vImag[i], &this->_vImag[j]);
//		}
//		uint16_t k = (this->_samples >> 1);
//		while (k <= j) {
//			j -= k;
//			k >>= 1;
//		}
//		j += k;
//	}
//	// Compute the FFT  /
//#ifdef __AVR__
//	uint8_t index = 0;
//#endif
//	double c1 = -1.0;
//	double c2 = 0.0;
//	uint16_t l2 = 1;
//	for (uint8_t l = 0; (l < this->_power); l++) {
//		uint16_t l1 = l2;
//		l2 <<= 1;
//		double u1 = 1.0;
//		double u2 = 0.0;
//		for (j = 0; j < l1; j++) {
//			 for (uint16_t i = j; i < this->_samples; i += l2) {
//					uint16_t i1 = i + l1;
//					double t1 = u1 * this->_vReal[i1] - u2 * this->_vImag[i1];
//					double t2 = u1 * this->_vImag[i1] + u2 * this->_vReal[i1];
//					this->_vReal[i1] = this->_vReal[i] - t1;
//					this->_vImag[i1] = this->_vImag[i] - t2;
//					this->_vReal[i] += t1;
//					this->_vImag[i] += t2;
//			 }
//			 double z = ((u1 * c1) - (u2 * c2));
//			 u2 = ((u1 * c2) + (u2 * c1));
//			 u1 = z;
//		}
//#ifdef __AVR__
//		c2 = pgm_read_float_near(&(_c2[index]));
//		c1 = pgm_read_float_near(&(_c1[index]));
//		index++;
//#else
//		c2 = sqrt((1.0 - c1) / 2.0);
//		c1 = sqrt((1.0 + c1) / 2.0);
//#endif
//		if (dir == FFT_FORWARD) {
//			c2 = -c2;
//		}
//	}
//	// Scaling for reverse transform /
//	if (dir != FFT_FORWARD) {
//		for (uint16_t i = 0; i < this->_samples; i++) {
//			 this->_vReal[i] /= this->_samples;
//			 this->_vImag[i] /= this->_samples;
//		}
//	}
//}

//void FFT_Compute_2(double *vReal, double *vImag, uint16_t samples, uint8_t power, uint8_t dir)
//{	// Computes in-place complex-to-complex FFT
//	// Reverse bits
////	#warning("This method is deprecated and may be removed on future revisions.")
//	uint16_t j = 0;
//	for (uint16_t i = 0; i < (samples - 1); i++) {
//		if (i < j) {
//			FFT_Swap(&vReal[i], &vReal[j]);
//			if(dir==FFT_REVERSE)
//				FFT_Swap(&vImag[i], &vImag[j]);
//		}
//		uint16_t k = (samples >> 1);
//		while (k <= j) {
//			j -= k;
//			k >>= 1;
//		}
//		j += k;
//	}
//	// Compute the FFT
//#ifdef __AVR__
//	uint8_t index = 0;
//#endif
//	double c1 = -1.0;
//	double c2 = 0.0;
//	uint16_t l2 = 1;
//	for (uint8_t l = 0; (l < power); l++) {
//		uint16_t l1 = l2;
//		l2 <<= 1;
//		double u1 = 1.0;
//		double u2 = 0.0;
//		for (j = 0; j < l1; j++) {
//			 for (uint16_t i = j; i < samples; i += l2) {
//					uint16_t i1 = i + l1;
//					double t1 = u1 * vReal[i1] - u2 * vImag[i1];
//					double t2 = u1 * vImag[i1] + u2 * vReal[i1];
//					vReal[i1] = vReal[i] - t1;
//					vImag[i1] = vImag[i] - t2;
//					vReal[i] += t1;
//					vImag[i] += t2;
//			 }
//			 double z = ((u1 * c1) - (u2 * c2));
//			 u2 = ((u1 * c2) + (u2 * c1));
//			 u1 = z;
//		}
//#ifdef __AVR__
//		c2 = pgm_read_float_near(&(_c2[index]));
//		c1 = pgm_read_float_near(&(_c1[index]));
//		index++;
//#else
//		c2 = sqrt((1.0 - c1) / 2.0);
//		c1 = sqrt((1.0 + c1) / 2.0);
//#endif
//		if (dir == FFT_FORWARD) {
//			c2 = -c2;
//		}
//	}
//	// Scaling for reverse transform
//	if (dir != FFT_FORWARD) {
//		for (uint16_t i = 0; i < samples; i++) {
//			 vReal[i] /= samples;
//			 vImag[i] /= samples;
//		}
//	}
//}
//

void FFT_Compute_f(float_t *vReal, float_t *vImag, uint16_t samples, uint8_t power, uint8_t dir)
{   // Computes in-place complex-to-complex FFT
    // Reverse bits
//  #warning("This method is deprecated and may be removed on future revisions.")
    uint16_t j = 0;
    for (uint16_t i = 0; i < (samples - 1); i++) {
        if (i < j) {
            FFT_Swap(&vReal[i], &vReal[j]);
            if(dir==FFT_REVERSE)
                FFT_Swap(&vImag[i], &vImag[j]);
        }
        uint16_t k = (samples >> 1);
        while (k <= j) {
            j -= k;
            k >>= 1;
        }
        j += k;
    }
    // Compute the FFT
//#ifdef __AVR__
//    uint8_t index = 0;
//#endif
    float_t c1 = -1.0;
    float_t c2 = 0.0;
    uint16_t l2 = 1;
//    int h = 0;
    for (uint8_t l = 0; (l < power); l++) {
        uint16_t l1 = l2;
        l2 <<= 1;
        float_t u1 = 1.0;
        float_t u2 = 0.0;
        for (j = 0; j < l1; j++) {
             for (uint16_t i = j; i < samples; i += l2) {
                    uint16_t i1 = i + l1;
                    float_t t1 = u1 * vReal[i1] - u2 * vImag[i1];
                    float_t t2 = u1 * vImag[i1] + u2 * vReal[i1];
                    vReal[i1] = vReal[i] - t1;
                    vImag[i1] = vImag[i] - t2;
                    vReal[i] += t1;
                    vImag[i] += t2;
             }
//                     millis();
             float_t z = ((u1 * c1) - (u2 * c2));
             u2 = ((u1 * c2) + (u2 * c1));
             u1 = z;

//                u2 = U2[h];
//                u1 = U1[h];
//                h ++;

//                printf("%f,%f @ \r\n", u2, u1);

        }

//        millis();
//        c2 = sqrt((1.0 - c1) / 2.0);
//        c1 = sqrt((1.0 + c1) / 2.0);
//        printf("%f,%f @ %f\r\n", c2, c1, millis());
        c2 = -C2[l];
        c1 = C1[l];

//#endif
//        if (dir == FFT_FORWARD) {
//            c2 = -c2;
//        }
    }
    // Scaling for reverse transform
    if (dir != FFT_FORWARD) {
        for (uint16_t i = 0; i < samples; i++) {
             vReal[i] /= samples;
             vImag[i] /= samples;
        }
    }
//    Delay_Ms(2000);
}


//void ComplexToMagnitude()
//{ // vM is half the size of vReal and vImag
//	for (uint16_t i = 0; i < this->_samples; i++) {
//		this->_vReal[i] = sqrt(sq(this->_vReal[i]) + sq(this->_vImag[i]));
//	}
//}

void FFT_ComplexToMagnitude(float_t *vReal, float_t *vImag, uint16_t samples)
{	// vM is half the size of vReal and vImag
//	#warning("This method is deprecated and may be removed on future revisions.")
	for (uint16_t i = 0; i < samples; i++) {
		vReal[i] = sqrt(sq(vReal[i]) + sq(vImag[i]));
	}
}
//
//void DCRemoval()
//{
//	// calculate the mean of vData
//	double mean = 0;
//	for (uint16_t i = 0; i < this->_samples; i++)
//	{
//		mean += this->_vReal[i];
//	}
//	mean /= this->_samples;
//	// Subtract the mean from vData
//	for (uint16_t i = 0; i < this->_samples; i++)
//	{
//		this->_vReal[i] -= mean;
//	}
//}

void FFT_DCRemoval(float_t *vData, uint16_t samples )
{
	// calculate the mean of vData
//	#warning("This method is deprecated and may be removed on future revisions.")
    float_t mean = 0;
//	for (uint16_t i = 0; i < samples>>1; i++)
    for (uint16_t i = 0; i < samples; i++)
	{
		mean += vData[i];
	}
	mean /= samples;
	// Subtract the mean from vData
	for (uint16_t i = 0; i < samples; i++)
	{
		vData[i] -= mean;
	}
}

//void Windowing(uint8_t windowType, uint8_t dir)
//{// Weighing factors are computed once before multiple use of FFT
//// The weighing function is symetric; half the weighs are recorded
//	double samplesMinusOne = (double(this->_samples) - 1.0);
//	for (uint16_t i = 0; i < (this->_samples >> 1); i++) {
//		double indexMinusOne = double(i);
//		double ratio = (indexMinusOne / samplesMinusOne);
//		double weighingFactor = 1.0;
//		// Compute and record weighting factor
//		switch (windowType) {
//		case FFT_WIN_TYP_RECTANGLE: // rectangle (box car)
//			weighingFactor = 1.0;
//			break;
//		case FFT_WIN_TYP_HAMMING: // hamming
//			weighingFactor = 0.54 - (0.46 * cos(twoPi * ratio));
//			break;
//		case FFT_WIN_TYP_HANN: // hann
//			weighingFactor = 0.54 * (1.0 - cos(twoPi * ratio));
//			break;
//		case FFT_WIN_TYP_TRIANGLE: // triangle (Bartlett)
//			#if defined(ESP8266) || defined(ESP32)
//			weighingFactor = 1.0 - ((2.0 * fabs(indexMinusOne - (samplesMinusOne / 2.0))) / samplesMinusOne);
//			#else
//			weighingFactor = 1.0 - ((2.0 * abs(indexMinusOne - (samplesMinusOne / 2.0))) / samplesMinusOne);
//			#endif
//			break;
//		case FFT_WIN_TYP_NUTTALL: // nuttall
//			weighingFactor = 0.355768 - (0.487396 * (cos(twoPi * ratio))) + (0.144232 * (cos(fourPi * ratio))) - (0.012604 * (cos(sixPi * ratio)));
//			break;
//		case FFT_WIN_TYP_BLACKMAN: // blackman
//			weighingFactor = 0.42323 - (0.49755 * (cos(twoPi * ratio))) + (0.07922 * (cos(fourPi * ratio)));
//			break;
//		case FFT_WIN_TYP_BLACKMAN_NUTTALL: // blackman nuttall
//			weighingFactor = 0.3635819 - (0.4891775 * (cos(twoPi * ratio))) + (0.1365995 * (cos(fourPi * ratio))) - (0.0106411 * (cos(sixPi * ratio)));
//			break;
//		case FFT_WIN_TYP_BLACKMAN_HARRIS: // blackman harris
//			weighingFactor = 0.35875 - (0.48829 * (cos(twoPi * ratio))) + (0.14128 * (cos(fourPi * ratio))) - (0.01168 * (cos(sixPi * ratio)));
//			break;
//		case FFT_WIN_TYP_FLT_TOP: // flat top
//			weighingFactor = 0.2810639 - (0.5208972 * cos(twoPi * ratio)) + (0.1980399 * cos(fourPi * ratio));
//			break;
//		case FFT_WIN_TYP_WELCH: // welch
//			weighingFactor = 1.0 - sq((indexMinusOne - samplesMinusOne / 2.0) / (samplesMinusOne / 2.0));
//			break;
//		}
//		if (dir == FFT_FORWARD) {
//			this->_vReal[i] *= weighingFactor;
//			this->_vReal[this->_samples - (i + 1)] *= weighingFactor;
//		}
//		else {
//			this->_vReal[i] /= weighingFactor;
//			this->_vReal[this->_samples - (i + 1)] /= weighingFactor;
//		}
//	}
//}

// for 64 point Hamming Windowing forward
void FFT_Windowing_Fast64(float_t *vData, uint16_t samples)
{
    for (uint16_t i = 0; i < (samples >> 1); i++) {

            vData[i] *= WF[i];
            vData[samples - (i + 1)] *= WF[i];

    }

}
//void FFT_Windowing(float_t *vData, uint16_t samples, uint8_t windowType, uint8_t dir)
//{// Weighing factors are computed once before multiple use of FFT
//// The weighing function is symetric; half the weighs are recorded
////	#warning("This method is deprecated and may be removed on future revisions.")
//    float_t samplesMinusOne = (float_t)(samples) - 1.0;
//	for (uint16_t i = 0; i < (samples >> 1); i++) {
//	    float_t indexMinusOne = (float_t)i;
//	    float_t ratio = (indexMinusOne / samplesMinusOne);
//	    float_t weighingFactor = 1.0;
//		// Compute and record weighting factor
//		switch (windowType) {
//		case FFT_WIN_TYP_RECTANGLE: // rectangle (box car)
//			weighingFactor = 1.0;
//			break;
//		case FFT_WIN_TYP_HAMMING: // hamming
//			weighingFactor = 0.54 - (0.46 * cos(twoPi * ratio));
////			printf("weighingFactor=%f\r\n",weighingFactor);
//			break;
//		case FFT_WIN_TYP_HANN: // hann
//			weighingFactor = 0.54 * (1.0 - cos(twoPi * ratio));
//			break;
//		case FFT_WIN_TYP_TRIANGLE: // triangle (Bartlett)
//			#if defined(ESP8266) || defined(ESP32)
//			weighingFactor = 1.0 - ((2.0 * fabs(indexMinusOne - (samplesMinusOne / 2.0))) / samplesMinusOne);
//			#else
//			weighingFactor = 1.0 - ((2.0 * abs(indexMinusOne - (samplesMinusOne / 2.0))) / samplesMinusOne);
//			#endif
//			break;
//		case FFT_WIN_TYP_NUTTALL: // nuttall
//			weighingFactor = 0.355768 - (0.487396 * (cos(twoPi * ratio))) + (0.144232 * (cos(fourPi * ratio))) - (0.012604 * (cos(sixPi * ratio)));
//			break;
//		case FFT_WIN_TYP_BLACKMAN: // blackman
//			weighingFactor = 0.42323 - (0.49755 * (cos(twoPi * ratio))) + (0.07922 * (cos(fourPi * ratio)));
//			break;
//		case FFT_WIN_TYP_BLACKMAN_NUTTALL: // blackman nuttall
//			weighingFactor = 0.3635819 - (0.4891775 * (cos(twoPi * ratio))) + (0.1365995 * (cos(fourPi * ratio))) - (0.0106411 * (cos(sixPi * ratio)));
//			break;
//		case FFT_WIN_TYP_BLACKMAN_HARRIS: // blackman harris
//			weighingFactor = 0.35875 - (0.48829 * (cos(twoPi * ratio))) + (0.14128 * (cos(fourPi * ratio))) - (0.01168 * (cos(sixPi * ratio)));
//			break;
//		case FFT_WIN_TYP_FLT_TOP: // flat top
//			weighingFactor = 0.2810639 - (0.5208972 * cos(twoPi * ratio)) + (0.1980399 * cos(fourPi * ratio));
//			break;
//		case FFT_WIN_TYP_WELCH: // welch
//			weighingFactor = 1.0 - sq((indexMinusOne - samplesMinusOne / 2.0) / (samplesMinusOne / 2.0));
//			break;
//		}
//		if (dir == FFT_FORWARD) {
//			vData[i] *= weighingFactor;
//			vData[samples - (i + 1)] *= weighingFactor;
//		}
//		else {
//			vData[i] /= weighingFactor;
//			vData[samples - (i + 1)] /= weighingFactor;
//		}
//	}
//}
//
//double MajorPeak()
//{
//	double maxY = 0;
//	uint16_t IndexOfMaxY = 0;
//	//If sampling_frequency = 2 * max_frequency in signal,
//	//value would be stored at position samples/2
//	for (uint16_t i = 1; i < ((this->_samples >> 1) + 1); i++) {
//		if ((this->_vReal[i-1] < this->_vReal[i]) && (this->_vReal[i] > this->_vReal[i+1])) {
//			if (this->_vReal[i] > maxY) {
//				maxY = this->_vReal[i];
//				IndexOfMaxY = i;
//			}
//		}
//	}
//	double delta = 0.5 * ((this->_vReal[IndexOfMaxY-1] - this->_vReal[IndexOfMaxY+1]) / (this->_vReal[IndexOfMaxY-1] - (2.0 * this->_vReal[IndexOfMaxY]) + this->_vReal[IndexOfMaxY+1]));
//	double interpolatedX = ((IndexOfMaxY + delta)  * this->_samplingFrequency) / (this->_samples-1);
//	if(IndexOfMaxY==(this->_samples >> 1)) //To improve calculation on edge values
//		interpolatedX = ((IndexOfMaxY + delta)  * this->_samplingFrequency) / (this->_samples);
//	// returned value: interpolated frequency peak apex
//	return(interpolatedX);
//}
//
//void MajorPeak(double *f, double *v)
//{
//	double maxY = 0;
//	uint16_t IndexOfMaxY = 0;
//	//If sampling_frequency = 2 * max_frequency in signal,
//	//value would be stored at position samples/2
//	for (uint16_t i = 1; i < ((this->_samples >> 1) + 1); i++) {
//		if ((this->_vReal[i - 1] < this->_vReal[i]) && (this->_vReal[i] > this->_vReal[i + 1])) {
//			if (this->_vReal[i] > maxY) {
//				maxY = this->_vReal[i];
//				IndexOfMaxY = i;
//			}
//		}
//	}
//	double delta = 0.5 * ((this->_vReal[IndexOfMaxY - 1] - this->_vReal[IndexOfMaxY + 1]) / (this->_vReal[IndexOfMaxY - 1] - (2.0 * this->_vReal[IndexOfMaxY]) + this->_vReal[IndexOfMaxY + 1]));
//	double interpolatedX = ((IndexOfMaxY + delta)  * this->_samplingFrequency) / (this->_samples - 1);
//	if (IndexOfMaxY == (this->_samples >> 1)) //To improve calculation on edge values
//		interpolatedX = ((IndexOfMaxY + delta)  * this->_samplingFrequency) / (this->_samples);
//	// returned value: interpolated frequency peak apex
//	*f = interpolatedX;
//	#if defined(ESP8266) || defined(ESP32)
//	*v = fabs(this->_vReal[IndexOfMaxY - 1] - (2.0 * this->_vReal[IndexOfMaxY]) + this->_vReal[IndexOfMaxY + 1]);
//	#else
//	*v = abs(this->_vReal[IndexOfMaxY - 1] - (2.0 * this->_vReal[IndexOfMaxY]) + this->_vReal[IndexOfMaxY + 1]);
//	#endif
//}
//
//double MajorPeak(double *vD, uint16_t samples, double samplingFrequency)
//{
//	#warning("This method is deprecated and may be removed on future revisions.")
//	double maxY = 0;
//	uint16_t IndexOfMaxY = 0;
	//If sampling_frequency = 2 * max_frequency in signal,
	//value would be stored at position samples/2
//	for (uint16_t i = 1; i < ((samples >> 1) + 1); i++) {
//		if ((vD[i-1] < vD[i]) && (vD[i] > vD[i+1])) {
//			if (vD[i] > maxY) {
//				maxY = vD[i];
//				IndexOfMaxY = i;
//			}
//		}
//	}
//	double delta = 0.5 * ((vD[IndexOfMaxY-1] - vD[IndexOfMaxY+1]) / (vD[IndexOfMaxY-1] - (2.0 * vD[IndexOfMaxY]) + vD[IndexOfMaxY+1]));
//	double interpolatedX = ((IndexOfMaxY + delta)  * samplingFrequency) / (samples-1);
//	if(IndexOfMaxY==(samples >> 1)) //To improve calculation on edge values
//		interpolatedX = ((IndexOfMaxY + delta)  * samplingFrequency) / (samples);
//	// returned value: interpolated frequency peak apex
//	return(interpolatedX);
//}
//
//void FFT_MajorPeak(float_t *vD, uint16_t samples, float_t samplingFrequency, float_t *f, float_t *v)
//{
////	#warning("This method is deprecated and may be removed on future revisions.")
//    float_t maxY = 0;
//	uint16_t IndexOfMaxY = 0;
//	//If sampling_frequency = 2 * max_frequency in signal,
//	//value would be stored at position samples/2
//	for (uint16_t i = 1; i < ((samples >> 1) + 1); i++) {
//		if ((vD[i - 1] < vD[i]) && (vD[i] > vD[i + 1])) {
//			if (vD[i] > maxY) {
//				maxY = vD[i];
//				IndexOfMaxY = i;
//			}
//		}
//	}
//	float_t delta = 0.5 * ((vD[IndexOfMaxY - 1] - vD[IndexOfMaxY + 1]) / (vD[IndexOfMaxY - 1] - (2.0 * vD[IndexOfMaxY]) + vD[IndexOfMaxY + 1]));
//	float_t interpolatedX = ((IndexOfMaxY + delta)  * samplingFrequency) / (samples - 1);
//	//double popo =
//	if (IndexOfMaxY == (samples >> 1)) //To improve calculation on edge values
//		interpolatedX = ((IndexOfMaxY + delta)  * samplingFrequency) / (samples);
//	// returned value: interpolated frequency peak apex
//	*f = interpolatedX;
//	#if defined(ESP8266) || defined(ESP32)
//	*v = fabs(vD[IndexOfMaxY - 1] - (2.0 * vD[IndexOfMaxY]) + vD[IndexOfMaxY + 1]);
//	#else
//	*v = abs(vD[IndexOfMaxY - 1] - (2.0 * vD[IndexOfMaxY]) + vD[IndexOfMaxY + 1]);
//	#endif
//}

uint8_t FFT_Exponent(uint16_t value)
{
//	#warning("This method may not be accessible on future revisions.")
	// Calculates the base 2 logarithm of a value
	uint8_t result = 0;
	while (((value >> result) & 1) != 1) result++;
	return(result);
}

// Private functions

void FFT_Swap(float_t *x, float_t *y)
{
	float_t temp = *x;
	*x = *y;
	*y = temp;
}
