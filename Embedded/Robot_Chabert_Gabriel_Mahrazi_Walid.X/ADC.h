/* 
 * File:   ADC.h
 * Author: GEII Robot
 *
 * Created on 7 octobre 2024, 12:13
 */

#ifndef ADC_H
#define	ADC_H


void InitADC1();
void ADC1StartConversionSequence();
unsigned int * ADCGetResult();
unsigned char ADCIsConversionFinished();
void ADCClearConversionFinishedFlag();

#endif	/* ADC_H */

