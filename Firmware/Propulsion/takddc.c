# include <stdint.h>
# include "takddc.h"


// Takahashi's Discrete Digital Control
// call with each analog sample of process 
int16_t takDdc(ddcParameters* pPar, int16_t setpoint, int16_t process)
{
 if(pPar->mode)  
   {
    const int8_t    fdk0 =  2, // finite difference coefficients
                    fdk1 = -5,
                    fdk2 =  4,
                    fdk3 = -1;
    int16_t dProp = pPar->c_n1 - process; 
    int16_t error = setpoint - process;
    int16_t dDer  = fdk3*pPar->c_n3 +
                    fdk2*pPar->c_n2 +
                    fdk1*pPar->c_n1 +
                    fdk0*process;

    pPar->m += pPar->k_p*(dProp + pPar->k_i*error - pPar->k_d*dDer); 
    
     { // clamp
      if(pPar->m < pPar->mMin) pPar->m = pPar->mMin;
         else if(pPar->m > pPar->mMax) pPar->m = pPar->mMax;
     }
   }

 
 pPar->c_n3 = pPar->c_n2;
 pPar->c_n2 = pPar->c_n1;
 pPar->c_n1 = process;

 return pPar->m;
}


// Takahashi Discrete Digital Control PID and Period initialization
// call once to set parameters, or when they are changed
void takDdcSetPid(ddcParameters* pPar, int16_t p, int16_t i, int16_t d,
                                        int16_t t) 
{
 pPar->t = t;
 pPar->k_p = (int16_t)p;
 pPar->k_i = pPar->t / (int16_t)i;
 pPar->k_d = (int16_t)d / pPar->t;
}

// Takahashi Discrete Digital Control Output initialization
// call once to set parameters, or when they are changed
// call immediately before initial control, if output or process are stale
void takDdcSetOut(ddcParameters* pPar, int16_t min, int16_t max,
                                           int16_t output, int16_t process) 
{
 pPar->mMin = min;
 pPar->mMax = max;
 pPar->m = output;
 pPar->c_n3 = (pPar->c_n2 = (pPar->c_n1 = process));
}



