/* Implemented in 2023 by João Gabriel, electronics coordinator of Mangue Baja 2023/24 */
#ifndef FIR_H
#define FIR_H

#include <stdio.h>

/* You can change the size of the vetor if you needed */
#define SIZE 2

/* Here you can change the filter coefficient manually */
//const float a = 0.6,
//            b = 0.6;

/* Can use only in one variable. Using the second variable the value of the static float has change */
class FIR  {
    public:
    /*========================================================================================================
        * Filter based on convolution between previous inputs.
        * Can suport only for order equal 2
        * @param x is the variable used for the filtering
        * @param a_coef, b_coef is the coefficients to the filter;
        *   The higher the value of the coefficient, the slower its update rate, but the stronger its filter.
        * @param type_filtering if needeed apply your filter two times type_filtering=true   
    ==========================================================================================================*/
        float filt(float x, float a_coef, float b_coef, bool type_filtering=false);

    private:
    /*=========================
        * Update the vetor
    ==========================*/
        void move_vec(float *vetorAddr, int size, float value); 
        /* Second filter */
        float filtfilt(float x2, float a_coef, float b_coef); // Second filter if you defined
        float double_y;
        bool flag;
    
    protected:
        float a_coef, b_coef;
};

float FIR::filt(float x, float a_coef, float b_coef, bool type_filtering)
{
    /* @var flag is the flag to determinate if your filter is apply two times */
    if (type_filtering) this->flag=type_filtering;

    static float y_pass[SIZE] = {0,0}, x_pass[SIZE] = {0,0};

    float y = (a_coef+b_coef)*y_pass[0] - a_coef*b_coef*y_pass[1] + (1-a_coef-b_coef + a_coef*b_coef)*x_pass[1];
    
    move_vec(y_pass, SIZE, y);
    move_vec(x_pass, SIZE, x);

    return (flag) ? filtfilt(y, a_coef, b_coef) : y; 
};

void FIR::move_vec(float *vetorAddr, int size, float value)
{
    for (int k=size-1; k>0; k--)
    {
        *(vetorAddr+k) = *(vetorAddr+k-1);
    }

    *vetorAddr = value;
};

float FIR::filtfilt(float x2, float a_coef, float b_coef)
{
    static float y2_pass[SIZE] = {0,0}, x2_pass[SIZE] = {0,0};

    float y2 = (a_coef+b_coef)*y2_pass[0] - a_coef*b_coef*y2_pass[1] + (1-a_coef-b_coef + a_coef*b_coef)*x2_pass[1];
    
    move_vec(y2_pass, SIZE, y2);
    move_vec(x2_pass, SIZE, x2);

    return y2;
};

#endif