#include "filter.hpp"
#include <iostream>
#include <math.h>
#include <stdio.h>

using namespace std;

FILTER::FILTER() {
    fk1 = 1.0;
    fk2 = 1.0;
    FilOld = 0;
    FilOut = 0;
}

FILTER::~FILTER() {

}

void
FILTER::set_para(double Ts, double Ft) {
    fk1 = (2.0 * Ft - Ts) / (2.0 * Ft + Ts);
    fk2 = (Ts) / (2.0 * Ft + Ts);
    FilOut = 0.0;
    FilOld = 0.0;
    valInit = false;
//    setParaFlag = 1;
}


void
FILTER::setVal(double value) {
    if (valInit) {
        FilOut = FilOut * fk1 + fk2 * (FilOld + value);
        FilOld = value;
    } else {
        FilOut = FilOld = value;
        valInit = true;
    }
}


double
FILTER::get_val() {
    return FilOut;
}


/* For vector process */


void FILTER::set_enc_num(int arg_int) {
    /* Initialize vector-style enc data filter*/
    enc_number = arg_int;
    FilOldVector.resize(arg_int, 0);
    FilOutVector.resize(arg_int, 0);
}

void FILTER::setVal(vector<float> enc_vector) {
    /* Multi encoder version */
//    cout << "multi" << endl;
    for (int i=0;i<enc_number;i++){
        if (valInit) {
            FilOutVector[i] = FilOutVector[i] * fk1 + fk2 * (FilOldVector[i] + enc_vector[i]);
            FilOldVector[i] = enc_vector[i];
        } else {
            // call one time
            FilOldVector[i] = FilOutVector[i] = enc_vector[i];
            valInit = true;
        }
    }

}


vector<float> FILTER::get_val_vector() {
    return FilOutVector;
}

