#ifndef __FILTER_H__
#define __FILTER_H__

#include <math.h>
#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <vector>

using namespace std;

class FILTER {
private:
    bool valInit;
    double fk1;
    double fk2;
    double FilOld;
    double FilOut;
    // add 20190726
    int enc_number;
    vector<float> FilOldVector;
    vector<float> FilOutVector;


public:
    FILTER();

    ~FILTER();

    void set_para(double Ts, double Ft);

    void set_enc_num(int arg_int);

    void setVal(double v);

    void setVal(vector<float> enc_vector);

    double get_val();

    vector<float> get_val_vector();

//    int setParaFlag=0;
};

#endif
