#include "filter.hpp"
#include <iostream>

using namespace std;

filter::filter(){
  fk1=1.0;
  fk2=1.0;
  FilOld=0;
  FilOut=0;
}

filter::~filter(){
}

void
filter::set_para(double Ts, double Ft){
  fk1=(2.*Ft-Ts)/(2.*Ft+Ts);
  fk2=Ts/(2.*Ft+Ts);
  FilOut=0;
  FilOld=0;
  valInit=false;
}

void
filter::setVal(double v){
  if(valInit){
    FilOut=FilOut*fk1+fk2*(FilOld+v);
    FilOld=v;
  }
  else{
    FilOut=FilOld=v;
    valInit=true;
  }
}

double
filter::get_val(){
  return FilOut;
}