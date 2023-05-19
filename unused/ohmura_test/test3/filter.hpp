#ifndef __FILTER_H__
#define __FILTER_H__

class filter{
private:
  bool valInit;
  double fk1;
  double fk2;
  double FilOld;
  double FilOut;
public:
  filter();
  ~filter();
  void set_para(double Ts, double Ft);
  void setVal(double v);
  double get_val();
};

#endif
