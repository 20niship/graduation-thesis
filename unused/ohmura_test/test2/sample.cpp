#include <libhr4c_comm.h>
#include <iostream>
#include <signal.h>
#include <fstream>
#include <sys/time.h>
#include <unistd.h>

using namespace std;

bool prog_finish=false;
void sig_int(int signal){
  prog_finish=true;
}


void print_mt_status(int status){
  if ( (status & 0x20) >0)
    std::cout<<"in position ";
  if( (status & 0x10) >0)
    std::cout<<"servo on ";
  else
    std::cout<<"servo off ";

  if( (status &0x08)>0)
    std::cout<<"alarm ";

  if( (status&0x4)>0)
    std::cout<<"limit alarm ";

  if( (status&0x2)>0)
    std::cout<<"spi error ";

  if ((status&0x1)>0)
    std::cout<<"connection error ";
  std::cout<<std::endl;
}

void initialize(int inst_no, bool alarm_reset){

  int mt_status[6];
  hr4capi_get_motor_status(inst_no, mt_status);
  for(int i=0;i<6;i++){
    std::cout<<i<<":  ";
    print_mt_status(mt_status[i]);
  }


  int ctl_mode[6];
  hr4capi_get_control_mode(inst_no, ctl_mode);
  for(int i=0;i<6;i++){
    std::cout<<ctl_mode[i]<<" ";
  }
  std::cout<<std::endl;

  if (alarm_reset){
    int joint_nos[6];
    for(int i=0;i<6;i++){
      joint_nos[i]=i;
    }
    hr4capi_alarm_reset(inst_no, joint_nos, 6);
  }
}


int main() {
    auto instance_no1 = hr4capi_open(const_cast<char*>("172.16.1.21"),
                                    54321,
				     6,"green");

    signal(SIGINT,sig_int);

    if (instance_no1 >= 0) {
      bool r1_connect=false;
      int ret=hr4capi_start(instance_no1);

      if(ret>0){
	std::cout<<"robot 1 connect\n";
	r1_connect=true;

	// current mode
	int ctl_mode[6]={3,3,3,3,3,3};
	hr4capi_set_control_mode(instance_no1, ctl_mode);
	initialize(instance_no1, true);	
      }

      int cnt=0;
      double time_sum=0;
      int mask[6]={0,1,1,1,1,1};
      if(r1_connect){

	double joint_angles1[6];

	double joint_ref[6]={0,0,0,0,0,0};
	
	double cur_cmd=0;

	hr4capi_get_joint_angle(instance_no1, joint_angles1);

	struct timeval tv0;
	struct timezone tz;
	gettimeofday(&tv0,&tz);
       
	while(!prog_finish){

	  struct timeval tv1;
	  gettimeofday(&tv1, &tz);
	  double time=(tv1.tv_sec-tv0.tv_sec)*1000+ (double)(tv1.tv_usec-tv0.tv_usec)/1000.;
	  time_sum+=time;
	  cnt++;
	  tv0=tv1;
	  
	  hr4capi_get_joint_angle(instance_no1, joint_angles1);
	  
	}
      }

      std::cout<<time_sum/cnt<<std::endl;
      
      if(r1_connect)
	hr4capi_stop(instance_no1);
      
      hr4capi_close(instance_no1);

    }

    
    return 0;
}
