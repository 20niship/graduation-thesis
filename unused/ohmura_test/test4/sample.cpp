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
				     6, "green");

    auto instance_no2 = hr4capi_open(const_cast<char*>("172.16.1.22"),
				     54321,
				     6, "green");

    signal(SIGINT,sig_int);

    ofstream fout("ang.log");
    
    if (instance_no1 >= 0 && instance_no2>=0 ) {
      bool r1_connect=false;
      bool r2_connect=false;
      int ret=hr4capi_start(instance_no1);

      if(ret>0){
	std::cout<<"robot 1 connect\n";
	r1_connect=true;

	// current mode
	int ctl_mode[6]={3,3,3,3,3,3};
	hr4capi_set_control_mode(instance_no1, ctl_mode);
	initialize(instance_no1, true);
	
      }
      ret=hr4capi_start(instance_no2);
      if(ret>0){
	std::cout<<"robot 2 connect\n";
	r2_connect=true;
	// speed mode
	int ctl_mode2[6]={2,2,2,2,2,2};
	hr4capi_set_control_mode(instance_no2, ctl_mode2);
	initialize(instance_no2, true);
      }

      int cnt=0;
      double time_sum=0;
      int mask[6]={0,1,1,1,1,1};
      if(r1_connect && r2_connect){

	double joint_angles1[6];
	double joint_angles2[6];

	double joint_speed1[6];
	double joint_speed2[6];
	
	double joint_current1[6];
	double joint_current2[6];

	double joint_ref[6]={0,0,0,0,0,0};
	

	int joint_no[6];
	joint_no[0]=0;
	hr4capi_get_joint_angle(instance_no1, joint_angles1);
	hr4capi_get_joint_angle(instance_no2, joint_angles2);
	hr4capi_set_joint_reference(instance_no2,joint_angles2, mask);
	
	hr4capi_servo_on(instance_no1, joint_no,1);
	hr4capi_servo_on(instance_no2, joint_no,1);
	initialize(instance_no1, false);
	initialize(instance_no2, false);
	
	struct timeval tv0;
	struct timezone tz;
	struct timeval tv1;
	struct timeval tv2;
	gettimeofday(&tv0,&tz);
	gettimeofday(&tv1,&tz);
       
	double stime=6;
	while(!prog_finish){
	  gettimeofday(&tv1, &tz);
	  double time=(tv1.tv_sec-tv0.tv_sec)*1000+ ((double)(tv1.tv_usec-tv0.tv_usec))/1000.;
	  tv0=tv1;
	  time_sum+=time;
	  cnt++;
	  
	  hr4capi_get_joint_angle(instance_no1, joint_angles1);
	  hr4capi_get_joint_angle(instance_no2, joint_angles2);
	  
	  double ref[6]={joint_angles1[0]-joint_angles2[0], 0 , 0 , 0, 0, 0};

	  double vref[6];
	  double gain=0.3;
	  for(int i=0;i<6;i++){
	    vref[i] = ref[i]/stime*1000*gain;
	  }

	  if(vref[0] >0.04/stime*1000)
	    vref[0]=0.04/stime*1000;
	  else if(vref[0]<-0.04/stime*1000)
	    vref[0]=-0.04/stime*1000;
	  
	  hr4capi_set_joint_reference(instance_no2,vref , mask);
	  
	  gettimeofday(&tv2, &tz);
	  double ptime=(tv2.tv_sec-tv1.tv_sec)*1000+(double)(tv2.tv_usec-tv1.tv_usec)/1000.;
	  if(stime>ptime){
	    usleep(1000*(stime-ptime));
	  }

	  fout<<time<<" ";
	  fout<<ptime<<" ";
	  for (auto i = 0; i < 6; ++i) {
	    fout << ref[i] << " ";
	  }
	  for (auto i = 0; i < 6; ++i) {
	    fout << joint_angles2[i] << " ";
	  }	
	  fout<<std::endl;

	}
      }

      std::cout<<time_sum<<" "<<cnt<<" "<<time_sum/cnt<<std::endl;
      
      std::cout<<"servo off\n";
      
      hr4capi_servo_all_off(instance_no1);
      hr4capi_servo_all_off(instance_no2);
      
      if(r1_connect)
	hr4capi_stop(instance_no1);

      if(r2_connect)
	hr4capi_stop(instance_no2);

      
      hr4capi_close(instance_no1);
      hr4capi_close(instance_no2);
    }

    
    return 0;
}