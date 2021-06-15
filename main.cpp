#include <limits.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <cstring>
#include <string.h>
#include "ethercat_xenomai_dualarm.h"


int main(int argc, char *argv[])
{   
    pthread_t thread1;
    int elmo_control_mode = 0; //0 = torque mode, 1 = velocity mode, 2 = position mode    
    int iret;    
    char default_mode[10] = "none";

    // Register signal and signal handler
   signal(SIGINT, signal_callback_handler);
   
    if (argc > 1)
    {
      iret = pthread_create( &thread1, NULL, &ecatcheck, (void*) &ctime);       

      if(argc == 2) //default: zero torque mode, when control mode is not commanded
      {
        ethercat_run(argv[1], default_mode);
      }
      else if (argc > 2) //when specific control mode is commanded
      {
        ethercat_run(argv[1], argv[2]);
      }
      pthread_join(iret, NULL );
    }
    else
    {
       printf("Usage: simple_test ifname1\nifname = eth0 for example\n");
    }
     printf("End program\n");
}