/**
 *  (c) 2014, Manuel Vonthron - OPAL-RT Technologies, inc.
 */

#pragma once
#ifndef __ECXENOMAI_H
#define __ECXENOMAI_H

#include <stdio.h>
#include <iostream>
#include <cstdlib>
#include <signal.h>
#include <chrono>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include <thread>
#include <pthread.h>
#include <math.h>
#include <fstream>
#include <term.h>
#include <termios.h>
#include <unistd.h>

#include "ethercattype.h"
#include "nicdrv.h"
#include "ethercatbase.h"
#include "ethercatmain.h"
#include "ethercatdc.h"
#include "ethercatcoe.h"
#include "ethercatfoe.h"
#include "ethercatconfig.h"
#include "ethercatprint.h"

#include "get_home.h"
#include "task_controller.h"

#include <eigen3/Eigen/Dense>

#define EC_TIMEOUTMON 500
#define MAX_TORQUE 1000
#define PERIOD_NS 1000000
#define INITIAL_POS 0
#define SEC_IN_NSEC 1000000000
#define SHORT short
#define ELMO_NUM 15
#define USHORT unsigned short
#define LONG long
#define _2PI 6.28318530717959
#define LOG_SIZE 50000

#define NO_PRISMATIC

using namespace std;

float log_mem[LOG_SIZE][1+ELMO_NUM*5]; //for logging data (read, write elmo) (txt file)
float log_mem_ctrl[LOG_SIZE][1+ELMO_NUM*3+12]; //for logging data (data from controller) (txt file)
int log_cnt;
bool bool_log_ctrl = false;

char IOmap[4096];
pthread_t thread1;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
boolean limit_safe = 0;
uint8 currentgroup = 0;
int cnt_err = 0;

int control_mode = 0;

bool ecat_number_ok = false;
bool ecat_WKC_ok = false;
bool de_shutdown = false;
	
USHORT	elmoState[ELMO_NUM];
char	modeState[ELMO_NUM];
USHORT	servoState[ELMO_NUM];
USHORT	targetReached[ELMO_NUM];

double  oneRevolute_CNT[ELMO_NUM];	// encoder count * gear ratio
double  Amp2Torq[ELMO_NUM];			// [Amp]	-> [Torque]
double  Torq2Amp[ELMO_NUM];		// [mAmp]	<- [Torque]	
const double set_lead = 0.005; // [m/rev]
const double set_efficiency_lead = 2.4; // need to find 0.57
const int    set_direction[ELMO_NUM]		= {		 1,      1,       1,      1,       1,      1,       1,       1,       1,       1,       1,       1,       1,       1,       1 };	// direction CW or CCW
const int    set_joint_type[ELMO_NUM]       = {     0,       0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      1}; //joint type, 0: revolute. 1: prismatic
const double    max_position_limit[ELMO_NUM] = {  90.0*DEG2RAD,  90.0*DEG2RAD,  90.0*DEG2RAD,  120.0*DEG2RAD,  90.0*DEG2RAD,  45.0*DEG2RAD,  60.0*DEG2RAD,  90.0*DEG2RAD,  15.0*DEG2RAD,  90.0*DEG2RAD,  30.0*DEG2RAD,  90.0*DEG2RAD,  90.0*DEG2RAD,  60.0*DEG2RAD, 0.35}; //  maximum limit degree (rad, last is m)
const double    min_position_limit[ELMO_NUM] = { -90.0*DEG2RAD, -15.0*DEG2RAD, -90.0*DEG2RAD, -30.0*DEG2RAD,  -90.0*DEG2RAD, -90.0*DEG2RAD, -60.0*DEG2RAD, -90.0*DEG2RAD, -90.0*DEG2RAD, -90.0*DEG2RAD, -120.0*DEG2RAD, -90.0*DEG2RAD, -45.0*DEG2RAD, -60.0*DEG2RAD, -0.35}; //  minimum limit degree (rad, last is m)
const LONG   set_gearRatio[ELMO_NUM]		= {    101,     101,     101,     101,     101,     101,     101,     101,     101,     101,     101,     101,     101,     101,       3 };	// harmonic gear ratio
const LONG   set_resolution[ELMO_NUM]		= {  10000,   10000,   10000,   10000,   10000,   10000,   10000,   10000,   10000,   10000,   10000,   10000,   10000,   10000,   10000 };	// encoder pulse 2500, qep 4, total 10000/rev = 2500*4
const double set_continuosCurrent[ELMO_NUM] = { 7470.0,  7470.0,  6860.0,  6860.0,  5350.0,  5350.0,  5350.0,  7470.0,  7470.0,  6860.0,  6860.0,  5350.0,  5350.0,  5350.0, 7470.0};	// motor continuous current, unit is [mA]	
const double set_torque_const[ELMO_NUM]		= { 0.0855,  0.0855,  0.0833,  0.0833,  0.0318,  0.0318,  0.0318,  0.0855,  0.0855,  0.0833,  0.0833,  0.0318,  0.0318,  0.0318, 0.0855};	// motor torque constant, unit is [Nm/A]
const LONG   set_maxVelocity[ELMO_NUM]		= {   5363,    5363,    5363,    5363,    5498,    5498,    5498,    5498,   17700,   17700,   17700,   17700,   17700,   17700, 5363 };	// motor maximal speed, unit is [RPM]
const double continuous_stall_torque[ELMO_NUM] = { 0.429,  0.429,  0.384,  0.384,  0.109,  0.109,  0.109,  0.429,  0.429,  0.384,  0.384,  0.109,  0.109,  0.109,  0.429 }; // RBE_01810 -> 0.429, RBE_01511 -> 0.384, RBE_00711 -> 0.109 [N/m]
const double mech_max_speed[ELMO_NUM]       = { 14000,  14000,  16500,  16500,  20000,  20000,  20000,  14000,  14000,  16500,  16500,  20000,  20000,  20000, 14000 }; // RBE_01810 -> 14000, RBE_01511 -> 16500, RBE_00711 -> 20000 [RPM]
const double touch_probe_position_rad[ELMO_NUM] = {-15.0*DEG2RAD, 45.0*DEG2RAD, -15.0*DEG2RAD, 30.0*DEG2RAD, -15.0*DEG2RAD, 0.0*DEG2RAD, 0.0*DEG2RAD, 15.0*DEG2RAD, -45.0*DEG2RAD, 15.0*DEG2RAD, -30.0*DEG2RAD, 15.0*DEG2RAD, 0.0, 0.0, 0.0}; //touch sensor position (distance from home position) rad & m

double  RadToPosition(double rad) {return (double)rad / _2PI * set_lead;}
double	CntToDeg(double  cnt, USHORT ielmo)		{return (double)cnt * 360.0 / oneRevolute_CNT[ielmo];}	// [Count]  -> [degree]
double	CntToRad(LONG  cnt, USHORT ielmo)		{return (double)cnt * _2PI / oneRevolute_CNT[ielmo];}
double	AmpToNm(double ampare, USHORT ielmo)	{return (ampare * Amp2Torq[ielmo]);}			// [Amp]	-> [Torque]
double	NmToAmp(double torque, USHORT ielmo)	{return (torque * Torq2Amp[ielmo]);}
//double  TorqToNewton(double torque, USHORT ielmo) {return (_2PI * 0.8 *(torque - ((9.81*0.5*25.62*0.008*set_gearRatio[ielmo])/_2PI*0.8)))/(0.008*set_gearRatio[ielmo]);} // [Torque] -> [Force]
double  NmToN(double torque, USHORT ielmo)      {return (_2PI * set_efficiency_lead *torque /(set_lead*set_gearRatio[ielmo]));} // [Torque] -> [Force]
double  NToNm(double force, USHORT ielmo)       {return ((force*set_lead*set_gearRatio[ielmo])/(_2PI * set_efficiency_lead));} // [Torque] -> [Force]
double  CntPSecToRadPsec(double CntPsec, USHORT ielmo) {return (double)CntPsec * _2PI / oneRevolute_CNT[ielmo];} // [Cnt/s] -> [Rad/s]
double  RadPsecToMPsec(double RadPsec)          {return (double)RadPsec / _2PI * set_lead;} // [Rad/s] -> [m/s]
double  mPsecToRadPsec(double mPsec)            {return (double)_2PI*mPsec/set_lead;} // [m/s] -> [Rad/s]
double  RadPsecToCntPsec(double RadPsec, USHORT ielmo) {return (double)oneRevolute_CNT[ielmo]*RadPsec/_2PI;}; //[Rad/s] -> [Cnt/s]

#pragma pack(push,1)
// rx, tx setting
namespace EtherCAT_Elmo
{
    enum MODE_OF_OPERATION
    {
        ProfilePositionmode = 1,
        ProfileVelocitymode = 3,
        ProfileTorquemode = 4,
        Homingmode = 6,
        InterpolatedPositionmode = 7,
        CyclicSynchronousPositionmode = 8,
        CyclicSynchronousVelocitymode = 9,
        CyclicSynchronousTorquemode = 10,
        CyclicSy = 11
    };

    struct PDO_SET
    {
        struct tx_SET
        {
            int32_t targetPosition;
            int32_t targetVelocity;
            int16_t targetTorque;
            uint16_t maxTorque;
            uint16_t controlWord;
            int8_t modeOfOperation;
        };
        struct rx_SET
        {
            int32_t positionActualValue;
            uint32_t homingSensor;
            uint16_t statusWord;            
            int32_t velocityActualValue;
            int16_t torqueActualValue;
            int32_t positionExternal;        
        };
    };
} // namespace EtherCAT_Elmo
#pragma pack(pop)

EtherCAT_Elmo::PDO_SET::tx_SET *txPDO[ELMO_NUM];
EtherCAT_Elmo::PDO_SET::rx_SET *rxPDO[ELMO_NUM];

int ElmoSafteyMode[ELMO_NUM];
int stateElmo[ELMO_NUM];
int ElmoMode[ELMO_NUM];
int touchState[ELMO_NUM]; //home sensor

int32_t positionElmo_cnt = 0;
double positionElmo_rad = 0.0;
double positionElmo_m = 0.0;
double positionElmo[ELMO_NUM];

int32_t velocityElmo_cntPsec = 0;
double velocityElmo_radPsec = 0.0;
double velocityElmo_mPsec = 0.0;
double velocityElmo[ELMO_NUM];

int16_t torqueElmo_percentage = 0;
double torqueElmo_current = 0.0;
double torqueElmo_Nm = 0.0;
double torqueElmo_N = 0.0;
double torqueElmo[ELMO_NUM];

double max_torque_range = 0.0;
double min_torque_range = 0.0;
double max_velocity_range = 0.0;
double min_velocity_range = 0.0;
double max_position_range = 0.0;
double min_position_range = 0.0;

double time_for_controller = 0.0;
double time_period_nsec = PERIOD_NS;
double sec_in_nsec = SEC_IN_NSEC;
double time_period_sec = time_period_nsec/sec_in_nsec;

double positionOffset_rad[ELMO_NUM];
double positionHome_rad[ELMO_NUM];

double positionDesired_rad = 0.0;
double positionDesired_m = 0.0;
double positionDesired[ELMO_NUM];
int32_t positionDesired_cnt[ELMO_NUM];
double velocityDesired_radPsec = 0.0;
double velocityDesired_mPsec = 0.0;
double velocityDesired[ELMO_NUM];
int32_t velocityDesired_cntPsec[ELMO_NUM];
double torqueDesired_Nm = 0.0;
double torqueDesired_N = 0.0;
double torqueDesired_current = 0.0;
double torqueDesired[ELMO_NUM];
int16_t torqueDesired_percentage[ELMO_NUM];

bool bool_ethecat_loop = true;

int joint_num = ELMO_NUM;
CHoming HomingControl(joint_num, set_joint_type);
CMoveHome MoveHomeControl(joint_num, touch_probe_position_rad);
CTaskController TaskControl(joint_num, time_period_sec, touch_probe_position_rad);

const int HOMING_START_BIT = 4;
const int FAULT_BIT = 3;
const int OPERATION_ENABLE_BIT = 2;
const int SWITCHED_ON_BIT = 1;
const int READY_TO_SWITCH_ON_BIT = 0;
enum
{
    CW_SHUTDOWN = 6,
    CW_SWITCHON = 7,
    CW_ENABLEOP = 15,
    CW_DISABLEOP = 7,
};

 enum
    {
        EM_POSITION = 11,
        EM_TORQUE = 22,
        EM_DEFAULT = 33,
        EM_COMMUTATION = 44,
    };


bool controlWordGenerate(const uint16_t statusWord, uint16_t &controlWord)
{
    if (!(statusWord & (1 << OPERATION_ENABLE_BIT))) //4
    {
        if (!(statusWord & (1 << SWITCHED_ON_BIT))) //2
        {
            if (!(statusWord & (1 << READY_TO_SWITCH_ON_BIT))) //1
            {
                if (statusWord & (1 << FAULT_BIT)) //8
                {
                    controlWord = 0x80;
                    cnt_err++;                       
                    return false;
                }
                else
                {
                    controlWord = CW_SHUTDOWN;
                    cnt_err++;
                    return false;                    
                }
            }
            else
            {
                controlWord = CW_SWITCHON;               
                return true;                
            }
        }
        else
        {
            controlWord = CW_ENABLEOP;                      
            return true;            
        }
    }
    else
    {
        controlWord = CW_ENABLEOP;             
        return true;        
    }    
    controlWord = 0;
    cnt_err++;
    return false;    
}

int getch(void)  
{  
  int ch;  
  struct termios buf;  
  struct termios save;  
  
   tcgetattr(0, &save);  
   buf = save;  
   buf.c_lflag &= ~(ICANON|ECHO);  
   buf.c_cc[VMIN] = 1;  
   buf.c_cc[VTIME] = 0;  
   tcsetattr(0, TCSAFLUSH, &buf);  
   ch = getchar();  
   tcsetattr(0, TCSAFLUSH, &save);  
   return ch;  
}  

void log_file() // text file open & write.
{
    cout << "--------- Logging Start ---------"<<endl;

    cout << "Logging <state_log.txt>"<<endl;
    ofstream fout;
    cout << "Log size: [" << log_cnt <<"]["<< ELMO_NUM*4 + 1 <<"]" <<endl;
    fout.open("/home/kist/catkin_ws/log/state_log.txt");
    fout << "time\t";
    fout <<"q0\tq1\tq2\tq3\tq4\tq5\tq6\tq7\tq8\tq9\tq10\tq11\tq12\tq13\tq14\t";
    fout <<"qdot0\tqdot1\tqdot2\tqdot3\tqdot4\tqdot5\tqdot6\tqdot7\tqdot8\tqdot9\tqdot10\tqdot11\tqdot12\tqdot13\tqdot14\t";
    fout <<"tau0\ttau1\ttau2\ttau3\ttau4\ttau5\ttau6\ttau7\ttau8\ttau9\ttau10\ttau11\ttau12\ttau13\ttau14\t";
    fout <<"touch0\ttouch1\ttouch2\ttouch3\ttouch4\ttouch5\ttouch6\ttouch7\ttouch8\ttouch9\ttouch10\ttouch11\ttouch12\ttouch13\ttouch14\t";    
    fout <<"ref0\tref1\tref2\tref3\tref4\tref5\tref6\tref7\tref8\tref9\tref10\tref11\tref12\tref13\tref14\t\n";

    for(int i=0; i<=log_cnt; i++)
    {        
        for(int j=0; j<ELMO_NUM*5 + 1; j++)
        {
            fout<<log_mem[i][j]<<"\t";
        }
        fout << "\t\n";
    }
    fout.close();
    cout << "<state_log.txt> Logging Complete."<<endl;

    if(bool_log_ctrl == true)
    {
        cout << "Logging <ctrl_log.txt>"<<endl;
        ofstream fout2;
        cout << "Log size: [" << log_cnt <<"]["<< ELMO_NUM + 1 <<"]" <<endl;
        fout2.open("/home/kist/catkin_ws/log/ctrl_log.txt");
        fout2 << "time\t";
        fout2 <<"q0\tq1\tq2\tq3\tq4\tq5\tq6\tq7\tq8\tq9\tq10\tq11\tq12\tq13\tq14\t";
        fout2 <<"qdot0\tqdot1\tqdot2\tqdot3\tqdot4\tqdot5\tqdot6\tqdot7\tqdot8\tqdot9\tqdot10\tqdot11\tqdot12\tqdot13\tqdot14\t";
        fout2 <<"qdes0\tqdes1\tqdes2\tqdes3\tqdes4\tqdes5\tqdes6\tqdes7\tqdes8\tqdes9\tqdes10\tqdes11\tqdes12\tqdes13\tqdes14\t";
        fout2 <<"lhand_x\tlhand_y\tlhand_z\trhand_x\trhand_y\trhand_z\t";
        fout2 <<"lhand_x_des\tlhand_y_des\tlhand_z_des\trhand_x_des\trhand_y_des\trhand_z_des\t\n";

        for(int i=0; i<=log_cnt; i++)
        {        
            for(int j=0; j<ELMO_NUM*3 + 1 + 12; j++)
            {
                fout2<<log_mem_ctrl[i][j]<<"\t";
            }
            fout2 << "\t\n";
        }
        fout2.close();
        cout << "<ctrl_log.txt> Logging Complete."<<endl;
    }

    cout << "--------- Logging Complete ---------"<<endl<<endl;
}


void signal_callback_handler(int signum) {
   cout << endl << "Caught Ctrl+C " << signum << endl;
   
  std::cout << "Terminating EtherCat!!"<< std::endl;
  bool_ethecat_loop = false;
  de_shutdown = true;
  
  log_file();
     // Terminate program
   exit(signum);
}

void ethercat_run(char *ifname, char *mode)
{    
    cout << endl << "------------- Dual Arm Control Start -------------"<<endl;
    if(strcmp(mode,"home")==0)
    {        
        cout << "Control Mode: Home Poistion Detection" << endl<<endl;
        control_mode = 1; //velocity mode
    }
    else if(strcmp(mode,"moveup")==0)
    {
        cout << "Control Mode: Move Linear Motor Up" << endl<<endl;
        control_mode = 1; //velocity mode
    }
    else if(strcmp(mode,"checkhome")==0)
    {
        cout << "Control Mode: Move to Touch Sensor" << endl<<endl;
        control_mode = 1; //position mode
    }
    else if(strcmp(mode,"movehome")==0)
    {
        cout << "Control Mode: Move to Home Position" << endl<<endl;
        control_mode = 1; //position mode
    }
    else if(strcmp(mode,"task")==0)
    {
        cout << "Control Mode: Task Space Control with Joint Torque Control" << endl<<endl;
        control_mode = 0; //torque mode
        bool_log_ctrl = true;
    }
    else if(strcmp(mode,"none")==0)
    {        
        cout << "Control Mode: Not determined!!" << endl << " Torque mode with Zero torque activated!!"<< endl<<endl;
        control_mode = 0; //torque mode
    }    
    else
    {
        cout << "Control Mode: Wrong command!!" << endl << " Torque mode with Zero torque activated!!"<< endl<<endl;
        control_mode = 0; //torque mode
    }

    if(strcmp(mode,"home")==0)
    {
    }
    else
    {
        ifstream fin;
        fin.open("/home/kist/catkin_ws/log/position_offset.txt"); 
        if (!fin)
        {
            cout << "Warning! Cannot find <position_offset.txt>" <<endl<<endl;
            exit(100);
        }
        else
        {
            cout << "Position offset:"<< endl;
            for(int i=0; i<ELMO_NUM; i++)
            {
                if (fin.eof()) // if it is end of file
                {
                    cout << "Warning! Error in <position_offset.txt>" <<endl<<endl;
                    exit(100);                    
                }

                fin >> positionOffset_rad[i];                                
                cout << positionOffset_rad[i] << " ";                
            }
            cout <<endl;            

            TaskControl.get_joint_position_offset(positionOffset_rad);
        }
    }

    std::cout << "------------- ethercatThread Start -------------" << std::endl;
    char IOmap[4096];
    bool reachedInitial[ELMO_NUM] = {false};
    bool exit_middle = false;

    if (ec_init(ifname))
    {
        printf("ELMO : ec_init on %s succeeded.\n", ifname);
        // find and auto-config slaves
        // network discovery
        if (ec_config_init(FALSE) > 0) // TRUE when using configtable to init slaves, FALSE otherwise
        {
            printf("ELMO : %d slaves found and configured.\n", ec_slavecount); // ec_slavecount -> slave num
            if (ec_slavecount == ELMO_NUM)
            {
                ecat_number_ok = true;
            }
            else
            {
                std::cout << "WARNING : SLAVE NUMBER INSUFFICIENT" << std::endl;
            }

            // CompleteAccess disabled for Elmo driver
            for (int slave = 1; slave <= ec_slavecount; slave++)
            {
                //printf("ELMO : Has Slave[%d] CA? %s\n", slave, ec_slave[slave].CoEdetails & ECT_COEDET_SDOCA ? "true" : "false");
                if (!(ec_slave[slave].CoEdetails & ECT_COEDET_SDOCA))
                {
                    printf("ELMO : slave[%d] CA? : false , shutdown request \n ", slave);
                    exit_middle = true;
                }
                ec_slave[slave].CoEdetails ^= ECT_COEDET_SDOCA;
            }

            if (!exit_middle)
            {                
                ec_statecheck(0, EC_STATE_PRE_OP,  EC_TIMEOUTSTATE);
                for (int slave = 1; slave <= ec_slavecount; slave++)
                {
                    //0x1605 :  Target Position             32bit
                    //          Target Velocity             32bit
                    //          Target Torque               16bit
                    //          Max Torque                  16bit
                    //          Control word                16bit
                    //          Modes of Operation          16bit                       
                    uint16 map_1c12[2] = {0x0001, 0x1605};

                    //0x1a00 :  position actual value       32bit
                    //          Digital Inputs              32bit
                    //          Status word                 16bit
                    //0x1a11 :  velocity actual value       32bit
                    //0x1a13 :  Torque actual value         16bit
                    uint16 map_1c13[4] = {0x0003, 0x1a00, 0x1a11, 0x1a13};
                    
                    int os;
                    os = sizeof(map_1c12);
                    int retVal;
                    retVal = ec_SDOwrite(slave, 0x1c12, 0, TRUE, os, &map_1c12, EC_TIMEOUTRXM);
                    os = sizeof(map_1c13);
                    retVal = ec_SDOwrite(slave, 0x1c13, 0, TRUE, os, &map_1c13, EC_TIMEOUTRXM);
                    
                }

                // if CA disable => automapping works
                ec_config_map(&IOmap);

                // wait for all slaves to reach SAFE_OP state
                printf("ELMO : EC WAITING STATE TO SAFE_OP\n");
                ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

                expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;

                printf("ELMO : Request operational state for all slaves. Calculated workcounter : %d\n", expectedWKC);
                if (expectedWKC != 3 * ELMO_NUM)
                {
                    std::cout << "WARNING : Calculated Workcounter insufficient!" << std::endl;
                    ecat_WKC_ok = true;
                }
                /// going operational
                ec_slave[0].state = EC_STATE_OPERATIONAL;
                 //ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE * 4);

                // send one valid process data to make outputs in slaves happy
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);

                // request OP state for all slaves 
                if (ec_slave[0].state == EC_STATE_OPERATIONAL)
                {
                    if (ecat_number_ok && ecat_WKC_ok)
                    {
                        std::cout << "All slaves Status OK" << std::endl;
                    }
                    else
                    {
                        std::cout << "Please Check Slave status" << std::endl;
                    }
                    std::cout << "STARTING IN 3 ... " << std::endl;
                    printf("ELMO : Operational state reached for all slaves! Starting in ... 3... ");
                    fflush(stdout);
                    usleep(1000000);//std::this_thread::sleep_for(std::chrono::seconds(1));
                    printf("2... ");
                    fflush(stdout);
                    usleep(1000000);//std::this_thread::sleep_for(std::chrono::seconds(1));
                    printf("1... ");
                    fflush(stdout);
                    usleep(1000000);//std::this_thread::sleep_for(std::chrono::seconds(1));
                    printf("0... Start! \n");

                    inOP = TRUE;

                    // cyclic loop
                    for (int slave = 1; slave <= ec_slavecount; slave++)
                    {
                        txPDO[slave - 1] = (EtherCAT_Elmo::PDO_SET::tx_SET *)(ec_slave[slave].outputs);
                        rxPDO[slave - 1] = (EtherCAT_Elmo::PDO_SET::rx_SET *)(ec_slave[slave].inputs);
                    }
                    for (int i=0; i<ELMO_NUM; i++)
                    {
                        txPDO[i]->targetVelocity = (int16_t) (0); 
                        txPDO[i]->targetTorque = (int16_t) (0); 
                    }

                    positionElmo_cnt = 0;
                    positionElmo_rad = 0.0;
                    positionElmo_m = 0.0;
                    velocityElmo_cntPsec = 0;
                    velocityElmo_radPsec = 0.0;
                    velocityElmo_mPsec = 0.0;
                    torqueElmo_percentage = 0;
                    torqueElmo_current = 0.0;
                    torqueElmo_Nm = 0.0;
                    torqueElmo_N = 0.0;
                    
                    struct timespec ts, ts1;
                    
                    // ts.tv_nsec = time at here
                    for (int i = 0; i < ec_slavecount; i++)
                    {
                        ElmoSafteyMode[i] = 0;
                    }
                    clock_gettime(CLOCK_MONOTONIC, &ts);
                    ts.tv_nsec += PERIOD_NS;
                    while (ts.tv_nsec >= SEC_IN_NSEC)
                    {
                        ts.tv_sec++;
                        ts.tv_nsec -= SEC_IN_NSEC;
                    }

                    time_for_controller = 0.0;
                    log_cnt = 0;

                    while(bool_ethecat_loop)
                    {
                        //Commutation Checking
                        std::cout << "ELMO : Initialization Mode" << std::endl;
                                                
                        while (!de_shutdown)
                        {
                            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
                            ts.tv_nsec += PERIOD_NS;

                            time_for_controller += (double) PERIOD_NS/SEC_IN_NSEC;                                

                            clock_gettime(CLOCK_MONOTONIC, &ts1);
                            
                            while (ts.tv_nsec >= SEC_IN_NSEC)
                            {
                                ts.tv_sec++;
                                ts.tv_nsec -= SEC_IN_NSEC;
                            }
                            ec_send_processdata();
                            wkc = ec_receive_processdata(0);               

                            if (wkc >= expectedWKC)
                            {
                                for (int slave = 1; slave <= ec_slavecount; slave++)
                                {
                                    if (controlWordGenerate(rxPDO[slave - 1]->statusWord, txPDO[slave - 1]->controlWord))
                                    {
                                        reachedInitial[slave - 1] = true;
                                    }
                                }

                                for(int i=0; i<ELMO_NUM; i++)
                                {
                                    oneRevolute_CNT[i] = set_direction[i] * set_gearRatio[i] * set_resolution[i];	// one revolution of joint module, ex) -1[dirction] * 100[gear ratio] * 10000[pulse] = 100000                                                                        
                                    Amp2Torq[i] = set_torque_const[i] * (set_direction[i] * set_gearRatio[i]);			// [A] to [Nm] @ gear outer Torque
                                    Torq2Amp[i] = 1.0/(set_torque_const[i] * (set_direction[i] * set_gearRatio[i]));// [Nm] to [A] @ gear inner Ampere
                                }

                                for (int i = 1; i <= ec_slavecount; i++)
                                {
                                    if (reachedInitial[i - 1])
                                    {  
                                        ////////////////////////////////////////////////////////
                                        ///////////////////  set control mode  /////////////////
                                        ////////////////////////////////////////////////////////

                                        //set control mode                      
                                        if(control_mode == 0)
                                        {
                                            txPDO[i-1]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousTorquemode;
                                        }
                                        else if(control_mode == 1)
                                        {
                                            txPDO[i-1]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousVelocitymode;

                                            #ifdef NO_PRISMATIC
                                            if(set_joint_type[i-1] == 1)
                                            {
                                                txPDO[i-1]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousTorquemode; //set torquemode for prismatic joint
                                            }
                                            #endif
                                        }
                                        else if(control_mode == 2)
                                        {
                                            txPDO[i-1]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousPositionmode;
                                            
                                            #ifdef NO_PRISMATIC
                                            if(set_joint_type[i-1] == 1)
                                            {
                                                txPDO[i-1]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousTorquemode; //set torquemode for prismatic joint
                                            }
                                            #endif
                                        }
                                        else
                                        {
                                            txPDO[i-1]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousTorquemode;                                                                                        
                                        }
                                    }
                                }

                                for (int i = 1; i <= ec_slavecount; i++)
                                {
                                    //initialize
                                    positionElmo[i-1] = 0.0;
                                    velocityElmo[i-1] = 0.0;
                                    torqueElmo[i-1] = 0.0;

                                    if (reachedInitial[i - 1])
                                    {   
                                        ////////////////////////////////////////////////////////
                                        ///////////////////  read from ELMO  ///////////////////
                                        ////////////////////////////////////////////////////////
                                                                                
                                        //current percentage (torque)                                        
                                        torqueElmo_percentage=
                                            ((((int16_t)ec_slave[i].inputs[14]) +
                                            ((int16_t)ec_slave[i].inputs[15] << 8))); //read percentage: continous torque

                                        //encoder 
                                        positionElmo_cnt = rxPDO[i - 1]-> positionActualValue;

                                        stateElmo[i - 1] =
                                            (((uint16_t)ec_slave[i].inputs[8]) +
                                            ((uint16_t)ec_slave[i].inputs[9] << 8));

                                        //motor velocity
                                        velocityElmo_cntPsec =
                                            (((int32_t)ec_slave[i].inputs[10]) +
                                            ((int32_t)ec_slave[i].inputs[11] << 8) +
                                            ((int32_t)ec_slave[i].inputs[12] << 16) +
                                            ((int32_t)ec_slave[i].inputs[13] << 24));

                                        //touchState[i-1] = rxPDO[i-1]->homingSensor;  
                                        touchState[i-1] = (((int32_t)ec_slave[i].inputs[4]));
                                        // +                                                ((int32_t)ec_slave[i].inputs[5] << 8));// +
                                            //((int32_t)ec_slave[i].inputs[6] << 16) +
                                            //((int32_t)ec_slave[i].inputs[7] << 24));                                                                                                                                                         
                                        ////////////////////////////////////////////////////////////////////
                                        ///////////////////  modify read data for Controller  //////////////
                                        ////////////////////////////////////////////////////////////////////
                            
                                        positionElmo_rad = CntToRad(positionElmo_cnt, i-1);
                                        velocityElmo_radPsec = CntPSecToRadPsec(velocityElmo_cntPsec,i-1);
                                        torqueElmo_current = torqueElmo_percentage/1000.0 * set_continuosCurrent[i-1]/1000.0; //calculate current
                                        torqueElmo_Nm = AmpToNm(torqueElmo_current, i-1); //calculate torque  (Nm)                         

                                        if(set_joint_type[i-1] == 1) // for prismatic joint 
                                        {                                               
                                            positionElmo_m = RadToPosition(positionElmo_rad);
                                            velocityElmo_mPsec = RadPsecToMPsec(velocityElmo_radPsec);
                                            torqueElmo_N = NmToN(torqueElmo_Nm, i-1);
                                            positionElmo[i-1] = positionElmo_m;
                                            velocityElmo[i-1] = velocityElmo_mPsec;
                                            torqueElmo[i-1] = torqueElmo_N;
                                        }
                                        else //for revolute joint
                                        {                                            
                                            positionElmo[i-1] = positionElmo_rad;
                                            velocityElmo[i-1] = velocityElmo_radPsec;
                                            torqueElmo[i-1] = torqueElmo_Nm;
                                        }

                                        ////////////////////////////////////////////////////////////////////
                                        ///////////////////  check safety limits  //////////////////////////
                                        ////////////////////////////////////////////////////////////////////

                                        //check limit for safety
                                        max_torque_range = 2.0*continuous_stall_torque[i-1] * set_gearRatio[i-1];
                                        min_torque_range = -2.0*continuous_stall_torque[i-1] * set_gearRatio[i-1];                                        
                                        max_velocity_range = (_2PI * mech_max_speed[i-1] / 60.0) / set_gearRatio[i-1];
                                        min_velocity_range = - (_2PI * mech_max_speed[i-1] / 60.0) / set_gearRatio[i-1];                                        
                                        max_position_range = max_position_limit[i-1];
                                        min_position_range = min_position_limit[i-1];


                                        if(set_joint_type[i-1] == 1) //for prismatic joint
                                        {
                                            max_torque_range = NmToN(2.0*continuous_stall_torque[i-1] * set_gearRatio[i-1], i-1);
                                            min_torque_range = NmToN(-2.0*continuous_stall_torque[i-1] * set_gearRatio[i-1], i-1);                                       
                                            max_velocity_range = RadPsecToMPsec((_2PI * mech_max_speed[i-1] / 60.0) / set_gearRatio[i-1]);
                                            min_velocity_range = RadPsecToMPsec(- (_2PI * mech_max_speed[i-1] / 60.0) / set_gearRatio[i-1]);
                                        }

                                        if (torqueElmo[i-1] < min_torque_range || torqueElmo[i-1] > max_torque_range || limit_safe == true) //torque limit , Nm
                                        {
                                            // if(limit_safe == false)                                                
                                            // {
                                            //     cout << endl << endl << "Warning: Joint Torque limit! Check joint " << i-1 <<"." <<endl;
                                            //     cout << "torque: " << torqueElmo[i-1] << ", min torque: " << min_torque_range << ", max torque: " << max_torque_range << endl << endl;                                                    
                                            // }
                                            // limit_safe = true;
                                        }
                                        else if (positionElmo[i-1] < min_position_range || positionElmo[i-1] > max_position_range || limit_safe == true) //position limit, deg
                                        {                                                
                                            if(limit_safe == false)                                                
                                            {
                                                cout << endl << endl << "Warning: Joint Position limit!! Check joint " << i-1 <<"."  <<endl;
                                                cout << "position: " << positionElmo[i-1] << ", min position: " << min_position_range << ", max position: " << max_position_range << endl << endl;
                                            }
                                            limit_safe = true;
                                        }
                                        else if(velocityElmo[i-1]  < min_velocity_range || velocityElmo[i-1] > max_velocity_range || limit_safe == true) //velocity limit, rad/sec
                                        {    
                                            if(limit_safe == false)                                                
                                            {                                            
                                                cout << endl << endl << "Warning: Joint Velocity limit!! Check joint " << i-1 <<"."  <<endl;
                                                cout << "velocity: " << velocityElmo[i-1] << ", min velocity: " << min_velocity_range << ", max velocity: " << max_velocity_range << endl << endl;
                                            }
                                            limit_safe = true;
                                        }
                                    } 
                                }

                                ///////////////////////////////////////////////////////////////
                                //////////////////// High Level Controller  ///////////////////
                                ///////////////////////////////////////////////////////////////
                                for (int i = 1; i <= ec_slavecount; i++) //initialize
                                {
                                    positionDesired[i-1] = 0.0;
                                    velocityDesired[i-1] = 0.0;
                                    torqueDesired[i-1] = 0.0;
                                }

                                if(strcmp(mode,"home")==0)
                                {
                                    if(time_for_controller >= 1.0) //wait 1.0sec before start
                                    {
                                        HomingControl.read(time_for_controller, positionElmo, touchState);
                                        //HomingControl.read(time_for_controller, positionElmo_rad, velocityElmo_radPsec, torqueElmo_Nm, touchState);
                                        HomingControl.homing_velocity_control();
                                        HomingControl.write(velocityDesired, positionOffset_rad);
                                    }
                                    else
                                    {
                                        for (int i = 1; i <= ec_slavecount; i++) //initialize
                                        {
                                            velocityDesired[i-1] = 0.0;
                                        }
                                    }
                                }
                                else if(strcmp(mode,"moveup")==0)
                                {
                                    for (int i = 1; i <= ec_slavecount; i++) //initialize
                                    {
                                        velocityDesired[i-1] = 0.0;
                                    }

                                    if(time_for_controller >= 5.0 && time_for_controller <= 15.0)
                                    {
                                        velocityDesired[14] = 0.03;
                                    }
                                }
                                else if(strcmp(mode,"checkhome")==0 || strcmp(mode,"movehome")==0)
                                {
                                    if(time_for_controller >= 5.0) //wait 5.0sec before start
                                    {
                                        MoveHomeControl.read(time_for_controller, positionElmo, positionOffset_rad, touchState);

                                        if(strcmp(mode,"checkhome")==0)
                                        {
                                            MoveHomeControl.move_touchsensor_position();
                                        }
                                        else if(strcmp(mode,"movehome")==0)
                                        {
                                            MoveHomeControl.move_home_position();
                                        }

                                        MoveHomeControl.write(velocityDesired);
                                    }
                                    else
                                    {
                                        for (int i = 1; i <= ec_slavecount; i++) //initialize
                                        {
                                            velocityDesired[i-1] = 0.0;
                                        }
                                    }
                                }
                                else if(strcmp(mode,"task")==0)
                                {
                                    if(time_for_controller >= 1.0) //wait 1sec before start
                                    {
                                        TaskControl.read(time_for_controller, positionElmo, velocityElmo, torqueElmo);
                                        TaskControl.compute();
                                        TaskControl.write(torqueDesired);
                                    }
                                    else
                                    {
                                        for (int i = 1; i <= ec_slavecount; i++) //initialize
                                        {
                                            torqueDesired[i-1] = 0.0;
                                        }
                                    }
                                }
                                else if(strcmp(mode,"none")==0)
                                {
                                    for (int i = 1; i <= ec_slavecount; i++) 
                                        {
                                            torqueDesired[i-1] = 0.0;
                                        }
                                }
                                else
                                {
                                    
                                }

                                for (int i = 1; i <= ec_slavecount; i++)
                                {
                                    if (reachedInitial[i - 1])
                                    {   
                                        /////////////////////////////////////////////////////////
                                        ///////////////////  modify data for ELMO  //////////////
                                        /////////////////////////////////////////////////////////

                                        if(set_joint_type[i-1] == 1) // for prismatic joint 
                                        { 
                                            //positionDesired_rad = ;                                      
                                            velocityDesired_radPsec = mPsecToRadPsec(velocityDesired[i-1]);
                                            torqueDesired_Nm = NToNm(torqueDesired[i-1], i-1);                                            
                                        }
                                        else
                                        {
                                            velocityDesired_radPsec = velocityDesired[i-1];
                                            torqueDesired_Nm = torqueDesired[i-1];
                                        }

                                        //positionDesired_cnt = ;
                                        
                                        velocityDesired_cntPsec[i-1] = (int32_t) (RadPsecToCntPsec(velocityDesired_radPsec, i-1));                                        
                                        torqueDesired_current = NmToAmp(torqueDesired_Nm, i-1);
                                        torqueDesired_percentage[i-1] = (int16_t) (torqueDesired_current*1000.0*1000.0/set_continuosCurrent[i-1]);//torqueElmo_percentage/1000.0 * set_continuosCurrent[i-1]/1000.0;
                                        

                                        ////////////////////////////////////////////////////////
                                        ////////////////////  write to ELMO  ///////////////////
                                        ////////////////////////////////////////////////////////                      

                                        if(control_mode == 0)
                                        {
                                            //target torque (when mode of operation is CyclicSynchronousTorquemode)
                                            txPDO[i-1]->targetTorque = (int16_t) torqueDesired_percentage[i-1];
                                        }
                                        else if(control_mode == 1)
                                        {
                                            //target velocity (when mode of operation is CyclicSynchronousPositionmode)
                                            txPDO[i-1]->targetVelocity = (int32_t) velocityDesired_cntPsec[i-1];
                                            //cout << i << ": " << txPDO[i-1]->targetVelocity << endl;

                                            #ifdef NO_PRISMATIC
                                            if(set_joint_type[i-1] == 1)
                                            {
                                                txPDO[i-1]->targetTorque = (int16_t) (0); 
                                            }
                                            #endif                                                                                      
                                        }
                                        else if(control_mode == 2)
                                        {
                                            //target position (when mode of operation is CyclicSynchronousVelocitymode)
                                            txPDO[i-1]->targetPosition = (int32_t) positionDesired_cnt[i-1];

                                            #ifdef NO_PRISMATIC
                                            if(set_joint_type[i-1] == 1)
                                            {
                                                txPDO[i-1]->targetTorque = (int16_t) (0); 
                                            }
                                            #endif
                                        }                                        

                                        txPDO[i - 1]->maxTorque = (uint16)MAX_TORQUE;
                                        

                                        if(limit_safe == true)                   
                                        {
                                            if(control_mode == 0) //torque mode
                                            {
                                                txPDO[i-1]->targetTorque = (int16_t) (0); 
                                            }
                                            else if(control_mode == 1) //velocity mode
                                            {                                           
                                                txPDO[i-1]->targetVelocity = (int32) (0);      
                                            }
                                            else if(control_mode == 2) //position mode, not working
                                            {
                                                txPDO[i-1]->targetPosition = positionElmo_cnt;
                                            }      
                                            else
                                            {
                                                txPDO[i-1]->targetTorque = (int16_t) (0);
                                            }                                     
                                        }
                                    }
                                }
  
                                log_mem[log_cnt][0] = time_for_controller;
                                for(int i=0; i<ELMO_NUM; i++)
                                {
                                    log_mem[log_cnt][1+i] = positionElmo[i];
                                    log_mem[log_cnt][1+15+i] = velocityElmo[i];
                                    log_mem[log_cnt][1+30+i] = torqueElmo[i];
                                    log_mem[log_cnt][1+45+i] = touchState[i];       
                                    if(control_mode == 0) //torque mode
                                    {
                                        log_mem[log_cnt][1+60+i] = torqueDesired[i]; 
                                    }
                                    else if(control_mode == 1) //velocity mode
                                    {                                           
                                        log_mem[log_cnt][1+60+i] = velocityDesired[i]; 
                                    }
                                    else if(control_mode == 2) //position mode, not working
                                    {
                                        log_mem[log_cnt][1+60+i] = positionDesired[i]; 
                                    }      
                                    else
                                    {
                                        log_mem[log_cnt][1+60+i] = torqueDesired[i]; 
                                    }                               
                                }

                                if(strcmp(mode,"task")==0)
                                {
                                    log_mem_ctrl[log_cnt][0] = time_for_controller;
                                    for(int i=0; i<ELMO_NUM; i++)
                                    {
                                        log_mem_ctrl[log_cnt][1+i] = TaskControl._q(i);
                                        log_mem_ctrl[log_cnt][1+ELMO_NUM+i] = TaskControl._qdot_lp(i);
                                        log_mem_ctrl[log_cnt][1+ELMO_NUM*2+i] = TaskControl._q_des(i);
                                    }
                                    for(int i=0; i<3; i++)
                                    {
                                        log_mem_ctrl[log_cnt][1+ELMO_NUM*3+i] = TaskControl._x_left_hand(i);
                                        log_mem_ctrl[log_cnt][1+ELMO_NUM*3+3+i] = TaskControl._x_right_hand(i);
                                        log_mem_ctrl[log_cnt][1+ELMO_NUM*3+6+i] = TaskControl._x_des_left_hand(i);
                                        log_mem_ctrl[log_cnt][1+ELMO_NUM*3+9+i] = TaskControl._x_des_right_hand(i);
                                    }
                                }

                                log_cnt = log_cnt + 1;
                                if(log_cnt >= LOG_SIZE)
                                {
                                    log_cnt = 0;
                                }
                            }
                            else
                            {                                
                                break;
                            }                         
                        }
                    }
                }
            }
        }
    }
}



void *ecatcheck( void *ptr )
{
    int slave;

    while(1)
    {
        if( inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            if (needlf)
            {
               needlf = FALSE;
               printf("\n");
            }
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++)
            {
               if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
               {
                  ec_group[currentgroup].docheckstate = TRUE;
                  if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                  {
                     printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                     ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                     ec_writestate(slave);
                  }
                  else if(ec_slave[slave].state == EC_STATE_SAFE_OP)
                  {
                     printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                     ec_slave[slave].state = EC_STATE_OPERATIONAL;
                     ec_writestate(slave);
                  }
                  else if(ec_slave[slave].state > 0)
                  {
                     if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d reconfigured\n",slave);
                     }
                  }
                  else if(!ec_slave[slave].islost)
                  {
                     /* re-check state */
                     ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                     if (!ec_slave[slave].state)
                     {
                        ec_slave[slave].islost = TRUE;
                        printf("ERROR : slave %d lost\n",slave);
                     }
                  }
               }
               if (ec_slave[slave].islost)
               {
                  if(!ec_slave[slave].state)
                  {
                     if (ec_recover_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d recovered\n",slave);
                     }
                  }
                  else
                  {
                     ec_slave[slave].islost = FALSE;
                     printf("MESSAGE : slave %d found\n",slave);
                  }
               }
            }
            if(!ec_group[currentgroup].docheckstate)
               printf(".");
        }
        usleep(250);
    }
}


#endif