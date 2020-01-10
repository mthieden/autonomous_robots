/*
 * An example SMR program.
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>

#include <sys/ioctl.h>
#include "rhd.h"
#include "componentserver.h"
#include "xmlio.h"

/*****************************************
 * odometry
 */
#define WHEEL_DIAMETER   0.06522	/* m */
#define WHEEL_SEPARATION 0.26	/* m */
#define DELTA_M (M_PI * WHEEL_DIAMETER / 2000)
#define ROBOTPORT	8000 //24902
#define SAMPLERATE	100 




struct
{
    double x, y, z, omega, phi, kappa, code,id,crc;
} gmk;


typedef struct
{
    int time;
    double left, right;
} logtype;

/*typedef struct
{
    int time;
    double x, y, theta;
} logodotype;*/

typedef struct
{
    int state,oldstate;
    int time;
}smtype;

typedef struct{ //input signals
		int left_enc,right_enc; // encoderticks
		// parameters
		double w;	// wheel separation
		double cr,cl;   // meters per encodertick
	        //output signals
		double right_pos,left_pos;
		// internal variables
		int left_enc_old, right_enc_old;
		int time;
		double x, y, theta;
		} odotype;

/*
 * Motion control struct
 */

typedef struct
{//input
    int cmd;
    int curcmd;
    double speedcmd;
    double dist;
    double angle;
    double GoalTheta;
    double left_pos,right_pos;
    // parameters
    double w;
    //output
    double motorspeed_l,motorspeed_r;
    int finished;
    // internal variables
    double startpos;
    double K;
    double domega;
    double dV;
}motiontype;

// Motion types
enum {mot_stop=1,mot_move,mot_turn};
// Movement types
enum {ms_init,ms_fwd,ms_turn,ms_end};


// Global varaibles
struct xml_in *xmldata;
struct xml_in *xmllaser;
componentservertype lmssrv,camsrv;
double visionpar[10];
double laserpar[10];

// SMR input/output data
symTableElement *  inputtable,*outputtable;
symTableElement *lenc,*renc,*linesensor,*irsensor, *speedl,*speedr,*resetmotorr,*resetmotorl;

odotype odo;
smtype mission;
motiontype mot;

/*
logtype log_main[10000];
int log_counter;
logodotype log_odo[10000];
int log_odo_counter;
*/

// Prototypes
void serverconnect(componentservertype *s);
void xml_proc(struct xml_in *x);
void xml_proca(struct xml_in *x);

void reset_odo(odotype *p);
void update_odo(odotype *p);

void update_motcon(motiontype *p);
int fwd(double dist, double speed,int time);
int turn(double angle, double speed,int time);

void sm_update(smtype *p);

symTableElement* getinputref(const char *sym_name, symTableElement * tab)
{
    int i;
    for (i=0; i< getSymbolTableSize('r'); i++)
    {
        if (strcmp (tab[i].name,sym_name) == 0)
            return &tab[i];
    }
    return 0;
}
symTableElement* getoutputref (const char *sym_name, symTableElement * tab)
{
    int i;
    for (i=0; i< getSymbolTableSize('w'); i++)
    {
        if (strcmp (tab[i].name,sym_name) == 0)
            return &tab[i];
    }
    return 0;
}

double* line(void)
{
   double linearray[8];
   for(int i=0; i<8; i++){
   linearray[i]=linesensor->data[i];
   linearray[i]=round(linearray[i]*0.0078);
   }
 return linearray;
}
double* linesensindex(line_cali)
{
   for(int i=0; i<8;i++){
   if(line_cali[i] == 0.0){
       return i+1;   
       }
   }
   return void;
}

int main()
{
    
    int running,n=0,arg,time=0;
    double dist=0,angle=0;
/*
    log_counter = 0;
    log_odo_counter = 1;
    log_odo[0].time = 0;
    log_odo[0].x = 0;
    log_odo[0].y = 2;
    log_odo[0].theta = 0;
*/

    /* Establish connection to robot sensors and actuators.
    */
    if (rhdConnect('w',"localhost",ROBOTPORT)!='w')
    {
        printf("Can't connect to rhd \n");
        exit(EXIT_FAILURE);
    }

    printf("connected to robot \n");
    if ((inputtable=getSymbolTable('r'))== NULL)
    {
        printf("Can't connect to rhd \n");
        exit(EXIT_FAILURE);
    }
    if ((outputtable=getSymbolTable('w'))== NULL)
    {
        printf("Can't connect to rhd \n");
        exit(EXIT_FAILURE);
    }
    // connect to robot I/O variables
    lenc=getinputref("encl",inputtable);
    renc=getinputref("encr",inputtable);
    linesensor=getinputref("linesensor",inputtable);
    irsensor=getinputref("irsensor",inputtable);

    speedl=getoutputref("speedl",outputtable);
    speedr=getoutputref("speedr",outputtable);
    resetmotorr=getoutputref("resetmotorr",outputtable);
    resetmotorl=getoutputref("resetmotorl",outputtable);
    // **************************************************
    //  Camera server code initialization
    //
    
    /* Create endpoint */
    lmssrv.port=24919;
    strcpy(lmssrv.host,"127.0.0.1");
    strcpy(lmssrv.name,"laserserver");
    lmssrv.status=1;
    camsrv.port=24920;
    strcpy(camsrv.host,"127.0.0.1");
    camsrv.config=1;
    strcpy(camsrv.name,"cameraserver");
    camsrv.status=1;

    if (camsrv.config)
    {
        int errno = 0;
        camsrv.sockfd = socket(AF_INET, SOCK_STREAM, 0);
        if ( camsrv.sockfd < 0 )
        {
            perror(strerror(errno));
            fprintf(stderr," Can not make  socket\n");
            exit(errno);
        }

        serverconnect(&camsrv);

        xmldata=xml_in_init(4096,32);
        printf(" camera server xml initialized \n");

    }




    // **************************************************
    //  LMS server code initialization
    //

    /* Create endpoint */
    lmssrv.config=1;
    if (lmssrv.config)
    {
        char buf[256];
        int errno = 0,len;
        lmssrv.sockfd = socket(AF_INET, SOCK_STREAM, 0);
        if ( lmssrv.sockfd < 0 )
        {
            perror(strerror(errno));
            fprintf(stderr," Can not make  socket\n");
            exit(errno);
        }

        serverconnect(&lmssrv);
        if (lmssrv.connected)
        {
            xmllaser=xml_in_init(4096,32);
            printf(" laserserver xml initialized \n");
            len=sprintf(buf,"push  t=0.2 cmd='mrcobst width=0.4'\n");
            send(lmssrv.sockfd,buf,len,0);
        }

    }


    /* Read sensors and zero our position.
    */
    rhdSync();

    odo.w=0.256;
    odo.cr=DELTA_M;
    odo.cl=odo.cr;
    odo.left_enc=lenc->data[0];
    odo.right_enc=renc->data[0];
    reset_odo(&odo);
    printf("position: %f, %f\n", odo.left_pos, odo.right_pos);
    mot.w=odo.w;
    running=1;
    mission.state=ms_init;
    mission.oldstate=-1;
    

    while (running)
    {
        if (lmssrv.config && lmssrv.status && lmssrv.connected)
        {
            while ( (xml_in_fd(xmllaser,lmssrv.sockfd) >0))
                xml_proca(xmllaser);
        }

        if (camsrv.config && camsrv.status && camsrv.connected)
        {
            while ( (xml_in_fd(xmldata,camsrv.sockfd) >0))
                xml_proc(xmldata);
        }


        rhdSync();
        odo.left_enc=lenc->data[0];
        odo.right_enc=renc->data[0];
        update_odo(&odo);
        double* line_cali=line();
        int lineindex = linesensindex(line_cali);
        printf("index %d \n",lineindex);
        

        /****************************************
          / mission statemachine
          */
        sm_update(&mission);
        switch (mission.state)
        {
            case ms_init:
                n=4; dist=1;angle=90.0/180*M_PI;
                mission.state= ms_fwd;
                break;

            case ms_fwd:

                if (fwd(dist,0.3,mission.time))  mission.state=ms_turn;
                break;

            case ms_turn:
                if (turn(angle,0.3,mission.time))
                {
                    n=n-1;
                    if (n==0)
                        mission.state=ms_end;
                    else
                        mission.state=ms_fwd;
                }
                break;

            case ms_end:
                mot.cmd=mot_stop;
                running=0;
                break;
        }
        /*  end of mission  */

        mot.left_pos=odo.left_pos;
        mot.right_pos=odo.right_pos;
        update_motcon(&mot);
        speedl->data[0]=100*mot.motorspeed_l;
        speedl->updated=1;
        speedr->data[0]=100*mot.motorspeed_r;
        speedr->updated=1;
        if (time  % 100 ==0)
            //    printf(" laser %f \n",laserpar[3]);
            time++;
        /* stop if keyboard is activated
         *
         */
        ioctl(0, FIONREAD, &arg);
        if (arg!=0)  running=0;

    }/* end of main control loop */
    speedl->data[0]=0;
    speedl->updated=1;
    speedr->data[0]=0;
    speedr->updated=1;
  
/*    //write log 
    FILE *fp;

    fp = fopen("/home/smr/k385/NHR/square/logaccturn02.dat", "w");
    for(int i = 0; i < log_odo_counter; i++){
   	fprintf(fp, "%d, %f, %f, %f\n", log_odo[i].time, log_odo[i].x, log_odo[i].y, log_odo[i].theta);
    }
    fclose(fp);
*/
    rhdSync();
    rhdDisconnect();
    exit(0);
}


/*
 * Routines to convert encoder values to positions.
 * Encoder steps have to be converted to meters, and
 * roll-over has to be detected and corrected.
 */


void reset_odo(odotype * p)
{
    p->right_pos = p->left_pos = 0.0;
    p->right_enc_old = p->right_enc;
    p->left_enc_old = p->left_enc;
}

void update_odo(odotype *p)
{
    int delta;

    delta = p->right_enc - p->right_enc_old;
    if (delta > 0x8000) delta -= 0x10000;
    else if (delta < -0x8000) delta += 0x10000;
    p->right_enc_old = p->right_enc;
    p->right_pos += delta * p->cr;
    double dUR = delta * p->cr;
  
    delta = p->left_enc - p->left_enc_old;
    if (delta > 0x8000) delta -= 0x10000;
    else if (delta < -0x8000) delta += 0x10000;
    p->left_enc_old = p->left_enc;
    p->left_pos += delta * p->cl;
    double dUL = delta * p->cl;

    double U = (dUR + dUL)/2;
    odo.theta += (dUR - dUL)/WHEEL_SEPARATION;
    odo.x += (U)*cos(odo.theta);
    odo.y += (U)*sin(odo.theta);
/*
    //update log
    log_odo[log_odo_counter].time = mission.time; 
    log_odo[log_odo_counter].x = odo.x;
    log_odo[log_odo_counter].y = odo.y;
    log_odo[log_odo_counter].theta = odo.theta;
    printf("time %d, x : %f, y : %f, theta : %f, speed: %f\n", mission.time, odo.x, odo.y,odo.theta, mot.speedcmd);
    log_odo_counter++;
*/
}

void update_motcon(motiontype *p)
{
    double accel = 0.5 /SAMPLERATE; // accelaration/sampletime
    double speed=0;

    if (p->cmd !=0)
    {
        p->finished=0;
        switch (p->cmd)
        {
            case mot_stop:
                p->curcmd=mot_stop;
                break;
            case mot_move:
                p->startpos=(p->left_pos+p->right_pos)/2;
                p->curcmd=mot_move;
                break;
            case mot_turn:
                if (p->angle > 0)
                    p->startpos=p->right_pos;
                else
                    p->startpos=p->left_pos;
                p->curcmd=mot_turn;
                break;
        }
        p->cmd=0;
    }

    if(p->curcmd == mot_move)
    {
        double traveldist = (p->right_pos+p->left_pos)/2 - p->startpos;
        double acceldist=(sqrt(2 * accel*SAMPLERATE * (p->dist - traveldist)));
        double hyst = accel;
    	mot.K = 0.01;
    	mot.domega = mot.K*(mot.GoalTheta-odo.theta);
    	mot.dV = mot.domega/(odo.w/2);
/*
        printf("\n acceldist : %f",acceldist);
        printf("   accel : %f",accel);
	printf("   speed_L : %f",p->motorspeed_l);
	printf("   speed_R : %f",p->motorspeed_r);
*/
        if(acceldist > p->motorspeed_l && !(p->motorspeed_l > (p->speedcmd)))
        {
          if( p->motorspeed_l + hyst >= p->speedcmd ||p->motorspeed_l - hyst >= p->speedcmd)
            {
                speed = p->speedcmd;
            }
          else
          {
              speed = fabs(p->motorspeed_l)+accel;
          }
        }
        else
        {
            speed = fabs(p->motorspeed_l)-accel;
        }
    }

    if(p->curcmd == mot_turn)
    {
        double angle_travel;
        double angle_dist;
        if (p->angle>0)
        {
            angle_travel = p->right_pos-p->startpos ;
            angle_dist = (p->angle*p->w)/2;
        }
        else
        {
            angle_travel = p->left_pos-p->startpos ;
            angle_dist = (fabs(p->angle)*p->w)/2;
        }
        double acceldist=(sqrt(2 * ((accel*SAMPLERATE)/2 )* (angle_dist - angle_travel)));
        double hyst = accel;
        //printf("angle_dist : %f, angle_travel : %f  acceldist %f",angle_dist, angle_travel, acceldist);

        if(acceldist > fabs(p->motorspeed_l)/2 && !(fabs(p->motorspeed_l) > (p->speedcmd)/2))
        {
            speed = fabs(p->motorspeed_l)+accel;
            if(fabs(p->motorspeed_l) + hyst >= p->speedcmd /2||fabs(p->motorspeed_l) - hyst >= p->speedcmd/2)
                speed = p->speedcmd/2;
        }
        else
        {
            speed = fabs(p->motorspeed_l)-accel;
        }
    }

    switch (p->curcmd)
    {
       case mot_stop:
            p->motorspeed_l=0;
            p->motorspeed_r=0;
            break;

       case mot_move:
            if ((p->right_pos+p->left_pos)/2- p->startpos > p->dist)
            {
                p->finished=1;
                p->motorspeed_l=0;
                p->motorspeed_r=0;
            }
            else if(fabs(mot.GoalTheta-odo.theta)<(5*M_PI)/180){
                p->motorspeed_l=speed;
                p->motorspeed_r=speed;
            }
	    else if(odo.theta>mot.GoalTheta){
                p->motorspeed_r-= mot.dV;
		printf("\n motorspeed_r: %f, motorspeed_l: %f", p->motorspeed_r, p->motorspeed_l);
            }
            else if(odo.theta<mot.GoalTheta)
            {
                p->motorspeed_l-= mot.dV;
		printf("\n motorspeed_r: %f, motorspeed_l: %f", p->motorspeed_r, p->motorspeed_l);
            }
      break;

        case mot_turn:
            if (p->angle>0)
            {
                p->motorspeed_l=0;
                if (p->right_pos-p->startpos < (p->angle*p->w)/2)
                {
                    p->motorspeed_l=-speed;
                    p->motorspeed_r=speed;
                }
                else
                {
                    p->motorspeed_l=0;
                    p->motorspeed_r=0;
                    p->finished=1;
                }
            }
            else
            {
                p->motorspeed_r=0;
                if (p->left_pos-p->startpos < (fabs(p->angle)*p->w)/2)
                {
                    p->motorspeed_l=speed;
                    p->motorspeed_r=-speed;

                }
                else
                {
                    p->motorspeed_l=0;
                    p->motorspeed_r=0;
                    p->finished=1;
                }
            }

            break;
    }
/*
    double travel = (p->right_pos+p->left_pos)/2- p->startpos;
    double curr = (p->right_pos+p->left_pos);
    double dist = p->dist;
    printf("time %d, travel %f, curr : %f, dist : %f", mission.time, travel, curr, dist);
    printf(" left : %f, right : %f\n", mot.motorspeed_l, mot.motorspeed_r);
    log_main[log_counter].time = mission.time;
    log_main[log_counter].left = mot.motorspeed_l;
    log_main[log_counter].right = mot.motorspeed_r;
    log_counter++;
*/
}


int fwd(double dist, double speed,int time)
{
    if (time==0)
    {
        mot.cmd=mot_move;
        mot.speedcmd=speed;
        mot.dist=dist;
        return 0;
    }
    else
        return mot.finished;
}

int turn(double angle, double speed,int time)
{
    if (time==0)
    {
        mot.cmd=mot_turn;
        mot.speedcmd=speed;
        mot.angle=angle;
	mot.GoalTheta+=angle;
        return 0;
    }
    else
        return mot.finished;
}



void sm_update(smtype *p)
{
    if (p->state!=p->oldstate)
    {
        p->time=0;
        p->oldstate=p->state;
    }
    else
    {
        p->time++;
    }
}



