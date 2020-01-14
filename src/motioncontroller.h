/*
 * An example SMR program.
 *
 */
#ifndef MOTIONCONTROLL   /* Include guard */
#define MOTIONCONTROLL

#define _GNU_SOURCE
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
enum {mot_stop=1,mot_move,mot_turn,mot_follow_line};
// mission types
enum {ms_init,ms_fwd,ms_turn,ms_end,ms_follow};


// Global varaibles
struct xml_in *xmldata;
struct xml_in *xmllaser;
componentservertype lmssrv,camsrv;
double visionpar[10];
double laserpar[10];
double LS_calib[8];
double laser_calib_black[8];
double laser_calib_white[8];

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
void update_lin_sens(void);

void update_motcon(motiontype *p);
int fwd(double dist, double speed,int time);
int follow_line(double dist, double speed,int time);
int turn(double angle, double speed,int time);
int lin_pos(void);
int lin_pos_com(void);
void sm_update(smtype *p);

symTableElement* getinputref(const char *sym_name, symTableElement * tab);
symTableElement* getoutputref (const char *sym_name, symTableElement * tab);
#endif // MOTIONCONTROLL
