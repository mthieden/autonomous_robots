#define _GNU_SOURCE
#include <stdio.h>
#include <time.h>
#include "motioncontroller.h"
#include "missions.h"

#define ROBOTPORT  8000     // on the simulation
//#define ROBOTPORT  24902  // on the robot
#define DEBUG 0


int main(int argc, char **argv)
{
    int running,arg,run_time=0;
    int mission_sq=0;
    // calibrate variables
    FILE * fp;
    char * line = NULL;
    size_t len = 0;
    ssize_t read;
    char * path;
    // logging variables
    char log_file_path[100];
    time_t now = time(NULL);
    struct tm *t = localtime(&now);

    /* Establish connection to robot sensors and actuators.
    */

    printf("**********************************************************************\n");
    printf("*********                  Start of program                  *********\n");
    printf("**********************************************************************\n\n");

    printf("- Attempting to connect robot ...\n");
    if (rhdConnect('w',"localhost",ROBOTPORT)!='w')
    {
        printf("Can't connect to rhd \n");
        exit(EXIT_FAILURE);
    }

    printf("Sucess! connected to robot \n");
    printf("- Attempting to get input get SymbolTable...\n");
    if ((inputtable=getSymbolTable('r'))== NULL)
    {
        printf("Can't connect to rhd \n");
        exit(EXIT_FAILURE);
    }
    printf("Success! connected to input SymbolTable \n");
    printf("- Attempting to get output get SymbolTable...\n");
    if ((outputtable=getSymbolTable('w'))== NULL)
    {
        printf("Can't connect to rhd \n");
        exit(EXIT_FAILURE);
    }
    printf("Success! connected to output SymbolTable \n\n");

    if(argc == 2)
    {
        printf("*******************************************\n");
        printf("**       Reading calibration data        **\n");
        printf("*******************************************\n\n");
        printf("- Reading laser calbration data \n\n");

        int r = asprintf(&path,"calib/smr%s_demo_ls_calib.dat",argv[1]);
        printf("%s:\n", path);
        fp = fopen(path, "r");
        if (fp != NULL || r !=-1)
        {
            while ((read = getline(&line, &len, fp)) != -1) {
                int index = line[18] - '0' -1 ;
                if(line[12]=='w')
                {
                    laser_calib_white[index] = strtod(line, NULL);
                }
                else if(line[12]=='b')
                {
                    laser_calib_black[index] = strtod(line, NULL);
                }
            }
            printf("Success! laser calbration data read \n\n");
            fclose(fp);
        }
        else
        {
            printf("Failed! could not read laser data!\n\n");
        }

        printf("- Reading odemetri calbration data \n\n");
        r = asprintf(&path,"calib/smr%s_demo_odo_calib.dat",argv[1]);
        printf("%s:\n", path);
        fp = fopen(path, "r");
        if (fp != NULL || r !=-1)
        {
            read = getline(&line, &len, fp);
            printf("%s\n",line);
            double l = strtof(line, NULL);
            double l1 = strtof(line, NULL);
            double l2 = strtof(line, NULL);
            printf("lol %f %f %f\n",l,l1,l2);
            printf("Success! odemetri calbration data read \n\n");
            fclose(fp);
        }
        else
        {
            printf("Failed! could not read odemetri data!\n\n");
        }

    }
    else
    {
        printf("*******************************************\n");
        printf("**       Creating calibration data\n\n");

        for(int i = 0; i < 8; i++)
        {
            laser_calib_black[i] = 0;
            laser_calib_white[i] = 128;
        }
    }
    printf("Current calibration data:\n");
    printf("black : ");
    for(int i = 0 ; i<8; i++)
    {
        printf("%f ",laser_calib_black[i]);

    }
    printf("\n");
    printf("white : ");
    for(int i = 0 ; i<8; i++)
    {
        printf("%f ",laser_calib_white[i]);

    }
    printf("\n");



    // set up logging
    strftime(log_file_path, sizeof(log_file_path)-1, "log/%Y-%m-%d_", t);
    int path_found =1;
    char temp[100];
    char *dest;
    while(path_found)
    {
        strcpy(temp, log_file_path);
        asprintf(&dest,"%05d.dat",path_found);

        strcat(temp, dest);
        if((fp = fopen(temp,"r"))!=NULL)
        {
            fclose(fp);
            path_found++;
        }
        else
        {
            strcpy(log_file_path, temp);
            path_found = 0;
        }
    }
    printf("Logfile name: %s\n", log_file_path);

    fp = fopen(log_file_path, "w");
    if (fp != NULL )
    {
            fprintf(fp ,"%14s %14s %14s %14s %14s %14s %14s %14s %14s %14s %14s %14s %14s %14s %14s %14s %14s %14s\n",
                    "time", "x", "y", "theta", "goal_theta", "motorspeed_l", "motorspeed_r", "speedcmd",
                    "mission_state", "motiontype", "linesensor0", "linesensor1", "linesensor2", "linesensor3",
                    "linesensor4", "linesensor5", "linesensor6", "linesensor7" );
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
    mot.w=odo.w;
    odo.cr=DELTA_M;
    odo.cl=odo.cr;
    odo.left_enc=lenc->data[0];
    odo.right_enc=renc->data[0];
    reset_odo(&odo);
    printf("position: %f, %f\n", odo.left_pos, odo.right_pos);
    running=1;
    mission.state=ms_init;
    mission.oldstate=-1;

    printf("Starting program\n");
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

        /***********************************
         *  mission statemachine
         */
        sm_update(&mission);
        if(!mission_sq)
            mission_sq = mission_square();
        else
            running =0;
        /*  end of mission  */

        mot.left_pos=odo.left_pos;
        mot.right_pos=odo.right_pos;
        update_motcon(&mot);

        fprintf(fp ,"%14d %14f %14f %14f %14f %14f %14f %14f %14d %14d %14f %14f %14f %14f %14f %14f %14f %14f\n",
                mission.time, odo.x, odo.y, odo.theta, mot.GoalTheta, mot.motorspeed_l, mot.motorspeed_r, mot.speedcmd,
                mission.state, mot.curcmd, LS_calib[0], LS_calib[1], LS_calib[2], LS_calib[3], LS_calib[4], LS_calib[5],
                LS_calib[6], LS_calib[7] );
        if (DEBUG)
        {
            printf("time %05d, x : %f, y : %f, theta : %f, "\
                   "goal_theta : %f, motorspeed_l : %f, motorspeed_r : %f"\
                   " speedcmd: %f, mission stat : %d  motiontype :%d\n",
                    mission.time, odo.x, odo.y, odo.theta, mot.GoalTheta,
                    mot.motorspeed_l, mot.motorspeed_r, mot.speedcmd,
                    mission.state, mot.curcmd);
            printf (  "Line sensor: ");
            for(int i=0; i<8; i++){
                printf ( "%f ",LS_calib[i]);
            };
            printf ( "\n");
        }

        speedl->data[0]=100*mot.motorspeed_l;
        speedl->updated=1;
        speedr->data[0]=100*mot.motorspeed_r;
        speedr->updated=1;
        if (run_time  % 100 ==0)
            //    printf(" laser %f \n",laserpar[3]);
            run_time++;
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
    fclose(fp);
    rhdSync();
    rhdDisconnect();
    exit(0);
}
