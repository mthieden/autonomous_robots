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
    int running,arg=0;
    int mission_status=0;
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

    printf("*******************************************\n");
    if(argc == 2)
    {
        printf("**       Reading calibration data        **\n");
        printf("*******************************************\n\n");
        printf("- Reading laser calbration data \n\n");

        int index;
        int r = asprintf(&path,"calib/smr%s_demo_ls_calib.dat",argv[1]);
        printf("%s:\n", path);
        fp = fopen(path, "r");
        if (fp != NULL || r !=-1)
        {
            while ((read = getline(&line, &len, fp)) != -1) {
                index = line[18] - '0' -1 ;
                if(line[12]=='w')
                {
                    laser_calib_white[index] = strtof(line, NULL);
                }
                else if(line[12]=='b')
                {
                    laser_calib_black[index] = strtof(line, NULL);
                }
            }
            printf("Success! laser calbration data read \n\n");
            fclose(fp);
        }
        else
        {
            for(int i = 0; i < 8; i++)
            {
                laser_calib_black[i] = 0;
                laser_calib_white[i] = 128;
            }
            printf("Failed! could not read laser data!\n\n");
        }

        printf("- Reading odemetri calbration data \n\n");
        r = asprintf(&path,"calib/smr%s_demo_odo_calib.dat",argv[1]);
        printf("%s:\n", path);
        fp = fopen(path, "r");
        if (fp != NULL || r !=-1)
        {
            read = getline(&line, &len, fp);
            odo.w = strtof(line, NULL);
            double l1 = strtof(line, NULL);
            double l2 = strtof(line, NULL);
            printf("lol %f %f %f\n",odo.w,l1,l2);
            printf("Success! odemetri calbration data read \n\n");
            fclose(fp);
        }
        else
        {
            printf("Failed! could not read odemetri data!\n\n");
        }

        printf("- Reading IR calbration data \n\n");
        r = asprintf(&path,"calib/smr%s_demo_ir_calib.dat",argv[1]);
        fp = fopen(path, "r");
        if (fp != NULL || r !=-1)
        {
            index = 0;
            while ((read = getline(&line, &len, fp)) != -1)
            {
                ir_calib_15[index] = strtof(line, NULL);
                ir_calib_40[index] = strtof(&line[7], NULL);
                index++;
            }
            printf("Success! IR calbration data read \n\n");
            printf("Success! IR calbration data read \n\n");
            fclose(fp);
        }
        else
        {
            for(int i = 0; i < 6; i++)
            {
                ir_calib_15[i] = 0;
                ir_calib_40[i] = 128;
            }
            printf("Failed! could not read IR data!\n\n");
        }

    }
    else
    {
        printf("**       Creating calibration data       **\n");
        printf("*******************************************\n\n");

        for(int i = 0; i < 8; i++)
        {
            laser_calib_black[i] = 0;
            laser_calib_white[i] = 128;
        }
        for(int i = 0; i < 6; i++)
        {
            ir_calib_15[i] = 0;
            ir_calib_40[i] = 128;
        }
        odo.w=0.256;
    }
    printf("**       Current calibration data:\n");
    printf("Black line values :\n");
    for(int i = 0 ; i<8; i++)
    {
        printf("%f ",laser_calib_black[i]);

    }
    printf("\n");
    printf("White line values :\n");
    for(int i = 0 ; i<8; i++)
    {
        printf("%f ",laser_calib_white[i]);

    }
    printf("\n");

    printf("IR 15 cm : \n");
    for(int i = 0 ; i<6; i++)
    {
        printf("%f ",ir_calib_15[i]);

    }
    printf("\n");
    printf("IR 40 cm : \n");
    for(int i = 0 ; i<6; i++)
    {
        printf("%f ",ir_calib_40[i]);

    }
    printf("\n");



    // set up logging
    printf("\n\n");
    printf("Creating log file \n");
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
    printf("Logfile name: %s\n\n", log_file_path);

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
            //len=sprintf(buf,"push  t=0.2 cmd='mrcobst width=0.4'\n");
            len=sprintf(buf,"scanpush cmd='zoneobst'\n");
            send(lmssrv.sockfd,buf,len,0);
        }

    }

    fp = fopen(log_file_path, "w");
    if (fp != NULL )
    {
        fprintf(fp ,"%14s %14s %14s %14s %14s %14s %14s %14s %14s %14s %14s %14s %14s %14s %14s %14s %14s %14s %14s %14s",
                "time", "x", "y", "theta", "goal_theta", "mot.dV", "motorspeed_l", "motorspeed_r", "speedcmd",
                "mission_state", "motiontype", "index", "linesensor0", "linesensor1", "linesensor2", "linesensor3",
                "linesensor4", "linesensor5", "linesensor6", "linesensor7" );
        if (lmssrv.config && lmssrv.status && lmssrv.connected)
        {
            fprintf(fp ," %14s %14s %14s %14s %14s %14s %14s %14s %14s %14s", "laserscan0", "laserscan1", "laserscan2",
                    "laserscan3", "laserscan4", "laserscan5", "laserscan6", "laserscan7", "laserscan8", "laserscan9");
        }
        fprintf(fp ,"\n");
    }


    /* Read sensors and zero our position.
    */
    rhdSync();

    //odo.w=0.256;
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
        update_lin_sens();

        /***********************************
         *  mission statemachine
         */
        sm_update(&mission);
        switch (mission_status)
        {
            case 0:
                if(mission_laser())
                {
                    mission_status++;
                    mission.state=ms_init;
                }
                break;
            case 1:
                if(mission_push_box())
                {
                    mission_status++;
                    mission.state=ms_init;
                }
                break;
            case 2:
            	if(mission_gate())
            	{
            		mission_status++;
            		mission.state=ms_init;
            	}
            case 3:
            	if(mission_big_wall())
            	{
            		mission_status++;
            		mission.state=ms_init;
            	}
            default:
                running =0;
                break;
        }
        /*  end of mission  */

        mot.left_pos=odo.left_pos;
        mot.right_pos=odo.right_pos;
        update_motcon(&mot);

        fprintf(fp ,"%14d %14f %14f %14f %14f %14f %14f %14f %14f %14d %14d %14f %14f %14f %14f %14f %14f %14f %14f %14f",
                mission.time, odo.x, odo.y, odo.theta, mot.GoalTheta, mot.dV, mot.motorspeed_l, mot.motorspeed_r, mot.speedcmd,
                mission.state, mot.curcmd, odo.index, LS_calib[0], LS_calib[1], LS_calib[2], LS_calib[3], LS_calib[4], LS_calib[5],
                LS_calib[6], LS_calib[7] );
        if (lmssrv.config && lmssrv.status && lmssrv.connected)
        {
            fprintf(fp ," %14f %14f %14f %14f %14f %14f %14f %14f %14f %14f", laserpar[0],laserpar[1],laserpar[2],
                    laserpar[3],laserpar[4],laserpar[5],laserpar[6],laserpar[7],laserpar[8],laserpar[9]);
        }
        fprintf(fp ,"\n");
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
        /*
        if (run_time  % 100 ==1)
            printf(" laser : %f %f %f %f %f %f %f %f %f %f\n",laserpar[0],laserpar[1],laserpar[2],laserpar[3],laserpar[4],laserpar[5],laserpar[6],laserpar[7],laserpar[8],laserpar[9]);
        run_time++;
        */
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
