#include "motioncontroller.h"
#include "missions.h"

int main(int argc, char **argv)
{
    int running,arg,time=0;
    int mission_sq=0;
    int debug = 1;
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
    if(argc == 2)
    {
        FILE * fp;
        char * line = NULL;
        size_t len = 0;
        ssize_t read;
        char * path;
        int r = asprintf(&path,"calib/smr%s_demo_ls_calib.dat",argv[1]);

        printf("%s:\n", path);
        fp = fopen(path, "r");
        if (fp != NULL || r !=-1)
        {
            while ((read = getline(&line, &len, fp)) != -1) {
                printf("Retrieved line of length %zu:\n", read);
                printf("%s", line);
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

            fclose(fp);

            printf("calibration data read:\n");
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
        }

    }
    if(laser_calib_black==NULL || laser_calib_white==NULL )
    {
        for(int i = 0; i < 8; i++)
        {
            laser_calib_black[i] = 1;
            laser_calib_white[i] = 1;
        }

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

        /****************************************
          / mission statemachine
          */
        sm_update(&mission);
        if(!mission_sq)
            mission_sq = mission_follow_line();
        else
            running =0;
        /*  end of mission  */

        mot.left_pos=odo.left_pos;
        mot.right_pos=odo.right_pos;
        update_motcon(&mot);

        if (debug)
        {
            printf("time %05d, x : %f, y : %f, theta : %f, goal_theta : %f, motorspeed_l : %f, motorspeed_r : %f speedcmd: %f, mission stat : %d  motiontype :%d\n", mission.time, odo.x, odo.y, odo.theta, mot.GoalTheta,mot.motorspeed_l, mot.motorspeed_r, mot.speedcmd, mission.state, mot.curcmd);
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
