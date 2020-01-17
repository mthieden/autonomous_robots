#include "missions.h"

int mission_square()
{
    double dist=3;
    double angle=90.0/180*M_PI;
    switch (mission.state)
    {
        case ms_init:
            n=4;
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
            return 1;
    }
    return 0;
}

int mission_follow_line()
{
    switch (mission.state)
    {

        case ms_init:
            n=1;
            mission.state= ms_follow;
            break;

        case ms_fwd:

            if (fwd(0.1,0.3,mission.time) )  mission.state=ms_follow;
            break;

        case ms_follow:
            if (follow_line(4,0.3,mission.time,"bl"))  {
                printf (  "follow \n ");
                mission.state=ms_end;
            }
            break;

        case ms_end:
            mot.cmd=mot_stop;
            return 1;
    }
    return 0;
}

int mission_follow_wall()
{
    switch (mission.state)
    {

        case ms_init:
            n=1;
            mission.state= ms_follow_wall;
            break;

        case ms_follow_wall:
            if (follow_wall(0.5,5,0.3,mission.time))
            {
            	mission.state=ms_end;
            }
            break;

        case ms_end:
            mot.cmd=mot_stop;
            return 1;
    }
    return 0;
}

int mission_fwd_turn()
{
    double dist=0.2;
    double angle = angle=5.0/180*M_PI;
    switch (mission.state)
    {

        case ms_init:
            n=2;
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
                    mission.state=ms_follow;
            }
            break;

        case ms_follow:
            if (follow_line(5,0.1,mission.time,"br"))  {
                printf (  "follow \n ");
                mission.state=ms_end;
            }
            break;

        case ms_end:
            mot.cmd=mot_stop;
            return 1;
    }
    return 0;
}

int mission_laser()
{
	double initialtheta=odo.theta;
	double angle=initialtheta-90*M_PI/180;
   	double objectdist=0;
   	double dist = 5;

    switch (mission.state)
    {

        case ms_init:
        	printf("initlized initial y");
        	initial_y=odo.y;
        	mission.state= ms_laser;
            break;

        case ms_laser:
            if (follow_line_angle(angle,dist,0.3,mission.time,"bm")){
	            for(int i=1;i<=8;i++){
					if(laserpar[i]<laserpar[i-1])
					{
						objectdist=laserpar[i]+0.235;
					}
				}
				double goaldist=objectdist+(fabs(odo.y));
				printf("\nDistance to object: %f\n",goaldist);
	            mission.state=ms_end;
            }
        	break;

        case ms_end:
            mot.cmd=mot_stop;
            return 1;
    }
    return 0;
}


int mission_push_box()
{
    double dist=3;
    double angle=90.0/180*M_PI;
    double objectdist=0;
    printf("time %d, mission state : %d, n :%d\n", mission.time, mission.state, n);
    switch (mission.state)
    {
        case ms_init:
            n=4;
            lines_crossed=0;
            on_line=0;
            mission.state= ms_fwd;
            break;

        case ms_fwd:
            if(n==4)
            {
                for(int i=1;i<=8;i++)
                {
                    if(laserpar[i]<laserpar[i-1])
                    {
                        objectdist=laserpar[i]+0.235;
                    }
                }
                printf("objedist = %f ",objectdist);
                if (fwd(dist,0.3,mission.time) ||objectdist<=0.4)
                {
                    mission.state= ms_turn;
                }
            }
            if(n==3)
            {
                if(line_cross()==1)
                {
                    if(on_line ==0)
                    {
                        lines_crossed++;
                    }
                    on_line=1;
                }
                else
                {
                    on_line=0;
                }
                printf("line_cross = %d, on_line %d ",lines_crossed, on_line);
                printf(" ls_calib = %f %f %f %f %f %f  ",LS_calib[0],LS_calib[1],LS_calib[2],LS_calib[3],LS_calib[4],LS_calib[5]);
                if (fwd(dist,0.3,mission.time) ||lines_crossed==2)
                    mission.state= ms_turn;
            }

            break;

        case ms_turn:
            if (n==4)
            {
                if (turn(angle,0.3,mission.time))
                {
                    n--;
                    mission.state=ms_fwd;
                }
            }
            else if(n==3)
            {
                if (turn(-angle,0.3,mission.time))
                {
                    n--;
                    if (n==0)
                        mission.state=ms_end;
                    else
                        mission.state=ms_fwd;
                }
            }
            break;

        case ms_end:
            mission.state= ms_init;
            return 1;
    }
    return 0;
}
int mission_funky_wall()
{
    double angle = angle=180.0/180*M_PI;
    switch (mission.state)
    {
        case ms_init:
            n=2;
            mission.state= ms_turn;
            break;

        case ms_turn:
            if (turn(-angle,0.3,mission.time))
            {
                n=n-1;
                if (n==0)
                    mission.state=ms_end;
            }
            break;

        case ms_end:
            mission.state=ms_init;
            return 1;
    }
    return 0;
}


