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
    double angle = angle=5.0/180*M_PI;
    switch (mission.state)
    {

        case ms_init:
            n=1;
            mission.state= ms_follow;
            break;

        case ms_fwd:

            if (fwd(0.5,0.3,mission.time))  mission.state=ms_follow;
            break;

        case ms_follow:
            if (follow_line(5,0.3,mission.time,'b'))  {
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
            if (follow_line(5,0.1,mission.time,'b'))  {
                printf (  "follow \n ");
                mission.state=ms_end;
            }
            break;

        case ms_end:
            mot.cmd=mot_stop;
            mission.time=0;
            return 1;
    }
    return 0;
}

int mission_find_start_point()
{
    double dist=3;
    double angle=-90.0/180*M_PI;
    switch (mission.state)
    {
        case ms_init:
            n=2;
            mission.state= ms_fwd;
            break;

        case ms_fwd:
            if(n==2)
            {
                dist = 0.35;
            }
            if(n==1)
            {
                dist = 0.2;
            }

            if (fwd(dist,0.3,mission.time))  mission.state=ms_turn;
            break;

        case ms_turn:
            if (n==2)
            {
                if (turn(angle,0.3,mission.time))
                {
                    n=n-1;
                }
            }
            else
            {
                if (turn(-angle,0.3,mission.time))
                {
                    n=n-1;
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

int mission_laser()
{
	double initialx=odo.x;
	double initialtheta=odo.theta;
	double angle=initialtheta-90*M_PI/180;
   	double objectdist=0;
   	double dist = 5;

    switch (mission.state)
    {

        case ms_init:
        	n=1;
            mission.state= ms_laser;
            break;

        case ms_laser:
            if (follow_line_angle(angle,dist,0.3,mission.time,'b')){
	            for(int i=1;i<=8;i++){
					if(laserpar[i]>laserpar[i-1])
					{
						objectdist=laserpar[i];
					}
				}
				double goaldist=objectdist+(odo.x-initialx);
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
