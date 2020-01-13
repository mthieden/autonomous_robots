#include "missions.h"

int mission_square()
{
    double dist=1;
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
            if (follow_line(5,0.1,mission.time))  {
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

