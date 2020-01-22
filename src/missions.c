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
        	//printf("initlized initial y");
        	initial_y=odo.y;
        	printf ("Linecross %d \n", line_cross());//mission.state= ms_laser;
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
                if (bck(dist,0.3,mission.time) ||objectdist<=0.4)
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
int mission_gate()
{
// We need a piece of codes that says if laser so and so then stop
//Mission is initialized when it hits a line_cross()
// 1. The SMR will have to follow the line until the laser senses a wall (Trigger laser dist<20 cm)
// 2. It will continue to follow the line until the laser measurement varies (Trigger laser distance>25 cm)
// 3. It will the drive fwd 5 cm, turn 90 deg, drive forward 0.5 meter, turn 90 degree, forward until linecross, turn 90 deg 
// 4. Drive 1 meter forward 

	switch (mission.state)
	{
		case ms_init:
			n=4;
			mission.state = ms_find_wall;
			break;

		case ms_find_wall:
			printf("gate");
			if (follow_line(5,0.1,mission.time,"bm") || (laserpar[0]<0.75 && laserpar[0]>0.50))
			{
			printf("laser1 %f \n",laserpar[0]);
			mission.state=ms_follow_wall;
			}
			break;
		
	         case ms_follow_wall:	
			if (follow_line(5,0.1,mission.time,"bm") || laserpar[0]>0.80)
			{
			printf("laser2 %f \n",laserpar[0]);
			mission.state=ms_fwd;
			}
			break;

		case ms_fwd:
			if(n==4){
				if(fwd(0.4,0.2,mission.time))
				{
				printf (  "fwd1 \n ");
				mission.state=ms_turn;
				}
				break;     	
			}
			else if(n==3){
				if(fwd(3,0.2,mission.time) || laserpar[4]<0.3)
				{

				mission.state=ms_turn;
				}
				break; 

			}
			else if(n==2){
				if(fwd(3,0.2,mission.time)|| line_cross()==1)
				{
				printf("end");
				mission.state=ms_end;
				}
			break;  

			}
		case ms_turn:
			if(n==4){
				if(turn(90.0/180*M_PI,0.1,mission.time)) {
				mission.state=ms_fwd;
				n--;
				}
 				break;
			}
			else if(n==3){
				if(turn(-89.0/180*M_PI,0.1,mission.time)) {
				mission.state=ms_fwd;
				n--;
				}
 				break;
			}

    		case ms_end:
            		mot.cmd=mot_stop;
            		return 1;
	}
	return 0;
}

int mission_big_wall()
{   
    switch (mission.state)
    {
        case ms_init:
            n=1;
            mission.state=ms_fwd;
            break;

        case ms_turn:
            if(n==4||n==8||n==10)
            {
                if(turn(-90*M_PI/180,0.3,mission.time))
                {
                    printf("\nDriving forward, missionstate:, %d\n",n);
                    n++;
                    mission.state=ms_fwd;
                }

            }
            else if(n==2||n==14)
            {
                if(turn(-90*M_PI/180,0.3,mission.time))
                {
                    printf("\nFollowing line, missionstate: %d\n",n);
                    n++;
                    mission.state=ms_follow;
                }
            }
            break;

        case ms_fwd:
            if (n==5||n==11)
            {
                if(fwd(0.5,0.3,mission.time))
                {
                    printf("\nFollowing wall, missionstate: %d\n",n);
                    n++;
                    mission.state=ms_follow_wall;
                }
            }
            else if (n==1)
            {
                if(fwd(0.2,0.3,mission.time))
                {
                    printf("\nTurning, missionstate: %d\n",n);
                    n++;
                    mission.state=ms_turn;
                }
            }
            else if (n==7)
            {
                if(fwd(0.5,0.3,mission.time))
                {
                    printf("\nTurning, missionstate: %d\n",n);
                    n++;
                    mission.state=ms_turn;
                }
            }
            else if (n==9)
            {
                if(fwd(0.75,0.3,mission.time))
                {
                    printf("\nTurning, missionstate: %d\n",n);
                    n++;
                    mission.state=ms_turn;
                }
            }
            else if (n==13)
            {
                if(fwd(0.4,0.3,mission.time)||line_cross())
                {
                    printf("\nTurning, missionstate: %d, line_cross: %d\n",n,line_cross());
                    n++;
                    mission.state=ms_turn;
                }
            }
            break;

        case ms_follow:
            if(n==3)
            {
                if(follow_line(3,0.3,mission.time,"bm") || line_cross())
                {
                    printf("\nTurning, missionstate: %d\n",n);
                    n++;
                    mission.state=ms_turn;
                }
            }
            else
            {
                if(follow_line(3,0.3,mission.time,"bm") || line_cross())
                {
                    printf("\nFinished run\n");
                    n++;
                    mission.state=ms_end;
                }
            }
            break;

        case ms_follow_wall:
            if (follow_wall(0.35,5,0.3,mission.time))
            {
                printf("\nDriving forward, missionstate:, %d\n",n);
                n++;
                mission.state=ms_fwd;
            }
            break;

        case ms_end:
            mission.state=ms_init;
            return 1;
    }
    return 0;
}
int mission_garage()
{
// We need a piece of codes that says if laser so and so then stop
//Mission is initialized when it hits a line_cross()
// 1. The SMR will have to follow the line until the laser senses a wall (Trigger laser dist<20 cm)
// 2. It will continue to follow the line until the laser measurement varies (Trigger laser distance>25 cm)
// 3. It will the drive fwd 5 cm, turn 90 deg, drive forward 0.5 meter, turn 90 degree, forward until linecross, turn 90 deg 
// 4. Drive 1 meter forward 

	switch (mission.state)
	{
		case ms_init:
			n=10;
			mission.state = ms_turn;
			break;

		case ms_turn:
			if(n==10){		
				if (turn(-90.0/180*M_PI,0.1,mission.time))
				{
				printf (  "turn1 \n ");
				mission.state=ms_find_garage;
				n--;
				}
				break;
			}
			else if(n==9){
				if (turn(90.0/180*M_PI,0.1,mission.time))
				{
				printf (  "turn2 \n ");
				mission.state=ms_follow_wall;
				n--;
				}
				break;
			}	
			else if(n==8){
				if (turn(-90.0/180*M_PI,0.1,mission.time))
				{
				printf (  "turn3 \n ");
				mission.state=ms_fwd;
				}
				break;
			}	
			else if(n==7){
				if (turn(-90.0/180*M_PI,0.1,mission.time))
				{
				printf (  "turn4 \n ");
				mission.state=ms_fwd;
				}
				break;
			}
			else if(n==6){
				if (turn(-90.0/180*M_PI,0.1,mission.time))
				{
				printf (  "turn5 \n ");
				mission.state=ms_fwd;
				}
				break;
			}
			else if(n==5){
				if (turn(90.0/180*M_PI,0.1,mission.time))
				{
				printf (  "turn6 \n ");
				mission.state=ms_fwd;
				}
				break;
			}
			else if(n==4){
				if (turn(90.0/180*M_PI,0.1,mission.time))
				{
				printf (  "turn7 \n ");
				mission.state=ms_fwd;
				}
				break;
			}	


		case ms_fwd:
			if(n==8){		
				if (fwd(0.5,0.2,mission.time))
				{
				printf (  "fwd1 \n ");
				mission.state=ms_turn;
				n--;
				}
				break;
			}
			else if(n==7){
				if (fwd(0.5,0.2,mission.time))
				{
				printf (  "fwd2 \n ");
				mission.state=ms_turn;
				n--;
				}
				break;
			}
			else if(n==6){
				if (fwd(0.4,0.2,mission.time))
				{
				printf (  "fwd3 \n ");
				mission.state=ms_turn;
				n--;
				}
				break;
			}
			else if(n==5){
				if (fwd(0.4,0.2,mission.time))
				{
				printf (  "fwd4 \n ");
				mission.state=ms_turn;
				n--;
				}
				break;
			}
			else if(n==4){
				if (fwd(0.5,0.2,mission.time))
				{
				printf (  "fwd5 \n ");
				mission.state=ms_end;
				}
				break;
			}

		case ms_find_garage:
			if (follow_line(5,0.1,mission.time,"bm") || line_cross()==1)
			{
			printf("wall found %f \n",laserpar[0]);
			mission.state=ms_turn;
			}
			break;

		case ms_follow_wall:
			if(fwd(1,0.1,mission.time) || (laserpar[8]>5 && laserpar[8]<6))
			{
			printf("wall end %f \n",laserpar[0]);
			mission.state=ms_turn;
			}
			break;
    		case ms_end:
            		mot.cmd=mot_stop;
            		return 1;
	}
	return 0;
}
int mission5_wl()
{
    switch (mission.state)
    {

        case ms_init:
            n=1;
            mission.state=ms_turn;
            break;

        case ms_turn:
            if (turn(90, 0.2,mission.time))
            {
                mission.state=ms_fwd;
            }
            break;

        case ms_fwd:
            if (fwd(0.2,0.3,mission.time))
            {
                mission.state=ms_follow_wl;
            }
            break;

        case ms_follow_wl:
            if (follow_line(3, 0.2,mission.time, "wm" )||line_cross())
            {
                mission.state=ms_fwd_line_cross;
            }
            break;

        case ms_fwd_line_cross:
            mot.fl_colour[1]='b';
            if (fwd(0.5,0.3,mission.time)||line_cross())
            {
                mission.state=ms_end;
            }
            break;

        case ms_end:
            mot.cmd=mot_stop;
            mission.state=ms_init;
            return 1;
    }
    return 0;
}
