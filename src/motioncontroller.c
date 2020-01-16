#include "motioncontroller.h"


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
    odo.index=3.5; 

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
            case mot_follow_line:
                p->startpos=(p->left_pos+p->right_pos)/2;
                p->curcmd=mot_follow_line;
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

    if(p->curcmd == mot_move || p->curcmd == mot_follow_line)
    {
        double traveldist = (p->right_pos+p->left_pos)/2 - p->startpos;
        double acceldist=(sqrt(2 * accel*SAMPLERATE * (p->dist - traveldist)));
        double hyst = accel;

        if(p->curcmd==mot_move)
        {
            mot.K = 0.003;
            mot.domega = mot.K*(mot.GoalTheta-odo.theta);
            mot.dV = fabs(mot.domega*(odo.w/2));
        }
        else if(p->curcmd==mot_follow_line)
        {
		      update_lin_sens();
              odo.index = lin_pos_com();
              mot.K = 3; //0.05
		//printf("\n   line index : %d", line_index);
            double line_com = 0;
            double line_k = -0.076/100;//0.01;
            if (odo.index == -1)
            {
                p->motorspeed_l=0;
                p->motorspeed_r=0;
                p->finished=1;
            }
            else if (odo.index >= 3.8 ||odo.index<=3.2)
            {
                 line_com = odo.index - 3.5;
            }


	        mot.GoalTheta -= line_k * line_com;
            mot.domega = fabs(mot.K*(mot.GoalTheta - odo.theta));

/*
            mot.domega = mot.K*(mot.GoalTheta - odo.theta - line_k * line_com);
            mot.GoalTheta -= mot.domega;
*/
            mot.dV = fabs(mot.domega*(odo.w/2));
            //printf("domega %f, dV %f  line_index : %d, line_com :%f",mot.domega, mot.dV, line_index, line_com);
        }
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

    if(p->curcmd == mot_turn )
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
       case mot_follow_line:
            if (((p->right_pos+p->left_pos)/2- p->startpos > p->dist)||(odo.index==-1 && p->curcmd==mot_follow_line))
            {
                p->finished=1;
                p->motorspeed_l=0;
                p->motorspeed_r=0;
            }
            else if(fabs(mot.GoalTheta-odo.theta)<(5*M_PI)/180)
            {
                p->motorspeed_l=speed;
                p->motorspeed_r=speed;
            }
            else if(odo.theta>mot.GoalTheta)
            {   
		if(p->curcmd==mot_follow_line)
		{
                p->motorspeed_r-= mot.dV/2;
		p->motorspeed_l+= mot.dV/2;
                //printf("\n motorspeed_r: %f, motorspeed_l: %f", p->motorspeed_r, p->motorspeed_l);
		}  
		else  p->motorspeed_r-= mot.dV;         

	    }
            else if(odo.theta<mot.GoalTheta)
            {
               	if(p->curcmd==mot_follow_line)
		{
                p->motorspeed_r+= mot.dV/2;
		p->motorspeed_l-= mot.dV/2;
                //printf("\n motorspeed_r: %f, motorspeed_l: %f", p->motorspeed_r, p->motorspeed_l);
		}  
		else  p->motorspeed_l-= mot.dV;  
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

int follow_line(double dist, double speed,int time, char colour)
{

    if(colour!='w' && colour !='b')
    //if(strcmp('w','w')!=0)
        {return -1;}
    else if (time==0)
    {
        mot.fl_colour=colour;
        mot.cmd=mot_follow_line;
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


void update_lin_sens(void)
{
    //laser_calib_black; 	//Low average
    //laser_calib_white; 	//High average
    if (mot.fl_colour=='b')
    {
        for(int i=0; i<8; i++)
        {
            LS_calib[i]=1-((linesensor->data[i]-laser_calib_black[i])/(laser_calib_white[i]-laser_calib_black[i]));        
        }
    }

    else
    {
    for(int i=0; i<8; i++)
        {
            LS_calib[i]=(linesensor->data[i]-laser_calib_black[i])/(laser_calib_white[i]-laser_calib_black[i]);
        }
    }
}

int line_cross(int time)
{
    if(!time==0){
    int line_trigger = 0;
    for(int i=0; i<8; i++)
    {
    	line_trigger+=LS_calib[i];
    }
	if(line_trigger<=2){
		return 1;  //Returns 1 if 4 or more linesensors give a HIGH signal
	}
	else{
		return 0; //Returns 0 if not
	}
    }
    return 0;
}

int lin_pos()
{
    int index=-1;
    double max=0.9;
    for (int i=0; i<8; i++)
    {
    	if(LS_calib[i]<max)
			{
   		        max=LS_calib[i];
                index=i;
   			}
    }
	return index;
}

double lin_pos_com()
{
    double index=-1;
    double sum=0;
    double weight_sum=0;
    for (int i=0; i<8; i++)
    {
        sum+=LS_calib[i];
        weight_sum+=(i+1)*(LS_calib[i]);
    }
    index = (weight_sum/sum)-1;
    //printf("\n sum: %f, weighted sum: %f, index: %f", sum, weight_sum, index);
    return index;
}

void update_ir(void){
	double ka[5] = {15.8158,15.8158,15.8158,15.8158,15.8158};
	double kb[5] = {74.6716,74.6716,74.6716,74.6716,74.6716};
	for(int i=0;i<5;i++){
		double irsensordata=irsensor->data[i];
		irdist[i]=ka[i]/(irsensordata-kb[i]);
	}
}
