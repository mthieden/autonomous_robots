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
            case mot_follow_line:
                p->startpos=(p->left_pos+p->right_pos)/2;
                p->curcmd=mot_follow_line;
                break;
            case mot_follow_line_angle:
                p->startpos=(p->left_pos+p->right_pos)/2;
                p->curcmd=mot_follow_line_angle;
                break;
            case mot_follow_wall:
                p->startpos=(p->left_pos+p->right_pos)/2;
                p->curcmd=mot_follow_wall;
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

    if(p->curcmd == mot_move || p->curcmd == mot_follow_line|| p->curcmd==mot_follow_line_angle || p->curcmd==mot_follow_wall)
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
        else if((p->curcmd==mot_follow_line)||(p->curcmd==mot_follow_line_angle))
            //FLAG: check follow angle styrer ind selvstændigt
        {
		    update_lin_sens();
            odo.index = lin_pos_com();
            double line_com = 0;
            double line_k = 0.05;//0.076/25;


            if (odo.index == -1)
            {
                p->motorspeed_l=0;
                p->motorspeed_r=0;
                p->finished=1;
            }
            else if (odo.index >= 3.52 ||odo.index<=3.48)
            {
                 line_com = odo.index - 3.5;
            }

            mot.domega = line_com*line_com*line_com*line_k;
            mot.dV = mot.domega*(odo.w/2);
        }
        else if (p->curcmd==mot_follow_wall)
        {
            mot.K=0.05;
            mot.domega = mot.K*(laserpar[8]-mot.walldist);
            mot.dV = fabs(mot.domega*(odo.w/2));

        }
        if(acceldist > p->motorspeed_l && !(p->motorspeed_l > (p->speedcmd)))
        {
          if( p->motorspeed_l + hyst >= p->speedcmd ||p->motorspeed_l - hyst >= p->speedcmd)
            {
                speed = p->speedcmd;
            }
          else
          {
              speed = fabs((p->motorspeed_l > p->motorspeed_r) ?p->motorspeed_l +accel: p->motorspeed_r+accel);
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
       case mot_follow_wall:
            if((((p->right_pos+p->left_pos)/2- p->startpos > p->dist)||(laserpar[8]>=mot.walldist+0.2))&&laserpar[8]!=0)
            {   
                p->finished=1;
                p->motorspeed_l=0;
                p->motorspeed_r=0;
            }
            else if((laserpar[8]<=fabs(mot.walldist+0.01))&&(laserpar[8]>=fabs(mot.walldist-0.01)))
            {
                p->motorspeed_l=speed;
                p->motorspeed_r=speed;
            }
            else if(laserpar[8]<=mot.walldist-0.01)
            {
                p->motorspeed_l-=mot.dV;
            }
            else if (laserpar[8]>=mot.walldist+0.01)
            {
                p->motorspeed_r-=mot.dV;
            }
            break;
       case mot_move:
            if (((p->right_pos+p->left_pos)/2- p->startpos > p->dist)||(odo.index==-1 && p->curcmd==mot_follow_line))
            {
                p->finished=1;
                p->motorspeed_l=0;
                p->motorspeed_r=0;
            }
            else if(fabs(mot.GoalTheta-odo.theta)<(1*M_PI)/180)
            {
                p->motorspeed_l=speed;
                p->motorspeed_r=speed;
            }
            else if(odo.theta>mot.GoalTheta)
            {   
                p->motorspeed_r-= mot.dV;         
            }
            else if(odo.theta<mot.GoalTheta)
            {   
                p->motorspeed_l-= mot.dV;         
            }

       case mot_follow_line_angle:
       case mot_follow_line:
       //INVERT LINESENSOR IF ACTAUL ROBOT (mot.dV < 0 for first case)
            if (((p->right_pos+p->left_pos)/2- p->startpos > p->dist)||(odo.index==-1 && p->curcmd==mot_follow_line)||(p->curcmd==mot_follow_line_angle && fabs(p->angle-odo.theta)<=1*M_PI/180))//
            {
                p->finished=1;
                p->motorspeed_l=0;
                p->motorspeed_r=0;
            }
            else if (p->motorspeed_r<=0 && p->motorspeed_l<=0)
            {
                p->motorspeed_r=0.5*mot.speedcmd;
                p->motorspeed_l=0.5*mot.speedcmd;   
            }
            else if(mot.dV==0)
            {
                p->motorspeed_l=speed;
                p->motorspeed_r=speed;
            }
            else if(mot.dV >0 && p->motorspeed_l>=0) 
            {
                if (p->motorspeed_l>speed)
                    {p->motorspeed_l=speed;}
                else
                    p->motorspeed_l-=fabs(mot.dV);
            }
            else if(mot.dV <0 && p->motorspeed_r>=0) 
            {
                if (p->motorspeed_r>speed)
                    {p->motorspeed_r=speed;}
                else
                    p->motorspeed_r-=fabs(mot.dV);
            }
            else if (p->motorspeed_r<0)
            {
                p->motorspeed_r=0.2*speed;
                p->motorspeed_l=speed;
            }
            else if (p->motorspeed_l<0)
            {
                p->motorspeed_r=speed;
                p->motorspeed_l=0.2*speed;
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

int follow_line(double dist, double speed,int time, char colour[])
{
    char states[6][2] ={"wr", "wm", "wl", "br", "bm", "bl"};
    int statecheck=0;
    for(int i=0;i>6;i++){statecheck+=abs(strcmp(states[i],colour));};
    if(statecheck != 0)
    {
    //if(strcmp('w','w')!=0)
        printf("Please select one of the possible solutions b(lack) or w(hite) and r(ight) m(iddle) or l(eft)\n");
        return -1;
    }
    else if (time==0)
    {
        strcpy(mot.fl_colour,colour);
        mot.cmd=mot_follow_line;
        mot.speedcmd=speed;
        mot.dist=dist;
        return 0;
    }
    else
        return mot.finished;
}

int follow_line_angle(double angle, double dist, double speed,int time, char colour[])
{

    char states[6][2] ={"wr", "wm", "wl", "br", "bm", "bl"};
    int statecheck=0;
    for(int i=0;i>6;i++){statecheck+=abs(strcmp(states[i],colour));};
    if(statecheck != 0)
    {
    //if(strcmp('w','w')!=0)
        printf("Please select one of the possible solutions b(lack) or w(hite) and r(ight) m(iddle) or l(eft)\n");
        return -1;
    }
    else if (time==0)
    {

        strcpy(mot.fl_colour,colour);
        mot.cmd=mot_follow_line_angle;
        mot.speedcmd=speed;
        mot.dist=dist;
        mot.angle=angle;
        return 0;
    }
    else
        return mot.finished;
}

int follow_wall(double walldist, double dist, double speed, int time)
{
    if (time==0)
    {
        mot.cmd=mot_follow_wall;
        mot.speedcmd=speed;
        mot.dist=dist;
        mot.walldist=walldist;
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
    double scaling[8]={1,1,1,1,1,1,1,1};
    if (mot.fl_colour[0]=='b')
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
    
    if (mot.fl_colour[1]=='r')
    {
        scaling[0]=1.6;
        scaling[1]=1.4;
        scaling[2]=1.2;
    }
    
    else if (mot.fl_colour[1]=='l')
    {
        scaling[7]=1.6;
        scaling[6]=1.4;
        scaling[5]=1.2;
    }
    for (int i = 0; i < 8; ++i)
    {
        LS_calib[i]=LS_calib[i]*scaling[i];
    }



}

int line_cross(void)
{
    int line_trigger = 0;
    for(int i=0; i<8; i++)
    {
    	line_trigger+=LS_calib[i];
    }
	if(line_trigger>=6){
		return 1;  //Returns 1 if 4 or more linesensors give a HIGH signal
	}
	else{
		return 0; //Returns 0 if not
	}
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

    return index;
}
