#include "formation_control.h"
#include<vector>
using namespace std;

formation_control::formation_control()
{

}
vector<float> formation_control::artifical_potential_rep_field(vector<vector<float> > environment,vector<float> self_position,bool is_boundary)
{
    if(is_boundary)
    {
        if(environment.empty()==0)
        {
            vector<float> combine={0,0};
            return combine;
        }
        if(environment.size()==1)
        {
            vector<float> buttom={self_position[0],environment[0][1]};
            vector<float> left={environment[0][0],self_position[1]};
            vector<float> buttom_force=potential_field_two_point(self_position,buttom,300);
            vector<float> left_force=potential_field_two_point(self_position,left,300);
            vector<float> combine={buttom_force[0]+left_force[1],buttom_force[1]+left_force[1]};
            return combine;
        }
        else if(environment.size()==2)
        {
            vector<float> buttom={self_position[0],environment[0][1]};
            vector<float> left={environment[0][0],self_position[1]};
            vector<float> right={environment[1][0],self_position[1]};
            vector<float> buttom_force=potential_field_two_point(self_position,buttom,300);
            vector<float> left_force=potential_field_two_point(self_position,left,300);
            vector<float> right_force=potential_field_two_point(self_position,right,300);
            vector<float> combine={buttom_force[0]+left_force[0]+right_force[0],buttom_force[1]+left_force[1]+right_force[1]};
            return combine;
        }
        else if(environment.size()>=3)
        {
            vector<float> buttom={self_position[0],environment[0][1]};
            vector<float> left={environment[0][0],self_position[1]};
            vector<float> right={environment[1][0],self_position[1]};
            vector<float> top={self_position[0],environment[2][1]};
            vector<float> buttom_force=potential_field_two_point(self_position,buttom,300);
            vector<float> left_force=potential_field_two_point(self_position,left,300);
            vector<float> right_force=potential_field_two_point(self_position,right,300);
            vector<float> top_force=potential_field_two_point(self_position,top,300);
            vector<float> combine={buttom_force[0]+left_force[0]+right_force[0]+top_force[0],buttom_force[1]+left_force[1]+right_force[1]+top_force[1]};
            return combine;
        }
        else
        {
            vector<float> combine={0,0};
            return combine;
        }
    }
    else
    {
        vector<float> rep_force={0,0};
        for(int i=0;i<environment.size();i++)
        {
            if(environment[i].empty())
                continue;
            vector<float> rep_force_i=potential_field_two_point(self_position,environment[i],300);
            rep_force[0]+=rep_force_i[0];
            rep_force[1]+=rep_force_i[1];
        }
        return rep_force;
    }
}

vector<float> formation_control::calc_rep_force(vector<vector<float> > boundary, vector<vector<float> > agents_position)
{
    vector<vector<float> > other_agents_position;
    vector<float> self_position=agents_position[robot_id-1];
    for(int i=0;i<agents_position.size();i++)
    {
        if(i==robot_id-1)
        {
            continue;
        }
        other_agents_position.push_back(agents_position[i]);
    }
    vector<float> rep_force_boundary=artifical_potential_rep_field(boundary,self_position,true);
    vector<float> rep_force_agents_between=artifical_potential_rep_field(other_agents_position,self_position);
    vector<float> rep_force;
    for(int i=0;i<rep_force_boundary.size();i++)
    {
        rep_force.push_back(rep_force_boundary[i]+rep_force_agents_between[i]);
    }
    return rep_force;
}

void formation_control::reverse_axis(vector<vector<vector<float> > > &original_data, int axis)
{
    for(int i=0;i<original_data.size();i++)
    {
        for(int j=0;j<original_data[i].size();j++)
        {
            if(axis>=original_data[i][j].size())
            {
                std::cerr<<"error reverse axis"<<std::endl;
                break;
            }
            original_data[i][j][axis]*=-1;
        }
    }
}

vector<vector<vector<float> > > formation_control::subtract_one_dim(vector<vector<vector<float> > > agents_postion_3D, int dim)
{
    vector<vector<vector<float> > >  agents_postion_subtracted;
    for(int i=0;i<agents_postion_3D.size();i++)
    {
        vector<vector<float> > agent_i_2D;
        for(int j=0;j<agents_postion_3D[i].size();j++)
        {
            vector<float> agent_2D;
            for(int k=0;k<agents_postion_3D[i][j].size();k++)
            {
                if(k!=dim)
                    agent_2D.push_back(agents_postion_3D[i][j][k]);
            }
            agent_i_2D.push_back(agent_2D);
        }
        agents_postion_subtracted.push_back(agent_i_2D);
    }
    return agents_postion_subtracted;
}

vector<vector<float> > formation_control::calc_relative_pos(vector<vector<float> > abs_pos, vector<float> original)
{
    vector<vector<float> > relative_pos;
    for(int i=0;i<abs_pos.size();i++)
    {
        vector<float> relative_one_point;
        for(int j=0;j<abs_pos[i].size();j++)
        {
            relative_one_point.push_back(abs_pos[i][j]-original[j]);
        }
        relative_pos.push_back(relative_one_point);
    }
    return relative_pos;
}


vector<vector<float> > formation_control::get_agents_position(vector<vector<vector<float> > > agents_postion_2D)
{
    vector<vector<float> > agents_postion;
     for(int i=0;i<agents_postion_2D.size()-1;i++)
     {
         for(int j=0;j<agents_postion_2D[i].size();j++)
         {
             agents_postion.push_back(agents_postion_2D[i][j]);
         }
     }
     return agents_postion;
}

vector<float> formation_control::convert_2D_disAng(vector<float> target_2D)
{
    float move_x=target_2D[0];
    float move_y=target_2D[1];

    vector<float> target_dist_ang;
    float target_dist=sqrt(pow(move_x,2)+pow(move_y,2));
    float target_ang=atan2(move_y,move_x);
    if(move_x>0&&move_y>0)
    {
        //第一象限
        target_ang=target_ang-1/2.0*PI;
    }
    else if(move_x>0&&move_y<0)
    {
        //第四象限
        target_ang=target_ang-PI/2;
    }
    else if(move_x<0&&move_y>0)
    {
        //第二象限
        target_ang=target_ang-PI/2;
    }
    else
    {
        //第三象限
        target_ang=3*PI/2+target_ang;
    }

    target_dist_ang.push_back(target_dist);
    target_dist_ang.push_back(target_ang);
    return target_dist_ang;
}

void formation_control::start_moving(vector<float> target_dist_ang)
{
    if(target_dist_ang.empty())
        return;
    kinematicController kine_control;
    float target_distance=target_dist_ang[0];
    float target_angle_rad=target_dist_ang[1];

    float move_speed_min=0.08;
    float move_speed_max=0.17;
    if(target_distance<30)
        return;
    else if(target_distance>800)
        target_distance=800;
    target_distance=target_distance;

    float feed_back_ratio=(0.1*log(1+target_distance)-0.34)/0.36;
    float move_speed=move_speed_min+feed_back_ratio*(move_speed_max-move_speed_min);

    kine_control.moveForward(move_speed,target_angle_rad,0.4);

}
