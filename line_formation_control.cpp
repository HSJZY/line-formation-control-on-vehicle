#include "line_formation_control.h"

line_formation_control::line_formation_control()
{
    this->m_direction_angle=0;
    this->m_inter_distance=600;
}
line_formation_control::line_formation_control(float direction,float inter_distance)
{
    this->m_direction_angle=direction;
    this->m_inter_distance=inter_distance;
}

void line_formation_control::start_line_formation()
{
    carStatus cur_robot_statue;
//    int last_drive_side=5;
//    kine_control.switchMode();
    while(1)
    {
        if (cur_robot_statue.get_formation_is_stop_state()==true)break;

        vector<vector<vector<float> > > agents_postion_3D=cur_robot_statue.get_agents_position();

//        vector<vector<vector<float> > > agents_postion_3D={{{40.68,-490.4,2529.24}},{{-524.63,130.59,2602.38}},{{-517.21,128.82,2567.48}},{{311.26,13.84,2633.51}},{{-1290.2,672.04,2613.99},{1790.2,672.04,2613.99},{1790.2,-927.96,2613.99}}};

        if(agents_postion_3D.empty()) continue;
        vector<vector<vector<float> > > agents_postion_2D=subtract_one_dim(agents_postion_3D,2);
        reverse_axis(agents_postion_2D,1);

        vector<vector<float> > boundary_sorted=agents_postion_2D[agents_postion_2D.size()-1];
        vector<vector<float> > agents_position=get_agents_position(agents_postion_2D);

        vector<float> rep_force=calc_rep_force(boundary_sorted,agents_position);
        if(boundary_sorted.empty())
        {
            cerr<<"检测出错啦！！！！";
            break;
        }
        vector<vector<float> > agents_relative_self_2D=calc_relative_pos(agents_position,agents_position[robot_id-1]);
        vector<vector<float> > agents_dist_ang=convert_2D_dist_ang(agents_relative_self_2D);
        vector<vector<float>> neighbor_2_agents=choose_nearest_two_neighbors_line(agents_dist_ang,this->m_direction_angle);

        vector<float> target_dist_ang=calc_target_dist_direction(neighbor_2_agents,rep_force);
        start_moving(target_dist_ang);
//        start_moving(target_dist_ang,first_pid,last_drive_side);
        cout<<"pause...";
    }
}

void line_formation_control::reverse_axis(vector<vector<vector<float> > > &original_data, int axis)
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

vector<float> line_formation_control::calc_rep_force(vector<vector<float> > boundary, vector<vector<float> > agents_position)
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

vector<vector<float> > line_formation_control::get_agents_position(vector<vector<vector<float> > > agents_postion_2D)
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

vector<vector<vector<float> > > line_formation_control::subtract_one_dim(vector<vector<vector<float> > > agents_postion_3D, int dim)
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

/// 傻瓜排序，我也是醉了！！！
/// \brief line_formation_control::calc_boundary
/// sort the agents postition to the sequence as follow
///   4..3    或者: ....  或者:  ....
///   .  .         .  .         .  .
///   1..2         1..2         1...
/// \param agents_position_2D
/// \return [1,2,3,4]或者[1,2]
///
vector<vector<float> > line_formation_control::calc_boundary(vector<vector<vector<float> > > agents_position_2D)
{
    vector<vector<float> > boundary_pos=agents_position_2D[agents_position_2D.size()-1];//只适用于4个或2个的情况
    vector<vector<float> > boundary_sorted;
    if(boundary_pos.size()==0)
    {
        cerr<<"without boundary";
        return boundary_sorted;
    }

    if(boundary_pos.size()==1)
    {
        return boundary_pos;
    }
    else if(boundary_pos.size()==2)
    {
        if(boundary_pos[0][0]<boundary_pos[1][0])
        {
            boundary_sorted.push_back(boundary_pos[0]);
            boundary_sorted.push_back(boundary_pos[1]);
        }
        else
        {
            boundary_sorted.push_back(boundary_pos[1]);
            boundary_sorted.push_back(boundary_pos[0]);
        }
        return boundary_sorted;
    }
    else if(boundary_pos.size()==4)
    {
        vector<float> bound_1;
        vector<float> bound_2;
        vector<float> bound_3;
        vector<float> bound_4;
        int min_x_1=0,min_x_2=1;

        for(int i=2;i<4;i++)
        {
            if(boundary_pos[min_x_1][0]>boundary_pos[min_x_2][0])
            {
                int tmp=min_x_1;
                min_x_1=min_x_2;
                min_x_2=tmp;
            }
            if(boundary_pos[i][0]<boundary_pos[min_x_2][1])
            {
                min_x_2=i;
            }
        }
        if(boundary_pos[min_x_1][1]<boundary_pos[min_x_2][1])
        {
            bound_1=boundary_pos[min_x_1];
            bound_4=boundary_pos[min_x_2];
        }
        else
        {
            bound_1=boundary_pos[min_x_2];
            bound_4=boundary_pos[min_x_1];
        }
        vector<vector<float> > pos_2_3;
        for(int i=0;i<4;i++)
        {
            if(i!=min_x_1&&i!=min_x_2)
            {
                pos_2_3.push_back(boundary_pos[i]);
            }
        }
        if(pos_2_3[0][1]<pos_2_3[1][1])
        {
            bound_2=pos_2_3[0];
            bound_3=pos_2_3[1];
        }
        else
        {
            bound_2=pos_2_3[1];
            bound_3=pos_2_3[0];
        }
        boundary_sorted.push_back(bound_1);
        boundary_sorted.push_back(bound_2);
        boundary_sorted.push_back(bound_3);
        boundary_sorted.push_back(bound_4);
        return boundary_sorted;
    }
    else
    {
        return boundary_sorted;
    }
}

vector<vector<float> > line_formation_control::calc_relative_pos(vector<vector<float> > abs_pos, vector<float> original)
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

vector<vector<float> > line_formation_control::convert_2D_dist_ang(vector<vector<float> > relative_pos)
{
    vector<vector<float> > vec_2D_dist_ang;
    for(int i=0;i<relative_pos.size();i++)
    {
        if(relative_pos[i].empty())
            continue;
        // remove it_self
        if(i==robot_id-1)
            continue;
        vector<float> single_dist_ang;
        float angle=atan2(relative_pos[i][1],relative_pos[i][0]);
        if(angle>=0 && angle<=PI)
        {
            angle=angle-PI/2.0;
        }
        else
        {
            if(relative_pos[i][0]<=0)
            {
                angle=angle+3*PI/2.0;
            }
            else
            {
                angle=angle-PI/2;
            }
        }

        float distance=sqrt(pow(relative_pos[i][0],2)+pow(relative_pos[i][1],2));
        single_dist_ang.push_back(distance);
        single_dist_ang.push_back(angle);
        vec_2D_dist_ang.push_back(single_dist_ang);
    }
    return vec_2D_dist_ang;
}

//挑选出直线编队中左右最近的机器人
vector<vector<float>> line_formation_control::choose_nearest_two_neighbors_line(vector<vector<float> > vec_total_dis_angle,float direction_angle)
{
    vector<vector<float>> both;
    vector<float> nearest_left;
    vector<float> nearest_right;
    //设定最小值
    float nearest_left_x=1000000000;
    float nearest_right_x=1000000000;

    for(int i=0;i<vec_total_dis_angle.size();i++)
    {
        float cur_distance=vec_total_dis_angle[i][0];
        float cur_angle_org=vec_total_dis_angle[i][1];
        //给定指定方向，计算当前点相对于给定方向的角度
        float cur_angle=cur_angle_org+(-direction_angle)/180.0*PI;
        //以下cur_x代表垂直于给定方位上的分量，cur_y代表给定方向上的分量
        float cur_x=-cur_distance*sin(cur_angle);
//        float cur_y=cur_distance*cos(cur_angle);
        if(cur_angle>0)
        {
            //此时是在左边的情况
            if(abs(cur_x)<nearest_left_x)
            {
                nearest_left_x=abs(cur_x);
                if(nearest_left.size()==2)
                {
                    nearest_left[0]=cur_distance;
                    nearest_left[1]=cur_angle_org;
                }
                else
                {
                    nearest_left.push_back(cur_distance);
                    nearest_left.push_back(cur_angle_org);
                }
            }
        }
        else
        {
            //此时是在右边的情况
            if(abs(cur_x)<nearest_right_x)
            {
                nearest_right_x=abs(cur_x);
                if(nearest_right.size()==2)
                {
                    nearest_right[0]=cur_distance;
                    nearest_right[1]=cur_angle_org;
                }
                else
                {
                    nearest_right.push_back(cur_distance);
                    nearest_right.push_back(cur_angle_org);
                }
            }
        }
    }
    both.push_back(nearest_left);
    both.push_back(nearest_right);
    return both;
}

vector<float> line_formation_control::calc_target_dist_direction(vector<vector<float> > two_nearby,vector<float> rep_force)
{
    float move_x,move_y;
    vector<float> nearest_left=two_nearby[0];
    vector<float> nearest_right=two_nearby[1];
    vector<float> target_dist_ang;

//    nearest_right[0]=876.920105;
//    nearest_right[1]=-1.344329;

    if(nearest_left.empty()&&nearest_right.empty())
    {
        //此时该机器人属于失联状态
        return target_dist_ang;
    }
    else if(nearest_left.empty())
    {
        //此时机器人在最左侧，切换下坐标系，以其右边机器人为原点
        float self_x=-nearest_right[0]*sin(abs(nearest_right[1]));
        float self_y=-nearest_right[0]*cos(nearest_right[1]);
        float target_x=-(this->m_inter_distance)*cos(this->m_direction_angle);
        float target_y=(this->m_inter_distance)*sin(-this->m_direction_angle);
        move_x=target_x-self_x;
        move_y=target_y-self_y;
    }
    else if(nearest_right.empty())
    {
        //此时机器人在最右侧,切换下坐标系，以其右边机器人为原点
        float self_x=nearest_left[0]*sin(nearest_left[1]);
        float self_y=-nearest_left[0]*cos(nearest_left[1]);
        float target_x=this->m_inter_distance*cos(this->m_direction_angle);
        float target_y=this->m_inter_distance*sin(this->m_direction_angle);
        move_x=target_x-self_x;
        move_y=target_y-self_y;
    }
    else
    {
        //此时机器人两侧都有邻近机器人,以自身为坐标系
        float target_x=(nearest_right[0]*abs(sin(nearest_right[1]))-nearest_left[0]*sin(nearest_left[1]))/2;
        float target_y=(nearest_left[0]*cos(nearest_left[1])+nearest_right[0]*cos(nearest_right[1]))/2.0;
        move_x=target_x;
        move_y=target_y;
    }

    move_x+=rep_force[0];
    move_y+=rep_force[1];

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

void line_formation_control::start_moving(vector<float> target_dist_ang)
{
    if(target_dist_ang.empty())
        return;
    kinematicController kine_control;
    float target_distance=target_dist_ang[0];
    float target_angle_rad=target_dist_ang[1];

    float move_speed_min=0.11;
    float move_speed_max=0.22;
    if(target_distance<30)
        return;
    else if(target_distance>800)
        target_distance=800;
    target_distance=target_distance;

    float feed_back_ratio=(0.1*log(1+target_distance)-0.34)/0.36;
    float move_speed=move_speed_min+feed_back_ratio*(move_speed_max-move_speed_min);

    kine_control.moveForward(move_speed,target_angle_rad,0.4);

}

vector<float> line_formation_control::artifical_potential_rep_field(vector<vector<float> > environment,vector<float> self_position,bool is_boundary)
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
