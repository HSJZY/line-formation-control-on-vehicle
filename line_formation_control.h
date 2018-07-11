#ifndef LINE_FORMATION_CONTROL_H
#define LINE_FORMATION_CONTROL_H
#include"globalsettings.h"

#include<iostream>
#include<vector>
#include<algorithm>
#include<math.h>
#include"utils.h"
#include"pid.h"
#include"kinematiccontroller.h"
#include"carstatus.h"


using namespace std;

class line_formation_control
{
public:
    line_formation_control();
    line_formation_control(float direction,float inter_distance);
    void start_line_formation();
private:
    vector<vector<float> > calc_boundary(vector<vector<vector<float> > > agents_position);
    vector<vector<vector<float> > > subtract_one_dim(vector<vector<vector<float> > > agents_postion_3D,int dim);
    void reverse_axis(vector<vector<vector<float> > > &original_data,int axis);

    vector<vector<float> > calc_relative_pos(vector<vector<float> > abs_pos,vector<float> original);

    vector<vector<float> > get_agents_position(vector<vector<vector<float> > > agents_postion_2D);

    vector<vector<float> > convert_2D_dist_ang(vector<vector<float> > relative_pos);
    vector<vector<float> > choose_nearest_two_neighbors_line(vector<vector<float> > vec_total_dis_angle,float direction_angle);
    vector<float> calc_target_dist_direction(vector<vector<float> > two_nearby,vector<float> rep_force);
public:
    void start_moving(vector<float> target_dist_ang);
    vector<float> calc_rep_force(vector<vector<float> > boundary,vector<vector<float> > agents_position);
//临时测试用
//public:
    vector<float> artifical_potential_rep_field(vector<vector<float>> boundary,vector<float> self_position,bool is_boundary=false);

private:
    float m_direction_angle;
    float m_inter_distance;
};

#endif // LINE_FORMATION_CONTROL_H
