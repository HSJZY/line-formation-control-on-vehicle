#ifndef FORMATION_CONTROL_H
#define FORMATION_CONTROL_H

#include<vector>
#include"utils.h"
#include"kinematiccontroller.h"
using namespace std;

class formation_control
{
public:
    formation_control();
protected:
    vector<vector<vector<float> > > subtract_one_dim(vector<vector<vector<float> > > agents_postion_3D,int dim);
    void reverse_axis(vector<vector<vector<float> > > &original_data,int axis);

    vector<float> calc_rep_force(vector<vector<float> > boundary,vector<vector<float> > agents_position);
    vector<float> artifical_potential_rep_field(vector<vector<float>> boundary,vector<float> self_position,bool is_boundary=false);
    vector<vector<float> > calc_relative_pos(vector<vector<float> > abs_pos,vector<float> original);
    vector<vector<float> > get_agents_position(vector<vector<vector<float> > > agents_postion_2D);
    vector<float> convert_2D_disAng(vector<float> target_2D);
public:
    void start_moving(vector<float> target_dist_ang);
};

#endif // FORMATION_CONTROL_H
