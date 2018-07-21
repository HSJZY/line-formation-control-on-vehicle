#ifndef ARBITARY_FORMATION_CONTROL_H
#define ARBITARY_FORMATION_CONTROL_H

#include<vector>
#include<iostream>
#include<thread>
#include"formation_control.h"
#include"globalsettings.h"
#include"carstatus.h"
using namespace std;

class arbitary_formation_control: protected formation_control
{
public:
    arbitary_formation_control(vector<vector<float> > assignment);
    void set_formation_assignment(vector<vector<float> > assignment);
    void start_formation();
public:
    vector<vector<float> > subtract_two_vector(vector<vector<float> > vec_1,vector<vector<float> > vec_2);
    vector<float> calc_input_ui(vector<vector<int> > L_G,vector<vector<float> > xi_di,vector<int> hung_assignment);
    vector<float> add_two_vector(vector<float> vec_1,vector<float> vec_2);

    vector<vector<float> > calc_hungarian_changed(vector<vector<float> > orig_matrix,vector<int> hung_array);

private:
    vector<vector<float> > m_target_formation;
    vector<vector<int> > m_formation_topology;
};

#endif // ARBITARY_FORMATION_CONTROL_H
