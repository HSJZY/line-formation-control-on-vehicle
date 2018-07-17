#include "test_program.h"
void test_subtract_two_vector()
{
    vector<vector<float> > vec_h={{1,2},{3,4},{3,4},{4,5}};
    arbitary_formation_control arbitary_formation(vec_h);
    vector<vector<float> > vec_1={{1,2},{3,2}};
    vector<vector<float> > vec_2={{5,3},{4,7}};

    auto vec_res=arbitary_formation.subtract_two_vector(vec_1,vec_2);
    cout<<"hello";
}

void tst_add_two_vector()
{
    vector<vector<float> > vec_h={{1,2},{3,4},{3,4},{4,5}};
    arbitary_formation_control arbitary_formation(vec_h);
    vector<float> vec_1={1,5.2};
    vector<float> vec_2={5,3};
    auto res=arbitary_formation.add_two_vector(vec_1,vec_2);
    cout<<"pause";
}

void test_calc_input_ui()
{
    vector<vector<float> > vec_h={{-1,-1},{-3,-3},{1,-1},{3,-3}};
    arbitary_formation_control arbitary_formation(vec_h);
    vector<vector<int> > L_G={{3,-1,-1,-1},{-1,3,-1,-1},{-1,-1,3,-1},{-1,-1,-1,3}};
    vector<vector<float> > xi_di={{0,0},{0,0},{0,-1},{0,0}};
    auto res=arbitary_formation.calc_input_ui(L_G,xi_di);
    cout<<"pause";
}
