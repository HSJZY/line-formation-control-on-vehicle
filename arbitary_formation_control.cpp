#include "arbitary_formation_control.h"

arbitary_formation_control::arbitary_formation_control(vector<vector<float> > assignment)
{
    set_formation_assignment(assignment);
}

void arbitary_formation_control::start_formation()
{
    carStatus cur_robot_statue;
    while(1)
    {
        if (cur_robot_statue.get_formation_is_stop_state()==true)break;
        vector<vector<vector<float> > > agents_postion_3D=cur_robot_statue.get_agents_position();
        vector<int> hung_assignment=cur_robot_statue.get_hung_assignment();

//        vector<vector<vector<float> > > agents_postion_3D={{{1236,-503,1}},{{-147.85,1.64,0}},{{-433,-436,0}},{{-183,506.3,0}},{}};
//        vector<int> hung_assignment={3,1,0,2};


        if(agents_postion_3D.empty()) continue;
        vector<vector<vector<float> > > agents_postion_2D=subtract_one_dim(agents_postion_3D,2);
        reverse_axis(agents_postion_2D,1);

        vector<vector<float> > boundary_sorted=agents_postion_2D[agents_postion_2D.size()-1];
        vector<vector<float> > agents_position=get_agents_position(agents_postion_2D);
        vector<float> rep_force=calc_rep_force(boundary_sorted,agents_position);

        vector<vector<float> > after_hungarian_changed=calc_hungarian_changed(agents_position,hung_assignment);

        vector<vector<float> > cur_others_self_2D=calc_relative_pos(after_hungarian_changed,agents_position[robot_id-1]);
        vector<vector<float> > cur_target_to_self=calc_relative_pos(this->m_target_formation,m_target_formation[hung_assignment[robot_id-1]]);
//        reverse_axis(this->m_target_formation,1);
//        vector<vector<float> > hung_target_formation=calc_hungarian_changed(this->m_target_formation,hung_assignment);

        vector<vector<float> > xi_di=subtract_two_vector(cur_others_self_2D,cur_target_to_self);
        vector<float> ui=calc_input_ui(this->m_formation_topology,xi_di,hung_assignment);
        vector<float> ui_a_rep=add_two_vector(rep_force,ui);
        vector<float> ui_target_dis_ang=convert_2D_disAng(ui_a_rep);
        start_moving(ui_target_dis_ang);
    }
}

vector<vector<float> > arbitary_formation_control::calc_hungarian_changed(vector<vector<float> > orig_matrix,vector<int> hung_array)
{
    vector<vector<float> > res_hungar_matrix(4,vector<float>());
    for(int i=0;i<hung_array.size();i++)
    {
        res_hungar_matrix[hung_array[i]]=orig_matrix[i];
    }
    return res_hungar_matrix;
}

void  arbitary_formation_control::set_formation_assignment(vector<vector<float> > assignment)
{
    vector<vector<float> > others_to_self;
    vector<float> self_position=assignment[robot_id-1];
    for(int i=0;i<assignment.size();i++)
    {
        vector<float> agent_i_to_self;
        for(int j=0;j<assignment[i].size();j++)
        {
            agent_i_to_self.push_back(assignment[i][j]-self_position[j]);
        }
        others_to_self.push_back(agent_i_to_self);
    }
    this->m_target_formation=others_to_self;

    // full connected graph
    vector<vector<int> > topology;
    for(int i=0;i<assignment.size();i++)
    {
        vector<int> topo_i;
        for(int j=0;j<assignment.size();j++)
        {
            if(j==i)
                topo_i.push_back(-(assignment.size()-1));
            else
                topo_i.push_back(1);
        }
        topology.push_back(topo_i);
    }
    this->m_formation_topology=topology;
}

vector<vector<float> > arbitary_formation_control::subtract_two_vector(vector<vector<float> > vec_1, vector<vector<float> > vec_2)
{
    vector<vector<float> > after_subtract;
    if(vec_1.size()!=vec_2.size()||vec_1[0].size()!=vec_2[0].size())
    {
        std::cerr<<"size of two vectors are not the same";
        return after_subtract;
    }
    for(int i=0;i<vec_1.size();i++)
    {
        vector<float> subtracted_i;
        for(int j=0;j<vec_1[0].size();j++)
        {
            subtracted_i.push_back(vec_1[i][j]-vec_2[i][j]);
        }
        after_subtract.push_back(subtracted_i);
    }
    return after_subtract;
}
vector<float> arbitary_formation_control::calc_input_ui(vector<vector<int> > L_G, vector<vector<float> > xi_di,vector<int> hung_assignment)
{
    vector<float> ui;
    vector<int> L_G_i=L_G[hung_assignment[robot_id-1]];

    for(int i=0;i<xi_di[0].size();i++)
    {
        float ui_axis=0;
        for(int j=0;j<L_G_i.size();j++)
        {
            ui_axis+=(L_G_i[j]*xi_di[j][i])/(L_G_i.size()-1);
        }
        ui.push_back(ui_axis);
    }
    return ui;
}

vector<float> arbitary_formation_control::add_two_vector(vector<float> vec_1, vector<float> vec_2)
{
    vector<float> v1_a_v2;
    if(vec_1.size()!=vec_2.size())
    {
        std::cerr<<"error:differ size of two vector";
        return v1_a_v2;
    }
    for(int i=0;i<vec_1.size();i++)
    {
        v1_a_v2.push_back(vec_1[i]+vec_2[i]);
    }
    return v1_a_v2;
}
