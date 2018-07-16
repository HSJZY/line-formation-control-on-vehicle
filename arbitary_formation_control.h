#ifndef ARBITARY_FORMATION_CONTROL_H
#define ARBITARY_FORMATION_CONTROL_H

#include<vector>
#include<iostream>
#include<thread>
using namespace std;

class arbitary_formation_control
{
public:
    arbitary_formation_control();
private:
    vector<int> formation_assignment;
    vector<int> formation_topology;
};

#endif // ARBITARY_FORMATION_CONTROL_H
