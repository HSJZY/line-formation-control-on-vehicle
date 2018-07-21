#ifndef UTILS_H
#define UTILS_H

#include<string>
#include<iostream>
#include<vector>
#include<math.h>
#include<sys/time.h>
#include"wiringPi.h"
#include"globalsettings.h"
#include"log.h"
#include"carstatus.h"
using namespace std;
static vector<int> hung_assign={1,2,3,4};

void split_string(const string original_str,vector<string>& vec_strs,const string& split_char);
vector<vector<vector<float> > > parse_agents_position(string global_infomation,vector<int>& hungarian_assignment=hung_assign);
vector<float> potential_field_two_point(vector<float> self,vector<float> obstacle,float in_range);
void write_yaw_file(string filename);
int sgn(float num);
#endif // UTILS_H
