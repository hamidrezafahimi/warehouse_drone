
// #include <opencv2/highgui/highgui.hpp>
// #include <iostream>
// using namespace cv;

#include<iostream>
// #include<vector> // for 2D vector
#include<algorithm> // for sort()
#include<vector>
#include<string>
#include <bits/stdc++.h>
#include <sstream>
using namespace std;

const int N = 6, M = 10, L = 5;
const float Percent = 60;
const float x_dist=3, y_dist=0.572, z_dist=0.3, x_=0.1425, y_=-0.17, z_=2.7845,
            x_ct = 0.2864, y_ct = -0.4585, z_ct = 2.78045;


string getBoxScript(int i, int j, int k, int n, vector<float> dim)
{
    vector<float> outp;
    // outp = {x_, y_+((j)*y_dist), z_-((i)*z_dist)+(dim[2]/2)};

    string num = to_string(n);
    string xyz = to_string(x_+(k*x_dist))+" "+to_string(y_+((j)*y_dist))+" "+to_string(z_-((i)*z_dist)+(dim[2]/2));
    string xyz_tag = to_string(x_+(k*x_dist)+(dim[0]/2))+" "+to_string(y_+((j)*y_dist))+" "+to_string(z_-((i)*z_dist)+(dim[2]/2));
    auto size = dim.size();
    string dims = " ";
    for(int i=0; i<size; ++i){
      dims = dims +" "+ to_string(dim[i]);
    }

    string script1 = "<link name=\"box";
    string script2 = "\">\n <pose>";
    string script3 = " 0 0 0</pose>\n <collision name=\"collision\">\n <geometry>\n <box>\n  <size>";
    string script4 = "</size>\n </box>\n </geometry>\n </collision>\n <visual name=\"visual\">\n"
    "<cast_shadows>false</cast_shadows>\n <geometry>\n <box>\n <size>";
    string script5 = "</size>\n </box>\n </geometry>\n <material>\n <ambient>0.1 0.1 0.1 1</ambi\
ent>\n <diffuse>0.79 0.64 0.45 1</diffuse>\n <specular>0 0 0 0</specular>\n <emissive>0 0 0 1</em\
issive>\n </material></visual>\n </link>\n\n<link name=\"tag";
    string script6 = " 0 0 0</pose>\n<visual name=\"visual\">\n <cast_shadows>false</cast_shadows>\n <geometry>\n <\
box>\n <size> 0.0001 0.10000 0.10000</size>\n </box>\n </geometry>\n <material>\n <script>\n <uri>model://\
warehouseShelf/materials/scripts</uri>\n <uri>model://warehouseShelf/materials/textures</uri>\n <name>";
    string script7 = "</name>\n </script>\n </material>\n </visual>\n </link>\n\n";

    string stri = script1+num+script2+xyz+script3+dims+script4+dims+script5+num+script2+xyz_tag+script6+num+script7;

    return stri;
}


string getColorTagsScript(int numx, int numy, int numz)
{
  int totalNum = numx*numy*numz, num=0;
  string script = "\n\n\n", xyz;
  string script1 = "<link name=\"colortag";
  string script2 = "\">\n <pose>";
  string script3 = " 0 0 0</pose>\n <visual name=\"visual\">\n <cast_shadows>false</cast_shadows>\n <geometry>\n <box>\n <size> 0.0001 0.05000 0.05000</size>\n  </box>\n  </geometry>\n  <material>\n  <ambient>0.1 0.1 0.1 1</ambient>\n  <diffuse>1 0.145 0.9765 1</diffuse>\n  <specular>0 0 0 0</specular>\n  <emissive>0 0 0 1</emissive>\n  </material>\n  </visual>\n </link>";
  for(int i=0; i<numx; ++i){
    for(int j=0; j<numy; ++j){
      for(int k=0; k<numz; ++k){
        num++;
        xyz = to_string(x_ct+(i*x_dist))+" "+to_string(y_ct+(j*y_dist))+" "+to_string(z_ct-(k*z_dist));
        script = script + "\n\n" + script1 + to_string(num) + script2 + xyz + script3;
      }
    }
  }
  return script;

}


string arrangeShelf(int m=M, int n=N, int l=L, float p=Percent)
{
    int boxNum;
    boxNum = (p/100)*m*n*l;
    int i=0;
    vector<vector<int>> inds;
    vector<float> randDim;
    string script = "";
    int ii, jj, kk;

    while(i<boxNum){
      ii = rand()%m;
      jj = rand()%n;
      kk = rand()%l;
      // cout<<ii<<'\t'<<jj<<endl;
      randDim.clear();
      randDim = {float(12+(rand()%9))/100, float((12+(rand()%29)))/100, float(12+(rand()%14))/100};
      // randDim.push_back(float(12+(rand()%9))/100);
      // randDim.push_back(float((12+(rand()%29)))/100);
      // randDim.push_back(float(12+(rand()%14))/100);
      bool flag = false;
      for (int k=0; k<inds.size(); ++k){
        if (inds[k][0]==ii && inds[k][1]==jj && inds[k][2]==kk){
          flag = true;
          break;
        }
      }
      if (flag)
        continue;
      else {
        script = script + "\n\n" + getBoxScript(ii, jj, kk, i+1, randDim);

        inds.push_back({ii, jj, kk});
        i += 1;
      }
    }
    script = script + getColorTagsScript(L, N+1, M);
    return script;
}


int main()
{
    srand((unsigned) time(0));
    // arrangeShelf();
    cout<<arrangeShelf();
    cout<<endl;

    return 0;
}
