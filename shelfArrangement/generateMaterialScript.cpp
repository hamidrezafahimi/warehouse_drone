
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

const int N = 180;


int main()
{
  string script1, script2, script3, tag, script;
  script1 = "material ";
  script2 = "\n{\n\ttechnique\n\t	{\n\t\tpass\n\t\t{\n\t\t\ttexture_unit\n\t\t\t{\n\t\t\t\ttexture ";
  script3 = ".jpg\n\t\t\t}\n\t\t}\n\t}\n}\n\n";
  for(int i=0; i<N; ++i){
    tag = to_string(i+1);
    script = script1+tag+script2+tag+script3;
    cout<<script<<endl;
  }
  return 0;
}
