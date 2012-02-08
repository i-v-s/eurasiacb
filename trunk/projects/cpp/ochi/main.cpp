#include <iostream>
#include <fstream>

//#include "/home/vasiliy/svn/eurasiacb/projects/cpp/libviso2/src/visualodometry.h"

using namespace std;

int main()
{
    ifstream::pos_type size;
    char * image;
    ifstream file ("/home/vasiliy/svn/eurasiacb/projects/cpp/ochi/I1_000000.png");

    if (file.is_open())
      {
        size = file.tellg();
        image = new char [size];
        file.seekg (0, ios::beg);
        file.read (image, size);
        file.close();



        delete[] image;
    } else cout << "Unable to open file\n";

    //Matcher m;

    //m.pushBack();

//    VisualOdometry vo;
//    vo.setCalibration(1.0, 1.0, 1.0, 1.0);
    return 1;
}


