#include <iostream>
#include <fstream>

//#include "/home/vasiliy/svn/eurasiacb/projects/cpp/libviso2/src/visualodometry.h"
#include <matcher.h>


using namespace std;

int main()
{
    ifstream::pos_type size;
    uint8_t* image;
    ifstream file ("/home/vasiliy/svn/eurasiacb/projects/cpp/ochi/I1_000000.png");

    Matcher m;

    if (file.is_open())
      {
        size = file.tellg();
        image = new uint8_t [size];
        file.seekg (0, ios::beg);
        file.read (image, size);
        file.close();


        //m.pushBack(image, 1267, 387, 1267);
        //m.getMatches();

        delete[] image;
    } else cout << "Unable to open file\n";

    //Matcher m;

    //m.pushBack();

//    VisualOdometry vo;
//    vo.setCalibration(1.0, 1.0, 1.0, 1.0);
    return 1;
}


