#include <iostream>
#include <fstream>

//#include "/home/vasiliy/svn/eurasiacb/projects/cpp/libviso2/src/visualodometry.h"
#include <matcher.h>


using namespace std;

int main()
{
    ifstream::pos_type size;
    ifstream::pos_type size2;
    char* image;
    char* image2;
    ifstream file;
    ifstream file2;
    file.open("/home/robot/shmelya/svn/eurasiacb/projects/cpp/ochi/I1_000000.png", ios_base::binary);
    file2.open("/home/robot/shmelya/svn/eurasiacb/projects/cpp/libviso2/img/I1_000001.png", ios_base::binary);

    Matcher m;

    if (file.is_open() && file2.is_open())
      {
        file.seekg (0, ios::end);
        size = file.tellg();
        file.seekg (0, ios::beg);
        image = new char [size];
        for(int i=0; i < size; i++)
            image[i] = '5';
        file.read (image, size);
        file.close();

        uint8_t* imageint = new uint8_t [size];

        for(int i=0; i<size; i++) {
            imageint[i] = static_cast<uint8_t>(image[i]);
        }

        file2.seekg (0, ios::end);
        size2 = file2.tellg();
        file2.seekg (0, ios::beg);
        image2 = new char [size2];
        for(int i=0; i < size2; i++)
            image2[i] = '5';
        file2.read (image2, size2);
        file2.close();

        uint8_t* imageint2 = new uint8_t [size2];

        for(int i=0; i<size2; i++) {
             imageint2[i] = static_cast<uint8_t>(image2[i]);

        }

        cout << size << " " << size2 << "\n";

        m.pushBack(imageint, 1267, 387, 1267, false);
        m.pushBack(imageint2, 1267, 387, 1267, false);
        m.matchFeatures(0);
        m.bucketFeatures(4,30,30);
               std::vector<Matcher::p_match> features = m.getMatches();
        cout << features.size() << "\n";


        delete[] image;
    } else cout << "Unable to open file\n";

    //Matcher m;

    //m.pushBack();

//    VisualOdometry vo;
//    vo.setCalibration(1.0, 1.0, 1.0, 1.0);
    return 1;
}


