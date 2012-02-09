#include <iostream>
#include <fstream>

//#include "/home/vasiliy/svn/eurasiacb/projects/cpp/libviso2/src/visualodometry.h"
#include <matcher.h>

#define HEIGHT 387
#define WIDTH 1267

using namespace std;

int main()
{
    char pixel;
    int size = HEIGHT * WIDTH;
    uint8_t* image1 = new uint8_t [size];
    uint8_t* image2 = new uint8_t [size];

    ifstream file;
    Matcher m;

    file.open("/home/vasiliy/svn/eurasiacb/projects/cpp/ochi/I1_000000.png", ios_base::binary);

    if (!file.is_open())
    {
        cout << "Crap!\n";
        return 0;
    }

    for(i=0; i<size; i++) {
        file.get(pixel);
        image1[i] = static_cast<uint8_t>(pixel);
    }

    file.close();

    /*
    file.open("/home/vasiliy/svn/eurasiacb/projects/cpp/ochi/I1_000000.png", ios_base::binary);

    if (!file.is_open()) // && file2.is_open())
    {
        cout << "What'a hell! \n";
        return 0;
    }

    for(i=0; i<size; i++) {
        file.get(pixel);
        image2[i] = static_cast<uint8_t>(pixel);
    }

    file.close();
    */

    m.pushBack(image1, 1267, 387, 1267, false);
    m.pushBack(image1, 1267, 387, 1267, false);
    m.matchFeatures(0);
    m.bucketFeatures(4,30,30);
    std::vector<Matcher::p_match> features = m.getMatches();
    cout << features.size() << "\n";


    delete[] image;

    //Matcher m;

    //m.pushBack();

//    VisualOdometry vo;
//    vo.setCalibration(1.0, 1.0, 1.0, 1.0);
    return 1;
}


