#include "xml_handler.cpp"
#include <iostream>

using namespace std;

int main()
{
    // WRITE

    cv::Mat my_trans = cv::Mat::ones(2, 3, CV_8U);
    XML_handler xml;
    xml.rootName("camera_params");
    xml.addParam("baceline", 0.1234f);
    xml.addParam<uint>("translation", my_trans);
    xml.save("camera_params.xml");


    // READ
    cv::Mat new_trans;
    float new_baceline;
    XML_handler xml1("camera_params.xml");
    new_baceline = xml1.readParam<float>("baceline");
    xml1.readParam<uint>("translation", new_trans);

    cout << "baceline: " << new_baceline << endl;

    cout << "translation: " << endl;
    for(int i = 0; i < new_trans.rows; i++){
        const uint* row = new_trans.ptr<uint>(i);

        for(int j = 0; j < new_trans.cols; j++) {
            cout << row[j] << " ";
        }
        cout << endl;
    }

    return 0;
}
