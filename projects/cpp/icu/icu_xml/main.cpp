#include "xml_handler.h"

int main()
{
    cv::Mat my_trans = cv::Mat::ones(2, 3, CV_8U);
    XML_handler xml;
    xml.rootName("camera_params");
    xml.addParam("translation", my_trans);
    xml.save("camera_params.xml");

    return 0;
}
