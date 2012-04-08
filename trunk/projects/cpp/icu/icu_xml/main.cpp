#include "xml_handler.cpp"

int main()
{
    cv::Mat my_trans = cv::Mat::ones(2, 3, CV_8U);
    XML_handler xml;
    xml.rootName("camera_params");
    xml.addParam<float>("float", 0.1234f);
    xml.addParam<uint>("translation", my_trans);
    xml.save("camera_params.xml");

    return 0;
}
