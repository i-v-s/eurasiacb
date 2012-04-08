#ifndef XML_HANDLER_H
#define XML_HANDLER_H

#include <opencv-2.3.1/opencv2/core/core.hpp>
#include <string>
#include <libxml++/libxml++.h>
#include <stdio.h>

using namespace std;
using namespace cv;
using namespace xmlpp;

class XML_handler
{
public:
    XML_handler();
    XML_handler(string filename);
    void rootName(string rootname);
    template <class T>
    void addParam(string paramName,T paramValue){
        Element* elem = root->add_child(paramName);
        elem->add_child_text(str_convert(paramValue));
    }
    //void addParam(string paramName, const Mat& paramValue);
    void save(string filename);
private:
    Document doc;
    Node* root;
    template <class T>
    string str_convert(T value);
};

#endif // XML_HANDLER_H
