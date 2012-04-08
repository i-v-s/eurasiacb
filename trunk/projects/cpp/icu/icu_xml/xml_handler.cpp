#ifndef XML_HANDLER_CPP
#define XML_HANDLER_CPP

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
    XML_handler(){
        root = doc.create_root_node("root");
    }
    XML_handler(string filename){

    }

    void rootName(string rootname){
        root->set_name(rootname);
    }

    template <class T>
    void addParam(string paramName,T paramValue){
        Element* elem = root->add_child(paramName);
        elem->add_child_text(str_convert(paramValue));
    }

    template <class T>
    void addParam(string paramName, const Mat& paramValue)
    {
        xmlpp::Node* param = root->add_child(paramName);

        xmlpp::Element* xml_row;
        xmlpp::Element* xml_elem;

        for(int i = 0; i < paramValue.rows; i++){
            const T* row = paramValue.ptr<T>(i);
            xml_row = param->add_child("row");
            xml_row->set_attribute("num", str_convert(i));

            for(int j = 0; j < paramValue.cols; j++) {
                xml_elem = xml_row->add_child("elem");
                xml_elem->set_attribute("num", str_convert(j));
                xml_elem->add_child_text(str_convert(row[j]));
            }
        }
    }

    void save(string filename){
        doc.write_to_file(filename);
    }

private:
    Document doc;
    Node* root;
    template <class T>
    string str_convert(T value){
        stringstream ss;
        string result;
        ss << value;
        ss >> result;
        ss.clear();
        return result;
    }
};

#endif // XML_HANDLER_CPP
