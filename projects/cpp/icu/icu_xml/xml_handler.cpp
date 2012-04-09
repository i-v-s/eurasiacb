#ifndef XML_HANDLER_CPP
#define XML_HANDLER_CPP

#include <opencv-2.3.1/opencv2/core/core.hpp>
#include <string>
#include <libxml++/libxml++.h>
#include <stdio.h>
#include <iostream>

using namespace std;
using namespace cv;
using namespace xmlpp;

class XML_handler
{
public:
    XML_handler(){
        doc = new Document();
        root = doc->create_root_node("root");
    }

    XML_handler(string filename){
        try {
            parser.set_substitute_entities();
            parser.parse_file(filename);
            if(parser) {
                doc = parser.get_document();
                root = doc->get_root_node();
            }
        } catch (const std::exception& ex) {
            cout << ex.what() << endl;
        }
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
        xmlpp::Element* param = root->add_child(paramName);
        param->set_attribute("rows", str_convert(paramValue.rows));
        param->set_attribute("cols", str_convert(paramValue.cols));

        xmlpp::Element* xml_row;
        xmlpp::Element* xml_elem;

        for(int i = 0; i < paramValue.rows; i++){
            const T* row = paramValue.ptr<T>(i);
            xml_row = param->add_child("row");
            xml_row->set_attribute("num", str_convert(i));

            for(int j = 0; j < paramValue.cols; j++) {
                xml_elem = xml_row->add_child("col");
                xml_elem->set_attribute("num", str_convert(j));
                xml_elem->add_child_text(str_convert(row[j]));
            }
        }
    }

    template <class T>
    T readParam(string paramName){

        T result;
        NodeSet set(root->find(paramName + "[1]"));
        for(xmlpp::NodeSet::iterator i=set.begin(); i != set.end(); ++i){
               xmlpp::Element * element = dynamic_cast<xmlpp::Element*>(*i);
               result = convert<T>(element->get_child_text()->get_content());
        }

        return result;
    }

    template <class T>
    void readParam(string paramName, Mat& paramValue)
    {
        NodeSet set(root->find(paramName + "[1]"));

        for(xmlpp::NodeSet::iterator i=set.begin(); i != set.end(); ++i){
               xmlpp::Element * element = dynamic_cast<xmlpp::Element*>(*i);
               int rows = convert<int>(element->get_attribute("rows")->get_value());
               int cols = convert<int>(element->get_attribute("cols")->get_value());
               paramValue = Mat(rows, cols, DataType<T>::type);

               xmlpp::Node::NodeList row_list = element->get_children();
               int r;
               int c;
               for(xmlpp::Node::NodeList::iterator iter = row_list.begin(); iter != row_list.end(); ++iter)
               {
                   xmlpp::Element * row = dynamic_cast<xmlpp::Element*>(*iter);
                   xmlpp::Node::NodeList col_list = row->get_children();
                   r = convert<int>(row->get_attribute("num")->get_value());

                   for(xmlpp::Node::NodeList::iterator col_iter = col_list.begin(); col_iter != col_list.end(); ++col_iter)
                   {
                       xmlpp::Element * col = dynamic_cast<xmlpp::Element*>(*col_iter);
                       c = convert<int>(col->get_attribute("num")->get_value());

                       //cout << "(rows, cols, value): " << r << " "  << c << " " << col->add_child_text()->get_content() << endl;
                       paramValue.at<T>(r,c)= convert<T>(col->add_child_text()->get_content());
                   }
               }
        }
    }

    void save(string filename){
        doc->write_to_file(filename);
    }

private:
    DomParser parser;
    Document* doc;
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

    template <class T>
    T convert(string value){
        stringstream ss;
        T result;
        ss << value;
        ss >> result;
        ss.clear();
        return result;
    }
};

#endif // XML_HANDLER_CPP
