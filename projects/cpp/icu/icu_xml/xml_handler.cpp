#include "xml_handler.h"

XML_handler::XML_handler()
{
    root = doc.create_root_node("root");
}

XML_handler::XML_handler(string filename)
{

}

void XML_handler::rootName(string rootname)
{
    root->set_name(rootname);
}

void XML_handler::addParam(string paramName, string paramValue)
{
    Element* elem = root->add_child(paramName);
    elem->add_child_text(paramValue);
}

void XML_handler::addParam(string paramName, const Mat& paramValue)
{
    xmlpp::Node* param = root->add_child(paramName);

    xmlpp::Element* xml_row;
    xmlpp::Element* xml_elem;

    for(int i = 0; i < paramValue.rows; i++){
        const uint* row = paramValue.ptr<uint>(i);
        xml_row = param->add_child("row");
        xml_row->set_attribute("num", str_convert(i));

        for(int j = 0; j < paramValue.cols; j++) {
            xml_elem = xml_row->add_child("elem");
            xml_elem->set_attribute("num", str_convert(j));
            xml_elem->add_child_text(str_convert(row[j]));
        }
    }
}

void XML_handler::save(std::string filename)
{
    doc.write_to_file(filename);
}

template <class T>
string XML_handler::str_convert(T value) {
    stringstream ss;
    string result;
    ss << value;
    ss >> result;
    ss.clear();
    return result;
}
