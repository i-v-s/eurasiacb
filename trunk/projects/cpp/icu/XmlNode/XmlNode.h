// XmlNode.h: interface for the CXmlNode class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_XMLNODE_H__B2E20BB6_282F_11D5_857B_0050DA5D7623__INCLUDED_)
#define AFX_XMLNODE_H__B2E20BB6_282F_11D5_857B_0050DA5D7623__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#define WIN32_LEAN_AND_MEAN		// Exclude rarely_used stuff from Windows headers

#include "portable.h"
#include <windows.h>
#include <fstream.h>
#include <string>
#include <map>


//================================================================================
    
void UTL_PutXmlHeader(::ofstream & ofs);
                             
//================================================================================

    class CXmlAttributeSet
{
protected:
   std::map<std::string, std::string>     m_AttributeMap;
public:
   CXmlAttributeSet();
   CXmlAttributeSet(LPCTSTR name, LPCTSTR value);
   virtual ~CXmlAttributeSet();

   CXmlAttributeSet & operator = (CXmlAttributeSet & r);


   virtual HRESULT AddAttribute(LPCTSTR name, LPCTSTR value);

   friend class CXmlNodeBase;
};

//================================================================================
//================================================================================
/* 
   UTF-8 :

   All characters in the range '\u0001' to '\u007F' are represented by a single byte: 

      0 : bits 0-7 

   Characters in the range '\u0080' to '\u07FF' are represented by a pair of bytes: 

   1 1 0 : bits 6-10 
   1 0   : bits 0-5 


   Characters in the range '\u0800' to '\uFFFF' are represented by three bytes: 
   1 1 1 0  : bits 12-15 
   1 0      : bits 6-11 
   1 0      : bits 0-5 

  (For more information, see X/Open Company Ltd., "File System Safe UCS 
   Transformation Format (FSS_UTF)", X/Open Preliminary Specification, 
   Document Number: P316. This information also appears in ISO/IEC 10646, Annex P.) 

*/

class CXmlNodeBase
{
protected:
   DWORD             m_cRef;;
   static LPCTSTR    m_szValueTrue;
   static LPCTSTR    m_szValueFalse;

   
   std::string       m_tag;
   bool              m_bDidStart;
   std::string       m_text;
   ::ofstream    &   m_ofs;

   CXmlAttributeSet  m_attribs;

   void  PutUTF8Char(int const wChar, std::string & outs);
   
   void  PutValue(LPCSTR value, std::string & outs);
   void  PutValue(LPCWSTR value, std::string & outs);

   // replaces special characters that might occur in the obuject name
   void  ReplaceSpecial(void);

public:
   static LPCTSTR    EMPTY_DATA;

   static void ValidateTagBuffer(IN LPCTSTR tag);
   static void ReplaceInvalidTagChars(IN OUT LPSTR tag);

   CXmlNodeBase(::ofstream & ofs, IN LPCSTR tag);
   virtual ~CXmlNodeBase();

   virtual void StartNode(void);
   virtual void AssignAttributeSet(CXmlAttributeSet & ras);

   DWORD  AddRef(void);
   DWORD  Release(void); 

public:
   // The exception that will be thrown if the tag contains special characters
   class CXmlNodeException
   { 
   public: 
      const char m_chTheWrongCharacter; // const => can be set only where it is set below
      CXmlNodeException(char ch) : m_chTheWrongCharacter(ch) {} 
   };
};

class CXmlNode : public CXmlNodeBase
{
public:
   CXmlNode(::ofstream & ofs, IN LPCSTR tag, IN LPCSTR value);
   CXmlNode(::ofstream & ofs, IN LPCSTR tag, IN LPCWSTR value);
   CXmlNode(::ofstream & ofs, IN LPCSTR tag, IN const int nValue);
   CXmlNode(::ofstream & ofs, IN LPCSTR tag, IN const DWORD nValue);
   CXmlNode(::ofstream & ofs, IN LPCSTR tag, IN const DWORDLONG nValue);
   CXmlNode(::ofstream & ofs, IN LPCSTR tag, IN const bool bValue);
   CXmlNode(::ofstream & ofs, IN LPCSTR tag, IN const double dValue);
   virtual ~CXmlNode();

   DWORD  AddRef(void)  {return CXmlNodeBase::AddRef();}
   DWORD  Release(void) {return CXmlNodeBase::Release();}
};


//================================================================================
#define START_NODE(F,N,V)     {  CXmlNode   xxxml((F),(N),(V)); xxxml.StartNode();
#define START_NODE_A(F,N,V,A) {  CXmlNode   xxxml((F),(N),(V)); \
                                 xxxml.AssignAttributeSet(A) ; xxxml.StartNode();
#define CLOSE_NODE            }
#define DO_NODE(F,N,V)        START_NODE(F,N,V); CLOSE_NODE
#define DO_NODE_A(F,N,V,A)    START_NODE_A(F,N,V,A); CLOSE_NODE
//================================================================================
//================================================================================


#endif // !defined(AFX_XMLNODE_H__B2E20BB6_282F_11D5_857B_0050DA5D7623__INCLUDED_)
