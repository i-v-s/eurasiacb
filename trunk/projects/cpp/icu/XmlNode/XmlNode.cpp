// XmlNode.cpp: implementation of the CXmlNodeBase class.
//
//////////////////////////////////////////////////////////////////////

#include "XmlNode.h"
#include <crtdbg.h>



void UTL_PutXmlHeader(::ofstream & ofs)
{
   ofs << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>" << endl;
}


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
CXmlAttributeSet::CXmlAttributeSet()
{
}

CXmlAttributeSet::CXmlAttributeSet(LPCTSTR name, LPCTSTR value)
{
   AddAttribute(name, value);
}

CXmlAttributeSet & CXmlAttributeSet::operator = (CXmlAttributeSet & r)
{
   std::map<std::string, std::string>::iterator    it;

   for ( it = r.m_AttributeMap.begin(); it != r.m_AttributeMap.end(); it++ ) {
      this->m_AttributeMap.insert(*it);
   }

   return *this;
}

CXmlAttributeSet::~CXmlAttributeSet()
{
}

//==============================================================================

HRESULT CXmlAttributeSet::AddAttribute(LPCTSTR name, LPCTSTR value)
{
   HRESULT ret = -1;

   try {
      CXmlNodeBase::ValidateTagBuffer(name);
      m_AttributeMap[name] = value;
      ret = S_OK;
   } catch (CXmlNodeBase::CXmlNodeException e) {
      e;
   }
   
   return (ret);
}


//==============================================================================
//==============================================================================
//==============================================================================

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
LPCTSTR CXmlNodeBase::EMPTY_DATA = (LPCTSTR)NULL;
LPCTSTR CXmlNodeBase::m_szValueTrue = _T("true");
LPCTSTR CXmlNodeBase::m_szValueFalse = _T("false");

CXmlNodeBase::CXmlNodeBase(::ofstream & ofs, IN LPCSTR tag)
 : m_cRef(0),
   m_bDidStart(false),
   m_ofs(ofs), 
   m_tag(tag)
{ 
   ReplaceSpecial();
}

CXmlNodeBase::~CXmlNodeBase() 
{ 
   if ( m_bDidStart ) {
      m_ofs << _T("</") << m_tag.c_str() << _T(">") << endl;
   }
}


void CXmlNodeBase::ReplaceSpecial(void)
{
   if ( m_tag.size() == 0 ) {
      throw CXmlNodeException('\0');
   }

   for ( int idx = 0; idx < m_tag.size(); idx++ ) {

      switch ( m_tag[idx] ) {
         case '&':
         case '<':
         case '>':
         case '\"':
         case '\'':
            throw CXmlNodeException(m_tag[idx]);
            break;
         case ' ':
            m_tag[idx] = '_';
            break;
         default:
            // OK
            break;
      }
   }
}

//==============================================================================
//==============================================================================

void CXmlNodeBase::ValidateTagBuffer(IN LPCTSTR tag)
{
   if ( lstrlen(tag) == 0 ) {
      throw CXmlNodeException('\0');
   }

   for ( int idx = 0; idx < lstrlen(tag); idx++ ) {

      switch ( tag[idx] ) {
         case '&':
         case '<':
         case '>':
         case '\"':
         case '\'':
            throw CXmlNodeException(tag[idx]);
            break;
         default:
            // OK
            break;
      }
   }
}

void CXmlNodeBase::ReplaceInvalidTagChars(IN OUT LPSTR tag)
{
   for ( int idx = 0; idx < lstrlen(tag); idx++ ) {
      switch ( tag[idx] ) {
         case '&':
         case '<':
         case '>':
         case '\"':
         case '\'':
         case ' ':
            tag[idx] = '_';
            break;
         default:
            // OK
            break;
      }
   }
}

//==============================================================================
//==============================================================================

void CXmlNodeBase::StartNode(void)
{
   m_ofs << _T("<") << m_tag.c_str();

   std::map<std::string, std::string>::iterator    it;

   for ( it = m_attribs.m_AttributeMap.begin(); it != m_attribs.m_AttributeMap.end(); it++ ) {
      std::string    strAttribValue;

      PutValue(it->second.c_str(), strAttribValue);

      m_ofs << " " << it->first.c_str() << "=\"" << strAttribValue.c_str() << "\""; 
   }

   m_ofs << _T(">") << endl; 

   if ( m_text.size() > 0 ) {
      m_ofs << m_text.c_str() << endl;
   }
   
   m_bDidStart = true;
}

void CXmlNodeBase::AssignAttributeSet(CXmlAttributeSet & ras)
{
   m_attribs = ras;
}

void  CXmlNodeBase::PutUTF8Char(int const wChar, std::string & outs)
{
   if ( wChar <= 0x7F ) {
      outs += (char)wChar;
   } else if ( wChar <= 0x07FF ) {
      outs += (char)(0xC0 | (wChar >> 6));
      outs += (char)(0x80 | (wChar & 0x3F));
   } else {
      outs += (char)(0xE0 | (wChar >> 12));
      outs += (char)(0x80 | (wChar & 0x3F));
      outs += (char)(0x80 | ((wChar >> 6) & 0x3F));
   }
}

void CXmlNodeBase::PutValue(LPCSTR value, std::string & outs) 
{
   for ( const unsigned char * szChar = (const unsigned char *)value; *szChar != '\0'; szChar++ ) {
      switch ( *szChar ) {
         case '&':
            outs += "&amp;";
            break;
         case '<':
            outs += "&lt;";
            break;
         case '>':
            outs += "&gt;";
            break;
         case '\"':
            outs += "&quot;";
            break;
         case '\'':
            outs += "&apos;";
            break;
         default:
            PutUTF8Char(szChar[0], outs);
            break;
      }
   }
}

void CXmlNodeBase::PutValue(LPCWSTR value, std::string & outs) 
{
   for ( const WCHAR * szwChar = (const WCHAR *)value; *szwChar != L'\0'; szwChar++ ) {
      switch ( *szwChar ) {
         case L'&':
            outs += "&amp;";
            break;
         case L'<':
            outs += "&lt;";
            break;
         case L'>':
            outs += "&gt;";
            break;
         case L'\"':
            outs += "&quot;";
            break;
         case L'\'':
            outs += "&apos;";
            break;
         default:
            PutUTF8Char(szwChar[0], outs);
            break;
      }
   }
}

DWORD CXmlNodeBase::AddRef(void)
{ 
   _ASSERTE((int)m_cRef >= 0);

   return ++m_cRef; 
}

DWORD CXmlNodeBase::Release(void) 
{ 
   _ASSERTE((int)m_cRef > 0);

   if (--m_cRef != 0) {
      return m_cRef; 
   }
   delete this; 
   return 0; 
}


//==============================================================================
//==============================================================================
//==============================================================================

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
CXmlNode::CXmlNode (
   ::ofstream & ofs, 
   IN LPCSTR tag, 
   IN LPCSTR value
)
 : CXmlNodeBase(ofs,tag)
{ 
   if ( value != NULL ) { 
      PutValue(value, m_text); 
   } 
}

CXmlNode::CXmlNode (
   ::ofstream & ofs, 
   IN LPCSTR tag, 
   IN LPCWSTR value
)
 : CXmlNodeBase(ofs,tag)
{ 
   if ( value != NULL ) { 
      PutValue(value, m_text); 
   } 
}

CXmlNode::CXmlNode (
   ::ofstream & ofs, 
   IN LPCSTR tag, 
   IN const int nValue
)
 : CXmlNodeBase(ofs,tag)
{ 
   TCHAR    szNumberBuf[64];
   
   wsprintf(szNumberBuf, _T("%d"), nValue);
   PutValue(szNumberBuf, m_text); 
}

CXmlNode::CXmlNode (
   ::ofstream & ofs, 
   IN LPCSTR tag, 
   IN const DWORD nValue
)
 : CXmlNodeBase(ofs,tag)
{ 
   TCHAR    szNumberBuf[64];
   
   sprintf(szNumberBuf, _T("%u"), nValue);
   PutValue(szNumberBuf, m_text); 
}

CXmlNode::CXmlNode (
   ::ofstream & ofs, 
   IN LPCSTR tag, 
   IN const DWORDLONG nValue
)
 : CXmlNodeBase(ofs,tag)
{ 
   TCHAR    szNumberBuf[64];
   
   sprintf(szNumberBuf, _T("%I64u"), nValue);
   PutValue(szNumberBuf, m_text); 
}

CXmlNode::CXmlNode (
   ::ofstream & ofs, 
   IN LPCSTR tag, 
   IN const bool bValue
)
 : CXmlNodeBase(ofs,tag)
{ 
   if ( bValue ) {
      PutValue(m_szValueTrue, m_text); 
   } else {
      PutValue(m_szValueFalse, m_text); 
   }
}

CXmlNode::CXmlNode (
   ::ofstream & ofs, 
   IN LPCSTR tag, 
   IN const double dValue
   )
 : CXmlNodeBase(ofs,tag)
{ 
   TCHAR    szNumberBuf[64];
   
   sprintf(szNumberBuf, _T("%f"), dValue);
   PutValue(szNumberBuf, m_text); 
}

CXmlNode::~CXmlNode()
{
}
