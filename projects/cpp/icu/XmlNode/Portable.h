#ifndef __PORTABLE_H__
#define __PORTABLE_H__

#if defined(_MSC_VER)
#define MSC_LANG
#else
#error >>> NOT PREPARED FOR COMPILERS OTHER THAN MSVC <<<
#endif

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#if (!defined(__STDAFX_H__) && !defined(__STDWIN_H__)) || (defined(__STDAFX_H__) && defined(__STDWIN_H__))
//#error Must include either 'stdafx.h' or 'stdwin.h' before including this file for PCH
#endif

#if defined(WIN32) && !defined(_WIN32)
#define _WIN32    1
#endif

#if defined(_WINDOWS) || defined(_Windows)
#define WINDOWS
#endif

#include "cmnhdr.h"
#include <crtdbg.h>
#include <tchar.h>


/* undefine previous possible declaration */
#ifdef HUGE
#undef HUGE
#endif

#ifdef EXPORT
#undef EXPORT
#endif

#ifdef _WIN32
   #define EXPORT __declspec(dllexport)
   #define HUGE
   #define HMEMCPY(a,b,c) memcpy(a,b,c)
#else
   #define EXPORT __export
   #define HUGE _huge
   #define HMEMCPY(a,b,c) hmemcpy(a,b,c)
#endif

typedef double    * LPDOUBLE;

#if !defined(_WIN32) || !defined (__cplusplus)
typedef unsigned char bool;
#endif


static const TCHAR   PATH_SEPARATOR_CHAR = _T('\\');


#endif /* _PORTABLE_H */
