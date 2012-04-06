/******************************************************************************
Module name: CmnHdr.h
Written by: Jeffrey Richter
Notices: Copyright (c) 1995-1997 Jeffrey Richter
Changes and updates: Petko Popov, 1998-2000
Purpose: Common header file containing handy macros and definitions used
         throughout all the applications in the book.
******************************************************************************/
#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

/* Disable ridiculous warnings so that the code */
/* compiles cleanly using warning level 4.      */

/* nonstandard extension 'single line comment' was used */
#pragma warning(disable: 4001)

// indirection to slightly different base types
#pragma warning(disable: 4057)

// unreferenced formal parameter
#pragma warning(disable: 4100)

// named type definition in parentheses
#pragma warning(disable: 4115)

// nonstandard extension used : nameless struct/union
#pragma warning(disable: 4201)

// nonstandard extension used : benign typedef redefinition
#pragma warning(disable: 4209)

// nonstandard extension used : bit field types other than int
#pragma warning(disable: 4214)

// unreferenced inline function has been removed
#pragma warning(disable: 4514)

// C++ language change: to explicitly specialize class template '%s' use the following syntax:
#pragma warning(disable: 4663)

// Note: Creating precompiled header
#pragma warning(disable: 4699)

// Identifier was truncated to '255' characters in the debug information
#pragma warning(disable: 4786)

//--------------------------------------------------------------------------------------------------
// Do not tolerate the following L4 warnings:

// 'function' : different types for formal and actual parameter 'number'
#pragma warning( error: 4024 )

// formal parameter 'number' different from declaration
#pragma warning( error: 4028 )

// formal parameter 'number' has different type when promoted
#pragma warning( error: 4032 )

// 'identifier1' : 'operator' : different levels of indirection from 'identifier2'
#pragma warning( error: 4047 )

//enumerate 'identifier' in switch of enum 'identifier' is not explicitly handled by a case label
#pragma warning( error: 4061 )

// unreferenced formal parameter
//#pragma warning(disable: 4100)

// unreferenced local variable
//#pragma warning(disable: 4101)

// alignment of a member was sensitive to packing
#pragma warning( error: 4121 )

// logical operation on address of string constant
#pragma warning( error: 4130 )

// const object should be initialized
#pragma warning( error: 4132 )

// An attempt was made to subtract two pointers of different types. (?!)
#pragma warning( error: 4133 )

// local variable is initialized but not referenced
//#pragma warning( error: 4189 )

// nonstandard extension used : float long
#pragma warning( error: 4216 )

// nonstandard extension used : 'identifier' : cannot be initialized using address of automatic variable
#pragma warning( error: 4221 )

// cast truncates constant value
#pragma warning( error: 4310 )

// '==' : operator has no effect; did you intend '='?
#pragma warning( error: 4553 )

// local variable 'identifier' used without having been initialized
#pragma warning( error: 4700 )

// local variable 'identifier' may be used without having been initialized
//#pragma warning( error: 4701 )

// assignment within conditional expression
#pragma warning( error: 4706 )

// not all control paths return a value
#pragma warning( error: 4715 )

//--------------------------------------------------------------------------------------------------


//////////////////////// Windows Version Build Option /////////////////////////


#define _WIN32_WINNT 0x0400


///////////////////////////// STRICT Build Option /////////////////////////////


// Force all EXEs/DLLs to use STRICT type checking.
#if !defined(NO_STRICT) && !defined(STRICT)
#define STRICT 1
#endif


/////////////////////////// CPU Portability Macros ////////////////////////////


// If no CPU platform was specified, default to the current platform.
#if !defined(_PPC_) && !defined(_ALPHA_) && !defined(_MIPS_) && !defined(_X86_)
   #if defined(_M_IX86)
      #define _X86_
   #endif
   #if defined(_M_MRX000)
      #define _MIPS_
   #endif
   #if defined(_M_ALPHA)
      #define _ALPHA_
   #endif
   #if defined(_M_PPC)
      #define _PPC_
   #endif
#endif


//////////////////////////// Unicode Build Option /////////////////////////////


// If we are not compiling for an x86 CPU, we always compile using Unicode.
#ifndef _X86_
#define UNICODE
#endif


// To compile using Unicode on the x86 CPU, uncomment the line below.
//#define UNICODE

// When using Unicode Win32 functions, use Unicode C-Runtime functions too.
#ifdef UNICODE
#define _UNICODE
#endif


//////////////////////////////// chDIMOF Macro ////////////////////////////////


// This macro evaluates to the number of elements in an array.
#define chDIMOF(Array) (sizeof(Array) / sizeof(Array[0]))


///////////////////////////// chBEGINTHREADEX Macro ///////////////////////////


// Create a chBEGINTHREADEX macro that calls the C run-time's
// _beginthreadex function. The C run-time library doesn't
// want to have any reliance on Win32 data types such as
// HANDLE. This means that a Win32 programmer needs to cast
// the return value to a HANDLE. This is terribly inconvenient,
// so I have created this macro to perform the casting.
typedef unsigned (__stdcall *PTHREAD_START) (void *);

#define chBEGINTHREADEX(lpsa, cbStack, lpStartAddr, \
   lpvThreadParm, fdwCreate, lpIDThread)            \
      ((HANDLE)_beginthreadex(                      \
         (void *) (lpsa),                           \
         (unsigned) (cbStack),                      \
         (PTHREAD_START) (lpStartAddr),             \
         (void *) (lpvThreadParm),                  \
         (unsigned) (fdwCreate),                    \
         (unsigned *) (lpIDThread)))


//////////////////////////// Assert/Verify Macros /////////////////////////////


#define chFAIL(szMSG) {                                                   \
      MessageBox(GetActiveWindow(), szMSG,                                \
         __TEXT("Assertion Failed"), MB_OK | MB_ICONERROR);               \
      DebugBreak();                                                       \
   }

// Put up an assertion failure message box.
#define chASSERTFAIL(file,line,expr) {                                    \
      TCHAR sz[128];                                                      \
      wsprintf(sz, __TEXT("File %hs, line %d : %hs"), file, line, expr);  \
      chFAIL(sz);                                                         \
   }

// Put up a message box if an assertion fails in a debug build.
#ifdef _DEBUG
#define chASSERT(x) if (!(x)) chASSERTFAIL(__FILE__, __LINE__, #x)
#else
#define chASSERT(x)
#endif

// Assert in debug builds, but don't remove the code in retail builds.
#ifdef _DEBUG
#define chVERIFY(x) chASSERT(x)
#else
#define chVERIFY(x) (x)
#endif


/////////////////////////// chHANDLE_DLGMSG Macro /////////////////////////////


// The normal HANDLE_MSG macro in WINDOWSX.H does not work properly for dialog
// boxes because DlgProc return a BOOL instead of an LRESULT (like
// WndProcs). This chHANDLE_DLGMSG macro corrects the problem:
#define chHANDLE_DLGMSG(hwnd, message, fn)                           \
   case (message): return (SetDlgMsgResult(hwnd, uMsg,               \
      HANDLE_##message((hwnd), (wParam), (lParam), (fn))))


/////////////////////////// Quick MessageBox Macro ////////////////////////////


#define chMB(s) {                                                    \
      TCHAR szTMP[128];                                              \
      GetModuleFileName(NULL, szTMP, chDIMOF(szTMP));                \
      MessageBox(GetActiveWindow(), s, szTMP, MB_OK);                \
   }


///////////////////////////// Zero Variable Macro /////////////////////////////


// Zero out a structure. If fInitSize is TRUE, initialize the first int to
// the size of the structure. Many structures like WNDCLASSEX and STARTUPINFO
// require that their first member be set to the size of the structure itself.
#define chINITSTRUCT(structure, fInitSize)                           \
   (ZeroMemory(&(structure), sizeof(structure)),                     \
   fInitSize ? (*(int*) &(structure) = sizeof(structure)) : 0)


//////////////////////// Dialog Box Icon Setting Macro ////////////////////////


// The call to SetClassLong is for Windows NT 3.51 or less.  The WM_SETICON
// messages are for Windows NT 4.0 and Windows 95.
#define chSETDLGICONS(hwnd, idiLarge, idiSmall)                               \
   {                                                                          \
      OSVERSIONINFO VerInfo;                                                  \
      chINITSTRUCT(VerInfo, TRUE);                                            \
      GetVersionEx(&VerInfo);                                                 \
      if ((VerInfo.dwPlatformId == VER_PLATFORM_WIN32_NT) &&                  \
          (VerInfo.dwMajorVersion <= 3 && VerInfo.dwMinorVersion <= 51)) {    \
         SetClassLong(hwnd, GCL_HICON, (LONG)                                 \
            LoadIcon(GetWindowInstance(hwnd), MAKEINTRESOURCE(idiLarge)));    \
      } else {                                                                \
         SendMessage(hwnd, WM_SETICON, TRUE,  (LPARAM)                        \
            LoadIcon(GetWindowInstance(hwnd), MAKEINTRESOURCE(idiLarge)));    \
         SendMessage(hwnd, WM_SETICON, FALSE, (LPARAM)                        \
            LoadIcon(GetWindowInstance(hwnd), MAKEINTRESOURCE(idiSmall)));    \
      }                                                                       \
   }


///////////////////////////// UNICODE Check Macro /////////////////////////////


#ifdef UNICODE

#define chWARNIFUNICODEUNDERWIN95()                                        \
   if (GetWindowsDirectoryW(NULL, 0) <= 0)                                 \
      MessageBoxA(NULL, "This operating system doesn't support Unicode.",  \
         NULL, MB_OK)

#else

#define chWARNIFUNICODEUNDERWIN95()

#endif


///////////////////////// Pragma message helper macro /////////////////////////

/*
When the compiler sees a line like this:
#pragma chMSG(Fix this later)

it outputs a line like this:
C:\Document\AdvWin\Code\Sysinfo.06\..\CmnHdr.H(296):Fix this later

You can easily jump directly to this line and examine the surrounding code.
*/
#define chSTR(x)     #x
#define chSTR2(x) chSTR(x)
#define chMSG(desc) message(__FILE__ "(" chSTR2(__LINE__) "): " #desc)


#if !defined (PURE)
#define PURE =0
#endif

////////////////////// Macro that defines Windows error codes /////////////////

// ERROR CODEDS
//   SEV is the severity code:
//       00 - Success
//       01 - Informational
//       10 - Warning
//       11 - Error
//   FAC is facility code
//   CODE is the error code
#define MAKE_ERROR_CODE(SEV,FAC,CODE) ((DWORD)( ((SEV & 0x3) << 30) | 0x20000000 | ((FAC & 0x0FFF) << 16) | ((CODE & 0xFFFF)) ))



///////////////////////////////// End of File /////////////////////////////////

template <class T> class SmartPtr {
    T* p;
public:
    SmartPtr(T* p_=NULL) : p(p_) { if (p) p->AddRef(); }
    ~SmartPtr(void)
    { 
       if (p) {
          p->Release(); 
       }
    }
    operator T*(void) { return p; }
    T& operator*(void) { return *p; } // can throw exception
    T* operator->(void) { return p; }
    SmartPtr& operator=(SmartPtr<T> &p_)
        {return operator=((T *) p_);}
    SmartPtr& operator=(T* p_) {
       if (p) 
          p->Release(); 
       p = p_; 
       if (p) 
          p->AddRef();
       return *this;
    }
};

///////////////////////////////// End of File /////////////////////////////////
