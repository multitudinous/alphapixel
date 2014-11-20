//! \file
//! \brief Main doxygen docs; also defines compiler signature exports.

#ifndef LANDSATLOOK_EXPORT_
#define LANDSATLOOK_EXPORT_

//! \mainpage LandsatLook Documentation
//!
//! \section intro Introduction
//!
//! Lorem ipsum dolor...
//!
//! \section usage A Quick Example
//!
//! \include docs/demo.cpp

#include <string>
#include <map>

#if defined(_MSC_VER) || defined(__CYGWIN__) || defined(__MINGW32__) || defined( __BCPLUSPLUS__)  || defined( __MWERKS__)
    #if defined( LandsatLook_EXPORTS )
    #	define LANDSATLOOK_EXPORT __declspec(dllexport)
    #else
    #	define LANDSATLOOK_EXPORT __declspec(dllimport)
    #endif
#else
    #define LANDSATLOOK_EXPORT
#endif

namespace landsatlook {


class MTLParse
{
public:
	MTLParse( std::string &MTLName );
	bool ParseSuccess( )	{ return m_Success; };

private:
	bool ParseMTL();
	bool m_Success;
	std::string m_MTLName;
	std::map<std::string, std::string> m_Contents;

};

class LANDSATLOOK_EXPORT SomeClass {
public:
	SomeClass();
	bool Operate();
};

int LANDSATLOOK_EXPORT function();

}

#endif

