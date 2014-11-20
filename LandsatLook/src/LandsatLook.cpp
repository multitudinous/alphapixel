#include "LandsatLook.h"
#include "ConfigFile.h"

#include <gdal/gdal.h>
#include <iostream>
#include <sstream>
#include <map>
#include <fstream>

namespace landsatlook {

MTLParse::MTLParse(  std::string &MTLName ) :
m_Success(  false )
{
	m_MTLName = MTLName;

	ConfigFile cfg( m_MTLName.c_str() );

	bool exists = cfg.keyExists("REFLECTANCE_MAXIMUM_BAND_1");
	std::cout << "REFLECTANCE_MAXIMUM_BAND_1: " << std::boolalpha << exists << "\n";
	exists = cfg.keyExists("REFLECTANCE_MINIMUM_BAND_1");
	std::cout << "REFLECTANCE_MINIMUM_BAND_1: " << exists << "\n";

	std::string stringVal = cfg.getValueOfKey<std::string>("REFLECTANCE_MAXIMUM_BAND_1");
	std::cout << "value of REFLECTANCE_MAXIMUM_BAND_1 string: " << stringVal << "\n";
	double doubleVal = cfg.getValueOfKey<double>("REFLECTANCE_MAXIMUM_BAND_1");
	std::cout << "value of REFLECTANCE_MAXIMUM_BAND_1 double: " << doubleVal << "\n\n";

	stringVal = cfg.getValueOfKey<std::string>("REFLECTANCE_MINIMUM_BAND_1");
	std::cout << "value of REFLECTANCE_MINIMUM_BAND_1 string: " << stringVal << "\n";
	doubleVal = cfg.getValueOfKey<double>("REFLECTANCE_MINIMUM_BAND_1");
	std::cout << "value of REFLECTANCE_MINIMUM_BAND_1 double: " << doubleVal << "\n\n";

	std::cin.get();

	m_Success = exists;

}

bool MTLParse::ParseMTL( )
{
	bool success = false;

	return success;
}

SomeClass::SomeClass()
{
}

bool SomeClass::Operate()
{
	bool success;
	MTLParse *parse;
	std::string filename = "C:\\Data\\Dev\\AgSolver-AlphaPixel\\alphapixel\\LandsatLook\\Data\\LC80260312014079LGN00_MTL.txt";
	parse = new MTLParse( filename );
	if ( parse )
	{
		success = parse->ParseSuccess();
		delete parse;
	}
	return success;
}

int function()
{
	return 22;
}

}

