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

// C++ headers
#include <string>
#include <map>
#include <vector>
// GDAL header
#include <gdal/gdal_priv.h>
// Project specific header
#include <ConfigFile.h>

// Library export definitions
#if defined(_MSC_VER) || defined(__CYGWIN__) || defined(__MINGW32__) || defined( __BCPLUSPLUS__)  || defined( __MWERKS__)
    #if defined( LandsatLook_EXPORTS )
    #	define LANDSATLOOK_EXPORT __declspec(dllexport)
    #else
    #	define LANDSATLOOK_EXPORT __declspec(dllimport)
    #endif
#else
    #define LANDSATLOOK_EXPORT
#endif

/*!\namespace landsatlook
 * The landsatlook namespace encompasses the entire library.
 */
namespace landsatlook {

/*!\file LandsatLook.h
 * MTLParse class API.
 * \class landsatlook::MTLParse LandsatLook.h src/LandsatLook.h
 * Class that contains ...
 */
class MTLParse
{
public:
	///Constructor
	///\param MTLName The ...
	MTLParse( const std::string &MTLName );
	///Destructor
	~MTLParse();
	///
	///\return True ...
	bool ParseSuccess( ) const	{ return m_Success; };
	///
	///\param ...
	///\param ...
	///\return True ...
	bool FindKey( const std::string &key, double &keyVal ) const;
	///
	///\param ...
	///\param ...
	///\return True ...
	bool FindKey( const std::string &key, int &keyVal ) const;

private:
	///
	ConfigFile *m_Config;
	///
	bool m_Success;
	///
	std::string m_MTLName;

};

/*!\file LandsatLook.h
 * MetaDataOwner class API.
 * \class landsatlook::MetaDataOwner LandsatLook.h src/LandsatLook.h
 * Class that contains ...
 */
class MetaDataOwner
{
public:
	///Constructor
	///\param band The ...
	MetaDataOwner( int band) : m_Band( band ) {};
	///
	///\return ...
	int getBandNumber()	const { return m_Band; };

	///
	std::map< std::string, double > m_KeyValues;
	///
	int m_Band;
};

// Degrees to radians conversion for Top of atmosphere sun angle correction
#define DegToRad ( 3.14159265 / 180.0 )

/*!\file LandsatLook.h
 * TopOfAtmosphere class API.
 * \class landsatlook::TopOfAtmosphere LandsatLook.h src/LandsatLook.h
 * Class that contains ...
 */
class TopOfAtmosphere : public MetaDataOwner
{
public:
	///Constructor
	///\param config The ...
	///\param band The ...
	TopOfAtmosphere( const MTLParse *config, int band );
	///
	///\param uncorrectedNumber 
	///\return ...
	double ComputeTOAReflectance( double uncorrectedNumber ) const;

private:
	///
	double m_SunElevation;
	///
	double m_Multiplier;
	///
	double m_BaseOffset;
};

/*!\file LandsatLook.h
 * NormalizedIndex class API.
 * \class landsatlook::NormalizedIndex LandsatLook.h src/LandsatLook.h
 * Class that contains ...
 */
class NormalizedIndex
{
public:
	///Constructor
	///\param bands The ...
	///\param reqBands The ...
	NormalizedIndex( std::map< int, double > &bands, const int *reqBands );
	///
	///\param bands 
	///\return ...
	double ComputeIndex( const std::map< int, double > &bands ) const;
	///
	///\param reflectance1 
	///\param reflectance2 
	///\return ...
	double ComputeIndex( double reflectance1, double reflectance2 ) const;

	///
	std::vector< int > m_reqBands;
};

/*!\file LandsatLook.h
 * NDVI class API.
 * \class landsatlook::NDVI LandsatLook.h src/LandsatLook.h
 * Class that contains ...
 */
class NDVI : public NormalizedIndex
{
public:
	///Constructor
	///\param bands The ...
	NDVI( std::map< int, double > &bands );

};

/*!\file LandsatLook.h
 * NDTI class API.
 * \class landsatlook::NDTI LandsatLook.h src/LandsatLook.h
 * Class that contains ...
 */
class NDTI : public NormalizedIndex
{
public:
	///Constructor
	///\param bands The ...
	NDTI( std::map< int, double > &bands );

};

// Definitions pertaining to the data types in Landsat image bands
#define INPUT_SIZE unsigned short int
// GDAL define for a unsigned short integer
#define INPUT_RASTER_TYPE GDT_UInt16

/*!\file LandsatLook.h
 * ImageBandData class API.
 * \class landsatlook::ImageBandData LandsatLook.h src/LandsatLook.h
 * Class that contains ...
 */
class ImageBandData
{
public:
	///Constructor
	ImageBandData();
	///Constructor
	///\param gdalData The ...
	///\param oneDataLine The ...
	///\param toa The ...
	ImageBandData( GDALDataset *gdalData, INPUT_SIZE *oneDataLine, TopOfAtmosphere *toa );
	///Destructor
	~ImageBandData();
	///
	///\param gdalData The ...
	void SetGDALData( GDALDataset *gdalData );
	///
	///\param oneDataLine The ...
	void SetOneDataLine( INPUT_SIZE *oneDataLine ) { m_OneDataLine = oneDataLine; };
	///
	///\param toa The ...
	void SetTOA( TopOfAtmosphere *toa ) { m_toa = toa; };
	///
	///\param lineNumber The ...
	///\param startCol The ...
	///\param numCols The ...
	///\return True ...
	bool RetrieveOneDataLine( int lineNumber, int startCol, int numCols );
	///
	///\param pixelNumber The ...
	///\return ... 
	double TOACorrectOnePixel( int pixelNumber );

	///
	GDALDataset *m_GdalData;
	///
	GDALRasterBand *m_GdalBand;
	///
	INPUT_SIZE *m_OneDataLine;
	///
	TopOfAtmosphere *m_toa;
};

// Typedef ImageBandMap for storing the data used in band-specific calculations
typedef std::map< int, ImageBandData * > ImageBandMap;

/*!\file LandsatLook.h
 * LandsatLook class API.
 * \class landsatlook::LandsatLook LandsatLook.h src/LandsatLook.h
 * Class that contains ...
 */
class LANDSATLOOK_EXPORT LandsatLook {
public:
	LandsatLook( bool exportNDVI, bool exportNDTI, const std::string &landsatPath, const std::string &landsatFileRoot,
		const double &ULBoundsX, const double &ULBoundsY, const double &LRBoundsX, const double &LRBoundsY );
	///
	void InitGDAL() const;
	///
	///\param parse The ...
	///\return True ... 
	GDALDataset* OpenLandsatBand( const std::string &landsatBandFileRoot, int bandNumber ) const;
	///
	///\param config The ...
	///\param ULX The ...
	///\param ULY The ...
	///\param LRX The ...
	///\param LRY The ...
	///\param startCol The ...
	///\param startRow The ...
	///\param cols The ...
	///\param rows The ...
	///\return True ... 
	bool ComputeFarmCellBounds( const MTLParse *config, double &ULX, double &ULY, double &LRX, double &LRY, int &startCol, int &startRow, int &cols, int &rows ) const;
	///
	///\param targetDataset The ...
	///\param imageBands The ...
	///\param startCol The ...
	///\param startRow The ...
	void SetGeoRefFromLandsat( GDALDataset *targetDataset, const ImageBandMap &imageBands, int startCol, int startRow ) const;
	///
	///\return True ... 
	bool Operate();
	///
	///\param parse The ...
	///\return True ... 
	bool ExportNDVI( MTLParse *parse );
	///
	///\param parse The ...
	///\return True ... 
	bool ExportNDTI( MTLParse *parse );

	///
	bool m_exportNDVI;
	///
	bool m_exportNDTI;
	///
	std::string m_landsatPath;
	///
	std::string m_landsatFileRoot;
	///
	int m_ULCellX;
	///
	int m_ULCellY;
	///
	int m_LRCellX;
	///
	int m_LRCellY;
	///
	double m_ULBoundsX;
	///
	double m_ULBoundsY;
	///
	double m_LRBoundsX;
	///
	double m_LRBoundsY;

};

int LANDSATLOOK_EXPORT function();

}

#endif

