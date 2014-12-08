//! \file
//! \brief Main doxygen docs; also defines compiler signature exports.

#ifndef LANDSATLOOK_EXPORT_
#define LANDSATLOOK_EXPORT_

//! \mainpage LandsatLook Documentation
//!
//! \section intro Introduction
//!
//! LandsatLook is a library that creates vegetation or tillage index maps
//! that go by their acronyms NDVI and NDTI - Normalized Difference Vegetation Index
//! and Normalized Difference Tillage Index. They are used for evaluating agricultural
//! characteristics from Landsat imagery.
//! 
//! The library takes as input a set of rectangular geographic bounds in UTM projection
//! and a Landsat scene's set of images that have a common metadata file with
//! an MTL extension. The scene images must also be in UTM and at least partially overlap 
//! the AOI in order to complete successfully.
//! 
//! The user tells the library what outputs are desired. Either or both indices can
//! be generated for the area of interest and they will be output as GeoTiff images 
//! in the default folder.
//! 
//! NDVI and NDTI have a common formula composed of input from bands of the
//! Landsat imagery but they use different bands. In order to arrive at the values
//! called for by the equations a conversion must be done on the raw band data which
//! comes as unsigned short integers and must be rescaled to the values known as top 
//! of atmosphere reflectance corrected for sun angle at the center of the Landsat 
//! scene at the time of recording.
//! 
//! The main class in the library that is publicly exposed is class LandsatLook.
//! Once instantiated in the user's application with an area of interest and paths
//! to the Landsat data and directives as to which output indices to generate,
//! the user calls Operate() to create the output index images.
//! 
//! If errors are encountered messages will be posted to the console and the 
//! library routine will exit.
//!
//! LandsatLook makes use of a separate class to parse the metadata file. It is found in
//! ConfigFile.h and ConfigFile.cpp. Authorship of the parser is found in 
//! its header file.
//!
//! \section usage A Quick Example
//!
//! \include docs/test.cpp

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
 * Class that contains tools for extracting all the tags and values in the Landsat
 * MTL metadata file.
 */
class MTLParse
{
public:
	///Constructor
	///\param MTLName The string containing the full path and file name of the MTL metadata file.
	MTLParse( const std::string &MTLName );
	///Destructor
	~MTLParse();
	///Tells if the MTL file has been parsed correctly.
	///\return True when the the MTL file has been parsed successfully and is ready for queries.
	bool ParseSuccess( ) const	{ return m_Success; };
	///Retrieves value associated with a key of floating point value type.
	///\param key A string containing the item to be searched for in the metadata.
	///\param keyVal A double for storing the value associated with the key.
	///\return True if the key and value were found successfully.
	bool FindKey( const std::string &key, double &keyVal ) const;
	///Retrieves value associated with a key of integer value type.
	///\param key A string containing the item to be searched for in the metadata.
	///\param keyVal An integer for storing the value associated with the key.
	///\return True if the key and value were found successfully.
	bool FindKey( const std::string &key, int &keyVal ) const;

private:
	///A ConfigFile class pointer. ConfigFile opens and parses the Landsat metadata file
	///and returns values associated with requested keys.
	ConfigFile *m_Config;
	///True when the the MTL file has been parsed successfully and is ready for queries.
	bool m_Success;
	///The string containing the full path and file name of the MTL metadata file.
	std::string m_MTLName;

};

/*!\file LandsatLook.h
 * MetaDataOwner class API.
 * \class landsatlook::MetaDataOwner LandsatLook.h src/LandsatLook.h
 * Class that contains basic elements that might be used by more than one derived class
 * (though currently it's not) to store information about the band and values
 * useful to the derived class for making its calculations.
 */
class MetaDataOwner
{
public:
	///Constructor
	///\param band The Landsat band number. Valid range is 1-11.
	MetaDataOwner( int band) : m_Band( band ) {};
	///Fetches the Landsat band number, m_Band.
	///\return The Landsat band number, m_Band. Valid range is 1-11.
	int getBandNumber()	const { return m_Band; };

	///Map containing the keys for values of interest to the derived class
	///along with the values associated with those keys for the lansat band
	///found in m_Band.
	std::map< std::string, double > m_KeyValues;
	///The Landsat band number. Valid range is 1-11.
	int m_Band;
};

// Degrees to radians conversion for Top of atmosphere sun angle correction
#define DegToRad ( 3.14159265 / 180.0 )

/*!\file LandsatLook.h
 * TopOfAtmosphere class API.
 * \class landsatlook::TopOfAtmosphere LandsatLook.h src/LandsatLook.h
 * Class that contains specific methods and variables for computing the
 *Top of Atmosphere correction and scaling necessary to translate the
 *digital numbers stored in landsat band rasters into reflectance values.
 */
class TopOfAtmosphere : public MetaDataOwner
{
public:
	///Constructor
	///\param config An MTLParse pointer that was used to examine the MTL
	///metadata file.
	///\param band The Landsat band number that will be associated with this TopOfAtmosphere object.
	TopOfAtmosphere( const MTLParse *config, int band );
	///Converts the digital number from the raw landsat band into a top of atmosphere
	///corrected reflectance value, complete with sun angle correction.
	///\param uncorrectedNumber The digital number from the Landsat band in need of correction.
	///\return Corrected number returned as reflectance.
	double ComputeTOAReflectance( double uncorrectedNumber ) const;

private:
	///The sun elevation angle at the center of the Landsat scene. Units: radians.
	double m_SunElevation;
	///The mulplicative factor used in calculating reflectance from the digital number.
	double m_Multiplier;
	///The additive value used in calculating reflectance from the digital number.
	double m_BaseOffset;
};

/*!\file LandsatLook.h
 * NormalizedIndex class API.
 * \class landsatlook::NormalizedIndex LandsatLook.h src/LandsatLook.h
 * Class that contains a list of Landsat band numbers and methods
 * in common between all derived classes of index.
 */
class NormalizedIndex
{
public:
	///Constructor
	///\param bands The map to which this object's required list of band numbers will be added.
	///\param reqBands The list of band numbers required for calculation of the derived index.
	NormalizedIndex( std::map< int, double > &bands, const int *reqBands );
	///Extracts the required values from the map and calls the second version of the method
	///to perform the actual index calculation.
	///\param bands A map keyed on band numbers with the values for each band in the second value.
	///\return The computed index value.
	double ComputeIndex( const std::map< int, double > &bands ) const;
	///The method that invokes the actual index computation formula. Formulas are of the
	///form i = (r1 - r2) / (r1 + r2)
	///\param reflectance1 The first reflectance parameter used in the index calculation.
	///\param reflectance2 The second reflectance parameter used in the index calculation.
	///\return The computed index value.
	double ComputeIndex( double reflectance1, double reflectance2 ) const;

	///A list of the band numbers required in the index computation.
	std::vector< int > m_reqBands;
};

/*!\file LandsatLook.h
 * NDVI class API.
 * \class landsatlook::NDVI LandsatLook.h src/LandsatLook.h
 * Class that contains knowledge of the band numbers required for the NDVI
 * index computation.
 */
class NDVI : public NormalizedIndex
{
public:
	///Constructor
	///\param bands The map which will be passed to the base class constructor for
	///addition of this index's required band numbers if not already in the list.
	NDVI( std::map< int, double > &bands );

};

/*!\file LandsatLook.h
 * NDTI class API.
 * \class landsatlook::NDTI LandsatLook.h src/LandsatLook.h
 * Class that contains knowledge of the band numbers required for the NDTI
 * index computation.
 */
class NDTI : public NormalizedIndex
{
public:
	///Constructor
	///\param bands The map which will be passed to the base class constructor for
	///addition of this index's required band numbers if not already in the list.
	NDTI( std::map< int, double > &bands );

};

// Definitions pertaining to the data types in Landsat image bands
#define INPUT_SIZE unsigned short int
// GDAL define for a unsigned short integer
#define INPUT_RASTER_TYPE GDT_UInt16

/*!\file LandsatLook.h
 * ImageBandData class API.
 * \class landsatlook::ImageBandData LandsatLook.h src/LandsatLook.h
 * Class that contains all the elements pertenant to a particular Landsat raster band
 * that will be used in the computation of the indices that rely on it.
 */
class ImageBandData
{
public:
	///Constructor
	ImageBandData();
	///Constructor
	///\param gdalData The GDALDataset that has been opened for reading the raster band.
	///\param oneDataLine An array of variables large enough to hold one raster line from the image band
	///of a width equal to the number of columns that cover the area of interest. INPUT_SIZE is defined
	///for Landsat imagery as unsigned short int (see above).
	///\param toa The TopOfAtmosphere class object that will be used to make reflectance values out
	///of the digital numbers stored in m_oneDataLine.
	ImageBandData( GDALDataset *gdalData, INPUT_SIZE *oneDataLine, TopOfAtmosphere *toa );
	///Destructor
	~ImageBandData();
	///Method for setting only the m_GDALDataset member. m_GdalBand is derived from m_GDALDataset.
	///\param gdalData The GDALDataset that has been opened for reading the raster band.
	void SetGDALData( GDALDataset *gdalData );
	///Method for setting only the m_OneDataLine member.
	///\param oneDataLine An array of variables large enough to hold one raster line from the image band
	///of a width equal to the number of columns that cover the area of interest. INPUT_SIZE is defined
	///for Landsat imagery as unsigned short int (see above).
	void SetOneDataLine( INPUT_SIZE *oneDataLine ) { m_OneDataLine = oneDataLine; };
	///Method for setting only the m_toa member.
	///\param toa The TopOfAtmosphere class object that will be used to make reflectance values out
	///of the digital numbers stored in m_oneDataLine.
	void SetTOA( TopOfAtmosphere *toa ) { m_toa = toa; };
	///Method to copy one line of data from m_GdalBand to m_OneDataLine.
	///\param lineNumber The row or y position of raster data from which to copy.
	///\param startCol The column or x position in the row to begin copying.
	///\param numCols The number of columns or x entries to copy.
	///\return True if data was copied successfully.
	bool RetrieveOneDataLine( int lineNumber, int startCol, int numCols );
	///Perform the top of atmosphere correction on a pixel's digital number to determine the reflectance
	///value also corrected for sun angle.
	///\param pixelNumber The pixel location within the m_OneDataLine array for which computation is to be performed.
	///\return The reflectance value for the given pixel, also corrected for sun angle.
	double TOACorrectOnePixel( int pixelNumber );

	///The GDALDataset that has been opened for reading the raster band.
	GDALDataset *m_GdalData;
	///The GDALRasterBand within m_GdalData that contains the raster band.
	GDALRasterBand *m_GdalBand;
	///An array of variables large enough to hold one raster line from the image band
	///of a width equal to the number of columns that cover the area of interest. INPUT_SIZE is defined
	///for Landsat imagery as unsigned short int (see above).
	INPUT_SIZE *m_OneDataLine;
	///The TopOfAtmosphere class object that will be used to make reflectance values out
	///of the digital numbers stored in m_oneDataLine.
	TopOfAtmosphere *m_toa;
};

// Typedef ImageBandMap for storing the data used in band-specific calculations
typedef std::map< int, ImageBandData * > ImageBandMap;

/*!\file LandsatLook.h
 * LandsatLook class API.
 * \class landsatlook::LandsatLook LandsatLook.h src/LandsatLook.h
 * Class that contains methods and vaiables for computing and outputting maps of either NDVI or NDTI indices
 * over an area of interest specified by the user and taking as input a openable Landsat scene with
 * the appropriate bands and metadata to facillitate the calculations. this is the class that must be created in
 * user applications and hence it is the one that is exported from the LandsatLook library.
 */
class LANDSATLOOK_EXPORT LandsatLook {
public:
	///Constructor.
	///\param exportNDVI True to build and write an NDVI image in GeoTiff format covering the area of interest.
	///\param exportNDTI True to build and write an NDTI image in GeoTiff format covering the area of interest.
	///\param landsatPath The file path at which will be found the Landsat files. Either a complete
	//path or path relative to the working folder. It should end in "/" or "\".
	///\param landsatFileRoot The root file name of the Landsat scene which should end in "_".
	///\param ULBoundsX The upper left corner x coordinate of the area of interest. Units same as Landsat scene imagery.
	///\param ULBoundsY The upper left corner y coordinate of the area of interest. Units same as Landsat scene imagery.
	///\param LRBoundsX The lower right corner x coordinate of the area of interest. Units same as Landsat scene imagery.
	///\param LRBoundsY The lower right corner y coordinate of the area of interest. Units same as Landsat scene imagery.
	LandsatLook( bool exportNDVI, bool exportNDTI, const std::string &landsatPath, const std::string &landsatFileRoot,
		const double &ULBoundsX, const double &ULBoundsY, const double &LRBoundsX, const double &LRBoundsY );
	///The method that calls the methods that make the indices and image outputs.
	///\return True if all requested outputs are created and saved successfully.
	bool Operate();

private:
	///Creates an NDVI map and outputs it as a GeoTiff image.
	///\param parse The The MTLParse object containing the metadata for the landsat imagery.
	///\return True if the NDVI map is created and saved successfully.
	bool ExportNDVI( MTLParse *parse );
	///Creates an NDTI map and outputs it as a GeoTiff image.
	///\param parse The MTLParse object containing the metadata for the landsat imagery.
	///\return True if the NDTI map is created and saved successfully.
	bool ExportNDTI( MTLParse *parse );
	///Initialize GDAL for georeferenced raster reading and writing.
	void InitGDAL() const;
	///Open a particular Landsat image band file.
	///\param landsatBandFileRoot A string containing the root name of the Landsat band imagery. It should contain the file path as well as the name and end with "_B".
	///\param bandNumber The raster band number for the file to be opened. Valid values are 1-11.
	///\return An active GDALDataset with the raster band image loaded. NULL if opening failed.
	GDALDataset* OpenLandsatBand( const std::string &landsatBandFileRoot, int bandNumber ) const;
	///Compute the range of cells in the Landsat imagery that overlies the area of interest.
	///\param config The MTLParse object which has opened and parsed the metadata file for the Landsat scene.
	///\param ULX The upper left corner x coordinate of the area of interest. Units same as Landsat scene imagery.
	///\param ULY The upper left corner y coordinate of the area of interest. Units same as Landsat scene imagery.
	///\param LRX The lower right corner x coordinate of the area of interest. Units same as Landsat scene imagery.
	///\param LRY The lower right corner y coordinate of the area of interest. Units same as Landsat scene imagery.
	///\param startCol The column or x cell at which the area of interest begins within the landsat scene image files.
	///startCol will be computed in this method.
	///\param startRow The row or y cell at which the area of interest begins within the landsat scene image files.
	///startRow will be computed in this method.
	///\param cols The number of columns or x cells required to cover the area of interest.
	///cols will be computed in this method.
	///\param rows The number of rows or y cells required to cover the area of interest.
	///rows will be computed in this method.
	///\return True if the area of interest at least partially overlies the Landsat imagery and the rows and cols have been computed.
	bool ComputeFarmCellBounds( const MTLParse *config, double &ULX, double &ULY, double &LRX, double &LRY, int &startCol, int &startRow, int &cols, int &rows ) const;
	///Assigns georeferencing to targetDataset, a GDALDataset created for output of NDVI or NDTI imagery, based on the projection of the input
	///Landsat imagery.
	///\param targetDataset The GDALDataset created for output of NDVI or NDTI imagery and for which georeferencing will be computed and attached.
	///\param imageBands An ImageBandMap containing at least one GDALDataset from which can be obtained reference WKT and transform parameters.
	///\param startCol The upper left column or x cell of the area of interest, relative to the original GDALDataset found in imageBands.
	///\param startRow The upper left row or y cell of the area of interest, relative to the original GDALDataset found in imageBands.
	void SetGeoRefFromLandsat( GDALDataset *targetDataset, const ImageBandMap &imageBands, int startCol, int startRow ) const;

	///True to build and write an NDVI image in GeoTiff format covering the area of interest.
	bool m_exportNDVI;
	///True to build and write an NDTI image in GeoTiff format covering the area of interest.
	bool m_exportNDTI;
	///The file path at which will be found the Landsat files. Either a complete
	//path or path relative to the working folder. It should end in "/" or "\".
	std::string m_landsatPath;
	///The root file name of the Landsat scene which should end in "_".
	std::string m_landsatFileRoot;
	///The upper left column or x cell in the Landsat scene covering the area of interest.
	int m_ULCellX;
	///The upper left row or y cell in the Landsat scene covering the area of interest.
	int m_ULCellY;
	///The lower right column or x cell in the Landsat scene covering the area of interest.
	int m_LRCellX;
	///The lower right row or y cell in the Landsat scene covering the area of interest.
	int m_LRCellY;
	///The upper left corner x coordinate of the area of interest. Units same as Landsat scene imagery.
	double m_ULBoundsX;
	///The upper left corner y coordinate of the area of interest. Units same as Landsat scene imagery.
	double m_ULBoundsY;
	///The lower right corner x coordinate of the area of interest. Units same as Landsat scene imagery.
	double m_LRBoundsX;
	///The lower right corner y coordinate of the area of interest. Units same as Landsat scene imagery.
	double m_LRBoundsY;

};

}

#endif

