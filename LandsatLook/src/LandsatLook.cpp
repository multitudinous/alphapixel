#include <gdal/gdal.h>
#include "LandsatLook.h"

#include <iostream>
#include <sstream>
#include <fstream>

namespace landsatlook {

MTLParse::MTLParse( const std::string &MTLName ) :
	m_Success(  false ),
	m_Config( 0 )
{
	m_MTLName = MTLName;

	m_Config = new ConfigFile( m_MTLName.c_str() );
	if ( m_Config )
	{
		// Debug testing
		std::cout << "Testing MTL parsing by finding reading some values." << std::endl;
		bool exists1 = m_Config->keyExists("REFLECTANCE_MAXIMUM_BAND_1");
		std::cout << "REFLECTANCE_MAXIMUM_BAND_1: " << std::boolalpha << exists1 << "\n";
		bool exists2 = m_Config->keyExists("REFLECTANCE_MINIMUM_BAND_1");
		std::cout << "REFLECTANCE_MINIMUM_BAND_1: " << exists2 << "\n";

		std::string stringVal = m_Config->getValueOfKey<std::string>("REFLECTANCE_MAXIMUM_BAND_1");
		std::cout << "value of REFLECTANCE_MAXIMUM_BAND_1 string: " << stringVal << "\n";
		double doubleVal = m_Config->getValueOfKey<double>("REFLECTANCE_MAXIMUM_BAND_1");
		std::cout << "value of REFLECTANCE_MAXIMUM_BAND_1 double: " << doubleVal << "\n\n";

		stringVal = m_Config->getValueOfKey<std::string>("REFLECTANCE_MINIMUM_BAND_1");
		std::cout << "value of REFLECTANCE_MINIMUM_BAND_1 string: " << stringVal << "\n";
		doubleVal = m_Config->getValueOfKey<double>("REFLECTANCE_MINIMUM_BAND_1");
		std::cout << "value of REFLECTANCE_MINIMUM_BAND_1 double: " << doubleVal << "\n\n";

		m_Success = exists1 && exists2;
	}
}

MTLParse::~MTLParse()
{
	if ( m_Config )
	{
		delete m_Config;
	}
}

bool MTLParse::FindKey( const std::string &key, double &keyVal ) const
{
		if ( m_Success && m_Config->keyExists( key ) )
		{
			keyVal = m_Config->getValueOfKey< double >( key );
			return true;
		}
		return false;
}

bool MTLParse::FindKey( const std::string &key, int &keyVal ) const
{
		if ( m_Success && m_Config->keyExists( key ) )
		{
			keyVal = m_Config->getValueOfKey< int >( key );
			return true;
		}
		return false;
}

TopOfAtmosphere::TopOfAtmosphere( const MTLParse *config, int band ) :
	MetaDataOwner( band ),
	m_SunElevation( 0.0 ),
	m_Multiplier( 0.0 ),
	m_BaseOffset( 0.0 )
{
	double keyVal;
	std::vector< std::string > keyList;
	std::string sunElevKey, multKey, addKey;
	char bandChar[12];

	sprintf_s( bandChar, "%d", band );
	multKey = "REFLECTANCE_MULT_BAND_";
	multKey += bandChar;
	keyList.push_back( multKey );

	addKey = "REFLECTANCE_ADD_BAND_";
	addKey += bandChar;
	keyList.push_back( addKey );

	sunElevKey = "SUN_ELEVATION";
	keyList.push_back( sunElevKey );

	for ( std::vector< std::string >::iterator it = keyList.begin(); it != keyList.end(); ++it )
	{
		std::string key = *it;
		if ( config->FindKey( key, keyVal ) )
		{
			m_KeyValues.insert( std::pair< std::string, double >( key, keyVal ) );
			if ( key == multKey )
			{
				m_Multiplier = keyVal;
			}
			else if ( key == addKey )
			{
				m_BaseOffset = keyVal;
			}
			else if ( key == sunElevKey )
			{
				m_SunElevation = sin( DegToRad * keyVal );
			}
		}
	}

}

double TopOfAtmosphere::ComputeTOAReflectance( double uncorrectedNumber ) const
{
	double toaReflectance;

	// From: http://landsat.usgs.gov/Landsat8_Using_Product.php
	// Formula for top of atmosphere reflectance is
	// R' = M * Qcal + A
	// where
	// R'         = TOA planetary reflectance, without correction for solar angle.  Note that ??' does not contain a correction for the sun angle. 
	// M          = Band-specific multiplicative rescaling factor from the metadata (REFLECTANCE_MULT_BAND_x, where x is the band number)
	// A          = Band-specific additive rescaling factor from the metadata (REFLECTANCE_ADD_BAND_x, where x is the band number)
	// Qcal        = Quantized and calibrated standard product pixel values (DN)

	// Correction for sun angle is:

	// R = 	R' / cos(SZ)	=	R' / sin(SE)
	// where:    
	// R'         = TOA planetary reflectance, without correction for solar angle.
	// R          = TOA planetary reflectance
	// SE         = Local sun elevation angle. The scene center sun elevation angle in degrees is provided in the metadata (SUN_ELEVATION). 
	// SZ         = Local solar zenith angle;  SZ = 90° - SE

	toaReflectance = uncorrectedNumber * m_Multiplier + m_BaseOffset;
	toaReflectance /= m_SunElevation;

	return toaReflectance;
}

NormalizedIndex::NormalizedIndex( std::map< int, double > &bands, const int *reqBands )
{

	for ( int reqBand = 0; reqBands[ reqBand ] > 0; ++reqBand )
	{
		bool found = false;
		for ( std::map< int, double >::iterator it = bands.begin(); it != bands.end(); ++it )
		{
			if ( it->first == reqBands[ reqBand ] )
			{
				found = true;
			}
		}
		if ( ! found )
		{
			bands.insert( std::pair< int, double >( reqBands[ reqBand ], 0.0 ) );
		}
		m_reqBands.push_back( reqBands[ reqBand ] );
	}

}

double NormalizedIndex::ComputeIndex( const std::map< int, double > &bands ) const
{
	std::map< int, double >::const_iterator it1, it2;

	int firstBand = m_reqBands[0];
	int secondBand = m_reqBands[1];

	it1 = bands.find( firstBand );
	it2 = bands.find( secondBand );
	if ( it1 != bands.end() && it2 != bands.end() )
	{
		return ComputeIndex( it1->second, it2->second );
	}
	return 0.0;
}

double NormalizedIndex::ComputeIndex( double reflectance1, double reflectance2 ) const
{
	return ( ( reflectance1 - reflectance2 ) / ( reflectance1 + reflectance2 ) );
}

int NDVIBands[] = { 4, 3, 0 };

NDVI::NDVI( std::map< int, double > &bands ) :
	NormalizedIndex( bands, NDVIBands )
{
	// near infrared and visible red
	// ( NIR - VR ) / ( NIR + VR )
	// ( TM4 - TM3 ) / ( TM4 + TM3 )
}

int NDTIBands[] = { 5, 7, 0 };

NDTI::NDTI( std::map< int, double > &bands ) :
NormalizedIndex( bands, NDTIBands )
{
	// thematic mapper bands 5 and 7
	// ( TM5 - TM7 ) / ( TM5 + TM7 )
}

ImageBandData::ImageBandData( GDALDataset *gdalData, INPUT_SIZE *oneDataLine, TopOfAtmosphere *toa ) :
	m_GdalData( gdalData ), 
	m_GdalBand( 0 ),
	m_OneDataLine( oneDataLine ),
	m_toa( toa )
{
	if ( m_GdalData )
	{
		m_GdalBand = m_GdalData->GetRasterBand( 1 );
	}
}

ImageBandData::ImageBandData( ) :
	m_GdalData( 0 ), 
	m_GdalBand( 0 ),
	m_OneDataLine( 0 ),
	m_toa( 0 )
{
}

ImageBandData::~ImageBandData( )
{
	if ( m_GdalData )
	{
		GDALClose ( (GDALDatasetH)m_GdalData );
	}
	if ( m_toa )
	{
		delete m_toa;
	}
	if ( m_OneDataLine )
	{
		CPLFree( m_OneDataLine );
	}
}

void ImageBandData::SetGDALData( GDALDataset *gdalData )
{
	m_GdalData = gdalData;
	if ( m_GdalData )
	{
		m_GdalBand = m_GdalData->GetRasterBand( 1 );
	}
}


bool ImageBandData::RetrieveOneDataLine( int lineNumber, int startCol, int numCols )
{
	CPLErr cplErr;
	const char*errorMsg;
	if ( m_GdalBand )
	{
		cplErr = m_GdalBand->RasterIO( GF_Read, startCol, lineNumber, numCols, 1, m_OneDataLine, numCols, 1, INPUT_RASTER_TYPE, 0, 0 );
		if ( cplErr == CE_Failure )
		{
			errorMsg = CPLGetLastErrorMsg();
			std::cout << "Error reading image band data. " << errorMsg << std::endl;
			return false;
		}
		return true;
	}
	return false;
}

double ImageBandData::TOACorrectOnePixel( int pixelNumber )
{
	return m_toa->ComputeTOAReflectance( static_cast< double >( m_OneDataLine[ pixelNumber ] ) );
}

LandsatLook::LandsatLook( bool exportNDVI, bool exportNDTI, const std::string &landsatPath, const std::string &landsatFileRoot,
	const double &ULBoundsX, const double &ULBoundsY, const double &LRBoundsX, const double &LRBoundsY ) :
	m_exportNDVI( exportNDVI ),
	m_exportNDTI( exportNDTI ),
	m_landsatPath( landsatPath ),
	m_landsatFileRoot( landsatFileRoot ),
	m_ULCellX( 0 ),
	m_ULCellY( 0 ),
	m_LRCellX( 0 ),
	m_LRCellY( 0 ),
	m_ULBoundsX( ULBoundsX ),
	m_ULBoundsY( ULBoundsY ),
	m_LRBoundsX( LRBoundsX ),
	m_LRBoundsY( LRBoundsY )

{
	InitGDAL();

	if ( m_exportNDVI && m_exportNDTI )
	{
		std::cout << "Operation Landsat Look begun. Outputs will be NDVI and NDTI.\n" << std::endl;
	}
	else if ( m_exportNDVI )
	{
		std::cout << "Operation Landsat Look begun. Output will be NDVI only.\n" << std::endl;
	}
	else if ( m_exportNDTI )
	{
		std::cout << "Operation Landsat Look begun. Output will be NDTI only.\n" << std::endl;
	}
	else
	{
		std::cout << "Operation Landsat Look begun. No outputs were specified for generation.\n" << std::endl;
	}

}

void LandsatLook::InitGDAL() const
{
	GDALAllRegister();
}

GDALDataset* LandsatLook::OpenLandsatBand( const std::string &landsatBandFileRoot, int bandNumber ) const
{
	GDALDataset* bandDataset = NULL;
	char bandChar[12];

	sprintf_s( bandChar, "%d", bandNumber );
	std::string filename = landsatBandFileRoot;
	filename += bandChar;
	filename += ".TIF";
	bandDataset = (GDALDataset *) GDALOpen( filename.c_str(), GA_ReadOnly );
	return bandDataset;
}

bool LandsatLook::ComputeFarmCellBounds( const MTLParse *config, double &ULX, double &ULY, double &LRX, double &LRY, int &startCol, int &startRow, int &cols, int &rows ) const
{
	// Fetch bounds of images from meta data
	double imageULX, imageULY, imageLRX, imageLRY, cellSize;
	int imageRows, imageCols;

	// Keys for image bounds in UTM
    // CORNER_UL_PROJECTION_X_PRODUCT
    // CORNER_UL_PROJECTION_Y_PRODUCT
    // CORNER_LR_PROJECTION_X_PRODUCT
    // CORNER_LR_PROJECTION_Y_PRODUCT
	// REFLECTIVE_LINES
	// REFLECTIVE_SAMPLES
	// GRID_CELL_SIZE_REFLECTIVE

	std::string key;
	double keyVal;
	int intKey;

	// Landsat corners are cell centers and need to be adjusted by half cell width for
	// correct bounds interpretation.
	key = "GRID_CELL_SIZE_REFLECTIVE";
	if ( config->FindKey( key, keyVal ) )
	{
		cellSize = keyVal;
	}
	key = "CORNER_UL_PROJECTION_X_PRODUCT";
	if ( config->FindKey( key, keyVal ) )
	{
		imageULX = keyVal - cellSize * .5;
	}
	key = "CORNER_UL_PROJECTION_Y_PRODUCT";
	if ( config->FindKey( key, keyVal ) )
	{
		imageULY = keyVal + cellSize * .5;
	}
	key = "CORNER_LR_PROJECTION_X_PRODUCT";
	if ( config->FindKey( key, keyVal ) )
	{
		imageLRX = keyVal + cellSize * .5;
	}
	key = "CORNER_LR_PROJECTION_Y_PRODUCT";
	if ( config->FindKey( key, keyVal ) )
	{
		imageLRY = keyVal - cellSize * .5;
	}
	key = "REFLECTIVE_LINES";
	if ( config->FindKey( key, intKey ) )
	{
		imageRows = intKey;
	}
	key = "REFLECTIVE_SAMPLES";
	if ( config->FindKey( key, intKey ) )
	{
		imageCols = intKey;
	}

	ULX = ULX > imageULX ? ULX: imageULX;
	LRX = LRX < imageLRX ? LRX: imageLRX;
	ULY = ULY < imageULY ? ULY: imageULY;
	LRY = LRY > imageLRY ? LRY: imageLRY;

	int endRow, endCol;

	startRow = static_cast< int >( floor( ( imageULY - ULY ) / cellSize ) );
	startCol = static_cast< int >( floor( ( ULX - imageULX ) / cellSize ) );
	endRow = static_cast< int >( ceil( ( imageULY - LRY ) / cellSize ) );
	endCol = static_cast< int >( ceil( ( LRX - imageULX ) / cellSize ) );
	rows = endRow - startRow;
	cols = endCol - startCol;

	std::cout << "Requested area begins at column " << startCol << std::endl;
	std::cout << "   and ends at column " << endCol << std::endl;
	std::cout << "Requested area begins at row " << startRow << std::endl;
	std::cout << "   and ends at row " << endRow << std::endl;
	std::cout << "For an output grid size of " << cols << " columns by " << rows << " rows.\n" << std::endl;

	if ( startRow >= 0 && endRow < imageRows && startCol >= 0 && endCol < imageCols )
	{
		return true;
	}
	return false;

}

void LandsatLook::SetGeoRefFromLandsat( GDALDataset *targetDataset, const ImageBandMap &imageBands, int startCol, int startRow ) const
{

	ImageBandMap::const_iterator it = imageBands.begin();
	if ( it != imageBands.end() )
	{
		double srcTransformFactors[6];
		double destTransformFactors[6];

		GDALDataset *srcDataset = it->second->m_GdalData;
		srcDataset->GetGeoTransform( srcTransformFactors );

		// Coords of cropped region
		destTransformFactors[0] = srcTransformFactors[0] + startCol * srcTransformFactors[1];
		destTransformFactors[1] = srcTransformFactors[1];
		destTransformFactors[2] = srcTransformFactors[2];
		destTransformFactors[3] = srcTransformFactors[3] + startRow * srcTransformFactors[5];
		destTransformFactors[4] = srcTransformFactors[4];
		destTransformFactors[5] = srcTransformFactors[5];

		std::cout << "GDAL raster transform factors for output region:" << std::endl;
		for ( int ct = 0; ct < 6; ++ct )
		{
			std::cout << "Transform[" << ct << "] = " << destTransformFactors[ct] << std::endl;

		}
		std::cout << std::endl;
		// Set the geotransform
		CPLErr cplErr = targetDataset->SetGeoTransform( destTransformFactors );
		if ( cplErr == CE_Failure )
		{
			std::cout << "Error setting GeoTransform on output raster." << std::endl;
		}

		const char *projRefWkt = srcDataset->GetProjectionRef();
		targetDataset->SetProjection( projRefWkt );
	}
}

#define OUTPUT_SIZE float
#define OUTPUT_RASTER_TYPE GDT_Float32
// Undefine SEPARATE_OUTPUTS to override user's preferences and export both NDTI and NDVI
#define SEPARATE_OUTPUTS

bool LandsatLook::Operate()
{
	bool success = true;
	MTLParse *parse;
	std::string baseFile = m_landsatPath;
	baseFile += m_landsatFileRoot;
	
	std::string MTLFilename = baseFile;
	MTLFilename += "MTL.txt";
	parse = new MTLParse( MTLFilename );
	if ( parse )
	{
		success = parse->ParseSuccess();
#ifdef SEPARATE_OUTPUTS
		if ( success && m_exportNDVI )
		{
			success = ExportNDVI( parse );
		}
		if (success &&  m_exportNDTI )
		{
			success = ExportNDTI( parse );
		}
#else
		// This section will output both NDVI and NDTI regardless of the caller's preference.
		// This code could be modified to handle either or both outputs a little more 
		// efficiently than doing each one separately though the code will be messy.
		// One list of all the bands required for both calculated indices.
		std::map< int, double > bands;
		NDVI ndvi( bands );
		NDTI ndti( bands );

		// Load the Landsat bands from their files into GDAL rasters
		std::string rootImageName = baseFile;
		rootImageName += "B";
		ImageBandMap imageBands;

		// Crop to farm boundary in UTM coords
		int startCol, startRow, cols, rows;
		success = ComputeFarmCellBounds( parse, m_ULBoundsX, m_ULBoundsY, m_LRBoundsX, m_LRBoundsY, startCol, startRow, cols, rows );
		if ( success )
		{
			for ( std::map< int, double >::iterator it = bands.begin(); it != bands.end(); ++it )
			{
				int bandNumber = it->first;
				TopOfAtmosphere *toa = new TopOfAtmosphere( parse, bandNumber );
				if ( toa )
				{
					GDALDataset *bandRaster = OpenLandsatBand( rootImageName, bandNumber );
					if ( bandRaster )
					{
						INPUT_SIZE *bData = ( INPUT_SIZE * )CPLMalloc( sizeof( INPUT_SIZE ) * cols );
						if ( bData )
						{
							ImageBandData *ibd = new ImageBandData( bandRaster, bData, toa );
							imageBands.insert( std::pair< int, ImageBandData * >( bandNumber, ibd ) );
						}
						else
						{
							std::cout << "Error allocating one line of band image memory. Band " << bandNumber << std::endl;
							success = false;
							break;
						}
					}
					else
					{
						// Failed to open the band file, fail gracefully.
						std::cout << "Error opening band file. Band " << bandNumber << std::endl;
						success = false;
						break;
					}
				}
				else
				{
					// Failed to open the band file, fail gracefully.
					std::cout << "Error determining top of atmosphere correction factors. Band " << bandNumber << std::endl;
					success = false;
					break;
				}
			}
		}
		else
		{
			std::cout << "Bounds are outside of the Landsat scene." << std::endl;
		}

		if ( success )
		{
			GDALDataset *ndviDataset = NULL;
			GDALDataset *ndtiDataset = NULL;
			// Create a geoTIF for both outputs in single precision float format
			// written to the default project directory since no path specified here.
			std::string outputNDVIname = "LandsatLookNDVI";
			std::string outputNDTIname = "LandsatLookNDTI";
			GDALDriverManager *Mgr = GetGDALDriverManager();
			int numDrivers = Mgr->GetDriverCount();
			GDALDriver *driver = Mgr->GetDriverByName( "GTiff" );
			if ( driver )
			{
				outputNDVIname += ".tif";
				outputNDTIname += ".tif";
			}
			else
			{
				success = false;
			}
			if ( success )
			{
				// Close previously created files. Seems to mess up the process of writing to the
				// output rasters if not created from scratch each time.
				FILE *ffile;
				if ( ffile = fopen( outputNDVIname.c_str(), "rb" ))
				{
					fclose( ffile );
					remove( outputNDVIname.c_str() );
				}
				if ( ffile = fopen( outputNDTIname.c_str(), "rb" ))
				{
					fclose( ffile );
					remove( outputNDTIname.c_str() );
				}
				// Create new output GDAL datasets for NDVI and NDTI
				char **createOptions = NULL;
				ndviDataset = driver->Create( outputNDVIname.c_str(), cols, rows, 1, OUTPUT_RASTER_TYPE, createOptions );
				if ( ndviDataset == NULL )
				{
					std::cout << outputNDVIname << CPLGetLastErrorMsg() << std::endl;
					std::cout << "Operation failed." << std::endl;
					success = false;
				}
				ndtiDataset = driver->Create( outputNDTIname.c_str(), cols, rows, 1, OUTPUT_RASTER_TYPE, createOptions );
				if ( ndtiDataset == NULL )
				{
					std::cout << outputNDTIname << CPLGetLastErrorMsg() << std::endl;
					std::cout << "Operation failed." << std::endl;
					success = false;
				}
			}

			if ( success )
			{
				GDALRasterBand *ndviBand = NULL;
				GDALRasterBand *ndtiBand = NULL;
				SetGeoRefFromLandsat( ndviDataset, imageBands, startCol, startRow );
				SetGeoRefFromLandsat( ndtiDataset, imageBands, startCol, startRow );
				// Get the raster band pointers
				ndviBand = ndviDataset->GetRasterBand( 1 );
				ndtiBand = ndtiDataset->GetRasterBand( 1 );
				// Just checking. Debug code.
				int outRows, outCols, outBands;
				outCols = ndviDataset->GetRasterXSize();
				outRows = ndviDataset->GetRasterYSize();
				outBands = ndviDataset->GetRasterCount();
				outCols = ndviBand->GetXSize();
				outRows = ndviBand->GetYSize();
					
				// Create single lines of memory for output
				OUTPUT_SIZE *computedNDVI = ( OUTPUT_SIZE * )CPLMalloc( sizeof( OUTPUT_SIZE ) * cols );
				OUTPUT_SIZE *computedNDTI = ( OUTPUT_SIZE * )CPLMalloc( sizeof( OUTPUT_SIZE ) * cols );
				// Iterate through rows of landsat imagery, each band
				for ( int row = 0; row < rows; ++row )
				{
					for ( ImageBandMap::iterator it = imageBands.begin(); it != imageBands.end(); ++it )
					{
						ImageBandData *bandData = it->second;
						// Copy one line at a time into active buffers
						bandData->RetrieveOneDataLine( row + startRow, startCol, cols );
					}
					// Iterate through the pixels in each buffer
					for ( int col = 0; col < cols; ++col )
					{
						for ( ImageBandMap::iterator it = imageBands.begin(); it != imageBands.end(); ++it )
						{
							ImageBandData *bandData = it->second;
							double correctedValue = bandData->TOACorrectOnePixel( col );
							bands[ it->first ] = correctedValue;
						}
						// Feed the pixel's digital numbers into the calculators for NDVI and NDTI
						// Store the calculated NDTI and NDVI values in their own one line buffers
						computedNDVI[ col ] = static_cast< OUTPUT_SIZE >( ndvi.ComputeIndex( bands ) );
						computedNDTI[ col ] = static_cast< OUTPUT_SIZE >( ndti.ComputeIndex( bands ) );
					}
					// Write a line of data at a time into output rasters
					CPLErr cplErr = ndviBand->RasterIO( GF_Write, 0, row, cols, 1, computedNDVI, cols, 1, OUTPUT_RASTER_TYPE, 0, 0 );
					if ( cplErr == CE_Failure )
					{
						const char*errorMsg = CPLGetLastErrorMsg();
						std::cout << "Error writing NDVI data. " << errorMsg << std::endl;
						success = false;
						break;
					}
					cplErr = ndtiBand->RasterIO( GF_Write, 0, row, cols, 1, computedNDTI, cols, 1, OUTPUT_RASTER_TYPE, 0, 0 );
					if ( cplErr == CE_Failure )
					{
						const char*errorMsg = CPLGetLastErrorMsg();
						std::cout << "Error writing NDTI data. " << errorMsg << std::endl;
						success = false;
						break;
					}
				}
				// CPLFree() is broken and crashes. -Gary Huber
				CPLFree( computedNDVI );
				CPLFree( computedNDTI );
			}
			if ( success )
			{
				ndviDataset->FlushCache();
				GDALClose ( (GDALDatasetH)ndviDataset );
				ndtiDataset->FlushCache();
				GDALClose ( (GDALDatasetH)ndtiDataset );
			}

		}
		for ( ImageBandMap::iterator it = imageBands.begin(); it != imageBands.end(); ++it )
		{
			ImageBandData *bandData = it->second;
			delete bandData;
			it->second = NULL;
		}
#endif
		delete parse;
	}
	else
	{
		success = false;
	}

	if ( success )
	{
		std::cout << "Operation Landsat Look completed successfully." << std::endl;
	}
	else
	{
		std::cout << "Operation Landsat Look failed to complete successfully." << std::endl;
	}
	return success;
}

bool LandsatLook::ExportNDVI( MTLParse *parse )
{
	bool success = true;
	std::string baseFile = m_landsatPath;
	baseFile += m_landsatFileRoot;

	std::cout << "Computation of NDVI begun.\n" << std::endl;
	// One list of all the bands required for both calculated indices.
	std::map< int, double > bands;
	NDVI ndvi( bands );

	// Load the Landsat bands from their files into GDAL rasters
	std::string rootImageName = baseFile;
	rootImageName += "B";
	ImageBandMap imageBands;

	OUTPUT_SIZE maxValue = std::numeric_limits< OUTPUT_SIZE >::min();
	OUTPUT_SIZE minValue = std::numeric_limits< OUTPUT_SIZE >::max();

	// Crop to farm boundary in UTM coords
	int startCol, startRow, cols, rows;
	success = ComputeFarmCellBounds( parse, m_ULBoundsX, m_ULBoundsY, m_LRBoundsX, m_LRBoundsY, startCol, startRow, cols, rows );
	if ( success )
	{
		for ( std::map< int, double >::iterator it = bands.begin(); it != bands.end(); ++it )
		{
			int bandNumber = it->first;
			TopOfAtmosphere *toa = new TopOfAtmosphere( parse, bandNumber );
			if ( toa )
			{
				GDALDataset *bandRaster = OpenLandsatBand( rootImageName, bandNumber );
				if ( bandRaster )
				{
					INPUT_SIZE *bData = ( INPUT_SIZE * )CPLMalloc( sizeof( INPUT_SIZE ) * cols );
					if ( bData )
					{
						ImageBandData *ibd = new ImageBandData( bandRaster, bData, toa );
						imageBands.insert( std::pair< int, ImageBandData * >( bandNumber, ibd ) );
					}
					else
					{
						std::cout << "Error allocating one line of band image memory. Band " << bandNumber << std::endl;
						success = false;
						break;
					}
				}
				else
				{
					// Failed to open the band file, fail gracefully.
					std::cout << "Error opening band file. Band " << bandNumber << std::endl;
					success = false;
					break;
				}
			}
			else
			{
				// Failed to open the band file, fail gracefully.
				std::cout << "Error determining top of atmosphere correction factors. Band " << bandNumber << std::endl;
				success = false;
				break;
			}
		}
	}
	else
	{
		std::cout << "Bounds are outside of the Landsat scene." << std::endl;
	}

	if ( success )
	{
		GDALDataset *ndviDataset = NULL;
		// Create a geoTIF for both outputs in single precision float format
		// written to the default project directory since no path specified here.
		std::string outputNDVIname = "LandsatLookNDVI";
		GDALDriverManager *Mgr = GetGDALDriverManager();
		GDALDriver *driver = Mgr->GetDriverByName( "GTiff" );
		if ( driver )
		{
			outputNDVIname += ".tif";
		}
		else
		{
			success = false;
		}
		if ( success )
		{
			// Close previously created files. Seems to mess up the process of writing to the
			// output rasters if not created from scratch each time.
			FILE *ffile;
			if ( ffile = fopen( outputNDVIname.c_str(), "rb" ))
			{
				fclose( ffile );
				remove( outputNDVIname.c_str() );
			}
			// Create new output GDAL datasets for NDVI
			char **createOptions = NULL;
			ndviDataset = driver->Create( outputNDVIname.c_str(), cols, rows, 1, OUTPUT_RASTER_TYPE, createOptions );
			if ( ndviDataset == NULL )
			{
				std::cout << outputNDVIname << CPLGetLastErrorMsg() << std::endl;
				std::cout << "Operation failed." << std::endl;
				success = false;
			}
		}

		if ( success )
		{
			GDALRasterBand *ndviBand = NULL;
			SetGeoRefFromLandsat( ndviDataset, imageBands, startCol, startRow );
			// Get the raster band pointers
			ndviBand = ndviDataset->GetRasterBand( 1 );
			// Just checking. Debug code.
			int outRows, outCols, outBands;
			outCols = ndviDataset->GetRasterXSize();
			outRows = ndviDataset->GetRasterYSize();
			outBands = ndviDataset->GetRasterCount();
			outCols = ndviBand->GetXSize();
			outRows = ndviBand->GetYSize();
					
			// Create single lines of memory for output
			OUTPUT_SIZE *computedNDVI = ( OUTPUT_SIZE * )CPLMalloc( sizeof( OUTPUT_SIZE ) * cols );
			// Iterate through rows of landsat imagery, each band
			for ( int row = 0; row < rows; ++row )
			{
				for ( ImageBandMap::iterator it = imageBands.begin(); it != imageBands.end(); ++it )
				{
					ImageBandData *bandData = it->second;
					// Copy one line at a time into active buffers
					bandData->RetrieveOneDataLine( row + startRow, startCol, cols );
				}
				// Iterate through the pixels in each buffer
				for ( int col = 0; col < cols; ++col )
				{
					for ( ImageBandMap::iterator it = imageBands.begin(); it != imageBands.end(); ++it )
					{
						ImageBandData *bandData = it->second;
						double correctedValue = bandData->TOACorrectOnePixel( col );
						bands[ it->first ] = correctedValue;
					}
					// Feed the pixel's digital numbers into the calculators for NDVI
					// Store the calculated NDVI values in its own one line buffer
					computedNDVI[ col ] = static_cast< OUTPUT_SIZE >( ndvi.ComputeIndex( bands ) );
					if ( computedNDVI[ col ] > maxValue )
					{
						maxValue = computedNDVI[ col ];
					}
					if ( computedNDVI[ col ] < minValue )
					{
						minValue = computedNDVI[ col ];
					}
				}
				// Write a line of data at a time into output rasters
				CPLErr cplErr = ndviBand->RasterIO( GF_Write, 0, row, cols, 1, computedNDVI, cols, 1, OUTPUT_RASTER_TYPE, 0, 0 );
				if ( cplErr == CE_Failure )
				{
					const char*errorMsg = CPLGetLastErrorMsg();
					std::cout << "Error writing NDVI data. " << errorMsg << std::endl;
					success = false;
					break;
				}
			}
			CPLFree( computedNDVI );
		}
		if ( success )
		{
			if ( ndviDataset )
			{
				ndviDataset->FlushCache();
				GDALClose ( (GDALDatasetH)ndviDataset );
			}
		}

	}

	for ( ImageBandMap::iterator it = imageBands.begin(); it != imageBands.end(); ++it )
	{
		ImageBandData *bandData = it->second;
		delete bandData;
		it->second = NULL;
	}

	if ( success )
	{
		std::cout << "Computation of NDVI finished successfully.\n" << std::endl;
		std::cout << "The range of NDVI values found was " << minValue << " to " << maxValue << std::endl << std::endl;
	}
	else
	{
		std::cout << "Computation of NDVI unsuccessful.\n" << std::endl;
	}
	return success;
}

bool LandsatLook::ExportNDTI( MTLParse *parse )
{
	bool success = true;
	std::string baseFile = m_landsatPath;
	baseFile += m_landsatFileRoot;

	std::cout << "Computation of NDTI begun.\n" << std::endl;
	// One list of all the bands required for both calculated indices.
	std::map< int, double > bands;
	NDTI ndti( bands );

	// Load the Landsat bands from their files into GDAL rasters
	std::string rootImageName = baseFile;
	rootImageName += "B";
	ImageBandMap imageBands;

	OUTPUT_SIZE maxValue = std::numeric_limits< OUTPUT_SIZE >::min();
	OUTPUT_SIZE minValue = std::numeric_limits< OUTPUT_SIZE >::max();

	// Crop to farm boundary in UTM coords
	int startCol, startRow, cols, rows;
	success = ComputeFarmCellBounds( parse, m_ULBoundsX, m_ULBoundsY, m_LRBoundsX, m_LRBoundsY, startCol, startRow, cols, rows );
	if ( success )
	{
		for ( std::map< int, double >::iterator it = bands.begin(); it != bands.end(); ++it )
		{
			int bandNumber = it->first;
			TopOfAtmosphere *toa = new TopOfAtmosphere( parse, bandNumber );
			if ( toa )
			{
				GDALDataset *bandRaster = OpenLandsatBand( rootImageName, bandNumber );
				if ( bandRaster )
				{
					INPUT_SIZE *bData = ( INPUT_SIZE * )CPLMalloc( sizeof( INPUT_SIZE ) * cols );
					if ( bData )
					{
						ImageBandData *ibd = new ImageBandData( bandRaster, bData, toa );
						imageBands.insert( std::pair< int, ImageBandData * >( bandNumber, ibd ) );
					}
					else
					{
						std::cout << "Error allocating one line of band image memory. Band " << bandNumber << std::endl;
						success = false;
						break;
					}
				}
				else
				{
					// Failed to open the band file, fail gracefully.
					std::cout << "Error opening band file. Band " << bandNumber << std::endl;
					success = false;
					break;
				}
			}
			else
			{
				// Failed to open the band file, fail gracefully.
				std::cout << "Error determining top of atmosphere correction factors. Band " << bandNumber << std::endl;
				success = false;
				break;
			}
		}
	}
	else
	{
		std::cout << "Bounds are outside of the Landsat scene." << std::endl;
	}

	if ( success )
	{
		GDALDataset *ndtiDataset = NULL;
		// Create a geoTIF for both outputs in single precision float format
		// written to the default project directory since no path specified here.
		std::string outputNDTIname = "LandsatLookNDTI";
		GDALDriverManager *Mgr = GetGDALDriverManager();
		GDALDriver *driver = Mgr->GetDriverByName( "GTiff" );
		if ( driver )
		{
			outputNDTIname += ".tif";
		}
		else
		{
			success = false;
		}
		if ( success )
		{
			// Close previously created files. Seems to mess up the process of writing to the
			// output rasters if not created from scratch each time.
			FILE *ffile;
			if ( ffile = fopen( outputNDTIname.c_str(), "rb" ))
			{
				fclose( ffile );
				remove( outputNDTIname.c_str() );
			}
			// Create new output GDAL datasets for NDTI
			char **createOptions = NULL;
			ndtiDataset = driver->Create( outputNDTIname.c_str(), cols, rows, 1, OUTPUT_RASTER_TYPE, createOptions );
			if ( ndtiDataset == NULL )
			{
				std::cout << outputNDTIname << CPLGetLastErrorMsg() << std::endl;
				std::cout << "Operation failed." << std::endl;
				success = false;
			}
		}

		if ( success )
		{
			GDALRasterBand *ndtiBand = NULL;
			SetGeoRefFromLandsat( ndtiDataset, imageBands, startCol, startRow );
			// Get the raster band pointers
			ndtiBand = ndtiDataset->GetRasterBand( 1 );
			// Just checking. Debug code.
			int outRows, outCols, outBands;
			outCols = ndtiDataset->GetRasterXSize();
			outRows = ndtiDataset->GetRasterYSize();
			outBands = ndtiDataset->GetRasterCount();
			outCols = ndtiBand->GetXSize();
			outRows = ndtiBand->GetYSize();
					
			// Create single lines of memory for output
			OUTPUT_SIZE *computedNDTI = ( OUTPUT_SIZE * )CPLMalloc( sizeof( OUTPUT_SIZE ) * cols );
			// Iterate through rows of landsat imagery, each band
			for ( int row = 0; row < rows; ++row )
			{
				for ( ImageBandMap::iterator it = imageBands.begin(); it != imageBands.end(); ++it )
				{
					ImageBandData *bandData = it->second;
					// Copy one line at a time into active buffers
					bandData->RetrieveOneDataLine( row + startRow, startCol, cols );
				}
				// Iterate through the pixels in each buffer
				for ( int col = 0; col < cols; ++col )
				{
					for ( ImageBandMap::iterator it = imageBands.begin(); it != imageBands.end(); ++it )
					{
						ImageBandData *bandData = it->second;
						double correctedValue = bandData->TOACorrectOnePixel( col );
						bands[ it->first ] = correctedValue;
					}
					// Feed the pixel's digital numbers into the calculators for NDTI
					// Store the calculated NDTI values in its own one line buffer
					computedNDTI[ col ] = static_cast< OUTPUT_SIZE >( ndti.ComputeIndex( bands ) );
					if ( computedNDTI[ col ] > maxValue )
					{
						maxValue = computedNDTI[ col ];
					}
					if ( computedNDTI[ col ] < minValue )
					{
						minValue = computedNDTI[ col ];
					}
				}
				// Write a line of data at a time into output rasters
				CPLErr cplErr = ndtiBand->RasterIO( GF_Write, 0, row, cols, 1, computedNDTI, cols, 1, OUTPUT_RASTER_TYPE, 0, 0 );
				if ( cplErr == CE_Failure )
				{
					const char*errorMsg = CPLGetLastErrorMsg();
					std::cout << "Error writing NDTI data. " << errorMsg << std::endl;
					success = false;
					break;
				}
			}
			CPLFree( computedNDTI );
		}
		if ( success )
		{
			if ( ndtiDataset )
			{
				ndtiDataset->FlushCache();
				GDALClose ( (GDALDatasetH)ndtiDataset );
			}
		}

	}

	for ( ImageBandMap::iterator it = imageBands.begin(); it != imageBands.end(); ++it )
	{
		ImageBandData *bandData = it->second;
		delete bandData;
		it->second = NULL;
	}

	if ( success )
	{
		std::cout << "Computation of NDTI finished successfully." << std::endl;
		std::cout << "The range of NDTI values found was " << minValue << " to " << maxValue << std::endl << std::endl;
	}
	else
	{
		std::cout << "Computation of NDTI unsuccessful.\n" << std::endl;
	}
	return success;
}

int function()
{
	return 22;
}

}

