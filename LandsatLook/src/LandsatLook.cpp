#include <gdal/gdal.h>
#include "LandsatLook.h"

#include <iostream>
#include <sstream>
#include <fstream>

namespace landsatlook {

MTLParse::MTLParse(  std::string &MTLName ) :
	m_Success(  false ),
	m_Config( 0 )
{
	m_MTLName = MTLName;

	m_Config = new ConfigFile( m_MTLName.c_str() );
	if ( m_Config )
	{
		// Debug testing
		std::cout << "Testing MTL parsing routine." << std::endl;
		bool exists = m_Config->keyExists("REFLECTANCE_MAXIMUM_BAND_1");
		std::cout << "REFLECTANCE_MAXIMUM_BAND_1: " << std::boolalpha << exists << "\n";
		exists = m_Config->keyExists("REFLECTANCE_MINIMUM_BAND_1");
		std::cout << "REFLECTANCE_MINIMUM_BAND_1: " << exists << "\n";

		std::string stringVal = m_Config->getValueOfKey<std::string>("REFLECTANCE_MAXIMUM_BAND_1");
		std::cout << "value of REFLECTANCE_MAXIMUM_BAND_1 string: " << stringVal << "\n";
		double doubleVal = m_Config->getValueOfKey<double>("REFLECTANCE_MAXIMUM_BAND_1");
		std::cout << "value of REFLECTANCE_MAXIMUM_BAND_1 double: " << doubleVal << "\n\n";

		stringVal = m_Config->getValueOfKey<std::string>("REFLECTANCE_MINIMUM_BAND_1");
		std::cout << "value of REFLECTANCE_MINIMUM_BAND_1 string: " << stringVal << "\n";
		doubleVal = m_Config->getValueOfKey<double>("REFLECTANCE_MINIMUM_BAND_1");
		std::cout << "value of REFLECTANCE_MINIMUM_BAND_1 double: " << doubleVal << "\n\n";

		m_Success = exists;
	}
}

MTLParse::~MTLParse()
{
	if ( m_Config )
	{
		delete m_Config;
	}
}

bool MTLParse::FindKey( std::string &key, double &keyVal ) const
{
		if ( m_Success && m_Config->keyExists( key ) )
		{
			keyVal = m_Config->getValueOfKey< double >( key );
			return true;
		}
		return false;
}

bool MTLParse::FindKey( std::string &key, int &keyVal ) const
{
		if ( m_Success && m_Config->keyExists( key ) )
		{
			keyVal = m_Config->getValueOfKey< int >( key );
			return true;
		}
		return false;
}

#define DegToRad ( 3.14159265 / 180.0 )

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

double TopOfAtmosphere::ComputeTOAReflectance( double uncorrectedNumber )
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

void NormalizedIndex::SetBandNumbers( std::map< int, double > &bands, const int *requiredBands )
{

	for ( int reqBand = 0; requiredBands[ reqBand ] > 0; ++reqBand )
	{
		bool found = false;
		for ( std::map< int, double >::iterator it = bands.begin(); it != bands.end(); ++it )
		{
			if ( it->first == requiredBands[ reqBand ] )
			{
				found = true;
			}
		}
		if ( ! found )
		{
			bands.insert( std::pair< int, double >(requiredBands[ reqBand ], 0.0 ) );
		}
	}

}

double NormalizedIndex::ComputeIndex( double reflectance1, double reflectance2 )
{
	return ( ( reflectance1 - reflectance2 ) / ( reflectance1 + reflectance2 ) );
}

NDVI::NDVI( std::map< int, double > &bands )
{
	SetBandNumbers( bands );
}

void NDVI::SetBandNumbers( std::map< int, double > &bands )
{
	// near infrared and visible red
	// ( NIR - VR ) / ( NIR + VR )
	// ( TM4 - TM3 ) / ( TM4 + TM3 )
	const int requiredBands[] = { 3, 4, 0 };

	NormalizedIndex::SetBandNumbers( bands, requiredBands );
}

double NDVI::ComputeIndex( std::map< int, double > &bands )
{
	std::map< int, double >::iterator it1, it2;

	// near infrared and visible red
	// ( NIR - VR ) / ( NIR + VR )
	// ( TM4 - TM3 ) / ( TM4 + TM3 )
	int firstBand = 4;
	int secondBand = 3;

	it1 = bands.find( firstBand );
	it2 = bands.find( secondBand );
	if ( it1 != bands.end() && it2 != bands.end() )
	{
		return NormalizedIndex::ComputeIndex( it1->second, it2->second );
	}
	return 0.0;
}

NDTI::NDTI( std::map< int, double > &bands )
{
	SetBandNumbers( bands );
}

void NDTI::SetBandNumbers( std::map< int, double > &bands )
{
	// thematic mapper bands 5 and 7
	// ( TM5 - TM7 ) / ( TM5 + TM7 )
	const int requiredBands[] = { 5, 7, 0 };

	NormalizedIndex::SetBandNumbers( bands, requiredBands );
}

double NDTI::ComputeIndex( std::map< int, double > &bands )
{
	std::map< int, double >::iterator it1, it2;

	// thematic mapper bands 5 and 7
	// ( TM5 - TM7 ) / ( TM5 + TM7 )
	int firstBand = 5;
	int secondBand = 7;

	it1 = bands.find( firstBand );
	it2 = bands.find( secondBand );
	if ( it1 != bands.end() && it2 != bands.end() )
	{
		return NormalizedIndex::ComputeIndex( it1->second, it2->second );
	}
	return 0.0;
}

ImageBandData::ImageBandData( GDALDataset *gdalData, unsigned short int *oneDataLine, TopOfAtmosphere *toa ) :
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
	GDALClose ( (GDALDatasetH)m_GdalData );
	delete m_toa;
	if ( m_OneDataLine )
	{
		// Causes crashing!!??
		//CPLFree( m_OneDataLine );
	}
}

void ImageBandData::Cleanup()
{
	GDALClose ( (GDALDatasetH)m_GdalData );
	m_GdalData = NULL;
	m_GdalBand = NULL;
	delete m_toa;
	m_toa = NULL;
	if ( m_OneDataLine )
	{
		//CPLFree( m_OneDataLine );
		//m_OneDataLine = NULL;
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
		cplErr = m_GdalBand->RasterIO( GF_Read, startCol, lineNumber, numCols, 1, m_OneDataLine, numCols, 1, GDT_UInt32, 0, 0 );
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

LandsatLook::LandsatLook()
{
	InitGDAL();
}

void LandsatLook::InitGDAL() const
{
	GDALAllRegister();
}

GDALDataset* LandsatLook::OpenLandsatBand( std::string &rootFileName, int bandNumber ) const
{
	GDALDataset* bandDataset = NULL;
	char bandChar[12];

	sprintf_s( bandChar, "%d", bandNumber );
	std::string filename = rootFileName;
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

	std::string key;
	double keyVal;
	int intKey;

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

	if ( startRow >= 0 && endRow < imageRows && startCol >= 0 && endCol < imageCols )
	{
		return true;
	}
	return false;

}

void LandsatLook::SetGeoRefFromLandsat( GDALDataset *targetDataset, ImageBandMap &imageBands, int startCol, int startRow ) const
{

	ImageBandMap::iterator it = imageBands.begin();
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

		for ( int ct = 0; ct < 6; ++ct )
		{
			std::cout << "Transform[" << ct << "] = " << destTransformFactors[ct] << std::endl;

		}
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

bool LandsatLook::Operate()
{
	bool success = true;
	MTLParse *parse;
	std::string rootFilepath = "C:\\Data\\Dev\\AgSolver-AlphaPixel\\alphapixel\\LandsatLook\\Data\\LC80260312014079LGN00_";
	
	std::string filename = rootFilepath;
	filename += "MTL.txt";
	parse = new MTLParse( filename );
	if ( parse )
	{
		success = parse->ParseSuccess();

		// One list of all the bands required for both calculated indices.
		std::map< int, double > bands;

		NDTI ndti( bands );
		NDVI ndvi( bands );

		// Load the Landsat bands from their files into GDAL rasters
		std::string rootImageName = rootFilepath;
		rootImageName += "B";
		ImageBandMap imageBands;

		// Crop to farm boundary in UTM coords
		// NW: UTM ( WGS84 ) - ( 476012.685, 4658427.925 )
		// SE: UTM ( WGS84 ) - ( 477569.273, 4656871.337 )
		double ULX = 476012.685;
		double ULY = 4658427.925;
		double LRX = 477569.273;
		double LRY = 4656871.337;
		int startCol, startRow, cols, rows;
		success = ComputeFarmCellBounds( parse, ULX, ULY, LRX, LRY, startCol, startRow, cols, rows );
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
						unsigned short int *bData = ( unsigned short int * )CPLMalloc( sizeof( unsigned short int ) * cols );
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
				ndviDataset = driver->Create( outputNDVIname.c_str(), cols, rows, 1, GDT_Float32, createOptions );
				if ( ndviDataset == NULL )
				{
					std::cout << outputNDVIname << CPLGetLastErrorMsg() << std::endl;
					std::cout << "Operation failed." << std::endl;
					success = false;
				}
				ndtiDataset = driver->Create( outputNDTIname.c_str(), cols, rows, 1, GDT_Float32, createOptions );
				if ( ndtiDataset == NULL )
				{
					std::cout << outputNDTIname << CPLGetLastErrorMsg() << std::endl;
					std::cout << "Operation failed." << std::endl;
					success = false;
				}
			}

			if ( success )
			{
				SetGeoRefFromLandsat( ndviDataset, imageBands, startCol, startRow );
				SetGeoRefFromLandsat( ndtiDataset, imageBands, startCol, startRow );
				// Get the raster band pointers
				GDALRasterBand *ndviBand = ndviDataset->GetRasterBand( 1 );
				GDALRasterBand *ndtiBand = ndtiDataset->GetRasterBand( 1 );

				// Just checking.
				int outRows, outCols, outBands;
				outCols = ndviDataset->GetRasterXSize();
				outRows = ndviDataset->GetRasterYSize();
				outBands = ndviDataset->GetRasterCount();
				outCols = ndtiBand->GetXSize();
				outRows = ndtiBand->GetYSize();

				// Create single lines of memory for output
				float *computedNDVI = ( float * )CPLMalloc( sizeof( float ) * cols );
				float *computedNDTI = ( float * )CPLMalloc( sizeof( float ) * cols );
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
						computedNDVI[ col ] = static_cast< float >( ndvi.ComputeIndex( bands ) );
						computedNDTI[ col ] = static_cast< float >( ndti.ComputeIndex( bands ) );
					}
					// Write a line of data at a time into output rasters
					CPLErr cplErr = ndviBand->RasterIO( GF_Write, 0, row, cols, 1, computedNDVI, cols, 1, GDT_Float32, 0, 0 );
					if ( cplErr == CE_Failure )
					{
						const char*errorMsg = CPLGetLastErrorMsg();
						std::cout << "Error writing NDVI data. " << errorMsg << std::endl;
						success = false;
						break;
					}
					cplErr = ndtiBand->RasterIO( GF_Write, 0, row, cols, 1, computedNDTI, cols, 1, GDT_Float32, 0, 0 );
					if ( cplErr == CE_Failure )
					{
						const char*errorMsg = CPLGetLastErrorMsg();
						std::cout << "Error writing NDTI data. " << errorMsg << std::endl;
						success = false;
						break;
					}
					
				}
			}
			if ( success )
			{
				ndviDataset->FlushCache();
				ndtiDataset->FlushCache();
				GDALClose ( (GDALDatasetH)ndviDataset );
				GDALClose ( (GDALDatasetH)ndtiDataset );
			}

		}

		// Crashes if deleting the ImageBandData member or the members within it.
		/*
		for ( ImageBandMap::iterator it = imageBands.begin(); it != imageBands.end(); ++it )
		{
			ImageBandData *bandData = it->second;
			bandData->Cleanup();
		}
		*/
		delete parse;
	}
	else
	{
		success = false;
	}

	return success;
}

int function()
{
	return 22;
}

}

