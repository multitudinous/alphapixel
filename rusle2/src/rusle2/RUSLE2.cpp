// Project headers
#include "RUSLE2.h"

// C++ headers
#include <iostream>
#include <fstream>

// GDAL headers
#include <gdalwarper.h>
#include <cpl_port.h>

namespace rusle2 {

// Define RUSLE2_WKT_OUTPUT to enable full well-known-text output to 
// console for files with a spatial reference system.
//#define RUSLE2_WKT_OUTPUT

// Define RUSLE2_NO_DOWNSIZE to prevent cropping the DEM to the area of 
// interest before processing. This will result in significantly more processing time.
//#define RUSLE2_NO_DOWNSIZE

// Define RUSLE2_WRITE_NAME_FIELD to have a name composed of the column 
// and row of the DEM attached to each arrow vector exported to the output arrow file.
// Normally it is disabled as it adds procesing time.
//#define RUSLE2_WRITE_NAME_FIELD

// Define RUSLE2_NO_FIX_FLAT_SPOTS to prevent the filling in of 
// low areas larger than a single DEM cell. These areas will halt flow accumulation
// and truncate the drainage channels where they occur.
//#define RUSLE2_NO_FIX_FLAT_SPOTS

// Define RUSLE2_FLAT_RECURSION_SOLUTION for a recursive method of healing the flow
// across flat areas of terrain. It tends to create unrealistic flow patterns and
// is not recommended except for testing.
//#define RUSLE2_FLAT_RECURSION_SOLUTION

//#define FORCE_VECTOR_TO_DEM_FROM_SERVER_EPSG_NO_MATTER_WHAT

Rusle2::Rusle2( const std::string& demFilename,
               const std::string& farmFilename,
               const double& resolutionFactor,
               const double& channelDrainageAreaAcres,
               const double& pitFillLevel )
    :
    m_gridShaper( 0 ),
    m_resampleDEM( 0 ),
    m_aspectDEM( 0 ),
    m_proximalInflowDEM( 0 ),
    m_totalInflowDEM( 0 ),
    m_drainageChannelDEM( 0 ),
    m_farmShape( 0 ),
    m_arrowShape( 0 ),
    m_demFilename( demFilename ),
    m_farmFilename( farmFilename ),
    m_resolutionFactor( resolutionFactor ),
    m_channelDrainageAreaAcres( channelDrainageAreaAcres ),
    m_pitFillLevel( pitFillLevel ),
    m_iCellRangeX( 0 ),
    m_iCellRangeY( 0 ),
    m_channelMinimumInflow( 0 ),
    m_reprojectAvailable( true ),
    m_fixFlats( false )
{
    Init();
}

Rusle2::~Rusle2()
{
    if( m_arrowShape )
    {
        OGRDataSource::DestroyDataSource ( m_arrowShape );
        /*std::string filename = m_baseName;
        filename += "arrows.prj";
        FILE *testOpen = fopen( filename.c_str(), "r" );
        if( testOpen )
        {
            fclose( testOpen );
            remove( filename.c_str() );
            std::string wktRefString;
            if( GetSpatialReferenceString( wktRefString ) )
            {
                testOpen = fopen( filename.c_str(), "w" );
                if( testOpen )
                {
                    fwrite( wktRefString.c_str(), 1, wktRefString.size(), testOpen );
                }
                fclose( testOpen );
            }
        }*/
    }
    if( m_gridShaper )
    {
        delete m_gridShaper;
    }
    if( m_aspectDEM )
    {
        GDALClose ( (GDALDatasetH)m_aspectDEM );
    }
    if( m_proximalInflowDEM )
    {
        GDALClose ( (GDALDatasetH)m_proximalInflowDEM );
    }
    if( m_totalInflowDEM )
    {
        GDALClose ( (GDALDatasetH)m_totalInflowDEM );
    }
    if( m_drainageChannelDEM )
    {
        GDALClose ( (GDALDatasetH)m_drainageChannelDEM );
    }
}

void Rusle2::Init()
{
    // Register GDAL and OGR for file access
    GDALAllRegister();
    OGRRegisterAll();

    // Process farm name for later use
    std::string tempName = GetFilePart( m_farmFilename );
    m_baseName = StripExtension( tempName );
    m_baseName += "_";

    OGRErr ogrErr = m_demRef.importFromEPSG( DEM_FROM_SERVER_EPSG );
    if( ogrErr != OGRERR_NONE )
    {
        m_reprojectAvailable = false;
    }
}

bool Rusle2::GetOutputGeoTransform6( double *paramArray6 ) const
{
    if( m_gridShaper )
    {
        return ( m_gridShaper->GetOutputGeoTransform6( paramArray6 ) );
    }
    return false;
}

bool Rusle2::GetSpatialReferenceString( std::string& refString ) const
{
    if( m_gridShaper )
    {
        return ( m_gridShaper->GetSpatialReferenceString( refString ) );
    }
    return false;
}

void Rusle2::GetOutputFileDims( int& cellsX, int& cellsY ) const
{
    cellsX = m_iCellRangeX;
    cellsY = m_iCellRangeY;
}

bool Rusle2::FetchCellValue( const Rusle2Rasters& rasterType, const int& xCell, const int& yCell, int& value ) const
{
    GDALRasterBand *band = NULL;
        
    switch ( rasterType )
    {
        case ASPECT:
        {
            band = m_aspectDEM->GetRasterBand( 1 );
            break;
        }
        case PROXIMALINFLOW:
        {
            band = m_proximalInflowDEM->GetRasterBand( 1 );
            break;
        }
        case TOTALINFLOW:
        {
            band = m_totalInflowDEM->GetRasterBand( 1 );
            break;
        }
        case CHANNEL:
        {
            band = m_drainageChannelDEM->GetRasterBand( 1 );
            break;
        }
    }

    if( band )
    {
        return FindBandCellValue( band, xCell, yCell, value );
    }
    return false;

}

bool Rusle2::LoadAndOperate()
{

    m_gridShaper = new GridShaper( m_demFilename, m_farmFilename, m_resolutionFactor );
    if( m_gridShaper )
    {
        m_resampleDEM = m_gridShaper->LoadAndOperate( m_farmShape );
        if( m_resampleDEM )
        {
            m_cellMaxX = m_gridShaper->GetCellMaxX();
            m_cellMinX = m_gridShaper->GetCellMinX();
            m_cellMaxY = m_gridShaper->GetCellMaxY();
            m_cellMinY = m_gridShaper->GetCellMinY();
            m_iCellMaxX = m_gridShaper->GetICellMaxX();
            m_iCellMinX = m_gridShaper->GetICellMinX();
            m_iCellMaxY = m_gridShaper->GetICellMaxY();
            m_iCellMinY = m_gridShaper->GetICellMinY();
            m_iCellRangeX = m_gridShaper->GetICellRangeX();
            m_iCellRangeY = m_gridShaper->GetICellRangeY();
            //m_farmBounds = m_gridShaper->GetFarmBounds();
        }
        else
        {
            return false;
        }
    }
    else
    {
        return false;
    }

#if 1
    m_aspectDEM = (GDALDataset*)( GDALOpen( "aspect_rbarnes.tif", GA_ReadOnly ) );
#else
    m_aspectDEM = BuildAspectDEM();
#endif
    if( m_aspectDEM == NULL )
    {
        return false;
    }

    #ifdef RUSLE2_NO_FIX_FLAT_SPOTS
    m_fixFlats = false;
    #endif
    if( ! FixFlats() )
    {
        return false;
    }

    m_arrowShape = CreateArrowDatabase( "arrows.shp", m_aspectDEM );
    if( m_arrowShape == NULL )
    {
        return false;
    }
    m_proximalInflowDEM = BuildProximalInflowDEM();
    if( m_proximalInflowDEM == NULL )
    {
        return false;
    }

    m_totalInflowDEM = BuildTotalInflowDEM();
    if( m_totalInflowDEM == NULL )
    {
        return false;
    }

    m_drainageChannelDEM = BuildDrainageChannelDEM();
    if( m_drainageChannelDEM == NULL )
    {
        return false;
    }

    std::cout << "Project completed successfully.\n" << std::endl;
    return true;
}

GDALDataset* Rusle2::BuildAspectDEM()
{
    CPLErr cplErr;
    const char*errorMsg;

    GDALDataset* aspectDataset = m_gridShaper->CreateFarmSizeDataset( "aspect", NULL, m_resampleDEM, GDT_Byte );
    if( aspectDataset == NULL )
    {
        return NULL;
    }

    // Get the output band
    GDALRasterBand* aspectBand = aspectDataset->GetRasterBand( 1 );
    if( aspectBand == NULL || aspectBand->GetBand() != 1 )
    {
        std::cout << "Could not obtain correct band number in aspect data set" << std::endl;
        GDALClose ( aspectDataset );
        return NULL;
    }
    GByte AspectNodataValue = static_cast<GByte>( aspectBand->GetMaximum() );
    aspectBand->SetNoDataValue( AspectNodataValue );
    cplErr = aspectBand->Fill( AspectNodataValue );
    if( cplErr == CE_Failure )
    {
        errorMsg = CPLGetLastErrorMsg();
        std::cout << "BuildAspectDEM error " << errorMsg << std::endl;
        GDALClose ( aspectDataset );
        return NULL;
    }

    // Get the nodata value from the raster band
    GDALRasterBand *demBand = m_resampleDEM->GetRasterBand( 1 );
    if( demBand == NULL || demBand->GetBand() != 1 )
    {
        std::cout << "Could not obtain correct band in DEM data set" << std::endl;
        GDALClose ( aspectDataset );
        return NULL;
    }
    int success = 0;
    bool testNodata = false;
    float nodataValue = static_cast<float>( demBand->GetNoDataValue( &success ) );
    if( success )
    {
        testNodata = true;
    }

    int pitsFilled = 0;
    int flatCells = 0;

    // Create buffer array to read three lines of data m_iCellRangeX + 2 wide
    int inputScanWidth = m_iCellRangeX + 2;
    int inputStartX = m_iCellMinX - 1;
    int inputStartY = m_iCellMinY - 1;
    float *threeScanLines;
    threeScanLines = (float *) CPLMalloc( sizeof(float) * inputScanWidth * 3 );
    float *inputElev[3] = { &threeScanLines[0], &threeScanLines[inputScanWidth], &threeScanLines[inputScanWidth * 2] };
    ValidRasterPointTest< float > *demTest = new ValidRasterPointTest< float >( inputScanWidth, 3, testNodata, nodataValue, threeScanLines );

#if 1
    BuildAspectRaster2( m_resampleDEM, aspectDataset );
#else
    // Create buffer array to store one line of slope direction m_iCellRangeX wide
    GByte* oneAspectLine = (GByte *) CPLMalloc( sizeof(GByte) * m_iCellRangeX );

    for ( int outAspectLine = 0; outAspectLine < m_iCellRangeY; ++outAspectLine, ++inputStartY )
    {
        bool writeDEMLine = false;
        // Outer loop reads three lines from source DEM
        cplErr = demBand->RasterIO( GF_Read, inputStartX, inputStartY, inputScanWidth, 3, threeScanLines, inputScanWidth, 3, GDT_Float32, 0, 0 );
        if( cplErr == CE_Failure )
        {
            errorMsg = CPLGetLastErrorMsg();
            std::cout << "BuildAspectDEM error 2 " << errorMsg << std::endl;
            GDALClose ( aspectDataset );
            CPLFree( threeScanLines );
            CPLFree( oneAspectLine );
            delete demTest;
            return NULL;
        }

        for ( int outPixel = 0; outPixel < m_iCellRangeX; ++outPixel )
        {
            // Inner loop calculates slope direction for each 9 cell matrix
            float elDiff, pitFillValue = std::numeric_limits<float>::max();
            float maxInflowElDiff = 0.0f;
            float maxOutflowElDiff = 0.0f;
            int outFlowDir = FLAT;    // default is flat
            int inFlowDir = FLAT;    // default is flat
            int flatFlowDir = FLAT;
            int ptsHigher = 0;

            if( demTest->TestPoint( outPixel + 1, 1 ) )
            {
                float thisEl = inputElev[1][outPixel + 1];
                float refEl;

                for ( int dir = NORTH; dir <= NORTHWEST; ++dir )
                {
                    int x, y;
                    int inputX = outPixel + 1;
                    int inputY = 1;
                    GetDirectionalIndices( static_cast< Rusle2Direction >( dir ), inputX, inputY, x, y );
                    if( demTest->TestPoint( x, y ) )
                    {
                        refEl = inputElev[y][x];
                        elDiff = refEl - thisEl;
                        if( elDiff == 0.0 )
                        {
                            flatFlowDir = dir;
                        }
                        if( elDiff < 0.0 && elDiff < maxOutflowElDiff )
                        {
                            maxOutflowElDiff = elDiff;
                            outFlowDir = dir;
                        }
                        if( elDiff > 0.0 )
                        {
                            ++ptsHigher;
                            if( elDiff > maxInflowElDiff )
                            {
                                maxInflowElDiff = elDiff;
                                inFlowDir = dir;
                            }
                            if( refEl < pitFillValue )
                            {
                                pitFillValue = refEl;
                            }
                        }
                    }
                }
                // adjust direction if there is no outflow to lower points
                if( outFlowDir == FLAT )
                {
                    // If both flow directions are 0 then the cell is flat
                    if( flatFlowDir != 0 )
                    {
                        ++flatCells;    //flatFlowDir;
                    }
                    else if( inFlowDir != 0 )
                    {
                        if( ptsHigher == 8 )
                        {
                            // Pit
                            inputElev[1][outPixel + 1] = pitFillValue;
                            writeDEMLine = true;
                            ++pitsFilled;
                            ++flatCells;
                        }
                        // Reverse direction of largest incoming slope
                        //outFlowDir = inFlowDir + 4 > 8 ? inFlowDir - 4: inFlowDir + 4;
                    }
                    else
                    {
                        ++flatCells;
                    }
                }
                // Store data
                oneAspectLine[outPixel] = outFlowDir;
            }
            else
            {
                oneAspectLine[outPixel] = AspectNodataValue;
            }
        }
        // Outer loop writes line of slope direction to raster
        cplErr = aspectBand->RasterIO( GF_Write, 0, outAspectLine, m_iCellRangeX, 1, oneAspectLine, m_iCellRangeX, 1, GDT_Byte, 0, 0 );
        if( cplErr == CE_Failure )
        {
            errorMsg = CPLGetLastErrorMsg();
            std::cout << "BuildAspectDEM error 3 " << errorMsg << std::endl;
            GDALClose ( aspectDataset );
            CPLFree( threeScanLines );
            CPLFree( oneAspectLine );
            delete demTest;
            return NULL;
        }
        if( writeDEMLine )
        {
            cplErr = demBand->RasterIO( GF_Write, inputStartX, inputStartY + 1, inputScanWidth, 1, inputElev[1], inputScanWidth, 1, GDT_Float32, 0, 0 );
            if( cplErr == CE_Failure )
            {
                errorMsg = CPLGetLastErrorMsg();
                std::cout << "BuildAspectDEM error 4 " << errorMsg << std::endl;
                GDALClose ( aspectDataset );
                CPLFree( threeScanLines );
                CPLFree( oneAspectLine );
                delete demTest;
                return NULL;
            }
        }
    }
    CPLFree( oneAspectLine );
#endif

    aspectDataset->FlushCache();
    m_resampleDEM->FlushCache();
    CPLFree( threeScanLines );
    delete demTest;
    
    GByte max = 0, min = 0;
    success = FindBandValidValueRange( aspectBand, min, max );
    if( ! success )
    {
        std::cout << "No non-NULL data was created in aspect map." << std::endl;
    }

    std::cout << "Range of aspect values found " << (int)min << " to " << (int)max << std::endl;
    std::cout << "Number of single cell DEM pits filled = " << pitsFilled << std::endl;
    std::cout << "Number of flat DEM cells after first round of pit filling = " << flatCells << std::endl;

    if( flatCells )
    {
        m_fixFlats = true;
    }

    std::cout << "Aspect map complete.\n" << std::endl;
    return aspectDataset;
}

bool Rusle2::FixFlats()
{
    // Load all aspect data and matching set of DEM data
    // Read the aspect one pixel at a time until finding a value of 0 (flat)
    // Read the elevation of the flat spot
    // Search contiguous 8 cells for any other flat cells of same elevation and for the lowest elevation value that has a flow direction that isn't into the flat area
    // Build a list of the flat cells. Store the location of the best spill point and its elevation. Store the points with the lowest elevation in a list too.
    // If a spill point isn't found raise all the flat points to the lowest elevation found and set all the points in the list of those with that elevation to an aspect of 0 and repeat the process with the new
    // higher elevation and larger area.

    CPLErr cplErr;
    const char*errorMsg;
    int flatsFilled = 0;

    // Get the band and nodata value from the aspect band
    GDALRasterBand *aspectBand = m_aspectDEM->GetRasterBand( 1 );
    if( aspectBand == NULL || aspectBand->GetBand() != 1 )
    {
        std::cout << "Could not obtain correct band in aspect data set" << std::endl;
        return false;
    }
    int success = 0;
    bool aspectTestNodata = false;
    GByte aspectNodataValue = static_cast<GByte>( aspectBand->GetNoDataValue( &success ) );
    if( success )
    {
        aspectTestNodata = true;
    }

    GByte *aspectBuffer = (GByte *) CPLMalloc( sizeof(GByte) * m_iCellRangeX * m_iCellRangeY );
    ValidRasterPointTest< GByte > *aspectTest = new ValidRasterPointTest< GByte >( m_iCellRangeX, m_iCellRangeY, aspectTestNodata, aspectNodataValue, aspectBuffer );
    CPLErr cplErr2 = aspectBand->RasterIO( GF_Read, 0, 0, m_iCellRangeX, m_iCellRangeY, aspectBuffer, m_iCellRangeX, m_iCellRangeY, GDT_Byte, 0, 0 );

    if( m_fixFlats )
    {
    // Get the band and nodata value from the DEM raster
        GDALRasterBand *demBand = m_resampleDEM->GetRasterBand( 1 );
        if( demBand == NULL || demBand->GetBand() != 1 )
        {
            std::cout << "Could not obtain correct band in DEM data set" << std::endl;
            return false;
        }
        bool demTestNodata = false;
        float demNodataValue = static_cast<float>(demBand->GetNoDataValue( &success ) );
        if( success )
        {
            demTestNodata = true;
        }

        float *demBuffer = (float *) CPLMalloc( sizeof(float) * m_iCellRangeX * m_iCellRangeY );
        ValidRasterPointTest< float > *demTest = new ValidRasterPointTest< float >( m_iCellRangeX, m_iCellRangeY, demTestNodata, demNodataValue, demBuffer );
    
        // Populate buffers from rasters
        cplErr = demBand->RasterIO( GF_Read, m_iCellMinX, m_iCellMinY, m_iCellRangeX, m_iCellRangeY, demBuffer, m_iCellRangeX, m_iCellRangeY, GDT_Float32, 0, 0 );
        if( cplErr == CE_Failure || cplErr2 == CE_Failure )
        {
            errorMsg = CPLGetLastErrorMsg();
            std::cout << errorMsg << std::endl;
            CPLFree( demBuffer );
            CPLFree( aspectBuffer );
            delete demTest;
            delete aspectTest;
            return false;
        }

        // Do the work here
        int curX, curY;
        int destX, destY;

        for ( int initialY = 0; initialY < m_iCellRangeY; ++initialY )
        {
            for (int initialX = 0; initialX < m_iCellRangeX; ++initialX )
            {
                int traceX = initialX, traceY = initialY;
                std::map< int, float > flatCellMap;
                std::map< int, float > borderCellMap;

                if( aspectTest->TestPoint( traceX, traceY ) )
                {
                    // Test this cell to see if it is marked as flat
                    int index = traceX + traceY * m_iCellRangeX;
                    float flatElev = demBuffer[ index ];
                    if( aspectBuffer[ index ] == 0 )
                    {
                        flatCellMap[ index ] = flatElev;

                        bool NotYetSatisfied = true;
                        while ( NotYetSatisfied )
                        {
                            NotYetSatisfied = false;
                            //bool stillFindingCells = true;
                            //while ( stillFindingCells )
                            {
                                //stillFindingCells = false;
                                int restartIndex = -1;
                                for ( std::map< int, float >::iterator it = flatCellMap.begin(); it != flatCellMap.end() || restartIndex >= 0; ++it )
                                {
                                    if( restartIndex >= 0 )
                                    {
                                        std::map< int, float >::iterator itTest = flatCellMap.find( restartIndex );
                                        if( itTest == flatCellMap.end() )
                                        {
                                            // This should never happen but just in case
                                            break;
                                        }
                                        it = itTest;
                                        restartIndex = -1;
                                    }
                                    curY = it->first / m_iCellRangeX;
                                    curX = it->first - curY * m_iCellRangeX;
                                    // Test cells contiguous for flatness or boundariness and for lower or equal elevation flowing away.
                                    // If a contiguous cell is found that meets the criteria then set flow direction for this cell and continue
                                    for ( int direction = NORTH; direction <= NORTHWEST; ++direction )
                                    {
                                        GetDirectionalIndices( static_cast< Rusle2Direction >( direction ), curX, curY, destX, destY );
                                        if( aspectTest->TestPoint( destX, destY ) )
                                        {
                                            int tempIndex = destX + destY * m_iCellRangeX;
                                            if( flatCellMap.find( tempIndex ) == flatCellMap.end() 
                                                && borderCellMap.find( tempIndex ) == borderCellMap.end() )
                                            {
                                                if( aspectBuffer[ tempIndex ] == 0 && demBuffer[ tempIndex ] <= flatElev )
                                                {
                                                    flatCellMap[ tempIndex ] = demBuffer[ tempIndex ];
                                                    //stillFindingCells = true;
                                                    if( tempIndex < it->first )
                                                    {
                                                        if( restartIndex < 0 || tempIndex < restartIndex )
                                                        {
                                                            // Reset where the loop position is on the next round in order
                                                            // to catch new entries.
                                                            restartIndex = tempIndex;
                                                        }
                                                    }
                                                }
                                                else
                                                {
                                                    borderCellMap[ tempIndex ] = demBuffer[ tempIndex ];
                                                }
                                            }
                                        }
                                    }
                                }
                            }

                            // Process the borderCellMap looking for the lowest elevation.
                            // If the cell with lowest elevation is directed towards one of the cells in the flat area, fill the flat area to that elevation, set all border cells of the same elevation to flat and move them to flatCellMap
                            // Repeat above process
                            float minElevFound = std::numeric_limits<float>::max();
                            int indexOfLowestPoint = -1;
                            std::map< int, float >::iterator itLowCell;
                            for ( std::map< int, float >::iterator it = borderCellMap.begin(); it != borderCellMap.end(); ++it )
                            {
                                if( it->second < minElevFound )
                                {
                                    minElevFound = it->second;
                                    indexOfLowestPoint = it->first;
                                    itLowCell = it;
                                }
                            }
                            // Set elevation of all flat cells to the lowest border cell or to the user-specified maximum fill level
                            if( indexOfLowestPoint >= 0 )
                            {
                                float fillDepth = minElevFound - flatElev;
                                bool reachedMaxFillLevel = false;
                                if( fillDepth > static_cast<float>( m_pitFillLevel ) )
                                {
                                    minElevFound = flatElev + static_cast<float>( m_pitFillLevel );
                                    reachedMaxFillLevel = true;
                                }
                                for ( std::map< int, float >::iterator it = flatCellMap.begin(); it != flatCellMap.end(); ++it )
                                {
                                    if( it->second < minElevFound )
                                    {
                                        it->second = minElevFound;
                                        demBuffer[ it->first ] = minElevFound;
                                    }
                                }
                                // Determine if the found cell flows away from the flat area or not
                                curY = indexOfLowestPoint / m_iCellRangeX;
                                curX = indexOfLowestPoint - curY * m_iCellRangeX;
                                GetDirectionalIndices( static_cast< Rusle2Direction >( aspectBuffer[ indexOfLowestPoint ] ), curX, curY, destX, destY );
                                int indexOfFlowedToCell = destX + destY * m_iCellRangeX;
                                if( flatCellMap.find( indexOfFlowedToCell ) != flatCellMap.end() )
                                {
                                    // The found cell is part of the depression we are trying to resolve so move low cell from border to flat and set direction to 0
                                    aspectBuffer[ itLowCell->first ] = 0;
                                    flatCellMap[ itLowCell->first ] = itLowCell->second;
                                    borderCellMap.erase( itLowCell );
                                    // Set flag to continue the loop looking for higher spill points unless we have reached the maximum allowed fill level
                                    if( ! reachedMaxFillLevel )
                                    {
                                        NotYetSatisfied = true;
                                    }
                                }
                                else
                                {
                                    // If the found cell flows away from the flat area then we proceed to map the flow of each flat cell toward the found cell
                                    for ( int direction = NORTH; direction <= NORTHWEST; ++direction )
                                    {
                                        GetDirectionalIndices( static_cast< Rusle2Direction >( direction ), curX, curY, destX, destY );
                                        if( aspectTest->TestPoint( destX, destY ) )
                                        {
                                            int tempIndex = destX + destY * m_iCellRangeX;
                                            if( flatCellMap.find( tempIndex ) != flatCellMap.end() )
                                            {
                                                aspectBuffer[ tempIndex ] = direction + 4 > 8 ? direction - 4: direction + 4;
                                            }
                                        }
                                    }

                                    bool stillLinking = true;
#ifdef RUSLE2_FLAT_RECURSION_SOLUTION
                                    bool recursive = true;
#else
                                    bool recursive = false;
#endif
                                    while ( stillLinking )
                                    {
                                        stillLinking = false;
                                        for ( std::map< int, float >::iterator it = flatCellMap.begin(); it != flatCellMap.end(); ++it )
                                        {
                                            if( aspectBuffer[ it->first ] != 0 )
                                            {
                                                if( FindAdjacentFlatCellsAndDirect( it->first, aspectBuffer, flatCellMap, recursive ) > 0 )
                                                {
                                                    stillLinking = true;
                                                }
                                            }
                                        }
                                    }
                                    ++flatsFilled;
                                }
                            }
                        }
                    }
                }
            }
        }

        if( flatsFilled )
        {
            cplErr = demBand->RasterIO( GF_Write, m_iCellMinX, m_iCellMinY, m_iCellRangeX, m_iCellRangeY, demBuffer, m_iCellRangeX, m_iCellRangeY, GDT_Float32, 0, 0 );
            cplErr2 = aspectBand->RasterIO( GF_Write, 0, 0, m_iCellRangeX, m_iCellRangeY, aspectBuffer, m_iCellRangeX, m_iCellRangeY, GDT_Byte, 0, 0 );
        }
        if( cplErr == CE_Failure || cplErr2 == CE_Failure )
        {
            errorMsg = CPLGetLastErrorMsg();
            std::cout << errorMsg << std::endl;
            CPLFree( demBuffer );
            CPLFree( aspectBuffer );
            delete demTest;
            delete aspectTest;
            return false;
        }

        m_aspectDEM->FlushCache();
        m_resampleDEM->FlushCache();
        CPLFree( demBuffer );
        delete demTest;
    }

    BuildArrowList( aspectBuffer, aspectTestNodata, aspectNodataValue );

    CPLFree( aspectBuffer );
    delete aspectTest;

    GByte max = 0, min = 0; 
    success = FindBandValidValueRange( aspectBand, min, max );
    if( !success )
    {
        std::cout << "No No non-NULL data was found in Aspect map." << std::endl;
    }

    std::cout << "Range of aspect values found " << (int)min << " to " << (int)max << std::endl;
    std::cout << "Number of flat spots corrected = " << flatsFilled << std::endl;
    std::cout << "Flat spot flow correction complete.\n" << std::endl;
    return true;
}

void Rusle2::BuildArrowList( const GByte *aspectBuffer, const bool& testNodata, const GByte& nodataValue )
{
    for ( int y = 0; y < m_iCellRangeY; ++y )
    {
        int index = y * m_iCellRangeX;
        for ( int x = 0; x < m_iCellRangeX; ++x, ++index )
        {
            if( ! testNodata || aspectBuffer[ index ] != nodataValue )
            {
                m_arrowList.push_back( Rusle2Arrow( x, y, aspectBuffer[ index ] ) );
            }
        }
    }
}

int Rusle2::FindAdjacentFlatCellsAndDirect( const int& centerCell, GByte *aspectBuffer, const std::map< int, float >& flatCellMap, const bool& recursive ) const
{

    int flatsFixed = 0;
    int curX, curY, destX, destY;
    curY = centerCell / m_iCellRangeX;
    curX = centerCell - curY * m_iCellRangeX;
    for ( int direction = NORTH; direction <= NORTHWEST; ++direction )
    {
        GetDirectionalIndices( static_cast< Rusle2Direction >( direction ), curX, curY, destX, destY );
        int tempIndex = destX + destY * m_iCellRangeX;
        if( flatCellMap.find( tempIndex ) != flatCellMap.end() )
        {
            if( aspectBuffer[ tempIndex ] == 0 )
            {
                aspectBuffer[ tempIndex ] = direction + 4 > 8 ? direction - 4: direction + 4;
                ++flatsFixed;
                if( recursive )
                {
                    flatsFixed += FindAdjacentFlatCellsAndDirect( tempIndex, aspectBuffer, flatCellMap, recursive );
                }
            }
        }
    }

    return flatsFixed;
}

GDALDataset *Rusle2::BuildProximalInflowDEM() const
{
    CPLErr cplErr;
    const char* errorMsg;

    GDALDataset *proximalInflowDataset = m_gridShaper->CreateFarmSizeDataset( "proximalinflow", m_aspectDEM, m_resampleDEM );
    if( proximalInflowDataset == NULL )
    {
        return NULL;
    }

    // Get the output band
    GDALRasterBand* proximalBand = proximalInflowDataset->GetRasterBand( 1 );
    if( proximalBand == NULL || proximalBand->GetBand() != 1 )
    {
        std::cout << "Could not obtain correct band number in proximal inflow data set" << std::endl;
        GDALClose ( proximalInflowDataset );
        return NULL;
    }
    GByte ProximalNodataValue = static_cast<GByte>( proximalBand->GetMaximum() );
    proximalBand->SetNoDataValue( ProximalNodataValue );

    // Get the nodata value from the aspect band
    GDALRasterBand* aspectBand = m_aspectDEM->GetRasterBand( 1 );
    if( aspectBand == NULL || aspectBand->GetBand() != 1 )
    {
        std::cout << "Could not obtain correct band in aspect data set" << std::endl;
        GDALClose ( proximalInflowDataset );
        return NULL;
    }
    int success = 0;
    bool testNodata = false;
    GByte nodataValue = static_cast<GByte>(aspectBand->GetNoDataValue( &success ) );
    if( success )
    {
        testNodata = true;
    }

    // Create buffer array to read three lines of data m_iCellRangeX wide
    GByte* threeScanLines;
    threeScanLines = (GByte *) CPLMalloc( sizeof(GByte) * m_iCellRangeX * 3 );
    GByte* inputAspect[3] = { &threeScanLines[0], &threeScanLines[m_iCellRangeX], &threeScanLines[m_iCellRangeX * 2] };
    ValidRasterPointTest< GByte >* aspectTest = new ValidRasterPointTest< GByte >( m_iCellRangeX, 3, testNodata, nodataValue, threeScanLines );

    // Create buffer array to store one line of slope direction m_iCellRangeX wide
    GByte* oneInflowLine;
    oneInflowLine = (GByte *) CPLMalloc( sizeof(GByte) * m_iCellRangeX );

    int pitCells = 0;

    for ( int outInflowLine = 0; outInflowLine < m_iCellRangeY; ++outInflowLine )
    {
        int lineOne = outInflowLine - 1;
        int lineThree = outInflowLine + 1;
        // Outer loop reads three lines from source DEM
        if( lineOne >= 0 )
        {
            if( lineThree < m_iCellRangeY )
            {
                // Read three lines
                cplErr = aspectBand->RasterIO( GF_Read, 0, lineOne, m_iCellRangeX, 3, threeScanLines, m_iCellRangeX, 3, GDT_Byte, 0, 0 );
            }
            else
            {
                // Read two lines into first two rows
                cplErr = aspectBand->RasterIO( GF_Read, 0, lineOne, m_iCellRangeX, 2, threeScanLines, m_iCellRangeX, 2, GDT_Byte, 0, 0 );
            }
        }
        else
        {
            if( lineThree < m_iCellRangeY )
            {
                // Read two lines into last two rows
                cplErr = aspectBand->RasterIO( GF_Read, 0, outInflowLine, m_iCellRangeX, 2, inputAspect[1], m_iCellRangeX, 2, GDT_Byte, 0, 0 );
            }
            else
            {
                // Read one line into middle row
                cplErr = aspectBand->RasterIO( GF_Read, 0, outInflowLine, m_iCellRangeX, 1, inputAspect[1], m_iCellRangeX, 1, GDT_Byte, 0, 0 );
            }
        }
        if( cplErr == CE_Failure )
        {
            errorMsg = CPLGetLastErrorMsg();
            std::cout << "BuildProximalInflowDEM error " << errorMsg << std::endl;
            GDALClose ( proximalInflowDataset );
            CPLFree( threeScanLines );
            CPLFree( oneInflowLine );
            delete aspectTest;
            return NULL;
        }

        for ( int outPixel = 0; outPixel < m_iCellRangeX; ++outPixel )
        {
            int inFlowingCells = 0;
            if( aspectTest->TestPoint( outPixel, 1 ) )
            {
                int pixelOne = outPixel - 1;
                int pixelThree = outPixel + 1;

                if( lineOne >= 0 )
                {
                    if( aspectTest->TestPoint( pixelOne, 0 ) && inputAspect[0][pixelOne] == SOUTHEAST ) ++inFlowingCells;

                    if( aspectTest->TestPoint( outPixel, 0 ) && inputAspect[0][outPixel] == SOUTH  ) ++inFlowingCells;

                    if( aspectTest->TestPoint( pixelThree, 0 ) && inputAspect[0][pixelThree] == SOUTHWEST  ) ++inFlowingCells;
                }

                if( aspectTest->TestPoint( pixelOne, 1 ) && inputAspect[1][pixelOne] == EAST  ) ++inFlowingCells;

                if( aspectTest->TestPoint( pixelThree, 1 ) && inputAspect[1][pixelThree] == WEST  ) ++inFlowingCells;

                if( lineThree < m_iCellRangeY)
                {
                    if( aspectTest->TestPoint( pixelOne, 2 ) && inputAspect[2][pixelOne] == NORTHEAST  ) ++inFlowingCells;

                    if( aspectTest->TestPoint( outPixel, 2 ) && inputAspect[2][outPixel] == NORTH  ) ++inFlowingCells;

                    if( aspectTest->TestPoint( pixelThree, 2 ) && inputAspect[2][pixelThree] == NORTHWEST  ) ++inFlowingCells;
                }
            }
            else
            {
                inFlowingCells = ProximalNodataValue;
            }
            // Store data
            oneInflowLine[outPixel] = inFlowingCells;

            if( inFlowingCells == 8 )
            {
                ++pitCells;
            }
        }
        // Outer loop writes line of slope direction to raster
        cplErr = proximalBand->RasterIO( GF_Write, 0, outInflowLine, m_iCellRangeX, 1, oneInflowLine, m_iCellRangeX, 1, GDT_Byte, 0, 0 );
        if( cplErr == CE_Failure )
        {
            errorMsg = CPLGetLastErrorMsg();
            std::cout << "BuildProximalInflowDEM error 2 " << errorMsg << std::endl;
            GDALClose( proximalInflowDataset );
            CPLFree( threeScanLines );
            CPLFree( oneInflowLine );
            delete aspectTest;
            return NULL;
        }

    }

    proximalInflowDataset->FlushCache();
    CPLFree( threeScanLines );
    CPLFree( oneInflowLine );
    delete aspectTest;

    GByte max = 0, min = 0; 
    success = FindBandValidValueRange( proximalBand, min, max );
    if( ! success )
    {
        std::cout << "No No non-NULL data was created in proximal inflow map." << std::endl;
    }

    std::cout << "Range of proximal inflow values found " << (int)min << " to " << (int)max << std::endl;
    std::cout << "Number of cells where all flow directions converge = " << pitCells << std::endl;
    std::cout << "Proximal inflow map complete.\n" << std::endl;
    return proximalInflowDataset;
}

GDALDataset* Rusle2::BuildTotalInflowDEM()
{
    CPLErr cplErr;
    const char* errorMsg = NULL;
    const std::string totalInflowFileDesc( "totalinflow" );

    GDALDataset* totalInflowDataset = m_gridShaper->CreateFarmSizeDataset( totalInflowFileDesc, NULL, m_resampleDEM, GDT_UInt32 );
    if( totalInflowDataset == NULL )
    {
        return NULL;
    }

    // Get the output band
    GDALRasterBand* totalBand = totalInflowDataset->GetRasterBand( 1 );
    if( totalBand == NULL || totalBand->GetBand() != 1 )
    {
        std::cout << "Could not obtain correct band number in total inflow data set" << std::endl;
        GDALClose ( totalInflowDataset );
        return NULL;
    }
    GUInt32 totalNodataValue = static_cast<GUInt32>( totalBand->GetMaximum() );
    totalBand->SetNoDataValue( totalNodataValue );
    totalBand->Fill( 0 );

    // Get the band and nodata value from the aspect band
    GDALRasterBand* aspectBand = m_aspectDEM->GetRasterBand( 1 );
    if( aspectBand == NULL || aspectBand->GetBand() != 1 )
    {
        std::cout << "Could not obtain correct band in aspect data set" << std::endl;
        GDALClose ( totalInflowDataset );
        return NULL;
    }
    int success = 0;
    bool aspectTestNodata = false;
    GByte aspectNodataValue = static_cast<GByte>( aspectBand->GetNoDataValue( &success ) );
    if( success )
    {
        aspectTestNodata = true;
    }

    GByte* aspectBuffer = NULL;
    GByte* touchedBuffer = NULL;
    GUInt32* totalFlowBuffer = NULL;
    
    aspectBuffer = (GByte *) CPLMalloc( sizeof(GByte) * m_iCellRangeX * m_iCellRangeY );
    ValidRasterPointTest< GByte >* aspectTest = new ValidRasterPointTest< GByte >( m_iCellRangeX, m_iCellRangeY, aspectTestNodata, aspectNodataValue, aspectBuffer );

    touchedBuffer = (GByte *) CPLMalloc( sizeof(GByte) * m_iCellRangeX * m_iCellRangeY );
    memset( touchedBuffer, 0, sizeof(GByte) * m_iCellRangeX * m_iCellRangeY );
    
    totalFlowBuffer = (GUInt32 *) CPLMalloc( sizeof(GUInt32) * m_iCellRangeX * m_iCellRangeY );
    memset( totalFlowBuffer, 0, sizeof(GUInt32) * m_iCellRangeX * m_iCellRangeY );

    // populate buffers from rasters
    cplErr = aspectBand->RasterIO( GF_Read, 0, 0, m_iCellRangeX, m_iCellRangeY, aspectBuffer, m_iCellRangeX, m_iCellRangeY, GDT_Byte, 0, 0 );
    if( cplErr == CE_Failure )
    {
        errorMsg = CPLGetLastErrorMsg();
        std::cout << "BuildTotalInflowDEM error " << errorMsg << std::endl;
        GDALClose ( totalInflowDataset );
        CPLFree( aspectBuffer );
        CPLFree( touchedBuffer );
        CPLFree( totalFlowBuffer );
        delete aspectTest;
        return NULL;
    }

    // Get down to business
    for ( int initialY = 0; initialY < m_iCellRangeY; ++initialY )
    {
        for (int initialX = 0; initialX < m_iCellRangeX; ++initialX )
        {
#if 1
            GUInt32 cellInflow = GetTotalInflow( initialX, initialY, initialX, initialY, aspectBuffer, aspectTest );
            int destIndex = initialX + initialY * m_iCellRangeX;
            totalFlowBuffer[ destIndex ] = cellInflow;
#else
            bool stillFlowing = true;
            int traceX, traceY, touchedMinX, touchedMaxX, touchedMinY, touchedMaxY;
            traceX = touchedMinX = touchedMaxX = initialX;
            traceY = touchedMinY = touchedMaxY = initialY;
            while ( stillFlowing )
            {
                if( aspectTest->TestPoint( traceX, traceY ) )
                {
                    int destX, destY;
                    int traceIndex = traceX + traceY * m_iCellRangeX;
                    int direction = aspectBuffer[ traceIndex ];
                    if( !GetDirectionalIndices( static_cast< Rusle2Direction >( direction ), traceX, traceY, destX, destY ) )
                    {
                        stillFlowing = false;
                    }
                    if( stillFlowing )
                    {
                        if( aspectTest->TestPoint( destX, destY ) )
                        {
                            int destIndex = destX + destY * m_iCellRangeX;
                            if( touchedBuffer[ destIndex ] )
                            {
                                // circular path, break now or forever remain in a loop
                                stillFlowing = false;
                            }
                            else
                            {
                                ++totalFlowBuffer[ destIndex ];
                                if( destX < touchedMinX ) touchedMinX = destX;
                                if( destX > touchedMaxX ) touchedMaxX = destX;
                                if( destY < touchedMinY ) touchedMinY = destY;
                                if( destY > touchedMaxY ) touchedMaxY = destY;
                                touchedBuffer[ destIndex ] = 1;
                                traceX = destX;
                                traceY = destY;
                            }
                        }
                        else
                        {
                            stillFlowing = false;
                        }
                    }
                }
                else
                {
                    stillFlowing = false;
                }
            }
            // Clear touchedBuffer in region touched by last point
            for ( int touchedY = touchedMinY; touchedY <= touchedMaxY; ++touchedY )
            {
                int touchedIndex = touchedY * m_iCellRangeX + touchedMinX;
                for ( int touchedX = touchedMinX; touchedX <= touchedMaxX; ++touchedX, ++touchedIndex )
                {
                    touchedBuffer[ touchedIndex ] = 0;
                }
            }
#endif
        }
    }

    const std::string txtFileOutput = m_baseName + "totalinflow_per_cell.txt";
    std::ofstream txtTotalInflow( txtFileOutput.c_str() );
    size_t tNumCells = m_iCellRangeX * m_iCellRangeY;
    for( size_t i = 0; i < tNumCells; ++i )
    {
        txtTotalInflow << i << " " << totalFlowBuffer[ i ] << std::endl;
    }
    cplErr = totalBand->RasterIO( GF_Write, 0, 0, m_iCellRangeX, m_iCellRangeY, totalFlowBuffer, m_iCellRangeX, m_iCellRangeY, GDT_UInt32, 0, 0 );
    if( cplErr )
    {
        errorMsg = CPLGetLastErrorMsg();
        std::cout << "BuildTotalInflowDEM error 2 " << errorMsg << std::endl;
        GDALClose ( totalInflowDataset );
        CPLFree( aspectBuffer );
        CPLFree( totalFlowBuffer );
        CPLFree( touchedBuffer );
        delete aspectTest;
        return NULL;
    }

    totalInflowDataset->FlushCache();
    CPLFree( aspectBuffer );
    CPLFree( totalFlowBuffer );
    delete aspectTest;

    GUInt32 max = 0, min = 0; 
    success = FindBandValidValueRange( totalBand, min, max );
    if( ! success )
    {
        std::cout << "No No non-NULL data was created in total inflow map." << std::endl;
    }

    // compute area of a DEM cell in acres
    double transforms[6];
    totalInflowDataset->GetGeoTransform( transforms );
    // we are going to assume meters since everything is EPSG 3857.
    double areaEPSGMetersPerCell = fabs( transforms[1] * transforms[5] );
    //Area factor
    //Since the DEM layer is 9 meters^2 in physical space but has an
    //EPSG area of 16 meters^2.
    double areaMetersPerEPSGArea = 9.0 / 16.0;
    // Square meters per acre from http://en.wikipedia.org/wiki/Acre
    double acresPerSqMeter = 1.0 / 4046.8564224;
    double acresPerCell = areaEPSGMetersPerCell * areaMetersPerEPSGArea * acresPerSqMeter;
    m_channelMinimumInflow = static_cast<int>( m_channelDrainageAreaAcres / acresPerCell );


    std::cout << "Range of total inflow values found " << (int)min << " to " << (int)max << std::endl;
    std::cout << "Total flow map complete.\n" << std::endl;
    return totalInflowDataset;
}

GDALDataset* Rusle2::BuildDrainageChannelDEM() const
{
    CPLErr cplErr;
    const char*errorMsg;

    GDALDataset* drainageChannelDataset = m_gridShaper->CreateFarmSizeDataset( "drainagechannel", m_aspectDEM, m_resampleDEM );
    if( drainageChannelDataset == NULL )
    {
        return NULL;
    }
    // Get the output band
    GDALRasterBand* channelBand = drainageChannelDataset->GetRasterBand( 1 );
    if( channelBand == NULL || channelBand->GetBand() != 1 )
    {
        std::cout << "Could not obtain correct band number in channel data set" << std::endl;
        GDALClose ( drainageChannelDataset );
        return NULL;
    }
    GByte channelFill = static_cast<GByte>( channelBand->GetMaximum() * .5 );


    // Get the input band
    GDALRasterBand* totalBand = m_totalInflowDEM->GetRasterBand( 1 );
    if( totalBand == NULL || totalBand->GetBand() != 1 )
    {
        std::cout << "Could not obtain correct band number in total inflow data set" << std::endl;
        GDALClose ( drainageChannelDataset );
        return NULL;
    }
    int success = 0;
    bool totalFlowTestNodata = false;
    GUInt32 totalFlowNodataValue = static_cast<GUInt32>(totalBand->GetNoDataValue( &success ) );
    if( success )
    {
        totalFlowTestNodata = true;
    }

    GUInt32* totalFlowBuffer;
    GByte* channelBuffer;

    totalFlowBuffer = (GUInt32 *) CPLMalloc( sizeof(GUInt32) * m_iCellRangeX );
    ValidRasterPointTest< GUInt32 >* totalFlowTest = new ValidRasterPointTest< GUInt32 >( m_iCellRangeX, 1, totalFlowTestNodata, totalFlowNodataValue, totalFlowBuffer );

    channelBuffer = (GByte *) CPLMalloc( sizeof(GByte) * m_iCellRangeX );

    for ( int channelLine = 0; channelLine < m_iCellRangeY; ++channelLine )
    {
        cplErr = totalBand->RasterIO( GF_Read, 0, channelLine, m_iCellRangeX, 1, totalFlowBuffer, m_iCellRangeX, 1, GDT_UInt32, 0, 0 );
        if( cplErr == CE_Failure )
        {
            errorMsg = CPLGetLastErrorMsg();
            std::cout << "BuildDrainageChannelDEM error " << errorMsg << std::endl;
            GDALClose ( drainageChannelDataset );
            CPLFree( totalFlowBuffer );
            CPLFree( channelBuffer );
            delete totalFlowTest;
            return NULL;
        }
        for ( int channelPoint = 0; channelPoint < m_iCellRangeX; ++channelPoint )
        {
            if( totalFlowTest->TestPoint( channelPoint, 0 ) )
            {
                if( totalFlowBuffer[ channelPoint ] >= m_channelMinimumInflow )
                {
                    channelBuffer[ channelPoint ] = 1;//channelFill;
                }
                else
                {
                    channelBuffer[ channelPoint ] = 0;
                }
            }
            else
            {
                channelBuffer[ channelPoint ] = 0;
            }
        }
        cplErr = channelBand->RasterIO( GF_Write, 0, channelLine, m_iCellRangeX, 1, channelBuffer, m_iCellRangeX, 1, GDT_Byte, 0, 0 );
        if( cplErr == CE_Failure )
        {
            errorMsg = CPLGetLastErrorMsg();
            std::cout << "BuildDrainageChannelDEM error " << errorMsg << std::endl;
            GDALClose ( drainageChannelDataset );
            CPLFree( totalFlowBuffer );
            CPLFree( channelBuffer );
            delete totalFlowTest;
            return NULL;
        }
    }

    drainageChannelDataset->FlushCache();
    CPLFree( totalFlowBuffer );
    CPLFree( channelBuffer );
    delete totalFlowTest;

    double transformFactors[6];
    drainageChannelDataset->GetGeoTransform( transformFactors );

    double areaContributing = fabs( m_channelMinimumInflow * transformFactors[1] * transformFactors[5] );
    std::cout << "Cells contributing to flow in order to be a considered channel are " << m_channelMinimumInflow << std::endl;
    std::cout << "Area contributing to be considered a channel is " << areaContributing << " square DEM areal units." << std::endl;
    std::cout << "Drainage channel map complete.\n" << std::endl;
    return drainageChannelDataset;
}

OGRDataSource* Rusle2::CreateArrowDatabase( const std::string& arrowFilename, GDALDataset* copyRasterSettings ) const
{
    std::string totalName = m_baseName + arrowFilename;

    std::cout << "Building drainage direction arrow database." << std::endl;

    OGRSFDriver* driver = m_farmShape->GetDriver();
    FILE* testOpen = fopen( totalName.c_str(), "r" );
    if( testOpen )
    {
        fclose( testOpen );
        driver->DeleteDataSource( totalName.c_str() );
    }
    OGRDataSource* arrowSource;

    arrowSource = driver->CreateDataSource( totalName.c_str(), NULL );
    if( arrowSource == NULL )
    {
        std::cout << "Creation of arrow file failed.\n";
        return NULL;
    }

    OGRSpatialReference demRef;
    #ifndef FORCE_VECTOR_TO_DEM_FROM_SERVER_EPSG_NO_MATTER_WHAT
        const char* demRefWkt = NULL;
        char charVersion[1048];
        char* charPtr;

        demRefWkt = copyRasterSettings->GetProjectionRef();
        strcpy( charVersion, demRefWkt );
        charPtr = &charVersion[0];
        if( demRefWkt)
        {
            OGRErr ogrErr = demRef.importFromWkt( &charPtr );
            if( ogrErr == CE_Failure )
            {
                std::cout << "Error : " << CPLGetLastErrorMsg() << std::endl;
            }
        }
    #else
        demRef.importFromEPSG( DEM_FROM_SERVER_EPSG );
    #endif

    OGRLayer* arrowLayer;

    arrowLayer = arrowSource->CreateLayer( "Arrows", &demRef, wkbLineString, NULL );
    if( arrowLayer == NULL )
    {
        std::cout << "Layer creation failed.\n";
        OGRDataSource::DestroyDataSource( arrowSource );
        return NULL;
    }

    #ifdef RUSLE2_WRITE_NAME_FIELD
    OGRFieldDefn* nameFldDefn = new OGRFieldDefn( "Name", OFTString );
    if( nameFldDefn )
    {
        nameFldDefn->Set( "Name", OFTString, 32, 0, OJLeft );
    }
    else
    {
        std::cout << "Creating field definitions failed." << std::endl;
        OGRDataSource::DestroyDataSource( arrowSource );
        return NULL;
    }
    if( arrowLayer->CreateField( nameFldDefn ) != OGRERR_NONE )
    {
        std::cout << "Creating Arrow Name field failed." << std::endl;
        OGRDataSource::DestroyDataSource( arrowSource );
        return NULL;
    }
    #endif // RUSLE2_WRITE_NAME_FIELD

#ifdef RUSLE2_WRITE_NAME_FIELD
    OGRFeatureDefn* ogrFeatureDefn = new OGRFeatureDefn( "Arrow" );
    char nameString[64];
    int nameIndex;
    if( ogrFeatureDefn )
    {
        // Add Field Definitions to Feature Definition
        ogrFeatureDefn->AddFieldDefn( nameFldDefn );
        // Get the field Indices from the Feature Def.
        nameIndex = ogrFeatureDefn->GetFieldIndex( nameFldDefn->GetNameRef() );
    }
#endif // RUSLE2_WRITE_NAME_FIELD

    double projTransforms[6];
    copyRasterSettings->GetGeoTransform( projTransforms );

    ArrowDatabase arrowDB( projTransforms[0], projTransforms[3], projTransforms[1], projTransforms[5] );

    for ( ArrowList::const_iterator ait = m_arrowList.begin(); ait != m_arrowList.end(); ++ait )
    {
        Rusle2Arrow arrow = *ait;
        
        arrowDB.computeArrowParts( arrow );

        OGRFeature* arrowShaftFeature;
        OGRFeature* arrowTip1Feature;
        OGRFeature* arrowTip2Feature;

        arrowShaftFeature = OGRFeature::CreateFeature( arrowLayer->GetLayerDefn() );
        arrowTip1Feature = OGRFeature::CreateFeature( arrowLayer->GetLayerDefn() );
        arrowTip2Feature = OGRFeature::CreateFeature( arrowLayer->GetLayerDefn() );

        OGRLineString arrowShaft;
        OGRLineString arrowTip1;
        OGRLineString arrowTip2;
        OGRPoint lineStart, lineEnd;

        // arrow shaft
#ifdef RUSLE2_WRITE_NAME_FIELD
        sprintf( nameString, "%d, %d", arrow.m_cellX, arrow.m_cellY );
        arrowShaftFeature->SetField( nameIndex, nameString );
#endif // RUSLE2_WRITE_NAME_FIELD
        lineStart.setX( arrow.m_shaftEndX );
        lineStart.setY( arrow.m_shaftEndY );
        lineEnd.setX( arrow.m_tipX );
        lineEnd.setY( arrow.m_tipY );
        arrowShaft.addPoint( &lineStart );
        arrowShaft.addPoint( &lineEnd );
        arrowShaftFeature->SetGeometry( &arrowShaft ); 
        if( arrowLayer->CreateFeature( arrowShaftFeature ) != OGRERR_NONE )
        {
            std::cout << "Failed to create feature in arrow shapefile.\n";
            OGRDataSource::DestroyDataSource( arrowSource );
            return NULL;
        }
        
        // arrow tip 1
        #ifdef RUSLE2_WRITE_NAME_FIELD
        arrowTip1Feature->SetField( nameIndex, nameString );
        #endif // RUSLE2_WRITE_NAME_FIELD
        lineStart.setX( arrow.m_tip1EndX );
        lineStart.setY( arrow.m_tip1EndY );
        arrowTip1.addPoint( &lineStart );
        arrowTip1.addPoint( &lineEnd );
        arrowTip1Feature->SetGeometry( &arrowTip1 ); 
        if( arrowLayer->CreateFeature( arrowTip1Feature ) != OGRERR_NONE )
        {
            std::cout << "Failed to create feature in arrow shapefile.\n";
            OGRDataSource::DestroyDataSource( arrowSource );
            return NULL;
        }
        
        // arrow tip 2
        #ifdef RUSLE2_WRITE_NAME_FIELD
        arrowTip2Feature->SetField( nameIndex, nameString );
        #endif // RUSLE2_WRITE_NAME_FIELD
        lineStart.setX( arrow.m_tip2EndX );
        lineStart.setY( arrow.m_tip2EndY );
        arrowTip2.addPoint( &lineStart );
        arrowTip2.addPoint( &lineEnd );
        arrowTip2Feature->SetGeometry( &arrowTip2 ); 
        if( arrowLayer->CreateFeature( arrowTip2Feature ) != OGRERR_NONE )
        {
            std::cout << "Failed to create feature in arrow shapefile.\n";
            OGRDataSource::DestroyDataSource( arrowSource );
            return NULL;
        }
        

        OGRFeature::DestroyFeature( arrowShaftFeature );
        OGRFeature::DestroyFeature( arrowTip1Feature );
        OGRFeature::DestroyFeature( arrowTip2Feature );
    }

    std::cout << "Drainage direction arrow database complete.\n" << std::endl;

    return arrowSource;
}

bool Rusle2::GetDirectionalIndices( const Rusle2Direction& direction, const int& inX, const int& inY, int& outX, int& outY ) const
{
    bool resolved = true;
    
    switch ( direction )
    {
        case NORTHWEST:
            outX = inX - 1;
            outY = inY - 1;
            break;
        case NORTH:
            outX = inX;
            outY = inY - 1;
            break;
        case NORTHEAST:
            outX = inX + 1;
            outY = inY - 1;
            break;
        case WEST:
            outX = inX - 1;
            outY = inY;
            break;
        case EAST:
            outX = inX + 1;
            outY = inY;
            break;
        case SOUTHWEST:
            outX = inX - 1;
            outY = inY + 1;
            break;
        case SOUTH:
            outX = inX;
            outY = inY + 1;
            break;
        case SOUTHEAST:
            outX = inX + 1;
            outY = inY + 1;
            break;
        default:
            resolved = false;
            break;
    }

    return resolved;
}

bool Rusle2::TestFarmShapeIntersection( const OGRPolygon& poly ) const
{

    //Fetch each polygon in the farm shape dataset and test with provided polygon
    int layers = m_farmShape->GetLayerCount();
    for ( int layerCt = 0; layerCt < layers; ++layerCt )
    {
        OGRLayer* currentLayer = m_farmShape->GetLayer( layerCt );
        OGRFeature* farmFeature;
        currentLayer->ResetReading();
        while( (farmFeature = currentLayer->GetNextFeature()) != NULL )
        {
            OGRGeometry* farmGeometry;

            farmGeometry = farmFeature->GetGeometryRef();
            if( farmGeometry != NULL 
                && wkbFlatten(farmGeometry->getGeometryType()) == wkbPolygon )
            {
                OGRPolygon* farmPoly = (OGRPolygon *) farmGeometry;

                //OGREnvelope farmEnv, polyEnv;
                //farmPoly->getEnvelope( &farmEnv );
                //poly.getEnvelope( &polyEnv );
                //std::cout << "Farm " << farmEnv.MinX << " " << farmEnv.MaxX << " " << farmEnv.MinY << " " << farmEnv.MaxY << std::endl;
                //std::cout << "Cell " << polyEnv.MinX << " " << polyEnv.MaxX << " " << polyEnv.MinY << " " << polyEnv.MaxY << std::endl;
                if( poly.Intersects( farmPoly ) )
                {
                    return true;
                }
            }
        }
    }
    return false;
}

OGRCoordinateTransformation* Rusle2::GetFarmTransformation() const
{

    if( m_gridShaper )
    {
        return m_gridShaper->GetFarmTransformation();
    }
    return NULL;
}
    
GUInt32 Rusle2::GetTotalInflow( int currentX, int currentY, int startX, int startY, GByte* aspectBuffer, ValidRasterPointTest< GByte >* aspectTest )
{
    GUInt32 cellCount = 0;
    //Find all of my neighbors
    for( int j = currentY - 1; j <= currentY + 1; ++j )
    {
        for( int i = currentX - 1; i <= currentX + 1; ++i )
        {
            //Do not consider the currentX,Y cell
            if( (i == currentX) && (j == currentY) )
            {
                continue;
            }
            
            //If the cell is not the start cell - if this is not a sink
            if( (i == startX) && (j == startY) )
            {
                continue;
            }

            //Is this a valid point
            if( aspectTest->TestPoint( i, j ) )
            {
                int destX, destY;
                int traceIndex = i + j * m_iCellRangeX;
                int direction = aspectBuffer[ traceIndex ];
                //Get the cell that the neighbor cell feeds
                if( GetDirectionalIndices( static_cast< Rusle2Direction >( direction ), i, j, destX, destY ) )
                {
                    //Does the neighbor feed currentX,currentY cell
                    if( (destX == currentX) && (destY == currentY) )
                    {
                        //This cell adds to the currentX,Y cell
                        cellCount += 1;
                        //Do any others that depend on this cell
                        cellCount += GetTotalInflow( i, j, startX, startY, aspectBuffer, aspectTest );
                    }
                }
            }
        }
    }
    
    return cellCount;
}

void Rusle2::BuildAspectRaster2( GDALDataset* demDataSet, GDALDataset* aspectDataSet )
{
    /*---------------------------------------
     * Open Dataset and get raster band (assuming it is band #1)
     */
    /*poDataset = (GDALDataset *) GDALOpen( pszFilename, GA_ReadOnly );
    if( poDataset == NULL )
    {
        printf( "Couldn't open dataset %s\n",
               pszFilename );
    }*/
    GDALRasterBand* poBand = demDataSet->GetRasterBand( 1 );
    //poDataset->GetGeoTransform( adfGeoTransform );
    
    // Variables related to input dataset
    //const double cellsizeY = adfGeoTransform[5];
    //const double cellsizeX = adfGeoTransform[1];
    //const float nullValue = (float) poBand->GetNoDataValue();
    //const float aspectNullValue = -9999.;

    const float degrees_to_radians = 3.14159 / 180.0;
    const float radians_to_degrees = 180.0 / 3.14159;
    
    float       dx;
    float       dy;
    float       aspect;
    
    /* -----------------------------------------
     * Open up the output datasets and copy over relevant metadata
     */
    //GDALDriver *poDriver;
    //poDriver = GetGDALDriverManager()->GetDriverByName(pszFormat);
    GDALDataset* poAspectDS = aspectDataSet;
    //char **papszOptions = NULL;
    /*
     * Open slope output map
     */
    //poAspectDS = poDriver->Create(pszAspectFilename,nXSize,nYSize,1,GDT_Float32,
    //                              papszOptions );
    //poAspectDS->SetGeoTransform( adfGeoTransform );
    //poAspectDS->SetProjection( poDataset->GetProjectionRef() );
    GDALRasterBand* poAspectBand = poAspectDS->GetRasterBand(1);
    //poAspectBand->SetNoDataValue(aspectNullValue);
    
    int inputStartX = m_iCellMinX;
    int inputStartY = m_iCellMinY;
    int inputEndX = m_iCellMaxX;
    int inputEndY = m_iCellMaxY;
    std::cout << m_iCellMinX << " " << m_iCellMinY << " " << m_iCellMaxX << " " << m_iCellMaxY << " " << m_iCellRangeX << std::endl;
    //const int   nXSize = m_iCellRangeX+2;
    //const int   nYSize = m_iCellRangeY+2;;
    
    float* win = (float *) CPLMalloc(sizeof(float)*9);
    GByte* aspectBuf = (GByte *) CPLMalloc(sizeof(GByte)*(m_iCellRangeX));

    /* ------------------------------------------
     * Get 3x3 window around each cell
     * (where the cell in question is #4)
     *
     *                 0 1 2
     *                 3 4 5
     *                 6 7 8
     *  and calculate slope and aspect
     */
    for ( size_t j = inputStartY; j < inputEndY; j++)
    {
        for ( size_t i = inputStartX; i < inputEndX; i++)
        {
            //containsNull = 0;
            
            // Read in 3x3 window
            CPLErr cplErr = poBand->RasterIO( GF_Read, i-1, j-1, 3, 3,
                                      win, 3, 3, GDT_Float32,
                                      0, 0 );
            if( cplErr == CE_Failure )
            {
                std::cout << "BuildAspectDEM error 4 " << CPLGetLastErrorMsg() << std::endl;
            }

            // Check if window has null value
            /*for ( n = 0; n <= 8; n++)
            {
                if(win[n] == nullValue)
                {
                    containsNull = 1;
                    break;
                }
            }*/
            
            /*if (containsNull == 1)
            {
                // We have nulls so write nullValues and move on
                aspectBuf[j-inputStartY] = nullValue;
                continue;
            }
            else*/
            {
                //337.5 - 22.5 - EAST
                //22.5 - 67.5 - NE
                //67.5 - 112.5 - NORTH
                //112.5 - 157.5 - NW
                //157.5 - 202.5 - WEST
                //202.5 - 247.5 - SW
                //247.5 - 292.5 - SOUTH
                //292.5 - 337.5 - SE
                // We have a valid 3x3 window to compute aspect
                
                dx = ((win[2] + win[5] + win[5] + win[8]) -
                      (win[0] + win[3] + win[3] + win[6]));
                
                dy = ((win[6] + win[7] + win[7] + win[8]) -
                      (win[0] + win[1] + win[1] + win[2]));
                
                //aspect = atan2(dy/8.0,-1.0*dx/8.0) / degrees_to_radians;
                //std::cout << " ** " << aspect << std::endl;
                aspect = atan2(dy,-1.0*dx) / degrees_to_radians;
                //if (dx == 0 && dy == 0)
                //{
                //    std::cout << aspect << std::endl;

                    /* Flat area */
                    //aspect = fDstNoDataValue;
                //aspect = FLAT;
                //}
                //else
                {
                    if (aspect < 0)
                        aspect += 360.0;
                }
                
                if (aspect == 360.0) 
                    aspect = 0.0;

                /*if (dx == 0)
                {
                    if (dy > 0)
                        aspect = 0.0;
                    else if (dy < 0)
                        aspect = 180.0;
                    else
                        aspect = aspectNullValue;
                }
                else
                {
                    if (aspect > 90.0)
                        aspect = 450.0 - aspect;
                    else
                        aspect = 90.0 - aspect;
                }
                
                if (aspect == 360.0)
                    aspect = 0.0;*/
                
                //337.5 - 22.5 - EAST
                //22.5 - 67.5 - NE
                //67.5 - 112.5 - NORTH
                //112.5 - 157.5 - NW
                //157.5 - 202.5 - WEST
                //202.5 - 247.5 - SW
                //247.5 - 292.5 - SOUTH
                //292.5 - 337.5 - SE

                if (dx == 0 && dy == 0)
                {
                    aspectBuf[i-inputStartX] = FLAT;
                }
                else if( ((aspect > 337.5) && (aspect < 360.0)) || ((aspect >= 0.0) && (aspect <= 22.5)) )
                {
                    aspectBuf[i-inputStartX] = EAST;
                }
                else if( (aspect > 22.5) && (aspect <= 67.5) )
                {
                    aspectBuf[i-inputStartX] = NORTHEAST;
                }
                else if( (aspect > 67.5) && (aspect <= 112.5) )
                {
                    aspectBuf[i-inputStartX] = NORTH;
                }
                else if( (aspect > 112.5) && (aspect <= 157.5) )
                {
                    aspectBuf[i-inputStartX] = NORTHWEST;
                }
                else if( (aspect > 157.5) && (aspect <= 202.5) )
                {
                    aspectBuf[i-inputStartX] = WEST;
                }
                else if( (aspect > 202.5) && (aspect <= 247.5) )
                {
                    aspectBuf[i-inputStartX] = SOUTHWEST;
                }
                else if( (aspect > 247.5) && (aspect <= 292.5) )
                {
                    aspectBuf[i-inputStartX] = SOUTH;
                }
                else if( (aspect > 292.5) && (aspect <= 337.5) )
                {
                    aspectBuf[i-inputStartX] = SOUTHEAST;
                }
            }
        }
        
        /* -----------------------------------------
         * Write Line to File
         */
        
        CPLErr cplErr = poAspectBand->RasterIO( GF_Write, 0, j-inputStartY, m_iCellRangeX, 1,
                               aspectBuf, m_iCellRangeX, 1, GDT_Byte, 0, 0 );
        if( cplErr == CE_Failure )
        {
            std::cout << "BuildAspectDEM error 4 " << CPLGetLastErrorMsg() << std::endl;
        }
    }
  
    //delete poAspectDS;
}
/****************************************************************************************/

void ArrowDatabase::computeArrowParts( Rusle2Arrow &arrow )
{
    double centerX, centerY;

    computeCellCenter( arrow.m_cellX, arrow.m_cellY, centerX, centerY );

    switch ( arrow.m_direction )
    {
        case NORTHWEST:
        {
            arrow.m_tipX = centerX - m_cellSizeX * .4;
            arrow.m_tipY = centerY - m_cellSizeY * .4;
            arrow.m_shaftEndX = centerX + m_cellSizeX * .4;
            arrow.m_shaftEndY = centerY + m_cellSizeY * .4;
            arrow.m_tip1EndX = arrow.m_tipX + m_cellSizeX * .2;
            arrow.m_tip1EndY = arrow.m_tipY;
            arrow.m_tip2EndX = arrow.m_tipX;
            arrow.m_tip2EndY = arrow.m_tipY + m_cellSizeY * .2;
            break;
        }
        case NORTH:
        {
            arrow.m_tipX = centerX;
            arrow.m_tipY = centerY - m_cellSizeY * .4;
            arrow.m_shaftEndX = centerX;
            arrow.m_shaftEndY = centerY + m_cellSizeY * .4;
            arrow.m_tip1EndX = arrow.m_tipX + m_cellSizeX * .2;
            arrow.m_tip1EndY = arrow.m_tipY + m_cellSizeY * .2;
            arrow.m_tip2EndX = arrow.m_tipX - m_cellSizeX * .2;
            arrow.m_tip2EndY = arrow.m_tipY + m_cellSizeY * .2;
            break;
        }
        case NORTHEAST:
        {
            arrow.m_tipX = centerX + m_cellSizeX * .4;
            arrow.m_tipY = centerY - m_cellSizeY * .4;
            arrow.m_shaftEndX = centerX - m_cellSizeX * .4;
            arrow.m_shaftEndY = centerY + m_cellSizeY * .4;
            arrow.m_tip1EndX = arrow.m_tipX - m_cellSizeX * .2;
            arrow.m_tip1EndY = arrow.m_tipY;
            arrow.m_tip2EndX = arrow.m_tipX;
            arrow.m_tip2EndY = arrow.m_tipY + m_cellSizeY * .2;
            break;
        }

        case WEST:
        {
            arrow.m_tipX = centerX - m_cellSizeX * .4;
            arrow.m_tipY = centerY;
            arrow.m_shaftEndX = centerX + m_cellSizeX * .4;
            arrow.m_shaftEndY = centerY;
            arrow.m_tip1EndX = arrow.m_tipX + m_cellSizeX * .2;
            arrow.m_tip1EndY = arrow.m_tipY + m_cellSizeY * .2;
            arrow.m_tip2EndX = arrow.m_tipX + m_cellSizeX * .2;
            arrow.m_tip2EndY = arrow.m_tipY - m_cellSizeY * .2;
            break;
        }
        case EAST:
        {
            arrow.m_tipX = centerX + m_cellSizeX * .4;
            arrow.m_tipY = centerY;
            arrow.m_shaftEndX = centerX - m_cellSizeX * .4;
            arrow.m_shaftEndY = centerY;
            arrow.m_tip1EndX = arrow.m_tipX - m_cellSizeX * .2;
            arrow.m_tip1EndY = arrow.m_tipY + m_cellSizeY * .2;
            arrow.m_tip2EndX = arrow.m_tipX - m_cellSizeX * .2;
            arrow.m_tip2EndY = arrow.m_tipY - m_cellSizeY * .2;
            break;
        }

        case SOUTHWEST:
        {
            arrow.m_tipX = centerX - m_cellSizeX * .4;
            arrow.m_tipY = centerY + m_cellSizeY * .4;
            arrow.m_shaftEndX = centerX + m_cellSizeX * .4;
            arrow.m_shaftEndY = centerY - m_cellSizeY * .4;
            arrow.m_tip1EndX = arrow.m_tipX + m_cellSizeX * .2;
            arrow.m_tip1EndY = arrow.m_tipY;
            arrow.m_tip2EndX = arrow.m_tipX;
            arrow.m_tip2EndY = arrow.m_tipY - m_cellSizeY * .2;
            break;
        }
        case SOUTH:
        {
            arrow.m_tipX = centerX;
            arrow.m_tipY = centerY + m_cellSizeY * .4;
            arrow.m_shaftEndX = centerX;
            arrow.m_shaftEndY = centerY - m_cellSizeY * .4;
            arrow.m_tip1EndX = arrow.m_tipX + m_cellSizeX * .2;
            arrow.m_tip1EndY = arrow.m_tipY - m_cellSizeY * .2;
            arrow.m_tip2EndX = arrow.m_tipX - m_cellSizeX * .2;
            arrow.m_tip2EndY = arrow.m_tipY - m_cellSizeY * .2;
            break;
        }
        case SOUTHEAST:
        {
            arrow.m_tipX = centerX + m_cellSizeX * .4;
            arrow.m_tipY = centerY + m_cellSizeY * .4;
            arrow.m_shaftEndX = centerX - m_cellSizeX * .4;
            arrow.m_shaftEndY = centerY - m_cellSizeY * .4;
            arrow.m_tip1EndX = arrow.m_tipX - m_cellSizeX * .2;
            arrow.m_tip1EndY = arrow.m_tipY;
            arrow.m_tip2EndX = arrow.m_tipX;
            arrow.m_tip2EndY = arrow.m_tipY - m_cellSizeY * .2;
            break;
        }
        default:
        {
            arrow.m_tipX = centerX;
            arrow.m_tipY = centerY;
            arrow.m_shaftEndX = centerX;
            arrow.m_shaftEndY = centerY;
            arrow.m_tip1EndX = centerX;
            arrow.m_tip1EndY = centerY;
            arrow.m_tip2EndX = centerX;
            arrow.m_tip2EndY = centerY;
        }
    }
}

void ArrowDatabase::computeCellCenter( const int& x, const int& y, double &centerX, double &centerY ) const
{
    centerX = m_ULX + x * m_cellSizeX + m_cellSizeX * .5;
    centerY = m_ULY + y * m_cellSizeY + m_cellSizeY * .5;
}

/****************************************************************************************/

template < class T > ValidRasterPointTest<T>::ValidRasterPointTest( const int& xCells, const int& yCells, const bool& nodataTestAvailable, const T& nodataValue, const T *data ) :
    m_xCells( xCells ),
    m_yCells( yCells ),
    m_nodataTestAvailable( nodataTestAvailable ),
    m_nodataValue( nodataValue ),
    m_data( data )
{
}

template < class T > bool ValidRasterPointTest<T>::TestPoint( const int& x, const int& y ) const
{

    if( x >= 0 && x < m_xCells && y >= 0 && y < m_yCells )
    {
        return TestDataPointNotNULL( x, y );
    }

    return false;
}

template < class T > bool ValidRasterPointTest<T>::TestDataPointNotNULL( const int& x, const int& y ) const
{
    if( m_nodataTestAvailable )
    {
        T dataValue = m_data[ x + y * m_xCells ];
        if( dataValue == m_nodataValue )
        {
            return false;
        }
    }

    return true;
}

/****************************************************************************************/

template < typename T > bool FindBandValidValueRange( GDALRasterBand* band, T& min, T& max )
{
    bool Success = false;
    GDALDataType dataType = band->GetRasterDataType();
    int nodataTestAvailable;
    T nodataValue = static_cast<T>( band->GetNoDataValue( &nodataTestAvailable ) );
    int xCells = band->GetXSize();
    int yCells = band->GetYSize();
    max = static_cast<T>( band->GetMinimum() );
    min = static_cast<T>( band->GetMaximum() );
    T *oneDataLine = (T *) CPLMalloc( sizeof(T) * xCells );
    CPLErr cplErr;

    for ( int y = 0; y < yCells; ++y )
    {
        // Fetch a row of data
        cplErr = band->RasterIO( GF_Read, 0, y, xCells, 1, oneDataLine, xCells, 1, dataType, 0, 0 );
        for ( int x = 0; x < xCells; ++x )
        {
            T dataValue = oneDataLine[ x ];
            if( nodataTestAvailable )
            {
                if( dataValue == nodataValue )
                {
                    continue;
                }
            }
            if( dataValue < min )
            {
                min = dataValue;
            }
            if( dataValue > max )
            {
                max = dataValue;
            }
        }
    }

    Success = ( max > min );
    CPLFree( oneDataLine );

    return Success;
}

std::string GetFilePart( const std::string& fullPath )
{
    if( fullPath.find_last_of( "/" ) != std::string::npos )
    {
        return fullPath.substr( fullPath.find_last_of( "/" ) + 1 );
    }
    return fullPath;
}


std::string StripExtension( const std::string& fileName )
{
    std::string copy = fileName;
    if( copy.find_last_of ( "." ) != std::string::npos )
    {
        std::string::size_type idx = copy.find_last_of( "." );
        copy.resize( idx );
    }
    return copy;
}


GridShaper::GridShaper( const std::string& demFilename,  const std::string& farmFilename, const double& resolutionFactor ) :
    m_origDEM( 0 ),
    m_downsizeDEM( 0 ),
    m_resampleDEM( 0 ),
    m_farmShape( 0 ),
    m_farmTransform( 0 ),
    m_demFilename( demFilename ),
    m_farmFilename( farmFilename ),
    m_resolutionFactor( resolutionFactor ),
    m_iCellRangeX( 0 ),
    m_iCellRangeY( 0 ),
    m_reprojectAvailable( true ),
    m_hewTo3xMultipleSize( false )
{
    Init();
}

GridShaper::~GridShaper()
{
    if( m_resampleDEM && m_resampleDEM != m_downsizeDEM)
    {
        GDALClose ( m_resampleDEM );
    }
    if( m_downsizeDEM && m_downsizeDEM != m_origDEM)
    {
        GDALClose ( m_downsizeDEM );
    }
    if( m_origDEM )
    {
        GDALClose ( m_origDEM );
    }
    if( m_farmShape )
    {
        OGRDataSource::DestroyDataSource ( m_farmShape );
    }
    if( m_farmTransform )
    {
        OGRCoordinateTransformation::DestroyCT( m_farmTransform );
    }
}

void GridShaper::Init()
{
    // Register GDAL and OGR for file access
    GDALAllRegister();
    OGRRegisterAll();

    // Process farm name for later use
    std::string tempName = GetFilePart( m_farmFilename );
    m_baseName = StripExtension( tempName );
    m_baseName += "_";
    m_hewTo3xMultipleSize = ( m_resolutionFactor == 1.0 );

    OGRErr ogrErr = m_demRef.importFromEPSG( DEM_FROM_SERVER_EPSG );
    if( ogrErr != OGRERR_NONE )
    {
        m_reprojectAvailable = false;
    }
}

bool GridShaper::GetOutputGeoTransform6( double *paramArray6 ) const
{
    if( m_resampleDEM )
    {
        m_resampleDEM->GetGeoTransform( paramArray6 );
        return true;
    }
    return false;
}

bool GridShaper::GetSpatialReferenceString( std::string& refString ) const
{
    if( m_resampleDEM )
    {
        char *wkt;
        m_demRef.exportToWkt( &wkt );
        refString = wkt;
        return true;
    }
    return false;
}

int GridShaper::GetDEMXSize() const
{
    if( m_resampleDEM )
    {
        return m_resampleDEM->GetRasterXSize();
    }
    return 0;
}

int GridShaper::GetDEMYSize() const
{
    if( m_resampleDEM )
    {
        return m_resampleDEM->GetRasterYSize();
    }
    return 0;
}

GDALDataset* GridShaper::LoadAndOperate( OGRDataSource *& farmshape )
{
    m_origDEM = LoadDEMFromServer();
    if( m_origDEM == NULL )
    {
        // Fall back to DEM file on local disk.
        m_origDEM = LoadDEM();
        if( m_origDEM == NULL )
        {
            std::cout << "Could not load DEM file from the server" << std::endl;
            return NULL;
        }
    }
    
    m_farmShape = LoadShape();
    if( m_farmShape == NULL )
    {
        std::cout << "Could not load the farm shapefile" << std::endl;
        return NULL;
    }

    m_downsizeDEM = DownsizeDEM();
    if( m_downsizeDEM == NULL )
    {
        std::cout << "Could not downsize the dem" << std::endl;
        return NULL;
    }

    m_resampleDEM = ResampleDEM();
    if( m_resampleDEM == NULL )
    {
        std::cout << "Could not resample the dem" << std::endl;
        return NULL;
    }
    
    if( ! FarmShapeToDEMCells( m_resampleDEM ) )
    {
        return NULL;
    }

    std::cout << "Data loaded successfully." << std::endl;
    farmshape = m_farmShape;
    return m_resampleDEM;
}

GDALDataset *GridShaper::LoadDEM()
{
    GDALDataset  *origDataset = NULL;
    const char *demRefWkt = NULL;
    char charVersion[1048];
    char *charPtr;

    origDataset = (GDALDataset *) GDALOpen( m_demFilename.c_str(), GA_ReadOnly );
    if( origDataset == NULL )
    {
        return NULL;
    }

    demRefWkt = origDataset->GetProjectionRef();
    strcpy( charVersion, demRefWkt );
    charPtr = &charVersion[0];
    if( demRefWkt)
    {
        #ifdef RUSLE2_WKT_OUTPUT
        std::cout << "DEM Coord Sys: " << demRefWkt << std::endl;
        std::cout << std::endl;
        #endif

        #ifndef FORCE_VECTOR_TO_DEM_FROM_SERVER_EPSG_NO_MATTER_WHAT
        OGRErr ogrErr = m_demRef.importFromWkt( &charPtr );
        if( ogrErr != OGRERR_NONE )
        {
            m_reprojectAvailable = false;
        }
        #endif
    }

    int xCells = origDataset->GetRasterXSize();
    int yCells = origDataset->GetRasterYSize();
    int nBands = origDataset->GetRasterCount();
    std::cout << "Original DEM " << m_demFilename << ", X cells "
        << xCells << ", Y cells " << yCells
        << ", Bands " << nBands << std::endl;

    double transformFactors[6];
    origDataset->GetGeoTransform( transformFactors );

    std::cout << "Orig DEM Transforms " << transformFactors[0] << " "
        << transformFactors[1] << " " << transformFactors[2] << " "
        << transformFactors[3] << " " << transformFactors[4] << " "
        << transformFactors[5] << std::endl;
    std::cout << "DEM loaded.\n" << std::endl;
    return origDataset;

}

GDALDataset* GridShaper::LoadDEMFromServer()
{
    GDALDataset* origDataset = NULL;
    //if( FILE *ffile = fopen( TEMPFILE_WCSDATA, "w" ) )
    {
        std::string xmlString = "\
<WCS_GDAL>\n\
  <ServiceURL>https://devgis.mmp360.com/geoserver/iademtiff/wcs?</ServiceURL>\n\
  <CoverageName>FullIowaDEM_3M_EPSG_3857</CoverageName>\n\
</WCS_GDAL>\n\
";
/*
        std::string xmlString = "\
<WCS_GDAL>\n\
  <ServiceURL>http://devgis.iowammp.com/geoserver/iademtiff/wcs?</ServiceURL>\n\
  <CoverageName>IowaFullDEMGeoTiff_new</CoverageName>\n\
</WCS_GDAL>\n\
";
*/
        //fwrite( xmlString.c_str(), strlen( xmlString.c_str() ), 1, ffile );
        //fclose( ffile );
        origDataset = (GDALDataset *) GDALOpen( xmlString.c_str(), GA_ReadOnly );
        if( origDataset ==  NULL )
        {
            std::cout << "Error opening DEM file on web server." << std::endl;
        }
        else
        {
            std::cout << "Opening DEM file on web server succeeded." << std::endl;
            double transforms[6];
            origDataset->GetGeoTransform( transforms );
            GDALRasterBand *band = origDataset->GetRasterBand( 1 );
            std::string bandType;
            GDALDataType type = band->GetRasterDataType();
            switch ( type )
            {
                case GDT_Byte:
                {
                    bandType = "byte";
                    break;
                }
                case GDT_UInt16:
                {
                    bandType = "unsigned short integer";
                    break;
                }
                case GDT_Int16:
                {
                    bandType = "signed short integer";
                    break;
                }
                case GDT_UInt32:
                {
                    bandType = "unsigned long integer";
                    break;
                }
                case GDT_Int32:
                {
                    bandType = "signed long integer";
                    break;
                }
                case GDT_Float32:
                {
                    bandType = "float real";
                    break;
                }
                case GDT_Float64:
                {
                    bandType = "double real";
                    break;
                }
                default:
                {
                    bandType = "unknown";
                }
            }
            std::cout << "Cell Size X/Y = " << transforms[1] << "/" << transforms[5] << std::endl;
            std::cout << "Upper Left coords X = " << transforms[0] << " Y = " << transforms[3] << std::endl;
            std::cout << "Data type is " << bandType << "." << std::endl;

            const char *demRefWkt = NULL;
            char charVersion[1048];
            char *charPtr;
            demRefWkt = origDataset->GetProjectionRef();
            strcpy( charVersion, demRefWkt );
            charPtr = &charVersion[0];
            if( demRefWkt)
            {
#ifdef RUSLE2_WKT_OUTPUT
                std::cout << "DEM Coord Sys: " << demRefWkt << std::endl;
                std::cout << std::endl;
#endif

#ifndef FORCE_VECTOR_TO_DEM_FROM_SERVER_EPSG_NO_MATTER_WHAT
                OGRErr ogrErr = m_demRef.importFromWkt( &charPtr );
                if( ogrErr != OGRERR_NONE )
                {
                    m_reprojectAvailable = false;
                }
#endif
            }
        }
    }

    return origDataset;
}

OGRDataSource* GridShaper::LoadShape()
{
    OGRDataSource *farmDataset = NULL;
    OGRSpatialReference *farmRef = NULL;

    farmDataset = OGRSFDriverRegistrar::Open( m_farmFilename.c_str(), FALSE );
    if( farmDataset == NULL )
    {
        std::cout << "Shape file load failed" << std::endl;
        return NULL;
    }

    int layers = farmDataset->GetLayerCount();
    for ( int layerCt = 0; layerCt < layers; ++layerCt )
    {
        OGRLayer *currentLayer = farmDataset->GetLayer( layerCt );
        OGRCoordinateTransformation *farmTransform = NULL;

        if( m_reprojectAvailable )
        {
            farmRef = currentLayer->GetSpatialRef();
            if( farmRef )
            {
                char *farmRefWkt = NULL;
                farmRef->exportToWkt( &farmRefWkt );

                #ifdef RUSLE2_WKT_OUTPUT
                std::cout << "Farm Coord Sys: " << farmRefWkt << std::endl;
                std::cout << std::endl;
                #endif

                farmTransform = OGRCreateCoordinateTransformation( farmRef, &m_demRef );
                // Inverse transform will be for going from DEM to farm
                m_farmTransform = OGRCreateCoordinateTransformation( &m_demRef, farmRef );
                CPLFree( farmRefWkt );
            }
        }
        OGRFeature *farmFeature;
        currentLayer->ResetReading();
        while( (farmFeature = currentLayer->GetNextFeature()) != NULL )
        {
            OGRGeometry *farmGeometry;

            farmGeometry = farmFeature->GetGeometryRef();
            if( farmGeometry != NULL 
                && wkbFlatten(farmGeometry->getGeometryType()) == wkbPolygon )
            {
                OGRPolygon *farmPoly = (OGRPolygon *) farmGeometry;
                OGREnvelope testBounds;
                farmPoly->getEnvelope( &testBounds );

                std::cout << "Polygon geometry found in layer " << layerCt << std::endl;
                std::cout << "X range " << testBounds.MinX << " - " << testBounds.MaxX
                    << ", Y range " << testBounds.MinY << " - " << testBounds.MaxY << std::endl;

                if( farmTransform )
                {
                    //farmRef = currentLayer->GetSpatialRef();
                    //if( farmRef )
                    //{
                    //    char *farmRefWkt = NULL;
                    //    farmRef->exportToWkt( &farmRefWkt );

                    //    char* demRefWkt = NULL;
                    //    m_demRef.exportToWkt( &demRefWkt );
                        
                    //    #ifdef RUSLE2_WKT_OUTPUT
                    //    std::cout << "Farm Coord Sys: " << farmRefWkt << std::endl << std::endl;
                    //    std::cout << "DEM Coord Sys: " << demRefWkt << std::endl << std::endl;
                    //    #endif

                    //    OGRCoordinateTransformation *farmTransform = OGRCreateCoordinateTransformation( farmRef, &m_demRef );
                    //    if( farmTransform != NULL )
                    //    {
                    //        farmGeometry->transform( farmTransform );
                    //    }
                    //    OGRCoordinateTransformation::DestroyCT( farmTransform );
                    //    CPLFree( farmRefWkt );
                    //}
                    farmGeometry->transform( farmTransform );
                }

                // Find the bounds of the farm again
                farmPoly->getEnvelope( &m_farmBounds );

                std::cout << "Reprojected X range " << m_farmBounds.MinX << " - " << m_farmBounds.MaxX
                    << ", Y range " << m_farmBounds.MinY << " - " << m_farmBounds.MaxY << std::endl;

                // We got our man, get out while we can
                break;
            }
            else
            {
                std::cout << "No polygon geometry, this feature in layer " << layerCt << std::endl;
            }
        }
        if( farmTransform )
        {
            OGRCoordinateTransformation::DestroyCT( farmTransform );
        }
    }

    std::cout << "Shape file loaded.\n" << std::endl;
    return farmDataset;
}

GDALDataset *GridShaper::DownsizeDEM()
{
    #ifdef RUSLE2_NO_DOWNSIZE
    return m_origDEM;
    #endif
    GDALDataset  *downsizeDataset = NULL;
    CPLErr cplErr;

    int xCells = m_origDEM->GetRasterXSize();
    int yCells = m_origDEM->GetRasterYSize();
    //int nBands = m_origDEM->GetRasterCount();

    double transformFactors[6];
    m_origDEM->GetGeoTransform( transformFactors );

    int sampleOversize = 4;
    if( !FarmShapeToDEMCells( m_origDEM, sampleOversize ) )
    {
        return NULL;
    }

    // Is DEM already as small as needed?
    int downsizeXCells = m_iCellRangeX;
    int downsizeYCells = m_iCellRangeY;
    if( xCells <= downsizeXCells && yCells <= downsizeYCells )
    {
        // Downsizing not necessary
        std::cout << "Downsizing not necessary.\n" << std::endl;
        return m_origDEM;
    }
    GDALRasterBand *demBand = m_origDEM->GetRasterBand( 1 );

    downsizeDataset = CreateFarmSizeDataset( "downsizedDEM", NULL, m_origDEM, demBand->GetRasterDataType() );
    GDALRasterBand* downsizeBand = downsizeDataset->GetRasterBand( 1 );
    int success = 0;
    float nodataValue = static_cast< float >( demBand->GetNoDataValue( &success ) );
    if( success )
    {
        downsizeBand->SetNoDataValue( nodataValue );
    }

    std::cout << "Downsized DEM X cells " << downsizeDataset->GetRasterXSize() << ", Y cells " << downsizeDataset->GetRasterYSize() << std::endl;

    downsizeDataset->GetGeoTransform( transformFactors );
    std::cout << "Downsized DEM Transforms " << transformFactors[0] << " " << transformFactors[1] << " " << transformFactors[2] << " " << transformFactors[3] << " " << transformFactors[4] << " " << transformFactors[5] << std::endl;

    // Create buffer array to read three lines of data m_iCellRangeX + 2 wide
    int inputScanWidth = downsizeDataset->GetRasterXSize();
    int inputScanHeight = downsizeDataset->GetRasterYSize();
    int inputStartX = m_iCellMinX;
    int inputStartY = m_iCellMinY;
    float *demBuffer;
    demBuffer = (float *) CPLMalloc( sizeof(float) * inputScanWidth * inputScanHeight );

    cplErr = demBand->RasterIO( GF_Read, inputStartX, inputStartY, inputScanWidth, inputScanHeight, demBuffer, inputScanWidth, inputScanHeight, GDT_Float32, 0, 0 );
    if( cplErr == CE_Failure )
    {
        std::cout << "Error reading raster band for input from original DEM." << std::endl;
        GDALClose ( downsizeDataset );    
        CPLFree( demBuffer );
        return NULL;    
    }
    cplErr = downsizeBand->RasterIO( GF_Write, 0, 0, inputScanWidth, inputScanHeight, demBuffer, inputScanWidth, inputScanHeight, GDT_Float32, 0, 0 );
    if( cplErr == CE_Failure )
    {
        std::cout << "Error writing raster band to downsized DEM." << std::endl;
        GDALClose ( downsizeDataset );    
        CPLFree( demBuffer );
        return NULL;    
    }

    downsizeDataset->FlushCache();
    CPLFree( demBuffer );

    std::cout << "Downsize complete.\n" << std::endl;
    return downsizeDataset;
}

GDALDataset* GridShaper::ResampleDEM() const
{
    GDALDataset* resampleDataset = NULL;
    CPLErr cplErr;

    int xCells = m_downsizeDEM->GetRasterXSize();
    int yCells = m_downsizeDEM->GetRasterYSize();
    //int nBands = m_downsizeDEM->GetRasterCount();

    double transformFactors[6];
    m_downsizeDEM->GetGeoTransform( transformFactors );

    // Is DEM already in desired resolution?
    if( m_resolutionFactor == 1.0 )
    {
        // No regridding needed
        std::cout << "Resampling not necessary.\n" << std::endl;
        return m_downsizeDEM;
    }

    // Calculate the new number of cells with desired resolution. .5 is for rounding new cell count.
    int resizeXCells = static_cast< int >( .5 + xCells / m_resolutionFactor );
    int resizeYCells = static_cast< int >( .5 + yCells / m_resolutionFactor );
    // Is change so small as to not require re-gridding?
    if( resizeXCells == xCells && resizeYCells == yCells )
    {
        // Regridding not necessary
        return m_downsizeDEM;
    }

    // Set up the driver and file name
    std::string newFilename = m_baseName + "resampledDEM";
    GDALDriver* driver = GetGDALDriverManager()->GetDriverByName( "GTiff" );
    if( !driver )
    {
        driver = m_origDEM->GetDriver();
        newFilename += ".img";
    }
    else
    {
        newFilename += ".tif";
    }

    // The size of the raster is given the new projection and pixel spacing
    // Using the values we calculated above. Also, setting it to store one band
    // and to use Float32 data type.

    // Coords of DEM
    double ulx = transformFactors[0];
    double uly = transformFactors[3];
    double lrx = transformFactors[0] + transformFactors[1] * xCells;
    double lry = transformFactors[3] + transformFactors[5] * yCells;
    GDALRasterBand *demBand = m_downsizeDEM->GetRasterBand( 1 );

    // Calculate the new geotransform at the new cell size
    double resampleTransforms[6] = { transformFactors[0], (lrx - ulx) / resizeXCells, transformFactors[2], transformFactors[3], transformFactors[4], (lry - uly) / resizeYCells };

    resampleDataset = driver->Create( newFilename.c_str(), resizeXCells, resizeYCells, 1, demBand->GetRasterDataType(), NULL );

    // Set the geotransform
    resampleDataset->SetGeoTransform( resampleTransforms );
    cplErr = resampleDataset->SetProjection ( m_downsizeDEM->GetProjectionRef() );
    if( cplErr == CE_Failure )
    {
        std::cout << "Error setting projection on resampled data" << std::endl;
    }
    GDALRasterBand* resampleBand = resampleDataset->GetRasterBand( 1 );
    int success = 0;
    float nodataValue = static_cast<float>( demBand->GetNoDataValue( &success ) );
    if( success )
    {
        resampleBand->SetNoDataValue( nodataValue );
    }

    GDALWarpOptions *psWarpOptions = GDALCreateWarpOptions();

    psWarpOptions->hSrcDS = m_downsizeDEM;
    psWarpOptions->hDstDS = resampleDataset;

    psWarpOptions->nBandCount = 1;
    psWarpOptions->panSrcBands = 
        (int *) CPLMalloc(sizeof(int) * psWarpOptions->nBandCount );
    psWarpOptions->panSrcBands[0] = 1;
    psWarpOptions->panDstBands = 
        (int *) CPLMalloc(sizeof(int) * psWarpOptions->nBandCount );
    psWarpOptions->panDstBands[0] = 1;
    
    psWarpOptions->pTransformerArg = GDALCreateGenImgProjTransformer( m_downsizeDEM, 
                                     GDALGetProjectionRef(m_downsizeDEM), 
                                     resampleDataset,
                                     GDALGetProjectionRef(resampleDataset), 
                                     FALSE, 0.0, 1 );
    psWarpOptions->pfnTransformer = GDALGenImgProjTransform;

    GDALWarpOperation warper;
    warper.Initialize( psWarpOptions );

    cplErr = warper.ChunkAndWarpImage( 0, 0, 
                              GDALGetRasterXSize( resampleDataset ), 
                              GDALGetRasterYSize( resampleDataset ) );
    if( cplErr == CE_Failure )
    {
        std::cout << "Error warping DEM into resampled grid." << std::endl;
        GDALClose ( resampleDataset );    
        GDALDestroyGenImgProjTransformer( psWarpOptions->pTransformerArg );
        GDALDestroyWarpOptions( psWarpOptions );
        return NULL;    
    }

    GDALDestroyGenImgProjTransformer( psWarpOptions->pTransformerArg );
    GDALDestroyWarpOptions( psWarpOptions );

    std::cout << "Resampled DEM Size X " << resizeXCells << ", Y " << resizeYCells << std::endl;
    std::cout << "Resampled DEM Transforms " << resampleTransforms[0] << " " << resampleTransforms[1] << " " << resampleTransforms[2] << " " << resampleTransforms[3] << " " << resampleTransforms[4] << " " << resampleTransforms[5] << std::endl;

    resampleDataset->FlushCache();

    std::cout << "Resample complete.\n" << std::endl;
    return resampleDataset;
}

bool GridShaper::FarmShapeToDEMCells( GDALDataset *dataset, const int& sizeAdjust )
{
    // Convert opposing corners of farm bounds to DEM cells

    // From http://www.gdal.org/gdal_tutorial.html

    // adfGeoTransform[0] /* top left x */
    // adfGeoTransform[1] /* w-e pixel resolution */
    // adfGeoTransform[2] /* rotation, 0 if image is "north up" */
    // adfGeoTransform[3] /* top left y */
    // adfGeoTransform[4] /* rotation, 0 if image is "north up" */
    // adfGeoTransform[5] /* n-s pixel resolution */

    double transformFactors[6];
    dataset->GetGeoTransform( transformFactors );

    // From http://www.gdal.org/classGDALDataset.html
    // Where P is the pixel and L is the line, Xp and Yp are the coordinates of a point
    // Xp = padfTransform[0] + P*padfTransform[1] + L*padfTransform[2];
    // Yp = padfTransform[3] + P*padfTransform[4] + L*padfTransform[5];

    double Pmax, Pmin, Lmax, Lmin;

    // Simplifies things if rotation factors are 0
    if( transformFactors[2] == 0.0 && transformFactors[4] == 0.0 )
    {
        // For the maximum coordinate pair
        Pmax = ( m_farmBounds.MaxX - transformFactors[0] ) / transformFactors[1];
        Lmax = ( m_farmBounds.MaxY - transformFactors[3] ) / transformFactors[5];

        // For the minimum pair
        Pmin = ( m_farmBounds.MinX - transformFactors[0] ) / transformFactors[1];
        Lmin = ( m_farmBounds.MinY - transformFactors[3] ) / transformFactors[5];

        std::cout << "Unsorted Farm Cell range X " << Pmin << " - " << Pmax << ", Y range " << Lmin << " - " << Lmax << std::endl;

    }
    else
    {
        std::cout << "Rotated DEMs within reference systems are not supported." << std::endl;
        return false;
    }

    m_cellMaxX = Pmax > Pmin ? Pmax: Pmin;
    m_cellMinX = Pmin < Pmax ? Pmin: Pmax;
    m_cellMaxY = Lmax > Lmin ? Lmax: Lmin;
    m_cellMinY = Lmin < Lmax ? Lmin: Lmax;

    std::cout << "Farm Cell range X " << m_cellMinX << " - " << m_cellMaxX
        << ", Y range " << m_cellMinY << " - " << m_cellMaxY << std::endl;

    // Expand sample area by one so that there is a row and column of data outside
    // the farm boundary so that all the cells within the boundary
    // are represented as the center of a 3 x 3 matrix of cells.
    m_iCellMaxX = static_cast<int>( ceil( m_cellMaxX ) + sizeAdjust );
    m_iCellMinX = static_cast<int>( floor( m_cellMinX ) - sizeAdjust );
    m_iCellMaxY = static_cast<int>( ceil( m_cellMaxY ) + sizeAdjust );
    m_iCellMinY = static_cast<int>( floor( m_cellMinY ) - sizeAdjust );

    if( m_iCellMaxX < 1 )
    {
        m_iCellMaxX = 1;
    }
    if( m_iCellMinX < 1 )
    {
        m_iCellMinX = 1;
    }
    if( m_iCellMaxY < 1 )
    {
        m_iCellMaxY = 1;
    }
    if( m_iCellMinY < 1 )
    {
        m_iCellMinY = 1;
    }

    int xCells = dataset->GetRasterXSize();
    int yCells = dataset->GetRasterYSize();
    std::cout << "Raster DEM cell count X " << xCells << " Y " << yCells << std::endl;
    
    if( m_iCellMaxX >= xCells - 1)
    {
        m_iCellMaxX = xCells - 2;
    }
    if( m_iCellMinX >= xCells - 1 )
    {
        m_iCellMinX = xCells - 2;
    }
    if( m_iCellMaxY >= yCells - 1 )
    {
        m_iCellMaxY = yCells - 2;
    }
    if( m_iCellMinY >= yCells - 1 )
    {
        m_iCellMinY = yCells - 2;
    }

    std::cout << "Adjusted Truncated Cell range X " << m_iCellMinX << " - " << m_iCellMaxX << ", Y range " << m_iCellMinY << " - " << m_iCellMaxY << std::endl;

    m_iCellRangeX = m_iCellMaxX - m_iCellMinX;
    m_iCellRangeY = m_iCellMaxY - m_iCellMinY;

    if( m_iCellRangeX == 0.0 || m_iCellRangeY == 0.0 )
    {
        std::cout << "Farm outline falls outside DEM coverage area. Execution terminated." << std::endl;
        return false;
    }

    if( m_hewTo3xMultipleSize )
    {
        int cellRemainder = m_iCellRangeX % 3;
        if( cellRemainder > 0 )
        {
            cellRemainder = 3 - cellRemainder;
            m_iCellRangeX += cellRemainder;
            m_iCellMaxX += cellRemainder;
            if( m_iCellMaxX >= xCells - 1)
            {
                m_iCellMaxX -= 3;
                m_iCellRangeX -= 3;
            }
        }
        cellRemainder = m_iCellRangeY % 3;
        if( cellRemainder > 0 )
        {
            cellRemainder = 3 - cellRemainder;
            m_iCellRangeY += cellRemainder;
            m_iCellMaxY += cellRemainder;
            if( m_iCellMaxY >= xCells - 1)
            {
                m_iCellMaxY -= 3;
                m_iCellRangeY -= 3;
            }
        }
    }
    std::cout << "Output raster dimensions X " << m_iCellRangeX << ", Y " << m_iCellRangeY << std::endl;
    std::cout << "Farm bounds computation complete.\n" << std::endl;
    
    return true;

}

GDALDataset *GridShaper::CreateFarmSizeDataset( const std::string& newFilename, GDALDataset *copyMe, GDALDataset *georefSource, const GDALDataType& dType ) const
{
    GDALDataset *newDataset = NULL;
    CPLErr cplErr;
    std::string totalName = m_baseName + newFilename;

    GDALDriver* driver = GetGDALDriverManager()->GetDriverByName( "GTiff" );
    if( driver == NULL )
    {
        driver = m_origDEM->GetDriver();
        totalName += ".img";
    }
    else
    {
        totalName += ".tif";
    }

    // Create new raster using Float32 data type.

    if( copyMe )
    {
        newDataset = driver->CreateCopy( totalName.c_str(), copyMe, false, NULL, NULL, NULL );
        if( newDataset == NULL )
        {
            std::cout << totalName << " " << CPLGetLastErrorMsg() << std::endl;
        }
    }
    else
    {
        char **createOptions = NULL;
        newDataset = driver->Create( totalName.c_str(), m_iCellRangeX, m_iCellRangeY, 1, dType, createOptions );
        if( newDataset == NULL )
        {
            std::cout << totalName << CPLGetLastErrorMsg() << std::endl;
            std::cout << "Operation failed." << std::endl;
            return NULL;
        }
        double transformFactors[6];
        georefSource->GetGeoTransform( transformFactors );

        // Coords of farm-size DEM
        double ulx = transformFactors[0] + m_iCellMinX * transformFactors[1];
        double uly = transformFactors[3] + m_iCellMinY * transformFactors[5];

        double newTransforms[6] = { ulx, transformFactors[1], transformFactors[2], uly, transformFactors[4], transformFactors[5] };

        // Set the geotransform
        cplErr = newDataset->SetGeoTransform( newTransforms );
        if( cplErr == CE_Failure )
        {
            std::cout << "Error setting GeoTransform on " << totalName << std::endl;
        }
        cplErr = newDataset->SetProjection ( georefSource->GetProjectionRef() );
        if( cplErr == CE_Failure )
        {
            std::cout << "Error setting Projection on " << totalName << std::endl;
        }
    }

    return newDataset;
}

//////////////////////////////////////////////////////////////////////////////////////////
GDALDataset* GridShaper::GetResampledDEM() const
{
    return m_resampleDEM;
}
//////////////////////////////////////////////////////////////////////////////////////////
GDALDataset* GridShaper::GetDownsizedDEM() const
{
    return m_downsizeDEM;
}
//////////////////////////////////////////////////////////////////////////////////////////
PointGridder::PointGridder( const bool& gridFullSize ) :
    m_xSize( 0 ),
    m_ySize( 0 ),
    m_xLow( 0.0 ),
    m_xHigh( 0.0 ),
    m_yLow( 0.0 ),
    m_yHigh( 0.0 ),
    m_xCellOverlapFraction( .5 ),
    m_yCellOverlapFraction( .5 ),
    m_numPoints( 0 ),
    m_xPoints( 0 ),
    m_yPoints( 0 ),
    m_zPoints( 0 ),
    m_gridFullSize( gridFullSize ),
    m_pointDataSet( 0 ),
    m_gridDataset( 0 ),
    m_dataType( GDT_Float32 ),
    m_gridAlgorithm( GGA_MovingAverage )
{
}

PointGridder::~PointGridder()
{
    if( m_pointDataSet )
    {
        OGRDataSource::DestroyDataSource ( m_pointDataSet );
    }
    if( m_gridDataset )
    {
        GDALClose ( (GDALDatasetH)m_gridDataset );
    }
}

void PointGridder::Init()
{
    // Register GDAL and OGR for file access
    GDALAllRegister();
    OGRRegisterAll();
}

void PointGridder::SetGridSize( const int& xSize, const int& ySize )
{
    m_xSize = xSize;
    m_ySize = ySize;
}

void PointGridder::SetGridSizeFromRusle2( const int& xSize, const int& ySize )
{
    int divisor = m_gridFullSize ? 1: 3;
    SetGridSize( xSize / divisor, ySize / divisor );
}

void PointGridder::SetGridBounds( const double& xLow, const double& xHigh, const double& yLow, const double& yHigh )
{
    m_xLow = xLow;
    m_xHigh = xHigh;
    m_yLow = yLow;
    m_yHigh = yHigh;
    //std::cout << "Grid bounds: X " << m_xLow << " to " << m_xHigh << " Y " << m_yLow << " to " << m_yHigh << std::endl;
}

void PointGridder::SetGridBoundsFromRusle2GeoTransform( const double *paramArray6, const int& cellsX, const int& cellsY )
{
    SetGridBounds( paramArray6[0], paramArray6[0] + cellsX * paramArray6[1], paramArray6[3] + cellsY * paramArray6[5], paramArray6[3] );
}

void PointGridder::SetCellOverlapSearchFraction( const double& xOverlap, const double& yOverlap )
{
    m_xCellOverlapFraction = xOverlap;
    m_yCellOverlapFraction = yOverlap;
}

void PointGridder::SetDataType( const GDALDataType& dataType )
{
    m_dataType = dataType;
}

void PointGridder::SetGridSpatialReference( const std::string& wktRefString )
{
    m_wktRefString = wktRefString;
}

void PointGridder::SetGridAttribute( const std::string& gridAttribute )
{
    m_gridAttribute = gridAttribute;
}

void PointGridder::SetPointFileName( const std::string& pointFileName )
{
    m_pointFileName = pointFileName;
}

bool PointGridder::LoadPointFile()
{
    OGRDataSource *pointDataset = NULL;

    pointDataset = OGRSFDriverRegistrar::Open( m_pointFileName.c_str(), FALSE );
    if( pointDataset == NULL )
    {
        std::cout << "Point data file load failed." << std::endl;
        std::cout << CPLGetLastErrorMsg() << std::endl;
        return false;
    }

    m_pointDataSet = pointDataset;
    std::cout << "Point file opened and read. " << m_pointFileName << std::endl;

    return true;
}

bool PointGridder::ParsePointDataset( double& xLow, double& xHigh, double& yLow, double& yHigh )
{
    int numPoints = 0, expectedNumPoints;
    double *xPoints;
    double *yPoints;
    double *zPoints;
    xLow = std::numeric_limits<float>::max();
    yLow = std::numeric_limits<float>::max();
    xHigh = -std::numeric_limits<float>::max();
    yHigh = -std::numeric_limits<float>::max();

    int layers = m_pointDataSet->GetLayerCount();
    for ( int layerCt = 0; layerCt < layers; ++layerCt )
    {
        OGRLayer *currentLayer = m_pointDataSet->GetLayer( layerCt );
        currentLayer->ResetReading();

        // Only point features are currently supported so test this feature.
        OGRGeometry *pointGeometry;
        OGRFeature *feature;
        feature = currentLayer->GetNextFeature();
        pointGeometry = feature->GetGeometryRef();
        if( pointGeometry != NULL 
            && wkbFlatten( pointGeometry->getGeometryType()) == wkbPoint )
        {
            currentLayer->ResetReading();
            expectedNumPoints = currentLayer->GetFeatureCount();

            // Allocate storage for x, y and z values for all points.
            xPoints = (double *)CPLMalloc( sizeof( double ) * expectedNumPoints );
            yPoints = (double *)CPLMalloc( sizeof( double ) * expectedNumPoints );
            zPoints = (double *)CPLMalloc( sizeof( double ) * expectedNumPoints );

            if( xPoints && yPoints && zPoints )
            {
                // fetch the spatial reference for the data set
                OGRCoordinateTransformation *pointTransform = NULL;
                OGRSpatialReference demRef;
                char refString[1048];
                strncpy( refString, m_wktRefString.c_str(), 1047 );
                refString[1047] = 0;
                char *charPtr = &refString[0];
                OGRErr ogrErr = demRef.importFromWkt( &charPtr );
                if( ogrErr == OGRERR_NONE )
                {
                    OGRSpatialReference *spatialRef = currentLayer->GetSpatialRef();
                    if( spatialRef )
                    {
                        char *refWkt = NULL;
                        ogrErr = spatialRef->exportToWkt( &refWkt );
                        if( ogrErr == OGRERR_NONE )
                        {
                            std::cout << "Input file spatial reference: " << refWkt << std::endl;
                            std::cout << "Points will be transformed to: " << m_wktRefString << std::endl;
                            pointTransform = OGRCreateCoordinateTransformation( spatialRef, &demRef );
                            CPLFree( refWkt );
                        }
                        else
                        {
                            std::cout << "Error obtaining spatial reference on point data set." << std::endl;
                        }
                    }
                    else
                    {
                        std::cout << "Error obtaining spatial reference from Rusle2." << std::endl;
                    }
                }
                else
                {
                    std::cout << "Gridded output file will not contain a spatial reference. Input file didn't have one." << std::endl;
                }

                // create a transform for the point data

                while( (feature = currentLayer->GetNextFeature()) != NULL && numPoints < expectedNumPoints )
                {
                    pointGeometry = feature->GetGeometryRef();
                    double gridValue = feature->GetFieldAsDouble( m_gridAttribute.c_str() );
                    OGRPoint *point = (OGRPoint *)pointGeometry;
                    if( pointTransform != NULL )
                    {
                        point->transform( pointTransform );
                    }

                    xPoints[ numPoints ] = point->getX();
                    yPoints[ numPoints ] = point->getY();
                    zPoints[ numPoints ] = gridValue;

                    // Test and expand geometry bounds
                    if( xPoints[ numPoints ] < xLow )
                    {
                        xLow = xPoints[ numPoints ];
                    }
                    if( xPoints[ numPoints ] > xHigh )
                    {
                        xHigh = xPoints[ numPoints ];
                    }
                    if( yPoints[ numPoints ] < yLow )
                    {
                        yLow = yPoints[ numPoints ];
                    }
                    if( yPoints[ numPoints ] > yHigh )
                    {
                        yHigh = yPoints[ numPoints ];
                    }

                    ++numPoints;
                }
                m_xPoints = xPoints;
                m_yPoints = yPoints;
                m_zPoints = zPoints;
                m_numPoints = numPoints;

                if( pointTransform )
                {
                    OGRCoordinateTransformation::DestroyCT( pointTransform );
                }
                std::cout << "Point data loaded. Number of points = " << numPoints << std::endl;
                return true;
            }
            else
            {
                std::cout << "Memory for points could not be allocated." << std::endl;
            }       
        }
        else
        {
            std::cout << "No point geometry found in layer " << layerCt << std::endl;
        }       

    }

    return false;
}

void *PointGridder::GridPointFile( void *suppliedStorage )
{
    /* from http://www.gdal.org/gdal__alg_8h.html#a1fdef40bcdbc98eff2328b0d093d3a22
    CPLErr GDALGridCreate    ( GDALGridAlgorithm     eAlgorithm,
        const void *     poOptions,
        GUInt32     nPoints,
        const double *     padfX,
        const double *     padfY,
        const double *     padfZ,
        double     dfXMin,
        double     dfXMax,
        double     dfYMin,
        double     dfYMax,
        GUInt32     nXSize,
        GUInt32     nYSize,
        GDALDataType     eType,
        void *     pData,
        GDALProgressFunc     pfnProgress,
        void *     pProgressArg     
        )
    */
    GDALGridMovingAverageOptions options;
    options.dfAngle = 0.0;
    //double outCellSearchRadiusX = .5;
    //double outCellSearchRadiusY = .5;
    double cellSizeX = ( m_xHigh - m_xLow ) / m_xSize;
    double cellSizeY = ( m_yHigh - m_yLow ) / m_ySize;
    options.dfRadius1 = cellSizeX * ( .5 + m_xCellOverlapFraction );    // will include half cell overlap
    options.dfRadius2 = cellSizeY * ( .5 + m_yCellOverlapFraction );
    options.nMinPoints = 1;
    options.dfNoDataValue = -10000.0;
    GDALGridAlgorithm eAlgorithm = m_gridAlgorithm;

    GUInt32 nPoints = m_numPoints;
    double *padfX = m_xPoints;
    double *padfY = m_yPoints;
    double *padfZ = m_zPoints;
    double dfXMin = m_xLow;
    double dfXMax = m_xHigh;
    double dfYMin = m_yLow;
    double dfYMax = m_yHigh;
    GUInt32 nXSize = m_xSize;
    GUInt32 nYSize = m_ySize;
    GDALDataType eType = m_dataType;
    void *pData = suppliedStorage;

    if( ! pData )
    {
        if( eType == GDT_Byte )
        {
            pData = CPLMalloc( sizeof( char ) * nXSize * nYSize );
        }
        else if( eType == GDT_Int32 )
        {
            pData = CPLMalloc( sizeof( int ) * nXSize * nYSize );
        }
        else if( eType == GDT_Float32 )
        {
            pData = CPLMalloc( sizeof( float ) * nXSize * nYSize );
        }
        else if( eType == GDT_Float64 )
        {
            pData = CPLMalloc( sizeof( double ) * nXSize * nYSize );
        }
        else
        {
            std::cout << "Unsupported data type for gridded output." << std::endl;
            return NULL;
        }
    }

    if( pData )
    {
        std::cout << "Point gridding begun." << std::endl;

        // Not able to locate any definitive description of the gridded output row ordering.
        // It seems that following the suggested bounds input the resulting grid has the low y rows first in the raster
        // leading to an upside down image when copied into a gdal raster band with RasterIO.
        CPLErr cplErr = GDALGridCreate( eAlgorithm,
            &options,
            nPoints,
            padfX,
            padfY,
            padfZ,
            dfXMin,
            dfXMax,
            dfYMax,    // Reversing the order of dfYMin and dfYMax sorts the gridded data into the correct row order.
            dfYMin,
            nXSize,
            nYSize,
            eType,
            pData,
            NULL,
            NULL     
            );
        if( cplErr == CE_Failure )
        {
            std::cout << "Error gridding point data." << std::endl;
            if( ! suppliedStorage )
            {
                CPLFree( pData );
            }
            return NULL;
        }
    }
    else
    {
        std::cout << "Error allocating memory for gridded point data." << std::endl;
    }

    if( pData )
    {
        std::cout << "Gridding point data complete." << std::endl;
    }
    return pData;
}

bool PointGridder::StoreGridRaster( void *griddedRaster )
{
    bool success = true;
    GDALDataset *newDataset = NULL;
    CPLErr cplErr;
    std::string totalName = OUTPUT_GRIDFILE_NAME;

    GDALDriver* driver = GetGDALDriverManager()->GetDriverByName( "HFA" );

    // Create new raster using data type from raster.

    char **createOptions = NULL;
    newDataset = driver->Create( totalName.c_str(), m_xSize, m_ySize, 1, m_dataType, createOptions );
    if( newDataset == NULL )
    {
        std::cout << totalName << CPLGetLastErrorMsg() << std::endl;
        std::cout << "Operation failed." << std::endl;
        return false;
    }
    double transformFactors[6];

    // Coords of farm-size DEM
    transformFactors[0] = m_xLow;
    transformFactors[1] = ( m_xHigh - m_xLow ) / m_xSize;
    transformFactors[2] = 0.0;
    transformFactors[3] = m_yHigh;
    transformFactors[4] = 0.0;
    transformFactors[5] = ( m_yLow - m_yHigh ) / m_ySize;

    for ( int ct = 0; ct < 6; ++ct )
    {
        std::cout << "Transform[" << ct << "] = " << transformFactors[ct] << std::endl;

    }
    // Set the geotransform
    cplErr = newDataset->SetGeoTransform( transformFactors );
    if( cplErr == CE_Failure )
    {
        std::cout << "Error setting GeoTransform on " << totalName << std::endl;
    }

    cplErr = newDataset->SetProjection ( m_wktRefString.c_str() );
    if( cplErr == CE_Failure )
    {
        std::cout << "Error setting Projection on " << totalName << std::endl;
    }

    // Copy gridded data into raster band
    GDALRasterBand *myRasterBand = newDataset->GetRasterBand( 1 );

    cplErr = myRasterBand->RasterIO( GF_Write, 0, 0, m_xSize, m_ySize, griddedRaster, m_xSize, m_ySize, m_dataType, 0, 0 );
    if( cplErr == CE_Failure )
    {
        std::cout << "Error saving file " << totalName << std::endl;
        success = false;
    }
    newDataset->FlushCache();
    
    if( success )
    {
        m_gridDataset = newDataset;
    }
    return success;

}

double PointGridder::SampleGridDataByCell( const int& xCell, const int& yCell ) const
{
    double readValue = 0.0;

    if( m_gridDataset )
    {
        GDALRasterBand *band = m_gridDataset->GetRasterBand( 1 );

        if( xCell >= 0 && xCell < band->GetXSize() && yCell >= 0 && yCell < band->GetYSize() )
        {
            band->RasterIO( GF_Read, xCell, yCell, 1, 1, &readValue, 1, 1, GDT_Float64, 0, 0 );
        }
    }

    return readValue;
}

double PointGridder::SampleGridDataByCoords( const double& xPos, const double& yPos ) const
{
    double readValue = 0.0;

    if( m_gridDataset )
    {
        GDALRasterBand *band = m_gridDataset->GetRasterBand( 1 );
        double geoTransforms[6];

        m_gridDataset->GetGeoTransform( geoTransforms );

        int xCell = static_cast<int>( ( xPos - geoTransforms[0] ) / geoTransforms[1] );
        int yCell = static_cast<int>( ( yPos - geoTransforms[3] ) / geoTransforms[5] );

        if( xCell >= 0 && xCell < band->GetXSize() && yCell >= 0 && yCell < band->GetYSize() )
        {
            band->RasterIO( GF_Read, xCell, yCell, 1, 1, &readValue, 1, 1, GDT_Float64, 0, 0 );
        }
    }

    return readValue;
}

}
