// Project headers
#include <FieldGridGen/FieldGridGen.h>
#include <rusle2/RUSLE2.h>

// C++ headers
#include <iostream>

using namespace fieldgridgen;
using namespace rusle2;

#define GRIDGEN_TEST_DEMFILE "data/DEM_3m_I_17_3857_Good_Elevation_Fixed.tif"
#define GRIDGEN_TEST_DEMFILE "data/CerroGordoExport.tif"
#ifndef BUILD_GARY
#define GRIDGEN_TEST_FARMFILE "data/field.shp"
#define GRIDGEN_TEST_POINTFILE "data/yield.shp"
#else
#define GRIDGEN_TEST_FARMFILE "data/webster_field/field.shp"
#define GRIDGEN_TEST_POINTFILE "data/webster_field/yield.shp"
#endif
#define GRIDGEN_TEST_GRIDATTRIBUTE "Yld_Vol_Dr"

// This is the Rusle2 grid resolution scaling factor. 1.0 is a magic number and should not be changed when testing FieldGridGen.
#define GRIDGEN_TEST_RUSLE2_RESOLUTION_FACTOR 1.0
// The only valid values for this are 1.0 or 3.0. Any value besides those will be treated as 3.0 and result in a downsampled grid.
#define GRIDGEN_TEST_RESOLUTION_FACTOR 1.0

#define GRIDGEN_TEST_FONTFILE "DejaVuSans.ttf"
#define GRIDGEN_TEST_FONTNAME "Sans"
#ifndef BUILD_GARY
#define GRIDGEN_TEST_MAPNIKPATH "/stuff/data/dev/mapnik-2.2.0/install-64-bit/lib"
#else
#define GRIDGEN_TEST_MAPNIKPATH "C:\\OSGDev/mapnik-v2.2.0/lib"
#endif
#define GRIDGEN_TEST_TEXTSIZE 15.0f
#define GRIDGEN_TEST_MAXDIMENSION 800
#define GRIDGEN_TEST_RUSLE2_OUTPUTS    true
#define GRIDGEN_GRID_FIELDDATA    true

Rusle2 *BuildRusle2()
{
    std::string demFile = GRIDGEN_TEST_DEMFILE;
    std::string farmFile = GRIDGEN_TEST_FARMFILE;
    // When resolution is 3.0 the output grid in rusle2 will have x and y cell counts that are divisible by three
    // to facilitate precise 1/3 downrezing in the FieldGridGen module.
    double resolution = GRIDGEN_TEST_RUSLE2_RESOLUTION_FACTOR;
    double channelDrainageAreaAcres = 1.0;
    double pitFillLevel = 10.0;
    Rusle2 *rusle2 = new Rusle2( demFile,  farmFile, resolution, channelDrainageAreaAcres, pitFillLevel );
    if ( rusle2 )
    {
        if ( rusle2->LoadAndOperate() )
        {
            return rusle2;
        }
        delete rusle2;
    }
    return NULL;
}

GridShaper *BuildGridShaper()
{
    std::string demFile = GRIDGEN_TEST_DEMFILE;
    std::string farmFile = GRIDGEN_TEST_FARMFILE;
    // When resolution is 3.0 the output grid in rusle2 will have x and y cell counts that are divisible by three
    // to facilitate precise 1/3 downrezing in the FieldGridGen module.
    double resolution = GRIDGEN_TEST_RUSLE2_RESOLUTION_FACTOR;
    GridShaper *shaper = new GridShaper( demFile, farmFile, resolution );
    if ( shaper )
    {
        OGRDataSource *farmshape = NULL;
        if ( shaper->LoadAndOperate( farmshape ) )
        {
            return shaper;
        }
        delete shaper;
    }
    return NULL;
}

// Grid the point data for extracting field values
PointGridder *BuildPointGridder( GridShaper *shaper )
{
    std::string pointFile = GRIDGEN_TEST_POINTFILE;
    std::string gridAttribute = GRIDGEN_TEST_GRIDATTRIBUTE;

    bool gridAtFullRusle2Resolution = GRIDGEN_TEST_RESOLUTION_FACTOR == GRIDGEN_TEST_RUSLE2_RESOLUTION_FACTOR ? true: false;
    PointGridder *pointGridder = new PointGridder( gridAtFullRusle2Resolution );
    if ( pointGridder )
    {
        bool pointGridSuccess = false;
        pointGridder->Init();
        pointGridder->SetPointFileName( pointFile );
        if ( pointGridder->LoadPointFile() )
        {
            double xLow, xHigh, yLow, yHigh;
            pointGridder->SetGridAttribute( gridAttribute );
            std::string wktRefString;
            shaper->GetSpatialReferenceString( wktRefString );
            pointGridder->SetGridSpatialReference( wktRefString );
            // Parsing the point data will fill in the dimensions of the data set.
            if ( pointGridder->ParsePointDataset( xLow, xHigh, yLow, yHigh ) )
            {
                // set grid bounds and resolution from GridShaper
                int demCellRangeX, demCellRangeY;
                demCellRangeX = shaper->GetDEMXSize();
                demCellRangeY = shaper->GetDEMYSize();
                pointGridder->SetGridSizeFromRusle2( demCellRangeX, demCellRangeY );

                double projParams6[6];
                shaper->GetOutputGeoTransform6( projParams6 );
                pointGridder->SetGridBoundsFromRusle2GeoTransform( projParams6, demCellRangeX, demCellRangeY );
                if ( gridAtFullRusle2Resolution )
                {
                    // defaults to .5 or half a cell overlap which may not be enough at full grid resolution
                    pointGridder->SetCellOverlapSearchFraction( 1.5, 1.5 );
                }

                void *storage = pointGridder->GridPointFile( );
                if ( storage )
                {
                    if ( pointGridder->StoreGridRaster( storage ) )
                    {
                        std::cout << "Point file gridded successfully and saved." << std::endl;
                        pointGridSuccess = true;
                    }
                    else
                    {
                        std::cout << "Point file not saved successfully." << std::endl;
                    }
                    CPLFree( storage );
                }
                else
                {
                    std::cout << "Point gridding failed in grid function." << std::endl;
                }
            }
            else
            {
                std::cout << "Point gridding failed. Unable to parse point values." << std::endl;
            }
        }
        else
        {
            std::cout << "Point gridding failed. Unable to load point file." << std::endl;
        }
        if ( pointGridSuccess )
        {
            return pointGridder;
        }

        delete pointGridder;
    }

    return NULL;
}

FieldGridGen *BuildFieldGridGen( Rusle2 *rusle2, GridShaper *shaper, PointGridder *pointGridder )
{
    if ( rusle2 || shaper )
    {
        if ( rusle2 && ! shaper )
        {
            shaper = rusle2->GetGridShaper();
        }
        if ( shaper )
        {
            ///"-mapnikPath" The absolute path to the mapnik root directory.
            std::string mapnikPath = GRIDGEN_TEST_MAPNIKPATH;
            ///"-fontName" The font name.
            std::string fontName = GRIDGEN_TEST_FONTNAME;
            ///"-fontFilename" The font file name.
            std::string fontFilename = GRIDGEN_TEST_FONTFILE;
            ///"-textSize" The text height in pixels.
            float textSize = GRIDGEN_TEST_TEXTSIZE;
            ///"-maxDimension" The maximum output image size
            int maxDimension = GRIDGEN_TEST_MAXDIMENSION;

            int demCellRangeX, demCellRangeY;
            demCellRangeX = shaper->GetDEMXSize();
            demCellRangeY = shaper->GetDEMYSize();

            std::string wktRefString;
            shaper->GetSpatialReferenceString( wktRefString );

            double projParams6[6];
            shaper->GetOutputGeoTransform6( projParams6 );

            // Create an instance of fieldGridGen class with the input parameters supplied from GridShaper
            bool gridAtFullRusle2Resolution = pointGridder->GetGridFullResolution();
            FieldGridGen *fieldGridGen = new FieldGridGen ( mapnikPath, demCellRangeX, demCellRangeY, wktRefString, projParams6, maxDimension, gridAtFullRusle2Resolution );
            if (fieldGridGen)
            {
                // Be sure cell borders are enabled (they are by default) and set line width.
                fieldGridGen->SetCellBordersEnabled( true );
                fieldGridGen->SetCellBorderWidth( .25 );

                // Add titles for key
                fieldGridGen->AddKeyTitle( GRIDGEN_TEST_GRIDATTRIBUTE, fontName, fontFilename, textSize * 2 );
                //fieldGridGen->AddKeyTitle( "Release Acres Below (250)", fontName, fontFilename, textSize * 1.5f );
                fieldGridGen->AddKeyTitle( "(bushels/acre)", fontName, fontFilename, textSize * 1.5f );

                // Add color ranges for key
                float upperColorRange[] = { 0, 109.9, 119.9, 129.9, 139.9, 149.9, 159.9, 169.9, 179.9, 189.9, 300.0 };
                std::string rangeLabels[] = { "No Row Crop", "100 - 109", "110 - 119", "120 - 129", "130 - 139", "140 - 149", "150 - 159",
                    "160 - 169", "170 - 179", "180 - 189", "190 - 300", "" };
                int Red[] = { 98, 255, 253, 249, 246, 242, 226, 197, 168, 137, 105 };
                int Green[] = { 98, 255, 235, 215, 196, 176, 148, 110, 74, 37, 0 };
                int Blue[] = { 98, 128, 109, 92, 73, 54, 40, 30, 20, 11, 0 };
                ColorIndexRGB rgb;
                for ( int ct = 0; rangeLabels[ct].size() > 0; ++ct )
                {
                    rgb.r = Red[ct];
                    rgb.g = Green[ct];
                    rgb.b = Blue[ct];
                    fieldGridGen->AddColorIndexRange( rangeLabels[ct], fontName, fontFilename, textSize, upperColorRange[ct], rgb );
                }

                /* Example for fetching data from a specific cell in the center of the grid
                int x, y;
                x = fieldGridGen->GetGridSizeX() / 2;
                y = fieldGridGen->GetGridSizeY() / 2;
                int cellID = y * fieldGridGen->GetGridSizeX() + x;

                GridCellArray gridArray = fieldGridGen->FetchGridCellArray();

                std::string json = gridArray[ cellID ].second;
                float value = gridArray[ cellID ].first;
                */

                // Load and process the input files and write the output maps and vector files to the working directory.
                // pointGridder may be NULL if there is no gridded data.
                if ( fieldGridGen->LoadAndOperate( pointGridder ) )
                {
                    // Report success.
                    std::cout << "Operation terminated successfully. All outputs were generated properly." << std::endl;
                    std::cout << "Testing retrieval of auxiliary grid data containing rusle2 outputs." << std::endl;
                    if ( rusle2 )
                    {
                        Rusle2CellArray gridArray = fieldGridGen->FetchRusle2CellArray( rusle2 );
                        if ( gridArray.size() > 0 )
                        {
                            // Report success.
                            std::cout << "All auxiliary grid data were retrieved properly." << std::endl;
                        }
                        else
                        {
                            std::cout << "Unable to retrieve auxiliary grid data." << std::endl;
                        }
                    }
                    else
                    {
                        std::cout << "Rusle2 was not run in this simulation. Outputs not available" << std::endl;
                    }
                    return fieldGridGen;
                }
                else
                {
                    std::cout << "Operation terminated prematurely. Not all outputs were generated properly." << std::endl;
                }
                delete fieldGridGen;
            }
        }
    }
    return NULL;
}

///Instantiate an instance of class fieldgridgen::FieldGridGen.
///Resulting files will be placed in the working directory.
int main(int argc, char** argv) {
    GridShaper *shaper = NULL;
    Rusle2 *rusle2 = NULL;

    std::cout.precision( std::numeric_limits< double >::digits10 );
    // Run either rusle2 or GridShaper
    if ( GRIDGEN_TEST_RUSLE2_OUTPUTS )
    {
        rusle2 = BuildRusle2();
        if ( rusle2 )
        {
            shaper = rusle2->GetGridShaper();
        }
    }
    else
    {
        // Create an instance of GridShaper to determine the extents of the map to be produced and find its resolution
        shaper = BuildGridShaper();
    }
    if ( shaper || rusle2 )
    {
        PointGridder *pointGridder = NULL;
        if ( GRIDGEN_GRID_FIELDDATA )
        {
            // Build a PointGridder to grid the attribute data
            pointGridder = BuildPointGridder( shaper );
        }
        // pointGridder may be NULL if there is no griddd data and that is all right.
        if ( pointGridder || ! GRIDGEN_GRID_FIELDDATA )
        {
            // Create a FieldGridGen to assemble the parts into a map of field attribute values
            FieldGridGen *fieldGridGen = BuildFieldGridGen( rusle2, shaper, pointGridder );
            if (fieldGridGen)
            {
                delete fieldGridGen;
            }
            delete pointGridder;
        }
    }
    if ( rusle2 )
    {
        delete rusle2;
        shaper = NULL;
    }
    if ( shaper )
    {
        delete shaper;
    }

    return 0;
}



