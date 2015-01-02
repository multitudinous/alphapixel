// Project headers
#include "RUSLE2.h"
#include "FieldGridGen/FieldGridGen.h"

// C++ headers
#include <iostream>

using namespace rusle2;
using namespace fieldgridgen;

///Instantiate an instance of class rusle2::Rusle2, pass arguments from commandline.
///Execute the rusle2::Rusle2::LoadAndOperate() method to perform all data input, analysis and output.
///Resulting files will be placed in the working directory.
int main(int argc, char** argv) {
    
    ///Default parameters may be overwritten by commandline arguments followed by values as in the example:
    ///Rusle2.exe -demfile "data/DEM_3M_I_85.img" -farmfile "data/JoeBobsFarm.shp" -resolution 3.0 -drainage .005 -pitfill 20

    ///"-demfile" The DEM file and path relative to working directory or an absolute path.
    std::string demName = "data/DEM_3M_I_39.img";
    ///"-farmfile" The shape file and path relative to working directory or an absolute path.
    std::string farmName = "data/field_trans.shp";
    ///"-resolution" Desired DEM sample resolution. Units are whatever the original DEM's projection parameters are.
    double resolution = 3.0;
    ///"-drainage" The area in acres that must feed a cell in
    ///order that the cell be considered a drainage channel. Larger values generate
    ///less and shorter channels. Smaller values generate more and longer channel lines.
    double channelDrainageAreaAcres = 1.0;
    ///"-pitfill" The depth limit for filling depressions in the terrain model. Units are the elevation units of the input DEM.
    double pitFillLevel = 10.0;

    // Parse commandline arguments.
    // Valid arguments are -demfile, -farmfile, -resolution, -drainage, -pitfill.
    // The file names can include a full path or a relative path from the working directory.
    // Numeric entities can be real or integer.
    std::cout << "Commandline args count = " << argc << std::endl;

    // First argument is executable name only.
    for ( int i = 1; i < argc - 1; i += 2 )
    {
        std::cout << "Argv[" << i << "] =" << argv[ i ] << "Argv[" << i + 1 << "] =" << argv[ i + 1 ] << std::endl;
        if ( std::string( argv[i] ) == "-demfile" )
        {
            demName = argv[ i + 1 ];
        }
        else if ( std::string( argv[i] ) == "-farmfile" )
        {
            farmName = argv[ i + 1 ];
        }
        else if ( std::string( argv[i] ) == "-resolution" )
        {
            resolution = atof( argv[ i + 1 ] );
        }
        else if ( std::string( argv[i] ) == "-drainage" )
        {
            channelDrainageAreaAcres = atof( argv[ i + 1 ] );
        }
        else if ( std::string( argv[i] ) == "-pitfill" )
        {
            pitFillLevel = atof( argv[ i + 1 ] );
        }
    }

    // Create an instance of Rusle2 class with the input parameters supplied
    Rusle2 *rusle = new Rusle2 ( demName, farmName, resolution, channelDrainageAreaAcres, pitFillLevel );
    if (rusle)
    {
        // Load and process the input files and write the output maps and vector files to the working directory.
        bool Success = rusle->LoadAndOperate();
        if (! Success )
        {
            std::cout << "Operation terminated prematurely. Not all outputs were generated properly." << std::endl;
        }
        else
        {
            //For use in field_gridder
            double geoTransforms[6];
            int cellsX, cellsY;
            std::string geoRef;
            rusle->GetOutputGeoTransform6( geoTransforms );
            rusle->GetSpatialReferenceString( geoRef );
            rusle->GetOutputFileDims( cellsX, cellsY );

            std::cout << "Operation terminated successfully. All outputs were generated properly." << std::endl;
        }
    }
    
    return 0;
}



