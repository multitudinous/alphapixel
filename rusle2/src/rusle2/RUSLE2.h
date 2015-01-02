#pragma once

// GDAL & OGR headers
#include <gdal_priv.h>
#include <gdal_alg.h>
#include <ogrsf_frmts.h>

// C++ headers
#include <string>
#include <map>

/** \mainpage RUSLE2 Documentation

 \section IntroSection Introduction

 The RUSLE2 package is a library that creates a drainage network from a digital elevation 
 model. It is based on GDAL and OGR for file handling and formatting of data internally. 

 \section LibrariesSection Libraries

 The RUSLE2 library is composed of a basic class that performs all the tasks
 required to produce the drainage network in a step by step process, first loading
 a DEM and a shape file which describes an area of interest, usually smaller than 
 the entire terrain model. Inputs to the process include a desired DEM sample resolution factor,
 a DEM file, a shape file, a drainage channel fraction and a maximum pit fill level.

 Several additional classes are used to produce an output shape file with an arrow for
 each DEM cell to indicate the direction of computed flow for the cell.

 Outputs include these raster files and the single vector file (with prepended area of interest file name):

 1. downsizedDEM.tif

 2. resampledDEM.tif (optional)

 3. aspect.tif

 4. proximalinflow.tif

 5. totalinflow.tif

 6. drainagechannel.tif

 7. arrows.shp

 File formats and extensions may differ depending on what drivers are available.

 \section AppsSection Applications
 \subsection testSection test
 */

/*!\namespace rusle2
 * The rusle2 namespace encompasses the entire library.
 */
namespace rusle2 {

#ifndef RUSLE2_EXPORT_
#define RUSLE2_EXPORT_

// Library export definitions
#if defined(_MSC_VER) || defined(__CYGWIN__) || defined(__MINGW32__) || defined( __BCPLUSPLUS__)  || defined( __MWERKS__)
    //#pragma warning( disable: 4251 ) // http://www.unknownroad.com/rtfm/VisualStudio/warningC4251.html
    #if defined(RUSLE2_EXPORTS)
        #define RUSLE2_EXPORT __declspec(dllexport)
    #else
        #define RUSLE2_EXPORT __declspec(dllimport)
    #endif
#else
    #define RUSLE2_EXPORT
#endif  

#endif

#define DEM_FROM_SERVER_EPSG 3857
#define TEMPFILE_WCSDATA "wcsFile.xml"
class GridShaper;

/*!\class rusle2::ValidRasterPointTest  RUSLE2.h src/RUSLE2.h
 * Templated class for testing validity of a value in an array.
 */
template < class T > class ValidRasterPointTest
{
public:
    ///Constructor.
    ///\param xCells The number of columns in the data array.
    ///\param yCells The number of rows in the data array.
    ///\param nodataTestAvailable A boolean that determines if a No Data value is to be tested.
    ///\param nodataValue The No Data value to be tested against if testing is enabled.
    ///\param data A pointer to the data array to be tested against.
    ValidRasterPointTest( const int& xCells, const int& yCells, const bool& nodataTestAvailable, const T& nodataValue, const T* data );
    ///Test a data cell to determine if it contains the No Data value.
    ///\return True if the cell contains a valid value, not the No Data value if
    ///No Data testing is enabled.
    bool TestDataPointNotNULL( const int& x, const int& y ) const;
    ///Test a data cell to determine if it contains a valid data value.
    ///\return True if the cell is inside the bounds of the array and contains a
    ///valid value, not the No Data value if No Data testing is enabled.
    bool TestPoint( const int& x, const int& y ) const;
    
    ///The number of columns in the data array.
    int m_xCells;
    ///The number of rows in the data array.
    int m_yCells;
    ///A boolean that determines if a No Data value is to be tested.
    bool m_nodataTestAvailable;
    ///The No Data value to be tested against if testing is enabled.
    T m_nodataValue;
    ///A pointer to the data array to be tested against.
    const T* m_data;
};

/*!\file RUSLE2.h
 * RUSLE2 class API.
 * \class rusle2::Rusle2Arrow RUSLE2.h src/RUSLE2.h
 * Class that contains information about the location and direction of an arrow
 * vector which can be output to a shape file. Arrow consists of a longer shaft line
 * and two shorter tip lines. Coordinates should be in the same feference
 * system as the DEM it overlays. the direction value relates to the enum list in 
 * Rusle2 class.
 */
class Rusle2Arrow
{
    public:
        ///Constructor
        ///\param cellX The column of the DEM the arrow pertains to, ordered from west to east.
        ///\param cellY The row of the DEM the arrow pertains to ordered from north to south.
        ///\param direction The direction in which the arrow points (range 0-8). 0 indicates no direction.
        Rusle2Arrow( const int& cellX, const int& cellY, const int& direction ) : m_cellX( cellX ), m_cellY( cellY ), m_direction( direction ) {};

        ///DEM column.
        ///\note Units = zero-based cells from left or west edge
        int m_cellX;
        ///DEM row.
        ///\note Units = zero-based cells from top or north edge
        int m_cellY;
        ///Arrow direction.
        ///\note Units = arbitrary values based on the enum in rusle2::Rusle2Direction
        ///with 0 indicating directionless
        int m_direction;
        ///Arrow tip x coordinate.
        ///\note Units = the same dimensional units as the underlying DEM's projection parameters
        double m_tipX;
        ///Arrow tip y coordinate.
        ///\note Units = the same dimensional units as the underlying DEM's projection parameters
        double m_tipY;
        ///Arrow shaft end x coordinate.
        ///\note Units = the same dimensional units as the underlying DEM's projection parameters
        double m_shaftEndX;
        ///Arrow shaft end y coordinate.
        ///\note Units = the same dimensional units as the underlying DEM's projection parameters
        double m_shaftEndY;
        ///First arrow tip end x coordinate.
        ///\note Units = the same dimensional units as the underlying DEM's projection parameters
        double m_tip1EndX;
        ///First arrow tip end y coordinate.
        ///\note Units = the same dimensional units as the underlying DEM's projection parameters
        double m_tip1EndY;
        ///Second arrow tip end x coordinate.
        ///\note Units = the same dimensional units as the underlying DEM's projection parameters
        double m_tip2EndX;
        ///Second arrow tip end y coordinate.
        ///\note Units = the same dimensional units as the underlying DEM's projection parameters
        double m_tip2EndY;
};

///Typedef for using Rusle2Arrow in a std::vector.
typedef std::vector< Rusle2Arrow > ArrowList;

/*!\class rusle2::ArrowDatabase  RUSLE2.h src/RUSLE2.h
 * Class that computes the location of a Rusle2Arrow's shaft
 * and tips using the spatial transform parameters of the DEM
 * that the arrows willoverlay. Arrows are computed to cross the
 * center of each DEM cell. Units of all members are the same 
 * as the underlying DEM's projection parameters as returned by 
 * GDAL command GetGeoTransform().
 */
class ArrowDatabase
{
    public:
        ///Constructor
        ///\param ULX The x coordinate of the upper left corner of the DEM.
        ///\param ULY The y coordinate of the upper left corner of the DEM.
        ///\param cellSizeX The x conversion factor from cells to distance or width of a cell.
        ///\param cellSizeY The y conversion factor from cells to distance or height of a cell.
        ArrowDatabase( const double& ULX, const double& ULY, const double& cellSizeX, const double& cellSizeY )
            : m_ULX( ULX ), m_ULY( ULY ), m_cellSizeX( cellSizeX ), m_cellSizeY( cellSizeY ) {};
        ///Compute the arrow shaft and tip locations in the same reference system as the underlying DEM.
        ///\param arrow The Rusle2Arrow class for which computaion is to be performed.
        ///\note Computational results are stored in the supplied class reference object.
        void computeArrowParts( Rusle2Arrow& arrow );

    private:
        ///Compute the center of the DEM cell in the same reference system as the underlying DEM.
        ///\param x The DEM column for which the center is to be found.
        ///\param y The DEM row for which the center is to be found.
        ///\param centerX The x coordinate of the center of the DEM cell.
        ///\param centerY The y coordinate of the center of the DEM cell.
        ///\note Computational results are stored in the supplied double references.
        void computeCellCenter( const int& x, const int& y, double& centerX, double& centerY ) const;

        ///The x coordinate of the upper left corner of the DEM.
        ///\note Units = the same dimensional units as the underlying DEM's projection parameters
        double m_ULX;
        ///The y coordinate of the upper left corner of the DEM.
        ///\note Units = the same dimensional units as the underlying DEM's projection parameters
        double m_ULY;
        ///The x conversion factor from cells to distance or width of a cell.
        ///\note Units = the same dimensional units as the underlying DEM's projection parameters
        double m_cellSizeX;
        ///The y conversion factor from cells to distance or height of a cell.
        ///\note Units = the same dimensional units as the underlying DEM's projection parameters
        double m_cellSizeY;
};

///Enum for flow directions.
enum Rusle2Direction {
    FLAT = 0,
    NORTH,
    NORTHEAST,
    EAST,
    SOUTHEAST,
    SOUTH,
    SOUTHWEST,
    WEST,
    NORTHWEST
    };

///Enum for flow directions.
/*enum Rusle2Direction {
    FLAT = 0,
    NORTH = 64,
    NORTHEAST = 128,
    EAST = 1,
    SOUTHEAST = 2,
    SOUTH = 4,
    SOUTHWEST = 8,
    WEST = 16,
    NORTHWEST = 32
};*/

///Enum for available rasters.
enum Rusle2Rasters {
    ASPECT = 0,
    PROXIMALINFLOW,
    TOTALINFLOW,
    CHANNEL
    };

/*!\class rusle2::Rusle2  RUSLE2.h src/RUSLE2.h
 * Class that creates a drainage network from a DEM and a farm outline along with an arrow
 * vector overlay showing the direction of drainage within each cell.
 */
class RUSLE2_EXPORT Rusle2
{
    public:
        ///Constructor.
        ///\param demFilename The name and path of the DEM file to be used for analysis. 
        ///Path can be absolute or relative to the working directory.
        ///\param farmFilename The name and path of the ESRI shape file that defines 
        ///the area of interest for analysis. Path can be absolute or 
        ///relative to the working directory.
        ///\param resolutionFactor Desired DEM sample resolution multiplier. Units are whatever the 
        ///supplied DEM's projection parameters are. 
        ///\param channelDrainageAreaAcres The area in acres that must feed a cell in
        ///order that the cell be considered a drainage channel. Larger values generate
        ///less and shorter channels. Smaller values generate more and longer channel lines.
        ///\param pitFillLevel The depth limit for filling depressions in the terrain model. 
        ///Units are the elevation units of the input DEM.
        Rusle2( const std::string& demFilename,  const std::string& farmFilename, const double& resolutionFactor, const double& channelDrainageAreaAcres, const double& pitFillLevel );
        ///Destructor.
        ///Closes and properly disposes of all the GDAL and OGR entities.
        ~Rusle2();
        ///Umbrella method that calls all the methods necessary to load, 
        ///analyze and create the desired outputs.
        ///\return True if all operations complete successfully.
        bool LoadAndOperate( );
        ///GeoTransforms are an array of 6 doubles that determine the placement and grid size ofa GDAL raster.
        ///\param paramArray6 The array of 6 doubles in which the transform factors will be returned.
        ///\return true if parameters successfully retrieved.
        bool GetOutputGeoTransform6( double* paramArray6 ) const;
        ///Fetches the spatial reference string for the output maps.
        ///\param refString The string into which the spatial reference will be copied.
        ///\return true if spatial reference successfully retrieved.
        bool GetSpatialReferenceString( std::string& refString ) const;
        ///Fetches the output map dimensions.
        ///\param cellsX The number of cells in X dimension.
        ///\param cellsY The number of cells in Y dimension.
        void GetOutputFileDims( int& cellsX, int& cellsY ) const;
        ///Fetches pointer to the GridShaper that created the DEM.
        ///\return Pointer to the GridShaper that created and downsized the DEM.
        GridShaper* GetGridShaper()    const { return m_gridShaper; };
        ///Fetches a value from the raster.
        ///\param xCell X cell to fetch value from.
        ///\param yCell Y cell to fetch value from.
        ///\param value Value to be filled in from the raster.
        ///\note The value indexes into a smaller raster than the complete DEM raster. Be sure the index
        ///matches this smaller raster later.
        ///\note 0,0 is the upper left hand corner.
        bool FetchCellValue( const Rusle2Rasters& rasterType, const int& xCell, const int& yCell, int& value ) const;
        ///Test if a polygon intersects with the farm outline.
        ///\param poly The polygon to test for intersection.
        ///\return True if intersection exists.
        bool TestFarmShapeIntersection( const OGRPolygon& poly ) const;
        ///Fetch the OGRCoordinateTransformation used to convert the farm polygons to the DEM's reference.
        ///\return The OGRCoordinateTransformation used to convert the farm polygons to the DEM's reference.
        OGRCoordinateTransformation* GetFarmTransformation() const;

    private:
        ///Initialize GDAL drivers.
        void Init();
        ///Load the reference DEM to be analyzed.
        ///\return The GDALDataset containing a raster band with the DEM elevation data and
        ///a spatial reference system plus projection parameters.
        ///\note The file name is supplied by the calling code when the class is instantiated.
        ///Any format supported by GDAL drivers on the host system can be used.
        ///the elevation band should be the first or only band and it should be of 32 bit 
        ///floating point data type.
        GDALDataset* BuildAspectDEM( );
        ///Determines the level required for fluid flow across most pits and depressions and
        ///defines the direciton of flow within each cell of an area that is either naturally flat
        ///or is made flat by the pit filling operation.
        ///\return True if the operation is completed successfully regardless of whether any
        ///depressions were filled or flat spots healed. False returns occur if there are file
        ///errors between the GDALDatasets involved in the operation.
        ///\note All inputs and outputs in the method are members of the Rusle2 class. The deepest
        ///amount of fill allowed is specified through the user interface in the pitFillLevel
        ///argument fo the rusle2::Rusle2 constructor. The units of the fill level are the elevation 
        ///units of the DEM.
        bool FixFlats( );
        ///After filling depressions and healing flow directions across them a list of arrows is
        ///built for input to the arrow database which is later written to a vector file.
        ///\param aspectBuffer The array of 8-bit values that characterize the flow direction
        ///of each cell in the DEM. Directions are of the type rusle2::Rusle2Direction. The array must be
        ///of the standard size created by CreateFarmSizeDataset();
        ///\param testNodata A boolean that determines if a No Data value is to be tested.
        ///\param nodataValue The No Data value to be tested against if testing is enabled.
        ///\note The Rusle2 member m_arrowList contains the resulting ArrowList which is an std::vector.
        void BuildArrowList( const GByte* aspectBuffer, const bool& testNodata, const GByte& nodataValue );
        ///Determines if any of the 8 immediately contiguous array cells are flat and directs the
        ///flow of those cells toward the central cell. The method is called as the final step in
        ///healing the flow across flat areas.
        ///\param centerCell The index value of the center cell to be investigated. The cell's 
        ///row and column are derived from the index.
        ///\param aspectBuffer The array of 8-bit values that characterize the flow direction
        ///of each cell in the DEM.
        ///\param flatCellMap An std::map used to store the flat cells being healed. Note that this
        ///map covers only the one immediate area of concern and will normally be rather small in size.
        ///\param recursive A boolean the determines if flat cells are healed recursively. Not
        ///recommened as it was determined that flow patterns resulting from that approach are not
        ///natural. Recursion is determined by the define value RUSLE2_FLAT_RECURSION_SOLUTION.
        ///\return The number of cells healed in the call to the method which will be up to 8
        ///unless recursion is enabled. A return of 0 indicates completion of the healing
        ///operation on the local level though other flat spots may remain to be operated upon.
        int FindAdjacentFlatCellsAndDirect( const int& centerCell, GByte* aspectBuffer, const std::map< int, float >& flatCellMap, const bool& recursive ) const;
        ///Compute the number of immediately contiguous cells draining into each DEM cell. 
        ///The number is stored as values from 0 to 8 with 0 meaning no inflow to the 
        ///cell and 8 meaning all surrounding cells flow into the cell.
        ///\return The inflow raster as a GDALDataset that contains an 8 bit 
        ///raster band, the new projection parameters and the same spatial reference as 
        ///the original DEM.
        ///\note The image will be saved to disk as a GeoTiff file if that driver is available.
        ///Otherwise the same format as the input DEM will be created.
        ///The output will be directed to the working directory. The image name will be
        ///proximalinflow with the area of interest file root name and an underscore prepended.
        GDALDataset* BuildProximalInflowDEM( ) const;
        ///Compute the total number of cells draining into each DEM cell. 
        ///The number is stored as values from 0 to the maximum value that can be stored
        ///as an int with 0 meaning no inflow to the cell.
        ///\return The total inflow raster as a GDALDataset that contains a 32 bit int
        ///raster band, the new projection parameters and the same spatial reference as 
        ///the original DEM.
        ///\note The image will be saved to disk as a GeoTiff file if that driver is available.
        ///Otherwise the same format as the input DEM will be created.
        ///The output will be directed to the working directory. The image name will be
        ///totalinflow with the area of interest file root name and an underscore prepended.
        GDALDataset* BuildTotalInflowDEM( );
        ///Compute the presence or absence of a defined drainage channel for each DEM cell.
        ///\return The drainage channel raster as a GDALDataset that contains an 8 bit
        ///raster band, the new projection parameters and the same spatial reference as 
        ///the original DEM.
        ///\note The constructor parameter drainageChannelFraction is used to compute
        ///the cells that are considered channels. Smaller numbers make longer channels.
        ///\note The image will be saved to disk as a GeoTiff file if that driver is available.
        ///Otherwise the same format as the input DEM will be created.
        ///The output will be directed to the working directory. The image name will be
        ///drainagechannel with the area of interest file root name and an underscore prepended.
        GDALDataset* BuildDrainageChannelDEM( ) const;
        ///Create an OGRDataSource containing arrow vectors for each DEM cell that indicate
        ///the direction of flow in the cell.
        ///\param arrowFilename The base output file name for the arrow database.
        ///\param copyRasterSettings The GDALDataset containing the projection parameters
        ///and spatial reference for the underlying DEM.
        ///\return The OGRDataSource containing the arrow vectors, georeferenced the
        ///same as the underlying DEM.
        ///\note The vectors will be saved to disk as a shape file if the
        ///area of interest file was in that format. Otherwise the same format will be used
        ///as the area of interest.
        ///The output will be directed to the working directory. The file name will be
        ///arrows with the area of interest file root name and an underscore prepended.
        OGRDataSource* CreateArrowDatabase( const std::string& arrowFilename, GDALDataset* copyRasterSettings ) const;
        ///Fetch the DEM cell coordinate pair given a starting cell and a direction to move.
        ///\param direction The direction of input cell as a type enum rusle2::Rusle2Direction.
        ///\param inX The column of the DEM cell from which to move.
        ///\param inY The row of the DEM cell from which to move.
        ///\param outX The column of the DEM cell to which direction points.
        ///\param outY The row of the DEM cell to which direction points.
        ///\return True if one of the named directions is found, false if the cell is directionless.
        ///If false the values in outX and outY are undefined. 
        bool GetDirectionalIndices( const Rusle2Direction& direction, const int& inX, const int& inY, int& outX, int& outY ) const;

        ///Try a recursive method with complete search for finding the total inflow for cells
        GUInt32 GetTotalInflow( int currentX, int currentY, int startX, int startY, GByte* aspectBuffer, ValidRasterPointTest< GByte >* aspectTest );

        ///Try to create a reasonable aspect raster
        void BuildAspectRaster2( GDALDataset* demDataSet, GDALDataset* aspectDataSet );

        GridShaper* m_gridShaper;
        ///The cropped DEM data resampled to the resolution factor specified by the user in the
        ///resolutionFactor argument of the rusle2::Rusle2 constructor. Areas of no data may be present.
        GDALDataset* m_resampleDEM;
        ///The 8-bit image raster containing the direction of slope for each DEM cell.
        ///Directions are of the type rusle2::Rusle2Direction. Areas of no data may be present.
        GDALDataset* m_aspectDEM;
        ///The 8-bit image raster containing the number of immediately contiguous DEM
        ///cells whose flow is directed to the cell in the DEM. Minimum value is 0,
        ///maximum is 8. Areas of no data may be present.
        GDALDataset* m_proximalInflowDEM;
        ///The 32-bit integer raster containing the number of cells draining through
        ///the DEM cell. The minimum is 0 and the maximum is the number of cells in the
        ///DEM though that value would be most unusual to achieve. Areas of no data will
        ///also have a value of 0.
        GDALDataset* m_totalInflowDEM;
        ///The 8-bit image raster containing the off or on indication of a drainage channel.
        ///The presence of a channel is indicated by the highest positive number that can be
        ///represened by a signed char. No data areas will have a value of 0 as well.
        GDALDataset* m_drainageChannelDEM;
        ///The area of interest polygon read from an ESRI shape file or other vector source.
        ///The spatial reference will be that of the input DEM as the original polygon
        ///is transformed upon loading. 
        OGRDataSource* m_farmShape;
        ///The vector data for arrows representing flow direction in each DEM cell.
        OGRDataSource* m_arrowShape;
        ///The spatial reference for the input DEM as well as the loaded area of interest polygon
        ///and all the derived and saved to disk maps and images.
        OGRSpatialReference m_demRef;
        ///The full path and file name of the input DEM supplied by the user in the 
        ///rusle2::Rusle2 constructor. It may be a complete path or relative to the working directory.
        std::string m_demFilename;
        ///The full path and file name of the input area of interest file supplied by the user in the 
        ///rusle2::Rusle2 constructor. It may be a complete path or relative to the working directory.
        std::string m_farmFilename;
        ///The base file name of the area of interest, stripped of path and extension for use in
        ///making output file names.
        std::string m_baseName;
        ///Desired DEM sample resolution factor specified in rusle2::Rusle2 constructor.
        double m_resolutionFactor;
        ///Desired drainagel channel fraction specified in rusle2::Rusle2 constructor.
        double m_channelDrainageAreaAcres;
        ///Desired maximum pit filling elevation level specified in rusle2::Rusle2 constructor.
        double m_pitFillLevel;
        ///Computed high column value underlying the area of interest as a real number.
        double m_cellMaxX;
        ///Computed low column value underlying the area of interest as a real number.
        double m_cellMinX;
        ///Computed high row value underlying the area of interest as a real number.
        double m_cellMaxY;
        ///Computed low row value underlying the area of interest as a real number.
        double m_cellMinY;
        ///Computed high column value underlying the area of interest as an integer.
        int m_iCellMaxX;
        ///Computed low column value underlying the area of interest as an integer.
        int m_iCellMinX;
        ///Computed high row value underlying the area of interest as an integer.
        int m_iCellMaxY;
        ///Computed low row value underlying the area of interest as an integer.
        int m_iCellMinY;
        ///The number of columns of DEM underlying the area of interest.
        ///The number of columns that will be in the derived maps and images.
        int m_iCellRangeX;
        ///The number of rows of DEM underlying the area of interest.
        ///The number of rows that will be in the derived maps and images.
        int m_iCellRangeY;
        ///Computed cutoff value for number of total inflow cells in order to
        ///qualify as a drainage channel. Derived from the input drainageChannelFraction
        ///in the rusle2::Rusle2 constructor and the highest flow value found in the
        ///total flow map m_totalInflowDEM.
        unsigned int m_channelMinimumInflow;
        ///Bounds of the area of interest computed automatically by OGR in the
        ///spatial reference of the input DEM which is also the reprojected 
        ///reference for the AOI.
        //OGREnvelope m_farmBounds;
        ///True if there is a spatial reference for the input DEM to which the 
        ///area of interest can be reprojected.
        bool m_reprojectAvailable;
        ///True if there are flat areas in the DEM that should be healed so flow 
        ///can be linked across them.
        bool m_fixFlats;
        ///The list of information about each cell that is used to build a vectorized map
        ///of flow directions.
        ArrowList m_arrowList;


};

///Templated method for finding the maximum and minimum values in a GDALRasterBand object.
///\param band The GDALRasterBand object to be searched for values.
///\param min The minimum value found.
///\param max The maximum value found.
///\return True if at least one valid value was found, false if no valid values were found.
template < typename T > bool FindBandValidValueRange( GDALRasterBand* band, T& min, T& max );

///Templated method for finding the maximum and minimum values in a GDALRasterBand object.
///\note 0,0 is the upper left hand corner.
///\param band The GDALRasterBand object to be searched for values.
///\param value The value found.
///\return True if valid value was found, false if valid value not were found.
template < typename T > bool FindBandCellValue( GDALRasterBand* band, const int& xCell, const int& yCell, T& value )
{
    //GDALDataType dataType = band->GetRasterDataType();
    double dValue;
    int nodataTestAvailable;
    double nodataValue = band->GetNoDataValue( &nodataTestAvailable );
    int xCells = band->GetXSize();
    int yCells = band->GetYSize();
    CPLErr cplErr;
    
    // Fetch a cell of data
    if ( xCell >= 0 && xCell < xCells && yCell >= 0 && yCell < yCells )
    {
        cplErr = band->RasterIO( GF_Read, xCell, yCell, 1, 1, &dValue, 1, 1, GDT_Float64, 0, 0 );
        if ( cplErr == CE_Failure )
        {
            return false;
        }
        if ( nodataTestAvailable )
        {
            if ( dValue == nodataValue )
            {
                return false;
            }
            value = static_cast<T>( dValue );
        }
        return true;
    }
    return false;
}

///Utility method to retrieve the file portion of a string.
///\param fullPath The string to be searched for the file portion.
///\return A string containing only the file part of a full path,
///stripped of any path components. It includes the extension if there is one.
std::string GetFilePart(const std::string& fullPath);

///Utility method to strip off the final file name extension in a string.
///\param fileName The string to be stripped of the final file extension.
///\return A string containing only the actual name of the file minus any 
///extension that may have been present in the input string.
std::string StripExtension(const std::string& fileName);

class RUSLE2_EXPORT GridShaper
{
    public:
        ///Constructor.
        ///\param demFilename The name and path of the DEM file to be used for analysis. 
        ///Path can be absolute or relative to the working directory.
        ///\param farmFilename The name and path of the ESRI shape file that defines 
        ///the area of interest for analysis. Path can be absolute or 
        ///relative to the working directory.
        ///\param resolutionFactor Desired DEM sample resolution multiplier. Units are whatever the 
        ///supplied DEM's projection parameters are. 
        ///\param channelDrainageAreaAcres The area in acres that must feed a cell in
        ///order that the cell be considered a drainage channel. Larger values generate
        ///less and shorter channels. Smaller values generate more and longer channel lines.
        ///\param pitFillLevel The depth limit for filling depressions in the terrain model. 
        ///Units are the elevation units of the input DEM.
        GridShaper( const std::string& demFilename,  const std::string& farmFilename, const double& resolutionFactor );
        ///Destructor.
        ///Closes and properly disposes of all the GDAL and OGR entities.
        ~GridShaper();
        ///Umbrella method that calls all the methods necessary to load, 
        ///analyze and create the desired outputs.
        ///\return True if all operations complete successfully.
        GDALDataset* LoadAndOperate( OGRDataSource* &farmshape );
        ///Numerous DEMs and images are built at the same size to cover the Area of interest 
        ///polygon. This method creates those GDALDataset objects either from scratch or by
        ///cloning and existing data set. Resulting data sets have one raster band of the type 
        ///specified and is spatially referenced the same as the input DEM.
        ///\param newFilename The name of the file that will be saved to disk with the new
        ///raster being created. The name of the input shape file will be prepended.
        ///\param copyMe A GDALDataset to be cloned if desired. The value can be NULL in
        ///which case a new GDALDataset is constructed from scratch in GeoTiff format.
        ///\param georefSource In the event a raster is to be built from scratch, this
        ///referenced GDALDataset provides the transformation parameters that will co-register
        ///the new data set with the others.
        ///\param dType In the event a raster is to be built from scratch, this is the data type
        ///that the raster will be composed of. Defined values in the GDAL framework are acceptable.
        ///If being built from scratch a valid data type must be provided and the default overriden.
        ///\return The construced GDALDataset, fully georeferenced and ready to use though the
        ///raster band will need to be cleared to a desired starting value.
        GDALDataset* CreateFarmSizeDataset( const std::string& newFilename, GDALDataset* copyMe, GDALDataset* georefSource, const GDALDataType& dType = GDT_Unknown ) const;
        ///Fetches the upper X bounds as a cell number for the part of the original DEM to be excerpted.
        ///\return The upper X bounds as a cell number for the part of the original DEM to be excerpted.
        double GetCellMaxX() const { return( m_cellMaxX ); };
        ///Fetches the lower X bounds as a cell number for the part of the original DEM to be excerpted.
        ///\return The lower X bounds as a cell number for the part of the original DEM to be excerpted.
        double GetCellMinX() const { return( m_cellMinX ); };
        ///Fetches the upper Y bounds as a cell number for the part of the original DEM to be excerpted.
        ///\return The upper Y bounds as a cell number for the part of the original DEM to be excerpted.
        double GetCellMaxY() const { return( m_cellMaxY ); };
        ///Fetches the lower Y bounds as a cell number for the part of the original DEM to be excerpted.
        ///\return The lower Y bounds as a cell number for the part of the original DEM to be excerpted.
        double GetCellMinY() const { return( m_cellMinY ); };
        ///Fetches the integerized upper X bounds as a cell number for the part of the original DEM to be excerpted.
        ///\return The integerized upper X bounds as a cell number for the part of the original DEM to be excerpted.
        int GetICellMaxX() const { return( m_iCellMaxX ); };
        ///Fetches the integerized lower X bounds as a cell number for the part of the original DEM to be excerpted.
        ///\return The integerized lower X bounds as a cell number for the part of the original DEM to be excerpted.
        int GetICellMinX() const { return( m_iCellMinX ); };
        ///Fetches the integerized upper Y bounds as a cell number for the part of the original DEM to be excerpted.
        ///\return The integerized upper Y bounds as a cell number for the part of the original DEM to be excerpted.
        int GetICellMaxY() const { return( m_iCellMaxY ); };
        ///Fetches the integerized lower Y bounds as a cell number for the part of the original DEM to be excerpted.
        ///\return The integerized lower Y bounds as a cell number for the part of the original DEM to be excerpted.
        int GetICellMinY() const { return( m_iCellMinY ); };
        ///Fetches the integerized X range as a cell number for the part of the original DEM to be excerpted.
        ///\return The integerized X range as a cell number for the part of the original DEM to be excerpted.
        int GetICellRangeX() const { return( m_iCellRangeX ); };
        ///Fetches the integerized Y range as a cell number for the part of the original DEM to be excerpted.
        ///\return The integerized Y range as a cell number for the part of the original DEM to be excerpted.
        int GetICellRangeY() const { return( m_iCellRangeY ); };
        ///GeoTransforms are an array of 6 doubles that determine the placement and grid size of a GDAL raster.
        ///\param paramArray6 The array of 6 doubles in which the transform factors will be returned.
        ///\return true if parameters successfully retrieved.
        bool GetOutputGeoTransform6( double* paramArray6 ) const;
        ///Fetches the spatial reference string for the output maps.
        ///\param refString The string into which the spatial reference will be copied.
        ///\return true if spatial reference successfully retrieved.
        bool GetSpatialReferenceString( std::string& refString ) const;
        ///Fetches the output map X dimension.
        ///\return The number of cells in X dimension.
        int GetDEMXSize() const;
        ///Fetches the output map Y dimension.
        ///\return The number of cells in Y dimension.
        int GetDEMYSize() const;
        ///Fetch the bounding box around the farm shape file in the DEM spatial reference system.
        ///\return The bounding envelope containing the farm outline in the DEM spatial reference system.
        OGREnvelope GetFarmBounds() const { return m_farmBounds; };
        ///Fetch the OGRCoordinateTransformation used to convert the farm polygons to the DEM's reference.
        ///\return The OGRCoordinateTransformation used to convert the farm polygons to the DEM's reference.
        OGRCoordinateTransformation* GetFarmTransformation() const { return m_farmTransform; };


        ///Get the resampled DEM layer
        GDALDataset* GetResampledDEM() const;
        ///Get the downsized DEM layer
        GDALDataset* GetDownsizedDEM() const;

    private:
        ///Initialize GDAL drivers.
        void Init();
        ///Load the reference DEM to be analyzed.
        ///\return The GDALDataset containing a raster band with the DEM elevation data and
        ///a spatial reference system plus projection parameters.
        ///\note The file name is supplied by the calling code when the class is instantiated.
        ///Any format supported by GDAL drivers on the host system can be used.
        ///the elevation band should be the first or only band and it should be of 32 bit 
        ///floating point data type.
        GDALDataset* LoadDEM();
        ///Load the reference DEM to be analyzed from a web server via WCS format request.
        ///\return The GDALDataset containing a raster band with the DEM elevation data and
        ///a spatial reference system plus projection parameters.
        GDALDataset* LoadDEMFromServer( );
        ///Load the ESRI shape file with an outline of the area of interest (AOI).
        ///\return The generated OGRDataSource containing the AOI outline and 
        ///spatial reference system.
        ///\note The file name is supplied by the calling code when the class is instantiated.
        ///While a shape file is expected, any vector format with an OGR driver can be used.
        OGRDataSource* LoadShape( );
        ///Crop the DEM to the area that contains the AOI plus a little surrounding area.
        ///for analysis.
        ///\return The cropped DEM as a GDALDataset that contains a 32 bit floating point
        ///raster band, the new projection parameters and the same spatial reference as 
        ///the original DEM.
        ///\note The DEM will be saved to disk as a GeoTiff file if that driver is available.
        ///Otherwise the same format as the input DEM will be created.
        ///The output will be directed to the working directory. The DEM name will be
        ///downsizedDEM with the shape file root name and an underscore prepended.
        ///Cropping can be disabled by defining RUSLE2_NO_DOWNSIZE.
        GDALDataset* DownsizeDEM( );
        ///Resample the DEM to the resolution factor specified by the caller when the class is
        ///instantiated. If the DEM is already that size, this operation is not performed.
        ///\return The resampled DEM as a GDALDataset that contains a 32 bit floating point
        ///raster band, the new projection parameters and the same spatial reference as 
        ///the original DEM.
        ///\note The DEM will be saved to disk as a GeoTiff file if that driver is available.
        ///Otherwise the same format as the input DEM will be created.
        ///The output will be directed to the working directory. The DEM name will be
        ///resampledDEM with the area of interest file root name and an underscore prepended.
        GDALDataset* ResampleDEM( ) const;
        ///Find the column and row boundaries in a GDALDataset (DEM) that encompass the bounding box 
        ///for the area of interest polygon.
        ///\param dataset The GDALDataset (DEM) to be intersected with the area of interest
        ///polygon.
        ///\param sizeAdjust If desired extra DEM cells may be added to provide some coverage
        ///beyond the actual AOI bounds. Typically this extra coverage would be to provide more 
        /// complete and accurate analysis near the bounds of the AOI.
        ///\return True if the area of interest intersects with the DEM and some cells are
        ///found to exist within the AOI.
        ///\note Resulting cell bounds and ranges are placed in the Rusle2 class members
        ///directly: m_cellMaxX, m_iCellMaxX, m_iCellRangeX and their related entities.
        ///\note Also note that the bounds of the AOI are taken from the user-provided
        ///shape file and the bounds automatically computed by OGR.
        bool FarmShapeToDEMCells( GDALDataset* dataset, const int& sizeAdjust = 0 );

        ///The original unmodified input DEM data set. Areas of no data may be present.
        GDALDataset* m_origDEM;
        ///The cropped DEM data covering only the area of interest. Areas of no data may be present.
        GDALDataset* m_downsizeDEM;
        ///The cropped DEM data resampled to the resolution factor specified by the user in the
        ///resolutionFactor argument of the rusle2::Rusle2 constructor. Areas of no data may be present.
        GDALDataset* m_resampleDEM;
        ///The area of interest polygon read from an ESRI shape file or other vector source.
        ///The spatial reference will be that of the input DEM as the original polygon
        ///is transformed upon loading. 
        OGRDataSource* m_farmShape;
        ///The transform to go from the spatial reference of the incoming farm shape
        ///to the spatial reference of the DEM.
        OGRCoordinateTransformation* m_farmTransform;
        ///The spatial reference for the input DEM as well as the loaded area of interest polygon
        ///and all the derived and saved to disk maps and images.
        OGRSpatialReference m_demRef;
        ///The full path and file name of the input DEM supplied by the user in the 
        ///rusle2::Rusle2 constructor. It may be a complete path or relative to the working directory.
        std::string m_demFilename;
        ///The full path and file name of the input area of interest file supplied by the user in the 
        ///rusle2::Rusle2 constructor. It may be a complete path or relative to the working directory.
        std::string m_farmFilename;
        ///The base file name of the area of interest, stripped of path and extension for use in
        ///making output file names.
        std::string m_baseName;
        ///Desired DEM sample resolution factor specified in rusle2::Rusle2 constructor.
        double m_resolutionFactor;
        ///Computed high column value underlying the area of interest as a real number.
        double m_cellMaxX;
        ///Computed low column value underlying the area of interest as a real number.
        double m_cellMinX;
        ///Computed high row value underlying the area of interest as a real number.
        double m_cellMaxY;
        ///Computed low row value underlying the area of interest as a real number.
        double m_cellMinY;
        ///Computed high column value underlying the area of interest as an integer.
        int m_iCellMaxX;
        ///Computed low column value underlying the area of interest as an integer.
        int m_iCellMinX;
        ///Computed high row value underlying the area of interest as an integer.
        int m_iCellMaxY;
        ///Computed low row value underlying the area of interest as an integer.
        int m_iCellMinY;
        ///The number of columns of DEM underlying the area of interest.
        ///The number of columns that will be in the derived maps and images.
        int m_iCellRangeX;
        ///The number of rows of DEM underlying the area of interest.
        ///The number of rows that will be in the derived maps and images.
        int m_iCellRangeY;
        ///Bounds of the area of interest computed automatically by OGR in the
        ///spatial reference of the input DEM which is also the reprojected 
        ///reference for the AOI.
        OGREnvelope m_farmBounds;
        ///True if there is a spatial reference for the input DEM to which the 
        ///area of interest can be reprojected.
        bool m_reprojectAvailable;
        ///True if the output image dimensions need to be a multiple of 3 to accomodate a 9:1 downsample.
        bool m_hewTo3xMultipleSize;
};

#define OUTPUT_GRIDFILE_NAME "point_gridder_output.img"

class RUSLE2_EXPORT PointGridder
{
    public:
        ///Constructor.
        ///\param gridFullSize Determines if gridding should take place at the full resolution 
        ///of the Rusle2/GridShaper output or at the reduced resolution which is 3x the linear cell dimensions.
        PointGridder( const bool& gridFullSize );
        ///Destructor.
        ///Free resources.
        ~PointGridder();
        ///Initialize GDAL drivers.
        void Init();
        ///Initialize the size of the output grid explicitly.
        ///\param xSize Number of grid cells in the X dimension.
        ///\param ySize Number of grid cells in the Y dimension.
        void SetGridSize( const int& xSize, const int& ySize );
        ///Initialize the size of the output grid taking into account that the supplied dimensions 
        ///come from Rusle2 or GridShaper output and need to be modified according to m_gridFullSize.
        ///\param xSize Number of grid cells in the X dimension.
        ///\param ySize Number of grid cells in the Y dimension.
        void SetGridSizeFromRusle2( const int& xSize, const int& ySize );
        ///Initialize grid bounds explicitly.
        ///\param xLow Low X dimension bound of the grid in units of the spatial reference provided through SetGridSpatialReference().
        ///\param xHigh High X dimension bound of the grid in units of the spatial reference provided through SetGridSpatialReference().
        ///\param yLow Low Y dimension bound of the grid in units of the spatial reference provided through SetGridSpatialReference().
        ///\param yHigh High Y dimension bound of the grid in units of the spatial reference provided through SetGridSpatialReference().
        void SetGridBounds( const double& xLow, const double& xHigh, const double& yLow, const double& yHigh );
        ///Initialize grid bounds through provided GeoTransform array from GDAL and grid size in cells.
        ///\param paramArray6 The GeoTransform parameters from the output of Rusle2/GridShaper.
        ///GeoTransforms are an array of 6 doubles that determine the placement and grid size ofa GDAL raster.
        ///\param cellsX Grid size to be generated in the X dimension.
        ///\param cellsY Grid size to be generated in the Y dimension.
        void SetGridBoundsFromRusle2GeoTransform( const double* paramArray6, const int& cellsX, const int& cellsY );
        ///Gridding will be done with a search radius of nearby cells for data points.
        ///\param xOverlap Number of cells to search in the X dimension from the center of the cell being gridded.
        ///\param yOverlap Number of cells to search in the Y dimension from the center of the cell being gridded.
        void SetCellOverlapSearchFraction( const double& xOverlap, const double& yOverlap );
        ///Set the data type for the gridded output.
        ///\param dataType The format of the gridded data values specified as an GDALDataType enum .
        void SetDataType( const GDALDataType& dataType );
        ///Set the spatial reference for the gridded output.
        ///\param wktRefString The spatial reference in which the output grid is to be generated.
        void SetGridSpatialReference( const std::string& wktRefString );
        ///Set the attribute field name for the values to be gridded from the input point data file.
        ///\param gridAttribute A sring containing the exact name of the attribute field to be gridded.
        void SetGridAttribute( const std::string& gridAttribute );
        ///Set the point file name in which the values to be gridded reside.
        ///\param  pointFileName The file name and path either relative to the working directory or an 
        ///absolute path to the input file containing values to be gridded.
        void SetPointFileName( const std::string& pointFileName );
        ///Load the point file containing the values to be gridded.
        ///\return True if successful.
        bool LoadPointFile();
        ///Read the point data file and extract the points and attribute value to be fed to the gridding algorithm.
        ///Coordinate ranges found in the data points will be filled into the passed references.
        ///\param xLow A reference to the lowest X coordinate found in the points to be gridded.
        ///\param xHigh A reference to the highest X coordinate found in the points to be gridded.
        ///\param yLow A reference to the lowest Y coordinate found in the points to be gridded.
        ///\param yHigh A reference to the highest Y coordinate found in the points to be gridded.
        bool ParsePointDataset( double& xLow, double& xHigh, double& yLow, double& yHigh );
        ///Invoke the GDAL gridding algorithm which executes gridding on the data points.
        ///\param suppliedStorage An optional pointer to the correct amount of storage for the gridded data.
        ///\return Storage for the gridded data. The same as input suppliedStorage if non-NULL.
        ///If suppliedStorage is NULL then a new array will be returned. Caller is responsible for
        ///memory cleanup in either event.
        void* GridPointFile( void* suppliedStorage = NULL );
        ///Writes the gridded raster to disk.
        ///\param griddedRaster A pointer to the data to be stored on disk.
        ///\return True if the raster was saved to disk successfully.
        bool StoreGridRaster( void* griddedRaster );
        ///Pulls a value from the gridded data array based on supplied cell column and row.
        ///\note 0,0 is the upper left hand corner.
        ///\param xCell The cell number in the X dimension for which to retrieve the gridded value.
        ///\param yCell The cell number in the Y dimension for which to retrieve the gridded value.
        ///\return The value found in the cell.
        double SampleGridDataByCell( const int& xCell, const int& yCell ) const;
        ///Pulls a value from the gridded data array based on supplied coordinates. Coordinates are
        ///assumed to be in the spatial reference of the gridded data.
        ///\param xPos The X coordinate for which to retrieve the gridded value.
        ///\param yPos The Y coordinate for which to retrieve the gridded value.
        ///\return The value found at the coordinates.
        double SampleGridDataByCoords( const double& xPos, const double& yPos ) const;
        ///Gets the directive to grid the data at the full Rusle2 resolution or not.
        ///\return True if the gridding is at the full Rusle2 resolution, False if gridding is at the 1/3 resolution
        ///which results in a 1:9 downsampling over a 3x3 cell array.
        bool GetGridFullResolution() { return m_gridFullSize; };

    private:
        ///The file name and path either relative to the working directory or an 
        ///absolute path to the input file containing values to be gridded.
        std::string m_pointFileName;
        ///The attribute field name for the values to be gridded from the input point data file.
        std::string m_gridAttribute;
        ///The spatial reference in WKT
        std::string m_wktRefString;
        ///X dimension number of cells to generate in gridding process.
        int m_xSize;
        ///Y dimension number of cells to generate in gridding process.
        int m_ySize;
        ///Low X dimension bounds of gridding region.
        double m_xLow;
        ///High X dimension bounds of gridding region.
        double m_xHigh;
        ///Low Y dimension bounds of gridding region.
        double m_yLow;
        ///High Y dimension bounds of gridding region.
        double m_yHigh;
        ///Fractional number of cells from which to search for available data points in X dimension.
        ///The value is actually a radius in cell units. A value of 0 might still miss corner points
        ///within the cell. Defaults to .5 but will probably need to be higher when m_gridFullSize is true.
        double m_xCellOverlapFraction;
        ///Fractional number of cells from which to search for available data points in Y dimension.
        ///The value is actually a radius in cell units. A value of 0 might still miss corner points
        ///within the cell. Defaults to .5 but will probably need to be higher when m_gridFullSize is true.
        double m_yCellOverlapFraction;
        ///Number of input data values to be gridded.
        int m_numPoints;
        ///An array of doubles equal to m_numPoints containing the X coordinates of the data values.
        double* m_xPoints;
        ///An array of doubles equal to m_numPoints containing the Y coordinates of the data values.
        double* m_yPoints;
        ///An array of doubles equal to m_numPoints containing the data values to be gridded.
        double* m_zPoints;
        ///Enables keeping full resolution from the rusle2 work, if false the output map will be reduced.
        bool m_gridFullSize;
        ///The OGR data source containing the points to be gridded.
        OGRDataSource* m_pointDataSet;
        ///The grid raster stored on disk by StoreGridRaster().
        GDALDataset* m_gridDataset;
        ///Data type as GDALDataType enumerated value. Defaults to GDT_Float32.
        GDALDataType m_dataType;
        ///The algorithm to be used for gridding. An enum of type GDALGridAlgorithm.
        ///Defaults to Moving Average.
        GDALGridAlgorithm m_gridAlgorithm;

};

}
