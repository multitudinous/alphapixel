#pragma once

// GDAL & OGR headers
#include <gdal_priv.h>
#include <ogrsf_frmts.h>
#include <cairo.h>

// C++ headers
#include <string>

// Mapnik header
#include <mapnik/map.hpp>

#define BOOST_ALL_NO_LIB

namespace rusle2
{
class PointGridder;
class Rusle2;
}

/** \mainpage FieldGridGen Documentation

 \section IntroSection Introduction

 The FieldGridGen package is a library that ...

 \section LibrariesSection Libraries

 The FieldGridGen library is composed of a basic class that performs all the tasks
 required to 

 \section AppsSection Applications
 \subsection testSection test
 */

/*!\namespace fieldgridgen
 * The fieldgridgen namespace encompasses the entire library.
 */
namespace fieldgridgen {

#ifndef FIELDGRIDGEN_EXPORT_
#define FIELDGRIDGEN_EXPORT_

// Library export definitions
#if defined(_MSC_VER) || defined(__CYGWIN__) || defined(__MINGW32__) || defined( __BCPLUSPLUS__)  || defined( __MWERKS__)
    //#pragma warning( disable: 4251 ) // http://www.unknownroad.com/rtfm/VisualStudio/warningC4251.html
    #if defined(FieldGridGen_EXPORTS)
        #define FieldGridGen_EXPORT __declspec(dllexport)
    #else
        #define FieldGridGen_EXPORT __declspec(dllimport)
    #endif
#else
    #define FieldGridGen_EXPORT
#endif  

#endif

/*!\struct fieldgridgen::ColorIndexRGB  FieldGridGen.h src/FieldGridGen.h
*/
struct ColorIndexRGB
{
    int r, g, b;
};

/*!\struct fieldgridgen::Point2D  FieldGridGen.h src/FieldGridGen.h
*/
struct Point2D
{
    double x, y;
};

/*!\class fieldgridgen::ColorIndexRange  FieldGridGen.h src/FieldGridGen.h
*/
class FieldGridGen_EXPORT ColorIndexRange
{
    public:
        ColorIndexRange( const std::string& labelText, const std::string& fontName, const std::string& fontFilename, 
            const float& textSize, const struct ColorIndexRGB& rgb );
        ColorIndexRange( const ColorIndexRange& cir );
        ColorIndexRange();

        std::string m_labelText;
        std::string m_fontName;
        std::string m_fontFilename;
        float m_textSize;
        struct ColorIndexRGB m_rgb;
};

typedef std::map< float, class ColorIndexRange > ColorIndexList;

/*!\class fieldgridgen::KeyTitle  FieldGridGen.h src/FieldGridGen.h
*/
class FieldGridGen_EXPORT KeyTitle
{
    public:
        KeyTitle( const std::string& titleText, const std::string& fontName, const std::string& fontFilename, const float& textSize );

        std::string m_titleText;
        std::string m_fontName;
        std::string m_fontFilename;
        float m_textSize;
};

///KeyTitleList
typedef std::vector< class KeyTitle > KeyTitleList;

///Stores a polygon geometry as a GeoJSON string.
typedef std::string Geometry;
///Combines the label and geometry into one unit.
typedef std::pair< float, Geometry > GridCell;
///An array of polygon geometries with their label fields.
typedef std::vector< GridCell > GridCellArray;

///Stores the rusle2 values for a cell as well as the gridded field value
struct Rusle2GridgenData
{
    float fieldValue, demHeight;
    int aspect, proximalInflow, totalInflow;
    size_t channel;
    bool isField;
    int xCell, yCell;
};
///Stores a polygon geometry as a GeoJSON string.
typedef std::string Geometry;
///Combines the rusle2 values and geometry into one unit.
typedef std::pair< struct Rusle2GridgenData, Geometry > Rusle2Cell;
///An array of polygon geometries with their rusle2 and gridded values.
typedef std::vector< Rusle2Cell > Rusle2CellArray;



///Defines for file names.
///Map of gridded values
#define OUTPUT_FILE_NAME "field_gridgen_output.png"
///Map Legend
#define LEGEND_FILE_NAME "field_gridgen_legend.png"
///Temporary file of cell polygons
#define TEMPFILE_CELLPOLYS "field_gridgen_cellpolys"

/*!\class fieldgridgen::FieldGridGen  FieldGridGen.h src/FieldGridGen.h
*/
class FieldGridGen_EXPORT FieldGridGen
{
    public:
        ///Constructor.
        ///\param mapnikPath The path either relative to the working directory or an absolute path to the Mapnik root directory.
        ///\param iCellRangeX The range of cells in the X dimension returned by the GridShaper and used in Rusle2.
        ///\param iCellRangeY The range of cells in the Y dimension returned by the GridShaper and used in Rusle2.
        ///\param wktRefString The spatial reference output by the GridShaper and Rusle2 as WKT.
        ///\param projParams6 The GeoTransform array of the GridShaper and Rusle2 outputs.
        ///\param maxDimension The largest pixel dimension for the output image.
        ///\param renderFullSize True makes the grid resolution the same as used in GridShaper and Rusle2.
        ///False causes a 3x lower resolution on each linear axis.
        FieldGridGen( const std::string& mapnikPath,
                     const int& iCellRangeX,
                     const int& iCellRangeY,
                     const std::string& wktRefString,
                     const double *projParams6,
                     const int& maxDimension,
                     const bool& renderFullSize );

        ///Destructor.
        ///Closes and properly disposes of all the GDAL and OGR entities.
        ~FieldGridGen();
        ///Umbrella method that calls all the methods necessary to load, 
        ///analyze and create the desired outputs.
        ///\return True if all operations complete successfully.
        bool LoadAndOperate( rusle2::PointGridder* pointGridder );
        ///Add a new range for indexed color. Ranges will appear in the key block in ascending order.
        void AddColorIndexRange( const std::string& labelText, const std::string& fontName, const std::string& fontFilename, 
            const float& textSize, /*const float& lowerRangeLimit, */const float& upperRangeLimit, const struct ColorIndexRGB& rgb );
        ///Add title for the key block. Titles will appear in the key block in the order added.
        void AddKeyTitle( const std::string& titleText, const std::string& fontName, const std::string& fontFilename, const float& textSize );
        ///Fetch number of color ranges.
        ///\return Number of color ranges.
        int GetNumColorRanges();
        ///Fetch the array of cells that will be used for color and geometry input to the final map image.
        ///\return Reference to the grid cell array created by FieldGridGen with the LoadAndOperate method.
        GridCellArray& FetchGridCellArray()    { return m_gridCells; };
        ///Fetch an array of cells which physically overlap the farm and contain Rusle2 output data.
        ///\return Reference to the cell array created by FieldGridGen with the BuildRusle2CellArray method.
        Rusle2CellArray& FetchRusle2CellArray( rusle2::Rusle2 *rusle2 );
        ///Set cell border width in pixels for output map.
        ///\param cellBorderWidth Cell border width in pixels.
        void SetCellBorderWidth( const double& cellBorderWidth )    { m_cellBorderWidth = cellBorderWidth; };
        ///Enable or disable cell border drawing in output map.
        ///\param cellBordersEnabled True to draw cell borders as lines in output map.
        void SetCellBordersEnabled( bool cellBordersEnabled )    { m_cellBordersEnabled = cellBordersEnabled; };
        ///Fetch the grid X dimension in cells.
        ///\return The grid X dimension in cells.
        int GetGridSizeX() { return m_iNewCellRangeX; };
        ///Fetch the grid Y dimension in cells.
        ///\return The grid Y dimension in cells.
        int GetGridSizeY() { return m_iNewCellRangeY; };

    private:
        ///Initialize GDAL drivers.
        void Init();
        ///Initialize mapnik, find fonts and plugins.
        ///\return True if successful in initializing the mapnik module.
        bool InitMapnik() const;
        ///Build the array of grid cells with the GeoJson strings for each cell in the map.
        ///\param pointGridder the rusle2::PointGridder from which the gridded field data can be obtained. 
        ///\return True if successful.
        bool BuildGridCellArray( rusle2::PointGridder *pointGridder );
        ///Build an array of cells with the GeoJson strings for each cell in the map that physically overlap
        ///the actual farm boundaries and the sampled field data as well as data generated by 
        ///Rusle2 drainage calculations.
        ///\param pointGridder the rusle2::PointGridder from which the gridded field data can be obtained. 
        ///\param rusle2 the rusle2::Rusle2 from which the drainage data can be obtained. 
        ///\return True if successful.
        bool BuildRusle2CellArray( rusle2::Rusle2 *rusle2 );
        ///Create an OGR Dataset that has all the cell polygons and the color indices as attributes.
        ///\return True if successful.
        bool CreateCellPolygonDatabase( mapnik::Map *map );
        ///Compute new projection parameters and bounds for resampled map
        void ComputeNewMapParams();
        ///Compute output image size for resampled imagery
        void ComputeNewImageSize();
        ///Compute output image bounds for resampled imagery
        void ComputeNewImageBounds();
        ///Process the gridded data into an output map.
        ///\return True if completed successfully.
        bool FinalizeMap( GridCellArray& gridCells );
        ///Build the legend as a separate output image using Cairo.
        ///\return True if the legend is created successfully.
        bool BuildCairoLegend();
        ///Cairo operations are called to add a row of text to the legend.
        void AddTextRow( cairo_t *cr, const char *text,    unsigned int& width, unsigned int& height, double xOffset = 0.0, double yOffset = 0.0 );
        ///Cairo operations are called to add a row of legend text to the legend.
        void AddLegendTextRow( cairo_t *cr, const char *text, unsigned int& width, unsigned int& height, double *color );
        ///Cairo operations are called to create the legend.
        cairo_surface_t *CreateLegend( unsigned int& width, unsigned int& height );

        ///The number of columns of DEM underlying the area of interest.
        ///The number of columns that will be in the derived maps and images.
        int m_iCellRangeX;
        ///The number of rows of DEM underlying the area of interest.
        ///The number of rows that will be in the derived maps and images.
        int m_iCellRangeY;
        ///The number of columns that will be in the resampled map.
        int m_iNewCellRangeX;
        ///The number of rows that will be in the resampled map.
        int m_iNewCellRangeY;
        ///Maximum output image dimension
        int m_maxDimension;
        ///The number of pixels wide the output image will be.
        int m_iOutputPixelSizeX;
        ///The number of pixels high the output image will be.
        int m_iOutputPixelSizeY;
        ///Enabled state of drawing cell borders in output map.
        bool m_cellBordersEnabled;
        ///Enables keeping full resolution from the rusle2 work, if false the output map will be reduced.
        bool m_renderFullSize;
        ///The spatial reference in WKT
        std::string m_wktRefString;
        ///The projection parameters for use in GDAL
        double m_projParams6[6];
        ///The computed projection parameters for new imagery
        double m_newProjParams6[6];
        ///The cell border width in pixels.
        double m_cellBorderWidth;
        ///The computed upper left coords for output image
        struct Point2D m_iOutputBoundsUL;
        ///The computed lower right coords for output image
        struct Point2D m_iOutputBoundsLR;
        ///The OGR spatial reference that the map and all vectors need to end up in.
        OGRSpatialReference *m_outputSpatialRef;
        ///A string containing the spatial reference of the polygon data as Proj4.
        std::string m_finalProj4String;
        ///The path either relative to the working directory or an absolute path to the Mapnik root directory.
        std::string m_mapnikPath;
        ///The actual name of the font.
        std::string m_fontName;
        ///The file name for the desired font.
        std::string m_fontFilename;
        ///Array of key titles
        KeyTitleList m_keyTitleList;
        ///Array of color indexes.
        ColorIndexList m_colorIndexList;
        ///Array of grid cells.
        GridCellArray m_gridCells;
        ///Array of cells containing the Rusle2 drainage outputs as well as gridded field data and json coords.
        Rusle2CellArray m_Rusle2Cells;
        ///The png output filename
        std::string m_pngFilename;
        ///The png legend filename
        std::string m_pngLegendFilename;

};

}



