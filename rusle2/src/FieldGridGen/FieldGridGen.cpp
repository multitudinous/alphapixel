// Project headers
#include "FieldGridGen.h"
#include <rusle2/RUSLE2.h>

// C++ headers
#include <iostream>

// GDAL headers
#include <gdalwarper.h>
#include <cpl_port.h>

///Additional mapnik headers
#include <mapnik/layer.hpp>
#include <mapnik/rule.hpp>
#include <mapnik/line_symbolizer.hpp>
#include <mapnik/polygon_symbolizer.hpp>
#include <mapnik/text_symbolizer.hpp>
#include <mapnik/feature_type_style.hpp>
#include <mapnik/graphics.hpp>
#include <mapnik/datasource_cache.hpp>
#include <mapnik/font_engine_freetype.hpp>
#include <mapnik/agg_renderer.hpp>
#include <mapnik/expression.hpp>
#include <mapnik/color_factory.hpp>
#include <mapnik/image_util.hpp>

///Additional Cairo mapnik outputs if desired
#if defined(HAVE_CAIRO)
#include <mapnik/cairo_renderer.hpp>
#include <mapnik/cairo_context.hpp>
#endif

///Namespace mapnik
using namespace mapnik;
using namespace rusle2;

namespace fieldgridgen {

// Since creating a data source file can take some time it can be disabled for testing purposes
#define CREATE_NEW_DATASOURCEFILE
//#define REMOVE_CELLPOLY_TEMPFILE

FieldGridGen::FieldGridGen(
    const std::string& mapnikPath,
    const int& iCellRangeX,
    const int& iCellRangeY,
    const std::string& wktRefString,
    const double *projParams6,
    const int& maxDimension,
    const bool& renderFullSize )
    :
    m_iCellRangeX( iCellRangeX ),
    m_iCellRangeY( iCellRangeY ),
    m_maxDimension( maxDimension ),
    m_mapnikPath( mapnikPath ),
    m_wktRefString( wktRefString ),
    m_cellBorderWidth( 0.5 ),
    m_cellBordersEnabled( true ),
    m_renderFullSize( renderFullSize ),
    m_outputSpatialRef( 0 )
{
    m_projParams6[0] = projParams6[0];
    m_projParams6[1] = projParams6[1];
    m_projParams6[2] = projParams6[2];
    m_projParams6[3] = projParams6[3];
    m_projParams6[4] = projParams6[4];
    m_projParams6[5] = projParams6[5];
    std::cout << m_wktRefString << std::endl;
    Init();
}

FieldGridGen::~FieldGridGen()
{
#ifdef REMOVE_CELLPOLY_TEMPFILE
    std::string filename = TEMPFILE_CELLPOLYS;
    filename += ".shp";
    FILE *testOpen = fopen( filename.c_str(), "r" );
    if( testOpen )
    {
        fclose( testOpen );
        remove( filename.c_str() );
    }
    filename = TEMPFILE_CELLPOLYS;
    filename += ".shx";
    testOpen = fopen( filename.c_str(), "r" );
    if( testOpen )
    {
        fclose( testOpen );
        remove( filename.c_str() );
    }
    filename = TEMPFILE_CELLPOLYS;
    filename += ".dbf";
    testOpen = fopen( filename.c_str(), "r" );
    if( testOpen )
    {
        fclose( testOpen );
        remove( filename.c_str() );
    }
    filename = TEMPFILE_CELLPOLYS;
    filename += ".prj";
    testOpen = fopen( filename.c_str(), "r" );
    if( testOpen )
    {
        fclose( testOpen );
        remove( filename.c_str() );
    }
    if( m_outputSpatialRef )
    {
        delete m_outputSpatialRef;
    }
#else
    std::string filename = TEMPFILE_CELLPOLYS;
    filename += ".prj";
    FILE *testOpen = fopen( filename.c_str(), "r" );
    if( testOpen )
    {
        fclose( testOpen );
        remove( filename.c_str() );
        testOpen = fopen( filename.c_str(), "w" );
        fwrite( m_wktRefString.c_str(), 1, m_wktRefString.size(), testOpen );
        fclose( testOpen );
    }
#endif
}

void FieldGridGen::Init()
{
    // Register GDAL and OGR for file access
    GDALAllRegister();
    OGRRegisterAll();
    m_outputSpatialRef = new OGRSpatialReference( m_wktRefString.c_str() );
    char *proj4Description = NULL;
    m_outputSpatialRef->exportToProj4( &proj4Description );
    m_finalProj4String = proj4Description;
    m_pngFilename = OUTPUT_FILE_NAME;
    m_pngLegendFilename = LEGEND_FILE_NAME;
}

bool FieldGridGen::InitMapnik() const
{
    // Find the fonts used in key
    try
    {
        std::cout << " initializing mapnik ... \n";
        std::string mapnik_dir = m_mapnikPath;
        std::cout << " looking for 'shape.input' plugin in... " << mapnik_dir << "/mapnik/input/" << "\n";
        //std::cout << " looking for 'raster.input' plugin in... " << mapnik_dir << "/mapnik/input/" << "\n";
        datasource_cache::instance().register_datasources( mapnik_dir + "/mapnik/input/" );
        if( !datasource_cache::instance().register_datasource( mapnik_dir + "/mapnik/input/shape.input" ) )
        {
            std::cout << "Unable to find the the Mapnik shapefile plugin." << std::endl
                << "Make sure that the " << mapnik_dir << " directory is correct." << std::endl;
        }

        std::string lastFont;
        // For each title and color range index which might have a different font
        for ( KeyTitleList::const_iterator kit = m_keyTitleList.begin(); kit != m_keyTitleList.end(); ++kit )
        {
            if( kit->m_fontFilename != lastFont )
            {
                std::cout << " looking for " << kit->m_fontName << " font in... " << mapnik_dir << "/mapnik/fonts/" << kit->m_fontFilename << "\n";
                freetype_engine::register_font (mapnik_dir + "/mapnik/fonts/" + kit->m_fontFilename );
                lastFont = kit->m_fontFilename;
            }
        }
        for ( ColorIndexList::const_iterator cit = m_colorIndexList.begin(); cit != m_colorIndexList.end(); ++cit )
        {
            if( cit->second.m_fontFilename != lastFont )
            {
                std::cout << " looking for " << cit->second.m_fontName << " font in... " << mapnik_dir << "/mapnik/fonts/" << cit->second.m_fontFilename << "\n";
                freetype_engine::register_font (mapnik_dir + "/mapnik/fonts/" + cit->second.m_fontFilename );
                lastFont = cit->second.m_fontFilename;
            }
        }
    }
    catch ( const std::exception & ex )
    {
        std::cerr << "### InitMapnik std::exception: " << ex.what() << std::endl;
        return false;
    }
    catch ( ... )
    {
        std::cerr << "### InitMapnik Unknown exception." << std::endl;
        return false;
    }
    return true;
}

bool FieldGridGen::LoadAndOperate( rusle2::PointGridder* pointGridder )
{
    if( InitMapnik() )
    {
        if( BuildGridCellArray( pointGridder ) )
        {
            if( FinalizeMap( FetchGridCellArray() ) )
            {
                return BuildCairoLegend();
            }
        }
    }
    return false;
}

Rusle2CellArray& FieldGridGen::FetchRusle2CellArray( rusle2::Rusle2 *rusle2 )
{
    BuildRusle2CellArray( rusle2 );
    return m_Rusle2Cells;
}

bool FieldGridGen::BuildGridCellArray( rusle2::PointGridder *pointGridder )
{
    // pointGridder may be NULL if there is no gridded data.
    // make all the computations for the new map and output image
    ComputeNewImageSize();
    ComputeNewMapParams();
    ComputeNewImageBounds();

    // Populate m_gridCells
    // Compute each cell coords in the map's bounds
    Point2D cellCornerUL, cellCornerLR;
    float tempValue = 0.0f;
    
    //We are starting in the upper left hand corner and marching in x
    //and then march in Y.
    for ( int cellY = 0; cellY < m_iNewCellRangeY; ++cellY )
    {
        for ( int cellX = 0; cellX < m_iNewCellRangeX; ++cellX )
        {
            cellCornerUL.x = m_iOutputBoundsUL.x + cellX * m_newProjParams6[1];
            cellCornerUL.y = m_iOutputBoundsUL.y + cellY * m_newProjParams6[5];
            cellCornerLR.x = cellCornerUL.x + m_newProjParams6[1];
            cellCornerLR.y = cellCornerUL.y + m_newProjParams6[5];

            OGRPolygon poly;
            OGRLinearRing linearRing;

            linearRing.addPoint( cellCornerUL.x, cellCornerUL.y );
            linearRing.addPoint( cellCornerLR.x, cellCornerUL.y );
            linearRing.addPoint( cellCornerLR.x, cellCornerLR.y );
            linearRing.addPoint( cellCornerUL.x, cellCornerLR.y );
            linearRing.addPoint( cellCornerUL.x, cellCornerUL.y );
            poly.addRing( &linearRing );

            if( pointGridder )
            {
                tempValue = static_cast< float >( pointGridder->SampleGridDataByCell( cellX, cellY ) );
            }
            //tempValue = static_cast< float >( pointGridder->SampleGridDataByCoords( (cellCornerUL.x + cellCornerLR.x ),
            //    (cellCornerUL.y + cellCornerLR.y) ) );
            char *json = poly.exportToJson();
            m_gridCells.push_back( GridCell( tempValue, Geometry( json ) ) );
        }
    }
    std::cout << "Cells generated X: " << m_iNewCellRangeX << ", Y: " << m_iNewCellRangeY << " Total: " << m_gridCells.size() << std::endl;
    return true;
}

bool FieldGridGen::BuildRusle2CellArray( rusle2::Rusle2* rusle2 )
{
    if( !rusle2 )
    {
        return false;
    }

    GDALRasterBand* demBand = rusle2->GetGridShaper()->GetDownsizedDEM()->GetRasterBand( 1 );
    OGRCoordinateTransformation* farmTransform = rusle2->GetFarmTransformation();
    //Need to get the cell count to offset the overall textures with the rusle2 output textures
    int iCellMinX = rusle2->GetGridShaper()->GetICellMinX();
    int iCellMinY = rusle2->GetGridShaper()->GetICellMinY();
    int iCellMaxX = rusle2->GetGridShaper()->GetICellMaxX();
    int iCellMaxY = rusle2->GetGridShaper()->GetICellMaxY();
    int xCell = 0;
    int yCell = 0;
    bool insideFlowRaster = false;
    for( GridCellArray::iterator it = m_gridCells.begin(); it != m_gridCells.end(); ++it, ++xCell )
    {
        insideFlowRaster = false;
        //We step in the x direction first
        if( xCell == m_iNewCellRangeX )
        {
            xCell = 0;
            ++yCell;
        }
        // compute bounds of cell and test for overlap with farm outline
        Point2D cellCornerUL, cellCornerLR;
        cellCornerUL.x = m_iOutputBoundsUL.x + xCell * m_newProjParams6[1];
        cellCornerUL.y = m_iOutputBoundsUL.y + yCell * m_newProjParams6[5];
        cellCornerLR.x = cellCornerUL.x + m_newProjParams6[1];
        cellCornerLR.y = cellCornerUL.y + m_newProjParams6[5];

        OGRPolygon poly;
        OGRLinearRing linearRing;

        linearRing.addPoint( cellCornerUL.x, cellCornerUL.y );
        linearRing.addPoint( cellCornerLR.x, cellCornerUL.y );
        linearRing.addPoint( cellCornerLR.x, cellCornerLR.y );
        linearRing.addPoint( cellCornerUL.x, cellCornerLR.y );
        linearRing.addPoint( cellCornerUL.x, cellCornerUL.y );
        poly.addRing( &linearRing );

        //Transform the cell into the farm spatial reference
        if( farmTransform )
        {
            poly.transform( farmTransform );
        }

        struct Rusle2GridgenData rusle2GridgenData;
        rusle2GridgenData.aspect = 0;
        rusle2GridgenData.proximalInflow = 0;
        rusle2GridgenData.totalInflow = 0;
        rusle2GridgenData.channel = 0;
        rusle2GridgenData.fieldValue = it->first;
        rusle2GridgenData.demHeight = 0.0;
        rusle2GridgenData.isField = false;
        rusle2GridgenData.xCell = xCell;
        rusle2GridgenData.yCell = yCell;

        //Check to see if the cell values is within the bounds of the
        //flow rasters
        //The min value is >= because we are working with 0 base and 1 base numbers
        if( (xCell >= iCellMinX) && (yCell >= iCellMinY) )
        {
            //Only grab < because the max values are 1 base numbers
            if( (xCell < iCellMaxX) && (yCell < iCellMaxY) )
            {
                insideFlowRaster = true;
            }
        }

        //If we are outside the flow raster then there is no way we are
        //inside the field bounds
        if( insideFlowRaster )
        {
            //Is this cell inside the bounds of the field
            if( rusle2->TestFarmShapeIntersection( poly ) )
            {
                rusle2GridgenData.isField = true;
            }
        }
        
        // Rusle2 data only available at full Rusle2 resolution
        if( m_renderFullSize )
        {
            //Get the dem data since we are here. The DEM includes rows of buffer cells so do not
            //modify the cell count.
            float demHeight = 0;
            demBand->RasterIO( GF_Read, xCell, yCell, 1, 1, &demHeight, 1, 1, GDT_Float32, 0, 0 );
            rusle2GridgenData.demHeight = demHeight;

            if( insideFlowRaster )
            {
                //Account for the smaller rusle2 textures sizes
                int adjXCell = xCell - iCellMinX;
                int adjYCell = yCell - iCellMinY;

                rusle2GridgenData.xCell = adjXCell;
                rusle2GridgenData.yCell = adjYCell;

                // fetch rusle2 data
                int iValue;
                bool valueValid;
                
                //Sample the rusle textures
                valueValid = rusle2->FetchCellValue( ASPECT, adjXCell, adjYCell, iValue );
                rusle2GridgenData.aspect = valueValid ? iValue: 0;
                
                valueValid = rusle2->FetchCellValue( PROXIMALINFLOW, adjXCell, adjYCell, iValue );
                rusle2GridgenData.proximalInflow = valueValid ? iValue: 0;
                
                valueValid = rusle2->FetchCellValue( TOTALINFLOW, adjXCell, adjYCell, iValue );
                rusle2GridgenData.totalInflow = valueValid ? iValue: 0;
                
                valueValid = rusle2->FetchCellValue( CHANNEL, adjXCell, adjYCell, iValue );
                rusle2GridgenData.channel = valueValid ? iValue: 0;

                m_Rusle2Cells.push_back( Rusle2Cell( rusle2GridgenData, it->second ) );
            }
        }
    }
    return true;
}

bool FieldGridGen::FinalizeMap( GridCellArray& gridCells )
{
    m_gridCells = gridCells;

    try
    {
        Map map( m_iOutputPixelSizeX, m_iOutputPixelSizeY );
        map.set_background( parse_color( "black" ) );
        map.set_srs( m_finalProj4String );

        // Generate a shape file that can be fed to mapnik with all the geometries of cell outlines
        // and with each having an attribute representing the appropriate color index.
        bool success = CreateCellPolygonDatabase( &map );

        // Generate mapnik polygon fill rules for each index color
        // Generate a mapnik polygon line rule for all cell outlines

        map.zoom_to_box( box2d<double>( m_iOutputBoundsUL.x, m_iOutputBoundsLR.y, 
            m_iOutputBoundsLR.x, m_iOutputBoundsUL.y ) );

        image_32 buf( map.width(), map.height() );
        agg_renderer< image_32 > ren( map, buf );
        ren.apply();

        std::string msg( "\nThese maps have been rendered using AGG in the current directory:\n" );
#ifdef HAVE_PNG
        save_to_file( buf, m_pngFilename.c_str(), "png" );
        msg += "- ";
        msg += m_pngFilename;
        msg += "\n";
#endif
        std::cout << msg;
    }
    catch( const std::exception & ex )
    {
        std::cerr << "### FinalizeMap std::exception: " << ex.what() << std::endl;
        return false;
    }
    catch( ... )
    {
        std::cerr << "### FinalizeMap Unknown exception." << std::endl;
        return false;
    }
    return true;
}

bool FieldGridGen::CreateCellPolygonDatabase( mapnik::Map *map )
{

    std::cout << "Database creation begun." << std::endl;
    OGRDataSource *cellDataset = NULL;

    // Create new shape file datasource.
    OGRSFDriverRegistrar* Mgr = OGRSFDriverRegistrar::GetRegistrar();
    OGRSFDriver* driver = Mgr->GetDriverByName( "ESRI Shapefile" );

    // Remove old datasource if present.
    std::string filename = TEMPFILE_CELLPOLYS;
    filename += ".shp";
    FILE *testOpen = fopen( filename.c_str(), "r" );
    if( testOpen )
    {
        fclose( testOpen );
#ifdef CREATE_NEW_DATASOURCEFILE
        driver->DeleteDataSource( filename.c_str() );
#endif
    }
    
#ifdef CREATE_NEW_DATASOURCEFILE
    OGRDataSource *copySource = driver->CreateDataSource( filename.c_str(), NULL );
    OGRLayer *copyLayer;
    OGRFieldDefn *colorRangeFldDefn = new OGRFieldDefn( "INDEX", OFTString );
    if( colorRangeFldDefn )
    {
        colorRangeFldDefn->Set( "INDEX", OFTString, 8, 0, OJLeft );
    }
    else
    {
        std::cout << "Creating field definition failed." << std::endl;
    }

    OGRFeatureDefn *ogrFeatureDefn = new OGRFeatureDefn( "Poly" );
    int colorRangeFieldIndex;
    if( ogrFeatureDefn )
    {
        // Add Field Definitions to Feature Definition
        ogrFeatureDefn->AddFieldDefn( colorRangeFldDefn );
        // Get the field Indices from the Feature Def.
        colorRangeFieldIndex = ogrFeatureDefn->GetFieldIndex( colorRangeFldDefn->GetNameRef( ) );
    }
#endif

    feature_type_style cellpoly_style;

    std::cout << "Color Ranges" << std::endl;
    int colorRange = 0;
    for ( ColorIndexList::const_iterator it = m_colorIndexList.begin(); it != m_colorIndexList.end(); ++it, ++colorRange )
    {
        char itemstr[64];
        sprintf( itemstr, "[INDEX] = '%d'", colorRange );
        std::cout << colorRange << " " << it->second.m_labelText << " upper limit: " << it->first << std::endl;

        rule cellpoly_rule;
        cellpoly_rule.set_filter( parse_expression( itemstr ));
        polygon_symbolizer sym;
        sym.set_opacity( 1.0 );
        sym.set_gamma( 0.0 );
        sym.set_fill( color( it->second.m_rgb.r, it->second.m_rgb.g, it->second.m_rgb.b ) );
        cellpoly_rule.append( sym );
        cellpoly_style.add_rule( cellpoly_rule );
    }
    
#ifdef CREATE_NEW_DATASOURCEFILE
    int itemNum = 0;
    for ( GridCellArray::iterator cit = m_gridCells.begin(); cit != m_gridCells.end(); ++cit, ++itemNum )
    {
        int curColorRange = -1;
        colorRange = 0;
        for ( ColorIndexList::const_iterator it = m_colorIndexList.begin(); it != m_colorIndexList.end(); ++it, ++colorRange )
        {
            if( cit->first < it->first )
            {
                curColorRange = colorRange;
                break;
            }
        }

        if( curColorRange >= 0 )
        {
            // Valid range found
            // Create OGR Dataset from the geometry of the cell
            const char *geometry = cit->second.c_str();

            cellDataset = OGRSFDriverRegistrar::Open( geometry );
            if( cellDataset == NULL )
            {
                std::cout << "Cell geometry read failed" << std::endl;
                return 0;
            }

            OGRLayer *olayer = cellDataset->GetLayer( 0 );
            if( olayer )
            {
                OGRFeatureDefn *layerDef = olayer->GetLayerDefn();
                if( layerDef )
                {
                    OGRFeature *feature = olayer->GetFeature( 0 );
                    if( feature )
                    {
                        OGRGeometry *geom = feature->GetGeometryRef();
                        if( geom )
                        {
                            // Copy the dataset into the shape file dataset with the field attribute set to the color index
                            if( cit == m_gridCells.begin() )
                            {
                                copyLayer = copySource->CreateLayer( "Cell_Polygons", m_outputSpatialRef, geom->getGeometryType() );
                                if( copyLayer )
                                {
                                    if( copyLayer->CreateField( colorRangeFldDefn, 0 ) != OGRERR_NONE )
                                    {
                                        std::cout << "Creating color range index field failed." << std::endl;
                                    }

                                    OGRFeature *newFeature;
                                    newFeature = OGRFeature::CreateFeature( copyLayer->GetLayerDefn() );
                                    if( newFeature )
                                    {
                                        char colorRangeTxt[64];
                                        sprintf( colorRangeTxt, "%d", curColorRange );
                                        newFeature->SetField( colorRangeFieldIndex, colorRangeTxt );
                                        newFeature->SetGeometry( geom );
                                        OGRErr err = copyLayer->CreateFeature( newFeature );
                                        if( err != OGRERR_NONE )
                                        {
                                            std::cout << "Creating first cell feature failed." << std::endl;
                                        }
                                    }
                                }
                            }
                            else
                            {
                                OGRFeature *newFeature;
                                newFeature = OGRFeature::CreateFeature( copyLayer->GetLayerDefn() );
                                newFeature->SetField( colorRangeFieldIndex, curColorRange );
                                newFeature->SetGeometry( geom );
                                OGRErr err = copyLayer->CreateFeature( newFeature );
                                if( err != OGRERR_NONE )
                                {
                                    std::cout << "Creating cell feature " << itemNum << " failed. Type = " << geom->getGeometryType() << std::endl;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    copySource->SyncToDisk();
    OGRDataSource::DestroyDataSource ( copySource );
#endif
    std::cout << "Database creation complete." << std::endl;

    map->insert_style( "cellpolys", cellpoly_style );

    if( m_cellBordersEnabled )
    {
        // Add style to mapnik (polyline)
        feature_type_style celllines_style;

        stroke celllines_stk ( color( 0, 0, 0 ), m_cellBorderWidth );

        rule celllines_rule;
        celllines_rule.append( line_symbolizer( celllines_stk ) );
        celllines_style.add_rule( celllines_rule );

        map->insert_style( "celllines", celllines_style );
    }

    // Add dataset to mapnik (polyline)
    {
        parameters p;
        p["type"] = "shape";
        p["file"] = filename.c_str();
        std::cout << "Adding " << filename << " to the map." << std::endl;
        layer lyr( "Cell" );
        lyr.set_srs( m_finalProj4String.c_str() );
        try
        {
            datasource_ptr dsPtr = datasource_cache::instance().create( p );
            lyr.set_datasource( dsPtr );
        }
        catch( std::exception& ex )
        {
            std::cout << " Unable to add cell layer to the map layer: " << ex.what() << std::endl
                << " *** Make sure that the <mapnik install directory>/lib/mapnik/input/ directory is specified."
                << std::endl;
            return false;
        }
        lyr.add_style( "cellpolys" );
        if( m_cellBordersEnabled )
        {
            lyr.add_style( "celllines" );
        }
        map->addLayer( lyr );
    }
    return true;
}

void FieldGridGen::ComputeNewMapParams()
{
    int divisor = m_renderFullSize ? 1: 3;
    // Computations assume that in GDAL the projection parameter array specifies the upper left corner coords as center of cell
    // Compute m_iNewCellRangeX, m_iNewCellRangeY, m_newProjParams6
    m_iNewCellRangeX = m_iCellRangeX / divisor;
    m_iNewCellRangeY = m_iCellRangeY / divisor;

    // Rotations should be 0
    m_newProjParams6[2] = m_projParams6[2];
    m_newProjParams6[4] = m_projParams6[4];

    // New Scales
    m_newProjParams6[1] = m_projParams6[1] * divisor;    // X scale/cell
    m_newProjParams6[5] = m_projParams6[5] * divisor;    // Y scale/cell

    // Upper left X
    m_newProjParams6[0] = m_projParams6[0];        
    // Upper left Y
    m_newProjParams6[3] = m_projParams6[3];
    
    std::cout << "Rusle2 bounds: " << m_iCellRangeX << " x " << m_iCellRangeY
        << " UL: " << m_projParams6[0] << ", " << m_projParams6[3]
        << " LR: " << m_projParams6[0] + m_iCellRangeX * m_projParams6[1] << ", " << m_projParams6[3] + m_iCellRangeY * m_projParams6[5] << std::endl;
    std::cout << "Gridder bounds: " << m_iNewCellRangeX << " x " << m_iNewCellRangeY
        << " UL: " << m_newProjParams6[0] << ", " << m_newProjParams6[3]
        << " LR: " << m_newProjParams6[0] + m_iNewCellRangeX * m_newProjParams6[1] << ", " << m_newProjParams6[3] + m_iNewCellRangeY * m_newProjParams6[5] << std::endl;
}

void FieldGridGen::ComputeNewImageSize()
{
    // Computations assume that in GDAL the projection parameter array specifies the upper left corner coords as outside corner of cell
    // Compute m_iOutputPixelSizeX, m_iOutputPixelSizeY
    if( m_iCellRangeX > m_iCellRangeY )
    {
        m_iOutputPixelSizeX = m_maxDimension;
        m_iOutputPixelSizeY = static_cast<int>( m_maxDimension * ( static_cast<double>( m_iCellRangeY ) / static_cast<double>( m_iCellRangeX ) ) );
    }
    else
    {
        m_iOutputPixelSizeY = m_maxDimension;
        m_iOutputPixelSizeX = static_cast<int>( m_maxDimension * ( static_cast<double>( m_iCellRangeX ) / static_cast<double>( m_iCellRangeY ) ) );
    }
}

void FieldGridGen::ComputeNewImageBounds()
{
    // Compute m_iOutputBoundsUL, m_iOutputBoundsLR
    // Results will be used in mapnik:    map.zoom_to_box( box2d<double>( m_iOutputBoundsUL.x, m_iOutputBoundsLR.y, 
    //        m_iOutputBoundsLR.x, m_iOutputBoundsUL.y ) );
    m_iOutputBoundsUL.x = m_newProjParams6[0];
    m_iOutputBoundsUL.y = m_newProjParams6[3];
    m_iOutputBoundsLR.x = m_newProjParams6[0] + m_iNewCellRangeX * m_newProjParams6[1];
    m_iOutputBoundsLR.y = m_newProjParams6[3] + m_iNewCellRangeY * m_newProjParams6[5];
}

void FieldGridGen::AddColorIndexRange( const std::string& labelText, const std::string& fontName, const std::string& fontFilename, 
    const float& textSize, const float& upperRangeLimit, const struct ColorIndexRGB& rgb )
{
    const ColorIndexRange cir( labelText, fontName, fontFilename, textSize, rgb );
    m_colorIndexList[ upperRangeLimit ] = cir;
    /* Test
    ColorIndexList::iterator it = m_colorIndexList.find( upperRangeLimit );
    if( it != m_colorIndexList.end() )
    {
        std::cout << it->second.m_textSize << " " << it->second.m_fontName << std::endl;
    }
    */
}

void FieldGridGen::AddKeyTitle( const std::string& titleText, const std::string& fontName, const std::string& fontFilename, const float& textSize )
{
    m_keyTitleList.push_back( KeyTitle( titleText, fontName, fontFilename, textSize ) );
}

int FieldGridGen::GetNumColorRanges()
{
    return m_colorIndexList.size();
}

bool FieldGridGen::BuildCairoLegend()
{
    unsigned int width  = 0;
    unsigned int height = 0;

    cairo_surface_t* legend = CreateLegend( width, height );
    cairo_surface_t* png    = cairo_image_surface_create( CAIRO_FORMAT_ARGB32, width, height );
    cairo_t*         cr     = cairo_create( png );

    cairo_set_source_surface( cr, legend, 0.0, 0.0 );
    cairo_paint( cr );

    cairo_surface_write_to_png( png, m_pngLegendFilename.c_str() );

    cairo_destroy( cr );
    cairo_surface_destroy( legend );
    cairo_surface_destroy( png );

    return true;
}

const double PADDING     = 5.0;
//const char*  HEADER_FONT = "Sans";
//const double HEADER_SIZE = 30.0;
const double BOX_WIDTH   = 75.0;

void FieldGridGen::AddTextRow( cairo_t *cr, const char *text,    unsigned int& width, unsigned int& height, double xOffset, double yOffset )
{
    cairo_text_extents_t extents;

    cairo_text_extents( cr, text, &extents );

    unsigned int w = static_cast<unsigned int>( extents.width + ( PADDING * 2.0 ) );

    if(w > width) width = w;

    unsigned int h = static_cast<unsigned int>( extents.height + ( PADDING * 2.0 ) );

    cairo_save( cr );

    cairo_translate( cr,
        (PADDING + xOffset) - extents.x_bearing,
        (PADDING + yOffset) - extents.y_bearing    );
    
    cairo_text_path( cr, text );
    cairo_fill( cr );
    cairo_restore( cr );

    cairo_translate( cr, 0.0, h );

    height += h;
}

void FieldGridGen::AddLegendTextRow( cairo_t *cr, const char *text, unsigned int& width, unsigned int& height, double *color )
{
    cairo_matrix_t fontMatrix;

    cairo_get_font_matrix( cr, &fontMatrix );

    double boxSize = round( fontMatrix.xx * 0.8 );

    // std::cout << fontMatrix.xx << std::endl;

    cairo_save( cr );
    cairo_translate( cr, PADDING, PADDING );
    cairo_rectangle( cr, 0.0, 0.0, BOX_WIDTH, boxSize );
    cairo_save(cr);
    cairo_set_source_rgba( cr, color[0], color[1], color[2], color[3] );
    cairo_fill_preserve( cr );
    cairo_restore( cr );
    cairo_stroke( cr );
    cairo_restore( cr );

    AddTextRow(cr, text, width, height, BOX_WIDTH + (PADDING * 2.0));
}

cairo_surface_t *FieldGridGen::CreateLegend( unsigned int& width, unsigned int& height )
{
    cairo_surface_t *surface = cairo_recording_surface_create( CAIRO_CONTENT_COLOR_ALPHA, 0 );
    cairo_t *cr = cairo_create( surface );

    bool firstTitle = true;
    for ( KeyTitleList::iterator it = m_keyTitleList.begin(); it != m_keyTitleList.end(); ++it )
    {
        if( firstTitle )
        {
            cairo_select_font_face( cr, it->m_fontName.c_str(), CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_BOLD );
        }
        else
        {
            cairo_select_font_face( cr, it->m_fontName.c_str(), CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL );
        }
        cairo_set_font_size( cr, it->m_textSize );

        AddTextRow(cr, it->m_titleText.c_str(), width, height);

        firstTitle = false;
    }

    for ( ColorIndexList::iterator it = m_colorIndexList.begin(); it != m_colorIndexList.end(); ++it )
    {
        cairo_set_font_size(cr, it->second.m_textSize );

        double color[4] = { it->second.m_rgb.r / 255.0, it->second.m_rgb.g / 255.0, it->second.m_rgb.b / 255.0, 1.0 };
        AddLegendTextRow( cr, it->second.m_labelText.c_str(), width, height, color );
    }

    cairo_destroy(cr);

    return surface;
}


KeyTitle::KeyTitle( const std::string& titleText, const std::string& fontName, const std::string& fontFilename, const float& textSize ) :
    m_titleText( titleText ),
    m_fontName( fontName ),
    m_fontFilename( fontFilename ),
    m_textSize( textSize )
{
}

ColorIndexRange::ColorIndexRange( const std::string& labelText, const std::string& fontName, const std::string& fontFilename, 
    const float& textSize, const struct ColorIndexRGB& rgb ) :
    m_labelText( labelText ),
    m_fontName( fontName ),
    m_fontFilename( fontFilename ),
    m_textSize( textSize ),
    m_rgb( rgb )
{
}

ColorIndexRange::ColorIndexRange( const ColorIndexRange& cir ) :
    m_labelText( cir.m_labelText ),
    m_fontName( cir.m_fontName ),
    m_fontFilename( cir.m_fontFilename ),
    m_textSize( cir.m_textSize ),
    m_rgb( cir.m_rgb )
{
}

ColorIndexRange::ColorIndexRange( )
{
    m_labelText = "";
    m_fontName = "";
    m_fontFilename = "";
    m_textSize = 0;
    m_rgb.r = m_rgb.g = m_rgb.b = 0;
}

}