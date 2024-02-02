
#ifndef OSMPARSER_H
#define OSMPARSER_H


// Standard library
#include <string>
#include <vector>
#include <numeric>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <math.h>
#include <cstdlib>  // for std::exit
#include <locale>   // std::locale, std::tolower
// Opencv header
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/persistence.hpp"

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry> 

// KD tree
#include "KDTree.hpp"

/*----------------------------For osmium ----------------------------*/
// For assembling multipolygons
#include <osmium/area/assembler.hpp>
#include <osmium/area/multipolygon_manager.hpp>

// For the DynamicHandler class
#include <osmium/dynamic_handler.hpp>

// For the WKT factory
#include <osmium/geom/wkt.hpp>

// For the Dump handler
#include <osmium/handler/dump.hpp>

// For the NodeLocationForWays handler
#include <osmium/handler/node_locations_for_ways.hpp>

// Allow any format of input files (XML, PBF, ...)
#include <osmium/io/any_input.hpp>

// For osmium::apply()
#include <osmium/visitor.hpp>

// For the location index. There are different types of indexes available.
// This will work for all input files keeping the index in memory.
#include <osmium/index/map/flex_mem.hpp>
/*---------------------------- End of osmium ----------------------------*/


// The type of index used. This must match the include file above
using index_type = osmium::index::map::FlexMem<osmium::unsigned_object_id_type, osmium::Location>;

// The location handler always depends on the index type
using location_handler_type = osmium::handler::NodeLocationsForWays<index_type>;


class OSMHandler : public osmium::handler::Handler {

	void output_node(const osmium::Node& node){
        // do something
	}
	
	void output_way(const osmium::Way& way){
		const osmium::TagList& tags = way.tags();
		if (tags.has_key("building")){	
		
			// do something
		}	
	}

public:

    pointVec KD_points; 

 	// This callback is called by osmium::apply for each node in the data.
    void node(const osmium::Node& node) {
        output_pubs(node);
    }

  	// This callback is called by osmium::apply for each way in the data.
    void way(const osmium::Way& way) {
    	output_way(way);
    }
}; // class OSMHandler

class MapOSM
{

public:
    pointVec KD_points;  
    KDTree tree;
    MapOSM();
    ~MapOSM();
    
    // Parse osm file
    void parseOSM(std::string osm_file);

};

#endif