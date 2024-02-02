#include "osmParser.h"

MapOSM::MapOSM()
{
}

MapOSM::~MapOSM()
{
}

void MapOSM::parseOSM(std::string path_to_osm_file)
{

    osmium::io::File input_file{path_to_osm_file};

    // Configuration for the multipolygon assembler. Here the default settings
    // are used, but you could change multiple settings.
    osmium::area::Assembler::config_type assembler_config;

    // Set up a filter matching only buildings. This will be used to only build
    // areas with matching tags.
    osmium::TagsFilter filter{false};
    filter.add_rule(true, "building", "yes");

    // Initialize the MultipolygonManager. Its job is to collect all
    // relations and member ways needed for each area. It then calls an
    // instance of the osmium::area::Assembler class (with the given config)
    // to actually assemble one area. The filter parameter is optional, if
    // it is not set, all areas will be built.
    osmium::area::MultipolygonManager<osmium::area::Assembler> mp_manager{assembler_config, filter};

    // We read the input file twice. In the first pass, only relations are
    // read and fed into the multipolygon manager.
    std::cerr << "Pass 1...\n";
    osmium::relations::read_relations(input_file, mp_manager);
    std::cerr << "Pass 1 done\n";

    // Output the amount of main memory used so far. All multipolygon relations
    // are in memory now.
    std::cerr << "Memory:\n";
    osmium::relations::print_used_memory(std::cerr, mp_manager.used_memory());

    // The index storing all node locations.
    index_type index;

    // The handler that stores all node locations in the index and adds them
    // to the ways.
    location_handler_type location_handler{index};

    // If a location is not available in the index, we ignore it. It might
    // not be needed (if it is not part of a multipolygon relation), so why
    // create an error?
    location_handler.ignore_errors();

    // Create Ziyang's handler
    OSMHandler handler;
    
    // On the second pass we read all objects and run them first through the
    // node location handler and then the multipolygon collector. The collector
    // will put the areas it has created into the "buffer" which are then
    // fed through our "handler".
    std::cerr << "Pass 2...\n";
    osmium::io::Reader reader{input_file};
    osmium::apply(reader, location_handler, mp_manager.handler([&handler](osmium::memory::Buffer &&buffer) 
    {
        osmium::apply(buffer, handler);
    }));
    reader.close();
    std::cerr << "Pass 2 done\n";

    // Output the amount of main memory used so far. All complete multipolygon
    // relations have been cleaned up.
    std::cerr << "Memory:\n";
    osmium::relations::print_used_memory(std::cerr, mp_manager.used_memory());

    // KD-tree part
    KD_points = handler.KD_points;
    KDTree _tree(KD_points);
    tree = _tree;
}