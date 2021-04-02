#!/bin/bash

# Run this script inside docker container

MAP_DIR=$(pwd)
DIR=$(pwd)
EXPORT_EXEC=./export
CONVERT_EXEC=./geojson2proto
LAYERS="Connection.geojson Crosswalk.geojson Junction.geojson Line.geojson Reference_line.geojson Signal.geojson Rule.geojson Sign.geojson"
REPORT="report.log"

cd $MAP_DIR
$CONVERT_EXEC -fix $LAYERS && \
$EXPORT_EXEC $LAYERS > export.geojson && \
cat $MAP_DIR/export.geojson | $CONVERT_EXEC > $MAP_DIR/base_map.txt && \
cd /apollo && \
/apollo/scripts/generate_routing_topo_graph.sh --map_dir $MAP_DIR 2>$REPORT && \
/apollo/bazel-bin/modules/map/tools/sim_map_generator --map-dir=$MAP_DIR --output-dir=$MAP_DIR 2>>$REPORT
rm $MAP_DIR/sim_map.bin # muddle different lanes -- use .txt version only!
cd $DIR
