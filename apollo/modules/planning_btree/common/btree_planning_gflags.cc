#include "modules/planning_btree/common/btree_planning_gflags.h"

#include <limits>

DEFINE_string(btree_context_config_file,
             "/apollo/modules/planning_btree/conf/btree_context_config.pb.txt",
             "behaviour tree context config file");

DEFINE_string(btree_stage_config_file,
             "/apollo/modules/planning_btree/conf/btree_stage_config.pb.txt",
             "behaviour tree stage config file");