#ifndef OPERATION_H_
#define OPERATION_H_

#include "trg_planner/include/utils/common.h"

struct OperationRequest {
    std::string type;      // "graph" or "path"
    std::string command;   // commands :
                           // - graph operations :
                           //   - "expand" : expand the graph
                           //   - "load" : load the graph
                           //   - "reset" : reset the graph
                           //   - "save" : save the graph
                           // - path operations :
                           //   - "plan" : plan the path
                           //   - "load" : load the path
                           //   - "refine" : refine the path
                           //   - "reset" : reset the path
                           //   - "save" : save the path
    std::string filepath;  // filepath
};

struct OperationResponse {
    bool success;
    std::string message;
};

#endif  // OPERATION_H_
