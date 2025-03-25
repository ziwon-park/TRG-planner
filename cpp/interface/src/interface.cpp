#include "trg_planner/include/utils/common.h"
#include "trg_planner/include/interface/operation.h"
#include "trg_planner/include/interface/interface.h"

int main(int argc, char** argv) {
  if (argc < 3) {
    trg::TRGInterface::printUsage();
    return 1;
  }
  
  OperationRequest request = trg::TRGInterface::createRequestFromArgs(argc, argv);
  
  if (request.type.empty()) {
    std::cerr << "Error: Operation type is required" << std::endl;
    trg::TRGInterface::printUsage();
    return 1;
  }
  
  if (request.command.empty()) {
    std::cerr << "Error: Command is required" << std::endl;
    trg::TRGInterface::printUsage();
    return 1;
  }
  
  OperationResponse response = trg::TRGInterface::sendCommand(request);
  
  if (response.success) {
    std::cout << "Command successful: " << response.message << std::endl;
    return 0;
  } else {
    std::cerr << "Command failed: " << response.message << std::endl;
    return 1;
  }
}