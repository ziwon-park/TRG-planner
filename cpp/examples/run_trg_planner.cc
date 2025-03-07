
#include "trg_planner/include/planner/trg_planner.h"
#include "trg_planner/include/utils/common.h"

int main(int argc, char **argv) {
  signal(SIGINT, signal_handler);  // to exit program when ctrl+c

  std::string map_config_path = std::string(TRG_DIR) + "/configs/map/default.yaml";
  TRGPlanner  trg_planner;

  if (argc > 1) {
    map_config_path = std::string(TRG_DIR) + "/configs/map/" + argv[1] + ".yaml";
  } else {
    print_error("Please provide a map config file");
    exit(1);
  }

  trg_planner.setParams(map_config_path);
  trg_planner.init();

  while (is_running.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    //// TODO: Add your visualization code here
  }
  return 0;
}
