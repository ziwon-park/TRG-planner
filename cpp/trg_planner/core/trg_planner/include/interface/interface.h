#ifndef TRG_INTERFACE_H
#define TRG_INTERFACE_H

#include "trg_planner/include/interface/operation.h"

namespace trg {

/**
 * @brief The TRGInterface class provides a command-line interface for controlling TRGPlanner
 * through Named Pipes
 */

class TRGInterface {
public:
    explicit TRGInterface(const std::string& pipePath = "/tmp/trg_planner_fifo");
    ~TRGInterface();
    
    bool init(std::function<OperationResponse(const OperationRequest&)> handler);
    bool startCommandListener();
    void stopCommandListener();

    static OperationResponse sendCommand(const OperationRequest& request, const std::string& pipePath = "/tmp/trg_planner_fifo");
    static OperationRequest createRequestFromArgs(int argc, char** argv);
    
    static void printUsage();

private:
    void commandListenerThread();
    bool setupNamedPipe();
    void cleanup();

private:
    std::string pipePath_;
    std::atomic<bool> running_{false};
    std::thread listenerThread_;
    std::function<OperationResponse(const OperationRequest&)> requestHandler_;
};

} // namespace trg

#endif // TRG_INTERFACE_H