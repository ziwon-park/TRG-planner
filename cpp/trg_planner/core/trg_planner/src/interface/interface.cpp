#include "trg_planner/include/interface/interface.h"

namespace trg {

TRGInterface::TRGInterface(const std::string& pipePath)
    : pipePath_(pipePath), running_(false) {
}

TRGInterface::~TRGInterface() {
    stopCommandListener();
    cleanup();
}

bool TRGInterface::init(std::function<OperationResponse(const OperationRequest&)> handler) {
    requestHandler_ = handler;
    return setupNamedPipe();
}

bool TRGInterface::setupNamedPipe() {
    unlink(pipePath_.c_str());
    
    if (mkfifo(pipePath_.c_str(), 0666) != 0) {
        std::cerr << "Error creating FIFO at " << pipePath_ << ": " << strerror(errno) << std::endl;
        return false;
    }
    
    std::cout << "Command pipe created at " << pipePath_ << std::endl;
    return true;
}

void TRGInterface::cleanup() {
    unlink(pipePath_.c_str());
}

bool TRGInterface::startCommandListener() {
    if (running_.load()) {
        std::cerr << "Command listener is already running" << std::endl;
        return false;
    }

    running_ = true;
    listenerThread_ = std::thread(&TRGInterface::commandListenerThread, this);
    return true;
}


void TRGInterface::stopCommandListener() {
    if (!running_.load()) {
        return;
    }
    
    running_ = false;
    
    if (listenerThread_.joinable()) {
        listenerThread_.join();
    }
}


void TRGInterface::commandListenerThread() {
    while (running_.load()) {
        int fd = open(pipePath_.c_str(), O_RDONLY | O_NONBLOCK);
        if (fd < 0) {
            std::cerr << "Failed to open FIFO for reading: " << strerror(errno) << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }
        
        int flags = fcntl(fd, F_GETFL);
        fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);
        
        FILE* fp = fdopen(fd, "r");
        if (!fp) {
            close(fd);
            std::cerr << "Failed to convert file descriptor to stream: " << strerror(errno) << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }
        
        char buffer[1024];
        std::string type, command, filepath;
        
        if (fgets(buffer, sizeof(buffer), fp)) {
            type = buffer;
            if (!type.empty() && type.back() == '\n') {
                type.pop_back();
            }
        }
        
        if (fgets(buffer, sizeof(buffer), fp)) {
            command = buffer;
            if (!command.empty() && command.back() == '\n') {
                command.pop_back();
            }
        }
        
        if (fgets(buffer, sizeof(buffer), fp)) {
            filepath = buffer;
            if (!filepath.empty() && filepath.back() == '\n') {
                filepath.pop_back();
            }
        }
        
        fclose(fp);
        
        if (!type.empty() && !command.empty()) {
            std::cout << "Received command: " << type << " " << command;
            if (!filepath.empty()) {
                std::cout << " (file: " << filepath << ")";
            }
            std::cout << std::endl;
            
            OperationRequest request;
            request.type = type;
            request.command = command;
            request.filepath = filepath;
            
            if (requestHandler_) {
                OperationResponse response = requestHandler_(request);
                
                std::cout << "Operation result: " << (response.success ? "Success" : "Failed")
                          << " - " << response.message << std::endl;
                
            } else {
                std::cerr << "No request handler registered" << std::endl;
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

OperationResponse TRGInterface::sendCommand(const OperationRequest& request, const std::string& pipePath) {
    OperationResponse response;
    response.success = false;
    
    struct stat st;
    if (stat(pipePath.c_str(), &st) != 0 || !S_ISFIFO(st.st_mode)) {
        response.message = "Command pipe not found. Is TRG Planner running?";
        std::cerr << response.message << std::endl;
        return response;
    }
    
    std::ofstream fifo(pipePath);
    if (!fifo) {
        response.message = "Failed to open command pipe. Is TRG Planner running?";
        std::cerr << response.message << std::endl;
        return response;
    }
    
    fifo << request.type << std::endl;
    fifo << request.command << std::endl;
    fifo << request.filepath << std::endl;
    fifo.close();
    
    response.success = true;
    response.message = "Command sent: " + request.type + " " + request.command;
    if (!request.filepath.empty()) {
        response.message += " (file: " + request.filepath + ")";
    }
    
    std::cout << response.message << std::endl;
    
    return response;
}

OperationRequest TRGInterface::createRequestFromArgs(int argc, char** argv) {
    OperationRequest request;
    
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "-t" || arg == "--type") {
            if (i + 1 < argc) {
                request.type = argv[++i];
            }
        } else if (arg == "-c" || arg == "--command") {
            if (i + 1 < argc) {
                request.command = argv[++i];
            }
        } else if (arg == "-f" || arg == "--filepath") {
            if (i + 1 < argc) {
                request.filepath = argv[++i];
            }
        } else if (arg == "-h" || arg == "--help") {
            printUsage();
            exit(0);
        } else if (request.type.empty()) {
            // Positional arguments
            request.type = arg;
        } else if (request.command.empty()) {
            request.command = arg;
        } else if (request.filepath.empty()) {
            request.filepath = arg;
        }
    }
    
    return request;
}

void TRGInterface::printUsage() {
    std::cout << "Usage: trg_cli [options] or trg_cli <type> <command> [filepath]" << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  -t, --type TYPE        Operation type (graph, path)" << std::endl;
    std::cout << "  -c, --command CMD      Command to execute" << std::endl;
    std::cout << "  -f, --filepath PATH    Filepath for load/save operations" << std::endl;
    std::cout << "  -h, --help             Show this help message" << std::endl;
    std::cout << std::endl;
    std::cout << "Available commands:" << std::endl;
    std::cout << "  For graph operations:" << std::endl;
    std::cout << "    - expand : Expand the graph" << std::endl;
    std::cout << "    - load   : Load a graph from file" << std::endl;
    std::cout << "    - reset  : Reset the graph" << std::endl;
    std::cout << "    - save   : Save the graph to file" << std::endl;
    std::cout << std::endl;
    std::cout << "  For path operations:" << std::endl;
    std::cout << "    - plan   : Plan a new path" << std::endl;
    std::cout << "    - load   : Load a path from file" << std::endl;
    std::cout << "    - refine : Refine existing path" << std::endl;
    std::cout << "    - reset  : Reset the path" << std::endl;
    std::cout << "    - save   : Save the path to file" << std::endl;
}

} // namespace trg