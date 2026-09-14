#include "SystemController/include/cControlManager.h"
#include "CANMocker/include/cCANMocker.h"
#include <iostream>
#include <thread>
#include <csignal>
#include <filesystem>
#ifndef RTMC_SOURCE_DIR
#define RTMC_SOURCE_DIR "."
#endif
namespace {
volatile std::sig_atomic_t stopping=0;
void OnSignal(int) {stopping=1;}
}
int main(int argc,char** argv) {
    std::signal(SIGINT,OnSignal); std::signal(SIGTERM,OnSignal); std::signal(SIGPIPE,SIG_IGN);
    const std::string root=argc==3 && std::string(argv[1])=="--assets"?argv[2]:RTMC_SOURCE_DIR;
    if(!std::filesystem::exists(root+"/ui/index.html") ||
       !std::filesystem::exists(root+"/data/collision/reference/parameters.json") ||
       !std::filesystem::exists(root+"/data/collision/reference/generated/scene.json") ||
       !std::filesystem::exists(root+"/data/collision/reference/generated/manifest.json")) {
        std::cerr<<"Missing dashboard assets. Use --assets /path/to/RTMC-System\n"; return 1;
    }
    cControlManager engine;
    if(!engine.InitializeSystem()) return 1;
    cCANMocker server(engine.GetCanController(),root);
    std::cout<<"RTMC live joystick + integrated 3D display: http://localhost:8082\n"
               "Predictive collision supervision enabled. Five-axis simulation.\n";
    if(!server.Start()) {std::cerr<<"Unable to listen on localhost:8082\n"; return 1;}
    while(!stopping) std::this_thread::sleep_for(std::chrono::milliseconds(50));
    server.Stop();
    return 0;
}
