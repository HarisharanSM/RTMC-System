#include "cCANMocker.h"
#include "cPCANController.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <map>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
namespace {
std::string ReadAsset(const std::string& path) {
    std::ifstream file(path); std::ostringstream contents; contents << file.rdbuf();
    return contents.str();
}
void Respond(int client, int status, const std::string& type, const std::string& body) {
    const std::string data = "HTTP/1.1 " + std::to_string(status) +
        (status == 200 ? " OK\r\n" : " Error\r\n") +
        "Content-Type: " + type + "\r\nCache-Control: no-store\r\nContent-Length: " +
        std::to_string(body.size()) + "\r\nConnection: close\r\n\r\n" + body;
    std::size_t offset=0;
    while(offset<data.size()) {
        const auto count=send(client,data.data()+offset,data.size()-offset,0);
        if(count<=0) break;
        offset+=static_cast<std::size_t>(count);
    }
}
}
cCANMocker::cCANMocker(std::shared_ptr<iPCANController> controller, std::string root)
    : m_PcanController(std::move(controller)), m_AssetRoot(std::move(root)) {}
cCANMocker::~cCANMocker() { Stop(); }
bool cCANMocker::Start() {
    if(m_IsRunning) return true;
    if(!m_PcanController) return false;
    m_ServerFd=socket(AF_INET,SOCK_STREAM,0);
    if(m_ServerFd<0) return false;
    int enabled=1;
    setsockopt(m_ServerFd,SOL_SOCKET,SO_REUSEADDR,&enabled,sizeof(enabled));
    sockaddr_in address{};
    address.sin_family=AF_INET;
    address.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
    address.sin_port=htons(8082);
    if(bind(m_ServerFd,reinterpret_cast<sockaddr*>(&address),sizeof(address))<0 ||
       listen(m_ServerFd,16)<0) {
        close(m_ServerFd); m_ServerFd=-1; return false;
    }
    m_IsRunning=true;
    try {m_WorkerThread=std::thread(&cCANMocker::MockingLoop,this);}
    catch(...) {m_IsRunning=false; close(m_ServerFd); m_ServerFd=-1; return false;}
    return true;
}
void cCANMocker::Stop() {
    if(!m_IsRunning.exchange(false)) return;
    shutdown(m_ServerFd,SHUT_RDWR);
    if(m_WorkerThread.joinable()) m_WorkerThread.join();
    close(m_ServerFd); m_ServerFd=-1;
}
void cCANMocker::MockingLoop() {
    const std::map<std::string,int> buttons{
        {"R-up",0},{"R-down",1},{"R-left",2},{"R-right",3},
        {"L-up",4},{"L-down",5},{"L-left",6},{"L-right",7}};
    auto controller=std::dynamic_pointer_cast<cPCANController>(m_PcanController);
    while(m_IsRunning) {
        const int client=accept(m_ServerFd,nullptr,nullptr);
        if(client<0) continue;
        timeval timeout{0,200000};
        setsockopt(client,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));
        setsockopt(client,SOL_SOCKET,SO_SNDTIMEO,&timeout,sizeof(timeout));
        char buffer[4096];
        const auto count=recv(client,buffer,sizeof(buffer),0);
        if(count<=0) {close(client); continue;}
        std::istringstream first(std::string(buffer,static_cast<std::size_t>(count)));
        std::string method,target; first>>method>>target;
        const auto query=target.find('?');
        const std::string path=target.substr(0,query);
        std::map<std::string,std::string> args;
        if(query!=std::string::npos) {
            std::istringstream fields(target.substr(query+1)); std::string field;
            while(std::getline(fields,field,'&')) {
                const auto equals=field.find('=');
                if(equals!=std::string::npos) args[field.substr(0,equals)]=field.substr(equals+1);
            }
        }
        if(path=="/state" && method=="GET" && controller) {
            Respond(client,200,"application/json",controller->TelemetryJson());
        } else if(path=="/command" && method=="POST" && controller) {
            TPCANMsg frame{}; frame.MSGTYPE=PCAN_MESSAGE_STANDARD; frame.LEN=1;
            const auto button=buttons.find(args["btn"]);
            bool valid=true;
            if(args["cmd"]=="stop") frame.ID=0x003;
            else if(button!=buttons.end() && (args["cmd"]=="start" || args["cmd"].empty())) {
                frame.ID=args["cmd"]=="start"?0x002:0x001;
                frame.DATA[0]=static_cast<BYTE>(1u<<button->second);
            } else valid=false;
            if(valid) controller->InjectReceivedMessage(frame);
            Respond(client,valid?200:400,"application/json",valid?"{\"accepted\":true}":"{\"accepted\":false}");
        } else if(method=="GET" && (path=="/" || path=="/index.html" || path=="/viewer")) {
            const auto body=ReadAsset(m_AssetRoot+(path=="/viewer"?
                "/data/collision/reference/generated/viewer.html":"/ui/index.html"));
            Respond(client,body.empty()?404:200,"text/html; charset=utf-8",body);
        } else Respond(client,404,"text/plain","Not found");
        close(client);
    }
}
