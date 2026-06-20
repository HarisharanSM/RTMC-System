#pragma once

class iSystemController {
public:
    virtual ~iSystemController() = default;
    virtual bool InitializeSystem() = 0;
    virtual void ShutdownSystem() = 0;
    virtual void ProcessUiCommand(int axisId, double velocity) = 0;
};