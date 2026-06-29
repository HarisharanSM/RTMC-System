#pragma once
#include "iPCANController.h"
#include <memory>

class iSystemController {
public:
    virtual ~iSystemController() = default;
    virtual bool InitializeSystem() = 0;
    virtual void ShutdownSystem() = 0;
    virtual void ProcessUiCommand(int axisId, double velocity) = 0;
    virtual std::shared_ptr<iPCANController> GetCanController() const = 0;
};