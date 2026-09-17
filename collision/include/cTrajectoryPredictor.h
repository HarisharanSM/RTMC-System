#pragma once

#include "cBodyKinematics.h"
#include "cCollisionTypes.h"
#include "cProximityBackend.h"
#include "cSceneRegistry.h"

namespace RTMCCollision {

class cTrajectoryPredictor {
public:
    explicit cTrajectoryPredictor(cSceneRegistry scene,
                                  PredictionSettings settings = PredictionSettings{});

    CollisionPermit Predict(const CollisionRequest& request) const;
    const cSceneRegistry& Scene() const { return m_Scene; }

private:
    enum class eIntervalResult { Clear, Hazard, Unknown };

    bool IsFinite(const CollisionRequest& request) const;
    bool AreSettingsValid() const;
    bool IsDirectionValid(const joystickSignal& direction) const;
    drivePosition PoseAt(const CollisionRequest& request, double linearTravelM,
                         double angularTravelRad, double fraction) const;
    bool SolvePose(const drivePosition& position, AxelPostion& axles) const;
    bool AxlesAt(const CollisionRequest& request, double linearTravelM,
                 double angularTravelRad, double fraction, AxelPostion& axles) const;
    double FeasibleFraction(const CollisionRequest& request, double linearTravelM,
                            double angularTravelRad) const;
    eIntervalResult CheckInterval(const CollisionRequest& request, double linearTravelM,
                                  double angularTravelRad, double begin, double end,
                                  CollisionPermit& result) const;
    eIntervalResult CheckPairInterval(const CollisionRequest& request,
                                      double linearTravelM, double angularTravelRad,
                                      const CollisionBody& moving, const CollisionBody& other,
                                      double begin, double end, int depth,
                                      CollisionPermit& result) const;
    eIntervalResult CheckPair(const CollisionBody& moving, const CollisionBody& other,
                              const AxelPostion& begin, const AxelPostion& middle,
                              const AxelPostion& end, CollisionPermit& result) const;
    bool ShouldCheckPair(const CollisionBody& lhs, const CollisionBody& rhs) const;

    cSceneRegistry m_Scene;
    PredictionSettings m_Settings;
    cBodyKinematics m_Kinematics;
    cProximityBackend m_Proximity;
};

} // namespace RTMCCollision
