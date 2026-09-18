#include "cCollisionMath.h"
#include "cCollisionSupervisor.h"
#include "cCANDriveHandler.h"
#include "cDriveCalculator.h"
#include "cDriveController.h"
#include "cProximityBackend.h"
#include "cSceneRegistry.h"
#include "cTrajectoryPredictor.h"
#include "iPCANController.h"

#include <chrono>
#include <atomic>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

using namespace RTMCCollision;

namespace {

int failures = 0;

void Check(bool condition, const std::string& message) {
    std::cout << (condition ? "[PASS] " : "[FAIL] ") << message << "\n";
    if (!condition) ++failures;
}

joystickSignal Direction(double x, double y, double lao, double cran, double a3 = 0.0) {
    return {x, y, lao, cran, a3};
}

CollisionRequest Request(const cSceneRegistry& scene, const joystickSignal& direction,
                         std::uint64_t session = 1, std::uint64_t sequence = 1) {
    cDriveCalculator calculator;
    CollisionRequest request{};
    request.session = session;
    request.sequence = sequence;
    request.sceneGeneration = scene.Generation();
    request.currentPosition = {0, 0, 0, 0};
    calculator.CalculateInverseKinematics(request.currentPosition, request.currentAxles);
    request.direction = direction;
    request.linearSpeedMps = 0.20;
    request.angularSpeedRadps = 1.0471975511965976;
    return request;
}

cSceneRegistry ProbeScene(const Vec3& obstacleCenter, const Vec3& obstacleSize) {
    cSceneRegistry scene = cSceneRegistry::CreateEmptyScene();
    scene.AddBody({"carm_probe", "carm", eBodyFrame::CArm,
                   {0, 0, 0}, {0.10, 0.10, 0.10}, 0.0, false});
    scene.AddStaticObstacle("obstacle", obstacleCenter, obstacleSize);
    return scene;
}

class cFakePcanController : public iPCANController {
public:
    bool Start() override { running = true; return true; }
    void Stop() override { running = false; }
    bool IsRunning() const override { return running; }
    void SubscribeMessage(DWORD, MessageCallback) override {}
    bool SendMessage(DWORD, TPCANMessageType, BYTE, const BYTE*) override { return true; }
    void SetSpeed(float value) override { speed.store(value); speedWrites.fetch_add(1); }
    void SetPosition(const AxelPostion& value) override { position = value; ++positionWrites; }

    bool running = false;
    std::atomic<float> speed{-1.0f};
    std::atomic<int> speedWrites{0};
    int positionWrites = 0;
    AxelPostion position{};
};

bool WaitForPermit(cCollisionSupervisor& supervisor, std::uint64_t session,
                   std::uint64_t sequence, CollisionPermit& permit) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(300);
    while (std::chrono::steady_clock::now() < deadline) {
        if (supervisor.TryConsume(session, sequence, permit)) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
}

void TestProximity() {
    std::cout << "\nAVOID-OBB - oriented box separation\n";
    cProximityBackend proximity;
    CollisionBody bodyA{"a", "a", eBodyFrame::World, {}, {1, 1, 1}, 0, true};
    CollisionBody bodyB{"b", "b", eBodyFrame::World, {}, {1, 1, 1}, 0, true};
    OrientedBox a{{0, 0, 0}, {.5, .5, .5}, RotationZ(.4), &bodyA};
    OrientedBox b{{2, 0, 0}, {.5, .5, .5}, RotationZ(-.2), &bodyB};
    const auto separated = proximity.Separation(a, b);
    Check(!separated.overlapping && separated.largestSeparatingGapM > 0.0,
          "rotated separated boxes have a positive SAT gap");
    b.center = {.2, 0, 0};
    Check(proximity.Separation(a, b).overlapping, "overlapping rotated boxes are detected");

    OrientedBox c{{0, 0, 0}, {.05, .05, .05}, Mat3{}, &bodyA};
    OrientedBox d{{.111, 0, 0}, {.05, .05, .05}, Mat3{}, &bodyB};
    Check(std::abs(proximity.SurfaceDistance(c, d) - .011) < 1e-12,
          "3D closest-feature distance reports an 11 mm surface gap");
}

void TestStartDirectionProtocol() {
    std::cout << "\nAVOID-INPUT - start-direction preservation\n";
    cCANDriveHandler decoder;
    const joystickSignal expected[] = {
        Direction(1, 0, 0, 0), Direction(-1, 0, 0, 0),
        Direction(0, 1, 0, 0), Direction(0, -1, 0, 0),
        Direction(0, 0, 0, 1), Direction(0, 0, 0, -1),
        Direction(0, 0, 1, 0), Direction(0, 0, -1, 0),
        Direction(0, 0, 0, 0, -1), Direction(0, 0, 0, 0, 1)
    };
    bool allDirections = true;
    for (int bit = 0; bit < 8; ++bit) {
        TPCANMsg frame{};
        frame.LEN = 1;
        frame.DATA[0] = static_cast<BYTE>(1u << bit);
        const joystickSignal actual = decoder.ConvertToJoystickSignal(frame);
        allDirections = allDirections && actual.x == expected[bit].x &&
            actual.y == expected[bit].y && actual.LAO == expected[bit].LAO &&
            actual.CRAN == expected[bit].CRAN;
    }
    Check(allDirections, "all eight CAN bit directions decode to their actual requested axis/sign");
    bool versionedDirections = true;
    for (int index = 0; index < 10; ++index) {
        TPCANMsg frame{};
        frame.LEN = 8;
        frame.DATA[0] = RTMC_CAN_PROTOCOL_VERSION;
        frame.DATA[1] = static_cast<BYTE>(index + 1);
        const joystickSignal actual = decoder.ConvertToJoystickSignal(frame);
        versionedDirections = versionedDirections && actual.x == expected[index].x &&
            actual.y == expected[index].y && actual.LAO == expected[index].LAO &&
            actual.CRAN == expected[index].CRAN && actual.A3 == expected[index].A3;
    }
    Check(versionedDirections, "revision-4 CAN direction enum preserves all ten joystick directions");
    TPCANMsg incompatible{};
    incompatible.LEN = 8;
    incompatible.DATA[0] = RTMC_CAN_PROTOCOL_VERSION - 1;
    incompatible.DATA[1] = 1;
    const auto rejected = decoder.ConvertToJoystickSignal(incompatible);
    Check(rejected.x == 0 && rejected.y == 0 && rejected.LAO == 0 && rejected.CRAN == 0,
          "incompatible versioned UI frames cannot create motion");
}

void TestPrediction() {
    std::cout << "\nAVOID-PREDICT - stopping-path prediction\n";
    const auto clearScene = ProbeScene({1.0, 0, 1.2}, {.05, .5, .5});
    cTrajectoryPredictor clearPredictor(clearScene);
    const CollisionPermit clear = clearPredictor.Predict(Request(clearScene, Direction(1, 0, 0, 0)));
    Check(clear.verdict == eCollisionVerdict::Clear,
          "far obstacle is clear through reaction and braking path");
    Check(std::abs(clear.predictedTravelM - .09) < 1e-10,
          "speed-capped prediction includes 90 mm of worst-case linear travel");
    CollisionRequest measured = Request(clearScene, Direction(1, 0, 0, 0));
    measured.velocityMeasured = true;
    measured.linearSpeedMps = 0.0;
    const CollisionPermit measuredPermit = clearPredictor.Predict(measured);
    Check(std::abs(measuredPermit.predictedTravelM - .0225) < 1e-10,
          "bounded measured speed is used when feedback is explicitly marked measured");

    PredictionSettings staticSettings{};
    staticSettings.reactionTimeS = 0.0;
    staticSettings.maximumLinearSpeedMps = 0.0;
    staticSettings.maximumLinearAccelerationMps2 = 0.0;
    staticSettings.maximumAngularSpeedRadps = 0.0;
    staticSettings.maximumAngularAccelerationRadps2 = 0.0;
    bool gapPolicy = true;
    for (const double gap : {.009, .010, .011}) {
        const auto scene = ProbeScene({.10 + gap, 0, 1.2}, {.10, .10, .10});
        cTrajectoryPredictor predictor(scene, staticSettings);
        CollisionRequest request = Request(scene, Direction(1, 0, 0, 0));
        request.linearSpeedMps = request.angularSpeedRadps = 0.0;
        const bool clear = predictor.Predict(request).verdict == eCollisionVerdict::Clear;
        gapPolicy &= clear == (gap > .010);
    }
    Check(gapPolicy, "10 mm residual gap denies 9 mm/equality and accepts 11 mm");

    const auto hazardScene = ProbeScene({.145, 0, 1.2}, {.02, .40, .40});
    cTrajectoryPredictor hazardPredictor(hazardScene);
    const CollisionPermit hazard = hazardPredictor.Predict(Request(hazardScene, Direction(1, 0, 0, 0)));
    Check(hazard.verdict != eCollisionVerdict::Clear,
          "obstacle clear at start but inside the stopping path is not authorized");
    Check(hazard.movingBody[0] != '\0' && std::string(hazard.obstacle) == "obstacle",
          "denial identifies the moving body and obstacle");

    auto invalid = Request(clearScene, Direction(1, 1, 0, 0));
    Check(clearPredictor.Predict(invalid).verdict == eCollisionVerdict::Unknown,
          "multi-axis direction outside the version-one contract is unknown");
    invalid = Request(clearScene, Direction(1, 0, 0, 0));
    invalid.currentPosition.X = std::nan("");
    Check(clearPredictor.Predict(invalid).verdict == eCollisionVerdict::Unknown,
          "non-finite motion state cannot produce a clear result");

    PredictionSettings invalidSettings{};
    invalidSettings.permitLifetimeS = 0.0;
    cTrajectoryPredictor invalidPredictor(clearScene, invalidSettings);
    Check(invalidPredictor.Predict(Request(clearScene, Direction(1, 0, 0, 0))).verdict ==
              eCollisionVerdict::Unknown,
          "invalid prediction settings cannot produce a clear result");

    CollisionRequest homeA3 = Request(clearScene, Direction(0, 0, 0, 0, 1));
    Check(clearPredictor.Predict(homeA3).verdict == eCollisionVerdict::Unknown,
          "A3 preflight at exact home is denied because the complete stop leaves the workspace");
    CollisionRequest interiorA3 = Request(clearScene, Direction(0, 0, 0, 0, 1));
    interiorA3.currentPosition.X = 50.0;
    cDriveCalculator calculator;
    calculator.CalculateInverseKinematics(interiorA3.currentPosition, interiorA3.currentAxles);
    Check(clearPredictor.Predict(interiorA3).verdict == eCollisionVerdict::Clear,
          "A3 preflight uses the independent carrier arc and clears it in open workspace");

    // Regression for a CRAN -> Stop -> CAUD reversal.  The old predictor
    // always projected the full 60 deg/s stop (30 degrees from rest), which
    // looked through the neutral pose into a remote table/source near-pass and
    // denied the safe return.  A modeled standstill may instead receive a
    // lower certified speed cap whose complete reaction-and-stop path is clear.
    const auto referenceScene = cSceneRegistry::CreateReferenceScene(true);
    cTrajectoryPredictor referencePredictor(referenceScene);
    CollisionRequest caudReturn = Request(referenceScene, Direction(0, 0, 0, -1));
    caudReturn.currentPosition.CRAN = 5.7;
    calculator.CalculateInverseKinematics(caudReturn.currentPosition,
                                          caudReturn.currentAxles);
    caudReturn.angularSpeedRadps = 0.0;
    caudReturn.velocityModeled = true;
    const CollisionPermit caudPermit = referencePredictor.Predict(caudReturn);
    std::cout << "[INFO] adaptive CAUD verdict=" << ToString(caudPermit.verdict)
              << " cap_radps=" << caudPermit.permittedAngularSpeedRadps
              << " travel_rad=" << caudPermit.predictedTravelRad
              << " pair=" << caudPermit.movingBody << "/" << caudPermit.obstacle
              << " reason=" << caudPermit.reason << "\n";
    Check(caudPermit.verdict == eCollisionVerdict::Clear &&
          caudPermit.permittedAngularSpeedRadps > 0.0,
          "modeled standstill receives a permit for the safe CAUD return");
    Check(caudPermit.predictedTravelRad < 10.0 * 3.14159265358979323846 / 180.0,
          "adaptive permit certifies the complete bounded stop without projecting past neutral");

    CollisionRequest constrainedCran = Request(referenceScene, Direction(0, 0, 0, -1));
    constrainedCran.currentPosition.CRAN = -20.0;
    calculator.CalculateInverseKinematics(constrainedCran.currentPosition,
                                          constrainedCran.currentAxles);
    constrainedCran.angularSpeedRadps = 0.0;
    constrainedCran.velocityModeled = true;
    const CollisionPermit constrainedPermit = referencePredictor.Predict(constrainedCran);
    Check(constrainedPermit.verdict == eCollisionVerdict::Clear &&
          constrainedPermit.permittedAngularSpeedRadps > 0.0 &&
          constrainedPermit.permittedAngularSpeedRadps < 60.0 * 3.14159265358979323846 / 180.0 &&
          std::string(constrainedPermit.reason) == "clear with collision-limited angular speed",
          "near geometry receives an enforceable reduced-speed permit instead of a false stop");

    cDriveCalculator cappedCalculator;
    drivePosition cappedPosition{0, 0, 0, -20.0, 0};
    AxelPostion cappedAxles{};
    cappedCalculator.CalculateInverseKinematics(cappedPosition, cappedAxles);
    double largestCappedStep = 0.0;
    for (int tick = 0; tick < 20; ++tick) {
        drivePosition next{};
        AxelPostion nextAxles{};
        cappedCalculator.CalculateNextPosition(cappedPosition, cappedAxles,
            Direction(0, 0, 0, -1), next, nextAxles, 50.0, 15.0);
        largestCappedStep = std::max(largestCappedStep,
                                     std::abs(nextAxles.A5 - cappedAxles.A5));
        cappedPosition = next;
        cappedAxles = nextAxles;
    }
    Check(cappedCalculator.GetProfileSpeedDps() <= 15.0 + 1e-12 &&
          largestCappedStep <= 0.75 + 1e-9,
          "drive profile cannot exceed the angular speed granted by collision");
    drivePosition slowedPosition{};
    AxelPostion slowedAxles{};
    cappedCalculator.CalculateNextPosition(cappedPosition, cappedAxles,
        Direction(0, 0, 0, -1), slowedPosition, slowedAxles, 50.0, 5.0);
    Check(cappedCalculator.GetProfileSpeedDps() <= 9.0 + 1e-12 &&
          std::abs(slowedAxles.A5 - cappedAxles.A5) <= 0.45 + 1e-9,
          "a lower renewed permit is enforced through the bounded deceleration transition");
}

void TestReferenceScene() {
    std::cout << "\nAVOID-SCENE - supplied static reference model\n";
    const auto scene = cSceneRegistry::CreateReferenceScene(true);
    cTrajectoryPredictor predictor(scene);
    const auto started = std::chrono::steady_clock::now();
    const CollisionPermit permit = predictor.Predict(Request(scene, Direction(1, 0, 0, 0)));
    const auto elapsed = std::chrono::steady_clock::now() - started;
    std::cout << "[INFO] reference home +X verdict=" << ToString(permit.verdict)
              << " pair=" << permit.movingBody << "/" << permit.obstacle
              << " reason=" << permit.reason << " elapsed_us="
              << std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() << "\n";
    Check(scene.IsValid(), "compiled reference scene has valid IDs, dimensions and frames");
    Check(permit.verdict == eCollisionVerdict::Clear,
          "closed-pose +X preflight is clear in the supplied simulation scene");
    Check(elapsed < std::chrono::milliseconds(50),
          "reference-scene prediction completes inside the 50 ms simulation budget");
}

void TestAsynchronousSupervisor() {
    std::cout << "\nAVOID-ASYNC - worker, mailbox and sticky revocation\n";
    const auto scene = ProbeScene({.145, 0, 1.2}, {.02, .40, .40});
    cCollisionSupervisor supervisor(scene);
    Check(supervisor.Start(), "collision worker starts");
    supervisor.BeginSession(7);
    const CollisionRequest request = Request(scene, Direction(1, 0, 0, 0), 7, 1);
    const auto before = std::chrono::steady_clock::now();
    Check(supervisor.Submit(request), "drive publishes a request without waiting for geometry work");
    const auto submitTime = std::chrono::steady_clock::now() - before;
    Check(submitTime < std::chrono::milliseconds(5), "request publication is bounded and non-blocking");
    CollisionPermit permit{};
    Check(WaitForPermit(supervisor, 7, 1, permit), "worker publishes the matching result");
    Check(permit.verdict != eCollisionVerdict::Clear && supervisor.IsStopRequested(7),
          "hazard or uncertainty sets a sticky stop request");
    supervisor.AcknowledgeControllerStop(7);
    Check(!supervisor.IsStopRequested(7), "matching controller Stop clears the stopped session");
    supervisor.Stop();
}

void TestDriveIntegration() {
    std::cout << "\nAVOID-DRIVE - preflight and stop latch\n";
    auto fake = std::make_shared<cFakePcanController>();
    fake->Start();
    auto scene = ProbeScene({1.0, 0, 1.2}, {.05, .5, .5});
    auto supervisor = std::make_unique<cCollisionSupervisor>(scene);
    cDriveController drive(fake, std::move(supervisor));

    const joystickSignal plusX = Direction(1, 0, 0, 0);
    drive.StartDrive(plusX);
    Check(fake->positionWrites == 0 && drive.GetLifecycleState() == cDriveController::eLifecycleState::Preflight,
          "Start keeps the actuator stationary while preflight runs");

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(300);
    while (fake->positionWrites == 0 && std::chrono::steady_clock::now() < deadline) {
        drive.HandleJoystick(plusX);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    Check(fake->positionWrites == 1 && drive.GetCurrentPosition().X > 0.0,
          "a matching clear permit allows one movement segment");

    const auto renewalDeadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
    while (fake->positionWrites < 8 && !drive.IsAvoidanceLatched() &&
           std::chrono::steady_clock::now() < renewalDeadline) {
        drive.HandleJoystick(plusX);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    Check(fake->positionWrites >= 8 && !drive.IsAvoidanceLatched(),
          "parallel worker renews permits while movement continues");

    drive.HandleJoystick(Direction(-1, 0, 0, 0));
    Check(drive.IsAvoidanceLatched() && std::abs(fake->speed.load()) < 1e-9,
          "direction change outside the permit invokes a protective stop");
    const int writesAtLatch = fake->positionWrites;
    drive.StartDrive(plusX);
    drive.HandleJoystick(plusX);
    Check(fake->positionWrites == writesAtLatch,
          "held commands and duplicate Start cannot restart a latched drive");
    drive.StopDrive(Direction(0, 0, 0, 0));
    Check(drive.GetLifecycleState() == cDriveController::eLifecycleState::Disarmed,
          "controller Stop acknowledges the session and returns to disarmed");

    auto hazardFake = std::make_shared<cFakePcanController>();
    hazardFake->Start();
    const auto hazardScene = ProbeScene({.145, 0, 1.2}, {.02, .40, .40});
    auto hazardSupervisor = std::make_unique<cCollisionSupervisor>(hazardScene);
    cDriveController blocked(hazardFake, std::move(hazardSupervisor));
    blocked.StartDrive(plusX);
    const auto hazardDeadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(300);
    while (!blocked.IsAvoidanceLatched() && std::chrono::steady_clock::now() < hazardDeadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    Check(blocked.IsAvoidanceLatched() && hazardFake->positionWrites == 0 &&
          hazardFake->speedWrites.load() >= 2 && std::abs(hazardFake->speed.load()) < 1e-9,
          "parallel monitor stops and latches a predicted collision without another drive callback");
    blocked.StartDrive(plusX);
    Check(hazardFake->positionWrites == 0,
          "predicted hazard cannot auto-restart when Start remains asserted");
    blocked.StopDrive(Direction(0, 0, 0, 0));
    Check(blocked.GetLifecycleState() == cDriveController::eLifecycleState::Disarmed,
          "controller Stop acknowledges a collision-triggered latch");
}

void TestReferenceDriveIntegration() {
    std::cout << "\nAVOID-REFERENCE-DRIVE - compiled model integration\n";
    auto fake = std::make_shared<cFakePcanController>();
    fake->Start();
    auto supervisor = std::make_unique<cCollisionSupervisor>(
        cSceneRegistry::CreateReferenceScene(true));
    cDriveController drive(fake, std::move(supervisor));
    const joystickSignal plusX = Direction(1, 0, 0, 0);
    drive.StartDrive(plusX);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
    while (fake->positionWrites < 3 && !drive.IsAvoidanceLatched() &&
           std::chrono::steady_clock::now() < deadline) {
        drive.HandleJoystick(plusX);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    Check(fake->positionWrites >= 3 && !drive.IsAvoidanceLatched(),
          "application reference scene grants and renews initial +X movement permits");
    drive.StopDrive(Direction(0, 0, 0, 0));
}

void TestFiveAxisHeadFrame() {
    cDriveCalculator calculator;
    cBodyKinematics kinematics;
    bool aligned = true, roundTrip = true;
    for (const drivePosition pose : {drivePosition{0,0,0,0}, {50,10,25,-15}, {100,-20,-30,20}}) {
        AxelPostion q{};
        aligned &= calculator.CalculateInverseKinematics(pose,q) == eKinematicStatus::Ok;
        aligned &= std::abs(q.A1+q.A2+q.A3) < 1e-10;
        roundTrip &= q.A4 == pose.LAO && q.A5 == pose.CRAN;
        const auto f=calculator.CalculateForwardKinematics(q);
        roundTrip &= std::abs(f.X-pose.X)<1e-9 && std::abs(f.Y-pose.Y)<1e-9 &&
                     f.LAO==pose.LAO && f.CRAN==pose.CRAN;
    }
    Check(aligned, "A3 compensates A1+A2 throughout patient-frame translation");
    Check(roundTrip, "five-axis IK/FK preserves X/Y and maps LAO=A4, CRAN=A5");
    const auto home=kinematics.CalculateFrames({-180,180,0,0,0});
    const auto tilted=kinematics.CalculateFrames({-180,180,0,25,-15});
    const auto h=home[static_cast<int>(eBodyFrame::CArm)];
    const auto t=tilted[static_cast<int>(eBodyFrame::CArm)];
    Check(Norm(h.translation-Vec3{0,0,1.2})<1e-12 && Norm(t.translation-h.translation)<1e-12,
          "LAO and CRAN rotate at the patient head without moving the home imaging center");
    CollisionBody centered{"probe","probe",eBodyFrame::CArm,{0,0,0},{.6,.4,.3},0,false};
    Check(kinematics.BoundBodyMotion(centered,{-180,180,0,0,0},{-180,180,0,0,30})>.1,
          "A5 sweep bounds include box corners even when center is stationary");

    const auto scene = cSceneRegistry::CreateReferenceScene(true);
    bool headSideSupport = false;
    for (const CollisionBody& body : scene.Bodies()) {
        if (body.id != "support_column") continue;
        const AxelPostion homeAxles{-180,180,0,0,0};
        const OrientedBox column = kinematics.CalculateBox(body, homeAxles);
        headSideSupport = column.center.x < -.70 && std::abs(column.center.y) < 1e-12;
    }
    Check(headSideSupport, "neutral C-arm support column is on the patient head side (-X)");

    cTrajectoryPredictor predictor(scene);
    bool centralRotationClear = true;
    for (const double x : {0.0, 10.0, 20.0, 30.0}) {
        for (const double sign : {-1.0, 1.0}) {
            CollisionRequest request = Request(scene, Direction(0,0,sign,0));
            request.currentPosition.X = x;
            calculator.CalculateInverseKinematics(request.currentPosition, request.currentAxles);
            const auto permit = predictor.Predict(request);
            if (permit.verdict != eCollisionVerdict::Clear) {
                std::cout << "[INFO] angular preflight X=" << x << " sign=" << sign
                          << " verdict=" << ToString(permit.verdict) << " pair="
                          << permit.movingBody << "/" << permit.obstacle
                          << " reason=" << permit.reason << "\n";
                centralRotationClear = false;
            }
        }
    }
    Check(centralRotationClear,
          "central workspace preflight permits both LAO and RAO at representative X positions");
}

void TestMissingRenewal() {
    auto fake=std::make_shared<cFakePcanController>(); fake->Start();
    auto scene=ProbeScene({1,0,1.2},{.05,.5,.5});
    cDriveController drive(fake,std::make_unique<cCollisionSupervisor>(scene));
    const auto direction=Direction(1,0,0,0);
    drive.StartDrive(direction);
    const auto until=std::chrono::steady_clock::now()+std::chrono::milliseconds(500);
    while(fake->positionWrites==0 && std::chrono::steady_clock::now()<until) {
        drive.HandleJoystick(direction); std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    Check(fake->positionWrites>0,"renewal watchdog fixture first obtains a movement permit");
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(500);
    while(!drive.IsAvoidanceLatched() && std::chrono::steady_clock::now()<deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    Check(drive.IsAvoidanceLatched() && fake->speed.load()==0,
          "missing callbacks expire permission and independently stop the running drive");
    drive.StopDrive({0,0,0,0});
}

} // namespace

int main() {
    std::cout << "RTMC predictive collision avoidance tests\n";
    TestProximity();
    TestStartDirectionProtocol();
    TestPrediction();
    TestReferenceScene();
    TestAsynchronousSupervisor();
    TestDriveIntegration();
    TestReferenceDriveIntegration();
    TestFiveAxisHeadFrame();
    TestMissingRenewal();
    std::cout << "\n" << (failures == 0 ? "all collision checks passed" :
                                           std::to_string(failures) + " collision checks failed") << "\n";
    return failures == 0 ? 0 : 1;
}
