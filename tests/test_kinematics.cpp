// Executable form of docs/kinematics-bdd.md.
// Scenario IDs printed here match the IDs in that document.

#include "cDriveCalculator.h"
#include "cDriveController.h"
#include "iPCANController.h"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace RTMCGeometry;

// ---------------------------------------------------------------------------
// Minimal BDD harness
// ---------------------------------------------------------------------------
namespace {

int g_scenarios = 0;
int g_scenariosFailed = 0;
int g_checks = 0;
int g_checksFailed = 0;
bool g_scenarioFailed = false;
std::vector<std::string> g_failures;
std::string g_scenarioId;

void Scenario(const std::string& id, const std::string& title) {
    g_scenarioId = id;
    g_scenarioFailed = false;
    ++g_scenarios;
    std::cout << "\n" << id << " - " << title << "\n";
}

void Given(const std::string& text) { std::cout << "    Given " << text << "\n"; }
void When(const std::string& text)  { std::cout << "    When  " << text << "\n"; }

void Check(const std::string& keyword, bool ok, const std::string& text) {
    ++g_checks;
    if (!ok) {
        ++g_checksFailed;
        g_scenarioFailed = true;
        g_failures.push_back(g_scenarioId + ": " + text);
    }
    std::cout << "    " << (ok ? "[PASS] " : "[FAIL] ") << keyword << "  " << text << "\n";
}

void Then(bool ok, const std::string& text)    { Check("Then ", ok, text); }
void AndThen(bool ok, const std::string& text) { Check("And  ", ok, text); }

void EndScenario() {
    if (g_scenarioFailed) ++g_scenariosFailed;
}

bool Near(double a, double b, double tol) { return std::abs(a - b) <= tol; }

std::string Num(double v, int precision = 6) {
    std::ostringstream os;
    os << std::fixed << std::setprecision(precision) << v;
    return os.str();
}

const double TICK_MS = TIME_DELTA_MS;
const double TICK_S = TIME_DELTA_MS / 1000.0;

joystickSignal Sig(double x, double y, double lao, double cran) {
    return joystickSignal{x, y, lao, cran};
}

drivePosition Pos(double x, double y, double lao = 0.0, double cran = 0.0) {
    return drivePosition{x, y, lao, cran};
}

bool AnyNaN(const AxelPostion& a) {
    return std::isnan(a.A1) || std::isnan(a.A2) || std::isnan(a.A3) || std::isnan(a.A4) || std::isnan(a.A5);
}

bool AnyNaN(const drivePosition& p) {
    return std::isnan(p.X) || std::isnan(p.Y) || std::isnan(p.LAO) || std::isnan(p.CRAN);
}

// A CAN controller that records what the drive asked the bus to do.
class cFakePcanController : public iPCANController {
public:
    bool Start() override { m_Running = true; return true; }
    void Stop() override { m_Running = false; }
    bool IsRunning() const override { return m_Running; }
    void SubscribeMessage(DWORD, MessageCallback) override {}
    bool SendMessage(DWORD, TPCANMessageType, BYTE, const BYTE*) override { return true; }
    void SetSpeed(float speed) override { m_LastSpeed = speed; ++m_SpeedWrites; }
    void SetPosition(const AxelPostion& position) override { m_LastPosition = position; ++m_PositionWrites; }

    float LastSpeed() const { return m_LastSpeed; }
    int PositionWrites() const { return m_PositionWrites; }
    AxelPostion LastPosition() const { return m_LastPosition; }

private:
    bool m_Running = false;
    float m_LastSpeed = -1.0f;
    int m_SpeedWrites = 0;
    int m_PositionWrites = 0;
    AxelPostion m_LastPosition{0, 0, 0, 0};
};

// Result of driving the calculator for a number of ticks.
struct RunTrace {
    drivePosition finalPos{0, 0, 0, 0};
    AxelPostion finalAxel{0, 0, 0, 0};
    eKinematicStatus finalStatus = eKinematicStatus::Ok;
    double maxJointStep = 0.0;          // largest per-tick axle move seen
    double maxJointBudgetBreach = 0.0;  // how far any tick exceeded its budget
    double maxCartesianStep = 0.0;      // largest per-tick tool move, cm
    bool allPosesValid = true;
    bool sawNaN = false;
    double maxFkDrift = 0.0;            // worst |FK(angles) - stored position|
    std::vector<drivePosition> poses;
};

RunTrace Drive(cDriveCalculator& calc, drivePosition start, const joystickSignal& signal, int ticks) {
    RunTrace trace;
    drivePosition current = start;
    AxelPostion currentAxel{};
    calc.CalculateInverseKinematics(current, currentAxel);

    for (int i = 0; i < ticks; ++i) {
        drivePosition next{};
        AxelPostion nextAxel{};
        const eKinematicStatus status =
            calc.CalculateNextPosition(current, signal, next, nextAxel, TICK_MS);

        const double budget = calc.GetProfileSpeedDps() * TICK_S;
        const double jointStep = std::max(std::max(std::abs(nextAxel.A1 - currentAxel.A1),
                                                   std::abs(nextAxel.A2 - currentAxel.A2)),
                                          std::max(std::abs(nextAxel.A3 - currentAxel.A3),
                                                   std::max(std::abs(nextAxel.A4 - currentAxel.A4), std::abs(nextAxel.A5 - currentAxel.A5))));
        trace.maxJointStep = std::max(trace.maxJointStep, jointStep);
        trace.maxJointBudgetBreach = std::max(trace.maxJointBudgetBreach, jointStep - budget);
        trace.maxCartesianStep = std::max(trace.maxCartesianStep,
                                          std::hypot(next.X - current.X, next.Y - current.Y));

        if (AnyNaN(next) || AnyNaN(nextAxel)) trace.sawNaN = true;

        // The anti-divergence guarantee: the pose the caller stores must be the
        // pose the angles being sent to the bus actually describe.
        const drivePosition fk = calc.CalculateForwardKinematics(nextAxel);
        trace.maxFkDrift = std::max(trace.maxFkDrift,
                                    std::max(std::abs(fk.X - next.X), std::abs(fk.Y - next.Y)));

        AxelPostion check{};
        if (calc.CalculateInverseKinematics(next, check) != eKinematicStatus::Ok) {
            trace.allPosesValid = false;
        }
        if (next.X < ENVELOPE_MIN_X_CM - GEOM_EPS || next.X > ENVELOPE_MAX_X_CM + GEOM_EPS ||
            next.Y < ENVELOPE_MIN_Y_CM - GEOM_EPS || next.Y > ENVELOPE_MAX_Y_CM + GEOM_EPS) {
            trace.allPosesValid = false;
        }

        trace.poses.push_back(next);
        current = next;
        currentAxel = nextAxel;
        trace.finalStatus = status;
        trace.finalPos = next;
        trace.finalAxel = nextAxel;
    }
    return trace;
}

} // namespace

// ---------------------------------------------------------------------------
// UC-1 - Home pose is the fully-closed configuration
// ---------------------------------------------------------------------------
void UC1_HomePose() {
    cDriveCalculator calc;

    Scenario("KIN-01", "Home maps to the closed pose");
    Given("the arm is commanded to world position (0, 0)");
    When("inverse kinematics is solved");
    AxelPostion home{};
    const eKinematicStatus status = calc.CalculateInverseKinematics(Pos(0, 0), home);
    Then(status == eKinematicStatus::Ok, "status is Ok (got " + std::string(ToString(status)) + ")");
    AndThen(Near(home.A2, 180.0, 1e-6), "A2 = 180 deg, fully closed (got " + Num(home.A2) + ")");
    AndThen(Near(home.A1, -180.0, 1e-6), "A1 = -180 deg (got " + Num(home.A1) + ")");
    EndScenario();

    Scenario("KIN-02", "Full extension is the far end of travel");
    Given("the arm is commanded to world position (150, 0)");
    When("inverse kinematics is solved");
    AxelPostion extended{};
    const eKinematicStatus extStatus = calc.CalculateInverseKinematics(Pos(150, 0), extended);
    Then(extStatus == eKinematicStatus::Ok, "status is Ok (got " + std::string(ToString(extStatus)) + ")");
    AndThen(Near(extended.A1, 0.0, 1e-6), "A1 = 0 deg (got " + Num(extended.A1) + ")");
    AndThen(Near(extended.A2, 0.0, 1e-6), "A2 = 0 deg (got " + Num(extended.A2) + ")");
    EndScenario();
}

// ---------------------------------------------------------------------------
// UC-2 - Forward and inverse kinematics agree
// ---------------------------------------------------------------------------
void UC2_RoundTrip() {
    cDriveCalculator calc;

    Scenario("KIN-03", "IK/FK round-trip");
    Given("a set of reachable targets across the envelope");
    When("IK is solved and the angles are fed back through FK");
    const std::vector<drivePosition> targets = {
        Pos(10, 0), Pos(40, 10), Pos(75, -15), Pos(120, 20),
        Pos(149, 0), Pos(60, -25), Pos(100, 25)
    };
    double worst = 0.0;
    bool allOk = true;
    for (const auto& target : targets) {
        AxelPostion axel{};
        if (calc.CalculateInverseKinematics(target, axel) != eKinematicStatus::Ok) allOk = false;
        const drivePosition back = calc.CalculateForwardKinematics(axel);
        worst = std::max(worst, std::max(std::abs(back.X - target.X), std::abs(back.Y - target.Y)));
    }
    Then(allOk, "every target solves Ok");
    AndThen(worst <= 1e-9, "round-trip error <= 1e-9 cm (worst " + Num(worst, 12) + ")");
    EndScenario();

    Scenario("KIN-04", "A2 is relative to link 1");
    Given("angles A1 = -180 deg, A2 = 180 deg");
    When("forward kinematics is evaluated");
    const drivePosition eof = calc.CalculateForwardKinematics(AxelPostion{-180.0, 180.0, 0.0, 0.0});
    Then(Near(eof.X, 0.0, 1e-9) && Near(eof.Y, 0.0, 1e-9),
         "end effector is at (0, 0) (got " + Num(eof.X, 9) + ", " + Num(eof.Y, 9) + ")");
    AndThen(!Near(eof.X, -100.0, 1e-6),
            "not the absolute-angle interpretation, which would give X = -100");
    EndScenario();
}

// ---------------------------------------------------------------------------
// UC-3 - Reach limits, not a hardcoded box, bound the motion
// ---------------------------------------------------------------------------
void UC3_ReachLimits() {
    cDriveCalculator calc;

    Scenario("KIN-05", "Beyond maximum reach is rejected, not silently retargeted");
    Given("a target at (200, 0), 225 cm from the base");
    When("inverse kinematics is solved");
    drivePosition target = Pos(200, 0);
    const drivePosition before = target;
    AxelPostion axel{};
    const eKinematicStatus status = calc.CalculateInverseKinematics(target, axel);
    Then(status == eKinematicStatus::OutOfReach, "status is OutOfReach (got " + std::string(ToString(status)) + ")");
    AndThen(target.X == before.X && target.Y == before.Y,
            "the caller's target is left untouched, not rescaled onto the annulus");
    EndScenario();

    Scenario("KIN-06", "Inside the folded dead zone is rejected");
    Given("a target at (-10, 0), 15 cm from the base");
    When("inverse kinematics is solved");
    AxelPostion inner{};
    const eKinematicStatus innerStatus = calc.CalculateInverseKinematics(Pos(-10, 0), inner);
    Then(innerStatus == eKinematicStatus::TooClose, "status is TooClose (got " + std::string(ToString(innerStatus)) + ")");
    EndScenario();

    Scenario("KIN-07", "Joint limits are enforced independently of reach");
    Given("a target at (62.5, 151.55): within reach but needing A1 near +60 deg");
    When("inverse kinematics is solved");
    AxelPostion cornered{};
    const eKinematicStatus jointStatus = calc.CalculateInverseKinematics(Pos(62.5, 151.55), cornered);
    Then(jointStatus == eKinematicStatus::JointLimit,
         "status is JointLimit (got " + std::string(ToString(jointStatus)) + ")");
    AndThen(cornered.A1 > A1_MAX_DEG, "A1 exceeds its limit (got " + Num(cornered.A1) + " > " + Num(A1_MAX_DEG) + ")");
    EndScenario();

    Scenario("KIN-08", "Declared envelope is consistent with reach");
    Given("the envelope end points on the X axis");
    When("each is solved and the reach boundary is searched");
    AxelPostion a{}, b{};
    const bool endsOk = calc.CalculateInverseKinematics(Pos(ENVELOPE_MIN_X_CM, 0), a) == eKinematicStatus::Ok &&
                        calc.CalculateInverseKinematics(Pos(ENVELOPE_MAX_X_CM, 0), b) == eKinematicStatus::Ok;
    double lo = 0.0, hi = 400.0;
    for (int i = 0; i < 60; ++i) {
        const double mid = 0.5 * (lo + hi);
        AxelPostion probe{};
        if (calc.CalculateInverseKinematics(Pos(mid, 0), probe) == eKinematicStatus::Ok) lo = mid;
        else hi = mid;
    }
    Then(endsOk, "both envelope end points on the X axis solve Ok");
    AndThen(Near(lo, 150.0, 0.01), "max reachable X at Y = 0 is 150 cm (found " + Num(lo, 4) + ")");
    EndScenario();
}

// ---------------------------------------------------------------------------
// UC-4 - Commanded position never diverges from the axle solution
// ---------------------------------------------------------------------------
void UC4_NoDivergence() {
    cDriveCalculator calc;

    Scenario("KIN-09", "Held button drives to the limit and stops there");
    Given("the arm starts at home (0, 0) with a fresh motion profile");
    When("+X is held for 400 ticks (20 s)");
    const RunTrace run = Drive(calc, Pos(0, 0), Sig(1, 0, 0, 0), 400);
    Then(run.finalPos.X <= ENVELOPE_MAX_X_CM + 1e-9,
         "final X <= 150 cm exactly, no epsilon overshoot (got " + Num(run.finalPos.X, 9) + ")");
    const drivePosition fk = calc.CalculateForwardKinematics(run.finalAxel);
    AndThen(Near(fk.X, run.finalPos.X, 1e-6) && Near(fk.Y, run.finalPos.Y, 1e-6),
            "FK of the commanded angles equals the stored position (dX " +
            Num(std::abs(fk.X - run.finalPos.X), 9) + ")");
    AndThen(run.finalStatus != eKinematicStatus::Ok,
            "final tick reports a constraint (got " + std::string(ToString(run.finalStatus)) + ")");
    EndScenario();

    Scenario("KIN-10", "Reversing at the limit moves on the very next tick");
    Given("the arm has been driven against the +X limit");
    When("-X is applied");
    calc.ResetMotionProfile();
    const drivePosition atLimit = run.finalPos;
    drivePosition reversed = atLimit;
    double afterOneTick = 0.0;
    for (int i = 0; i < 10; ++i) {
        drivePosition next{};
        AxelPostion axel{};
        calc.CalculateNextPosition(reversed, Sig(-1, 0, 0, 0), next, axel, TICK_MS);
        reversed = next;
        if (i == 0) afterOneTick = next.X;
    }
    Then(afterOneTick < atLimit.X,
         "X strictly decreases on the very first tick (" + Num(atLimit.X) + " -> " + Num(afterOneTick) + ")");
    AndThen(atLimit.X - reversed.X > 1.0,
            "more than 1 cm recovered within 10 ticks (0.5 s), versus the 3.9 s dead zone the "
            "silent-retarget bug produced (moved " + Num(atLimit.X - reversed.X, 3) + " cm)");
    EndScenario();

    Scenario("KIN-11", "Every committed pose is valid");
    Given("a 400-tick +X run followed by a 400-tick -X run");
    When("each committed pose is re-validated");
    cDriveCalculator calc2;
    const RunTrace out = Drive(calc2, Pos(0, 0), Sig(1, 0, 0, 0), 400);
    calc2.ResetMotionProfile();
    const RunTrace back = Drive(calc2, out.finalPos, Sig(-1, 0, 0, 0), 400);
    Then(out.allPosesValid && back.allPosesValid,
         "all " + std::to_string(out.poses.size() + back.poses.size()) +
         " committed poses solve Ok and sit inside the envelope");
    AndThen(!out.sawNaN && !back.sawNaN, "no coordinate or angle was ever NaN");
    const double drift = std::max(out.maxFkDrift, back.maxFkDrift);
    AndThen(drift <= 1e-6,
            "stored position matches FK of the commanded angles on every tick (worst drift " +
            Num(drift, 12) + " cm, versus 79.9 cm before the fix)");
    EndScenario();

    Scenario("KIN-22", "A diagonal command keeps its direction and does not creep");
    Given("the arm at (75, 0) with +X and +Y held together");
    When("100 ticks are commanded, long past the point Y saturates");
    cDriveCalculator diag;
    const RunTrace run2 = Drive(diag, Pos(75, 0), Sig(1, 1, 0, 0), 100);
    Then(run2.allPosesValid, "every committed pose is valid");
    AndThen(Near(run2.poses[19].X - 75.0, run2.poses[19].Y, 1e-9),
            "while both axes are free the move stays on the commanded 45 degree line, "
            "no radial retargeting (at tick 20: dX " + Num(run2.poses[19].X - 75.0, 9) +
            ", dY " + Num(run2.poses[19].Y, 9) + ")");
    AndThen(Near(run2.finalPos.Y, ENVELOPE_MAX_Y_CM, 1e-9),
            "Y saturates exactly at its +25 cm limit (got " + Num(run2.finalPos.Y, 9) + ")");
    // Coordinated motion: a limit on one axis halts the whole move rather than
    // silently turning it into a slide along the wall. Direction is preserved,
    // which for a positioner is more predictable than changing it mid-command.
    AndThen(Near(run2.poses[99].X, run2.poses[49].X, 1e-12),
            "once Y saturates the move halts instead of creeping along the wall "
            "(X at tick 50 " + Num(run2.poses[49].X, 9) + " vs tick 100 " +
            Num(run2.poses[99].X, 9) + ")");
    EndScenario();

    Scenario("KIN-24", "A saturated axis does not drift over a long hold");
    Given("the arm held against the +X limit and against a saturated diagonal");
    When("20000 further ticks are commanded (about 17 minutes of held button)");
    cDriveCalculator soakX;
    const RunTrace settleX = Drive(soakX, Pos(0, 0), Sig(1, 0, 0, 0), 400);
    const RunTrace soakedX = Drive(soakX, settleX.finalPos, Sig(1, 0, 0, 0), 20000);
    cDriveCalculator soakD;
    const RunTrace settleD = Drive(soakD, Pos(75, 0), Sig(1, 1, 0, 0), 100);
    const RunTrace soakedD = Drive(soakD, settleD.finalPos, Sig(1, 1, 0, 0), 20000);
    Then(Near(soakedX.finalPos.X, settleX.finalPos.X, 1e-12),
         "X at the limit is bit-identical after 20000 ticks (" +
         Num(settleX.finalPos.X, 9) + " -> " + Num(soakedX.finalPos.X, 9) + ")");
    AndThen(Near(soakedD.finalPos.X, settleD.finalPos.X, 1e-12) &&
            Near(soakedD.finalPos.Y, settleD.finalPos.Y, 1e-12),
            "the free axis of a saturated diagonal does not creep (X " +
            Num(soakedD.finalPos.X, 9) + ", Y " + Num(soakedD.finalPos.Y, 9) + ")");
    AndThen(soakedX.allPosesValid && soakedD.allPosesValid && !soakedX.sawNaN && !soakedD.sawNaN,
            "all 40000 soak poses stay valid and finite");
    EndScenario();

    Scenario("KIN-23", "The Y envelope limit is enforced exactly");
    Given("the arm at (75, 0) with +Y held");
    When("200 ticks are commanded");
    cDriveCalculator vertical;
    const RunTrace run3 = Drive(vertical, Pos(75, 0), Sig(0, 1, 0, 0), 200);
    Then(run3.finalPos.Y <= ENVELOPE_MAX_Y_CM + 1e-9,
         "Y never exceeds +25 cm (got " + Num(run3.finalPos.Y, 9) + ")");
    AndThen(Near(run3.finalPos.X, 75.0, 1e-9),
            "X is untouched by pure-Y motion (got " + Num(run3.finalPos.X, 9) + ")");
    EndScenario();
}

// ---------------------------------------------------------------------------
// UC-5 - Motion respects the velocity and acceleration profile
// ---------------------------------------------------------------------------
void UC5_MotionProfile() {
    Scenario("KIN-12", "Per-tick joint step never exceeds the profile budget");
    Given("the drive starts at a mid-workspace pose (100, 0)");
    When("+X is held for 100 ticks");
    cDriveCalculator calc;
    const RunTrace run = Drive(calc, Pos(100, 0), Sig(1, 0, 0, 0), 100);
    Then(run.maxJointBudgetBreach <= 1e-8,
         "worst budget breach <= 1e-8 deg (got " + Num(run.maxJointBudgetBreach, 12) + ")");
    AndThen(run.maxJointStep <= MAX_JOINT_SPEED_DPS * TICK_S + 1e-8,
            "no tick exceeds " + Num(MAX_JOINT_SPEED_DPS * TICK_S, 3) + " deg (peak " +
            Num(run.maxJointStep, 9) + ")");
    EndScenario();

    Scenario("KIN-13", "Speed ramps rather than stepping to maximum");
    Given("a freshly reset motion profile");
    When("ten consecutive ticks are commanded");
    cDriveCalculator ramp;
    ramp.ResetMotionProfile();
    drivePosition current = Pos(100, 0);
    std::vector<double> speeds;
    bool monotonic = true;
    double previous = -1.0;
    for (int i = 0; i < 10; ++i) {
        drivePosition next{};
        AxelPostion axel{};
        ramp.CalculateNextPosition(current, Sig(1, 0, 0, 0), next, axel, TICK_MS);
        current = next;
        const double speed = ramp.GetProfileSpeedDps();
        if (speed <= previous) monotonic = false;
        previous = speed;
        speeds.push_back(speed);
    }
    Then(monotonic, "profile speed increases every tick");
    AndThen(Near(speeds.back(), MAX_JOINT_SPEED_DPS, 1e-9),
            "reaches MAX_JOINT_SPEED after 10 ticks (got " + Num(speeds.back()) + ")");
    drivePosition beyond{};
    AxelPostion beyondAxel{};
    ramp.CalculateNextPosition(current, Sig(1, 0, 0, 0), beyond, beyondAxel, TICK_MS);
    AndThen(ramp.GetProfileSpeedDps() <= MAX_JOINT_SPEED_DPS + 1e-9,
            "never exceeds MAX_JOINT_SPEED (got " + Num(ramp.GetProfileSpeedDps()) + ")");
    EndScenario();

    Scenario("KIN-14", "Stop resets the profile");
    Given("the profile has ramped to maximum");
    auto fake = std::make_shared<cFakePcanController>();
    fake->Start();
    cDriveController controller(fake);
    controller.StartDrive(Sig(0, 0, 1, 0));
    for (int i = 0; i < 15; ++i) controller.HandleJoystick(Sig(0, 0, 1, 0));
    const double rampedSpeed = controller.GetProfileSpeedDps();
    When("StopDrive is issued");
    controller.StopDrive(Sig(0, 0, 0, 0));
    Then(Near(rampedSpeed, MAX_JOINT_SPEED_DPS, 1e-9),
         "profile had reached maximum before the stop (got " + Num(rampedSpeed) + ")");
    AndThen(Near(controller.GetProfileSpeedDps(), 0.0, 1e-12),
            "profile speed returns to 0 (got " + Num(controller.GetProfileSpeedDps()) + ")");
    controller.HandleJoystick(Sig(0, 0, 1, 0));
    AndThen(Near(controller.GetProfileSpeedDps(), JOINT_ACCEL_DPSS * TICK_S, 1e-9),
            "the next motion restarts at the bottom of the ramp (got " +
            Num(controller.GetProfileSpeedDps()) + " deg/s)");
    EndScenario();

    Scenario("KIN-15", "The limited path is never faster than the unlimited request");
    Given("a held-button run across the whole workspace");
    When("the achieved Cartesian step is measured on every tick");
    cDriveCalculator sweep;
    const RunTrace full = Drive(sweep, Pos(0, 0), Sig(1, 0, 0, 0), 400);
    const double allowed = MAX_LINEAR_SPEED_CMPS * TICK_S;
    Then(full.maxCartesianStep <= allowed + 1e-9,
         "peak step " + Num(full.maxCartesianStep) + " cm <= " + Num(allowed) + " cm");
    EndScenario();
}

// ---------------------------------------------------------------------------
// UC-6 - Angular axes move in their own units
// ---------------------------------------------------------------------------
void UC6_AngularAxes() {
    Scenario("KIN-16", "LAO/CRAN move at the angular speed, not the linear one");
    Given("the profile is fully ramped at a mid-workspace pose");
    cDriveCalculator calc;
    drivePosition current = Pos(100, 0);
    for (int i = 0; i < 12; ++i) {
        drivePosition next{};
        AxelPostion axel{};
        calc.CalculateNextPosition(current, Sig(0, 0, 0, 1), next, axel, TICK_MS);
        current = next;
    }
    When("CRAN+ is held for one more tick");
    const drivePosition before = current;
    drivePosition after{};
    AxelPostion axel{};
    calc.CalculateNextPosition(before, Sig(0, 0, 0, 1), after, axel, TICK_MS);
    const double expected = MAX_ANGULAR_SPEED_DPS * TICK_S;
    Then(Near(after.CRAN - before.CRAN, expected, 1e-6),
         "dCRAN = " + Num(expected, 3) + " deg (got " + Num(after.CRAN - before.CRAN) + ")");
    AndThen(Near(after.X, before.X, 1e-12) && Near(after.Y, before.Y, 1e-12),
            "X and Y are unchanged");
    EndScenario();

    Scenario("KIN-17", "Angular axes clamp at their travel limits");
    Given("LAO is driven positive for 200 ticks");
    cDriveCalculator lao;
    const RunTrace run = Drive(lao, Pos(100, 0), Sig(0, 0, 1, 0), 200);
    When("the final LAO is inspected");
    Then(run.finalPos.LAO <= A4_MAX_DEG + 1e-9,
         "LAO never exceeds +180 deg, no epsilon overshoot (got " + Num(run.finalPos.LAO, 9) + ")");
    AndThen(Near(run.finalPos.LAO, A4_MAX_DEG, 1e-9),
            "LAO saturates exactly at its limit rather than stalling early");
    EndScenario();
}

// ---------------------------------------------------------------------------
// UC-7 - Numerical robustness
// ---------------------------------------------------------------------------
void UC7_Robustness() {
    cDriveCalculator calc;

    Scenario("KIN-18", "No NaN at the exact reach boundaries");
    Given("targets on and immediately either side of both reach circles");
    When("inverse kinematics is solved for each");
    const std::vector<double> probes = {
        0.0, -1e-12, 1e-12,
        ENVELOPE_MAX_X_CM, ENVELOPE_MAX_X_CM - 1e-12, ENVELOPE_MAX_X_CM + 1e-12
    };
    bool sawNaN = false;
    for (double x : probes) {
        AxelPostion axel{};
        calc.CalculateInverseKinematics(Pos(x, 0), axel);
        if (AnyNaN(axel)) sawNaN = true;
    }
    Then(!sawNaN, "no returned angle is NaN at any boundary probe");
    AxelPostion inner{}, outer{};
    const eKinematicStatus innerStatus = calc.CalculateInverseKinematics(Pos(0, 0), inner);
    const eKinematicStatus outerStatus = calc.CalculateInverseKinematics(Pos(ENVELOPE_MAX_X_CM, 0), outer);
    AndThen(innerStatus == eKinematicStatus::Ok && outerStatus == eKinematicStatus::Ok,
            "both boundary poses themselves report Ok");
    EndScenario();

    Scenario("KIN-19", "Degenerate joystick input is inert");
    Given("a profile that has already ramped up");
    cDriveCalculator idle;
    drivePosition current = Pos(100, 0);
    for (int i = 0; i < 5; ++i) {
        drivePosition next{};
        AxelPostion axel{};
        idle.CalculateNextPosition(current, Sig(1, 0, 0, 0), next, axel, TICK_MS);
        current = next;
    }
    When("an all-zero joystick signal is processed");
    drivePosition after{};
    AxelPostion axel{};
    const eKinematicStatus status = idle.CalculateNextPosition(current, Sig(0, 0, 0, 0), after, axel, TICK_MS);
    Then(Near(after.X, current.X, 1e-12) && Near(after.Y, current.Y, 1e-12),
         "the position is unchanged");
    AndThen(Near(idle.GetProfileSpeedDps(), 0.0, 1e-12), "the motion profile is reset");
    AndThen(status == eKinematicStatus::Ok, "status is Ok (got " + std::string(ToString(status)) + ")");
    EndScenario();
}

// ---------------------------------------------------------------------------
// UC-8 - Safety interlocks still dominate
// ---------------------------------------------------------------------------
void UC8_SafetyInterlocks() {
    Scenario("KIN-20", "Emergency stop blocks motion and zeroes speed");
    auto fake = std::make_shared<cFakePcanController>();
    fake->Start();
    cDriveController controller(fake);
    Given("the drive is running");
    controller.StartDrive(Sig(1, 0, 0, 0));
    for (int i = 0; i < 5; ++i) controller.HandleJoystick(Sig(1, 0, 0, 0));
    const drivePosition beforeStop = controller.GetCurrentPosition();
    When("SetEmgStop is called and +X is then held for 10 ticks");
    controller.SetEmgStop();
    for (int i = 0; i < 10; ++i) controller.HandleJoystick(Sig(1, 0, 0, 0));
    const drivePosition afterStop = controller.GetCurrentPosition();
    Then(Near(afterStop.X, beforeStop.X, 1e-12) && Near(afterStop.Y, beforeStop.Y, 1e-12),
         "the position does not change");
    AndThen(Near(fake->LastSpeed(), 0.0f, 1e-9), "the last speed written to the bus is 0");
    EndScenario();

    Scenario("KIN-21", "An active fault blocks motion until cleared");
    auto fake2 = std::make_shared<cFakePcanController>();
    fake2->Start();
    cDriveController faulted(fake2);
    Given("SetError(7) has been called");
    faulted.StartDrive(Sig(1, 0, 0, 0));
    faulted.SetError(7);
    const drivePosition beforeFault = faulted.GetCurrentPosition();
    When("+X is held for 10 ticks");
    for (int i = 0; i < 10; ++i) faulted.HandleJoystick(Sig(1, 0, 0, 0));
    const drivePosition duringFault = faulted.GetCurrentPosition();
    Then(Near(duringFault.X, beforeFault.X, 1e-12), "the position does not change");
    faulted.SetError(0);
    for (int i = 0; i < 10; ++i) faulted.HandleJoystick(Sig(1, 0, 0, 0));
    AndThen(faulted.GetCurrentPosition().X > duringFault.X, "after SetError(0) the arm moves again");
    EndScenario();
}

int main() {
    std::cout << "RTMC drive kinematics - BDD suite\n";
    std::cout << "geometry: L1=" << LINK1_LEN_CM << " L2=" << LINK2_LEN_CM
              << " base=(" << BASE_X_CM << "," << BASE_Y_CM << ")"
              << " reach=[" << MIN_REACH_CM << "," << MAX_REACH_CM << "]"
              << " envelope X=[" << ENVELOPE_MIN_X_CM << "," << ENVELOPE_MAX_X_CM << "]"
              << " Y=[" << ENVELOPE_MIN_Y_CM << "," << ENVELOPE_MAX_Y_CM << "]\n";

    UC1_HomePose();
    UC2_RoundTrip();
    UC3_ReachLimits();
    UC4_NoDivergence();
    UC5_MotionProfile();
    UC6_AngularAxes();
    UC7_Robustness();
    UC8_SafetyInterlocks();

    std::cout << "\n----------------------------------------------------------\n";
    std::cout << "scenarios: " << (g_scenarios - g_scenariosFailed) << "/" << g_scenarios << " passed\n";
    std::cout << "checks:    " << (g_checks - g_checksFailed) << "/" << g_checks << " passed\n";
    if (!g_failures.empty()) {
        std::cout << "\nfailures:\n";
        for (const auto& failure : g_failures) std::cout << "  - " << failure << "\n";
    }
    std::cout << "----------------------------------------------------------\n";
    return g_checksFailed == 0 ? 0 : 1;
}
