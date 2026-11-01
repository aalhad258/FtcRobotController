package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Pinpoint-based "turn, then drive" pathing.
 *
 * Coordinate conventions (Pinpoint / FTC standard):
 *   +X = robot forward at heading 0, +Y = robot left, heading is CCW-positive.
 *
 * Robot-relative errors:
 *   errorX = how far the target is IN FRONT of the robot (+ = ahead)
 *   errorY = how far the target is to the LEFT of the robot (+ = left)
 *   errorH = heading error in degrees (+ = need to turn CCW / left)
 *
 * If the robot moves the wrong way, DO NOT edit the math. Run the
 * "Pinpoint Axis Test" TeleOp and flip the three SIGN constants below.
 */
public class PinpointPathing {

    // ---------------- DRIVE SIGN CONFIG (tune with PinpointAxisTest) ----------------
    // Drive.setPower(axial + = forward, lateral + = RIGHT, yaw + = clockwise)
    private static final double FORWARD_SIGN = 1.0;   // forward command -> axial
    private static final double STRAFE_SIGN  = -1.0;  // LEFT command -> lateral (right is +)
    private static final double TURN_SIGN    = -1.0;  // CCW command -> yaw (clockwise is +)

    // ---------------- TUNING ----------------
    private static final double KP_DRIVE = 0.04;   // power per inch
    private static final double KD_DRIVE = 0.002;
    private static final double MAX_DRIVE = 0.5;
    private static final double MIN_DRIVE = 0.12;  // enough to actually overcome friction

    private static final double KP_TURN = 0.012;   // power per degree
    private static final double KD_TURN = 0.0005;
    private static final double MAX_TURN = 0.4;
    private static final double MIN_TURN = 0.10;

    private static final double MAX_HOLD = 0.25;   // heading correction while driving

    private double toleranceX = 1.0;
    private double toleranceY = 1.0;
    private double toleranceH = 3.0;

    private static final double DEFAULT_TIMEOUT_SECONDS = 8.0;

    // ---------------- INTERNALS ----------------
    private final Drive drive = new Drive();
    private GoBildaPinpointDriver pinpoint;

    private Pose2D startPose;
    private Pose2D currentPose;
    private Pose2D targetPose;

    private boolean poseApplied = false;

    private enum State { TURN, MOVE, DONE }
    private State state = State.DONE;

    private double errorX, errorY, errorH;

    private double curTime;
    private double startTime;
    private double timeout = DEFAULT_TIMEOUT_SECONDS;
    private boolean timedOut = false;

    private final PD fwdPD    = new PD(KP_DRIVE, KD_DRIVE, MAX_DRIVE, MIN_DRIVE);
    private final PD strafePD = new PD(KP_DRIVE, KD_DRIVE, MAX_DRIVE, MIN_DRIVE);
    private final PD turnPD   = new PD(KP_TURN, KD_TURN, MAX_TURN, MIN_TURN);
    private final PD holdPD   = new PD(KP_TURN, KD_TURN, MAX_HOLD, 0.0);

    /** PD controller that doesn't spike on its first loop and enforces a minimum output. */
    private static class PD {
        final double kP, kD, max, min;
        double lastErr, lastT;
        boolean has = false;

        PD(double kP, double kD, double max, double min) {
            this.kP = kP; this.kD = kD; this.max = max; this.min = min;
        }

        void reset() { has = false; }

        double calc(double err, double t) {
            double d = 0;
            if (has) {
                double dt = t - lastT;
                if (dt > 1e-4) d = ((err - lastErr) / dt) * kD;
            }
            lastErr = err;
            lastT = t;
            has = true;

            double out = Range.clip(err * kP + d, -max, max);
            if (Math.abs(out) < min) out = Math.signum(err) * min;
            return out;
        }
    }

    // ---------------- INITIALIZATION ----------------

    public void init(HardwareMap hardwareMap, Pose2D start) {

        startPose = start;

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        pinpoint.setOffsets(2, -6.5, DistanceUnit.INCH);

        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

        // Zero the IMU, give it time, THEN set our start pose.
        pinpoint.resetPosAndIMU();
        sleepMs(300);

        drive.init(hardwareMap);

        currentPose = start;
        poseApplied = false;
        state = State.DONE;

        updatePose(); // applies start pose now if the Pinpoint is ready
    }

    private void sleepMs(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public boolean isReady() {
        return pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
    }

    public String getPinpointStatus() {
        return pinpoint.getDeviceStatus().toString();
    }

    public boolean isPoseApplied() {
        return poseApplied;
    }

    /** Reads the Pinpoint. Call this in init_loop() too, so the start pose gets applied once it's ready. */
    public void updatePose() {
        pinpoint.update();

        if (!poseApplied && isReady()) {
            pinpoint.setPosition(startPose);
            pinpoint.update();
            poseApplied = true;
        }

        currentPose = pinpoint.getPosition();
    }

    // ---------------- SET TARGET ----------------

    public void setTarget(Pose2D target) {
        setTarget(target, DEFAULT_TIMEOUT_SECONDS);
    }

    public void setTarget(Pose2D target, double timeoutSeconds) {

        targetPose = target;
        timeout = timeoutSeconds;
        timedOut = false;
        startTime = now();

        resetControllers();

        updatePose();
        computeErrors();

        state = (Math.abs(errorH) > toleranceH) ? State.TURN : State.MOVE;
    }

    // ---------------- MAIN UPDATE ----------------

    /** Call once per loop. Always refreshes the pose, even when idle. */
    public void update() {

        updatePose();

        if (state == State.DONE) {
            drive.setPower(0, 0, 0, 1.0);
            return;
        }

        curTime = now();
        computeErrors();

        if (curTime - startTime > timeout) {
            timedOut = true;
            finish();
            return;
        }

        if (state == State.TURN) {
            updateTurn();
        } else {
            updateMove();
        }
    }

    // ---------------- TURN ----------------

    private void updateTurn() {

        if (Math.abs(errorH) <= toleranceH) {
            drive.setPower(0, 0, 0, 1.0);
            fwdPD.reset();
            strafePD.reset();
            turnPD.reset();
            holdPD.reset();
            state = State.MOVE;
            return;
        }

        double rotate = turnPD.calc(errorH, curTime);

        drive.setPower(0, 0, TURN_SIGN * rotate, 1.0);
    }

    // ---------------- MOVE ----------------

    private void updateMove() {

        boolean xDone = Math.abs(errorX) <= toleranceX;
        boolean yDone = Math.abs(errorY) <= toleranceY;
        boolean hDone = Math.abs(errorH) <= toleranceH;

        if (xDone && yDone && hDone) {
            finish();
            return;
        }

        // Position is good, only heading is off: just rotate in place.
        if (xDone && yDone) {
            double rotate = turnPD.calc(errorH, curTime);
            drive.setPower(0, 0, TURN_SIGN * rotate, 1.0);
            return;
        }

        double forward = 0;
        double strafe = 0;
        double rotate = 0;

        if (!xDone) forward = fwdPD.calc(errorX, curTime);
        else fwdPD.reset();

        if (!yDone) strafe = strafePD.calc(errorY, curTime);
        else strafePD.reset();

        if (!hDone) rotate = holdPD.calc(errorH, curTime);
        else holdPD.reset();

        // forward error -> axial, LEFT error -> lateral
        drive.setPower(
                FORWARD_SIGN * forward,
                STRAFE_SIGN * strafe,
                TURN_SIGN * rotate,
                1.0);
    }

    // ---------------- HELPERS ----------------

    private void computeErrors() {

        double fieldErrorX = targetPose.getX(DistanceUnit.INCH)
                - currentPose.getX(DistanceUnit.INCH);

        double fieldErrorY = targetPose.getY(DistanceUnit.INCH)
                - currentPose.getY(DistanceUnit.INCH);

        double headingRad = currentPose.getHeading(AngleUnit.RADIANS);

        double cosH = Math.cos(headingRad);
        double sinH = Math.sin(headingRad);

        // Rotate field error into the robot frame.
        errorX = fieldErrorX * cosH + fieldErrorY * sinH;    // ahead of robot
        errorY = -fieldErrorX * sinH + fieldErrorY * cosH;   // left of robot

        errorH = AngleUnit.normalizeDegrees(
                targetPose.getHeading(AngleUnit.DEGREES)
                        - currentPose.getHeading(AngleUnit.DEGREES));
    }

    private void resetControllers() {
        fwdPD.reset();
        strafePD.reset();
        turnPD.reset();
        holdPD.reset();
    }

    private void finish() {
        state = State.DONE;
        drive.setPower(0, 0, 0, 1.0);
    }

    private double now() {
        return System.nanoTime() * 1e-9;
    }

    // ---------------- PUBLIC API ----------------

    /** Direct motor control for the axis test OpMode (no signs applied). */
    public void rawDrive(double axial, double lateral, double yaw) {
        drive.setPower(axial, lateral, yaw, 1.0);
    }

    public boolean isBusy() {
        return state != State.DONE;
    }

    public boolean didTimeOut() {
        return timedOut;
    }

    public void stop() {
        state = State.DONE;
        drive.setPower(0, 0, 0, 1.0);
    }

    public Pose2D getCurrentPose() {
        return currentPose;
    }

    public String getState() {
        return state.toString();
    }

    /** Robot-frame error: + means target is AHEAD of the robot. */
    public double getErrorX() {
        return errorX;
    }

    /** Robot-frame error: + means target is to the LEFT of the robot. */
    public double getErrorY() {
        return errorY;
    }

    public double getErrorH() {
        return errorH;
    }
}