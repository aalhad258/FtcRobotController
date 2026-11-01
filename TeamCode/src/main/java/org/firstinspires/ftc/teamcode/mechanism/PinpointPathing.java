package org.firstinspires.ftc.teamcode.mechanism;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class PinpointPathing {

    private final Drive drive = new Drive();

    private GoBildaPinpointDriver pinpoint;

    private Pose2D currentPose;

    private Pose2D targetPose;

    private enum State {
        TURN,
        MOVE,
        DONE
    }

    private State state;

    // ---------------- PID CONSTANTS ----------------

    private double kPX = 0.1;
    private double kDX = 0.001;

    private double kPY = 0.1;
    private double kDY = 0.001;

    private double kPH = 0.01;
    private double kDH = 0.001;

    private double toleranceX = 0.5;
    private double toleranceY = 0.5;
    private double toleranceH = 2.0;

    // ---------------- ERROR TRACKING ----------------

    private double errorX;
    private double errorY;
    private double errorH;

    private double lastErrorX;
    private double lastErrorY;
    private double lastErrorH;

    private double lastTimeX;
    private double lastTimeY;
    private double lastTimeH;

    private double forward;
    private double strafe;
    private double rotate;

    private double curTime;

    // ---------------- INITIALIZATION ----------------

    public void init(HardwareMap hardwareMap, Pose2D startPose) {

        pinpoint = hardwareMap.get(
                GoBildaPinpointDriver.class,
                "pinpoint"
        );

        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );

        pinpoint.setOffsets(2, -6.5, DistanceUnit.INCH);

        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );

        pinpoint.setPosition(startPose);

        drive.init(hardwareMap);

        currentPose = startPose;

        state = State.DONE;
    }

    // ---------------- SET TARGET ----------------

    public void setTarget(Pose2D target) {

        targetPose = target;

        resetPID();

        pinpoint.update();
        currentPose = pinpoint.getPosition();

        errorH = AngleUnit.normalizeDegrees(
                targetPose.getHeading(AngleUnit.DEGREES)
                        - currentPose.getHeading(AngleUnit.DEGREES)
        );

        if (Math.abs(errorH) > toleranceH) {
            state = State.TURN;
        } else {
            state = State.MOVE;
        }
    }

    // ---------------- MAIN UPDATE ----------------

    public void update() {

        if (state == State.DONE) {
            drive.setPower(0, 0, 0, 1.0);
            return;
        }

        pinpoint.update();
        currentPose = pinpoint.getPosition();

        curTime = System.nanoTime() * 1e-9;

        if (state == State.TURN) {
            updateTurn();
        }

        else if (state == State.MOVE) {
            updateMove();
        }
    }

    // ---------------- TURN ----------------

    private void updateTurn() {

        errorH = AngleUnit.normalizeDegrees(
                targetPose.getHeading(AngleUnit.DEGREES)
                        - currentPose.getHeading(AngleUnit.DEGREES)
        );

        if (Math.abs(errorH) <= toleranceH) {

            drive.setPower(0, 0, 0, 1.0);

            state = State.MOVE;

            lastErrorX = 0;
            lastErrorY = 0;

            lastTimeX = curTime;
            lastTimeY = curTime;

            return;
        }

        double pTerm = errorH * kPH;

        double dT = curTime - lastTimeH;

        double dTerm = 0;

        if (dT > 0) {
            dTerm = ((errorH - lastErrorH) / dT) * kDH;
        }

        rotate = Range.clip(
                pTerm + dTerm,
                -0.4,
                0.4
        );

        drive.setPower(0, 0, -rotate, 1.0);

        lastTimeH = curTime;
        lastErrorH = errorH;
    }

    // ---------------- MOVE ----------------

    private void updateMove() {

        double fieldErrorX =
                targetPose.getX(DistanceUnit.INCH)
                        - currentPose.getX(DistanceUnit.INCH);

        double fieldErrorY =
                targetPose.getY(DistanceUnit.INCH)
                        - currentPose.getY(DistanceUnit.INCH);

        double headingRad = Math.toRadians(
                currentPose.getHeading(AngleUnit.DEGREES)
        );

        double cosH = Math.cos(headingRad);
        double sinH = Math.sin(headingRad);

        errorX = fieldErrorX * cosH + fieldErrorY * sinH;

        errorY = -fieldErrorX * sinH + fieldErrorY * cosH;

        // Heading correction

        errorH = AngleUnit.normalizeDegrees(
                targetPose.getHeading(AngleUnit.DEGREES)
                        - currentPose.getHeading(AngleUnit.DEGREES)
        );

        rotate = Range.clip(
                errorH * kPH,
                -0.3,
                0.3
        );

        // Y controller

        if (Math.abs(errorY) > toleranceY) {

            double pTerm = errorY * kPY;

            double dT = curTime - lastTimeY;

            double dTerm = 0;

            if (dT > 0) {
                dTerm = ((errorY - lastErrorY) / dT) * kDY;
            }

            forward = Range.clip(
                    pTerm + dTerm,
                    -0.4,
                    0.4
            );

        } else {
            forward = 0;
        }

        // X controller

        if (Math.abs(errorX) > toleranceX) {

            double pTerm = errorX * kPX;

            double dT = curTime - lastTimeX;

            double dTerm = 0;

            if (dT > 0) {
                dTerm = ((errorX - lastErrorX) / dT) * kDX;
            }

            strafe = Range.clip(
                    pTerm + dTerm,
                    -0.4,
                    0.4
            );

        } else {
            strafe = 0;
        }

        // Target reached

        if (Math.abs(errorX) <= toleranceX &&
                Math.abs(errorY) <= toleranceY) {

            state = State.DONE;

            drive.setPower(0, 0, 0, 1.0);

            return;
        }

        // Preserve your tested Drive command conventions.

        drive.setPower(
                strafe,
                -forward,
                -rotate,
                1.0
        );

        lastErrorX = errorX;
        lastErrorY = errorY;

        lastTimeX = curTime;
        lastTimeY = curTime;
    }

    // ---------------- HELPERS ----------------

    private void resetPID() {

        lastErrorX = 0;
        lastErrorY = 0;
        lastErrorH = 0;

        lastTimeX = System.nanoTime() * 1e-9;
        lastTimeY = lastTimeX;
        lastTimeH = lastTimeX;

        forward = 0;
        strafe = 0;
        rotate = 0;
    }

    public boolean isBusy() {
        return state != State.DONE;
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

    public double getErrorX() {
        return errorX;
    }

    public double getErrorY() {
        return errorY;
    }

    public double getErrorH() {
        return errorH;
    }
}