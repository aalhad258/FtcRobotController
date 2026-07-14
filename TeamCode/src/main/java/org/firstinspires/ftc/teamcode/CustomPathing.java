package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.mechanism.Drive;

@Autonomous
public class CustomPathing extends OpMode {

    private final Drive drive = new Drive();
    private final Pose2D startPose = new Pose2D(DistanceUnit.INCH,0,0, AngleUnit.DEGREES, 0);
    private final Pose2D endPose = new Pose2D(DistanceUnit.INCH,10,5, AngleUnit.DEGREES, 0);
    private Pose2D currentPose = new Pose2D(DistanceUnit.INCH,0,0, AngleUnit.DEGREES, 0);
    GoBildaPinpointDriver pinpoint;

    double kPX = 0.002;
    double errorX = 0;
    double lastErrorX = 0;
    double toleranceX = 0.5;
    double kDX = 0.0001;
    double curTime = 0;
    double lastTime = 0;
    double strafe;
    double kPY = 0.002;
    double errorY = 0;
    double lastErrorY = 0;
    double toleranceY = 0.5;
    double kDY = 0.0001;
    double forward;
    double kPH = 0.002;
    double errorH = 0;
    double lastErrorH = 0;
    double toleranceH= 0.5;
    double kDH = 0.0001;
    double rotate;
    private enum State {
        TURN,
        MOVE,
        DONE
    }

    private State state;

    @Override
    public void init() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(0, 10, DistanceUnit.INCH);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.setPosition(startPose);
        drive.init(hardwareMap);
        if (startPose.getHeading(AngleUnit.DEGREES) != endPose.getHeading(AngleUnit.DEGREES)) {
            state = State.TURN;
        } else {
            state = State.MOVE;
        }
    }

    public void start() {
        resetRuntime();
        curTime = getRuntime();
    }

    @Override
    public void loop() {
        if (state == State.TURN) {
            pinpoint.update();
            currentPose = pinpoint.getPosition();
            curTime = getRuntime();
            errorH = AngleUnit.normalizeDegrees(endPose.getHeading(AngleUnit.DEGREES)-currentPose.getHeading(AngleUnit.DEGREES));
            if (Math.abs(errorH) > toleranceH) {
                double pTerm = errorH * kPH;
                double dT = curTime - lastTime;
                double dTerm;
                if (dT != 0) {
                    dTerm = ((errorH - lastErrorH) / dT) * kDH;
                } else {
                    dTerm = 0;
                }
                rotate = Range.clip(pTerm + dTerm, -0.4, 0.4);
                drive.setPower(0, 0, rotate, 1.0);
            } else {
                drive.setPower(0, 0, 0, 1.0);
                state = State.MOVE;
                lastErrorX = 0;
                lastErrorY = 0;
            }
            lastTime = curTime;
            lastErrorH = errorH;
        } else if (state == State.MOVE) {
            pinpoint.update();
            currentPose = pinpoint.getPosition();
            curTime = getRuntime();
            double fieldErrorX = endPose.getX(DistanceUnit.INCH) - currentPose.getX(DistanceUnit.INCH);
            double fieldErrorY = endPose.getY(DistanceUnit.INCH) - currentPose.getY(DistanceUnit.INCH);
            double headingRad = Math.toRadians(currentPose.getHeading(AngleUnit.DEGREES));
            double cosH = Math.cos(headingRad);
            double sinH = Math.sin(headingRad);
            errorX =  fieldErrorX * cosH + fieldErrorY * sinH;
            errorY = -fieldErrorX * sinH + fieldErrorY * cosH;

            if (Math.abs(errorY) > toleranceY) {
                double pTerm = errorY * kPY;
                double dT = curTime - lastTime;
                double dTerm;
                if (dT != 0) {
                    dTerm = ((errorY - lastErrorY) / dT) * kDY;
                } else {
                    dTerm = 0;
                }
                forward = Range.clip(pTerm + dTerm, -0.4, 0.4);
            } else {
                forward = 0;
            }

            if (Math.abs(errorX) > toleranceX) {
                double pTerm = errorX * kPX;
                double dT = curTime - lastTime;
                double dTerm;
                if (dT != 0) {
                    dTerm = ((errorX - lastErrorX) / dT) * kDX;
                } else {
                    dTerm = 0;
                }
                strafe = Range.clip(pTerm + dTerm, -0.4, 0.4);
            } else {
                strafe = 0;
            }

            if ((forward == 0) && (strafe == 0)) {
                state = State.DONE;
            }

            drive.setPower(strafe, forward, 0, 1.0);
            lastErrorY = errorY;
            lastErrorX = errorX;
            lastTime = curTime;
        } else if (state == State.DONE) {
            drive.setPower(0, 0, 0, 1.0);
        }
    }
}
