package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.mechanism.Drive;

@Autonomous
public class ObstacleNavigation extends OpMode {
    AnalogInput ranger;
    double distance;

    private final Pose2D startPose = new Pose2D(DistanceUnit.INCH,0,0, AngleUnit.DEGREES, 0);
    private Pose2D currentPose = new Pose2D(DistanceUnit.INCH,0,0, AngleUnit.DEGREES, 0);
    private Pose2D lastPose = new Pose2D(DistanceUnit.INCH,0,0, AngleUnit.DEGREES, 0);
    GoBildaPinpointDriver pinpoint;

    Drive drive = new Drive();

    private enum State {
        DRIVE,
        STRAFE_UNTIL_CLEAR,
        STRAFE_AFTER_CLEAR
    }

    private State state;

    @Override
    public void init() {
        ranger = hardwareMap.get(AnalogInput.class, "ranger");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(2, -6.5, DistanceUnit.INCH);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.setPosition(startPose);
        drive.init(hardwareMap);
        state = State.DRIVE;
    }

    @Override
    public void loop() {
        pinpoint.update();
        currentPose = pinpoint.getPosition();
        distance = (ranger.getVoltage()*48.7)-4.9;

        telemetry.addData("Inch 20DEG 0-0 Mode: ", distance);

        telemetry.update();

        if (state == State.DRIVE) {
            if (distance <= 20) {
                state = State.STRAFE_UNTIL_CLEAR;
            } else {
                drive.setPower(-1.0, 0.0, 0.0, 0.5);
            }
        } else if (state == State.STRAFE_UNTIL_CLEAR){
            drive.setPower(0.0, 1.0, 0.0, 0.5);
            if (distance > 20) {
                state = State.STRAFE_AFTER_CLEAR;
                lastPose = currentPose;
            }
        } else if (state == State.STRAFE_AFTER_CLEAR) {
            if (Math.abs(currentPose.getY(DistanceUnit.INCH) - lastPose.getY(DistanceUnit.INCH)) > 10) {
                state = State.DRIVE;
            } else {
                drive.setPower(0.0, 1.0, 0.0, 0.5);
            }
        }
    }
}
