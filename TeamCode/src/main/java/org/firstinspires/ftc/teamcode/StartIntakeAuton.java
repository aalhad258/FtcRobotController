package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanism.AprilTagLimelight;
import org.firstinspires.ftc.teamcode.mechanism.DcMotor;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Timer;

@Autonomous
public class StartIntakeAuton extends OpMode {
    private Follower follower;
    DcMotor intake = new DcMotor();

    public enum PathState {
        TURN,
        DRIVE,
        TOTAG,
        ALIGN
    }

    private final Pose startPose = new Pose(94, 8, Math.toRadians(90));
    private final Pose nextPose = new Pose(94, 8, Math.toRadians(0));
    private final Pose intakePose = new Pose(132, 8, Math.toRadians(0));
    private final Pose tagPose = new Pose(94, 60, Math.toRadians(180));

    private Timer pathTimer;
    private Pose currentPose;
    private final AprilTagLimelight atlimelight = new AprilTagLimelight();

    PathState pathState;

    private PathChain drivePath, tagPath;

    public void buildPaths() {
        drivePath = follower.pathBuilder()
                .addPath(new BezierLine(nextPose, intakePose))
                .setLinearHeadingInterpolation(nextPose.getHeading(), intakePose.getHeading())
                .build();
        tagPath = follower.pathBuilder()
                .addPath(new BezierLine(intakePose, tagPose))
                .setLinearHeadingInterpolation(intakePose.getHeading(), tagPose.getHeading())
                .build();
    }

    @Override
    public void init() {
        intake.init(hardwareMap, "intake");
        pathState = PathState.TURN;

        atlimelight.init(hardwareMap, 8);

        follower = Constants.createFollower(hardwareMap);

        buildPaths();

        follower.setPose(startPose);

        pathTimer = new Timer();
    }

    @Override
    public void start() {
        atlimelight.start();
    }

    @Override
    public void loop() {

        follower.update();
        atlimelight.update();

        telemetry.addData("state", pathState);
        telemetry.update();

        switch (pathState) {

            case TURN:

                follower.turnTo(Math.toRadians(0));

                double headingError = Math.abs(
                        Math.toDegrees(
                                Math.atan2(
                                        Math.sin(follower.getHeading() - Math.toRadians(0)),
                                        Math.cos(follower.getHeading() - Math.toRadians(0))
                                )
                        )
                );

                if (headingError < 2) {
                    pathState = PathState.DRIVE;
                    follower.followPath(drivePath, true);
                }

                break;

            case DRIVE:

                intake.setMotorSpeed(-1.0);
                if (!follower.isBusy()) {
                    pathState = PathState.TOTAG;
                    follower.followPath(tagPath, true);
                }

                break;

            case TOTAG:
                intake.setMotorSpeed(-1.0);
                if (!follower.isBusy()) {
                    pathState = PathState.ALIGN;
                }

                break;

            case ALIGN:
                if (atlimelight.hasTag(0) && atlimelight.hasValidResult()) {
                    follower.turnTo(follower.getHeading() + Math.toRadians(atlimelight.getTx()));
                }
        }
    }
}
