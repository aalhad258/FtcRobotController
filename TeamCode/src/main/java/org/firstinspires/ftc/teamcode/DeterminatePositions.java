package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.mechanism.DcMotor;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class DeterminatePositions extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    DcMotor intake = new DcMotor();

    public enum PathState {
        DRIVE_S_PATH,
        DONE
    }

    PathState pathState;

    // S PATH:
    // (0,0) → (5,10) → (10,15) → (5,20) → (0,25)

    private final Pose startPose =
            new Pose(0, 0, Math.toRadians(90));

    private final Pose point1 =
            new Pose(20, 20, Math.toRadians(90));

    private final Pose point2 =
            new Pose(40, 40, Math.toRadians(90));

    private final Pose point3 =
            new Pose(20, 20, Math.toRadians(90));

    private final Pose endPose =
            new Pose(0, 0, Math.toRadians(90));

    private PathChain sPath;

    public void buildPaths() {

        sPath = follower.pathBuilder()

                // First curve: (0,0) → (5,10) → (10,15)
                .addPath(new BezierCurve(
                        startPose,
                        point1,
                        point2
                ))

                // Second curve: (10,15) → (5,20) → (0,25)
                .addPath(new BezierCurve(
                        point2,
                        point3,
                        endPose
                ))

                // Keep heading facing forward/up the field
                .setConstantHeadingInterpolation(
                        Math.toRadians(90)
                )

                .build();
    }

    public void statePathUpdate() {
        switch(pathState) {

            case DRIVE_S_PATH:

                follower.followPath(sPath, true);

                setPathState(PathState.DONE);
                break;

            case DONE:

                if (!follower.isBusy()) {
                    telemetry.addLine("S Path Complete!");
                }

                break;

            default:

                telemetry.addLine("No State Commanded");
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        intake.init(hardwareMap, "intake");

        pathState = PathState.DRIVE_S_PATH;

        pathTimer = new Timer();
        opModeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);

        buildPaths();

        follower.setPose(startPose);

        telemetry.addLine("S Path Auto Ready");

        telemetry.addData(
                "Start",
                "(%.1f, %.1f)",
                startPose.getX(),
                startPose.getY()
        );

        telemetry.addData(
                "End",
                "(%.1f, %.1f)",
                endPose.getX(),
                endPose.getY()
        );

        telemetry.update();
    }

    @Override
    public void start() {

        opModeTimer.resetTimer();

        setPathState(PathState.DRIVE_S_PATH);
    }

    @Override
    public void loop() {
        intake.setMotorSpeed(1.0);

        follower.update();

        statePathUpdate();

        telemetry.addData(
                "Path State",
                pathState.toString()
        );

        telemetry.addData(
                "X",
                follower.getPose().getX()
        );

        telemetry.addData(
                "Y",
                follower.getPose().getY()
        );

        telemetry.addData(
                "Heading",
                Math.toDegrees(
                        follower.getPose().getHeading()
                )
        );

        telemetry.addData(
                "Busy?",
                follower.isBusy()
        );

        telemetry.update();
    }
}