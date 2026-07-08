package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class SampleAutoPathing extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;
    public enum PathState {
        // START POSITION_END POSITION
        // DRIVE > MOVEMENT STATE
        // SCORE > ATTEMPT TO SCORE THE POLLEN
        DRIVE_STARTPOS_SCORE_POS,
        SCORE_PRELOAD,
        DRIVE_SCOREPOS_ENDPOS
    }

    PathState pathState;
    private final Pose startPose = new Pose(56.0, 8.0, Math.toRadians(90));
    private final Pose scorePose = new Pose(70, 36, Math.toRadians(90));
    private final Pose endPose = new Pose(52.61595163563719, 52.27124208861158, Math.toRadians(180));
    private PathChain driveStartPosScorePos, driveScorePosEndPos;

    public void buildPaths() {
        // put in coordinates for starting pose > ending pose
        driveStartPosScorePos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
        driveScorePosEndPos = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, endPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch(pathState) {
            case DRIVE_STARTPOS_SCORE_POS:
                follower.followPath(driveStartPosScorePos, true);
                setPathState(PathState.SCORE_PRELOAD);
                break;
            case SCORE_PRELOAD:
                // check if follower done its path
                // and check 5 seconds has elapsed
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    follower.followPath(driveScorePosEndPos, true);
                    setPathState(PathState.DRIVE_SCOREPOS_ENDPOS);
                }
                break;
            case DRIVE_SCOREPOS_ENDPOS:
                // all done!
                if (!follower.isBusy()) {
                    telemetry.addLine("Done path 1");
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
        pathState = PathState.DRIVE_STARTPOS_SCORE_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        // TO DO: add in any other mechanisms like limelight

        buildPaths();
        follower.setPose(startPose);
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("path time", pathTimer.getElapsedTimeSeconds());
    }
}
