package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * ============================================================================
 *  SAMPLE / TEMPLATE PEDROPATHING STATE-MACHINE AUTO
 * ============================================================================
 *
 *  This is the base "3 path, state machine" pattern used throughout this
 *  codebase for autonomous path following (compare CustomPathing.java's
 *  simpler single-segment PID version, and LimeLightBasic.java's
 *  vision-driven version, which both build on this same idea: an enum of
 *  named PathStates, a switch statement that starts the next path and
 *  advances the enum, and a Timer to gate time-based waits). Treat this
 *  file as the reference template: the naming convention for PathState
 *  values here (`DRIVE_<START>_<END>`, `SCORE_<WHAT>`) is intended to be
 *  copied when building out a real match auto with more than 3 paths.
 *
 *  !!! COMPETITION-CRITICAL FLAG !!!
 *  This OpMode is annotated @TeleOp, NOT @Autonomous. That means it will
 *  show up in the Driver Station's TeleOp OpMode list, not the Autonomous
 *  list. If this is meant to be run during the autonomous period of a
 *  match, it currently WON'T be selectable there. This was
 *  left as @TeleOp for bench-testing convenience (so a driver can trigger
 *  it manually without the 30-second auto period timeout), but it is worth
 *  double-checking before relying on this pattern in a real match auto.
 *
 *  Reference docs:
 *   - PedroPathing Follower / PathBuilder / PathChain / BezierLine:
 *     https://pedropathing.com/docs/
 *   - PedroPathing Timer utility (com.pedropathing.util.Timer): a simple
 *     elapsed-time-in-seconds stopwatch, reset via resetTimer() and read via
 *     getElapsedTimeSeconds() -- used below both for the overall OpMode
 *     runtime (opModeTimer) and for timing how long we've been in the
 *     current PathState (pathTimer).
 * ============================================================================
 */
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

    // Hardcoded field-coordinate waypoints (inches, PedroPathing field
    // frame). These are SAMPLE/PLACEHOLDER values for this template file --
    // for a real match auto these must be measured/tuned against the actual
    // game field for the current season, the same way startPose is handled
    // in LimeLightBasic.java and CustomPathing.java.
    private final Pose startPose = new Pose(56.0, 8.0, Math.toRadians(90));
    private final Pose scorePose = new Pose(70, 36, Math.toRadians(90));
    private final Pose endPose = new Pose(52.61595163563719, 52.27124208861158, Math.toRadians(180));
    private PathChain driveStartPosScorePos, driveScorePosEndPos;

    /**
     * Pre-builds every PathChain this auto will use, once, during init().
     * PedroPathing PathChains are relatively expensive to construct
     * (geometry + curvature calculations), so building them all up front in
     * init() -- rather than lazily inside statePathUpdate() -- avoids doing
     * that work mid-match where it could cause a loop-time hiccup right as
     * a path is about to start.
     */
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

    /**
     * The core state machine. Runs every loop(); each case either (a) fires
     * off the next path/action and immediately advances pathState so this
     * case's "start" logic doesn't re-fire next loop, or (b) checks a
     * completion condition and advances pathState only once that condition
     * is true. This is the same start-once-then-advance pattern used in
     * CustomPathing.java's TURN/MOVE states and LimeLightBasic.java's
     * ROTATE/SAMPLE/DRIVE/PATH_TO_BALL states.
     */
    public void statePathUpdate() {
        switch(pathState) {
            case DRIVE_STARTPOS_SCORE_POS:
                // Fires immediately on the first loop() call (pathState is
                // initialized to this value in init()). followPath(..., true)
                // starts the path and enables PedroPathing's holdEnd
                // behavior so the robot actively corrects to stay at
                // scorePose once it arrives, rather than drifting off it.
                follower.followPath(driveStartPosScorePos, true);
                setPathState(PathState.SCORE_PRELOAD);
                break;
            case SCORE_PRELOAD:
                // check if follower done its path
                // and check 5 seconds has elapsed
                // NOTE: there is no actual scoring-mechanism call here (no
                // servo/motor action) -- this state is purely a placeholder
                // wait. In a real match auto this is where you'd trigger
                // whatever mechanism scores the preload (e.g. a
                // ServoClass.setServoPos() / setServoRot() call), and the
                // 5-second wait would likely be replaced or shortened once
                // that mechanism's actual timing is known.
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    follower.followPath(driveScorePosEndPos, true);
                    setPathState(PathState.DRIVE_SCOREPOS_ENDPOS);
                }
                break;
            case DRIVE_SCOREPOS_ENDPOS:
                // all done!
                // Terminal state: once the follower reports it's no longer
                // busy, we just log completion and otherwise do nothing --
                // the Follower's holdEnd behavior (from followPath(..., true))
                // keeps the robot actively parked at endPose for the
                // remainder of the autonomous period.
                if (!follower.isBusy()) {
                    telemetry.addLine("Done path 1");
                }
                break;
            default:
                // Defensive fallback -- should only be reachable if pathState
                // somehow ends up null/uninitialized, since every declared
                // enum value is handled above.
                telemetry.addLine("No State Commanded");
                break;
        }
    }

    /**
     * Transitions to a new PathState and resets pathTimer so that any
     * time-based check in the next state (e.g. SCORE_PRELOAD's 5-second
     * wait) is measured from the moment we ENTERED that state, not from
     * OpMode start.
     */
    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathState = PathState.DRIVE_STARTPOS_SCORE_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        // Builds the shared, globally-tuned PedroPathing Follower (motor
        // wiring, PIDF gains, odometry config -- see Constants.java).
        follower = Constants.createFollower(hardwareMap);
        // TO DO: add in any other mechanisms like limelight

        buildPaths();
        follower.setPose(startPose);

        // --- ADDED TELEMETRY: confirm init completed and show the plan ---
        telemetry.addLine("SampleAutoPathing: init complete");
        telemetry.addData("startPose (x,y,heading deg)", "%.2f, %.2f, %.2f",
                startPose.getX(), startPose.getY(), Math.toDegrees(startPose.getHeading()));
        telemetry.addData("scorePose (x,y,heading deg)", "%.2f, %.2f, %.2f",
                scorePose.getX(), scorePose.getY(), Math.toDegrees(scorePose.getHeading()));
        telemetry.addData("endPose (x,y,heading deg)", "%.2f, %.2f, %.2f",
                endPose.getX(), endPose.getY(), Math.toDegrees(endPose.getHeading()));
        telemetry.addData("initial pathState", pathState);
        telemetry.update();
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        // Re-invokes setPathState with the SAME state that init() already
        // set, purely so pathTimer is reset at the moment the driver
        // presses "start" rather than at the moment init() ran (which can
        // be much earlier, e.g. while sitting on the field waiting for the
        // match to begin).
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

        // --- ADDED TELEMETRY: extra fields useful for diagnosing a stuck/incomplete
        // auto from the driver station log after the fact ---
        telemetry.addData("opMode total time (s)", opModeTimer.getElapsedTimeSeconds());
        telemetry.addData("follower busy?", follower.isBusy());
        telemetry.addData("heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }
}