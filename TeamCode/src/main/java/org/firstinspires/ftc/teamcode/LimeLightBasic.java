package org.firstinspires.ftc.teamcode;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechanism.Blob;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


import java.util.ArrayList;
import java.util.List;


/**
 * ============================================================================
 *  LIMELIGHT-GUIDED "FIND AND DRIVE TO OBJECT" AUTONOMOUS
 * ============================================================================
 *
 *  High-level behavior: the robot spins in place in 30-degree increments,
 *  and at each stop asks the Limelight3A (running its COLOR/BLOB detection
 *  pipeline, index 0 -- NOT the AprilTag pipeline used elsewhere in this
 *  codebase, see AprilTagLimelight.java which uses pipeline 8) whether it
 *  sees anything. Every detected "blob" (a colored object, e.g. a ball) is
 *  converted from a camera-relative angle into an absolute FIELD-frame
 *  bearing and stored. After a full 360-degree sweep, the robot picks the
 *  best cluster of nearby blobs (i.e. the densest/nearest pile of objects),
 *  estimates its distance using camera-angle trigonometry, and drives to it
 *  using a PedroPathing BezierLine path.
 *
 *  Reference docs:
 *   - PedroPathing Follower / PathChain / BezierLine:
 *     https://pedropathing.com/docs/
 *   - Limelight3A FTC integration (getLatestResult, DetectorResult,
 *     pipelineSwitch, start()):
 *     https://docs.limelightvision.io/docs/docs-limelight/getting-started/ftc
 *   - Distance-from-camera-angle estimation technique used in the DRIVE
 *     state below (height-difference / tan(angle)):
 *     https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance
 *
 *  STATE MACHINE OVERVIEW (see the State enum below):
 *    ROTATE       -> turn the robot to the next sweep angle
 *    SAMPLE       -> take one Limelight reading at this angle, record blobs
 *    DRIVE        -> (after full sweep) compute the best cluster's field
 *                    position and build/start a path to it
 *    PATH_TO_BALL -> wait for the PedroPathing follower to finish that path
 *    DONE         -> sit still (follower holds last commanded pose)
 *
 *  COMPETITION DEBUGGING CHEAT SHEET:
 *    - Robot spins forever / never finds anything -> check limelight
 *      pipeline index (0) is actually configured as a color/blob pipeline
 *      in the Limelight web UI, and that blobResults is actually
 *      populating (see the new telemetry added below in SAMPLE).
 *    - Robot finds objects but drives to totally the wrong spot -> suspect
 *      the CAMERA_* calibration constants below (height/angle/offsets) are
 *      unmeasured placeholders (see TODOs) rather than actual robot
 *      measurements.
 *    - Robot turns the wrong way while sweeping, or the reported bearing
 *      to a blob is mirrored -> suspect the tx sign-flip in SAMPLE
 *      (`-blob.getTargetXDegrees()`), which assumes a specific camera
 *      mounting orientation.
 *    - Robot never leaves ROTATE state -> follower.turnTo()/isBusy() not
 *      resolving; check turnCommandIssued guard logic below.
 * ============================================================================
 */
@Autonomous
public class LimeLightBasic extends OpMode {
    private Limelight3A limelight3A;
    private List<Blob> blobResults;


    private Follower follower;


    // Sliding-window width (degrees) used by findBestCluster() to decide
    // which detected blobs count as "part of the same pile". Two blobs more
    // than this many degrees apart (in absolute field bearing) are never
    // grouped into the same cluster.
    private static final double CLUSTER_WINDOW_DEG = 20.0;
    // Cluster scoring weights (see findBestCluster()): a cluster's score is
    // SIZE_WEIGHT * (number of blobs in it) - DISTANCE_WEIGHT * (avg distance).
    // With these values, "more blobs clustered together" dominates the
    // decision (weight 1.0) and "closer is slightly better among similar-size
    // clusters" is a tiebreaker (weight 0.1). Raise DISTANCE_WEIGHT if the
    // robot keeps choosing far-away piles over closer smaller ones.
    private static final double SIZE_WEIGHT = 1.0;
    private static final double DISTANCE_WEIGHT = 0.1;


    // ---- Camera calibration ----
    // NOTE: these three constants feed directly into the distance formula in
    // the DRIVE state: distance = (CAMERA_HEIGHT_IN - BALL_HEIGHT_IN) / tan(angle).
    // This is the standard fixed-mount-angle vision ranging technique (see
    // the Limelight "Estimating Distance" doc linked above).
    private static final double CAMERA_HEIGHT_IN = 5.5;
    private static final double BALL_HEIGHT_IN = 2.8;
    private static final double CAMERA_MOUNT_ANGLE_DEG = 10.0;


    // ---- Camera's position offset from the robot's center of rotation, robot-local frame ----
    // Used in the DRIVE state to correct the computed target point for the
    // fact the camera is not physically at the robot's tracked center point.
    // Also still
    private static final double CAMERA_OFFSET_FORWARD_IN = 0.0;
    private static final double CAMERA_OFFSET_LEFT_IN = 0.0;


    // ---- Where the robot starts, in field coordinates (match your actual autonomous start) ----
    // CRITICAL: this MUST match the robot's real physical starting position
    // and heading on the field at the start of the match, in the same field
    // coordinate system used everywhere else in this codebase (see
    // SampleAutoPathing.java for the same startPose pattern). If this is
    // wrong, every absolute bearing/position computed in this file will be
    // offset by the same error, even though the robot's relative behavior
    // (sweep, cluster, approach) will still "look" correct in telemetry.
    private final Pose startPose = new Pose(56.0, 8.0, Math.toRadians(90)); // TODO: match real start pose


    // The set of blobs (from findBestCluster()) belonging to the winning
    // cluster. Populated only once, at the end of the sweep, then read in
    // the DRIVE state to compute where to drive.
    private List<Blob> bestCluster;


    private enum State {
        SWEEP,
        DRIVE,
        PATH_TO_BALL,
        DONE
    }


    private State state;
    // New fields, replacing targetAngle / turnCommandIssued
    private static final double SWEEP_TURN_POWER = 0.2; // tune this — start slow, verify direction
    private double lastHeadingDeg;
    private double accumulatedRotationDeg;


    private double lastStaleness = -1;


    // Absolute field-frame heading (degrees) we're currently sweeping toward / driving toward.
    // Since everything is now referenced off follower.getPose().getHeading(), this is a
    // FIELD-frame angle, not a robot-relative one.
    private double clusterAngle;
    private double distance;


    // Guards so we only issue a turnTo()/followPath() command ONCE per state entry,
    // instead of re-issuing it every loop() call (which would restart the motion).
    // This matters because PedroPathing's turnTo()/followPath() calls START a new
    // motion command each time they're invoked -- calling them every loop while
    // still mid-turn would keep resetting the turn and it would never finish.


    @Override
    public void init() {
        // Pipeline 0 on the Limelight is configured (in the Limelight web UI,
        // NOT in this code) as a color/blob-detection pipeline for this
        // OpMode. Contrast with AprilTagLimelight.java, which switches the
        // same physical camera to pipeline 8 for AprilTag detection -- if
        // both OpModes have run in the same session, double check the
        // Limelight is actually on pipeline 0 before trusting blob data here.
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(0);
        blobResults = new ArrayList<>();


        // Builds the shared, globally-tuned PedroPathing Follower (motor
        // wiring, PIDF gains, odometry config -- see Constants.java) for
        // this OpMode's autonomous run.
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);


        // Sweep starts at the robot's current field heading + 30, not a raw "30 degrees".
        state = State.SWEEP;
        resetSweep();
        lastHeadingDeg = Math.toDegrees(startPose.getHeading());
        accumulatedRotationDeg = 0;


        // --- ADDED TELEMETRY: confirm init completed and show starting config ---
        telemetry.addLine("LimeLightBasic: init complete");
        telemetry.addData("startPose (x,y,heading deg)", "%.2f, %.2f, %.2f",
                startPose.getX(), startPose.getY(), Math.toDegrees(startPose.getHeading()));
        telemetry.update();
    }


    @Override
    public void start() {
        // Limelight3A requires an explicit start() call to begin actively
        // streaming/processing pipeline results -- without this,
        // getLatestResult() in SAMPLE will keep returning null/invalid data
        // even though init()/pipelineSwitch() succeeded.
        limelight3A.start();
        resetRuntime();
        follower.startTeleopDrive();
    }


    @Override
    public void loop() {
        // Pedro needs this called every loop no matter what state we're in,
        // or its localization/path-following stalls.
        follower.update();


        double headingDeg = Math.toDegrees(follower.getPose().getHeading());


        if (state == State.SWEEP) {
            double headingDegNow = Math.toDegrees(follower.getPose().getHeading());
            // wrapped delta — this is the fix for the ±180 crossing bug, applied per-step
            // instead of to one big unbounded angle
            double deltaDeg = AngleUnit.normalizeDegrees(headingDegNow - lastHeadingDeg);
            accumulatedRotationDeg += deltaDeg;
            lastHeadingDeg = headingDegNow;

            if ((Math.abs(accumulatedRotationDeg) < 360)) {
                follower.setTeleOpDrive(0, 0, SWEEP_TURN_POWER, true);


                // --- sampling, same logic as your old SAMPLE state, just running every loop now ---
                // --- sampling, same logic as your old SAMPLE state, just running every loop now ---
                LLResult llResult = limelight3A.getLatestResult();
                if (llResult != null && llResult.isValid()) {
                    long staleness = llResult.getStaleness();
                    boolean isNewFrame = (lastStaleness < 0) || (staleness < lastStaleness);
                    lastStaleness = staleness;

                    if (isNewFrame) {
                        List<LLResultTypes.DetectorResult> blobs = llResult.getDetectorResults();
                        for (LLResultTypes.DetectorResult blob : blobs) {
                            boolean blobThere = false;
                            double trueX = AngleUnit.normalizeDegrees(-blob.getTargetXDegrees() + headingDegNow);
                            for (Blob blobResult : blobResults) {
                                double angleDiff = AngleUnit.normalizeDegrees(blobResult.tx - trueX);
                                if (Math.abs(angleDiff) < 3 && Math.abs(blobResult.ty - blob.getTargetYDegrees()) < 3) {
                                    // Same physical object as an existing entry — refine
                                    // its stored position instead of discarding this
                                    // reading, so repeated noisy samples of one object
                                    // converge instead of silently doing nothing.
                                    blobResult.tx = AngleUnit.normalizeDegrees(
                                            (blobResult.tx + trueX) / 2.0);
                                    blobResult.ty = (blobResult.ty + blob.getTargetYDegrees()) / 2.0;
                                    blobThere = true;
                                    break;
                                }
                            }
                            if (!blobThere) {
                                blobResults.add(new Blob(trueX, blob.getTargetYDegrees(), blob.getTargetArea()));
                            }
                        }
                    }
                }
                telemetry.addData("SWEEP: accumulated rotation (deg)", accumulatedRotationDeg);
                telemetry.addData("SWEEP: total unique blobs so far", blobResults.size());


            } else {
                follower.setTeleOpDrive(0, 0, 0, true); // stop turning
                if (blobResults.isEmpty()) {
                    telemetry.addLine("SWEEP COMPLETE: no blobs detected anywhere -> going to DONE");
                    state = State.DONE;
                } else {
                    clusterAngle = findBestCluster(blobResults);
                    telemetry.addLine("SWEEP COMPLETE: best cluster selected");
                    state = State.DRIVE;
                }
            }
        }


        else if (state == State.DRIVE) {
            double sumTy = 0;
            for (Blob b : bestCluster) {
                sumTy += b.ty;
            }
            double avgTy = sumTy / bestCluster.size();
            double effAngle = CAMERA_MOUNT_ANGLE_DEG + avgTy;
            effAngle = Math.max(5, Math.min(85, effAngle)); // keep tan() well-behaved
            double rawDistance = (CAMERA_HEIGHT_IN - BALL_HEIGHT_IN) / Math.tan(Math.toRadians(effAngle));
            distance = Math.max(6, Math.min(rawDistance, 96)); // sane min/max for field, tune these


            Pose currentPose = follower.getPose();
            double robotX = currentPose.getX();
            double robotY = currentPose.getY();
            double robotHeadingRad = currentPose.getHeading();


            // clusterAngle is ALREADY an absolute field bearing (camera angle + heading was
            // baked in back in SAMPLE), so we project distance directly along it, no second
            // rotation by robotHeading here, unlike the earlier version.
            // DESIGN NOTE: this projects from the robot's CURRENT pose (at DRIVE-state
            // entry), not its pose at the moment the winning samples were actually taken.
            // Since DRIVE runs immediately after the sweep finishes and the robot isn't
            // translating during ROTATE/SAMPLE (only turning, which doesn't move the
            // tracked center much), this approximation is normally fine -- but if targeting
            // is off at competition, this is a spot to double check.
            double clusterAngleRad = Math.toRadians(clusterAngle);
            double fieldX = robotX + distance * Math.cos(clusterAngleRad);
            double fieldY = robotY + distance * Math.sin(clusterAngleRad);


            // The camera mount offset IS robot-relative, so this one still needs exactly
            // ONE rotation by the robot's current heading to convert into field frame.
            double cameraFieldOffsetX = CAMERA_OFFSET_FORWARD_IN * Math.cos(robotHeadingRad) - CAMERA_OFFSET_LEFT_IN * Math.sin(robotHeadingRad);
            double cameraFieldOffsetY = CAMERA_OFFSET_FORWARD_IN * Math.sin(robotHeadingRad) + CAMERA_OFFSET_LEFT_IN * Math.cos(robotHeadingRad);
            fieldX += cameraFieldOffsetX;
            fieldY += cameraFieldOffsetY;

            startIntake();

            follower.turnTo(clusterAngleRad);
            Pose ballPose = new Pose(fieldX, fieldY, clusterAngleRad);
            PathChain driveToCluster = follower.pathBuilder()
                    .addPath(new BezierLine(currentPose, ballPose))
                    .setLinearHeadingInterpolation(currentPose.getHeading(), ballPose.getHeading())
                    .build();


            // The trailing `true` here follows PedroPathing's
            // followPath(PathChain, boolean holdEnd) convention: once the
            // path finishes, the follower keeps actively correcting to stay
            // at the path's final pose (closed-loop hold) instead of just
            // cutting power and coasting/drifting.
            follower.followPath(driveToCluster, true);
            state = State.PATH_TO_BALL;


            // --- ADDED TELEMETRY: full breakdown of the distance/target math, so a
            // bad drive-to-target can be root-caused from the driver station log alone ---
            telemetry.addLine("DRIVE: computed target from best cluster");
            telemetry.addData("DRIVE: avgTy (deg)", avgTy);
            telemetry.addData("DRIVE: effAngle used for tan() (deg, clamped 5-85)", effAngle);
            telemetry.addData("DRIVE: rawDistance before clamp (in)", rawDistance);
            telemetry.addData("DRIVE: distance used (in, clamped 6-96)", distance);
            telemetry.addData("DRIVE: robot pose at plan time (x,y,heading deg)", "%.2f, %.2f, %.2f",
                    robotX, robotY, Math.toDegrees(robotHeadingRad));
            telemetry.addData("DRIVE: camera field offset (x,y in)", "%.2f, %.2f",
                    cameraFieldOffsetX, cameraFieldOffsetY);
            telemetry.addData("DRIVE: target ballPose (x,y,heading deg)", "%.2f, %.2f, %.2f",
                    fieldX, fieldY, clusterAngle);
        }


        else if (state == State.PATH_TO_BALL) {
            if (!follower.isBusy()) {
                resetSweep();
                state = State.SWEEP;
                // --- ADDED TELEMETRY: mark arrival explicitly ---
                telemetry.addLine("PATH_TO_BALL: follower reports path complete -> DONE");
            }
        }


        else if (state == State.DONE) {
            // Follower holds position automatically once idle.
        }


        telemetry.addData("state", state.toString());
        telemetry.addData("clusterAngle", clusterAngle);
        telemetry.addData("distance", distance);
        telemetry.addData("robot pose", follower.getPose().toString());
        telemetry.addData("blobs ", blobResults.size());
        for (Blob b: blobResults) {
            telemetry.addData("Blob tx", b.tx);
            telemetry.addData("Blob ty", b.ty);
            telemetry.addData("Blob ta", b.ta);
        }
        // --- ADDED TELEMETRY: loop timing, useful for spotting a loop that's running
        // slower than expected (e.g. due to Limelight polling), which can make PID
        // derivative terms elsewhere in the codebase misbehave ---
        telemetry.addData("loop: runtime (s)", getRuntime());
        telemetry.update();
    }


    /**
     * Groups the given blobs into clusters of nearby field-bearings using a
     * sliding-window (two-pointer) technique, scores each candidate cluster,
     * and returns the CIRCULAR MEAN bearing of the best-scoring cluster. Also
     * populates the `bestCluster` field with the actual Blob objects in that
     * winning window, for use by the DRIVE state (distance estimation).
     *
     * WHY A CIRCULAR MEAN (sumSin/sumCos/atan2) INSTEAD OF A PLAIN AVERAGE:
     * these are absolute field bearings (0-360, wrapping). A plain arithmetic
     * mean of, say, 350 degrees and 10 degrees would incorrectly give 180
     * degrees; the circular mean correctly gives 0 degrees. This matters
     * because clusters can legitimately straddle the 0/360 wrap boundary.
     *
     * WHY THE ANGLE LIST IS DUPLICATED WITH +360 APPENDED: to let the
     * sliding window naturally consider windows that cross the 0/360
     * boundary without special-casing modulo arithmetic -- the second half
     * of the list is just the first half shifted by a full revolution, so a
     * window like [350, 10] appears as a contiguous, sorted [350, 370] in
     * the doubled list.
     *
     * @param blobResults all uniquely-detected blobs from the sweep
     * @return the field-frame bearing (degrees) of the best cluster's
     *         circular-mean angle
     */
    public double findBestCluster(List<Blob> blobResults) {
        List<Blob> sorted = new ArrayList<>(blobResults);
        sorted.sort((a, b) -> Double.compare(a.tx, b.tx));
        int n = sorted.size();


        List<Double> angles = new ArrayList<>();
        for (Blob b : sorted) angles.add(b.tx);
        for (Blob b : sorted) angles.add(b.tx + 360.0);


        int windowStart = 0;
        double bestScore = Double.NEGATIVE_INFINITY;
        int bestStart = 0;
        int bestEnd = 0;


        for (int windowEnd = 0; windowEnd < angles.size(); windowEnd++) {
            // Shrink the window from the left until it's within CLUSTER_WINDOW_DEG wide.
            while (angles.get(windowEnd) - angles.get(windowStart) > CLUSTER_WINDOW_DEG) {
                windowStart++;
            }
            // Skip windows that are entirely inside the "duplicated" second half of the
            // list (windowStart >= n means we've wrapped past all original, non-duplicated
            // entries) to avoid double-scoring the same physical cluster twice.
            if (windowStart >= n) continue;


            int count = windowEnd - windowStart + 1;


            double sumTy = 0;
            for (int i = windowStart; i <= windowEnd; i++) {
                sumTy += sorted.get(i % n).ty;
            }
            double avgTy = sumTy / count;
            double avgDistance = (CAMERA_HEIGHT_IN - BALL_HEIGHT_IN)
                    / Math.tan(Math.toRadians(CAMERA_MOUNT_ANGLE_DEG + avgTy));


            // Bigger clusters score higher (SIZE_WEIGHT dominates); among
            // similar-size clusters, closer ones score slightly higher
            // (DISTANCE_WEIGHT is a tiebreaker, not a primary factor).
            double score = SIZE_WEIGHT * count - DISTANCE_WEIGHT * avgDistance;


            if (score > bestScore) {
                bestScore = score;
                bestStart = windowStart;
                bestEnd = windowEnd;
            }
        }


        bestCluster = new ArrayList<>();
        double sumSin = 0, sumCos = 0;
        for (int i = bestStart; i <= bestEnd; i++) {
            Blob b = sorted.get(i % n);
            bestCluster.add(b);
            double rad = Math.toRadians(angles.get(i));
            sumSin += Math.sin(rad);
            sumCos += Math.cos(rad);
        }
        return AngleUnit.normalizeDegrees(Math.toDegrees(Math.atan2(sumSin, sumCos)));
    }
    private void resetSweep() {
        stopIntake();
        blobResults = new ArrayList<>();
        lastHeadingDeg = Math.toDegrees(follower.getPose().getHeading());
        accumulatedRotationDeg = 0;
        lastStaleness = -1;
    }

    private void startIntake() {

    }

    private void stopIntake() {

    }
}

