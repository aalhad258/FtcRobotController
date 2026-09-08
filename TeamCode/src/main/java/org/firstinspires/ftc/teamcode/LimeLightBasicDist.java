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
import org.firstinspires.ftc.teamcode.mechanism.DcMotor;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;

/**
 * ============================================================================
 *  VISION-GUIDED "FIND, DRIVE TO, AND COLLECT" AUTO -- FIELD-SPACE, CONTINUOUS SWEEP
 * ============================================================================
 *
 *  Part of the same 4-file family as LimeLightBasic.java (see that file's
 *  header for the full overview table). THIS FILE is the "field-space
 *  clustering" sibling of LimeLightBasic.java -- same continuous-rotation
 *  sweep strategy (AXIS 2: spin at SWEEP_TURN_POWER, sample every loop,
 *  8-... actually see below, this file doesn't even need the wide angle
 *  dedup tolerance the angle-space version needed, for reasons explained
 *  at the dedup check below), but a fundamentally different representation
 *  and clustering strategy for detections (AXIS 1):
 *
 *  KEY DIFFERENCE FROM LimeLightBasic.java: EVERY detection is converted to
 *  an actual FIELD X/Y COORDINATE (the new FieldBlob type below) THE MOMENT
 *  it's seen, using the same height/camera-angle trig LimeLightBasic.java
 *  only applies once at the very end. That means:
 *    - Deduplication happens by PHYSICAL DISTANCE ON THE FIELD (inches,
 *      DUP_TOLERANCE_IN) instead of by camera bearing angle -- which is why
 *      this file does NOT need the angle-space sibling's widened 8-degree
 *      dedup tolerance for motion-blur compensation: two detections of the
 *      truly-the-same-object will land at nearly the same field X/Y
 *      regardless of exactly which sweep angle/moment they were seen at,
 *      so a modest inch-based tolerance (4.0 in) is enough on its own.
 *    - Clustering (findBestFieldCluster() below) groups by real Euclidean
 *      distance on the field (CLUSTER_RADIUS_IN), which correctly keeps
 *      apart two objects that happen to share a similar bearing from the
 *      robot but are actually far apart physically -- the exact scenario
 *      LimeLightBasic.java's angle-only clustering can get wrong.
 *    - The tradeoff: since distance estimation now feeds directly into
 *      WHERE a detection is considered to be (not just how far to
 *      eventually drive), a bad distance estimate here can misplace a
 *      detection enough to dedupe/cluster it incorrectly -- so the
 *      DISTANCE_MULTIPLIER/DISTANCE_OFFSET_IN calibration below matters
 *      more here than the equivalent single, end-of-pipeline
 *      DISTANCE_OFFSET_IN in LimeLightBasic.java.
 *
 *  Reference docs: see LimeLightBasic.java's header (PedroPathing
 *  Follower.setTeleOpDrive()/startTeleopDrive(), Limelight3A
 *  getTimestamp()) -- all apply identically here.
 *
 *  COMPETITION DEBUGGING CHEAT SHEET (this file specifically):
 *    - Two objects that ARE physically separate get merged into one target
 *      -> check CLUSTER_RADIUS_IN (24 in) isn't too generous for how
 *      closely game objects can legitimately sit next to each other.
 *    - The same physical object gets recorded as two separate blobs ->
 *      check DUP_TOLERANCE_IN (4 in) isn't too tight relative to your
 *      actual distance-estimation error/noise.
 *    - Robot drives to an empty spot on the field -> since detections here
 *      are placed by ESTIMATED distance (not just angle), a systematic
 *      distance-estimation bias will directly relocate every stored
 *      detection, not just the final drive target -- re-tune
 *      DISTANCE_MULTIPLIER/DISTANCE_OFFSET_IN/CAMERA_HEIGHT_IN/
 *      BALL_HEIGHT_IN together before assuming clustering logic is at
 *      fault.
 *    - Robot drives to objects but doesn't collect them -> startIntake()/
 *      stopIntake() are still empty stub methods, same as
 *      LimeLightBasic.java.
 * ============================================================================
 */
@Autonomous
public class LimeLightBasicDist extends OpMode {
    private Limelight3A limelight3A;
    private List<FieldBlob> fieldBlobs;
    private Follower follower;

    // Radius in field inches to group nearby detected objects into a single cluster
    private static final double CLUSTER_RADIUS_IN = 24.0;
    // Minimum physical separation (inches) on the field to register as a separate object
    private static final double DUP_TOLERANCE_IN = 4.0;

    // Weight parameters: Count heavily dominates
    // Note SIZE_WEIGHT here (10.0) is DOUBLE LimeLightBasic.java's (5.0),
    // relative to the same DISTANCE_WEIGHT (0.05) -- i.e. cluster size is
    // weighted even more heavily here (a 200:1 ratio vs. LimeLightBasic's
    // 100:1). These are independently-tunable per file; don't assume they
    // should match across the family.
    private static final double SIZE_WEIGHT = 10.0;
    private static final double DISTANCE_WEIGHT = 0.05;

    // Camera Calibration -- measured values, consistent with LimeLightBasic.java.
    private static final double CAMERA_HEIGHT_IN = 5.5;
    private static final double BALL_HEIGHT_IN = 2.8;
    private static final double CAMERA_MOUNT_ANGLE_DEG = 10.0;

    // Distance Calibration Tweaks
    // Unlike LimeLightBasic.java's single DISTANCE_OFFSET_IN applied once
    // to the final chosen cluster, BOTH of these are applied to EVERY
    // individual detection's distance estimate at collection time (see the
    // calcDist computation in the SWEEP state below) -- because that
    // estimate directly determines the detection's stored field X/Y here.
    private static final double DISTANCE_OFFSET_IN = 5.0; // Flat extra inches added to drive target
    private static final double DISTANCE_MULTIPLIER = 1.05; // Scaling factor for distance calculation

    private static final double CAMERA_OFFSET_FORWARD_IN = 0.0;
    private static final double CAMERA_OFFSET_LEFT_IN = 0.0;

    private final Pose startPose = new Pose(56.0, 8.0, Math.toRadians(90));

    private FieldBlob targetClusterCenter;
    private DcMotor intake = new DcMotor();

    private enum State {
        SWEEP,
        TURN,
        DRIVE,
        PATH_TO_BALL,
        DONE
    }

    private State state;
    private static final double SWEEP_TURN_POWER = 0.2;
    private double lastHeadingDeg;
    private double accumulatedRotationDeg;
    private double lastCaptureTimestamp = 0;
    private boolean turnCommandIssued = false;

    private double targetDistance;
    private double targetHeadingDeg;

    /**
     * Helper class representing a detected object in absolute Field X, Y space.
     *
     * This is the core representational difference from LimeLightBasic.java's
     * Blob class (which stores camera-relative/bearing-space tx/ty/ta). Every
     * FieldBlob's x/y is an ABSOLUTE FIELD COORDINATE (inches), already fully
     * converted from the raw camera reading at the moment it was recorded --
     * see the SWEEP state below for that conversion math.
     */
    public static class FieldBlob {
        public double x;
        public double y;
        public double area;

        public FieldBlob(double x, double y, double area) {
            this.x = x;
            this.y = y;
            this.area = area;
        }
    }

    @Override
    public void init() {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(0);
        fieldBlobs = new ArrayList<>();
        intake.init(hardwareMap, "intake");

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

        state = State.SWEEP;
        resetSweep();

        telemetry.addLine("LimeLightBasic Initialized (Field-Space Deduplication)");
        // --- ADDED TELEMETRY: config snapshot at init, before any motion ---
        telemetry.addData("startPose (x,y,heading deg)", "%.2f, %.2f, %.2f",
                startPose.getX(), startPose.getY(), Math.toDegrees(startPose.getHeading()));
        telemetry.addData("CLUSTER_RADIUS_IN / DUP_TOLERANCE_IN", CLUSTER_RADIUS_IN + " / " + DUP_TOLERANCE_IN);
        telemetry.addData("DISTANCE_MULTIPLIER / DISTANCE_OFFSET_IN", DISTANCE_MULTIPLIER + " / " + DISTANCE_OFFSET_IN);
        telemetry.update();
    }

    @Override
    public void start() {
        limelight3A.start();
        resetRuntime();
        // Switches the Follower into manual/open-loop drive mode so the
        // setTeleOpDrive() calls in SWEEP/DONE below take effect (see the
        // identical pattern and note in LimeLightBasic.java).
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();

        if (state == State.SWEEP) {
            Pose currentPose = follower.getPose();
            double headingDegNow = Math.toDegrees(currentPose.getHeading());
            double deltaDeg = AngleUnit.normalizeDegrees(headingDegNow - lastHeadingDeg);
            accumulatedRotationDeg += deltaDeg;
            lastHeadingDeg = headingDegNow;

            if (Math.abs(accumulatedRotationDeg) < 360.0) {
                follower.setTeleOpDrive(0, 0, SWEEP_TURN_POWER, true);

                LLResult llResult = limelight3A.getLatestResult();
                if (llResult != null && llResult.isValid()) {
                    double currentTimestamp = llResult.getTimestamp();
                    // Same new-frame guard as LimeLightBasic.java -- avoids
                    // reprocessing one camera frame multiple times in a
                    // single fast loop.
                    boolean isNewFrame = (currentTimestamp > lastCaptureTimestamp);

                    if (isNewFrame) {
                        lastCaptureTimestamp = currentTimestamp;
                        List<LLResultTypes.DetectorResult> blobs = llResult.getDetectorResults();

                        for (LLResultTypes.DetectorResult blob : blobs) {
                            double tx = blob.getTargetXDegrees();
                            double ty = blob.getTargetYDegrees();

                            // Calculate raw ground distance to detected blob
                            // -- same height-difference/tan(angle) technique
                            // used everywhere else in this codebase, but
                            // computed PER DETECTION here, immediately, not
                            // deferred until after clustering.
                            double effAngle = Math.max(5, Math.min(85, CAMERA_MOUNT_ANGLE_DEG + ty));
                            double rawDist = (CAMERA_HEIGHT_IN - BALL_HEIGHT_IN) / Math.tan(Math.toRadians(effAngle));
                            double calcDist = (rawDist * DISTANCE_MULTIPLIER) + DISTANCE_OFFSET_IN;

                            // Compute global field heading to the blob
                            // (equivalent to LimeLightBasic.java's `trueX`
                            // calc, just written as a subtraction here
                            // instead of a leading negation -- same result).
                            double blobAngleDeg = AngleUnit.normalizeDegrees(headingDegNow - tx);
                            double blobAngleRad = Math.toRadians(blobAngleDeg);

                            // Calculate candidate Field X and Y
                            double fieldX = currentPose.getX() + calcDist * Math.cos(blobAngleRad);
                            double fieldY = currentPose.getY() + calcDist * Math.sin(blobAngleRad);

                            // Add camera position offsets
                            double robotHeadingRad = currentPose.getHeading();
                            double camX = CAMERA_OFFSET_FORWARD_IN * Math.cos(robotHeadingRad) - CAMERA_OFFSET_LEFT_IN * Math.sin(robotHeadingRad);
                            double camY = CAMERA_OFFSET_FORWARD_IN * Math.sin(robotHeadingRad) + CAMERA_OFFSET_LEFT_IN * Math.cos(robotHeadingRad);
                            fieldX += camX;
                            fieldY += camY;

                            // Deduplicate based on physical distance on field (inches)
                            // -- straight-line (Euclidean) distance between this
                            // candidate's computed field position and every
                            // already-recorded FieldBlob. This is the field-space
                            // equivalent of LimeLightBasic.java's angle+ty dedup
                            // check, but operating in physical inches instead of
                            // camera degrees.
                            boolean duplicate = false;
                            for (FieldBlob existing : fieldBlobs) {
                                double distToExisting = Math.hypot(existing.x - fieldX, existing.y - fieldY);
                                if (distToExisting < DUP_TOLERANCE_IN) {
                                    duplicate = true;
                                    break;
                                }
                            }

                            if (!duplicate) {
                                fieldBlobs.add(new FieldBlob(fieldX, fieldY, blob.getTargetArea()));
                            }
                        }
                    }
                }
            } else {
                follower.setTeleOpDrive(0, 0, 0, true);
                if (fieldBlobs.isEmpty()) {
                    state = State.DONE;
                    // --- ADDED TELEMETRY ---
                    telemetry.addLine("SWEEP complete: no blobs found -> DONE");
                } else {
                    targetClusterCenter = findBestFieldCluster(fieldBlobs);
                    turnCommandIssued = false;
                    state = State.TURN;
                    // --- ADDED TELEMETRY ---
                    telemetry.addLine("SWEEP complete: field cluster selected -> TURN");
                    telemetry.addData("targetClusterCenter (x,y)", "%.2f, %.2f",
                            targetClusterCenter.x, targetClusterCenter.y);
                }
            }
        }

        else if (state == State.TURN) {
            if (targetClusterCenter == null) {
                state = State.DONE;
                return;
            }

            Pose currentPose = follower.getPose();
            // Heading toward the target is computed DIRECTLY from field
            // geometry here (atan2 of the displacement vector), rather
            // than reusing a bearing angle that was already known from the
            // vision pipeline (as LimeLightBasic.java's TURN state does
            // with clusterAngle). This is possible specifically because
            // this file already has an absolute field X/Y target to aim
            // at -- a natural consequence of doing field-space clustering
            // in the first place.
            double dx = targetClusterCenter.x - currentPose.getX();
            double dy = targetClusterCenter.y - currentPose.getY();
            targetDistance = Math.hypot(dx, dy);
            targetHeadingDeg = Math.toDegrees(Math.atan2(dy, dx));

            if (!turnCommandIssued) {
                follower.turnTo(Math.toRadians(targetHeadingDeg));
                turnCommandIssued = true;
            }

            double currentHeadingDeg = Math.toDegrees(currentPose.getHeading());
            double headingError = Math.abs(AngleUnit.normalizeDegrees(currentHeadingDeg - targetHeadingDeg));

            // Same "whichever comes first" completion pattern as
            // LimeLightBasic.java's TURN state -- see that file's comment
            // for why.
            if (!follower.isBusy() || headingError < 3.0) {
                turnCommandIssued = false;
                startIntake();
                state = State.DRIVE;
            }
        }

        else if (state == State.DRIVE) {
            if (targetClusterCenter == null) {
                state = State.DONE;
                return;
            }

            Pose currentPose = follower.getPose();

            // ballPose's heading uses targetHeadingDeg (computed fresh in
            // TURN, above) -- the robot ends the path facing directly at
            // the cluster center, same intent as LimeLightBasic.java's
            // ballPose but derived via atan2 instead of reused bearing.
            Pose ballPose = new Pose(targetClusterCenter.x + 5, targetClusterCenter.y + 5, Math.toRadians(targetHeadingDeg));

            PathChain driveToCluster = follower.pathBuilder()
                    .addPath(new BezierLine(currentPose, ballPose))
                    .setLinearHeadingInterpolation(currentPose.getHeading(), ballPose.getHeading())
                    .build();

            follower.followPath(driveToCluster, true);
            state = State.PATH_TO_BALL;
            // --- ADDED TELEMETRY ---
            telemetry.addData("DRIVE: target ballPose (x,y,heading deg)", "%.2f, %.2f, %.2f",
                    targetClusterCenter.x, targetClusterCenter.y, targetHeadingDeg);
        }

        else if (state == State.PATH_TO_BALL) {
            if (!follower.isBusy()) {
                stopIntake();
                follower.startTeleopDrive();
                resetSweep();
                state = State.SWEEP;
                // --- ADDED TELEMETRY ---
                telemetry.addLine("PATH_TO_BALL complete: arrived, restarting SWEEP for next object");
            }
        }

        else if (state == State.DONE) {
            follower.setTeleOpDrive(0, 0, 0, true);
        }

        telemetry.addData("State", state.toString());
        telemetry.addData("Blobs Found", fieldBlobs.size());
        telemetry.addData("Target Heading", targetHeadingDeg);
        telemetry.addData("Target Distance (in)", targetDistance);
        // --- ADDED TELEMETRY ---
        telemetry.addData("accumulatedRotationDeg (SWEEP progress)", accumulatedRotationDeg);
        telemetry.addData("follower busy?", follower.isBusy());
        telemetry.addData("loop runtime (s)", getRuntime());
        telemetry.update();
    }

    /**
     * Finds the cluster with the highest density/score using physical field coordinates.
     *
     * GREEDY CENTROID APPROACH: for every detected blob (used as a
     * candidate cluster CENTER), counts how many blobs (including itself)
     * fall within CLUSTER_RADIUS_IN of it, and averages THEIR positions
     * into a candidate centroid. The candidate whose neighborhood scores
     * best (by the same SIZE_WEIGHT/DISTANCE_WEIGHT formula used
     * throughout this family) wins, and the function returns a NEW
     * FieldBlob at that neighborhood's AVERAGE position -- i.e. the
     * "center of mass" of the winning group, not necessarily the position
     * of any single actual detection. This means the robot ends up
     * targeting the middle of a pile of objects rather than any one
     * specific member of it.
     *
     * COMPLEXITY NOTE: this is an O(n^2) all-pairs comparison (every blob
     * is tried as a candidate center, and for each candidate every OTHER
     * blob is checked against it). Perfectly fine for the small number of
     * detections a vision sweep typically produces, but worth knowing if
     * this is ever adapted for a much larger detection count.
     *
     * NOTE: follower.getPose() is called fresh INSIDE the outer loop, once
     * per candidate -- so it's a live odometry read repeated N times per
     * call to this method, not a single snapshot reused throughout. For
     * the typically-small N here this executes essentially instantaneously
     * and the robot isn't moving during this synchronous computation, so
     * in practice all N reads should agree -- but it's worth knowing this
     * isn't cached the way currentPose is cached-and-reused elsewhere in
     * this file's loop() (e.g. within the SWEEP state above).
     */
    private FieldBlob findBestFieldCluster(List<FieldBlob> blobs) {
        if (blobs.isEmpty()) return null;

        FieldBlob bestCenter = blobs.get(0);
        double maxScore = Double.NEGATIVE_INFINITY;

        for (FieldBlob centerCandidate : blobs) {
            int count = 0;
            double sumX = 0;
            double sumY = 0;

            for (FieldBlob other : blobs) {
                double dist = Math.hypot(other.x - centerCandidate.x, other.y - centerCandidate.y);
                if (dist <= CLUSTER_RADIUS_IN) {
                    count++;
                    sumX += other.x;
                    sumY += other.y;
                }
            }

            Pose currentPose = follower.getPose();
            double avgX = sumX / count;
            double avgY = sumY / count;
            double distFromRobot = Math.hypot(avgX - currentPose.getX(), avgY - currentPose.getY());

            double score = (SIZE_WEIGHT * count) - (DISTANCE_WEIGHT * distFromRobot);

            if (score > maxScore) {
                maxScore = score;
                bestCenter = new FieldBlob(avgX, avgY, 0);
            }
        }

        return bestCenter;
    }

    /**
     * Clears sweep state so the next SWEEP phase starts clean -- called
     * once from init() and again every time PATH_TO_BALL completes (this
     * OpMode loops to collect multiple objects per autonomous period).
     */
    private void resetSweep() {
        fieldBlobs = new ArrayList<>();
        lastHeadingDeg = Math.toDegrees(follower.getPose().getHeading());
        accumulatedRotationDeg = 0;
        lastCaptureTimestamp = 0;
        targetClusterCenter = null;
    }

    // --- STUB METHODS: no physical intake mechanism wired up yet -- see the
    // identical note in LimeLightBasic.java. ---
    private void startIntake() {
        intake.setMotorSpeed(-1.0);
    }

    private void stopIntake() {
        intake.setMotorSpeed(0.0);
    }
}