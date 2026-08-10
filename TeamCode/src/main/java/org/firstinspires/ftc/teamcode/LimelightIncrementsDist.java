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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;

/**
 * ============================================================================
 *  VISION-GUIDED "FIND, DRIVE TO, AND COLLECT" AUTO -- FIELD-SPACE, STEPPED SWEEP
 * ============================================================================
 *
 *  The fourth and final member of this file family (see LimeLightBasic.java's
 *  header for the full overview table). THIS FILE combines:
 *    - FIELD-SPACE clustering (shared with LimeLightBasicDist.java, AXIS 1):
 *      every detection is immediately converted to an absolute field X/Y
 *      (the FieldBlob type below) and deduplicated/clustered by physical
 *      distance on the field, not camera bearing angle. See
 *      LimeLightBasicDist.java's header for the full explanation of why
 *      this avoids the angle-space clustering blind spot (objects at
 *      similar bearing but very different distance being wrongly merged).
 *    - DISCRETE STEPPED sweep (shared with LimelightIncrements.java, AXIS 2):
 *      12 counted ROTATE -> SETTLE(150ms) -> SAMPLE cycles at 30-degree
 *      increments, rather than one continuous spin. See
 *      LimelightIncrements.java's header for the full explanation of the
 *      deterministic-termination and cleaner-sample-quality tradeoffs this
 *      brings relative to the continuous-sweep "Basic" siblings.
 *
 *  This file is, in a sense, the "most conservative/most robust" member of
 *  the family: field-space clustering avoids the angle-only clustering
 *  blind spot, AND stepped sweeping avoids the timing-dependent sweep
 *  termination and motion-blur concerns of continuous sweeping -- at the
 *  cost of being the SLOWEST of the four (full stop/settle/sample cycles,
 *  12 times, per object collected).
 *
 *  !!! CAMERA CALIBRATION -- KEPT IN SYNC WITH THE REST OF THE FAMILY !!!
 *  CAMERA_HEIGHT_IN / BALL_HEIGHT_IN below are set to 5.5 / 2.8, matching
 *  LimeLightBasic.java / LimeLightBasicDist.java / LimelightIncrements.java
 *  (this file previously used different, older placeholder numbers -- now
 *  brought in line with its siblings).
 *
 *  Reference docs: see LimeLightBasic.java's header (PedroPathing,
 *  Limelight3A) and LimelightIncrements.java's header (ElapsedTime) -- all
 *  apply identically here.
 *
 *  COMPETITION DEBUGGING CHEAT SHEET (this file specifically):
 *    - Slowest of the four files to complete a sweep -> expected by design,
 *      see above; not a bug.
 *    - Two physically separate objects merged, or the same object recorded
 *      twice -> tune CLUSTER_RADIUS_IN / DUP_TOLERANCE_IN respectively (see
 *      LimeLightBasicDist.java's cheat sheet -- identical concern here).
 *    - Robot drives to objects but doesn't collect them -> startIntake()/
 *      stopIntake() are still empty stub methods, same as every other file
 *      in this family.
 * ============================================================================
 */
@Autonomous
public class LimelightIncrementsDist extends OpMode {
    private Limelight3A limelight3A;
    private List<FieldBlob> fieldBlobs;
    private Follower follower;

    // Field Clustering & Deduplication Parameters
    private static final double CLUSTER_RADIUS_IN = 24.0;
    private static final double DUP_TOLERANCE_IN = 4.0; // Deduplicate if within 4 inches on field

    private static final double SIZE_WEIGHT = 10.0;
    private static final double DISTANCE_WEIGHT = 0.05;

    // Camera Calibration -- updated to match the rest of the family (was
    // previously 8.0 / 2.0).
    private static final double CAMERA_HEIGHT_IN = 5.5;
    private static final double BALL_HEIGHT_IN = 2.8;
    private static final double CAMERA_MOUNT_ANGLE_DEG = 10.0;

    // Distance Calibration Tweaks
    // Same per-detection correction approach as LimeLightBasicDist.java
    // (applied at collection time, not just once at the end) -- see that
    // file's header for why this matters more here than in the angle-space
    // siblings.
    private static final double DISTANCE_OFFSET_IN = 5.0; // Flat extra inches added to reach target
    private static final double DISTANCE_MULTIPLIER = 1.05; // Scaling factor for perspective skew

    private static final double CAMERA_OFFSET_FORWARD_IN = 0.0;
    private static final double CAMERA_OFFSET_LEFT_IN = 0.0;

    private final Pose startPose = new Pose(56.0, 8.0, Math.toRadians(90));
    private FieldBlob targetClusterCenter;

    private enum State {
        ROTATE,
        SETTLE,
        SAMPLE,
        TURN,
        DRIVE,
        PATH_TO_BALL,
        DONE
    }

    private State state;
    private double targetAngle;
    private double clusterAngleRad;
    private double targetHeadingDeg;
    private double targetDistance;
    private boolean turnCommandIssued = false;

    // Guaranteed 360-degree sweep termination counter
    private int sweepStepCount = 0;
    private static final int TOTAL_SWEEP_STEPS = 12; // 12 steps * 30 deg = 360 deg

    private ElapsedTime stateTimer = new ElapsedTime();

    /**
     * Helper class representing a detected object in absolute Field X, Y space.
     * Identical role to LimeLightBasicDist.java's FieldBlob -- see that
     * file's javadoc for the full explanation.
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

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

        state = State.ROTATE;
        sweepStepCount = 0;

        double startHeadingDeg = Math.toDegrees(startPose.getHeading());
        targetAngle = AngleUnit.normalizeDegrees(startHeadingDeg + 30);

        telemetry.addLine("LimelightIncrementsDist Initialized (Field-Space Coordinates)");
        // --- ADDED TELEMETRY: config snapshot before any motion ---
        telemetry.addData("startPose (x,y,heading deg)", "%.2f, %.2f, %.2f",
                startPose.getX(), startPose.getY(), Math.toDegrees(startPose.getHeading()));
        telemetry.addData("CLUSTER_RADIUS_IN / DUP_TOLERANCE_IN", CLUSTER_RADIUS_IN + " / " + DUP_TOLERANCE_IN);
        telemetry.addData("CAMERA_HEIGHT_IN / BALL_HEIGHT_IN", CAMERA_HEIGHT_IN + " / " + BALL_HEIGHT_IN);
        telemetry.addData("TOTAL_SWEEP_STEPS", TOTAL_SWEEP_STEPS);
        telemetry.update();
    }

    @Override
    public void start() {
        limelight3A.start();
        resetRuntime();
    }

    @Override
    public void loop() {
        follower.update();
        Pose currentPose = follower.getPose();
        double headingDeg = Math.toDegrees(currentPose.getHeading());

        switch (state) {
            case ROTATE:
                if (!turnCommandIssued) {
                    follower.turnTo(Math.toRadians(targetAngle));
                    turnCommandIssued = true;
                }

                double headingError = Math.abs(AngleUnit.normalizeDegrees(headingDeg - targetAngle));

                if (!follower.isBusy() || headingError < 3.0) {
                    turnCommandIssued = false;
                    stateTimer.reset();
                    state = State.SETTLE;
                    // --- ADDED TELEMETRY ---
                    telemetry.addData("ROTATE complete via", !follower.isBusy() ? "follower not busy" : "heading within 3deg");
                }
                break;

            case SETTLE:
                // Robot is completely stopped here, ensuring crisp vision snapshot
                if (stateTimer.milliseconds() > 150) {
                    state = State.SAMPLE;
                }
                break;

            case SAMPLE:
                LLResult llResult = limelight3A.getLatestResult();
                // --- ADDED TELEMETRY ---
                telemetry.addData("SAMPLE: LL result valid?", llResult != null && llResult.isValid());
                if (llResult != null && llResult.isValid()) {
                    List<LLResultTypes.DetectorResult> blobs = llResult.getDetectorResults();

                    for (LLResultTypes.DetectorResult blob : blobs) {
                        double tx = blob.getTargetXDegrees();
                        double ty = blob.getTargetYDegrees();

                        // 1. Calculate raw ground distance and apply distance multipliers
                        double effAngle = Math.max(5, Math.min(85, CAMERA_MOUNT_ANGLE_DEG + ty));
                        double rawDist = (CAMERA_HEIGHT_IN - BALL_HEIGHT_IN) / Math.tan(Math.toRadians(effAngle));
                        double calcDist = (rawDist * DISTANCE_MULTIPLIER) + DISTANCE_OFFSET_IN;

                        // 2. Convert tx angle + robot heading into global field heading
                        double blobAngleDeg = AngleUnit.normalizeDegrees(headingDeg - tx);
                        double blobAngleRad = Math.toRadians(blobAngleDeg);

                        // 3. Project to exact Field X and Y
                        double fieldX = currentPose.getX() + calcDist * Math.cos(blobAngleRad);
                        double fieldY = currentPose.getY() + calcDist * Math.sin(blobAngleRad);

                        double robotHeadingRad = currentPose.getHeading();
                        double camX = CAMERA_OFFSET_FORWARD_IN * Math.cos(robotHeadingRad) - CAMERA_OFFSET_LEFT_IN * Math.sin(robotHeadingRad);
                        double camY = CAMERA_OFFSET_FORWARD_IN * Math.sin(robotHeadingRad) + CAMERA_OFFSET_LEFT_IN * Math.cos(robotHeadingRad);
                        fieldX += camX;
                        fieldY += camY;

                        // 4. Deduplicate using actual carpet distance (inches)
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

                sweepStepCount++;

                if (sweepStepCount >= TOTAL_SWEEP_STEPS) {
                    if (fieldBlobs.isEmpty()) {
                        telemetry.addLine("No blobs found after 360 sweep.");
                        state = State.DONE;
                    } else {
                        targetClusterCenter = findBestFieldCluster(fieldBlobs);
                        turnCommandIssued = false;
                        state = State.TURN;
                        // --- ADDED TELEMETRY ---
                        telemetry.addLine("Sweep complete (12/12 steps): field cluster selected -> TURN");
                        telemetry.addData("targetClusterCenter (x,y)", "%.2f, %.2f",
                                targetClusterCenter.x, targetClusterCenter.y);
                    }
                } else {
                    targetAngle = AngleUnit.normalizeDegrees(targetAngle + 30);
                    state = State.ROTATE;
                }
                break;

            case TURN:
                if (targetClusterCenter == null) {
                    state = State.DONE;
                    break;
                }

                // Vector from current robot position to cluster center
                // -- heading computed directly via atan2 from field
                // geometry, same approach as LimeLightBasicDist.java's TURN
                // state (see that file's comment for why this is natural
                // once you already have an absolute field-space target).
                double dx = targetClusterCenter.x - currentPose.getX();
                double dy = targetClusterCenter.y - currentPose.getY();
                targetDistance = Math.hypot(dx, dy);
                targetHeadingDeg = Math.toDegrees(Math.atan2(dy, dx));
                clusterAngleRad = Math.toRadians(targetHeadingDeg);

                if (!turnCommandIssued) {
                    follower.turnTo(clusterAngleRad);
                    turnCommandIssued = true;
                }

                headingError = Math.abs(AngleUnit.normalizeDegrees(headingDeg - targetHeadingDeg));

                if (!follower.isBusy() || headingError < 3.0) {
                    turnCommandIssued = false;
                    startIntake();
                    state = State.DRIVE;
                }
                break;

            case DRIVE:
                if (targetClusterCenter == null) {
                    state = State.DONE;
                    break;
                }

                Pose ballPose = new Pose(targetClusterCenter.x, targetClusterCenter.y, clusterAngleRad);
                PathChain driveToCluster = follower.pathBuilder()
                        .addPath(new BezierLine(currentPose, ballPose))
                        .setLinearHeadingInterpolation(currentPose.getHeading(), ballPose.getHeading())
                        .build();

                follower.followPath(driveToCluster, true);
                state = State.PATH_TO_BALL;
                // --- ADDED TELEMETRY ---
                telemetry.addData("DRIVE: target ballPose (x,y,heading deg)", "%.2f, %.2f, %.2f",
                        targetClusterCenter.x, targetClusterCenter.y, Math.toDegrees(clusterAngleRad));
                break;

            case PATH_TO_BALL:
                if (!follower.isBusy()) {
                    stopIntake();
                    resetSweep();
                    state = State.ROTATE;
                    // --- ADDED TELEMETRY ---
                    telemetry.addLine("PATH_TO_BALL complete: arrived, restarting sweep for next object");
                }
                break;

            case DONE:
                break;
        }

        telemetry.addData("State", state);
        telemetry.addData("Sweep Steps", sweepStepCount + " / " + TOTAL_SWEEP_STEPS);
        telemetry.addData("Blobs Found", fieldBlobs.size());
        telemetry.addData("Target Heading", targetHeadingDeg);
        telemetry.addData("Target Distance (in)", targetDistance);
        // --- ADDED TELEMETRY ---
        telemetry.addData("follower busy?", follower.isBusy());
        telemetry.addData("loop runtime (s)", getRuntime());
        telemetry.update();
    }

    /**
     * Resets sweep bookkeeping for the next 360-degree pass. See
     * LimelightIncrements.java's identical note about targetAngle not being
     * reset here -- ROTATE re-evaluates turnCommandIssued fresh regardless.
     */
    public void resetSweep() {
        fieldBlobs = new ArrayList<>();
        sweepStepCount = 0;
        targetClusterCenter = null;
    }

    /**
     * Evaluates clusters in 2D field-space using a physical radius on the mat.
     * Identical greedy-centroid technique to LimeLightBasicDist.java's
     * findBestFieldCluster() -- see that file's javadoc for the full
     * explanation (O(n^2) all-pairs comparison, returns the winning
     * neighborhood's AVERAGE position rather than any single detection's
     * position, and reads follower.getPose() fresh once per candidate).
     * Reproduced here rather than shared -- see LimelightIncrements.java's
     * note about the angle-space clustering logic being duplicated across
     * files for the same reason.
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

    // --- STUB METHODS: no physical intake mechanism wired up yet -- see the
    // identical note in every other file in this family. ---
    private void startIntake() {

    }

    private void stopIntake() {

    }
}