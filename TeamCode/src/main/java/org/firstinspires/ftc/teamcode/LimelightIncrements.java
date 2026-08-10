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
import org.firstinspires.ftc.teamcode.mechanism.Blob;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;

/**
 * ============================================================================
 *  VISION-GUIDED "FIND, DRIVE TO, AND COLLECT" AUTO -- ANGLE-SPACE, STEPPED SWEEP
 * ============================================================================
 *
 *  Part of the same 4-file family as LimeLightBasic.java / LimeLightBasicDist.java
 *  (see LimeLightBasic.java's header for the full overview table). THIS FILE
 *  is the "angle-space clustering" sibling of LimelightIncrementsDist.java,
 *  and the "discrete stepped sweep" sibling of LimeLightBasic.java:
 *
 *  CLUSTERING (shared with LimeLightBasic.java, AXIS 1): detections are
 *  stored as Blob { tx = absolute field bearing, ty = camera vertical
 *  angle, ta = target area } and grouped by a sliding ANGLE window
 *  (findBestCluster() below) -- see LimeLightBasic.java's header for the
 *  geometric blind spot this shares (objects at similar bearing but very
 *  different distance can be merged).
 *
 *  SWEEP STRATEGY (AXIS 2 -- this is what makes this file different from
 *  LimeLightBasic.java): instead of one continuous spin, this file performs
 *  12 DISCRETE steps of ROTATE -> SETTLE -> SAMPLE:
 *    ROTATE  - turnTo() the next 30-degree increment (same idea as the very
 *              first version of this vision auto, before the continuous-
 *              sweep "Basic" variants existed)
 *    SETTLE  - a fixed 150ms pause (via the added `stateTimer`,
 *              com.qualcomm.robotcore.util.ElapsedTime) AFTER the turn
 *              reports complete, before trusting the camera -- gives any
 *              residual chassis vibration/oscillation time to die down so
 *              the Limelight sees a genuinely stationary scene
 *    SAMPLE  - take exactly ONE Limelight reading at this now-settled angle
 *  This is repeated exactly TOTAL_SWEEP_STEPS (12) times -- 12 * 30 degrees
 *  = a full 360-degree sweep, counted with a plain int (sweepStepCount)
 *  rather than accumulated from float heading deltas. That makes sweep
 *  TERMINATION deterministic and exact regardless of loop timing/odometry
 *  noise -- contrast with LimeLightBasic.java's accumulatedRotationDeg
 *  float check, which is timing-dependent. The cost is speed: 12 full
 *  rotate+settle+sample cycles inherently take longer than one continuous
 *  spin.
 *
 *  Because sampling only ever happens while FULLY STOPPED (thanks to
 *  SETTLE), the per-detection dedup angle tolerance below only needs to be
 *  5 degrees -- tighter than LimeLightBasic.java's 8-degree tolerance,
 *  which exists specifically to cover motion blur from sampling while
 *  still spinning. This is concrete in-code evidence of the stepped vs.
 *  continuous sweep tradeoff described in LimeLightBasic.java's header.
 *
 *  !!! CAMERA CALIBRATION -- KEPT IN SYNC WITH THE REST OF THE FAMILY !!!
 *  CAMERA_HEIGHT_IN / BALL_HEIGHT_IN below are set to 5.5 / 2.8, matching
 *  LimeLightBasic.java and LimeLightBasicDist.java's measured values (this
 *  file previously used different, older placeholder numbers -- it's now
 *  been brought in line with its siblings).
 *
 *  Note this file also has NO equivalent of LimeLightBasic.java's
 *  DISTANCE_OFFSET_IN -- the DRIVE state below uses the raw computed
 *  distance directly (just clamped to [6, 96] inches), with no additive
 *  correction term. If the robot consistently falls short of/overshoots
 *  targets specifically when running THIS file (but not LimeLightBasic.java),
 *  that missing offset term is the first place to look.
 *
 *  Reference docs: see LimeLightBasic.java's header for PedroPathing /
 *  Limelight3A references, which apply equally here. Additionally:
 *   - FTC SDK ElapsedTime (com.qualcomm.robotcore.util.ElapsedTime): a
 *     simple millisecond-precision stopwatch, used here purely for the
 *     SETTLE state's fixed delay -- reset() starts it, milliseconds()
 *     reads elapsed time since the last reset().
 *
 *  COMPETITION DEBUGGING CHEAT SHEET (this file specifically):
 *    - Sweep takes noticeably longer than the "Basic" variants -> expected
 *      by design (12 stop/settle/go cycles vs. one continuous spin); this
 *      is the speed-for-data-quality tradeoff described above, not a bug.
 *    - Robot consistently stops short of / overshoots targets -> check for
 *      the MISSING DISTANCE_OFFSET_IN noted above before assuming
 *      CAMERA_HEIGHT_IN/BALL_HEIGHT_IN calibration is wrong.
 *    - Sweep exits early / late -> shouldn't happen here (sweepStepCount is
 *      a deterministic int counter), which is precisely the robustness
 *      advantage this file has over LimeLightBasic.java's float-based
 *      accumulatedRotationDeg check -- if you DO see early/late exit here,
 *      suspect TOTAL_SWEEP_STEPS or the ROTATE increment (30 degrees)
 *      being edited inconsistently with each other, not sweep timing.
 * ============================================================================
 */
@Autonomous
public class LimelightIncrements extends OpMode {
    private Limelight3A limelight3A;
    private List<Blob> blobResults;
    private Follower follower;

    private static final double CLUSTER_WINDOW_DEG = 25.0;
    private static final double SIZE_WEIGHT = 5.0;
    private static final double DISTANCE_WEIGHT = 0.05;

    // Camera calibration -- updated to match LimeLightBasic.java /
    // LimeLightBasicDist.java's measured values (was previously 8.0 / 2.0).
    private static final double CAMERA_HEIGHT_IN = 5.5;
    private static final double BALL_HEIGHT_IN = 2.8;
    private static final double CAMERA_MOUNT_ANGLE_DEG = 10.0;

    private static final double CAMERA_OFFSET_FORWARD_IN = 0.0;
    private static final double CAMERA_OFFSET_LEFT_IN = 0.0;

    private final Pose startPose = new Pose(56.0, 8.0, Math.toRadians(90));
    private List<Blob> bestCluster = new ArrayList<>();

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
    private double clusterAngle;
    private double clusterAngleRad;
    private double distance;
    private boolean turnCommandIssued = false;

    // Guaranteed 360-degree sweep termination counter
    // Deterministic sweep-completion tracking: exactly 12 ROTATE/SETTLE/
    // SAMPLE cycles at 30 degrees each = 360 degrees, regardless of any
    // per-step timing variance. See the class-level note above for why this
    // is more robust than LimeLightBasic.java's accumulated-heading-delta
    // approach.
    private int sweepStepCount = 0;
    private static final int TOTAL_SWEEP_STEPS = 12; // 12 steps * 30 deg = 360 deg

    // Simple stopwatch used only by the SETTLE state below to enforce a
    // fixed post-turn pause before sampling.
    private ElapsedTime stateTimer = new ElapsedTime();

    @Override
    public void init() {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(0);
        blobResults = new ArrayList<>();

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

        state = State.ROTATE;
        sweepStepCount = 0;

        double startHeadingDeg = Math.toDegrees(startPose.getHeading());
        targetAngle = AngleUnit.normalizeDegrees(startHeadingDeg + 30);

        telemetry.addLine("LimelightIncrements initialized");
        // --- ADDED TELEMETRY: config snapshot before any motion ---
        telemetry.addData("startPose (x,y,heading deg)", "%.2f, %.2f, %.2f",
                startPose.getX(), startPose.getY(), Math.toDegrees(startPose.getHeading()));
        telemetry.addData("CAMERA_HEIGHT_IN / BALL_HEIGHT_IN", CAMERA_HEIGHT_IN + " / " + BALL_HEIGHT_IN);
        telemetry.addData("TOTAL_SWEEP_STEPS", TOTAL_SWEEP_STEPS);
        telemetry.update();
    }

    @Override
    public void start() {
        // Limelight3A requires an explicit start() call to begin actively
        // streaming/processing pipeline results.
        limelight3A.start();
        resetRuntime();
    }

    @Override
    public void loop() {
        follower.update();
        double headingDeg = Math.toDegrees(follower.getPose().getHeading());

        switch (state) {
            case ROTATE:
                if (!turnCommandIssued) {
                    follower.turnTo(Math.toRadians(targetAngle));
                    turnCommandIssued = true;
                }

                // Calculate angular distance remaining to target
                double headingError = Math.abs(AngleUnit.normalizeDegrees(headingDeg - targetAngle));

                // Transition if follower finished, OR within 3 deg tolerance
                // (same "whichever comes first" pattern used in the TURN
                // states of the "Basic" siblings -- see LimeLightBasic.java's
                // note on why).
                if (!follower.isBusy() || headingError < 3.0) {
                    turnCommandIssued = false;
                    stateTimer.reset();
                    state = State.SETTLE;
                    // --- ADDED TELEMETRY ---
                    telemetry.addData("ROTATE complete via", !follower.isBusy() ? "follower not busy" : "heading within 3deg");
                }
                break;

            case SETTLE:
                // Pure fixed-delay wait -- gives residual vibration/momentum
                // from the ROTATE step time to die down before trusting the
                // camera. This state does not touch drivetrain power at all
                // (the robot should already be stopped from ROTATE's
                // completion); it purely burns 150ms of wall-clock time.
                if (stateTimer.milliseconds() > 150) {
                    state = State.SAMPLE;
                }
                break;

            case SAMPLE:
                LLResult llResult = limelight3A.getLatestResult();
                // --- ADDED TELEMETRY: surface whether this sample had usable data,
                // matching the diagnostic added to the original single-shot
                // LimeLightBasic.java's SAMPLE state ---
                telemetry.addData("SAMPLE: LL result valid?", llResult != null && llResult.isValid());
                if (llResult != null && llResult.isValid()) {
                    List<LLResultTypes.DetectorResult> blobs = llResult.getDetectorResults();
                    for (LLResultTypes.DetectorResult blob : blobs) {
                        double trueX = AngleUnit.normalizeDegrees(-blob.getTargetXDegrees() + headingDeg);

                        boolean duplicate = false;
                        for (Blob existing : blobResults) {
                            double angleDiff = AngleUnit.normalizeDegrees(existing.tx - trueX);
                            // 5-degree tolerance -- tighter than LimeLightBasic.java's
                            // 8-degree tolerance, since sampling here only ever happens
                            // while fully stopped (post-SETTLE), so there's far less
                            // motion blur/latency to compensate for.
                            if (Math.abs(angleDiff) < 5.0 && Math.abs(existing.ty - blob.getTargetYDegrees()) < 5.0) {
                                duplicate = true;
                                break;
                            }
                        }
                        if (!duplicate) {
                            blobResults.add(new Blob(trueX, blob.getTargetYDegrees(), blob.getTargetArea()));
                        }
                    }
                }

                sweepStepCount++;

                // Stop after exactly 12 increments (360 degrees)
                if (sweepStepCount >= TOTAL_SWEEP_STEPS) {
                    if (blobResults.isEmpty()) {
                        telemetry.addLine("No blobs found after 360 sweep.");
                        state = State.DONE;
                    } else {
                        clusterAngle = findBestCluster(blobResults);
                        clusterAngleRad = Math.toRadians(clusterAngle);
                        turnCommandIssued = false;
                        state = State.TURN;
                        // --- ADDED TELEMETRY ---
                        telemetry.addLine("Sweep complete (12/12 steps): cluster selected -> TURN");
                        telemetry.addData("chosen clusterAngle (deg)", clusterAngle);
                    }
                } else {
                    targetAngle = AngleUnit.normalizeDegrees(targetAngle + 30);
                    state = State.ROTATE;
                }
                break;

            case TURN:
                if (!turnCommandIssued) {
                    follower.turnTo(clusterAngleRad);
                    turnCommandIssued = true;
                }

                // Calculate angular distance remaining to target
                headingError = Math.abs(AngleUnit.normalizeDegrees(headingDeg - clusterAngle));

                if (!follower.isBusy() || headingError < 3.0) {
                    turnCommandIssued = false;
                    startIntake();
                    state = State.DRIVE;
                }
                break;

            case DRIVE:
                if (bestCluster.isEmpty()) {
                    state = State.DONE;
                    break;
                }

                double sumTy = 0;
                for (Blob b : bestCluster) {
                    sumTy += b.ty;
                }
                double avgTy = sumTy / bestCluster.size();

                double effAngle = Math.max(5, Math.min(85, CAMERA_MOUNT_ANGLE_DEG + avgTy));
                double rawDistance = (CAMERA_HEIGHT_IN - BALL_HEIGHT_IN) / Math.tan(Math.toRadians(effAngle));
                // NOTE: no additive DISTANCE_OFFSET_IN correction here,
                // unlike LimeLightBasic.java's DRIVE state -- see the
                // class-level note above.
                distance = Math.max(6, Math.min(rawDistance, 96));

                Pose currentPose = follower.getPose();

                double fieldX = currentPose.getX() + distance * Math.cos(clusterAngleRad);
                double fieldY = currentPose.getY() + distance * Math.sin(clusterAngleRad);

                double robotHeadingRad = currentPose.getHeading();
                double cameraFieldOffsetX = CAMERA_OFFSET_FORWARD_IN * Math.cos(robotHeadingRad) - CAMERA_OFFSET_LEFT_IN * Math.sin(robotHeadingRad);
                double cameraFieldOffsetY = CAMERA_OFFSET_FORWARD_IN * Math.sin(robotHeadingRad) + CAMERA_OFFSET_LEFT_IN * Math.cos(robotHeadingRad);
                fieldX += cameraFieldOffsetX;
                fieldY += cameraFieldOffsetY;

                Pose ballPose = new Pose(fieldX, fieldY, clusterAngleRad);
                PathChain driveToCluster = follower.pathBuilder()
                        .addPath(new BezierLine(currentPose, ballPose))
                        .setLinearHeadingInterpolation(currentPose.getHeading(), ballPose.getHeading())
                        .build();

                follower.followPath(driveToCluster, true);
                state = State.PATH_TO_BALL;
                // --- ADDED TELEMETRY ---
                telemetry.addData("DRIVE: avgTy (deg)", avgTy);
                telemetry.addData("DRIVE: rawDistance (no offset applied, in)", rawDistance);
                telemetry.addData("DRIVE: target ballPose (x,y,heading deg)", "%.2f, %.2f, %.2f",
                        fieldX, fieldY, clusterAngle);
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
        telemetry.addData("Blobs Found", blobResults.size());
        telemetry.addData("Target Cluster Angle", clusterAngle);
        telemetry.addData("Target Distance (in)", distance);
        // --- ADDED TELEMETRY ---
        telemetry.addData("follower busy?", follower.isBusy());
        telemetry.addData("loop runtime (s)", getRuntime());
        telemetry.update();
    }

    /**
     * Resets sweep bookkeeping for the next 360-degree pass -- note this
     * does NOT reset `targetAngle` back to a fresh +30-from-current value
     * the way init() does; PATH_TO_BALL's transition back to ROTATE reuses
     * whatever targetAngle/turnCommandIssued state was left over from the
     * end of the previous sweep pass, which is fine since ROTATE always
     * re-evaluates turnCommandIssued and issues a fresh turnTo() as needed.
     */
    public void resetSweep() {
        blobResults = new ArrayList<>();
        sweepStepCount = 0;
    }

    /**
     * Angle-space sliding-window clustering -- identical technique to
     * LimeLightBasic.java's findBestCluster() (see that file's javadoc for
     * the full explanation of the AngleBlob normalization, the doubled-list
     * wraparound trick, and the circular-mean return value). Reproduced
     * here rather than shared as a common utility -- if clustering behavior
     * is ever changed, remember it needs to be changed in BOTH this file
     * and LimeLightBasic.java (and, for the field-space equivalent, in BOTH
     * LimeLightBasicDist.java and LimelightIncrementsDist.java) since none
     * of the four currently share this logic through a common class.
     */
    public double findBestCluster(List<Blob> blobs) {
        if (blobs.isEmpty()) return 0;

        class AngleBlob {
            Blob blob;
            double posTx;
            AngleBlob(Blob b) {
                this.blob = b;
                this.posTx = (b.tx % 360 + 360) % 360;
            }
        }

        List<AngleBlob> list = new ArrayList<>();
        for (Blob b : blobs) {
            list.add(new AngleBlob(b));
        }
        list.sort((a, b) -> Double.compare(a.posTx, b.posTx));

        int n = list.size();
        List<AngleBlob> doubled = new ArrayList<>(list);
        for (AngleBlob ab : list) {
            AngleBlob wrap = new AngleBlob(ab.blob);
            wrap.posTx = ab.posTx + 360.0;
            doubled.add(wrap);
        }

        int windowStart = 0;
        double bestScore = Double.NEGATIVE_INFINITY;
        int bestStart = 0;
        int bestEnd = 0;

        for (int windowEnd = 0; windowEnd < doubled.size(); windowEnd++) {
            while (doubled.get(windowEnd).posTx - doubled.get(windowStart).posTx > CLUSTER_WINDOW_DEG) {
                windowStart++;
            }
            if (windowStart >= n) continue;

            int count = windowEnd - windowStart + 1;
            double sumTy = 0;
            for (int i = windowStart; i <= windowEnd; i++) {
                sumTy += doubled.get(i).blob.ty;
            }
            double avgTy = sumTy / count;
            double effAngle = Math.max(5, Math.min(85, CAMERA_MOUNT_ANGLE_DEG + avgTy));
            double avgDist = (CAMERA_HEIGHT_IN - BALL_HEIGHT_IN) / Math.tan(Math.toRadians(effAngle));

            double score = (SIZE_WEIGHT * count) - (DISTANCE_WEIGHT * avgDist);

            if (score > bestScore) {
                bestScore = score;
                bestStart = windowStart;
                bestEnd = windowEnd;
            }
        }

        bestCluster = new ArrayList<>();
        double sumSin = 0, sumCos = 0;
        for (int i = bestStart; i <= bestEnd; i++) {
            Blob b = doubled.get(i).blob;
            bestCluster.add(b);
            double rad = Math.toRadians(b.tx);
            sumSin += Math.sin(rad);
            sumCos += Math.cos(rad);
        }

        return AngleUnit.normalizeDegrees(Math.toDegrees(Math.atan2(sumSin, sumCos)));
    }

    // --- STUB METHODS: no physical intake mechanism wired up yet -- see the
    // identical note in LimeLightBasic.java / LimeLightBasicDist.java. ---
    private void startIntake() {

    }

    private void stopIntake() {

    }
}