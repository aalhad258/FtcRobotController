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
import org.firstinspires.ftc.teamcode.mechanism.DcMotor;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;

/**
 * ============================================================================
 *  VISION-GUIDED "FIND, DRIVE TO, AND COLLECT" AUTO -- ANGLE-SPACE, CONTINUOUS SWEEP
 * ============================================================================
 *
 *  !!! THIS FILE IS PART OF A 4-FILE FAMILY -- READ THIS OVERVIEW FIRST !!!
 *  This codebase contains FOUR closely related "spin, look for objects with
 *  the Limelight, drive to the best cluster, repeat" autonomous OpModes:
 *
 *      1. LimeLightBasic          (THIS FILE)  -- angle-space clustering,
 *                                                  continuous rotation sweep
 *      2. LimeLightBasicDist                    -- FIELD-space clustering,
 *                                                  continuous rotation sweep
 *      3. LimelightIncrements                   -- angle-space clustering,
 *                                                  discrete stop-and-sample sweep
 *      4. LimelightIncrementsDist               -- FIELD-space clustering,
 *                                                  discrete stop-and-sample sweep
 *
 *  All four solve the same problem and share the same overall shape (a
 *  SWEEP/scan phase to build a list of detections, a clustering step to pick
 *  the best pile of objects, a TURN + DRIVE to approach it, then a re-sweep
 *  to look for more), but differ along two independent axes:
 *
 *  AXIS 1 -- HOW DETECTIONS ARE CLUSTERED ("Basic"/"Increments" vs. "...Dist"):
 *    - THIS FILE (angle-space, like LimelightIncrements): stores each
 *      detection as a Blob { tx = absolute FIELD BEARING in degrees, ty =
 *      raw camera vertical angle, ta = target area } and groups detections
 *      that are close together IN BEARING ANGLE (see findBestCluster()'s
 *      sliding degree-window below). This is simple and cheap, but has a
 *      real geometric blind spot: two physically separate objects that
 *      happen to sit at the SAME bearing from the robot but at very
 *      DIFFERENT DISTANCES (e.g. one right in front of the robot, another
 *      far away directly behind it from the robot's rotating perspective)
 *      can be merged into the same "cluster" purely because their angles
 *      are close, even though they aren't actually near each other on the
 *      field.
 *    - The "...Dist" siblings (LimeLightBasicDist, LimelightIncrementsDist)
 *      instead convert every detection into an actual FIELD X/Y COORDINATE
 *      immediately (using the same height/angle trig this file uses, just
 *      applied per-detection instead of only at the end) and cluster by
 *      real EUCLIDEAN DISTANCE on the field (a physical inch radius). This
 *      is geometrically correct in a way angle-space clustering isn't, at
 *      the cost of being more sensitive to distance-estimation error (since
 *      a bad distance estimate now directly corrupts the detection's
 *      stored X/Y, and therefore which cluster it gets grouped into --
 *      whereas in THIS file, distance is only computed once, AFTER
 *      clustering, purely to decide how far to drive).
 *
 *  AXIS 2 -- HOW THE SWEEP IS PERFORMED ("Basic" vs. "Increments"):
 *    - THIS FILE (continuous, like LimeLightBasicDist): spins continuously
 *      at a fixed power (SWEEP_TURN_POWER) via follower.setTeleOpDrive(),
 *      tracking total rotation via accumulated heading deltas each loop,
 *      and samples the Limelight EVERY LOOP while still moving (gated by a
 *      result timestamp check so the same camera frame isn't processed
 *      twice). Faster (one continuous motion instead of 12 stop/settle/go
 *      cycles), but samples are taken while the robot is IN MOTION, so the
 *      per-detection dedup angle tolerance below is deliberately widened to
 *      8 degrees specifically to compensate for motion blur/latency (see
 *      the inline comment at that check) -- contrast with the Increments
 *      siblings' 5-degree tolerance, which they can afford because they
 *      only ever sample while fully stopped.
 *    - The "Increments" siblings instead do 12 discrete 30-degree
 *      ROTATE -> SETTLE(150ms) -> SAMPLE steps (TOTAL_SWEEP_STEPS-counted,
 *      guaranteeing exact sweep termination regardless of loop timing --
 *      contrast with this file's accumulatedRotationDeg float check, which
 *      is timing-dependent and could in principle over/undershoot 360 by
 *      whatever the last loop's rotation delta happened to be). Slower
 *      overall, but each sample is taken from a fully stopped, "settled"
 *      robot, giving cleaner vision data per sample.
 *
 *  Reference docs:
 *   - PedroPathing Follower.setTeleOpDrive() / startTeleopDrive() (manual,
 *     open-loop-style driving through the Follower, as opposed to
 *     followPath()): https://pedropathing.com/docs/
 *   - Limelight3A LLResult.getTimestamp() (used below to detect a genuinely
 *     NEW camera frame vs. re-reading the same frame twice in one fast
 *     loop): https://docs.limelightvision.io/docs/docs-limelight/getting-started/ftc
 *
 *  COMPETITION DEBUGGING CHEAT SHEET (this file specifically):
 *    - Sweep never ends / spins forever -> accumulatedRotationDeg isn't
 *      reaching 360; check SWEEP_TURN_POWER isn't so low the robot barely
 *      moves, and that follower.getPose() heading is actually updating
 *      (odometry issue would masquerade as "sweep never completes").
 *    - Robot finds objects but keeps re-detecting the same one as "new" --
 *      check the 8-degree dedup tolerance below; if the sweep spins faster
 *      than intended, motion blur/latency could exceed even that widened
 *      tolerance.
 *    - Robot never actually collects anything -> startIntake()/stopIntake()
 *      are still EMPTY STUB METHODS at the bottom of this file. No physical
 *      intake mechanism is wired up yet.
 *    - Robot drives to an object but the object isn't actually there once
 *      it arrives -> suspect the angle-space clustering blind spot
 *      described in AXIS 1 above, OR the still-placeholder-looking
 *      DISTANCE_OFFSET_IN tuning constant below.
 * ============================================================================
 */
@Autonomous
public class LimeLightBasic extends OpMode {
    private Limelight3A limelight3A;
    private List<Blob> blobResults;
    private Follower follower;

    // Sliding-window width (degrees) for angle-space clustering in
    // findBestCluster() below. See AXIS 1 in the class-level note above for
    // why this angle-only approach can merge physically-separate objects
    // that happen to share a similar bearing at different distances.
    private static final double CLUSTER_WINDOW_DEG = 25.0;

    // Weight parameters: Count heavily dominates
    // Cluster scoring: score = SIZE_WEIGHT*count - DISTANCE_WEIGHT*avgDist.
    // With SIZE_WEIGHT=5.0 vs DISTANCE_WEIGHT=0.05 (a 100:1 ratio), cluster
    // SIZE overwhelmingly dominates the choice -- distance only matters as
    // a tiebreaker between similarly-sized clusters.
    private static final double SIZE_WEIGHT = 5.0;
    private static final double DISTANCE_WEIGHT = 0.05;

    // Camera Calibration
    private static final double CAMERA_HEIGHT_IN = 5.5;
    private static final double BALL_HEIGHT_IN = 2.8;
    private static final double CAMERA_MOUNT_ANGLE_DEG = 10.0;

    // Distance offset tuning parameter to reach the balls completely
    // Applied ONCE, in the DRIVE state below, to the final selected
    // cluster's estimated distance -- NOT per-detection during sweep/sample
    // (contrast with LimeLightBasicDist, which applies a similar correction
    // to EVERY individual detection at collection time, before clustering).
    private static final double DISTANCE_OFFSET_IN = 4.0;

    private static final double CAMERA_OFFSET_FORWARD_IN = 0.0;
    private static final double CAMERA_OFFSET_LEFT_IN = 0.0;

    // Field-frame starting pose -- must match the robot's real physical
    // starting position/heading at the start of the match (see the same
    // pattern/warning in every other PedroPathing-based OpMode in this
    // codebase, e.g. SampleAutoPathing.java).
    private final Pose startPose = new Pose(56.0, 8.0, Math.toRadians(90));

    // The winning cluster's actual Blob objects, populated by
    // findBestCluster() and consumed by the DRIVE state. Initialized to an
    // empty list (rather than left null) so the bestCluster.isEmpty() guard
    // at the top of the (missing here, but see LimelightIncrements) DRIVE
    // state logic -- actually present a few lines below -- has something
    // safe to check even before a sweep ever completes.
    private List<Blob> bestCluster = new ArrayList<>();

    DcMotor intake = new DcMotor();

    private enum State {
        SWEEP,
        TURN,
        DRIVE,
        PATH_TO_BALL,
        DONE
    }

    private State state;
    // Open-loop rotational power used for the continuous sweep spin (see
    // AXIS 2 in the class-level note above). Tuned slow/gentle (0.2) to
    // keep vision samples reasonably clean despite being taken in motion.
    private static final double SWEEP_TURN_POWER = 0.2;
    private double lastHeadingDeg;
    private double accumulatedRotationDeg;
    private double lastCaptureTimestamp = 0;
    private boolean turnCommandIssued = false;

    private double clusterAngle;
    private double distance;

    @Override
    public void init() {
        // Pipeline 0 = color/blob detection (matches every other
        // blob-detection OpMode in this codebase; contrast with
        // AprilTagLimelight.java's pipeline 8 for AprilTag detection on the
        // same physical camera).
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(0);
        blobResults = new ArrayList<>();

        // Builds the shared, globally-tuned PedroPathing Follower (see
        // Constants.java for the underlying PIDF/localizer/drivetrain
        // tuning this relies on).
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

        intake.init(hardwareMap, "intake");

        state = State.SWEEP;
        resetSweep();

        telemetry.addLine("LimeLightBasic initialized");
        // --- ADDED TELEMETRY: show the plan/config right at init, before anything
        // moves, so a pre-match check can confirm calibration constants at a glance ---
        telemetry.addData("startPose (x,y,heading deg)", "%.2f, %.2f, %.2f",
                startPose.getX(), startPose.getY(), Math.toDegrees(startPose.getHeading()));
        telemetry.addData("CAMERA_HEIGHT_IN / BALL_HEIGHT_IN", CAMERA_HEIGHT_IN + " / " + BALL_HEIGHT_IN);
        telemetry.addData("SWEEP_TURN_POWER", SWEEP_TURN_POWER);
        telemetry.update();
    }

    @Override
    public void start() {
        // Limelight3A requires an explicit start() call to begin actively
        // streaming/processing pipeline results.
        limelight3A.start();
        resetRuntime();
        // startTeleopDrive() switches the Follower into manual/open-loop
        // drive mode (as opposed to autonomous path-following mode), which
        // is what makes the setTeleOpDrive() calls in the SWEEP/DONE states
        // below meaningful -- without this, the Follower would still be
        // expecting a followPath() call rather than direct manual power
        // commands.
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        // Pedro needs this called every loop no matter what state we're in,
        // or its localization/path-following stalls.
        follower.update();

        if (state == State.SWEEP) {
            double headingDegNow = Math.toDegrees(follower.getPose().getHeading());
            // Accumulates SIGNED heading change loop-over-loop into a
            // running total, rather than comparing current heading against
            // a fixed target -- this is what lets the sweep track "have we
            // gone all the way around" even though the robot is spinning
            // continuously rather than stepping to discrete target angles.
            double deltaDeg = AngleUnit.normalizeDegrees(headingDegNow - lastHeadingDeg);
            accumulatedRotationDeg += deltaDeg;
            lastHeadingDeg = headingDegNow;

            if (Math.abs(accumulatedRotationDeg) < 360.0) {
                // Open-loop continuous spin -- true = field-centric (per
                // PedroPathing's setTeleOpDrive(x, y, turn, robotCentric)
                // signature convention, the boolean selects robot-centric
                // vs field-centric control; here 0,0 translation means it
                // doesn't actually matter for this pure-rotation command,
                // but it's worth knowing that flag exists if this is ever
                // extended to also translate while sweeping).
                follower.setTeleOpDrive(0, 0, SWEEP_TURN_POWER, true);

                LLResult llResult = limelight3A.getLatestResult();
                if (llResult != null && llResult.isValid()) {
                    double currentTimestamp = llResult.getTimestamp();
                    // Guards against processing the SAME camera frame twice
                    // if this loop() iteration runs faster than the
                    // Limelight produces new frames -- without this, a
                    // single physical detection could get sampled multiple
                    // times per actual new image, wasting the dedup check's
                    // effort (and, worse, subtly biasing which detections
                    // survive dedup based on loop-timing luck rather than
                    // actual new visual information).
                    boolean isNewFrame = (currentTimestamp > lastCaptureTimestamp);

                    if (isNewFrame) {
                        lastCaptureTimestamp = currentTimestamp;
                        List<LLResultTypes.DetectorResult> blobs = llResult.getDetectorResults();
                        for (LLResultTypes.DetectorResult blob : blobs) {
                            double trueX = AngleUnit.normalizeDegrees(-blob.getTargetXDegrees() + headingDegNow);
                            boolean duplicate = false;

                            for (Blob existing : blobResults) {
                                double angleDiff = AngleUnit.normalizeDegrees(existing.tx - trueX);
                                // Widen tolerance to 8 degrees due to motion blur and latency
                                // (This 8-degree tolerance, vs. the Increments siblings' tighter
                                // 5-degree tolerance, is direct in-code evidence of the continuous-
                                // vs-discrete sweep tradeoff described in AXIS 2 at the top of this
                                // file: sampling while still moving needs more slack.)
                                if (Math.abs(angleDiff) < 8.0 && Math.abs(existing.ty - blob.getTargetYDegrees()) < 8.0) {
                                    duplicate = true;
                                    break;
                                }
                            }
                            if (!duplicate) {
                                blobResults.add(new Blob(trueX, blob.getTargetYDegrees(), blob.getTargetArea()));
                            }
                        }
                    }
                }
            } else {
                // Full rotation complete: stop spinning and move on to
                // clustering + approach.
                follower.setTeleOpDrive(0, 0, 0, true);
                if (blobResults.isEmpty()) {
                    state = State.DONE;
                    // --- ADDED TELEMETRY: make a "found nothing" outcome explicit ---
                    telemetry.addLine("SWEEP complete: no blobs found -> DONE");
                } else {
                    clusterAngle = findBestCluster(blobResults);
                    turnCommandIssued = false;
                    state = State.TURN;
                    // --- ADDED TELEMETRY: announce the sweep result and chosen target ---
                    telemetry.addLine("SWEEP complete: cluster selected -> TURN");
                    telemetry.addData("chosen clusterAngle (deg)", clusterAngle);
                    telemetry.addData("blobs in chosen cluster", bestCluster.size());
                }
            }
        }

        else if (state == State.TURN) {
            if (!turnCommandIssued) {
                follower.turnTo(Math.toRadians(clusterAngle));
                turnCommandIssued = true;
            }

            double currentHeadingDeg = Math.toDegrees(follower.getPose().getHeading());
            double headingError = Math.abs(AngleUnit.normalizeDegrees(currentHeadingDeg - clusterAngle));

            // Advances on EITHER the follower reporting the turn done, OR
            // heading already being within 3 degrees -- whichever happens
            // FIRST. This is a deliberate safety net: if turnTo()'s own
            // internal completion criteria ever takes longer to settle than
            // necessary (e.g. fighting a tight tolerance/oscillating near
            // the target), the explicit 3-degree check here still lets the
            // state machine move on once "close enough" is reached, rather
            // than potentially stalling the whole autonomous waiting for a
            // perfect settle.
            if (!follower.isBusy() || headingError < 3.0) {
                turnCommandIssued = false;
                startIntake();
                state = State.DRIVE;
                // --- ADDED TELEMETRY: mark the TURN->DRIVE handoff and which
                // completion condition actually triggered it ---
                telemetry.addData("TURN complete via", !follower.isBusy() ? "follower not busy" : "heading within 3deg");
            }
        }

        else if (state == State.DRIVE) {
            if (bestCluster.isEmpty()) {
                // Defensive guard: shouldn't normally happen (TURN only
                // runs after a non-empty cluster was found), but protects
                // against a division-by-zero in the avgTy computation below
                // if bestCluster somehow ended up empty anyway.
                state = State.DONE;
                return;
            }

            double sumTy = 0;
            for (Blob b : bestCluster) {
                sumTy += b.ty;
            }
            double avgTy = sumTy / bestCluster.size();
            double effAngle = Math.max(5, Math.min(85, CAMERA_MOUNT_ANGLE_DEG + avgTy));

            double rawDistance = (CAMERA_HEIGHT_IN - BALL_HEIGHT_IN) / Math.tan(Math.toRadians(effAngle));

            // Add distance offset to ensure driving all the way to target
            // (see class-level note: this correction is applied ONCE here,
            // to the final chosen cluster only -- NOT per-detection the way
            // LimeLightBasicDist applies its equivalent correction.)
            distance = Math.max(6, Math.min(rawDistance + DISTANCE_OFFSET_IN, 96));

            Pose currentPose = follower.getPose();
            double robotHeadingRad = currentPose.getHeading();
            double clusterAngleRad = Math.toRadians(clusterAngle);

            double fieldX = currentPose.getX() + distance * Math.cos(clusterAngleRad);
            double fieldY = currentPose.getY() + distance * Math.sin(clusterAngleRad);

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
            // --- ADDED TELEMETRY: full distance/target math breakdown, matching the
            // level of detail added to the original single-shot LimeLightBasic.java ---
            telemetry.addData("DRIVE: avgTy (deg)", avgTy);
            telemetry.addData("DRIVE: rawDistance / +offset distance (in)", rawDistance + " / " + distance);
            telemetry.addData("DRIVE: target ballPose (x,y,heading deg)", "%.2f, %.2f, %.2f",
                    fieldX, fieldY, clusterAngle);
        }

        else if (state == State.PATH_TO_BALL) {
            if (!follower.isBusy()) {
                stopIntake();
                // Switches back to manual/teleop-style drive mode so the
                // next SWEEP state's setTeleOpDrive() calls work correctly
                // (mirrors the same call made once in start() -- necessary
                // again here because followPath() above switches the
                // Follower into its autonomous path-following mode).
                follower.startTeleopDrive();
                resetSweep();
                state = State.SWEEP;
                // --- ADDED TELEMETRY: mark that we're looping back for another object ---
                telemetry.addLine("PATH_TO_BALL complete: arrived, restarting SWEEP for next object");
            }
        }

        else if (state == State.DONE) {
            follower.setTeleOpDrive(0, 0, 0, true);
        }

        telemetry.addData("State", state.toString());
        telemetry.addData("Cluster Angle", clusterAngle);
        telemetry.addData("Target Distance", distance);
        telemetry.addData("Blobs Found", blobResults.size());
        // --- ADDED TELEMETRY: fields useful for live competition debugging that
        // weren't previously surfaced every loop ---
        telemetry.addData("accumulatedRotationDeg (SWEEP progress)", accumulatedRotationDeg);
        telemetry.addData("follower busy?", follower.isBusy());
        telemetry.addData("loop runtime (s)", getRuntime());
        telemetry.update();
    }

    /**
     * Fixes negative angle sorting and window wraparound bugs across the 0/360 boundary.
     *
     * ANGLE-SPACE sliding-window clustering (see AXIS 1 in the class-level
     * note at the top of this file for how this differs from the
     * "...Dist" siblings' field-space clustering). Normalizes every blob's
     * bearing into [0, 360) via the inner AngleBlob helper, sorts by that
     * normalized angle, then duplicates the sorted list shifted by +360 so
     * the sliding window can consider windows that straddle the 0/360
     * wraparound boundary without special-casing modulo arithmetic (same
     * technique used in the original single-shot LimeLightBasic.java's
     * findBestCluster()).
     */
    public double findBestCluster(List<Blob> blobs) {
        if (blobs.isEmpty()) return 0;

        class AngleBlob {
            Blob blob;
            double posTx;
            AngleBlob(Blob b) {
                this.blob = b;
                // Normalizes tx into [0, 360) regardless of what range
                // AngleUnit.normalizeDegrees() elsewhere in this file
                // returned it in (that helper can return negative values);
                // this local normalization keeps the sort/window logic
                // below simple and always non-negative.
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
            // Skip windows fully inside the duplicated second half, to
            // avoid scoring the same physical cluster twice.
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

        // Circular mean of the winning window's angles (see the original
        // LimeLightBasic.java's findBestCluster() javadoc for why a plain
        // arithmetic mean would be wrong for wraparound angles).
        return AngleUnit.normalizeDegrees(Math.toDegrees(Math.atan2(sumSin, sumCos)));
    }

    /**
     * Clears sweep state so the next SWEEP phase starts clean -- called
     * once from init() and again every time PATH_TO_BALL completes (since
     * this OpMode loops back to collect multiple objects per autonomous
     * period, unlike the original single-shot LimeLightBasic.java).
     */
    private void resetSweep() {
        blobResults = new ArrayList<>();
        lastHeadingDeg = Math.toDegrees(follower.getPose().getHeading());
        accumulatedRotationDeg = 0;
        lastCaptureTimestamp = 0;
    }

    // --- STUB METHODS: no physical intake mechanism is wired up here yet. ---
    // These are called at the TURN->DRIVE and PATH_TO_BALL->SWEEP transitions
    // respectively (i.e. "start collecting once aimed at the target" / "stop
    // collecting once arrived and about to look for the next one"), but both
    // bodies are currently empty. If the robot drives to objects correctly
    // but doesn't actually pick anything up, THIS is why -- these need real
    // mechanism calls (e.g. a ServoClass or DcMotor mechanism instance)
    // before this auto can actually collect anything.
    private void startIntake() {
        intake.setMotorSpeed(-1.0);
    }

    private void stopIntake() {
        intake.setMotorSpeed(0.0);
    }
}