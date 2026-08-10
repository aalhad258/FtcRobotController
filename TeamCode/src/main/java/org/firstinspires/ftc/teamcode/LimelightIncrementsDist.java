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

    // Camera Calibration
    private static final double CAMERA_HEIGHT_IN = 8.0;
    private static final double BALL_HEIGHT_IN = 2.0;
    private static final double CAMERA_MOUNT_ANGLE_DEG = 10.0;

    // Distance Calibration Tweaks
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
                break;

            case PATH_TO_BALL:
                if (!follower.isBusy()) {
                    stopIntake();
                    resetSweep();
                    state = State.ROTATE;
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
        telemetry.update();
    }

    public void resetSweep() {
        fieldBlobs = new ArrayList<>();
        sweepStepCount = 0;
        targetClusterCenter = null;
    }

    /**
     * Evaluates clusters in 2D field-space using a physical radius on the mat.
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

    private void startIntake() {

    }

    private void stopIntake() {

    }
}