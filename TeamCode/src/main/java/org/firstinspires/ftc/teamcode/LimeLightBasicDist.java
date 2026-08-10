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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;

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
    private static final double SIZE_WEIGHT = 10.0;
    private static final double DISTANCE_WEIGHT = 0.05;

    // Camera Calibration
    private static final double CAMERA_HEIGHT_IN = 5.5;
    private static final double BALL_HEIGHT_IN = 2.8;
    private static final double CAMERA_MOUNT_ANGLE_DEG = 10.0;

    // Distance Calibration Tweaks
    private static final double DISTANCE_OFFSET_IN = 5.0; // Flat extra inches added to drive target
    private static final double DISTANCE_MULTIPLIER = 1.05; // Scaling factor for distance calculation

    private static final double CAMERA_OFFSET_FORWARD_IN = 0.0;
    private static final double CAMERA_OFFSET_LEFT_IN = 0.0;

    private final Pose startPose = new Pose(56.0, 8.0, Math.toRadians(90));

    private FieldBlob targetClusterCenter;

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

        state = State.SWEEP;
        resetSweep();

        telemetry.addLine("LimeLightBasic Initialized (Field-Space Deduplication)");
        telemetry.update();
    }

    @Override
    public void start() {
        limelight3A.start();
        resetRuntime();
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
                    boolean isNewFrame = (currentTimestamp > lastCaptureTimestamp);

                    if (isNewFrame) {
                        lastCaptureTimestamp = currentTimestamp;
                        List<LLResultTypes.DetectorResult> blobs = llResult.getDetectorResults();

                        for (LLResultTypes.DetectorResult blob : blobs) {
                            double tx = blob.getTargetXDegrees();
                            double ty = blob.getTargetYDegrees();

                            // Calculate raw ground distance to detected blob
                            double effAngle = Math.max(5, Math.min(85, CAMERA_MOUNT_ANGLE_DEG + ty));
                            double rawDist = (CAMERA_HEIGHT_IN - BALL_HEIGHT_IN) / Math.tan(Math.toRadians(effAngle));
                            double calcDist = (rawDist * DISTANCE_MULTIPLIER) + DISTANCE_OFFSET_IN;

                            // Compute global field heading to the blob
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
                } else {
                    targetClusterCenter = findBestFieldCluster(fieldBlobs);
                    turnCommandIssued = false;
                    state = State.TURN;
                }
            }
        }

        else if (state == State.TURN) {
            if (targetClusterCenter == null) {
                state = State.DONE;
                return;
            }

            Pose currentPose = follower.getPose();
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

            Pose ballPose = new Pose(targetClusterCenter.x, targetClusterCenter.y, Math.toRadians(targetHeadingDeg));

            PathChain driveToCluster = follower.pathBuilder()
                    .addPath(new BezierLine(currentPose, ballPose))
                    .setLinearHeadingInterpolation(currentPose.getHeading(), ballPose.getHeading())
                    .build();

            follower.followPath(driveToCluster, true);
            state = State.PATH_TO_BALL;
        }

        else if (state == State.PATH_TO_BALL) {
            if (!follower.isBusy()) {
                stopIntake();
                follower.startTeleopDrive();
                resetSweep();
                state = State.SWEEP;
            }
        }

        else if (state == State.DONE) {
            follower.setTeleOpDrive(0, 0, 0, true);
        }

        telemetry.addData("State", state.toString());
        telemetry.addData("Blobs Found", fieldBlobs.size());
        telemetry.addData("Target Heading", targetHeadingDeg);
        telemetry.addData("Target Distance (in)", targetDistance);
        telemetry.update();
    }

    /**
     * Finds the cluster with the highest density/score using physical field coordinates.
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

    private void resetSweep() {
        fieldBlobs = new ArrayList<>();
        lastHeadingDeg = Math.toDegrees(follower.getPose().getHeading());
        accumulatedRotationDeg = 0;
        lastCaptureTimestamp = 0;
        targetClusterCenter = null;
    }

    private void startIntake() {

    }

    private void stopIntake() {

    }
}