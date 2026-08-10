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

@Autonomous
public class LimeLightBasic extends OpMode {
    private Limelight3A limelight3A;
    private List<Blob> blobResults;
    private Follower follower;

    private static final double CLUSTER_WINDOW_DEG = 25.0;

    // Weight parameters: Count heavily dominates
    private static final double SIZE_WEIGHT = 5.0;
    private static final double DISTANCE_WEIGHT = 0.05;

    // Camera Calibration
    private static final double CAMERA_HEIGHT_IN = 5.5;
    private static final double BALL_HEIGHT_IN = 2.8;
    private static final double CAMERA_MOUNT_ANGLE_DEG = 10.0;

    // Distance offset tuning parameter to reach the balls completely
    private static final double DISTANCE_OFFSET_IN = 4.0;

    private static final double CAMERA_OFFSET_FORWARD_IN = 0.0;
    private static final double CAMERA_OFFSET_LEFT_IN = 0.0;

    private final Pose startPose = new Pose(56.0, 8.0, Math.toRadians(90));

    private List<Blob> bestCluster = new ArrayList<>();

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

    private double clusterAngle;
    private double distance;

    @Override
    public void init() {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(0);
        blobResults = new ArrayList<>();

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

        state = State.SWEEP;
        resetSweep();

        telemetry.addLine("LimeLightBasic initialized");
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
            double headingDegNow = Math.toDegrees(follower.getPose().getHeading());
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
                            double trueX = AngleUnit.normalizeDegrees(-blob.getTargetXDegrees() + headingDegNow);
                            boolean duplicate = false;

                            for (Blob existing : blobResults) {
                                double angleDiff = AngleUnit.normalizeDegrees(existing.tx - trueX);
                                // Widen tolerance to 8 degrees due to motion blur and latency
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
                follower.setTeleOpDrive(0, 0, 0, true);
                if (blobResults.isEmpty()) {
                    state = State.DONE;
                } else {
                    clusterAngle = findBestCluster(blobResults);
                    turnCommandIssued = false;
                    state = State.TURN;
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

            if (!follower.isBusy() || headingError < 3.0) {
                turnCommandIssued = false;
                startIntake();
                state = State.DRIVE;
            }
        }

        else if (state == State.DRIVE) {
            if (bestCluster.isEmpty()) {
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
        telemetry.addData("Cluster Angle", clusterAngle);
        telemetry.addData("Target Distance", distance);
        telemetry.addData("Blobs Found", blobResults.size());
        telemetry.update();
    }

    /**
     * Fixes negative angle sorting and window wraparound bugs across the 0/360 boundary.
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

    private void resetSweep() {
        blobResults = new ArrayList<>();
        lastHeadingDeg = Math.toDegrees(follower.getPose().getHeading());
        accumulatedRotationDeg = 0;
        lastCaptureTimestamp = 0;
    }

    private void startIntake() {

    }

    private void stopIntake() {

    }
}