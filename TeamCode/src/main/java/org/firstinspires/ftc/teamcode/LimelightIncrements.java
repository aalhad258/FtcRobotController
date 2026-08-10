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

@Autonomous
public class LimelightIncrements extends OpMode {
    private Limelight3A limelight3A;
    private List<Blob> blobResults;
    private Follower follower;

    private static final double CLUSTER_WINDOW_DEG = 25.0;
    private static final double SIZE_WEIGHT = 5.0;
    private static final double DISTANCE_WEIGHT = 0.05;

    private static final double CAMERA_HEIGHT_IN = 8.0;
    private static final double BALL_HEIGHT_IN = 2.0;
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
    private int sweepStepCount = 0;
    private static final int TOTAL_SWEEP_STEPS = 12; // 12 steps * 30 deg = 360 deg

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
                if (!follower.isBusy() || headingError < 3.0) {
                    turnCommandIssued = false;
                    stateTimer.reset();
                    state = State.SETTLE;
                }
                break;

            case SETTLE:
                if (stateTimer.milliseconds() > 150) {
                    state = State.SAMPLE;
                }
                break;

            case SAMPLE:
                LLResult llResult = limelight3A.getLatestResult();
                if (llResult != null && llResult.isValid()) {
                    List<LLResultTypes.DetectorResult> blobs = llResult.getDetectorResults();
                    for (LLResultTypes.DetectorResult blob : blobs) {
                        double trueX = AngleUnit.normalizeDegrees(-blob.getTargetXDegrees() + headingDeg);

                        boolean duplicate = false;
                        for (Blob existing : blobResults) {
                            double angleDiff = AngleUnit.normalizeDegrees(existing.tx - trueX);
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
        telemetry.addData("Blobs Found", blobResults.size());
        telemetry.addData("Target Cluster Angle", clusterAngle);
        telemetry.addData("Target Distance (in)", distance);
        telemetry.update();
    }

    public void resetSweep() {
        blobResults = new ArrayList<>();
        sweepStepCount = 0;
    }

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

    private void startIntake() {

    }

    private void stopIntake() {

    }
}