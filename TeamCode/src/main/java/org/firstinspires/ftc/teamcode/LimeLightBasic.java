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

    private static final double CLUSTER_WINDOW_DEG = 20.0;
    private static final double SIZE_WEIGHT = 1.0;
    private static final double DISTANCE_WEIGHT = 0.5;

    // ---- Camera calibration ----
    private static final double CAMERA_HEIGHT_IN = 8.0;        // TODO: height of camera lens off the ground
    private static final double BALL_HEIGHT_IN = 2.0;          // TODO: ball center height off ground (~radius)
    private static final double CAMERA_MOUNT_ANGLE_DEG = 10.0; // TODO: downward tilt of camera from horizontal

    // ---- Camera's position offset from the robot's center of rotation, robot-local frame ----
    private static final double CAMERA_OFFSET_FORWARD_IN = 0.0; // TODO: + = forward of center
    private static final double CAMERA_OFFSET_LEFT_IN = 3.0;    // TODO: + = left of center

    // ---- Where the robot starts, in field coordinates (match your actual autonomous start) ----
    private final Pose startPose = new Pose(56.0, 8.0, Math.toRadians(90)); // TODO: match your real start pose

    private List<Blob> bestCluster;

    private enum State {
        ROTATE,
        SAMPLE,
        TURN,
        DRIVE,
        PATH_TO_BALL,
        DONE
    }

    private State state;

    // Absolute field-frame heading (degrees) we're currently sweeping toward / driving toward.
    // Since everything is now referenced off follower.getPose().getHeading(), this is a
    // FIELD-frame angle, not a robot-relative one.
    private double targetAngle;
    private double clusterAngle;
    private double distance;

    // Guards so we only issue a turnTo()/followPath() command ONCE per state entry,
    // instead of re-issuing it every loop() call (which would restart the motion).
    private boolean turnCommandIssued = false;

    @Override
    public void init() {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(0);
        blobResults = new ArrayList<>();

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

        state = State.ROTATE;
        // Sweep starts at the robot's current field heading + 30, not a raw "30 degrees".
        targetAngle = AngleUnit.normalizeDegrees(Math.toDegrees(startPose.getHeading()) + 30);
    }

    @Override
    public void start() {
        limelight3A.start();
        resetRuntime();
    }

    @Override
    public void loop() {
        // Pedro needs this called every loop no matter what state we're in,
        // or its localization/path-following stalls.
        follower.update();

        double headingDeg = Math.toDegrees(follower.getPose().getHeading());

        if (state == State.ROTATE) {
            if (!turnCommandIssued) {
                follower.turnTo(Math.toRadians(targetAngle));
                turnCommandIssued = true;
            }
            if (!follower.isBusy()) {
                turnCommandIssued = false;
                state = State.SAMPLE;
            }
        }

        else if (state == State.SAMPLE) {
            LLResult llResult = limelight3A.getLatestResult();
            if (llResult != null && llResult.isValid()) {
                List<LLResultTypes.DetectorResult> blobs = llResult.getDetectorResults();
                for (LLResultTypes.DetectorResult blob : blobs) {
                    boolean blobThere = false;
                    // Camera-relative angle + current FIELD heading = absolute field bearing.
                    double trueX = AngleUnit.normalizeDegrees(-blob.getTargetXDegrees() + headingDeg);
                    for (Blob blobResult : blobResults) {
                        if (
                                (Math.abs(blobResult.tx - trueX) < 3) &&
                                        (Math.abs(blobResult.ty - blob.getTargetYDegrees()) < 3)
                        ) {
                            blobThere = true;
                            break;
                        }
                    }
                    if (!blobThere) {
                        blobResults.add(new Blob(
                                trueX,
                                blob.getTargetYDegrees(),
                                blob.getTargetArea()
                        ));
                    }
                }
            }

            targetAngle += 30;
            // Compare against the sweep's starting point (startPose heading), not raw 360,
            // since targetAngle is now a field-frame absolute angle that could start anywhere.
            if (targetAngle > Math.toDegrees(startPose.getHeading()) + 360) {
                if (blobResults.isEmpty()) {
                    state = State.DONE;
                    return;
                }
                clusterAngle = findBestCluster(blobResults);
                state = State.TURN;
            } else {
                state = State.ROTATE;
            }
        }

        else if (state == State.TURN) {
            if (!turnCommandIssued) {
                follower.turnTo(Math.toRadians(clusterAngle));
                turnCommandIssued = true;
            }
            if (!follower.isBusy()) {
                turnCommandIssued = false;
                state = State.DRIVE;
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
            // baked in back in SAMPLE), so we project distance directly along it â€” no second
            // rotation by robotHeading here, unlike the earlier version.
            double clusterAngleRad = Math.toRadians(clusterAngle);
            double fieldX = robotX + distance * Math.cos(clusterAngleRad);
            double fieldY = robotY + distance * Math.sin(clusterAngleRad);

            // The camera mount offset IS robot-relative, so this one still needs exactly
            // ONE rotation by the robot's current heading to convert into field frame.
            double cameraFieldOffsetX = CAMERA_OFFSET_FORWARD_IN * Math.cos(robotHeadingRad) - CAMERA_OFFSET_LEFT_IN * Math.sin(robotHeadingRad);
            double cameraFieldOffsetY = CAMERA_OFFSET_FORWARD_IN * Math.sin(robotHeadingRad) + CAMERA_OFFSET_LEFT_IN * Math.cos(robotHeadingRad);
            fieldX += cameraFieldOffsetX;
            fieldY += cameraFieldOffsetY;

            Pose ballPose = new Pose(fieldX, fieldY, robotHeadingRad);
            PathChain driveToCluster = follower.pathBuilder()
                    .addPath(new BezierLine(currentPose, ballPose))
                    .setLinearHeadingInterpolation(currentPose.getHeading(), ballPose.getHeading())
                    .build();

            follower.followPath(driveToCluster, true);
            state = State.PATH_TO_BALL;
        }

        else if (state == State.PATH_TO_BALL) {
            if (!follower.isBusy()) {
                state = State.DONE;
            }
        }

        else if (state == State.DONE) {
            // Follower holds position automatically once idle.
        }

        telemetry.addData("state", state.toString());
        telemetry.addData("targetAngle", targetAngle);
        telemetry.addData("clusterAngle", clusterAngle);
        telemetry.addData("distance", distance);
        telemetry.addData("robot pose", follower.getPose().toString());
    }

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
            while (angles.get(windowEnd) - angles.get(windowStart) > CLUSTER_WINDOW_DEG) {
                windowStart++;
            }
            if (windowStart >= n) continue;

            int count = windowEnd - windowStart + 1;

            double sumTy = 0;
            for (int i = windowStart; i <= windowEnd; i++) {
                sumTy += sorted.get(i % n).ty;
            }
            double avgTy = sumTy / count;
            double avgDistance = (CAMERA_HEIGHT_IN - BALL_HEIGHT_IN)
                    / Math.tan(Math.toRadians(CAMERA_MOUNT_ANGLE_DEG + avgTy));

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
}