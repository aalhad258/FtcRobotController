package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechanism.Blob;
import org.firstinspires.ftc.teamcode.mechanism.Drive;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class LimeLightBasic extends OpMode {
    private Limelight3A limelight3A;
    private List<Blob> blobResults;

    private IMU imu;

    Drive drive = new Drive();

    private enum State {
        ROTATE,
        SAMPLE,
        TURN,
        DRIVE,
        DONE
    }

    private State state;

    private double targetAngle;

    private double kP = 0.01;
    private double kD = 0.0;

    private double lastError = 0;
    private double yaw = 0;
    int maxIndex;
    double distance;
    private List<Double> scores;

    @Override
    public void init() {
        limelight3A= hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(9);
        blobResults = new ArrayList<>();
        drive.init(hardwareMap);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        state = State.ROTATE;
        targetAngle = 30;
        scores = new ArrayList<>();
    }

    @Override
    public void start() {
        limelight3A.start();
    }

    @Override
    public void loop() {
        // add error stuff
        if (state == State.ROTATE) {
            yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double error = AngleUnit.normalizeDegrees(targetAngle - yaw);
            // ----- PD ----- TO BE TUNED
            double derivative = error - lastError;
            double power = (kP * error) + (kD * derivative);
            lastError = error;
            power = Math.max(-0.4, Math.min(0.4, power));
            drive.setPower(0, 0, power, 1.0);
            if (Math.abs(error) < 2) {
                drive.setPower(0, 0, 0, 1.0);
                lastError = 0;
                state = State.SAMPLE;
            }
        }

        else if (state == State.SAMPLE) {
            LLResult llResult = limelight3A.getLatestResult();
            if (llResult != null && llResult.isValid()) {
                List<LLResultTypes.ColorResult> blobs = llResult.getColorResults();
                for (LLResultTypes.ColorResult blob : blobs) {
                    boolean blobThere = false;
                    double trueX = AngleUnit.normalizeDegrees(blob.getTargetXDegrees() + yaw);
                    for (Blob blobResult : blobResults) {
                        if (
                                (Math.abs(blobResult.tx - trueX) < 6) &&
                                        (Math.abs(blobResult.ty - blob.getTargetYDegrees()) < 6)
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
                if (blobResults.isEmpty()) {
                    state = State.DONE;
                    return;
                }

            }
            targetAngle += 30;
            if (targetAngle > 360) {
                for (Blob blob : blobResults) {
                    // WE WILL SEE IF THIS TERM IS CALCULATED BASED ON AREA OR HEIGHT OR WHAT
                    distance = 1/Math.sqrt(blob.ta);
                    double score = 0.7* blob.ta-0.05*Math.abs((AngleUnit.normalizeDegrees(blob.tx - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)))-0.25/distance);
                    scores.add(score);
                }
                double max = Double.NEGATIVE_INFINITY;
                for (int i = 0; i < scores.size(); i++) {
                    if (scores.get(i) > max) {
                        maxIndex = i;
                        max = scores.get(i);
                    }
                }
                state = State.TURN;
            } else {
                state = State.ROTATE;
            }
        }

        else if (state == State.TURN) {
            yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double error = AngleUnit.normalizeDegrees(blobResults.get(maxIndex).tx - yaw);
            // ----- PD ----- TO BE TUNED
            double derivative = error - lastError;
            double power = (kP * error) + (kD * derivative);
            lastError = error;
            power = Math.max(-0.4, Math.min(0.4, power));
            drive.setPower(0, 0, power, 1.0);
            if (Math.abs(error) < 2) {
                drive.setPower(0, 0, 0, 1.0);
                lastError = 0;
                state = State.DRIVE;
            }
        }

        else if (state == State.DRIVE) {
            Blob blob = blobResults.get(maxIndex);
            distance = 1/Math.sqrt(blob.ta);
            double headingToBlob = Math.toRadians(blob.tx);
            double relX = distance * Math.cos(headingToBlob);
            double relY = distance * Math.sin(headingToBlob);
            double robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double heading = Math.toRadians(robotHeading);
            double robotX = 0; // WE WILL REPLACE THESE WITH ACC X AND Y ONCE PEDROPATHING AND LOCALIZATION IS SETUP
            double robotY = 0;
            double fieldX =
                    robotX
                            + relX * Math.cos(heading)
                            - relY * Math.sin(heading);

            double fieldY =
                    robotY
                            + relX * Math.sin(heading)
                            + relY * Math.cos(heading);
            // WE WILL FEED IN FIELDX AND FIELDY INTO PEDROPATHING
        }

        else if (state == State.DONE) {
            drive.setPower(0, 0, 0, 1.0);
        }

    }
}
