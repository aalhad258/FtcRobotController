package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.mechanism.AprilTagLimelight;
import org.firstinspires.ftc.teamcode.mechanism.Drive;
import org.firstinspires.ftc.teamcode.mechanism.TurretMechanism;

@TeleOp(name = "TurretAutoAlign")
public class TurretAutoAlign extends OpMode {

    // =========================================================
    // HARDWARE
    // =========================================================

    private final Drive drive = new Drive();

    private final org.firstinspires.ftc.teamcode.mechanism.DcMotor intake =
            new org.firstinspires.ftc.teamcode.mechanism.DcMotor();

    private final TurretMechanism turret = new TurretMechanism();

    private final AprilTagLimelight atlimelight = new AprilTagLimelight();

    private DcMotorEx shooter;

    private GoBildaPinpointDriver pinpoint;

    // =========================================================
    // SHOOTER
    // =========================================================

    private static final double TICKS_PER_REV = 28.0;

    private static final double FLYWHEEL_P = 2.0;
    private static final double FLYWHEEL_I = 0.0;
    private static final double FLYWHEEL_D = 0.5;
    private static final double FLYWHEEL_F = 10.6;

    private boolean shooterActive = false;
    private boolean lastAState = false;

    private double targetRPM = 0.0;
    private double actualRPM = 0.0;

    // =========================================================
    // TARGET POSITION
    // Target is 54 inches forward along the X-axis.
    // =========================================================

    private static final double TARGET_X = 54.0;
    private static final double TARGET_Y = 0.0;

    private static final double SHOOTING_ANGLE_DEG = 70.8;
    private static final double SHOOTING_ANGLE_RAD =
            Math.toRadians(SHOOTING_ANGLE_DEG);

    private static final double HIVE_HEIGHT = 63.0;
    private static final double SHOOTER_HEIGHT = 10.0;

    private static final double WHEEL_RADIUS_METERS = 0.036;
    private static final double GRAVITY = 9.81;

    // =========================================================
    // LIMELIGHT / TURRET ALIGNMENT
    // =========================================================

    private static final double ALIGN_TOLERANCE_DEG = 0.2;

    private double currentTX = 0.0;

    private boolean hasTarget = false;
    private boolean aligned = false;
    private boolean intakeActive = false;
    private boolean lastBState = false;

    private String selectedTag = "None";
    private int visibleTagCount = 0;

    // =========================================================
    // PID TUNING: GAMEPAD 2
    // =========================================================

    private final double[] stepSizes = {
            0.1,
            0.01,
            0.001,
            0.0001,
            0.00001
    };

    private int stepIndex = 2;

    // =========================================================
    // PINPOINT POSITION
    // =========================================================

    private Pose2D currentPose = new Pose2D(
            DistanceUnit.INCH,
            0,
            0,
            AngleUnit.DEGREES,
            0
    );

    // =========================================================
    // INIT
    // =========================================================

    @Override
    public void init() {

        drive.init(hardwareMap);

        intake.init(hardwareMap, "intake");

        turret.init(hardwareMap);

        atlimelight.init(hardwareMap, 0);

        shooter = hardwareMap.get(
                DcMotorEx.class,
                "shooter"
        );

        shooter.setMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER
        );

        shooter.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER
        );

        shooter.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(
                        FLYWHEEL_P,
                        FLYWHEEL_I,
                        FLYWHEEL_D,
                        FLYWHEEL_F
                )
        );

        pinpoint = hardwareMap.get(
                GoBildaPinpointDriver.class,
                "pinpoint"
        );

        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods
                        .goBILDA_4_BAR_POD
        );

        pinpoint.setOffsets(
                2,
                -6.5,
                DistanceUnit.INCH
        );

        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );

        pinpoint.setPosition(currentPose);

        telemetry.addLine("TurretAutoAlign initialized");

        telemetry.addLine("GAMEPAD 1:");
        telemetry.addLine("Left stick = Drive");
        telemetry.addLine("Right stick = Turn");
        telemetry.addLine("A = Toggle shooter");
        telemetry.addLine("LB = Intake");

        telemetry.addLine("GAMEPAD 2:");
        telemetry.addLine("D-pad = Tune turret PID");
        telemetry.addLine("B = Change PID step size");

        telemetry.update();
    }

    @Override
    public void start() {
        atlimelight.start();
        turret.resetTimer();
    }

    // =========================================================
    // MAIN LOOP
    // =========================================================

    @Override
    public void loop() {

        // -----------------------------------------------------
        // PINPOINT
        // -----------------------------------------------------

        pinpoint.update();

        currentPose = pinpoint.getPosition();

        double robotX = currentPose.getX(DistanceUnit.INCH);
        double robotY = currentPose.getY(DistanceUnit.INCH);

        double robotHeading =
                currentPose.getHeading(AngleUnit.DEGREES);

        // -----------------------------------------------------
        // DRIVE
        // -----------------------------------------------------

        double axial = -gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

        drive.setPower(axial, lateral, yaw, 1.0);

        // -----------------------------------------------------
        // LIMELIGHT UPDATE
        // -----------------------------------------------------

        atlimelight.update();

        LLResult result = atlimelight.getResult();

        hasTarget = false;
        aligned = false;
        currentTX = 0.0;
        selectedTag = "None";
        visibleTagCount = 0;

        boolean resultValid = result != null && result.isValid();

        boolean has35 = false;
        boolean has36 = false;
        boolean hasFallback = false;

        double tx35 = 0.0;
        double tx36 = 0.0;
        double fallbackTX = 0.0;
        int fallbackID = -1;

        // -----------------------------------------------------
        // RAW LIMELIGHT DEBUG TELEMETRY
        // -----------------------------------------------------

        telemetry.addLine("========== LIMELIGHT DEBUG ==========");

        telemetry.addData("Result Is Null", result == null);
        telemetry.addData("Result Valid", resultValid);

        if (result == null) {

            telemetry.addLine("ERROR: Limelight result is NULL");

        } else {

            // Prints the result object's string representation.
            telemetry.addData("Raw Result", result.toString());

            if (resultValid) {

                // Print every AprilTag returned by the Limelight.
                for (LLResultTypes.FiducialResult tag :
                        result.getFiducialResults()) {

                    int id = tag.getFiducialId();
                    double tx = tag.getTargetXDegrees();

                    visibleTagCount++;

                    telemetry.addData(
                            "Detected Tag " + visibleTagCount,
                            "ID=%d | TX=%.2f degrees",
                            id,
                            tx
                    );

                    if (id == 35) {

                        tx35 = tx;
                        has35 = true;

                    } else if (id == 36) {

                        tx36 = tx;
                        has36 = true;

                    } else if (id == 34 || id == 37) {

                        if (!hasFallback) {
                            fallbackTX = tx;
                            fallbackID = id;
                            hasFallback = true;
                        }
                    }
                }

            } else {

                telemetry.addLine(
                        "Result invalid: checking pipeline/camera"
                );
            }
        }

        // -----------------------------------------------------
        // SELECT APRILTAG
        // Priority: average 35+36, then 35, then 36,
        // then either 34 or 37.
        // -----------------------------------------------------

        if (has35 && has36) {

            currentTX = (tx35 + tx36) / 2.0;
            selectedTag = "35 + 36";
            hasTarget = true;

        } else if (has35) {

            currentTX = tx35;
            selectedTag = "35";
            hasTarget = true;

        } else if (has36) {

            currentTX = tx36;
            selectedTag = "36";
            hasTarget = true;

        } else if (hasFallback) {

            currentTX = fallbackTX;
            selectedTag = String.valueOf(fallbackID);
            hasTarget = true;
        }

        telemetry.addData("Total Tags Detected", visibleTagCount);
        telemetry.addData("Selected Tag", selectedTag);
        telemetry.addData("Selected TX", "%.2f degrees", currentTX);
        telemetry.addData("Supported Target Found", hasTarget);

        telemetry.addLine("=====================================");

        // -----------------------------------------------------
        // TURRET ALIGNMENT OR SEARCH
        // -----------------------------------------------------

        if (hasTarget) {

            turret.update(currentTX);

            aligned =
                    Math.abs(currentTX) < ALIGN_TOLERANCE_DEG;

        } else {

            turret.stop();
        }

        // -----------------------------------------------------
        // PID TUNING: GAMEPAD 2
        // -----------------------------------------------------

        if (gamepad2.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad2.dpadLeftWasPressed()) {
            turret.setKP(
                    turret.getKP() - stepSizes[stepIndex]
            );
        }

        if (gamepad2.dpadRightWasPressed()) {
            turret.setKP(
                    turret.getKP() + stepSizes[stepIndex]
            );
        }

        if (gamepad2.dpadUpWasPressed()) {
            turret.setKD(
                    turret.getKD() + stepSizes[stepIndex]
            );
        }

        if (gamepad2.dpadDownWasPressed()) {
            turret.setKD(
                    turret.getKD() - stepSizes[stepIndex]
            );
        }

        // -----------------------------------------------------
        // SHOOTER TOGGLE: GAMEPAD 1 A
        // -----------------------------------------------------

        if (gamepad1.a && !lastAState) {
            shooterActive = !shooterActive;
        }

        lastAState = gamepad1.a;

        // -----------------------------------------------------
        // CALCULATE TARGET RPM
        // -----------------------------------------------------

        targetRPM = calculateTargetRPM(robotX, robotY);

        // -----------------------------------------------------
        // CONTROL SHOOTER
        // -----------------------------------------------------

        if (shooterActive) {

            double targetTicksPerSec =
                    (targetRPM / 60.0) * TICKS_PER_REV;

            shooter.setVelocity(targetTicksPerSec);

        } else {

            shooter.setVelocity(0.0);
        }

        actualRPM =
                (shooter.getVelocity() / TICKS_PER_REV) * 60.0;

        // -----------------------------------------------------
        // INTAKE TOGGLE: GAMEPAD 1 B
        // -----------------------------------------------------

        if (gamepad1.b && !lastBState) {
            intakeActive = !intakeActive;
        }

        lastBState = gamepad1.b;

        if (intakeActive) {
            intake.setMotorSpeed(1.0);
        } else {
            intake.setMotorSpeed(0.0);
        }

        // -----------------------------------------------------
        // GENERAL TELEMETRY
        // -----------------------------------------------------

        telemetry.addLine("========== TURRET ==========");

        telemetry.addData(
                "Turret Status",
                hasTarget
                        ? (aligned ? "ALIGNED" : "TRACKING")
                        : (turret.isSearching()
                           ? "SEARCHING"
                           : "SEARCH COMPLETE")
        );

        telemetry.addData(
                "Turret Status",
                hasTarget
                        ? (aligned ? "ALIGNED" : "TRACKING")
                        : "SEARCHING"
        );

        telemetry.addData("P Gain", "%.6f", turret.getKP());
        telemetry.addData("D Gain", "%.6f", turret.getKD());

        telemetry.addData(
                "PID Step Size",
                "%.5f",
                stepSizes[stepIndex]
        );

        telemetry.addLine("========== SHOOTER ==========");

        telemetry.addData(
                "Shooter State",
                shooterActive ? "ACTIVE" : "OFF"
        );

        telemetry.addData("Target RPM", "%.1f", targetRPM);
        telemetry.addData("Actual RPM", "%.1f", actualRPM);

        telemetry.addLine("========== PINPOINT ==========");

        telemetry.addData("Robot X", "%.2f in", robotX);
        telemetry.addData("Robot Y", "%.2f in", robotY);

        telemetry.addData(
                "Robot Heading",
                "%.2f degrees",
                robotHeading
        );

        telemetry.update();
    }

    // =========================================================
    // SHOOTING CALCULATION
    // =========================================================

    private double calculateTargetRPM(
            double robotX,
            double robotY
    ) {

        double dx = TARGET_X - robotX;
        double dy = TARGET_Y - robotY;

        double shootDistance =
                0.0254 * Math.hypot(dx, dy);

        double deltaY =
                (HIVE_HEIGHT - SHOOTER_HEIGHT) * 0.0254;

        double cosTheta =
                Math.cos(SHOOTING_ANGLE_RAD);

        double tanTheta =
                Math.tan(SHOOTING_ANGLE_RAD);

        double denominator =
                2.0 * cosTheta * cosTheta
                        * (shootDistance * tanTheta - deltaY);

        if (denominator <= 0.0 || shootDistance <= 0.0) {
            return 0.0;
        }

        double velocity = Math.sqrt(
                (GRAVITY * shootDistance * shootDistance)
                        / denominator
        );

        double calculatedRPM =
                (velocity / WHEEL_RADIUS_METERS)
                        * (60.0 / (2.0 * Math.PI));

        return calculatedRPM * 2.2;
    }

    // =========================================================
    // STOP
    // =========================================================

    @Override
    public void stop() {

        turret.stop();

        if (shooter != null) {
            shooter.setPower(0.0);
        }
    }
}