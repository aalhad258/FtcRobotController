package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.mechanism.Drive;
import org.firstinspires.ftc.teamcode.mechanism.TurretMechanism;
import org.firstinspires.ftc.teamcode.mechanism.PoseStorage;

@TeleOp(name = "TeleOp Class")
public class TeleOpClass extends OpMode {

    // =========================================================
    // HARDWARE
    // =========================================================

    private Drive drive = new Drive();

    private org.firstinspires.ftc.teamcode.mechanism.DcMotor intake =
            new org.firstinspires.ftc.teamcode.mechanism.DcMotor();

    private DcMotorEx shooter;
    private GoBildaPinpointDriver pinpoint;
    private TurretMechanism turret;


    // =========================================================
    // FIELD CONFIGURATION
    // =========================================================

    private static final double START_Y = 50.0;
    private static final double START_X = 8.0;
    private static final double START_HEADING = 0.0;

    private static final double TARGET_Y = 50.0;

    private static final double TARGET_X_1 = 54.0;
    private static final double TARGET_X_2 = 89.0;

    private double targetX = TARGET_X_1;


    // =========================================================
    // SHOOTER CONFIGURATION
    // =========================================================

    static final double SHOOTING_ANGLE_DEG = 70.8;
    static final double SHOOTING_ANGLE_RAD =
            Math.toRadians(SHOOTING_ANGLE_DEG);

    static final double HIVE_HEIGHT = 63.0;
    static final double SHOOTER_HEIGHT = 10.0;

    static final double WHEEL_RADIUS_METERS = 0.036;
    static final double TICKS_PER_REV = 28.0;

    static final double FLYWHEEL_P = 2.0;
    static final double FLYWHEEL_I = 0.0;
    static final double FLYWHEEL_D = 0.5;
    static final double FLYWHEEL_F = 9.85;


    // =========================================================
    // STATES
    // =========================================================

    private boolean shooterActive = false;

    private boolean lastAState = false;
    private boolean lastXState = false;

    private double calculatedRPM = 0.0;
    private double actualRPM = 0.0;


    // =========================================================
    // ROBOT POSITION
    // =========================================================

    private Pose2D currentPose;


    // =========================================================
    // TURRET TARGETING
    // =========================================================

    private double targetFieldAngle = 0.0;
    private double desiredTurretAngle = 0.0;
    private double turretError = 0.0;


    // =========================================================
    // INITIALIZATION
    // =========================================================

    @Override
    public void init() {

        // DRIVE
        drive.init(hardwareMap);


        // INTAKE
        intake.init(hardwareMap, "intake");


        // SHOOTER
        shooter = hardwareMap.get(
                DcMotorEx.class,
                "shooter"
        );

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients flywheelPIDF =
                new PIDFCoefficients(
                        FLYWHEEL_P,
                        FLYWHEEL_I,
                        FLYWHEEL_D,
                        FLYWHEEL_F
                );

        shooter.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                flywheelPIDF
        );


        // TURRET
        turret = new TurretMechanism();
        turret.init(hardwareMap);


        // PINPOINT
        pinpoint = hardwareMap.get(
                GoBildaPinpointDriver.class,
                "pinpoint"
        );

        pinpoint.setOffsets(
                5,
                0,
                DistanceUnit.INCH
        );

        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );

        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        // Reset the Pinpoint's position and IMU.
        // The actual starting field position is assigned in start().
        pinpoint.resetPosAndIMU();


        telemetry.addLine("SHOOTING AUTO TURRET INITIALIZED");
        telemetry.addLine("Drive: Ready");
        telemetry.addLine("Intake: Ready");
        telemetry.addLine("Shooter: Ready");
        telemetry.addLine("Turret: Ready");
        telemetry.addLine("Pinpoint: Ready");

        telemetry.update();
    }


    // =========================================================
    // START
    // =========================================================

    @Override
    public void start() {

        double startX = START_X;
        double startY = START_Y;
        double startHeading = START_HEADING;

        if (PoseStorage.hasPose) {
            startX = PoseStorage.x;
            startY = PoseStorage.y;
            startHeading = PoseStorage.headingDeg;
        }

        pinpoint.setPosition(
                new Pose2D(
                        DistanceUnit.INCH,
                        startX,
                        startY,
                        AngleUnit.DEGREES,
                        startHeading
                )
        );

        pinpoint.update();
        currentPose = pinpoint.getPosition();
    }


    // =========================================================
    // MAIN LOOP
    // =========================================================

    @Override
    public void loop() {

        // UPDATE PINPOINT
        pinpoint.update();

        currentPose = pinpoint.getPosition();

        double robotX = currentPose.getX(DistanceUnit.INCH);
        double robotY = currentPose.getY(DistanceUnit.INCH);

        double robotHeading =
                currentPose.getHeading(AngleUnit.DEGREES);


        // =====================================================
        // TARGET POSITION TOGGLE (GAMEPAD X)
        // =====================================================

        if (gamepad1.x && !lastXState) {

            if (targetX == TARGET_X_1) {
                targetX = TARGET_X_2;
            } else {
                targetX = TARGET_X_1;
            }
        }

        lastXState = gamepad1.x;


        // =====================================================
        // AUTOMATIC TURRET ALIGNMENT
        // =====================================================

        double dx = targetX - robotX;
        double dy = TARGET_Y - robotY;

        targetFieldAngle = Math.toDegrees(
                Math.atan2(dy, dx)
        );

        desiredTurretAngle =
                targetFieldAngle - robotHeading;

        // Normalize angle to -180 to +180 degrees.
        desiredTurretAngle = Math.toDegrees(
                Math.atan2(
                        Math.sin(Math.toRadians(desiredTurretAngle)),
                        Math.cos(Math.toRadians(desiredTurretAngle))
                )
        );

        turret.update(desiredTurretAngle);

        turretError =
                desiredTurretAngle - turret.getTurretPositionDegrees();


        // =====================================================
        // DRIVE
        // =====================================================

        double axial = -gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;
        drive.setPower(
                axial,
                lateral,
                yaw,
                1.0
        );


        // =====================================================
        // INTAKE
        // =====================================================

        if (gamepad1.left_bumper) {

            intake.setMotorSpeed(1.0);

        } else if (gamepad1.right_bumper) {

            intake.setMotorSpeed(-1.0);

        } else {

            intake.setMotorSpeed(0.0);
        }


        // =====================================================
        // SHOOTER TOGGLE (GAMEPAD A)
        // =====================================================

        if (gamepad1.a && !lastAState) {
            shooterActive = !shooterActive;
        }

        lastAState = gamepad1.a;


        // =====================================================
        // DYNAMIC SHOOTER RPM
        // =====================================================

        if (shooterActive) {

            calculatedRPM = calculateShot(
                    robotX,
                    robotY
            );

            double targetTicksPerSec =
                    (calculatedRPM / 60.0) * TICKS_PER_REV;

            shooter.setVelocity(targetTicksPerSec);

        } else {

            calculatedRPM = 0.0;

            shooter.setVelocity(0.0);
        }


        actualRPM =
                (shooter.getVelocity() / TICKS_PER_REV) * 60.0;


        // =====================================================
        // TELEMETRY
        // =====================================================

        telemetry.addLine("===== DRIVE =====");

        telemetry.addData("Axial", axial);
        telemetry.addData("Lateral", lateral);
        telemetry.addData("Yaw", yaw);

        telemetry.addLine("===== POSITION =====");

        telemetry.addData("Robot X", "%.2f", robotX);
        telemetry.addData("Robot Y", "%.2f", robotY);
        telemetry.addData("Robot Heading", "%.2f", robotHeading);

        telemetry.addData(
                "Pinpoint Status",
                pinpoint.getDeviceStatus()
        );

        telemetry.addLine("===== TURRET =====");

        telemetry.addData(
                "Target Field Angle",
                "%.2f deg",
                targetFieldAngle
        );

        telemetry.addData(
                "Desired Turret Angle",
                "%.2f deg",
                desiredTurretAngle
        );

        telemetry.addData(
                "Actual Turret Angle",
                "%.2f deg",
                turret.getTurretPositionDegrees()
        );

        telemetry.addData(
                "Turret Error",
                "%.2f deg",
                turretError
        );

        telemetry.addData("Turret kP", turret.getKP());
        telemetry.addData("Turret kD", turret.getKD());

        telemetry.addLine("===== SHOOTER =====");

        telemetry.addData(
                "Shooter State",
                shooterActive ? "ACTIVE" : "OFF"
        );

        telemetry.addData(
                "Target RPM",
                "%.2f",
                calculatedRPM
        );

        telemetry.addData(
                "Actual RPM",
                "%.2f",
                actualRPM
        );

        telemetry.addLine("===== TARGET =====");

        telemetry.addData("Target X", targetX);
        telemetry.addData("Target Y", TARGET_Y);

        telemetry.addData(
                "Target Position",
                targetX == TARGET_X_1 ? "CLOSE" : "FAR"
        );

        telemetry.addData(
                "Shooting Angle",
                SHOOTING_ANGLE_DEG
        );

        telemetry.addData("Hive Height", HIVE_HEIGHT);

        telemetry.addLine("------------------------");

        telemetry.addData("Loop Runtime", "%.4f s", getRuntime());
        telemetry.addData(
                "Turret Power",
                "%.3f",
                turret.getPower()
        );

        telemetry.update();
    }


    // =========================================================
    // PROJECTILE MOTION CALCULATION
    // =========================================================

    private double calculateShot(
            double robotX,
            double robotY
    ) {

        double dx = targetX - robotX;
        double dy = TARGET_Y - robotY;

        double shootX =
                0.0254 * Math.hypot(dx, dy);

        double deltaY =
                (HIVE_HEIGHT - SHOOTER_HEIGHT) * 0.0254;

        double cosTheta =
                Math.cos(SHOOTING_ANGLE_RAD);

        double tanTheta =
                Math.tan(SHOOTING_ANGLE_RAD);

        double denominator =
                2 * cosTheta * cosTheta *
                        (shootX * tanTheta - deltaY);

        if (denominator <= 0 || shootX <= 0) {
            return 0.0;
        }

        double velocity = Math.sqrt(
                (9.81 * shootX * shootX) / denominator
        );

        double targetRPM =
                (velocity / WHEEL_RADIUS_METERS) *
                        (60.0 / (2.0 * Math.PI));

        return targetRPM * 2.8;
    }


    // =========================================================
    // STOP
    // =========================================================

    @Override
    public void stop() {

        drive.setPower(0, 0, 0, 0);

        intake.setMotorSpeed(0);

        shooter.setPower(0);

        turret.stop();

        PoseStorage.hasPose = false;
    }
}