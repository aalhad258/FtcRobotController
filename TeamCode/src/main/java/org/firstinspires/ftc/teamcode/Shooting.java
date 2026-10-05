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

@TeleOp
public class Shooting extends OpMode {

    // =========================================================
    // HARDWARE
    // =========================================================

    Drive drive = new Drive();
    org.firstinspires.ftc.teamcode.mechanism.DcMotor intake = new org.firstinspires.ftc.teamcode.mechanism.DcMotor();

    // DcMotorEx for closed-loop PID velocity control
    DcMotorEx shooter;

    GoBildaPinpointDriver pinpoint;


    // =========================================================
    // SHOOTER / GEOMETRY
    // =========================================================

    // Fixed shooting angle
    static final double SHOOTING_ANGLE_DEG = 70.8;
    static final double SHOOTING_ANGLE_RAD = Math.toRadians(SHOOTING_ANGLE_DEG);

    // Geometry heights (in inches)
    static final double HIVE_HEIGHT = 55.0;      // Target height in inches
    static final double SHOOTER_HEIGHT = 10.0;   // Height of flywheel axle off ground in inches

    // Flywheel mechanical specs
    static final double WHEEL_RADIUS_METERS = 0.036; // 72mm diameter wheel (36mm radius)
    static final double TICKS_PER_REV = 28.0;         // goBILDA 6000 RPM (1:1) motor

    // Tuned PIDF parameters for high-RPM / low-CPR 1:1 motor
    // F = 32767 / (6000 RPM / 60 * 28 ticks/rev) = 11.7
    static final double FLYWHEEL_P = 2.0;
    static final double FLYWHEEL_I = 0.0;
    static final double FLYWHEEL_D = 0.5;
    static final double FLYWHEEL_F = 11.7;

    // Toggle variables for shooter control
    private boolean shooterActive = false;
    private boolean lastAState = false;

    // Telemetry tracking
    private double calculatedRPM = 0.0;
    private double actualRPM = 0.0;


    // =========================================================
    // ROBOT POSITION
    // =========================================================

    Pose2D currentPose =
            new Pose2D(
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

        // -------------------------
        // DRIVE
        // -------------------------

        drive.init(hardwareMap);


        // -------------------------
        // INTAKE
        // -------------------------

        intake.init(hardwareMap, "intake");


        // -------------------------
        // SHOOTER
        // -------------------------

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        // Reset encoder position and enable internal PID loop
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Apply tuned PIDF coefficients to resolve 3400 RPM jump issue
        PIDFCoefficients flywheelPIDF = new PIDFCoefficients(
                FLYWHEEL_P,
                FLYWHEEL_I,
                FLYWHEEL_D,
                FLYWHEEL_F
        );
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheelPIDF);


        // -------------------------
        // PINPOINT
        // -------------------------

        pinpoint = hardwareMap.get(
                GoBildaPinpointDriver.class,
                "pinpoint"
        );

        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
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


        telemetry.addLine("ShootDriveTest: init complete");
        telemetry.update();

        pinpoint.setPosition(currentPose);
    }


    // =========================================================
    // LOOP
    // =========================================================

    @Override
    public void loop() {

        // =====================================================
        // UPDATE PINPOINT
        // =====================================================

        pinpoint.update();

        currentPose = pinpoint.getPosition();


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
        // ROBOT POSITION & SHOOTER TOGGLE
        // =====================================================

        double robotX = currentPose.getX(DistanceUnit.INCH);
        double robotY = currentPose.getY(DistanceUnit.INCH);
        double robotHeading = currentPose.getHeading(AngleUnit.DEGREES);

        // Toggle shooter state on button press (rising edge)
        if (gamepad1.a && !lastAState) {
            shooterActive = !shooterActive;
        }
        lastAState = gamepad1.a;

        if (shooterActive) {
            calculatedRPM = calculateShot(
                    robotX,
                    robotY,
                    robotHeading
            );

            // Convert target RPM (w) to Encoder Ticks per Second
            double targetTicksPerSec = (calculatedRPM / 60.0) * TICKS_PER_REV;

            // Command closed-loop PID velocity
            shooter.setVelocity(targetTicksPerSec);
        } else {
            calculatedRPM = 0.0;
            shooter.setVelocity(0.0);
        }

        // Calculate real-time actual RPM from encoder readings
        actualRPM = (shooter.getVelocity() / TICKS_PER_REV) * 60.0;


        // =====================================================
        // TELEMETRY
        // =====================================================

        telemetry.addData("axial", axial);
        telemetry.addData("lateral", lateral);
        telemetry.addData("yaw", yaw);

        telemetry.addData(
                "Shooter State",
                shooterActive ? "ACTIVE" : "OFF"
        );
        telemetry.addData(
                "Target RPM (w)",
                "%.2f RPM",
                calculatedRPM
        );
        telemetry.addData(
                "Actual RPM",
                "%.2f RPM",
                actualRPM
        );

        telemetry.addLine("--------------------");

        telemetry.addData(
                "Robot X",
                "%.2f in",
                robotX
        );

        telemetry.addData(
                "Robot Y",
                "%.2f in",
                robotY
        );

        telemetry.addData(
                "Robot Heading",
                "%.2f deg",
                robotHeading
        );

        telemetry.addLine("--------------------");

        telemetry.addData(
                "Shooting Angle",
                "%.2f deg",
                SHOOTING_ANGLE_DEG
        );

        telemetry.addData(
                "HIVE Height",
                "%.2f in",
                HIVE_HEIGHT
        );

        telemetry.addLine("--------------------");

        telemetry.addData(
                "loop runtime (s)",
                getRuntime()
        );

        telemetry.update();
    }


    // =========================================================
    // SHOOTING CALCULATION
    // =========================================================

    private double calculateShot(
            double robotX,
            double robotY,
            double robotHeading
    ) {
        // Horizontal distance to target (converted to meters)
        double dx = 45.0 - robotX;
        double dy = 0.0 - robotY;
        double shootX = 0.0254 * Math.hypot(dx, dy);

        // Vertical displacement delta (converted to meters)
        double deltaY = (HIVE_HEIGHT - SHOOTER_HEIGHT) * 0.0254;

        // Kinematic trajectory formula: v = sqrt( (g * x^2) / (2 * cos^2(theta) * (x * tan(theta) - y)) )
        double cosTheta = Math.cos(SHOOTING_ANGLE_RAD);
        double tanTheta = Math.tan(SHOOTING_ANGLE_RAD);

        double denominator = 2 * cosTheta * cosTheta * (shootX * tanTheta - deltaY);

        // Safety check to avoid NaN/negative square roots if too close/far
        if (denominator <= 0) {
            return 0.0;
        }

        double velocity = Math.sqrt((9.81 * shootX * shootX) / denominator);

        // Convert exit linear velocity (m/s) to target RPM
        // v = w * r  =>  w = v / r (rad/s)  =>  RPM = (v / r) * (60 / 2pi)
        double targetRPM = (velocity / WHEEL_RADIUS_METERS) * (60.0 / (2.0 * Math.PI));

        return targetRPM * 2.5;
    }
}