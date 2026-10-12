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
import org.firstinspires.ftc.teamcode.mechanism.ServoClass;

import java.time.chrono.HijrahChronology;

@TeleOp
public class ProperShooting extends OpMode {

    // =========================================================
    // HARDWARE
    // =========================================================

    Drive drive = new Drive();
    org.firstinspires.ftc.teamcode.mechanism.DcMotor intake =
            new org.firstinspires.ftc.teamcode.mechanism.DcMotor();

    DcMotorEx shooter;
    ServoClass angleServo = new ServoClass();
    ServoClass whichBall = new ServoClass();

    GoBildaPinpointDriver pinpoint;


    // =========================================================
    // SHOOTER / GEOMETRY
    // =========================================================

    double SHOOTING_ANGLE_DEG;

    static final double HIVE_HEIGHT = 63.0;
    static final double SHOOTER_HEIGHT = 10.0;

    static final double WHEEL_RADIUS_METERS = 0.036;
    static final double TICKS_PER_REV = 28.0;

    static final double FLYWHEEL_P = 2.0;
    static final double FLYWHEEL_I = 0.0;
    static final double FLYWHEEL_D = 0.5;
    static final double FLYWHEEL_F = 10.6;
    static final double CONSTANT = 2.5;

    private boolean shooterActive = false;
    private boolean lastAState = false;

    static final double RPM = 3000;
    static double V_INIT =
            WHEEL_RADIUS_METERS * RPM * 2 * 3.1415 / 60.0;

    static final double SMALL_CONSTANT = 2.5;
    static final double BIG_CONSTANT = 3.0;
    static final double SMALL_POSITION = 0;
    static final double BIG_POSITION = 0.5;

    boolean ball = true;

    // Prevents unlimited velocity increases
    static final int MAX_VELOCITY_RETRIES = 50;
    static final double VELOCITY_INCREMENT = 500;


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

        drive.init(hardwareMap);

        intake.init(hardwareMap, "intake");

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        angleServo.init(hardwareMap);
        whichBall.init(hardwareMap);

        shooter.setMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER
        );
        shooter.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER
        );

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
        double robotHeading =
                currentPose.getHeading(AngleUnit.DEGREES);

        // Toggle shooter state on A press
        if (gamepad1.a && !lastAState) {
            shooterActive = !shooterActive;
        }
        lastAState = gamepad1.a;


        // =====================================================
        // SMALL / BIG BALL SELECTION
        // =====================================================

        if (gamepad1.dpad_up) {
            ball = true;
        } else if (gamepad1.dpad_down) {
            ball = false;
        }

        if (ball) {
            whichBall.setServoPos(SMALL_POSITION);
        } else {
            whichBall.setServoPos(BIG_POSITION);
        }


        // =====================================================
        // SHOOTER CONTROL
        // =====================================================

        if (shooterActive) {

            double angle = calculateShot(robotX, robotY, ball);

            SHOOTING_ANGLE_DEG = Math.toDegrees(angle);

            double position = SHOOTING_ANGLE_DEG / 180.0;
            angleServo.setServoPos(position);

            double targetTicksPerSec =
                    (RPM / 60.0) * TICKS_PER_REV;

            shooter.setVelocity(targetTicksPerSec);

        } else {
            shooter.setVelocity(0.0);
        }


        // Calculate actual RPM
        double actualRPM =
                (shooter.getVelocity() / TICKS_PER_REV) * 60.0;


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
                "Actual RPM",
                "%.2f RPM",
                actualRPM
        );

        telemetry.addData(
                "Target RPM",
                "%.2f RPM",
                RPM
        );

        telemetry.addData(
                "Selected Ball",
                ball ? "SMALL" : "BIG"
        );

        telemetry.addData(
                "V_INIT",
                "%.2f m/s",
                V_INIT
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
                "Hive Height",
                "%.2f in",
                HIVE_HEIGHT
        );

        telemetry.addLine("--------------------");

        telemetry.addData(
                "Loop runtime (s)",
                getRuntime()
        );

        telemetry.update();
    }


    // =========================================================
    // PROJECTILE CALCULATION
    // =========================================================

    private double calculateShot(
            double robotX,
            double robotY,
            boolean chooseBall
    ) {

        double dx = 45.0 - robotX;
        double dy = 0.0 - robotY;
        double shootX = 0.0254 * Math.hypot(dx, dy);

        double deltaY =
                (HIVE_HEIGHT - SHOOTER_HEIGHT) * 0.0254;

        double V_INITIAL;

        if (chooseBall) {
            V_INITIAL = V_INIT / SMALL_CONSTANT;
        } else {
            V_INITIAL = V_INIT / BIG_CONSTANT;
        }

        double aConstant =
                9.81 * shootX * shootX
                        / (2 * V_INITIAL * V_INITIAL);

        double bConstant = -shootX;

        double cConstant =
                9.81 * shootX * shootX
                        / (2 * V_INITIAL * V_INITIAL)
                        + deltaY;

        double discriminant =
                bConstant * bConstant
                        - 4 * aConstant * cConstant;

        int retries = 0;

        // Increase V_INIT until a real solution is available,
        // with a limit to prevent infinite recursion.
        while (discriminant < 0
                && retries < MAX_VELOCITY_RETRIES) {

            V_INIT += VELOCITY_INCREMENT;
            retries++;

            if (chooseBall) {
                V_INITIAL = V_INIT / SMALL_CONSTANT;
            } else {
                V_INITIAL = V_INIT / BIG_CONSTANT;
            }

            aConstant =
                    9.81 * shootX * shootX
                            / (2 * V_INITIAL * V_INITIAL);

            bConstant = -shootX;

            cConstant =
                    9.81 * shootX * shootX
                            / (2 * V_INITIAL * V_INITIAL)
                            + deltaY;

            discriminant =
                    bConstant * bConstant
                            - 4 * aConstant * cConstant;
        }

        // If there is still no real solution, return 0
        // instead of continuing indefinitely.
        if (discriminant < 0) {
            return 0.0;
        }

        double tangent =
                (-bConstant + Math.sqrt(discriminant))
                        / (2 * aConstant);

        return Math.atan(tangent);
    }
}