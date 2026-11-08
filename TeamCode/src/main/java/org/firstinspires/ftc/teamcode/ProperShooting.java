package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.mechanism.Drive;
import org.firstinspires.ftc.teamcode.mechanism.ServoClass;
import org.firstinspires.ftc.teamcode.mechanism.CalculateShot;

@TeleOp
public class ProperShooting extends OpMode {

    // =========================================================
    // HARDWARE
    // =========================================================

    private final Drive drive = new Drive();

    private final org.firstinspires.ftc.teamcode.mechanism.DcMotor intake =
            new org.firstinspires.ftc.teamcode.mechanism.DcMotor();

    private DcMotorEx shooter;

    private final ServoClass angleServo = new ServoClass();
    private final ServoClass whichBall1 = new ServoClass();
    private final ServoClass whichBall2 = new ServoClass();

    private GoBildaPinpointDriver pinpoint;

    // =========================================================
    // SHOOTING
    // =========================================================

    private final CalculateShot calculateShot = new CalculateShot();

    private  double RPM = 3000.0;
    private static final double TICKS_PER_REV = 28.0;

    private static final double FLYWHEEL_P = 2.0;
    private static final double FLYWHEEL_I = 0.0;
    private static final double FLYWHEEL_D = 0.5;
    private static final double FLYWHEEL_F = 10.6;

    private static final double SMALL_POSITION_1 = 1.0;
    private static final double SMALL_POSITION_2 = 0.8;

    private static final double BIG_POSITION_1 = 0.0;
    private static final double BIG_POSITION_2 = 0.2;

    private boolean shooterActive = false;
    private boolean lastAState = false;
    private boolean ball = true;

    private double shootingAngleDeg = 0.0;

    // =========================================================
    // ROBOT POSITION
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

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        angleServo.init(hardwareMap, "angular servo");
        whichBall1.init(hardwareMap, "right servo");
        whichBall2.init(hardwareMap, "left servo");

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients flywheelPIDF = new PIDFCoefficients(
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

        pinpoint.setPosition(currentPose);

        telemetry.addLine("ProperShooting initialized");
        telemetry.update();
    }

    // =========================================================
    // LOOP
    // =========================================================

    @Override
    public void loop() {

        // -----------------------------------------------------
        // UPDATE POSITION
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
        // INTAKE
        // -----------------------------------------------------

        if (gamepad1.left_bumper) {
            intake.setMotorSpeed(1.0);
        } else if (gamepad1.right_bumper) {
            intake.setMotorSpeed(-1.0);
        } else {
            intake.setMotorSpeed(0.0);
        }

        // -----------------------------------------------------
        // SHOOTER TOGGLE
        // -----------------------------------------------------

        if (gamepad1.a && !lastAState) {
            shooterActive = !shooterActive;
        }

        lastAState = gamepad1.a;

        // -----------------------------------------------------
        // BALL SELECTION
        // -----------------------------------------------------

        if (gamepad1.dpad_up) {
            ball = true;
        } else if (gamepad1.dpad_down) {
            ball = false;
        }

        if (ball) {
            whichBall1.setServoPos(SMALL_POSITION_1);
            whichBall2.setServoPos(SMALL_POSITION_2);
        } else {
            whichBall1.setServoPos(BIG_POSITION_1);
            whichBall2.setServoPos(BIG_POSITION_2);
        }

        // -----------------------------------------------------
        // SHOOTER CONTROL
        // -----------------------------------------------------

        if (shooterActive) {

            // Calculate angle using the separate CalculateShot class
            double angle = calculateShot.calculateShot(
                    robotX,
                    robotY,
                    ball,
                    RPM
            );

            shootingAngleDeg = Math.toDegrees(angle);
            while (shootingAngleDeg > 60.0) {
                RPM += 200;
                shootingAngleDeg = Math.toDegrees(calculateShot.calculateShot(
                        robotX,
                        robotY,
                        ball,
                        RPM
                ));
            }
            while (shootingAngleDeg < 50.0) {
                RPM -= 200;
                shootingAngleDeg = Math.toDegrees(calculateShot.calculateShot(
                        robotX,
                        robotY,
                        ball,
                        RPM
                ));
            }

            double servoPosition = Range.scale(shootingAngleDeg, 50, 60, 1, 0);;

            angleServo.setServoPos(servoPosition);

            double targetTicksPerSec =
                    (RPM / 60.0) * TICKS_PER_REV;

            shooter.setVelocity(targetTicksPerSec);
            telemetry.addData("target", targetTicksPerSec);
            telemetry.addData("acc", shooter.getVelocity());
            telemetry.update();

        } else {
            shooter.setVelocity(0.0);
            RPM = 3000;
        }

        // -----------------------------------------------------
        // TELEMETRY
        // -----------------------------------------------------

        double actualRPM =
                (shooter.getVelocity() / TICKS_PER_REV) * 60.0;

        telemetry.addData("Shooter State",
                shooterActive ? "ACTIVE" : "OFF");

        telemetry.addData("Actual RPM", "%.2f", actualRPM);
        telemetry.addData("Target RPM", "%.2f", RPM);

        telemetry.addData("Selected Ball",
                ball ? "SMALL" : "BIG");

        telemetry.addData("Robot X", "%.2f in", robotX);
        telemetry.addData("Robot Y", "%.2f in", robotY);
        telemetry.addData("Robot Heading", "%.2f deg", robotHeading);

        telemetry.addData("Shooting Angle", "%.2f deg",
                shootingAngleDeg);

        telemetry.addData("Loop Runtime", "%.2f s", getRuntime());

        telemetry.update();
    }
}