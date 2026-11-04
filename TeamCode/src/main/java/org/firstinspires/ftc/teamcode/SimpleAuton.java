package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.mechanism.DcMotor;
import org.firstinspires.ftc.teamcode.mechanism.Drive;

import com.pedropathing.geometry.Pose;


@Autonomous(name = "Last Minute Auton")
public class SimpleAuton extends OpMode {

    // HARDWARE

    private final Drive drive = new Drive();
    private final DcMotor intake = new DcMotor();

    private DcMotorEx shooter;

    private ElapsedTime timer = new ElapsedTime();


    // CONSTANTS

    private static final double DRIVE_TIME = 1.0;
    private static final double DRIVE_POWER = 0.3;

    private static final double SPINUP_TIME = 1.5;
    private static final double FEED_TIME = 5.0;

    private static final double INTAKE_POWER = 1.0;

    private static final double HIVE_HEIGHT = 59.55;
    private static final double SHOOTER_HEIGHT = 8.0;

    private static final double SHOOTING_ANGLE =
            Math.toRadians(60);

    private static final double WHEEL_RADIUS = 0.036;
    private static final double TICKS_PER_REV = 28.0;

    private static final double FLYWHEEL_P = 2.0;
    private static final double FLYWHEEL_I = 0.0;
    private static final double FLYWHEEL_D = 0.5;
    private static final double FLYWHEEL_F = 11.7;


    // TARGET

    private final Pose targetPose =
            new Pose(55, 96, 0);


    // STATES

    private enum State {
        DRIVE_FORWARD,
        SPINUP,
        SHOOT,
        DONE
    }

    private State state = State.DRIVE_FORWARD;


    // INITIALIZATION

    @Override
    public void init() {

        drive.init(hardwareMap);

        intake.init(hardwareMap, "intake");

        shooter = hardwareMap.get(
                DcMotorEx.class,
                "shooter"
        );

        shooter.setMode(
                com.qualcomm.robotcore.hardware.DcMotor.RunMode
                        .STOP_AND_RESET_ENCODER
        );

        shooter.setMode(
                com.qualcomm.robotcore.hardware.DcMotor.RunMode
                        .RUN_USING_ENCODER
        );

        shooter.setPIDFCoefficients(
                com.qualcomm.robotcore.hardware.DcMotor.RunMode
                        .RUN_USING_ENCODER,
                new PIDFCoefficients(
                        FLYWHEEL_P,
                        FLYWHEEL_I,
                        FLYWHEEL_D,
                        FLYWHEEL_F
                )
        );

        shooter.setVelocity(0);

        intake.setMotorSpeed(0);

        telemetry.addLine("Last Minute Auton Ready");
    }


    @Override
    public void start() {

        state = State.DRIVE_FORWARD;

        timer.reset();
    }


    // LOOP

    @Override
    public void loop() {

        switch (state) {

            // =================================
            // DRIVE FORWARD FOR 1 SECOND
            // =================================

            case DRIVE_FORWARD:

                drive.setPower(
                        -1,
                        0,
                        0,
                        DRIVE_POWER
                );

                if (timer.seconds() >= DRIVE_TIME) {

                    drive.setPower(0, 0, 0, 0);

                    timer.reset();

                    state = State.SPINUP;
                }

                break;


            // =================================
            // SPIN UP SHOOTER
            // =================================

            case SPINUP:

                double targetRPM = calculateShot();

                double targetTicksPerSecond =
                        (targetRPM / 60.0) * TICKS_PER_REV;

                shooter.setVelocity(targetTicksPerSecond);

                if (timer.seconds() >= SPINUP_TIME) {

                    intake.setMotorSpeed(INTAKE_POWER);

                    timer.reset();

                    state = State.SHOOT;
                }

                break;


            // =================================
            // FEED BALLS
            // =================================

            case SHOOT:

                intake.setMotorSpeed(INTAKE_POWER);

                if (timer.seconds() >= FEED_TIME) {

                    intake.setMotorSpeed(0);

                    shooter.setVelocity(0);

                    state = State.DONE;
                }

                break;


            // =================================
            // DONE
            // =================================

            case DONE:

                drive.setPower(0, 0, 0, 0);

                intake.setMotorSpeed(0);

                shooter.setVelocity(0);

                break;
        }


        telemetry.addData("State", state);
        telemetry.addData("Timer", "%.2f", timer.seconds());
        telemetry.addData("Shooter Velocity", shooter.getVelocity());

        telemetry.update();
    }


    // SHOOTING CALCULATION

    private double calculateShot() {

        double dx = 0;

        double dy = 43;

        double distanceMeters =
                0.0254 * Math.hypot(dx, dy);

        double heightDifference =
                (HIVE_HEIGHT - SHOOTER_HEIGHT) * 0.0254;

        double cosTheta = Math.cos(SHOOTING_ANGLE);
        double tanTheta = Math.tan(SHOOTING_ANGLE);

        double denominator =
                2 * cosTheta * cosTheta *
                        (distanceMeters * tanTheta - heightDifference);

        if (denominator <= 0) {
            return 0;
        }

        double velocity = Math.sqrt(
                (9.81 * distanceMeters * distanceMeters)
                        / denominator
        );

        double rpm =
                (velocity / WHEEL_RADIUS)
                        * (60.0 / (2.0 * Math.PI));

        return rpm * 1.8;
    }


    @Override
    public void stop() {

        drive.setPower(0, 0, 0, 0);

        intake.setMotorSpeed(0);

        if (shooter != null) {
            shooter.setVelocity(0);
        }
    }
}