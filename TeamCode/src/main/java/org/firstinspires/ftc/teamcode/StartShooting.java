package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanism.AprilTagLimelight;
import org.firstinspires.ftc.teamcode.mechanism.DcMotor;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class StartShooting extends OpMode {

    // =========================================================
    // HARDWARE & DRIVER
    // =========================================================

    private Follower follower;
    DcMotor intake = new DcMotor();
    DcMotorEx shooter;

    private ElapsedTime shootTimer;
    private boolean shootTimerStarted = false;

    private Pose currentPose;

    private final AprilTagLimelight atlimelight = new AprilTagLimelight();


    // =========================================================
    // SHOOTER / GEOMETRY CONSTANTS
    // =========================================================

    static final double TARGET_X = 55.0;
    static final double TARGET_Y = 11.75;

    static final double SHOOTING_ANGLE_DEG = 70.8;
    static final double SHOOTING_ANGLE_RAD =
            Math.toRadians(SHOOTING_ANGLE_DEG);

    static final double HIVE_HEIGHT = 55.0;
    static final double SHOOTER_HEIGHT = 10.0;

    static final double WHEEL_RADIUS_METERS = 0.036;
    static final double TICKS_PER_REV = 28.0;

    // Tuned PIDF parameters
    static final double FLYWHEEL_P = 2.0;
    static final double FLYWHEEL_I = 0.0;
    static final double FLYWHEEL_D = 0.5;
    static final double FLYWHEEL_F = 11.7;


    // =========================================================
    // AUTONOMOUS STATES & POSES
    // =========================================================

    public enum PathState {
        DRIVE,
        SHOOT_FIRST,
        BACK,
        TURNONE,
        DRIVETOGARDEN,
        TURNTWO,
        SHOOT_SECOND,
        DONE
    }

    private PathState pathState;
    private boolean stateInitialized = false;

    private final Pose startPose =
            new Pose(8, 11.75, Math.toRadians(0));

    private final Pose shootPose =
            new Pose(13, 11.75, Math.toRadians(0));

    private final Pose startTwoPose =
            new Pose(8, 11.75, Math.toRadians(90));

    private final Pose gardenPose =
            new Pose(8, 60, Math.toRadians(90));

    private PathChain drivePath;
    private PathChain backPath;
    private PathChain gardenPath;


    // =========================================================
    // BUILD PATHS
    // =========================================================

    public void buildPaths() {

        drivePath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(
                        startPose.getHeading(),
                        shootPose.getHeading()
                )
                .build();

        backPath = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, startPose))
                .setLinearHeadingInterpolation(
                        shootPose.getHeading(),
                        startPose.getHeading()
                )
                .build();

        gardenPath = follower.pathBuilder()
                .addPath(new BezierLine(startTwoPose, gardenPose))
                .setLinearHeadingInterpolation(
                        startTwoPose.getHeading(),
                        gardenPose.getHeading()
                )
                .build();
    }


    // =========================================================
    // INIT
    // =========================================================

    @Override
    public void init() {

        intake.init(hardwareMap, "intake");

        follower = Constants.createFollower(hardwareMap);

        buildPaths();

        follower.setPose(startPose);

        shootTimer = new ElapsedTime();

        shooter = hardwareMap.get(
                DcMotorEx.class,
                "shooter"
        );

        // Reset encoder
        shooter.setMode(
                com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER
        );

        shooter.setMode(
                com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER
        );

        // PIDF
        PIDFCoefficients flywheelPIDF =
                new PIDFCoefficients(
                        FLYWHEEL_P,
                        FLYWHEEL_I,
                        FLYWHEEL_D,
                        FLYWHEEL_F
                );

        shooter.setPIDFCoefficients(
                com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER,
                flywheelPIDF
        );

        setPathState(PathState.DRIVE);

        follower.followPath(drivePath, true);
    }


    // =========================================================
    // STATE MANAGEMENT
    // =========================================================

    private void setPathState(PathState newState) {
        pathState = newState;
        stateInitialized = false;
    }


    // =========================================================
    // LOOP
    // =========================================================

    @Override
    public void loop() {

        follower.update();

        currentPose = follower.getPose();

        telemetry.addData("State", pathState);

        telemetry.addData(
                "Heading",
                "%.2f°",
                Math.toDegrees(currentPose.getHeading())
        );

        telemetry.addData(
                "Follower Busy",
                follower.isBusy()
        );

        telemetry.addData(
                "Shooter RPM",
                (shooter.getVelocity() / TICKS_PER_REV) * 60.0
        );

        if (pathState == PathState.SHOOT_FIRST ||
                pathState == PathState.SHOOT_SECOND) {

            if (shootTimerStarted) {
                telemetry.addData(
                        "Shoot Timer",
                        "%.1f s",
                        shootTimer.seconds()
                );
            }
        }

        telemetry.update();


        switch (pathState) {

            // =====================================================
            // DRIVE TO FIRST SHOOTING POSITION
            // =====================================================

            case DRIVE:

                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_FIRST);
                }

                break;


            // =====================================================
            // FIRST SHOOT
            // =====================================================

            case SHOOT_FIRST:

                if (!shootTimerStarted) {

                    double targetRPM = calculateShot(
                            currentPose.getX(),
                            currentPose.getY(),
                            currentPose.getHeading()
                    );

                    double targetTicksPerSec =
                            (targetRPM / 60.0) * TICKS_PER_REV;

                    shooter.setVelocity(targetTicksPerSec);

                    shootTimer.reset();
                    shootTimerStarted = true;
                }

                if (shootTimer.seconds() >= 10.0) {

                    shooter.setVelocity(0);

                    shootTimerStarted = false;

                    setPathState(PathState.BACK);

                    follower.followPath(backPath, true);
                }

                break;


            // =====================================================
            // DRIVE BACK
            // =====================================================

            case BACK:

                if (!follower.isBusy()) {
                    setPathState(PathState.TURNONE);
                }

                break;


            // =====================================================
            // TURN TOWARD GARDEN
            // =====================================================

            case TURNONE:

                if (!stateInitialized) {

                    follower.turnTo(Math.toRadians(90));

                    stateInitialized = true;
                }

                double turnOneTarget = Math.toRadians(90);

                // Normalize angular difference to [-PI, PI]
                double turnOneError = Math.atan2(
                        Math.sin(
                                turnOneTarget -
                                        currentPose.getHeading()
                        ),
                        Math.cos(
                                turnOneTarget -
                                        currentPose.getHeading()
                        )
                );

                telemetry.addData(
                        "TURNONE Heading",
                        "%.2f°",
                        Math.toDegrees(currentPose.getHeading())
                );

                telemetry.addData(
                        "TURNONE Error",
                        "%.2f°",
                        Math.toDegrees(turnOneError)
                );

                telemetry.addData(
                        "TURNONE Busy",
                        follower.isBusy()
                );

                // Transition if either:
                // 1. Follower is no longer busy
                // OR
                // 2. Robot is within 3 degrees of target
                if (!follower.isBusy() ||
                        Math.abs(turnOneError) < Math.toRadians(5)) {

                    intake.setMotorSpeed(-1.0);

                    setPathState(PathState.DRIVETOGARDEN);

                    follower.followPath(gardenPath, true);
                }

                break;


            // =====================================================
            // DRIVE TO GARDEN
            // =====================================================

            case DRIVETOGARDEN:

                if (!follower.isBusy()) {

                    intake.setMotorSpeed(0);

                    setPathState(PathState.TURNTWO);
                }

                break;


            // =====================================================
            // TURN TO -45° FOR SECOND SHOT
            // =====================================================

            case TURNTWO:

                if (!stateInitialized) {

                    follower.turnTo(Math.toRadians(-45));

                    stateInitialized = true;
                }

                double turnTwoTarget = Math.toRadians(-45);

                // Normalize angular difference to [-PI, PI]
                double turnTwoError = Math.atan2(
                        Math.sin(
                                turnTwoTarget -
                                        currentPose.getHeading()
                        ),
                        Math.cos(
                                turnTwoTarget -
                                        currentPose.getHeading()
                        )
                );

                telemetry.addData(
                        "TURNTWO Heading",
                        "%.2f°",
                        Math.toDegrees(currentPose.getHeading())
                );

                telemetry.addData(
                        "TURNTWO Target",
                        "-45.00°"
                );

                telemetry.addData(
                        "TURNTWO Error",
                        "%.2f°",
                        Math.toDegrees(turnTwoError)
                );

                telemetry.addData(
                        "TURNTWO Busy",
                        follower.isBusy()
                );

                if (!follower.isBusy() ||
                        Math.abs(turnTwoError) < Math.toRadians(3)) {

                    setPathState(PathState.SHOOT_SECOND);
                }

                break;


            // =====================================================
            // SECOND SHOOT
            // =====================================================

            case SHOOT_SECOND:

                if (!shootTimerStarted) {

                    double targetRPM = calculateShot(
                            currentPose.getX(),
                            currentPose.getY(),
                            currentPose.getHeading()
                    );

                    double targetTicksPerSec =
                            (targetRPM / 60.0) * TICKS_PER_REV;

                    shooter.setVelocity(targetTicksPerSec);

                    shootTimer.reset();
                    shootTimerStarted = true;
                }

                if (shootTimer.seconds() >= 10.0) {

                    shooter.setVelocity(0);

                    shootTimerStarted = false;

                    setPathState(PathState.DONE);
                }

                break;


            // =====================================================
            // DONE
            // =====================================================

            case DONE:

                shooter.setVelocity(0);
                intake.setMotorSpeed(0);

                break;
        }
    }


    // =========================================================
    // SHOOTING CALCULATION
    // =========================================================

    private double calculateShot(
            double robotX,
            double robotY,
            double robotHeading
    ) {

        double dx = TARGET_X - robotX;
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

        if (denominator <= 0) {
            return 0.0;
        }

        double velocity =
                Math.sqrt(
                        (9.81 * shootX * shootX) /
                                denominator
                );

        double targetRPM =
                (velocity / WHEEL_RADIUS_METERS) *
                        (60.0 / (2.0 * Math.PI));

        return targetRPM * 2.5;
    }
}