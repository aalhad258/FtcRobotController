package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.mechanism.AprilTagLimelight;
import org.firstinspires.ftc.teamcode.mechanism.DcMotor;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.mechanism.TurretMechanism;
import org.firstinspires.ftc.teamcode.mechanism.PoseStorage;

@Autonomous
public class AutonBlue extends OpMode {

    // =========================================================
    // HARDWARE
    // =========================================================

    private Follower follower;

    private DcMotor intake = new DcMotor();
    private DcMotorEx shooter;

    private final AprilTagLimelight atlimelight =
            new AprilTagLimelight();

    private ElapsedTime shootTimer;
    private ElapsedTime tagCheckTimer;

    private Pose currentPose;

    private boolean feedingStarted = false;

    // Telemetry
    private String shotMessage = "";
    private String postShotMessage = "";
    private String routeMessage = "";
    private TurretMechanism turret;
    private double turretErrorDeg = 0.0;


    // =========================================================
    // TAG CHECK RESULTS
    // =========================================================

    private boolean sawTagAfterFirstShot = false;
    private boolean sawTagAfterSecondShot = false;


    // =========================================================
    // FIELD CONSTANTS
    // =========================================================
    // These are the current working route coordinates.
    // They must be verified against your actual field setup.
    //
    // PedroPathing convention for your setup:
    // X = vertical
    // Y = horizontal
    // =========================================================

    static final double Y = 94;

    static final double FIELD_SIZE_IN = 144.0;

    static final double FLOWER_X = 60.0;

    static final double SHOOT_X = 13.0;

    static final double DIFF_SHOT_X =
            FIELD_SIZE_IN - SHOOT_X; // 131.0


    // =========================================================
    // SHOOTER CONSTANTS
    // =========================================================

    // Current working target coordinates.
    // Verify these against the actual center of the Hive opening.
    static final double TARGET_X_CLOSE = 54.0;
    static final double TARGET_X_FAR = 89.0;
    private double targetX = TARGET_X_CLOSE;
    static final double TARGET_Y = Y;

    // Current assumed launch angle.
    static final double SHOOTING_ANGLE_DEG = 70.8;

    static final double SHOOTING_ANGLE_RAD =
            Math.toRadians(SHOOTING_ANGLE_DEG);

    // Approximate opening center:
    // (53.5 + 65.6) / 2 = 59.55 inches
    static final double HIVE_HEIGHT = 63.0;

    // Your assumed shooter exit height
    static final double SHOOTER_HEIGHT = 10.0;

    // Current wheel assumptions
    static final double WHEEL_RADIUS_METERS = 0.036;

    static final double TICKS_PER_REV = 28.0;

    // Shooter PIDF
    static final double FLYWHEEL_P = 2.0;
    static final double FLYWHEEL_I = 0.0;
    static final double FLYWHEEL_D = 0.5;
    static final double FLYWHEEL_F = 9.85;

    // Shooting timings
    static final double SHOOTER_SPINUP_SECONDS = 1.5;
    static final double FEED_SECONDS = 3.0;

    // Motor powers
    static final double INTAKE_POWER = 1.0;


    // =========================================================
    // AUTONOMOUS STATES
    // =========================================================

    public enum PathState {

        DRIVE,

        SHOOT_FIRST,

        BACK,

        CHECK_TAG_AFTER_FIRST,

        TURNONE,

        DRIVETOGARDEN,

        CHECK_TAG,

        SHOOT_SECOND,

        RECHECK_TAG,

        TO_FLOWER,

        SAME_SHOT,

        DIFF_SHOT,

        DIFF_DOWN,

        SHOOT_THIRD,

        DONE
    }

    private PathState pathState;

    private boolean stateInitialized = false;


    // =========================================================
    // POSES
    // =========================================================

    private final Pose startPose =
            new Pose(8, Y, Math.toRadians(0));

    private final Pose shootPose =
            new Pose(SHOOT_X, Y, Math.toRadians(0));

    private final Pose startOnePose =
            new Pose(14, Y, Math.toRadians(0));

    private final Pose startTwoPose =
            new Pose(8, Y, Math.toRadians(90));

    private final Pose gardenPose =
            new Pose(8, 125, Math.toRadians(90));

    private final Pose flowerPose =
            new Pose(
                    FLOWER_X,
                    gardenPose.getY(),
                    Math.toRadians(0)
            );

    private final Pose diffCornerPose =
            new Pose(
                    DIFF_SHOT_X,
                    flowerPose.getY(),
                    Math.toRadians(0)
            );

    private final Pose diffShotPose =
            new Pose(
                    DIFF_SHOT_X,
                    Y,
                    Math.toRadians(180)
            );


    // =========================================================
    // PATHS
    // =========================================================

    private PathChain drivePath;
    private PathChain backPath;
    private PathChain gardenPath;
    private PathChain flowerPath;
    private PathChain sameShotPath;

    private PathChain gardenToDiffPath;
    private PathChain diffAcrossPathTwo;
    private PathChain diffDownPath;


    // =========================================================
    // BUILD PATHS
    // =========================================================

    public void buildPaths() {

        // Start -> first shooting position
        drivePath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(
                        startPose.getHeading(),
                        shootPose.getHeading()
                )
                .build();


        // First shooting position -> starting position
        backPath = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, startOnePose))
                .setLinearHeadingInterpolation(
                        shootPose.getHeading(),
                        startOnePose.getHeading()
                )
                .build();


        // Starting position -> garden
        gardenPath = follower.pathBuilder()
                .addPath(new BezierLine(startTwoPose, gardenPose))
                .setLinearHeadingInterpolation(
                        startTwoPose.getHeading(),
                        gardenPose.getHeading()
                )
                .build();


        // Garden -> flower
        flowerPath = follower.pathBuilder()
                .addPath(new BezierLine(gardenPose, flowerPose))
                .setLinearHeadingInterpolation(
                        Math.toRadians(90),
                        flowerPose.getHeading()
                )
                .build();


        // Flower -> first shooting position
        sameShotPath = follower.pathBuilder()
                .addPath(new BezierLine(flowerPose, shootPose))
                .setLinearHeadingInterpolation(
                        flowerPose.getHeading(),
                        shootPose.getHeading()
                )
                .build();


        // Garden -> far side
        // Used when tag #1 is not visible.
        gardenToDiffPath = follower.pathBuilder()
                .addPath(new BezierLine(gardenPose, diffCornerPose))
                .setLinearHeadingInterpolation(
                        Math.toRadians(90),
                        Math.toRadians(180)
                )
                .build();


        // Flower -> far side
        // Used when tag #2 is not visible.
        diffAcrossPathTwo = follower.pathBuilder()
                .addPath(new BezierLine(flowerPose, diffCornerPose))
                .setLinearHeadingInterpolation(
                        flowerPose.getHeading(),
                        Math.toRadians(180)
                )
                .build();


        // Far-side corner -> far-side shooting position
        diffDownPath = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                diffCornerPose,
                                diffShotPose
                        )
                )
                .setConstantHeadingInterpolation(
                        Math.toRadians(180)
                )
                .build();
    }


    // =========================================================
    // INIT
    // =========================================================

    @Override
    public void init() {
        PoseStorage.hasPose = false;

        // Initialize intake
        intake.init(hardwareMap, "intake");

        // Initialize Limelight
        atlimelight.init(hardwareMap, 0);

        // Initialize PedroPathing
        follower = Constants.createFollower(hardwareMap);

        buildPaths();

        follower.setPose(startPose);

        shootTimer = new ElapsedTime();
        tagCheckTimer = new ElapsedTime();

        turret = new TurretMechanism();
        turret.init(hardwareMap);

        // Initialize shooter
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

        PIDFCoefficients flywheelPIDF =
                new PIDFCoefficients(
                        FLYWHEEL_P,
                        FLYWHEEL_I,
                        FLYWHEEL_D,
                        FLYWHEEL_F
                );

        shooter.setPIDFCoefficients(
                com.qualcomm.robotcore.hardware.DcMotor.RunMode
                        .RUN_USING_ENCODER,
                flywheelPIDF
        );

        shooter.setVelocity(0);

        intake.setMotorSpeed(0);

        setPathState(PathState.DRIVE);

        follower.followPath(drivePath, true);
    }


    @Override
    public void start() {
        atlimelight.start();
    }


    // =========================================================
    // STATE MANAGEMENT
    // =========================================================

    private void setPathState(PathState newState) {

        pathState = newState;

        stateInitialized = false;
    }


    // =========================================================
    // TAG HELPERS
    // =========================================================

    private boolean canSeeShootTag() {

        // Blue far-side CELL tags.
        // This tracks the initially upward-facing CELL only.
        return (atlimelight.hasTag(42)
                || atlimelight.hasTag(43)
                || atlimelight.hasTag(44)
                || atlimelight.hasTag(45));
    }


    // =========================================================
    // TURN HELPERS
    // =========================================================


    private double headingError(double targetRad) {

        return Math.atan2(
                Math.sin(targetRad - currentPose.getHeading()),
                Math.cos(targetRad - currentPose.getHeading())
        );
    }


    // =========================================================
    // SHOOTING HELPER
    // =========================================================

    private void runShootingSequence(
            int shotNumber,
            PathState nextState
    ) {

        // PHASE 1: Start the shooter
        if (!stateInitialized) {

            double targetRPM = calculateShot(
                    currentPose.getX(),
                    currentPose.getY(),
                    currentPose.getHeading()
            );

            // Convert RPM to encoder ticks per second
            double targetTicksPerSecond =
                    (targetRPM / 60.0) * TICKS_PER_REV;

            shooter.setVelocity(targetTicksPerSecond);

            intake.setMotorSpeed(0);

            feedingStarted = false;

            shootTimer.reset();

            stateInitialized = true;

            shotMessage = "SHOOTING #" + shotNumber
                    + " - SPINNING UP";
        }


        // PHASE 2: Wait for shooter to spin up
        if (!feedingStarted) {

            telemetry.addData(
                    "Shooter Spinup",
                    "%.2f / %.2f seconds",
                    shootTimer.seconds(),
                    SHOOTER_SPINUP_SECONDS
            );

            if (shootTimer.seconds() >= SHOOTER_SPINUP_SECONDS) {

                // Start feeding balls
                intake.setMotorSpeed(INTAKE_POWER);

                feedingStarted = true;

                shootTimer.reset();

                shotMessage = "SHOOTING #" + shotNumber
                        + " - FEEDING";
            }
        }


        // PHASE 3: Feed balls for 5 seconds
        else {

            telemetry.addData(
                    "Feeding",
                    "%.2f / %.2f seconds",
                    shootTimer.seconds(),
                    FEED_SECONDS
            );

            if (shootTimer.seconds() >= FEED_SECONDS) {

                // Stop both motors
                intake.setMotorSpeed(0);

                shooter.setVelocity(0);

                feedingStarted = false;

                shotMessage = "SHOT #" + shotNumber + " COMPLETE";

                setPathState(nextState);
            }
        }
    }


    // =========================================================
    // LOOP
    // =========================================================

    @Override
    public void loop() {

        follower.update();

        atlimelight.update();

        currentPose = follower.getPose();

        PoseStorage.x = currentPose.getX();
        PoseStorage.y = currentPose.getY();
        PoseStorage.headingDeg = Math.toDegrees(currentPose.getHeading());
        PoseStorage.hasPose = true;

        double dx = targetX - currentPose.getX();
        double dy = TARGET_Y - currentPose.getY();

        double fieldAngle = Math.toDegrees(Math.atan2(dy, dx));
        double desiredTurret = fieldAngle - Math.toDegrees(currentPose.getHeading());

        // normalize to -180..180
        desiredTurret = Math.toDegrees(Math.atan2(
                Math.sin(Math.toRadians(desiredTurret)),
                Math.cos(Math.toRadians(desiredTurret))));

        turret.update(desiredTurret);
        turretErrorDeg = desiredTurret - turret.getTurretPositionDegrees();

        telemetry.addData("Turret Error", "%.2f deg", turretErrorDeg);


        // =====================================================
        // GENERAL TELEMETRY
        // =====================================================

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
                "Sees Blue Far-Side Tag NOW",
                canSeeShootTag() ? "YES" : "NO"
        );

        telemetry.addData(
                "Tag After Shot #1",
                sawTagAfterFirstShot
        );

        telemetry.addData(
                "Tag After Shot #2",
                sawTagAfterSecondShot
        );

        telemetry.addData(
                "Shooter Velocity",
                "%.1f ticks/sec",
                shooter.getVelocity()
        );

        if (!shotMessage.isEmpty()) {
            telemetry.addData("Shot", shotMessage);
        }

        if (!postShotMessage.isEmpty()) {
            telemetry.addData("After Shot", postShotMessage);
        }

        if (!routeMessage.isEmpty()) {
            telemetry.addData("Route", routeMessage);
        }


        // =====================================================
        // STATE MACHINE
        // =====================================================

        switch (pathState) {


            // =================================================
            // DRIVE TO FIRST SHOOTING POSITION
            // =================================================

            case DRIVE:

                if (!follower.isBusy()) {

                    setPathState(PathState.SHOOT_FIRST);
                }

                break;


            // =================================================
            // SHOOT #1
            // =================================================

            case SHOOT_FIRST:

                runShootingSequence(
                        1,
                        PathState.BACK
                );

                if (pathState == PathState.BACK) {

                    follower.followPath(backPath, true);
                }

                break;


            // =================================================
            // BACK UP
            // =================================================

            case BACK:

                if (!follower.isBusy()) {

                    tagCheckTimer.reset();

                    setPathState(
                            PathState.CHECK_TAG_AFTER_FIRST
                    );
                }

                break;


            // =================================================
            // TAG CHECK #1
            // =================================================

            case CHECK_TAG_AFTER_FIRST:

                if (tagCheckTimer.seconds() >= 1.0) {
                    sawTagAfterFirstShot = canSeeShootTag();

                    if (sawTagAfterFirstShot) {

                        postShotMessage =
                                "Tag visible after shot #1";

                        routeMessage =
                                "Continue to shot #2";

                    } else {

                        postShotMessage =
                                "No tag after shot #1";

                        routeMessage =
                                "Skip shot #2 and flower";
                    }

                    setPathState(PathState.TURNONE);
                }

                break;


            // =================================================
            // TURN TOWARD GARDEN
            // =================================================

            case TURNONE:

                if (!stateInitialized) {

                    follower.turnTo(Math.toRadians(90));

                    stateInitialized = true;
                }

                double turnOneTarget = Math.toRadians(90);

                double turnOneError = headingError(turnOneTarget);

                telemetry.addData(
                        "TURNONE Error",
                        "%.2f°",
                        Math.toDegrees(turnOneError)
                );

                if (!follower.isBusy()
                        || Math.abs(turnOneError)
                        < Math.toRadians(5)) {

                    intake.setMotorSpeed(INTAKE_POWER);

                    setPathState(PathState.DRIVETOGARDEN);

                    follower.followPath(gardenPath, true);
                }

                break;


            // =================================================
            // DRIVE TO GARDEN
            // =================================================

            case DRIVETOGARDEN:

                if (!follower.isBusy()) {

                    intake.setMotorSpeed(0);

                    setPathState(PathState.CHECK_TAG);
                }

                break;


            // =================================================
            // DECIDE WHETHER TO SHOOT #2
            // Use the SAVED result from tag check #1.
            // =================================================

            case CHECK_TAG:

                if (sawTagAfterFirstShot && canSeeShootTag()) {

                    shotMessage =
                            "Tag #1 visible - shooting #2";

                    routeMessage =
                            "Shot #2 -> Check tag #2 -> Flower";

                    setPathState(PathState.SHOOT_SECOND);
                }

                else {

                    shotMessage =
                            "Tag #1 missing - skipping shot #2";

                    routeMessage =
                            "Garden -> Far side -> Shot #3";

                    setPathState(PathState.DIFF_SHOT);

                    follower.followPath(
                            gardenToDiffPath,
                            true
                    );
                }

                break;


            // =================================================
            // SHOOT #2
            // =================================================

            case SHOOT_SECOND:

                runShootingSequence(
                        2,
                        PathState.RECHECK_TAG
                );

                break;


            // =================================================
            // TAG CHECK #2
            // ALWAYS GO TO FLOWER AFTER THIS CHECK.
            // =================================================

            case RECHECK_TAG:

                sawTagAfterSecondShot = canSeeShootTag();

                if (sawTagAfterSecondShot) {

                    postShotMessage =
                            "Tag STILL visible after shot #2";

                    routeMessage =
                            "Flower -> First shooting position";

                } else {

                    postShotMessage =
                            "Tag no longer visible after shot #2";

                    routeMessage =
                            "Flower -> Far side";
                }

                intake.setMotorSpeed(INTAKE_POWER);

                setPathState(PathState.TO_FLOWER);

                follower.followPath(
                        flowerPath,
                        true
                );

                break;


            // =================================================
            // DRIVE TO FLOWER
            // =================================================

            case TO_FLOWER:

                if (!follower.isBusy()) {

                    intake.setMotorSpeed(0);

                    if (sawTagAfterSecondShot) {

                        setPathState(PathState.SAME_SHOT);

                        follower.followPath(
                                sameShotPath,
                                true
                        );

                    } else {

                        setPathState(PathState.DIFF_SHOT);

                        follower.followPath(
                                diffAcrossPathTwo,
                                true
                        );
                    }
                }

                break;


            // =================================================
            // RETURN TO FIRST SHOOTING POSITION
            // =================================================

            case SAME_SHOT:

                if (!follower.isBusy()) {

                    setPathState(PathState.SHOOT_THIRD);
                }

                break;


            // =================================================
            // DRIVE ACROSS FIELD
            // =================================================

            case DIFF_SHOT:

                if (!follower.isBusy()) {

                    setPathState(PathState.DIFF_DOWN);

                    follower.followPath(
                            diffDownPath,
                            true
                    );
                }

                break;


            // =================================================
            // DRIVE TO FAR-SIDE SHOOTING POSITION
            // =================================================

            case DIFF_DOWN:
                if (!follower.isBusy()) {
                    targetX = TARGET_X_FAR;
                    setPathState(PathState.SHOOT_THIRD);
                }
                break;

            // =================================================
            // SHOOT #3
            // =================================================

            case SHOOT_THIRD:

                runShootingSequence(
                        3,
                        PathState.DONE
                );

                break;


            // =================================================
            // DONE
            // =================================================

            case DONE:

                intake.setMotorSpeed(0);

                shooter.setVelocity(0);

                break;
        }

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
                2 * cosTheta * cosTheta
                        * (shootX * tanTheta - deltaY);

        if (denominator <= 0) {

            return 0.0;
        }

        double velocity =
                Math.sqrt(
                        (9.81 * shootX * shootX)
                                / denominator
                );

        double targetRPM =
                (velocity / WHEEL_RADIUS_METERS)
                        * (60.0 / (2.0 * Math.PI));

        return targetRPM * 2.8;
    }


    // =========================================================
    // STOP
    // =========================================================

    @Override
    public void stop() {

        intake.setMotorSpeed(0);

        if (shooter != null) {
            shooter.setVelocity(0);
        }

        if (turret != null) {
            turret.stop();
        }
    }
}