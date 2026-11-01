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


@Autonomous
public class AutonTwo extends OpMode {

    // =========================================================
    // HARDWARE
    // =========================================================

    private Follower follower;

    private final DcMotor intake = new DcMotor();
    private DcMotorEx shooter;

    private final AprilTagLimelight atlimelight =
            new AprilTagLimelight();

    private ElapsedTime shootTimer;

    private Pose currentPose;

    private boolean stateInitialized = false;
    private boolean feedingStarted = false;

    private String shotMessage = "";
    private String routeMessage = "";

    private boolean sawTarget1Tag = false;

    private Pose activeTargetPose;

    private int shotNumber = 1;

    private PathState afterShootState;


    // =========================================================
    // FIELD / SHOOTING CONSTANTS
    // =========================================================

    // Keep these coordinates in your current Pedro coordinate frame.

    private static final double HIVE_HEIGHT = 59.55;
    private static final double SHOOTER_HEIGHT = 8.0;

    private static final double SHOOTING_ANGLE_DEG = 37.8;
    private static final double SHOOTING_ANGLE_RAD =
            Math.toRadians(SHOOTING_ANGLE_DEG);

    private static final double WHEEL_RADIUS_METERS = 0.036;
    private static final double TICKS_PER_REV = 28.0;

    private static final double FLYWHEEL_P = 2.0;
    private static final double FLYWHEEL_I = 0.0;
    private static final double FLYWHEEL_D = 0.5;
    private static final double FLYWHEEL_F = 11.7;

    private static final double SHOOTER_SPINUP_SECONDS = 1.5;
    private static final double FEED_SECONDS = 5.0;

    private static final double INTAKE_POWER = -1.0;


    // =========================================================
    // POSES
    // =========================================================

    // Starting position.

    private final Pose startPose =
            new Pose(8, 96, Math.toRadians(0));


    // ---------------------------------------------------------
    // FIRST SHOOTING POSITION AND TARGET
    // ---------------------------------------------------------

    // Where the robot stands for target 1.

    private final Pose shootPose1 =
            new Pose(13, 96, Math.toRadians(0));

    // Actual center of target 1.
    // Measure and adjust this coordinate.

    private final Pose target1Pose =
            new Pose(55, 96, 0);


    // ---------------------------------------------------------
    // GARDEN
    // ---------------------------------------------------------

    // Same location as startPose, but facing toward the garden.

    private final Pose gardenStartPose =
            new Pose(8, 96, Math.toRadians(90));

    // Position at the garden, facing 90 degrees before driving forward.
    private final Pose gardenForwardPose =
            new Pose(8, 144, Math.toRadians(0));

    // Far corner after driving forward from the garden.
    private final Pose farSidePose =
            new Pose(131, 144, Math.toRadians(0));

    private final Pose gardenPose =
            new Pose(8, 144, Math.toRadians(90));


    // ---------------------------------------------------------
    // SECOND SHOOTING POSITION AND TARGET
    // ---------------------------------------------------------

    // Where the robot stands to shoot from the other side.
    // MEASURE AND ADJUST THIS POSE.

    private final Pose shootPose2 =
            new Pose(131, 96, Math.toRadians(180));

    // Actual center of target 2.
    // MEASURE AND ADJUST THIS INDEPENDENTLY FROM TARGET 1.
    //
    // These are deliberately separate coordinates.
    // Do not assume the example values are field measurements.

    private final Pose target2Pose =
            new Pose(92, 96, 0);


    // =========================================================
    // STATES
    // =========================================================

    public enum PathState {

        DRIVE_TO_SHOOT1,

        ALIGN_SHOT,

        SHOOT,

        BACK_TO_START,

        CHECK_TAG,

        TURN_TO_GARDEN,

        DRIVE_TO_GARDEN,

        CHOOSE_SHOT2,

        RETURN_TO_SHOOT1,

        DRIVE_TO_SHOOT2,

        DONE,
        TURN_TO_SHOOT2_LANE,

        DRIVE_TO_FAR_SIDE,
    }

    private PathState pathState;


    // =========================================================
    // PATHS
    // =========================================================

    private PathChain driveToShoot1Path;
    private PathChain backToStartPath;
    private PathChain gardenPath;
    private PathChain gardenToShoot1Path;
    private PathChain gardenToFarSidePath;
    private PathChain farSideToShoot2Path;


    // =========================================================
    // BUILD PATHS
    // =========================================================

    private void buildPaths() {

        // Start -> first shooting position

        driveToShoot1Path = follower.pathBuilder()
                .addPath(
                        new BezierLine(startPose, shootPose1)
                )
                .setLinearHeadingInterpolation(
                        startPose.getHeading(),
                        shootPose1.getHeading()
                )
                .build();


        // First shooting position -> start

        backToStartPath = follower.pathBuilder()
                .addPath(
                        new BezierLine(shootPose1, startPose)
                )
                .setLinearHeadingInterpolation(
                        shootPose1.getHeading(),
                        startPose.getHeading()
                )
                .build();


        // Start -> garden

        gardenPath = follower.pathBuilder()
                .addPath(
                        new BezierLine(gardenStartPose, gardenPose)
                )
                .setLinearHeadingInterpolation(
                        gardenStartPose.getHeading(),
                        gardenPose.getHeading()
                )
                .build();


        // Garden -> first shooting position
        // Used if target 1's tag was still visible.

        gardenToShoot1Path = follower.pathBuilder()
                .addPath(
                        new BezierLine(gardenPose, shootPose1)
                )
                .setLinearHeadingInterpolation(
                        gardenPose.getHeading(),
                        shootPose1.getHeading()
                )
                .build();


        // Garden -> second shooting position
        // Used if target 1's tag was NOT visible.

        // Garden -> far side of field.
// Robot should already be facing 90 degrees.

        gardenToFarSidePath = follower.pathBuilder()
                .addPath(
                        new BezierLine(gardenForwardPose, farSidePose)
                )
                .setConstantHeadingInterpolation(
                        gardenForwardPose.getHeading()
                )
                .build();


// Far side -> second shooting position.

        farSideToShoot2Path = follower.pathBuilder()
                .addPath(
                        new BezierLine(farSidePose, shootPose2)
                )
                .setLinearHeadingInterpolation(
                        farSidePose.getHeading(),
                        shootPose2.getHeading()
                )
                .build();
    }


    // =========================================================
    // INITIALIZATION
    // =========================================================

    @Override
    public void init() {

        intake.init(hardwareMap, "intake");

        atlimelight.init(hardwareMap, 0);

        follower = Constants.createFollower(hardwareMap);

        buildPaths();

        follower.setPose(startPose);

        shootTimer = new ElapsedTime();


        // Shooter initialization

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


        // First shot always targets target 1.

        activeTargetPose = target1Pose;

        shotNumber = 1;

        afterShootState = PathState.BACK_TO_START;

        setPathState(PathState.DRIVE_TO_SHOOT1);
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
    // APRILTAG CHECK
    // =========================================================

    private boolean canSeeTarget1Tag() {

        // Blue far-side CELL tags.

        return atlimelight.hasTag(42)
                || atlimelight.hasTag(43)
                || atlimelight.hasTag(44)
                || atlimelight.hasTag(45);
    }


    // =========================================================
    // HEADING HELPERS
    // =========================================================

    private double headingError(double targetRad) {

        return Math.atan2(
                Math.sin(targetRad - currentPose.getHeading()),
                Math.cos(targetRad - currentPose.getHeading())
        );
    }


    private double getHeadingToTarget(Pose targetPose) {

        return Math.atan2(
                targetPose.getY() - currentPose.getY(),
                targetPose.getX() - currentPose.getX()
        );
    }


    private boolean turnDone(
            double targetRad,
            double toleranceRad
    ) {

        double error = headingError(targetRad);

        telemetry.addData(
                "Turn Target",
                "%.2f degrees",
                Math.toDegrees(targetRad)
        );

        telemetry.addData(
                "Turn Error",
                "%.2f degrees",
                Math.toDegrees(error)
        );

        telemetry.addData(
                "Turn Busy",
                follower.isBusy()
        );

        return !follower.isBusy()
                && Math.abs(error) < toleranceRad;
    }


    // =========================================================
    // SHOOTING SEQUENCE
    // =========================================================

    private void runShootingSequence() {

        // Start shooter.

        if (!stateInitialized) {

            double targetRPM = calculateShot(
                    currentPose,
                    activeTargetPose
            );

            double targetTicksPerSecond =
                    (targetRPM / 60.0) * TICKS_PER_REV;

            shooter.setVelocity(targetTicksPerSecond);

            intake.setMotorSpeed(0);

            feedingStarted = false;

            shootTimer.reset();

            stateInitialized = true;

            shotMessage = "SHOT " + shotNumber
                    + " - SPINNING UP";

            telemetry.addData(
                    "Active Target",
                    shotNumber == 1 || activeTargetPose == target1Pose
                            ? "TARGET 1"
                            : "TARGET 2"
            );

            telemetry.addData(
                    "Target X",
                    activeTargetPose.getX()
            );

            telemetry.addData(
                    "Target Y",
                    activeTargetPose.getY()
            );
        }


        // Wait for shooter spin-up.

        if (!feedingStarted) {

            if (shootTimer.seconds() >= SHOOTER_SPINUP_SECONDS) {

                intake.setMotorSpeed(INTAKE_POWER);

                feedingStarted = true;

                shootTimer.reset();

                shotMessage = "SHOT " + shotNumber
                        + " - FEEDING";
            }
        }


        // Feed.

        else {

            if (shootTimer.seconds() >= FEED_SECONDS) {

                intake.setMotorSpeed(0);

                shooter.setVelocity(0);

                feedingStarted = false;

                shotMessage = "SHOT " + shotNumber
                        + " COMPLETE";

                setPathState(afterShootState);
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


        // -----------------------------------------------------
        // TELEMETRY
        // -----------------------------------------------------

        telemetry.addData("State", pathState);

        telemetry.addData(
                "Robot X",
                "%.2f",
                currentPose.getX()
        );

        telemetry.addData(
                "Robot Y",
                "%.2f",
                currentPose.getY()
        );

        telemetry.addData(
                "Heading",
                "%.2f degrees",
                Math.toDegrees(currentPose.getHeading())
        );

        telemetry.addData(
                "Follower Busy",
                follower.isBusy()
        );

        telemetry.addData(
                "Target 1 Tag Visible",
                canSeeTarget1Tag()
        );

        telemetry.addData(
                "Saved Tag Result",
                sawTarget1Tag
        );

        telemetry.addData(
                "Shooter Velocity",
                "%.1f ticks/sec",
                shooter.getVelocity()
        );

        telemetry.addData("Shot", shotMessage);

        telemetry.addData("Route", routeMessage);


        // -----------------------------------------------------
        // STATE MACHINE
        // -----------------------------------------------------

        switch (pathState) {


            // ================================================
            // 1. DRIVE TO FIRST SHOOTING POSITION
            // ================================================

            case DRIVE_TO_SHOOT1:

                if (!stateInitialized) {

                    follower.followPath(
                            driveToShoot1Path,
                            true
                    );

                    stateInitialized = true;
                }

                if (!follower.isBusy()) {

                    activeTargetPose = target1Pose;

                    shotNumber = 1;

                    afterShootState =
                            PathState.BACK_TO_START;

                    setPathState(PathState.ALIGN_SHOT);
                }

                break;


            // ================================================
            // 2. ALIGN WITH THE CURRENT TARGET
            // ================================================

            case ALIGN_SHOT:

                if (!stateInitialized) {

                    double targetHeading;

                    if (activeTargetPose == target1Pose) {
                        targetHeading = shootPose1.getHeading();
                    } else {
                        targetHeading = shootPose2.getHeading();
                    }

                    follower.turnTo(targetHeading);

                    stateInitialized = true;
                }

                double desiredHeading =
                        activeTargetPose == target1Pose
                                ? shootPose1.getHeading()
                                : shootPose2.getHeading();

                if (turnDone(
                        desiredHeading,
                        Math.toRadians(3)
                )) {

                    setPathState(PathState.SHOOT);
                }

                break;


            // ================================================
            // 3. SHOOT
            // ================================================

            case SHOOT:
                intake.setMotorSpeed(-1.0);

                runShootingSequence();

                break;


            // ================================================
            // 4. BACK UP TO START
            // ================================================

            case BACK_TO_START:

                if (!stateInitialized) {

                    follower.followPath(
                            backToStartPath,
                            true
                    );

                    stateInitialized = true;
                }

                if (!follower.isBusy()) {

                    setPathState(PathState.CHECK_TAG);
                }

                break;


            // ================================================
            // 5. CHECK WHETHER TARGET 1 TAG IS STILL VISIBLE
            // ================================================

            case CHECK_TAG:

                sawTarget1Tag = canSeeTarget1Tag();

                if (sawTarget1Tag) {

                    routeMessage =
                            "Target 1 tag visible. Will shoot target 1 again.";

                } else {

                    routeMessage =
                            "Target 1 tag missing. Will shoot target 2.";
                }

                setPathState(PathState.TURN_TO_GARDEN);

                break;


            // ================================================
            // 6. TURN TOWARD GARDEN
            // ================================================

            case TURN_TO_GARDEN:

                if (!stateInitialized) {
                    follower.turnTo(Math.toRadians(180));
                    stateInitialized = true;
                }

                if (turnDone(
                        Math.toRadians(180),
                        Math.toRadians(3)
                )) {
                    setPathState(PathState.DRIVE_TO_GARDEN);
                }

                break;

            // ================================================
            // 7. DRIVE TO GARDEN
            // ================================================

            case DRIVE_TO_GARDEN:
                intake.setMotorSpeed(-1.0);

                if (!stateInitialized) {

                    follower.followPath(
                            gardenPath,
                            true
                    );

                    stateInitialized = true;
                }

                if (!follower.isBusy()) {

                    setPathState(PathState.CHOOSE_SHOT2);
                }

                break;


            // ================================================
            // 8. CHOOSE WHICH TARGET TO SHOOT NEXT
            // ================================================

            case CHOOSE_SHOT2:

                shotNumber = 2;

                afterShootState = PathState.DONE;

                if (sawTarget1Tag) {

                    // Tag visible: return to shooting position 1.

                    activeTargetPose = target1Pose;

                    routeMessage =
                            "Returning to shooting position 1.";

                    setPathState(PathState.RETURN_TO_SHOOT1);

                } else {

                    // Tag not visible: go to shooting position 2.

                    activeTargetPose = target2Pose;

                    routeMessage =
                            "Driving to shooting position 2.";

                    setPathState(PathState.TURN_TO_SHOOT2_LANE);
                }

                break;


            // ================================================
            // 9A. RETURN TO SHOOTING POSITION 1
            // ================================================

            case RETURN_TO_SHOOT1:

                if (!stateInitialized) {

                    follower.followPath(
                            gardenToShoot1Path,
                            true
                    );

                    stateInitialized = true;
                }

                if (!follower.isBusy()) {

                    setPathState(PathState.ALIGN_SHOT);
                }

                break;


            // ================================================
            // 9B. DRIVE TO SHOOTING POSITION 2
            // ================================================


            // ================================================
            // DONE
            // ================================================

            case DONE:

                intake.setMotorSpeed(0);

                shooter.setVelocity(0);

                routeMessage = "Autonomous complete.";

                break;

            // ================================================
// TURN TO FACE 90 DEGREES AT THE GARDEN
// ================================================

            case TURN_TO_SHOOT2_LANE:

                if (!stateInitialized) {

                    follower.turnTo(Math.toRadians(90));

                    stateInitialized = true;
                }

                if (turnDone(
                        Math.toRadians(90),
                        Math.toRadians(3)
                )) {

                    setPathState(PathState.DRIVE_TO_FAR_SIDE);
                }

                break;


// ================================================
// DRIVE FORWARD TO (131, 144)
// ================================================

            case DRIVE_TO_FAR_SIDE:

                if (!stateInitialized) {

                    follower.followPath(
                            gardenToFarSidePath,
                            true
                    );

                    stateInitialized = true;
                }

                if (!follower.isBusy()) {

                    setPathState(PathState.DRIVE_TO_SHOOT2);
                }

                break;


// ================================================
// DRIVE TO (131, 96, 270)
// ================================================

            case DRIVE_TO_SHOOT2:

                if (!stateInitialized) {

                    follower.followPath(
                            farSideToShoot2Path,
                            true
                    );

                    stateInitialized = true;
                }

                if (!follower.isBusy()) {

                    setPathState(PathState.ALIGN_SHOT);
                }

                break;
        }

        telemetry.update();
    }


    // =========================================================
    // SHOOTING CALCULATION
    // =========================================================

    private double calculateShot(
            Pose robotPose,
            Pose targetPose
    ) {

        // IMPORTANT:
        // This uses the active target's coordinates.
        // Shot 1 and shot 2 can therefore have different distances
        // and different calculated flywheel velocities.

        double dx =
                targetPose.getX() - robotPose.getX();

        double dy =
                targetPose.getY() - robotPose.getY();

        double distanceMeters =
                0.0254 * Math.hypot(dx, dy);

        double heightDifference =
                (HIVE_HEIGHT - SHOOTER_HEIGHT) * 0.0254;

        double cosTheta =
                Math.cos(SHOOTING_ANGLE_RAD);

        double tanTheta =
                Math.tan(SHOOTING_ANGLE_RAD);

        double denominator =
                2 * cosTheta * cosTheta
                        * (distanceMeters * tanTheta
                        - heightDifference);

        if (denominator <= 0) {

            return 0.0;
        }

        double velocity =
                Math.sqrt(
                        (9.81 * distanceMeters * distanceMeters)
                                / denominator
                );

        double targetRPM =
                (velocity / WHEEL_RADIUS_METERS)
                        * (60.0 / (2.0 * Math.PI));

        // Existing calibration multiplier.

        return targetRPM * 2.5;
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
    }
}