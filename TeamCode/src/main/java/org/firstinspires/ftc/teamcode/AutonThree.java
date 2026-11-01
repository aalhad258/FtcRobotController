package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.mechanism.AprilTagLimelight;
import org.firstinspires.ftc.teamcode.mechanism.DcMotor;
import org.firstinspires.ftc.teamcode.mechanism.PinpointPathing;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


@Autonomous
public class AutonThree extends OpMode {

    // =========================================================
    // HARDWARE
    // =========================================================

    private final PinpointPathing pathing = new PinpointPathing();

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

    private final Pose startPose =
            new Pose(8, 96, Math.toRadians(0));


    // ---------------------------------------------------------
    // FIRST SHOOTING POSITION AND TARGET
    // ---------------------------------------------------------

    private final Pose shootPose1 =
            new Pose(13, 96, Math.toRadians(0));

    private final Pose target1Pose =
            new Pose(55, 96, 0);


    // ---------------------------------------------------------
    // GARDEN
    // ---------------------------------------------------------

    private final Pose gardenStartPose =
            new Pose(8, 96, Math.toRadians(90));

    private final Pose gardenForwardPose =
            new Pose(8, 144, Math.toRadians(0));

    private final Pose farSidePose =
            new Pose(131, 144, Math.toRadians(0));

    private final Pose gardenPose =
            new Pose(8, 144, Math.toRadians(90));


    // ---------------------------------------------------------
    // SECOND SHOOTING POSITION AND TARGET
    // ---------------------------------------------------------

    private final Pose shootPose2 =
            new Pose(131, 96, Math.toRadians(180));

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

        TURN_TO_SHOOT2_LANE,

        DRIVE_TO_FAR_SIDE,

        DRIVE_TO_SHOOT2,

        DONE
    }

    private PathState pathState;


    // =========================================================
    // POSE CONVERSION
    // =========================================================

    // Your waypoint variables use Pedro's Pose class.
    // PinpointPathing uses the FTC SDK's Pose2D class.
    // This converts between them without changing coordinates.

    private Pose2D convertPose(Pose pose) {

        return new Pose2D(
                DistanceUnit.INCH,
                pose.getX(),
                pose.getY(),
                AngleUnit.RADIANS,
                pose.getHeading()
        );
    }


    // =========================================================
    // INITIALIZATION
    // =========================================================

    @Override
    public void init() {

        intake.init(hardwareMap, "intake");

        atlimelight.init(hardwareMap, 0);

        // Initialize custom Pinpoint pathing.

        pathing.init(
                hardwareMap,
                convertPose(startPose)
        );

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

        return atlimelight.hasTag(42)
                || atlimelight.hasTag(43)
                || atlimelight.hasTag(44)
                || atlimelight.hasTag(45);
    }


    // =========================================================
    // SHOOTING SEQUENCE
    // =========================================================

    private void runShootingSequence() {

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
                    activeTargetPose == target1Pose
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


        if (!feedingStarted) {

            if (shootTimer.seconds() >= SHOOTER_SPINUP_SECONDS) {

                intake.setMotorSpeed(INTAKE_POWER);

                feedingStarted = true;

                shootTimer.reset();

                shotMessage = "SHOT " + shotNumber
                        + " - FEEDING";
            }
        }

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

        // Update Limelight every loop.

        atlimelight.update();

        // Get latest Pinpoint pose.

        Pose2D pinpointPose = pathing.getCurrentPose();

        currentPose = new Pose(
                pinpointPose.getX(DistanceUnit.INCH),
                pinpointPose.getY(DistanceUnit.INCH),
                Math.toRadians(
                        pinpointPose.getHeading(AngleUnit.DEGREES)
                )
        );


        // =====================================================
        // TELEMETRY
        // =====================================================

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
                "Pathing State",
                pathing.getState()
        );

        telemetry.addData(
                "Pathing Busy",
                pathing.isBusy()
        );

        telemetry.addData(
                "Error X",
                "%.2f",
                pathing.getErrorX()
        );

        telemetry.addData(
                "Error Y",
                "%.2f",
                pathing.getErrorY()
        );

        telemetry.addData(
                "Error Heading",
                "%.2f",
                pathing.getErrorH()
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


        // =====================================================
        // STATE MACHINE
        // =====================================================

        switch (pathState) {


            // ================================================
            // 1. DRIVE TO FIRST SHOOTING POSITION
            // ================================================

            case DRIVE_TO_SHOOT1:

                if (!stateInitialized) {

                    pathing.setTarget(
                            convertPose(shootPose1)
                    );

                    stateInitialized = true;
                }

                pathing.update();

                if (!pathing.isBusy()) {

                    activeTargetPose = target1Pose;

                    shotNumber = 1;

                    afterShootState =
                            PathState.BACK_TO_START;

                    setPathState(PathState.ALIGN_SHOT);
                }

                break;


            // ================================================
            // 2. ALIGN WITH CURRENT TARGET
            // ================================================

            case ALIGN_SHOT:

                // The pathing mechanism already turns to the
                // heading specified in the shooting pose.

                setPathState(PathState.SHOOT);

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

                    pathing.setTarget(
                            convertPose(startPose)
                    );

                    stateInitialized = true;
                }

                pathing.update();

                if (!pathing.isBusy()) {

                    setPathState(PathState.CHECK_TAG);
                }

                break;


            // ================================================
            // 5. CHECK TARGET 1 TAG
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

                    pathing.setTarget(
                            convertPose(gardenStartPose)
                    );

                    stateInitialized = true;
                }

                pathing.update();

                if (!pathing.isBusy()) {

                    setPathState(PathState.DRIVE_TO_GARDEN);
                }

                break;


            // ================================================
            // 7. DRIVE TO GARDEN
            // ================================================

            case DRIVE_TO_GARDEN:
                intake.setMotorSpeed(-1.0);

                if (!stateInitialized) {

                    pathing.setTarget(
                            convertPose(gardenPose)
                    );

                    stateInitialized = true;
                }

                pathing.update();

                if (!pathing.isBusy()) {

                    setPathState(PathState.CHOOSE_SHOT2);
                }

                break;


            // ================================================
            // 8. CHOOSE NEXT TARGET
            // ================================================

            case CHOOSE_SHOT2:

                shotNumber = 2;

                afterShootState = PathState.DONE;

                if (sawTarget1Tag) {

                    activeTargetPose = target1Pose;

                    routeMessage =
                            "Returning to shooting position 1.";

                    setPathState(PathState.RETURN_TO_SHOOT1);

                } else {

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

                    pathing.setTarget(
                            convertPose(shootPose1)
                    );

                    stateInitialized = true;
                }

                pathing.update();

                if (!pathing.isBusy()) {

                    setPathState(PathState.ALIGN_SHOT);
                }

                break;


            // ================================================
            // 9B. TURN TO FACE SHOOTING LANE
            // ================================================

            case TURN_TO_SHOOT2_LANE:

                if (!stateInitialized) {

                    pathing.setTarget(
                            convertPose(gardenForwardPose)
                    );

                    stateInitialized = true;
                }

                pathing.update();

                if (!pathing.isBusy()) {

                    setPathState(PathState.DRIVE_TO_FAR_SIDE);
                }

                break;


            // ================================================
            // 10. DRIVE TO FAR SIDE
            // ================================================

            case DRIVE_TO_FAR_SIDE:

                if (!stateInitialized) {

                    pathing.setTarget(
                            convertPose(farSidePose)
                    );

                    stateInitialized = true;
                }

                pathing.update();

                if (!pathing.isBusy()) {

                    setPathState(PathState.DRIVE_TO_SHOOT2);
                }

                break;


            // ================================================
            // 11. DRIVE TO SHOOTING POSITION 2
            // ================================================

            case DRIVE_TO_SHOOT2:

                if (!stateInitialized) {

                    pathing.setTarget(
                            convertPose(shootPose2)
                    );

                    stateInitialized = true;
                }

                pathing.update();

                if (!pathing.isBusy()) {

                    setPathState(PathState.ALIGN_SHOT);
                }

                break;


            // ================================================
            // DONE
            // ================================================

            case DONE:

                pathing.stop();

                intake.setMotorSpeed(0);

                shooter.setVelocity(0);

                routeMessage = "Autonomous complete.";

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

        return targetRPM * 2.5;
    }


    // =========================================================
    // STOP
    // =========================================================

    @Override
    public void stop() {

        pathing.stop();

        intake.setMotorSpeed(0);

        if (shooter != null) {

            shooter.setVelocity(0);
        }
    }
}