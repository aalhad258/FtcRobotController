package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
// SHOOTER (turret not mounted yet) - re-enable when it is
// import com.qualcomm.robotcore.hardware.DcMotorEx;
// import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.mechanism.AprilTagLimelight;
// INTAKE DISABLED - re-enable when it is wired up
// import org.firstinspires.ftc.teamcode.mechanism.DcMotor;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class StartShooting extends OpMode {

    // =========================================================
    // HARDWARE & DRIVER
    // =========================================================

    private Follower follower;

    // INTAKE DISABLED
    // DcMotor intake = new DcMotor();

    // SHOOTER DISABLED (turret not mounted)
    // DcMotorEx shooter;

    private ElapsedTime shootTimer;
    private boolean shootTimerStarted = false;

    private ElapsedTime alignTimer;
    private int alignAttempts = 0;

    private Pose currentPose;

    private final AprilTagLimelight atlimelight = new AprilTagLimelight();

    // Simulated hardware values (telemetry only)
    private double simTargetRPM = 0.0;
    private double simIntakePower = 0.0;
    private String shotMessage = "";
    private String postShotMessage = "";
    private String routeMessage = "";

    // Heading (radians) from the robot to the goal, calculated when TURNTWO starts
    private double turnTwoTarget = 0.0;

    // Heading (radians) to the goal for the final turn before shot #3
    private double goalTurnTarget = 0.0;

    // Did we still see a tag after shot #2? Decides same shot vs different shot
    private boolean sawTagAfterSecondShot = false;


    // =========================================================
    // FIELD SHIFT
    // =========================================================

    // Start pose and goal are both moved this many inches to the left
    // (+Y in Pedro coordinates when facing heading 0)
    static final double LEFT_SHIFT_IN = 25.0;

    static final double BASE_Y = 11.75;
    static final double Y = BASE_Y + LEFT_SHIFT_IN;   // 36.75

    // Field is 144 in x 144 in
    static final double FIELD_SIZE_IN = 144.0;

    // Flower X (same Y as the garden)
    static final double FLOWER_X = 60.0;

    // X of the first shooting pose, and its mirror on the other side of the field
    static final double SHOOT_X = 13.0;
    static final double DIFF_SHOT_X = FIELD_SIZE_IN - SHOOT_X;   // 131


    // =========================================================
    // SHOOTER / GEOMETRY CONSTANTS
    // =========================================================

    // Goal position (shifted 25 in left)
    static final double TARGET_X = 55.0;
    static final double TARGET_Y = Y;

    static final double SHOOTING_ANGLE_DEG = 70.8;
    static final double SHOOTING_ANGLE_RAD =
            Math.toRadians(SHOOTING_ANGLE_DEG);

    static final double HIVE_HEIGHT = 63.0;
    static final double SHOOTER_HEIGHT = 10.0;

    static final double WHEEL_RADIUS_METERS = 0.036;
    static final double TICKS_PER_REV = 28.0;

    // Tuned PIDF parameters (only needed once shooter is mounted)
    // static final double FLYWHEEL_P = 2.0;
    // static final double FLYWHEEL_I = 0.0;
    // static final double FLYWHEEL_D = 0.5;
    // static final double FLYWHEEL_F = 11.7;

    // Fake shoot duration (was 10 s with the real flywheel)
    static final double FAKE_SHOOT_SECONDS = 5.0;

    // Tag alignment tuning
    static final double ALIGN_TOLERANCE_DEG = 2.0;
    static final double ALIGN_TIMEOUT_SECONDS = 4.0;


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
        CHECK_TAG,      // can we see a tag before the second shot?
        ALIGN_TAG,      // line up on the tag using Limelight tx
        SHOOT_SECOND,   // fake shoot
        RECHECK_TAG,    // look for tags again after shooting
        TO_FLOWER,      // drive to the flower, intaking
        SAME_SHOT,      // (tag seen) drive back to where we shot first
        DIFF_SHOT,      // (no tag) drive across the field to the far-side X
        DIFF_TURN,      // (no tag) turn right
        DIFF_DOWN,      // (no tag) drive forward to the different shoot pose
        TURN_TO_GOAL,   // turn toward the goal
        SHOOT_THIRD,    // fake shoot
        DONE
    }

    private PathState pathState;
    private boolean stateInitialized = false;

    // Everything near the start/goal is shifted 25 in left (Y = 36.75)
    private final Pose startPose =
            new Pose(8, Y, Math.toRadians(0));

    private final Pose shootPose =
            new Pose(SHOOT_X, Y, Math.toRadians(0));

    private final Pose startTwoPose =
            new Pose(8, Y, Math.toRadians(90));

    // Garden stays where it is on the field, so the drive there is
    // now 25 in shorter (60 - 36.75 = 23.25 in instead of 48.25 in)
    private final Pose gardenPose =
            new Pose(8, 60, Math.toRadians(90));

    // Flower: same Y as the garden, driving straight across (+X) so facing 0
    private final Pose flowerPose =
            new Pose(
                    FLOWER_X,
                    gardenPose.getY(),
                    Math.toRadians(0)
            );

    // "Different" shot route: across the field at the flower's Y, ...
    private final Pose diffCornerPose =
            new Pose(DIFF_SHOT_X, flowerPose.getY(), Math.toRadians(0));

    // ... turn right (facing 270 = -90), ...
    private final Pose diffDownStartPose =
            new Pose(DIFF_SHOT_X, flowerPose.getY(), Math.toRadians(270));

    // ... then drive forward to the mirror of the first shoot pose, facing 180
    private final Pose diffShotPose =
            new Pose(DIFF_SHOT_X, Y, Math.toRadians(180));

    private PathChain drivePath;
    private PathChain backPath;
    private PathChain gardenPath;
    private PathChain flowerPath;
    private PathChain sameShotPath;
    private PathChain diffAcrossPath;
    private PathChain diffDownPath;


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

        // Garden -> flower (facing 0 the whole way)
        flowerPath = follower.pathBuilder()
                .addPath(new BezierLine(gardenPose, flowerPose))
                .setConstantHeadingInterpolation(flowerPose.getHeading())
                .build();

        // Flower -> where we shot first
        sameShotPath = follower.pathBuilder()
                .addPath(new BezierLine(flowerPose, shootPose))
                .setLinearHeadingInterpolation(
                        flowerPose.getHeading(),
                        shootPose.getHeading()
                )
                .build();

        // Flower -> across the field (already facing 0)
        diffAcrossPath = follower.pathBuilder()
                .addPath(new BezierLine(flowerPose, diffCornerPose))
                .setLinearHeadingInterpolation(
                        flowerPose.getHeading(),
                        diffCornerPose.getHeading()
                )
                .build();

        // After turning right: drive forward down to the different shoot pose
        diffDownPath = follower.pathBuilder()
                .addPath(new BezierLine(diffDownStartPose, diffShotPose))
                .setConstantHeadingInterpolation(diffDownStartPose.getHeading())
                .build();
    }


    // =========================================================
    // INIT
    // =========================================================

    @Override
    public void init() {

        // INTAKE DISABLED
        // intake.init(hardwareMap, "intake");

        atlimelight.init(hardwareMap, 0);   // pipeline 0

        follower = Constants.createFollower(hardwareMap);

        buildPaths();

        follower.setPose(startPose);

        shootTimer = new ElapsedTime();
        alignTimer = new ElapsedTime();

        // -------------------------
        // SHOOTER (DISABLED)
        // -------------------------
        // shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        //
        // shooter.setMode(
        //         com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER
        // );
        // shooter.setMode(
        //         com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER
        // );
        //
        // PIDFCoefficients flywheelPIDF =
        //         new PIDFCoefficients(
        //                 FLYWHEEL_P,
        //                 FLYWHEEL_I,
        //                 FLYWHEEL_D,
        //                 FLYWHEEL_F
        //         );
        //
        // shooter.setPIDFCoefficients(
        //         com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER,
        //         flywheelPIDF
        // );

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

    // Same tags the Clusters class checks for
    private boolean canSeeShootTag() {
        return atlimelight.hasTag(34)
                || atlimelight.hasTag(35)
                || atlimelight.hasTag(36)
                || atlimelight.hasTag(37);
    }


    // =========================================================
    // TURN HELPERS
    // =========================================================

    // Wrap to [0, 2PI) - turnTo targets are kept positive (e.g. 270 not -90)
    private double wrapTwoPi(double a) {
        double twoPi = 2.0 * Math.PI;
        return ((a % twoPi) + twoPi) % twoPi;
    }

    // Normalized angular difference to [-PI, PI]
    private double headingError(double targetRad) {
        return Math.atan2(
                Math.sin(targetRad - currentPose.getHeading()),
                Math.cos(targetRad - currentPose.getHeading())
        );
    }

    // Shows turn telemetry and returns true when the turn is finished
    private boolean turnDone(double targetRad, double toleranceRad, String label) {

        double err = headingError(targetRad);

        telemetry.addData(label + " Heading", "%.2f°", Math.toDegrees(currentPose.getHeading()));
        telemetry.addData(label + " Target", "%.2f°", Math.toDegrees(targetRad));
        telemetry.addData(label + " Error", "%.2f°", Math.toDegrees(err));
        telemetry.addData(label + " Busy", follower.isBusy());

        return !follower.isBusy() || Math.abs(err) < toleranceRad;
    }


    // =========================================================
    // LOOP
    // =========================================================

    @Override
    public void loop() {

        follower.update();
        atlimelight.update();

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

        telemetry.addData("Sees Shoot Tag", canSeeShootTag() ? "YES" : "NO");

        // Simulated hardware readouts (nothing attached)
        telemetry.addData(
                "Sim Shooter RPM",
                "%.2f",
                simTargetRPM
        );

        telemetry.addData(
                "Sim Intake Power",
                "%.1f",
                simIntakePower
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

        if (pathState == PathState.SHOOT_FIRST ||
                pathState == PathState.SHOOT_SECOND ||
                pathState == PathState.SHOOT_THIRD) {

            if (shootTimerStarted) {
                telemetry.addData(
                        "Fake Shoot Timer",
                        "%.1f s",
                        shootTimer.seconds()
                );
            }
        }


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
            // FIRST SHOOT (FAKE)
            // =====================================================

            case SHOOT_FIRST:

                if (!shootTimerStarted) {

                    simTargetRPM = calculateShot(
                            currentPose.getX(),
                            currentPose.getY(),
                            currentPose.getHeading()
                    );

                    // double targetTicksPerSec = (simTargetRPM / 60.0) * TICKS_PER_REV;
                    // shooter.setVelocity(targetTicksPerSec);

                    shotMessage = "FAKE SHOOTING #1 at "
                            + Math.round(simTargetRPM) + " RPM";

                    shootTimer.reset();
                    shootTimerStarted = true;
                }

                if (shootTimer.seconds() >= FAKE_SHOOT_SECONDS) {

                    // shooter.setVelocity(0);
                    simTargetRPM = 0.0;
                    shotMessage = "Fake shot #1 done";

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

                if (!follower.isBusy() ||
                        Math.abs(turnOneError) < Math.toRadians(5)) {

                    // intake.setMotorSpeed(-1.0);
                    simIntakePower = -1.0;

                    setPathState(PathState.DRIVETOGARDEN);

                    follower.followPath(gardenPath, true);
                }

                break;


            // =====================================================
            // DRIVE TO GARDEN
            // =====================================================

            case DRIVETOGARDEN:

                if (!follower.isBusy()) {

                    // intake.setMotorSpeed(0);
                    simIntakePower = 0.0;

                    setPathState(PathState.TURNTWO);
                }

                break;


            // =====================================================
            // TURN TOWARD THE GOAL FOR SECOND SHOT
            // =====================================================

            case TURNTWO:

                if (!stateInitialized) {

                    // Heading from where we are now to the goal
                    turnTwoTarget = Math.toRadians(315);

                    follower.turnTo(turnTwoTarget);

                    stateInitialized = true;
                }

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
                        "TURNTWO Target (goal)",
                        "%.2f°",
                        Math.toDegrees(turnTwoTarget)
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

                    setPathState(PathState.CHECK_TAG);
                }

                break;


            // =====================================================
            // CHECK FOR A TAG BEFORE SECOND SHOT
            // =====================================================

            case CHECK_TAG:

                if (canSeeShootTag()) {
                    shotMessage = "Tag seen - aligning";
                    setPathState(PathState.ALIGN_TAG);
                } else {
                    shotMessage = "No tag seen - skipping second shot";
                    setPathState(PathState.DONE);
                }

                break;


            // =====================================================
            // ALIGN TO TAG (Limelight tx)
            // =====================================================

            case ALIGN_TAG:

                if (!stateInitialized) {
                    alignTimer.reset();
                    alignAttempts = 0;
                    stateInitialized = true;
                }

                boolean alignValid = atlimelight.hasValidResult();
                double alignTx = alignValid ? atlimelight.getTx() : 0.0;

                telemetry.addData("Align Valid", alignValid);
                telemetry.addData("Align tx", "%.2f°", alignTx);
                telemetry.addData("Align Attempts", alignAttempts);

                if (alignTimer.seconds() >= ALIGN_TIMEOUT_SECONDS) {

                    // Couldn't get perfectly lined up in time - shoot anyway
                    shotMessage = "Align timed out - fake shooting anyway";
                    setPathState(PathState.SHOOT_SECOND);

                } else if (!follower.isBusy()) {

                    if (alignValid && Math.abs(alignTx) < ALIGN_TOLERANCE_DEG) {

                        shotMessage = "Aligned!";
                        setPathState(PathState.SHOOT_SECOND);

                    } else if (alignValid) {

                        // Turn by the remaining tx error, then re-check once done
                        follower.turnTo(
                                currentPose.getHeading() + Math.toRadians(alignTx)
                        );
                        alignAttempts++;
                    }
                }

                break;


            // =====================================================
            // SECOND SHOOT (FAKE)
            // =====================================================

            case SHOOT_SECOND:

                if (!shootTimerStarted) {

                    simTargetRPM = calculateShot(
                            currentPose.getX(),
                            currentPose.getY(),
                            currentPose.getHeading()
                    );

                    // double targetTicksPerSec = (simTargetRPM / 60.0) * TICKS_PER_REV;
                    // shooter.setVelocity(targetTicksPerSec);

                    shotMessage = "FAKE SHOOTING #2 at "
                            + Math.round(simTargetRPM) + " RPM";

                    shootTimer.reset();
                    shootTimerStarted = true;
                }

                if (shootTimer.seconds() >= FAKE_SHOOT_SECONDS) {

                    // shooter.setVelocity(0);
                    simTargetRPM = 0.0;
                    shotMessage = "Fake shot #2 done";

                    shootTimerStarted = false;

                    setPathState(PathState.RECHECK_TAG);
                }

                break;


            // =====================================================
            // LOOK FOR TAGS AGAIN AFTER SHOOTING, THEN HEAD TO FLOWER
            // =====================================================

            case RECHECK_TAG:

                sawTagAfterSecondShot = canSeeShootTag();

                if (sawTagAfterSecondShot) {
                    postShotMessage = "shoot yes - tag still visible";
                    routeMessage = "Flower -> SAME shot";
                } else {
                    // no tag: no "shoot yes" message
                    routeMessage = "Flower -> DIFFERENT shot";
                }

                // Intake on while driving to the flower
                // intake.setMotorSpeed(-1.0);
                simIntakePower = -1.0;

                setPathState(PathState.TO_FLOWER);

                follower.followPath(flowerPath, true);

                break;


            // =====================================================
            // DRIVE TO FLOWER (INTAKING)
            // =====================================================

            case TO_FLOWER:

                if (!follower.isBusy()) {

                    // intake.setMotorSpeed(0);
                    simIntakePower = 0.0;

                    if (sawTagAfterSecondShot) {

                        setPathState(PathState.SAME_SHOT);
                        follower.followPath(sameShotPath, true);

                    } else {

                        setPathState(PathState.DIFF_SHOT);
                        follower.followPath(diffAcrossPath, true);
                    }
                }

                break;


            // =====================================================
            // SAME SHOT: DRIVE BACK TO WHERE WE SHOT FIRST
            // =====================================================

            case SAME_SHOT:

                if (!follower.isBusy()) {
                    setPathState(PathState.TURN_TO_GOAL);
                }

                break;


            // =====================================================
            // DIFFERENT SHOT, STEP 1: DRIVE ACROSS THE FIELD
            // =====================================================

            case DIFF_SHOT:

                if (!follower.isBusy()) {
                    setPathState(PathState.DIFF_TURN);
                }

                break;


            // =====================================================
            // DIFFERENT SHOT, STEP 2: TURN RIGHT (0 -> 270 / -90)
            // =====================================================

            case DIFF_TURN:

                double diffTurnTarget = Math.toRadians(270);

                if (!stateInitialized) {

                    follower.turnTo(diffTurnTarget);

                    stateInitialized = true;
                }

                if (turnDone(diffTurnTarget, Math.toRadians(5), "DIFF_TURN")) {

                    setPathState(PathState.DIFF_DOWN);

                    follower.followPath(diffDownPath, true);
                }

                break;


            // =====================================================
            // DIFFERENT SHOT, STEP 3: DRIVE FORWARD TO THE SHOOT POSE
            // =====================================================

            case DIFF_DOWN:

                if (!follower.isBusy()) {
                    setPathState(PathState.TURN_TO_GOAL);
                }

                break;


            // =====================================================
            // TURN TOWARD THE GOAL (both routes end up here)
            // =====================================================

            case TURN_TO_GOAL:

                if (!stateInitialized) {

                    // Heading from where we are now to the goal, kept in [0, 2PI)
                    goalTurnTarget = wrapTwoPi(Math.atan2(
                            TARGET_Y - currentPose.getY(),
                            TARGET_X - currentPose.getX()
                    ));

                    follower.turnTo(goalTurnTarget);

                    stateInitialized = true;
                }

                if (turnDone(goalTurnTarget, Math.toRadians(3), "GOAL TURN")) {
                    setPathState(PathState.SHOOT_THIRD);
                }

                break;


            // =====================================================
            // THIRD SHOOT (FAKE)
            // =====================================================

            case SHOOT_THIRD:

                if (!shootTimerStarted) {

                    simTargetRPM = calculateShot(
                            currentPose.getX(),
                            currentPose.getY(),
                            currentPose.getHeading()
                    );

                    // double targetTicksPerSec = (simTargetRPM / 60.0) * TICKS_PER_REV;
                    // shooter.setVelocity(targetTicksPerSec);

                    shotMessage = "FAKE SHOOTING #3 at "
                            + Math.round(simTargetRPM) + " RPM";

                    shootTimer.reset();
                    shootTimerStarted = true;
                }

                if (shootTimer.seconds() >= FAKE_SHOOT_SECONDS) {

                    // shooter.setVelocity(0);
                    simTargetRPM = 0.0;
                    shotMessage = "Fake shot #3 done";

                    shootTimerStarted = false;

                    setPathState(PathState.DONE);
                }

                break;


            // =====================================================
            // DONE
            // =====================================================

            case DONE:

                // shooter.setVelocity(0);
                // intake.setMotorSpeed(0);
                simTargetRPM = 0.0;
                simIntakePower = 0.0;

                break;
        }

        // Update at the end so everything added inside the states shows up
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