package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * ============================================================================
 *  PEDROPATHING GLOBAL CONFIGURATION
 * ============================================================================
 *
 *  This class is the single source of truth for every tuning constant that
 *  PedroPathing (the follower/path-planning library, see
 *  https://pedropathing.com/ and the PedroPathing javadocs) uses to drive
 *  the robot. Every Autonomous OpMode that uses a Follower (see
 *  CustomPathing.java, SampleAutoPathing.java, LimeLightBasic.java) calls
 *  Constants.createFollower(hardwareMap) to build a fully configured
 *  Follower instance from the values below.
 *
 *  IF AUTO IS MISBEHAVING AT COMPETITION, THIS IS ONE OF THE FIRST FILES TO
 *  CHECK:
 *    - Robot overshooting / oscillating around target points -> PIDF gains
 *      in followerConstants are likely too aggressive (too high) or too
 *      weak (too low). See translationalPIDFCoefficients / headingPIDFCoefficients.
 *    - Robot drifts off a straight line while driving fast -> zero power
 *      acceleration constants or centripetal scaling need retuning.
 *    - Robot strafes/drives the wrong direction entirely, or spins instead
 *      of strafing -> check driveConstants motor name <-> physical port
 *      mapping AND motor directions below. This is the most common single
 *      point of failure after a robot rebuild or motor swap.
 *    - Odometry position reads garbage / drifts over a match -> check
 *      localizerConstants (pod offsets, encoder directions, encoder
 *      resolution / pod type).
 *
 *  NOTE: All tuning values below (mass, accelerations, PIDF gains,
 *  velocities) were derived from PedroPathing's official tuning process
 *  (the "Tuning" OpModes bundled with the PedroPathing quickstart -
 *  see https://pedropathing.com/docs/tuning/). They are ROBOT-SPECIFIC.
 *  If the chassis, weight, or wheels change, these MUST be re-tuned using
 *  that process again - do not hand-edit them without re-running the
 *  tuner, or path-following accuracy will silently degrade.
 * ============================================================================
 */
public class Constants {

    /**
     * ------------------------------------------------------------------
     *  FOLLOWER CONSTANTS
     * ------------------------------------------------------------------
     *  These are the "physics + control loop" tuning values PedroPathing
     *  uses to model how the robot accelerates/decelerates and to correct
     *  path-following error. Reference: PedroPathing FollowerConstants
     *  javadoc / tuning guide (https://pedropathing.com/docs/tuning/).
     */
    public static FollowerConstants followerConstants = new FollowerConstants()
            // Robot mass in kilograms. Used by PedroPathing's motion profiling
            // to predict how much power is needed to accelerate/decelerate the
            // robot along a path. Measured directly (put robot on a scale),
            // NOT auto-tuned. If you add/remove significant hardware (e.g. a
            // new arm, battery relocation), re-weigh the robot and update this.
            .mass(5.5)

            // Forward zero-power acceleration: how quickly the robot decelerates
            // (in in/sec^2) when forward/backward motor power is cut to zero,
            // i.e. how much the robot "coasts" forward. Negative because it's a
            // deceleration. Found via PedroPathing's ForwardZeroPowerAcceleration
            // tuner OpMode. Directly affects how PedroPathing predicts stopping
            // distance at the end of a path -- if the robot consistently
            // overshoots/undershoots the END of paths (not mid-path), suspect
            // this value first.
            .forwardZeroPowerAcceleration(-41.84250218343561)

            // Same idea as above, but for lateral (strafing) motion. Mecanum
            // wheels have different friction characteristics strafing vs.
            // driving forward, which is why this is tuned and stored separately
            // from forwardZeroPowerAcceleration.
            .lateralZeroPowerAcceleration(-49.46989327634026)

            // Translational PIDF: corrects XY position error while following a
            // path (i.e. "am I physically where the path says I should be right
            // now"). PIDFCoefficients(P, I, D, F):
            //   P = 0.075 proportional response to positional error (inches)
            //   I = 0     no integral term (typically left at 0 to avoid windup)
            //   D = 0.01  damps oscillation / overshoot as error closes
            //   F = 0.023 feedforward term, a small constant "push" so the
            //             controller doesn't rely purely on error to move
            // If the robot "hunts" (oscillates) around the path -> lower P or
            // raise D slightly. If it lags behind the path under load -> raise
            // P or F slightly.
            .translationalPIDFCoefficients(new PIDFCoefficients(0.075, 0, 0.01, 0.023))

            // Heading PIDF: corrects rotational (yaw) error while following a
            // path or during turnTo() calls (see LimeLightBasic.java's ROTATE
            // state, which calls follower.turnTo()). Gains here are much larger
            // than translational because heading (degrees) and position
            // (inches) are different units/scales.
            //   P = 0.5, D = 0.05, F = 0.033
            // If turnTo() overshoots/oscillates the target heading -> lower P
            // or raise D. If it's sluggish to reach heading -> raise P.
            .headingPIDFCoefficients(new PIDFCoefficients(0.5, 0, 0.05, 0.033))

            // Drive PIDF (filtered): the low-level velocity controller that
            // converts "how fast do I want this drivetrain axis moving" into
            // motor power, with an added low-pass filter term (T=0.6, the 4th
            // FilteredPIDFCoefficients argument) to smooth out encoder-velocity
            // noise. This runs "underneath" the translational/heading PIDFs
            // above. If motors are twitchy/jittery during path following
            // (visible/audible motor stutter) -> the filter constant (0.6) or
            // D-term (0.01) here is the first place to look.
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.5, 0.0, 0.01, 0.6, 0.01))

            // Centripetal scaling: how much extra correction power is applied
            // on curved path segments to fight the outward "slingshot" force of
            // turning while translating. If the robot cuts corners / slides
            // wide on curved (non-BezierLine) paths, increase this slightly.
            .centripetalScaling(0.0005);

    /**
     * ------------------------------------------------------------------
     *  DRIVETRAIN (MECANUM) CONSTANTS
     * ------------------------------------------------------------------
     *  !!! IMPORTANT ROBOT-SPECIFIC DESIGN NOTE !!!
     *  Our mecanum wheels are mounted in an "O" roller configuration
     *  (rollers form an O shape when viewed from above) rather than the
     *  more common "X" configuration. Because of this, the physical
     *  front-left/front-right/back-left/back-right roles do NOT line up
     *  1:1 with PedroPathing's expected logical roles the way they would
     *  on a standard "X" mecanum chassis. To compensate, the hardwareMap
     *  device names below are DELIBERATELY CROSSED relative to what you'd
     *  naively expect:
     *      rightFrontMotorName -> "rightBackMotor"   (physical back wheel)
     *      rightRearMotorName  -> "rightFrontMotor"  (physical front wheel)
     *      leftRearMotorName   -> "leftFrontMotor"   (physical front wheel)
     *      leftFrontMotorName  -> "leftBackMotor"    (physical back wheel)
     *  This is NOT a bug/typo -- it is the intentional fix for our O-pattern
     *  roller orientation so that PedroPathing's internal kinematics (which
     *  assume a standard X pattern) still produce correct strafing/rotation
     *  behavior. DO NOT "simplify" or "correct" this mapping back to a
     *  1:1 name match without re-validating strafe direction on the robot,
     *  or strafing will invert / rotation will behave incorrectly.
     *
     *  If at competition the robot strafes the wrong way, or spins in
     *  place when you expect it to strafe, re-check this section AND the
     *  Direction values below before touching PIDF tuning.
     */
    public static MecanumConstants driveConstants = new MecanumConstants()
            // Hard cap (0-1) on motor power PedroPathing will ever command.
            // 1 = no artificial cap; safe to lower temporarily while tuning to
            // reduce risk of a runaway robot.
            .maxPower(1)

            // See the O-vs-X roller note above: these hardwareMap names are
            // intentionally cross-mapped, not a 1:1 physical-to-logical match.
            .rightFrontMotorName("rightBackMotor")
            .rightRearMotorName("rightFrontMotor")
            .leftRearMotorName("leftFrontMotor")
            .leftFrontMotorName("leftBackMotor")

            // Motor spin directions as PedroPathing sees them (post name-swap
            // above). These were determined empirically by running the
            // PedroPathing localization/drive test tuner and observing actual
            // robot motion vs. commanded motion, NOT derived analytically --
            // if you swap a motor's physical port or controller, re-verify
            // these rather than assuming symmetry (e.g. don't assume
            // left/right should mirror each other, since the O-roller +
            // name-swap combination breaks that intuition).
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)

            // Max achievable robot velocity in in/sec along the X (forward)
            // and Y (strafe) axes at full power, from PedroPathing's
            // velocity-tuning OpMode. Used by the motion profiler to plan
            // realistic path timing/speed; if these are wrong, PedroPathing's
            // internal "expected vs actual" comparisons drift and path
            // following gets less accurate the longer/faster a path is.
            .xVelocity(43.88266207477239)
            .yVelocity(41.0740498219888);

    /**
     * ------------------------------------------------------------------
     *  LOCALIZER (GOBILDA PINPOINT ODOMETRY) CONSTANTS
     * ------------------------------------------------------------------
     *  Configures how PedroPathing reads position from the GoBilda
     *  Pinpoint odometry computer (see
     *  https://www.gobilda.com/pinpoint-odometry-computer/). This is the
     *  robot's "GPS" -- if this is misconfigured, the robot will THINK
     *  it's somewhere it isn't, and every path/PID correction above will
     *  be chasing the wrong target. When diagnosing "robot ends up in
     *  totally the wrong spot", check this block before followerConstants.
     */
    public static PinpointConstants localizerConstants = new PinpointConstants()
            // Physical mounting offsets (inches) of the odometry pods from the
            // robot's center of rotation/tracking point. forwardPodY is the
            // pod that measures forward/back travel; its Y offset is how far
            // left(+)/right(-) of center it's mounted. strafePodX is the pod
            // that measures strafe travel; its X offset is how far
            // forward(+)/back(-) of center it's mounted. These must match
            // physical measurements on the robot with a ruler/calipers -- a
            // wrong offset here causes position error that GROWS with
            // rotation (robot position "swims" as it turns even if it isn't
            // actually translating).
            .forwardPodY(-6.5)
            .strafePodX(2)
            .distanceUnit(DistanceUnit.INCH)
            // hardwareMap device name for the Pinpoint computer itself (must
            // match the name configured in the Driver Station/Robot Controller
            // config, and must match the "pinpoint" name used directly in
            // CustomPathing.java's manual GoBildaPinpointDriver usage).
            .hardwareMapName("pinpoint")
            // Must match the physical odometry pod model in use (4-bar pods
            // here). Wrong pod type here silently scales all distance
            // measurements incorrectly (robot thinks it moved a different
            // distance than it actually did).
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            // Encoder count directions for each pod. Both REVERSED here.
            // Determined empirically: push the robot in a known direction and
            // confirm the Pinpoint-reported X/Y increases the way you'd
            // expect; flip these if position tracks backwards relative to
            // real motion.
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    /**
     * ------------------------------------------------------------------
     *  PATH CONSTRAINTS
     * ------------------------------------------------------------------
     *  Governs when PedroPathing considers a path "done" / how tightly it
     *  must track the path before advancing. See PathConstraints javadoc.
     *  Constructor here (in this PedroPathing version) takes, in order:
     *    (tValueConstraint, velocityConstraint, translationalConstraint,
     *     headingConstraint)
     *  i.e.:
     *    0.99 -> the path parameter (t, 0-1 along the path) must reach 0.99
     *            before the path is considered geometrically complete
     *    100  -> velocity constraint (robot must be under this speed,
     *            in/sec, to be considered "settled" at the end of path)
     *    1    -> translational error tolerance (inches) to consider the
     *            end point reached
     *    1    -> heading error tolerance (degrees) to consider the end
     *            heading reached
     *  If autonomous paths finish "early" (robot jerks to the next path
     *  before really arriving) or "late" (robot pauses/hunts at the end
     *  of every path before continuing), these tolerances are the first
     *  thing to loosen/tighten.
     */
    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            100,
            1,
            1);

    /**
     * Builds a fully-configured PedroPathing Follower using all the
     * constants above. Every Autonomous OpMode that drives via paths
     * should call this ONCE in init() and reuse the same Follower
     * instance for the rest of the OpMode's lifetime (see
     * SampleAutoPathing.init() / LimeLightBasic.init() for the standard
     * usage pattern). Do not call this repeatedly inside loop() -- it
     * rebuilds the entire follower/localizer stack and will reset
     * odometry state.
     *
     * @param hardwareMap the OpMode's hardwareMap, used to look up the
     *                    drive motors and the Pinpoint device by the
     *                    names configured above
     * @return a Follower wired up with this robot's tuned physics
     *         (followerConstants), drivetrain wiring (driveConstants),
     *         odometry (localizerConstants), and path-completion
     *         tolerances (pathConstraints)
     */
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}