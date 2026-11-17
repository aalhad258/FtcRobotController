//Only for team robot.
package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * ============================================================================
 *  PEDROPATHING GLOBAL CONFIGURATION - TEAM ROBOT
 * ============================================================================
 *
 *  This class is the single source of truth for every tuning constant that
 *  PedroPathing uses to drive the team robot.
 *  Call Constants_Team.createFollower(hardwareMap) to build a fully configured
 *  Follower instance from the values below.
 *
 *  Pedro 2.x version. The Pedro 3.x version is saved in TeamCode/pedro3-backup.
 * ============================================================================
 */

public class Constants_Team {

    /**
     * ------------------------------------------------------------------
     *  FOLLOWER CONSTANTS
     * ------------------------------------------------------------------
     */
    public static FollowerConstants followerConstants = new FollowerConstants()
            .translationalPIDFCoefficients(new PIDFCoefficients(0.075, 0, 0, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(0.7, 0, 0, 0))
            .forwardZeroPowerAcceleration(-41.84250218343561)
            .lateralZeroPowerAcceleration(-49.46989327634026);

    /**
     * ------------------------------------------------------------------
     *  DRIVETRAIN (MECANUM) CONSTANTS
     * ------------------------------------------------------------------
     *  !!! IMPORTANT ROBOT-SPECIFIC DESIGN NOTE !!!
     *  Our mecanum wheels are mounted in an "O" roller configuration
     *  (rollers form an O shape when viewed from above) rather than the
     *  more common "X" configuration.
     *  Device names and directions:
     *      leftFrontName  -> "leftFrontMotor"
     *      backLeftName   -> "leftBackMotor"
     *      frontRightName -> "rightFrontMotor"
     *      backRightName  -> "rightBackMotor"
     */
    public static MecanumConstants drivetrainConstants = new MecanumConstants()
            .leftFrontMotorName("lfm")
            .leftRearMotorName("lbm")
            .rightFrontMotorName("rfm")
            .rightRearMotorName("rbm")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(43.88266207477239)
            .yVelocity(41.0740498219888);

    /**
     * ------------------------------------------------------------------
     *  LOCALIZER (GOBILDA PINPOINT ODOMETRY) CONSTANTS
     * ------------------------------------------------------------------
     */
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .hardwareMapName("pinpoint")
            .forwardPodY(0.0)
            .strafePodX(-5.0)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .distanceUnit(DistanceUnit.INCH)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

    /**
     * Builds a fully-configured PedroPathing Follower using the
     * constants above.
     *
     * @param hardwareMap the OpMode's hardwareMap
     * @return a Follower instance
     */
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(PathConstraints.defaultConstraints)
                .mecanumDrivetrain(drivetrainConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
