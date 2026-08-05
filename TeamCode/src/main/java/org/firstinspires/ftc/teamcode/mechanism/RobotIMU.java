package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * ============================================================================
 *  ROBOTIMU: STANDALONE CONTROL/EXPANSION HUB IMU WRAPPER
 * ============================================================================
 *
 *  Thin wrapper around the Control Hub / Expansion Hub's built-in IMU
 *  (gyro + accelerometer), exposing just the current yaw/heading reading.
 *  Not currently used by any OpMode in this codebase (nothing calls
 *  RobotIMU -- see TestRobotIMU.java, which is its bench-test harness but
 *  has its own separate init/read-order issues, covered in that file's own
 *  comments later in this pass).
 *
 *  !!! CROSS-FILE NOTE: TWO OTHER, DIFFERENT SOURCES OF "HEADING" EXIST !!!
 *  This is not the only place in the codebase that reads robot orientation.
 *  Be aware there are effectively THREE independent orientation sources
 *  across this codebase, which are NOT guaranteed to read identically at
 *  any given moment:
 *    1. This class (RobotIMU) -- reads the Control/Expansion Hub's own
 *       internal IMU directly via the FTC SDK's IMU class, device name
 *       "imu".
 *    2. AprilTagLimelight.java -- ALSO reads the Control/Expansion Hub's
 *       IMU directly (same device name "imu", same
 *       RevHubOrientationOnRobot orientation parameters), independently of
 *       this class, purely to feed yaw into the Limelight for its MegaTag2
 *       pose solve. If BOTH this class and AprilTagLimelight are
 *       initialized in the same OpMode, imu.initialize() would effectively
 *       be called twice on the same underlying hardware device -- probably
 *       harmless, but worth knowing if you ever see a heading value
 *       unexpectedly reset/reset-to-zero part way through an OpMode.
 *    3. The GoBilda Pinpoint odometry computer (Constants.java's
 *       PinpointConstants / CustomPathing.java's direct pinpoint usage) --
 *       computes its OWN heading estimate from its own onboard sensors,
 *       independent of the Control/Expansion Hub's IMU entirely. This is
 *       the heading source PedroPathing-based OpModes and CustomPathing.java
 *       actually drive off of -- NOT this RobotIMU class.
 *  If a driver-facing heading readout (e.g. RobotIMU's output) ever seems
 *  to disagree with the robot's actual driving/pathing heading, remember
 *  these are genuinely different sensors with independent drift
 *  characteristics, not two readings of the same underlying value.
 *
 *  Reference docs:
 *   - FTC SDK IMU / IMU.Parameters / RevHubOrientationOnRobot (how to tell
 *     the SDK which way the hub is physically mounted so yaw/pitch/roll are
 *     reported correctly): https://ftc-docs.firstinspires.org/
 * ============================================================================
 */
public class RobotIMU {
    private IMU imu;

    /**
     * @param hwMap the OpMode's hardwareMap; looks up the hub IMU under the
     *              device name "imu" (must match the Driver Station
     *              configuration, and must be consistent with
     *              AprilTagLimelight.java's own "imu" lookup -- see the
     *              class-level note above about both classes independently
     *              initializing the same physical device)
     */
    public void init(HardwareMap hwMap){
        imu = hwMap.get(IMU.class, "imu");

        // Tells the SDK how the Control/Expansion Hub is physically mounted
        // on the robot (which way its logo and USB ports face) so it can
        // correctly translate the hub's raw internal IMU axes into
        // robot-frame yaw/pitch/roll. LogoFacingDirection.UP +
        // UsbFacingDirection.FORWARD describes one specific hub mounting
        // orientation -- if the hub is ever physically remounted in a
        // different orientation, THIS is the configuration that must change
        // to match, in both this class and AprilTagLimelight.java (which
        // uses the identical orientation parameters).
        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        imu.initialize(new IMU.Parameters(RevOrientation));
    }

    /**
     * @param angleUnit DEGREES or RADIANS -- determines the unit of the
     *                  returned yaw value
     * @return the hub IMU's current yaw (heading) reading, in the
     *         requested unit. This is a LIVE call into the IMU each time --
     *         it is not cached, so calling this frequently (e.g. every
     *         loop) has real per-call cost, though typically small.
     */
    public double getHeading(AngleUnit angleUnit){
        return imu.getRobotYawPitchRollAngles().getYaw(angleUnit);
    }
}