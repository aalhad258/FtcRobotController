package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * ============================================================================
 *  MECANUM DRIVETRAIN MECHANISM (LOW-LEVEL, NO PEDROPATHING)
 * ============================================================================
 *
 *  This is the low-level "4 motors -> mecanum kinematics" wrapper used by
 *  every OpMode in this codebase that drives the robot WITHOUT going
 *  through PedroPathing's Follower (see CustomPathing.java, TestDrive.java,
 *  AprilTagAutoAlignment.java). PedroPathing-based OpModes (LimeLightBasic,
 *  SampleAutoPathing) instead go through Follower/MecanumConstants in
 *  Constants.java, which drives the SAME four physical motors through
 *  PedroPathing's own internal kinematics rather than this class.
 *
 *  Reference docs:
 *   - Standard mecanum drive kinematics (the axial/lateral/yaw -> 4-wheel
 *     power formula used in setPower() below is the classic "X-pattern"
 *     equations): https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
 *   - FTC DcMotor / DcMotorSimple.Direction / ZeroPowerBehavior:
 *     https://ftc-docs.firstinspires.org/
 * ============================================================================
 */
public class Drive {
    private DcMotor lfMotor;
    private DcMotor lbMotor;
    private DcMotor rfMotor;
    private DcMotor rbMotor;

    public void init(HardwareMap hwMap) {
        lfMotor = hwMap.get(DcMotor.class, "lfm");
        lbMotor = hwMap.get(DcMotor.class, "lbm");
        rfMotor = hwMap.get(DcMotor.class, "rfm");
        rbMotor = hwMap.get(DcMotor.class, "rbm");
        // RUN_WITHOUT_ENCODER: pure open-loop power control, no internal
        // velocity PID from the motor controller itself. Set explicitly here
        // (rather than relying on SDK default) so it's unambiguous that this
        // drivetrain is meant to be driven by raw power values, not
        // encoder-controlled velocity/position targets.
        lfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lbMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rbMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // BRAKE: motors actively resist being turned when commanded power is
        // 0 (as opposed to FLOAT, which lets them coast freely). This makes
        // the robot stop crisply when the driver releases the sticks or a
        // PID loop commands 0 power, which matters both for precise manual
        // driving and for autonomous PID loops (CustomPathing.java,
        // AprilTagAutoAlignment.java) that rely on the robot actually
        // stopping rather than drifting once error is within tolerance.
        lfMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lbMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rfMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rbMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lfMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        lbMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rfMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rbMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    /**
     * Converts a desired robot-relative motion (drive forward/back, strafe
     * left/right, and rotate) into individual power values for each of the
     * four mecanum wheels
     *
     * @param axial   forward(+)/backward(-) power, -1.0 to 1.0
     * @param lateral strafe right(+)/left(-) power, -1.0 to 1.0 (sign
     *                convention depends on the caller -- see each OpMode's
     *                own comments for how it maps its inputs to this
     *                parameter)
     * @param yaw     rotate power, -1.0 to 1.0 (sign convention likewise
     *                depends on the caller)
     * @param speed   global scalar multiplier applied AFTER normalization
     *                (e.g. for a driver-selectable "slow mode"); 1.0 = full
     *                available power
     */
    public void setPower(double axial, double lateral, double yaw, double speed) {
        // accepts values from -1.0 to 0.1
        double max;

        double lfPower = axial + lateral + yaw;
        double lbPower = axial - lateral - yaw;
        double rfPower = axial - lateral + yaw;
        double rbPower = axial + lateral - yaw;

        // Find the largest-magnitude commanded power across all 4 wheels.
        max = Math.max(Math.abs(lfPower), Math.abs(rfPower));
        max = Math.max(max, Math.abs(lbPower));
        max = Math.max(max, Math.abs(rbPower));

        // If any wheel's raw power would exceed +/-1.0 (which the axial +
        // lateral + yaw sum can easily do when multiple inputs are large
        // simultaneously), scale ALL FOUR wheels down proportionally by the
        // same factor. This preserves the RATIO of power between wheels
        // (and therefore the intended direction of motion) instead of just
        // clipping each wheel independently, which would distort the
        // robot's actual heading of travel under combined stick inputs.
        if (max > 1.0) {
            lfPower /= max;
            rfPower /= max;
            lbPower /= max;
            rbPower /= max;
        }

        // `speed` is applied last, after normalization, as a simple global
        // throttle -- e.g. an OpMode could pass speed=0.5 for a driver-
        // selected slow/precision mode without needing to touch the
        // kinematics or normalization logic above.
        lfMotor.setPower(lfPower*speed);
        lbMotor.setPower(lbPower*speed);
        rfMotor.setPower(rfPower*speed);
        rbMotor.setPower(rbPower*speed);
    }
}