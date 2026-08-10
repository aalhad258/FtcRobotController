package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * ============================================================================
 *  DCMOTOR: SINGLE GENERIC BENCH-TEST MOTOR WRAPPER
 * ============================================================================
 *
 *  A minimal wrapper around ONE generic DC motor, used by TestDcMotor.java
 *  as a standalone bench test (NOT part of the drivetrain -- contrast with
 *  Drive.java, which wraps the 4 named drivetrain motors). Useful for
 *  bring-up testing a single new motor/mechanism (an arm, intake, lift,
 *  etc.) in isolation, including reading back encoder revolutions, before
 *  it gets wired into a dedicated mechanism class of its own.
 *
 *  !!! NAMING COLLISION GOTCHA, READ BEFORE EDITING THIS FILE !!!
 *  This class is named `DcMotor`, which is THE SAME SIMPLE NAME as the FTC
 *  SDK's own `com.qualcomm.robotcore.hardware.DcMotor` interface (the type
 *  every other motor-handling class in this codebase, like Drive.java,
 *  imports and uses directly). Because of that collision, this file
 *  DELIBERATELY does NOT import com.qualcomm.robotcore.hardware.DcMotor --
 *  it refers to it with its FULLY QUALIFIED name everywhere below
 *  (`com.qualcomm.robotcore.hardware.DcMotor`) so the compiler can tell it
 *  apart from this wrapper class's own name. If we ever add an import for
 *  `com.qualcomm.robotcore.hardware.DcMotor` at the top of this file "to
 *  clean it up", it will immediately collide with this class's own name and
 *  fail to compile (or silently refer to the wrong type, depending on
 *  where it's used) -- leave the fully-qualified references as they are.
 *
 *  Reference docs:
 *   - FTC DcMotor.RunMode (RUN_USING_ENCODER vs RUN_WITHOUT_ENCODER),
 *     ZeroPowerBehavior, MotorConfigurationType.getTicksPerRev():
 *     https://ftc-docs.firstinspires.org/
 * ============================================================================
 */
public class DcMotor {
    // Fully-qualified type reference -- see the class-level naming-collision
    // note above for why this can't just be `DcMotor motor;`.
    private com.qualcomm.robotcore.hardware.DcMotor motor;
    private double ticksPerRev;

    /**
     * Looks up a single motor by the hardwareMap name "motor" (a generic
     * bench-test name -- if wiring this into a real mechanism long-term,
     * this hardcoded name would typically be replaced with something
     * mechanism-specific, e.g. "armMotor" or "intakeMotor").
     *
     * @param hwMap the OpMode's hardwareMap
     */
    public void init(HardwareMap hwMap) {
        // DC motor
        motor = hwMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, "motor");
        // RUN_USING_ENCODER: the motor controller's internal closed-loop
        // velocity control is active, using the encoder to hold a
        // requested speed under varying load -- contrast with Drive.java's
        // drivetrain motors, which explicitly use RUN_WITHOUT_ENCODER for
        // pure open-loop power control. This wrapper is intended for
        // encoder-equipped mechanisms where knowing/controlling actual
        // revolutions matters (see getMotorRevs() below), so closed-loop
        // velocity control is the more appropriate default here.
        motor.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER);
        // Reads the motor's rated encoder ticks-per-revolution from its
        // configured motor type (set in the Driver Station hardware config,
        // e.g. "goBILDA 5202 series" or similar) -- this makes
        // getMotorRevs() below automatically correct for whichever motor
        // model is actually configured, rather than hardcoding a
        // tick-per-rev constant that would silently be wrong if the motor
        // model configuration ever changes.
        ticksPerRev = motor.getMotorType().getTicksPerRev();
        // BRAKE: motor actively resists turning at zero power, rather than
        // coasting -- see Drive.java for the same choice on drivetrain
        // motors and why it matters for precise stopping.
        motor.setZeroPowerBehavior(com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * @param speed desired motor power/speed, -1.0 to 1.0. (The parameter
     *              comment below says "-1.0 to 0.1" in the original source
     *              -- that upper bound looks like a typo for "1.0", but per
     *              your instructions the code and its comments are left
     *              exactly as-is here.)
     */
    public void setMotorSpeed(double speed) {
        // accepts values from -1.0 to 0.1
        motor.setPower(speed);
    }

    /**
     * @return the configured motor type's rated encoder ticks per
     *         revolution (NOT a live reading -- this is fixed metadata from
     *         the motor's configured type, used as the divisor in
     *         getMotorRevs() below)
     */
    public double getTicksPerRev() {
        return ticksPerRev;
    }

    /**
     * @return the raw, cumulative encoder tick count since the motor was
     *         last reset/initialized (RUN_USING_ENCODER mode does not zero
     *         this automatically on init -- if a fresh zero point is
     *         needed, that would require an explicit
     *         STOP_AND_RESET_ENCODER cycle, which this class does not
     *         perform)
     */
    public double currentPosition() {
        return motor.getCurrentPosition();
    }

    /**
     * @return current encoder position converted into motor-shaft
     *         revolutions (currentPosition() / ticksPerRev) -- useful for
     *         sanity-checking a new motor/gearbox against its expected
     *         ticks-per-rev spec during bring-up testing
     */
    public double getMotorRevs() {
        return motor.getCurrentPosition() / ticksPerRev;
    }
}