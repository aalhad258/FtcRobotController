package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * GamePadPractice
 * ----------------
 * ARCHITECTURE / PURPOSE:
 *   This is a scratch/practice OpMode used to explore gamepad input behavior (stick curves,
 *   trigger sums, button reads, etc.) in isolation from any real drivetrain or mechanism code.
 *   It does NOT drive any hardware -- there is no HardwareMap usage and no Robot/Drive
 *   instantiation. It exists purely to read raw gamepad1 values and print them to the Driver
 *   Station via telemetry so a driver/programmer can sanity-check joystick/trigger ranges and
 *   sign conventions before wiring them into real control code (e.g., TeleOp.java's drivetrain
 *   mixing math).
 *
 * WHY IT'S @Disabled:
 *   FTC's SDK "OpMode" annotation processor lists every @TeleOp/@Autonomous class on the Driver
 *   Station's opmode list. @Disabled prevents this practice/experimental OpMode from cluttering
 *   that list (and from being accidentally selected) during actual matches or normal practice
 *   driving. To run it intentionally, temporarily remove/comment out the @Disabled annotation,
 *   redeploy, then re-add it afterward -- don't leave it enabled for competition builds.
 *   Reference: FTC SDK docs on OpMode annotations (@TeleOp, @Autonomous, @Disabled).
 *
 * CONTROL SCHEME UNDER TEST (gamepad1 only):
 *   - left_stick_y  -> "speedForward": inverted (FTC sticks report negative Y for "up"/forward
 *                      push) and halved (divided by 2.0) here, likely to test a reduced-speed /
 *                      "practice mode" throttle scaling before committing that scale factor to
 *                      real drive code.
 *   - left_stick_x vs right_stick_x -> "difference": exploring whether left-stick-x minus
 *                      right-stick-x is a useful turning/strafe input signal shape.
 *   - left_trigger + right_trigger -> "sumTriggers": exploring combined trigger magnitude, e.g.
 *                      for a boost/intake-power input that should ignore which trigger is pressed.
 *   - a / b buttons -> raw boolean reads, likely just to confirm button wiring/response.
 */
@Disabled
@TeleOp
public class GamePadPractice extends OpMode {
    @Override
    public void init() {
        // No hardware to initialize -- this OpMode is gamepad-input-only, so init() is
        // intentionally empty. If this class is ever extended to drive real hardware, hardware
        // map lookups (e.g., hardwareMap.get(...)) would belong here, run once before start.
    }

    @Override
    public void loop() {
        // DESIGN NOTE: FTC gamepad sticks report left_stick_y as POSITIVE when pushed DOWN and
        // NEGATIVE when pushed UP (this is the FTC/Android gamepad convention, not a bug). The
        // negation below (-gamepad1.left_stick_y) flips that so pushing the stick FORWARD (up)
        // yields a POSITIVE speedForward value, matching typical "forward = positive" robot
        // convention. Dividing by 2.0 halves max speed -- likely a deliberate "practice mode"
        // throttle cap so new drivers/testers can't immediately full-send the input.
        double speedForward = -gamepad1.left_stick_y / 2.0;

        // Simple difference between the two sticks' X axes -- being evaluated here as a candidate
        // turning/strafe signal shape (no drivetrain math is actually applied to it in this file).
        double difference = gamepad1.left_stick_x - gamepad1.right_stick_x;

        // Combined trigger pull magnitude, ignoring which trigger was pressed. Each FTC trigger
        // axis reports 0.0 (released) to 1.0 (fully pressed), so this sum ranges 0.0-2.0 if both
        // triggers are pressed simultaneously.
        double sumTriggers = gamepad1.left_trigger + gamepad1.right_trigger;

        // --- Telemetry: original values (unchanged) ---
        telemetry.addData("x left", gamepad1.left_stick_x);
        telemetry.addData("y left", speedForward);
        telemetry.addData("a button", gamepad1.a);
        telemetry.addData("b button", gamepad1.b);
        telemetry.addData("x right", gamepad1.right_stick_x);
        telemetry.addData("y right", gamepad1.right_stick_y);
        telemetry.addData("difference", difference);
        telemetry.addData("sum of the triggers", sumTriggers);

        // --- ADDED telemetry: raw/unmodified stick values for direct comparison against the
        // derived values above. Useful when validating sign conventions (e.g., confirming
        // left_stick_y really is negative-up) or diagnosing a misbehaving/miscalibrated gamepad
        // stick during pit troubleshooting. ---
        telemetry.addData("[raw] left_stick_y (unflipped)", gamepad1.left_stick_y);
        telemetry.addData("[raw] left_trigger", gamepad1.left_trigger);
        telemetry.addData("[raw] right_trigger", gamepad1.right_trigger);

        // --- ADDED telemetry: loop timing, useful for spotting an OpMode that's running slower
        // than expected (a classic source of "sluggish" or "laggy" driver feel in competition). ---
        telemetry.addData("[loop] time (s)", getRuntime());
    }
}