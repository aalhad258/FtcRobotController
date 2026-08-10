package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanism.DcMotor;

/**
 * ============================================================================
 *  BENCH TEST: SINGLE GENERIC MOTOR + ENCODER VERIFICATION
 * ============================================================================
 *
 *  Bare-bones bench harness for the DcMotor mechanism wrapper (see
 *  DcMotor.java) -- lets you spin one motor from the left stick and watch
 *  its encoder-derived tick count / revolution count on the driver station.
 *  Intended for bring-up testing a NEW motor/mechanism in isolation (e.g.
 *  confirming a freshly-wired arm/intake/lift motor actually turns the
 *  right way and its encoder counts up correctly) before it's built into a
 *  dedicated mechanism class of its own -- this is NOT the drivetrain (see
 *  TestDrive.java for that).
 *
 *  !!! SIGN-CONVENTION NOTE, WORTH DOUBLE-CHECKING ON THE BENCH !!!
 *  Unlike TestDrive.java (and every other stick-driven OpMode in this
 *  codebase), the raw gamepad1.left_stick_y value is used here DIRECTLY as
 *  motorSpeed, WITHOUT the `-` negation applied everywhere else (see
 *  TestDrive.java's `axial = -gamepad1.left_stick_y`). Since FTC gamepad
 *  sticks report a NEGATIVE y value when pushed forward/away from the
 *  driver, that means pushing the stick forward here will command
 *  NEGATIVE motor power (spin one way), and pulling it back will command
 *  POSITIVE power (spin the other way) -- the opposite intuitive mapping
 *  used elsewhere in this codebase. This is intentional for this
 *  particular bench test (there's no "forward" concept for a single
 *  arbitrary test motor the way there is for a drivetrain), but it's worth
 *  knowing about so it doesn't cause confusion if you're expecting
 *  "push stick up = motor spins one particular way" to match TestDrive's
 *  convention.
 * ============================================================================
 */

@Disabled
@TeleOp
public class TestDcMotor extends OpMode {
    DcMotor bench = new DcMotor();

    @Override
    public void init() {
        bench.init(hardwareMap);

        // --- ADDED TELEMETRY: confirm init completed and show the motor's rated
        // ticks-per-rev right away, before any motion, so a bad/unexpected motor
        // type in the hardware config is visible immediately rather than only being
        // inferable later from odd getMotorRevs() output ---
        telemetry.addLine("TestDcMotor: init complete");
        telemetry.addData("configured ticks per rev", bench.getTicksPerRev());
        telemetry.update();
    }

    @Override
    public void loop() {
        double motorSpeed = gamepad1.left_stick_y;
        bench.setMotorSpeed(motorSpeed);
        telemetry.addData("Ticks per Revolutions", bench.getTicksPerRev());
        telemetry.addData("current pos", bench.currentPosition());
        telemetry.addData("Motor Revs", bench.getMotorRevs());

        // --- ADDED TELEMETRY: the commanded speed and raw stick value side by side,
        // so the sign-convention behavior noted above is visible in real time rather
        // than something you have to remember from this file's comments ---
        telemetry.addData("commanded motorSpeed", motorSpeed);
        telemetry.addData("raw left_stick_y", gamepad1.left_stick_y);
        telemetry.addData("loop runtime (s)", getRuntime());
        telemetry.update();
    }
}