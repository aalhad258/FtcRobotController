package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanism.Drive;

/**
 * ============================================================================
 *  BENCH TEST: RAW MECANUM DRIVE VERIFICATION (NO PEDROPATHING, NO PID)
 * ============================================================================
 *
 *  Bare-bones manual drive OpMode with nothing but a direct stick -> Drive
 *  mapping and telemetry -- no PedroPathing, no auto-align, no PID. This is
 *  the file to run FIRST after any drivetrain rebuild, motor swap, or
 *  hardware config change, BEFORE trusting any of the PedroPathing- or
 *  PID-based autos (CustomPathing.java, LimeLightBasic.java,
 *  SampleAutoPathing.java, AprilTagAutoAlignment.java) -- because if the
 *  raw kinematics/wiring in Drive.java are wrong, every one of those higher
 *  level OpModes will misbehave in ways that are much harder to diagnose
 *  through their extra layers of PID/vision logic.
 *
 *  HOW TO USE THIS FOR VERIFICATION AT COMPETITION / ON THE BENCH:
 *    - Push the left stick straight up (forward) -> all 4 wheels should
 *      drive the robot straight forward with no rotation or drift.
 *    - Push the left stick straight right -> the robot should strafe
 *      directly right with no forward motion or rotation (this is the
 *      motion most likely to expose an O-vs-X roller/direction wiring
 *      issue -- see the notes in Drive.java -- so it's the most important
 *      one to sanity check after any drivetrain work).
 *    - Push the right stick right -> the robot should rotate clockwise (as
 *      viewed from above) in place.
 *    - If any of the above produces the WRONG motion (e.g. strafing right
 *      instead spins the robot, or driving forward pulls it diagonally),
 *      the fix belongs in Drive.java's Direction flags or hardwareMap
 *      names -- do not try to "compensate" for a wiring bug by inverting
 *      inputs in a higher-level OpMode.
 * ============================================================================
 */
@TeleOp
public class TestDrive extends OpMode {
    Drive drive = new Drive();;
    @Override
    public void init() {
        drive.init(hardwareMap);

        // --- ADDED TELEMETRY: confirm init completed ---
        telemetry.addLine("TestDrive: init complete -- raw stick-to-drivetrain bench test");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Standard FTC gamepad convention: left_stick_y is NEGATIVE when
        // pushed forward, so it's negated to make "push forward" = positive
        // axial (forward) power.
        double axial = -gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;
        drive.setPower(axial, lateral, yaw, 1.0);
        telemetry.addData("axial", axial);
        telemetry.addData("lateral", lateral);
        telemetry.addData("yaw", yaw);

        // --- ADDED TELEMETRY: raw gamepad stick values side-by-side with the axial/
        // lateral/yaw values actually sent to Drive.setPower(), so a sign/mapping bug
        // can be spotted at a glance (e.g. "stick says X but axial says Y") rather than
        // having to reason about the negation above from memory ---
        telemetry.addData("raw left_stick_x", gamepad1.left_stick_x);
        telemetry.addData("raw left_stick_y", gamepad1.left_stick_y);
        telemetry.addData("raw right_stick_x", gamepad1.right_stick_x);
        telemetry.addData("loop runtime (s)", getRuntime());
        telemetry.update();
    }
}