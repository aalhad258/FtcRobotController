package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.mechanism.AprilTagLimelight;
import org.firstinspires.ftc.teamcode.mechanism.Drive;

import java.awt.font.NumericShaper;

/**
 * ============================================================================
 *  DRIVER-CONTROLLED TELEOP WITH APRILTAG AUTO-ALIGN ASSIST
 * ============================================================================
 *
 *  Normal field-centric-free ("robot-centric") manual driving from the
 *  gamepad sticks, PLUS a hold-to-activate auto-rotation assist: while
 *  gamepad1's left trigger is held past a deadzone, and the Limelight can
 *  see AprilTag ID 11, a PD controller overrides the manual `rotate` input
 *  and instead spins the robot to center that tag horizontally in the
 *  camera's view (drives Limelight `tx`, the tag's horizontal offset in
 *  degrees, to ~0 / to `goalX`). Forward/strafe stick input is NOT
 *  overridden -- the driver keeps full manual translation control even
 *  while auto-align is rotating the robot for them.
 *
 *  Also includes a driver-facing LIVE PID TUNER: the D-pad nudges kP/kD by
 *  a selectable step size (cycled with the B button) without needing to
 *  redeploy code, so alignment gains can be dialed in on the practice field
 *  in real time. This mirrors the same PD-controller pattern used in
 *  CustomPathing.java's TURN state, but scoped to a single axis (rotation
 *  only) and driven by AprilTag error instead of odometry heading error.
 *
 *  Reference docs:
 *   - Limelight3A / AprilTag `tx` (target horizontal offset, degrees,
 *     positive = target right of camera center):
 *     https://docs.limelightvision.io/docs/docs-limelight/getting-started/ftc
 *   - FTC Gamepad button-edge helpers (bWasPressed(), dpadLeftWasPressed(),
 *     etc. -- these fire exactly once per physical press, unlike raw
 *     gamepad1.b/gamepad1.dpad_left which stay true for the whole time the
 *     button is held): see the FTC SDK Gamepad class docs,
 *     https://ftc-docs.firstinspires.org/
 *
 *  COMPETITION DEBUGGING CHEAT SHEET:
 *    - Auto-align never engages -> check left_trigger deadzone (0.3) is
 *      actually being exceeded, AND that tag 11 is the correct ID for the
 *      current game/target (hardcoded below), AND that the Limelight is on
 *      pipeline 8 (see AprilTagLimelight.init() call in init() below).
 *    - Auto-align engages but rotates the wrong way / never settles -> use
 *      the added telemetry below (goalX, tx, error, rotate) to see whether
 *      the SIGN of `error` matches the direction the robot actually needs
 *      to turn; if it's inverted, the fix belongs in how `error` is
 *      computed, not in flipping kP's sign band-aid style.
 *    - Manual driving feels sluggish/inverted -> that's independent of the
 *      align logic; check the raw stick-to-forward/strafe/rotate mapping at
 *      the top of loop(), not this PD block.
 * ============================================================================
 */
@TeleOp
public class AprilTagAutoAlignment extends OpMode {
    private final AprilTagLimelight atlimelight = new AprilTagLimelight();
    private final Drive drive = new Drive();

    // ----------- PD Controller --------------
    // Rotation-only PD controller that drives Limelight tx -> goalX. Tuned
    // live on the field via the D-pad (see loop() below) rather than by
    // redeploying code -- so the numbers here are just REASONABLE STARTING
    // POINTS, not the final competition values. Check telemetry at the last
    // practice session for the actual gains in use if these look stale.
    double kP = 0.002;
    double error = 0;
    double lastError = 0;
    double goalX = 0; //  ADD OFFSET HERE
    // ^ If the AprilTag target isn't meant to be dead-center in the camera
    // frame at the robot's desired final alignment (e.g. the camera is
    // mounted off to one side, or you want to align to a point offset from
    // the tag itself), set a nonzero goalX here so the controller centers on
    // "tx == goalX" instead of "tx == 0".
    double angleTolerance = 0.4; // MAY NEED TO TWEAK THIS
    double kD = 0.0001;
    double curTime = 0;
    double lastTime = 0;

    // ------------- Driving Setup --------------
    double forward, strafe, rotate;

    // ------------- Controller Based PD Tuning ---------------
    // Selectable step sizes for live-tuning kP/kD from the gamepad D-pad.
    // stepIndex starts at 2 -> stepSizes[2] == 0.0001, the FINEST
    // adjustment, so early nudges are small/safe by default; press B to
    // cycle to coarser steps (0.001, then 0.01) for faster large changes.
    double[] stepSizes = {0.01, 0.001, 0.0001};
    int stepIndex = 2;

    @Override
    public void init() {
        // Pipeline 8 = AprilTag detection pipeline (matches
        // AprilTagLimelightTest.java's usage of the same pipeline index;
        // contrast with LimeLightBasic.java, which uses pipeline 0 for
        // color/blob detection on the same physical camera hardware).
        atlimelight.init(hardwareMap, 8);
        drive.init(hardwareMap);

        telemetry.addLine("Initialized");
        // --- ADDED TELEMETRY: show the starting tuning values so a driver/coach
        // reviewing a match log can see what gains the match actually started with ---
        telemetry.addData("initial kP", kP);
        telemetry.addData("initial kD", kD);
        telemetry.addData("goalX", goalX);
        telemetry.addData("angleTolerance", angleTolerance);
        telemetry.update();
    }

    public void start() {
        resetRuntime();
        curTime = getRuntime();
    }

    @Override
    public void loop() {
        // ------------- Get Drive Inputs ------------
        // Standard FTC gamepad convention: left_stick_y is NEGATIVE when the
        // stick is pushed forward/away from the driver, so it's negated here to
        // make "push stick forward" correspond to positive forward power.
        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        // ------------- Get April Tag Info --------------
        // Feeds the IMU's current yaw into the Limelight so its (unused in
        // this OpMode, but computed internally) MegaTag2 field-pose solve has
        // an accurate robot orientation to work with. Even though this
        // OpMode only reads tx (not botpose) below, this update() call is
        // still required for the Limelight's internal AprilTag pipeline
        // state to stay current -- do not remove it even though botpose
        // isn't consumed here.
        atlimelight.update();

        // ------------- AutoAlign Rotation Logic -------------
        // Deadzone of 0.3 on the trigger so a barely-touched trigger doesn't
        // accidentally engage auto-align.
        if (gamepad1.left_trigger > 0.3) {
            if (atlimelight.hasTag(11) && atlimelight.hasValidResult()) {
                error = goalX - atlimelight.getTx();

                if (Math.abs(error) < angleTolerance) {
                    // Close enough: stop correcting so the robot doesn't
                    // dither/jitter right at the tolerance boundary.
                    rotate = 0;
                } else {
                    double pTerm = error * kP;
                    curTime = getRuntime();
                    double dT = curTime - lastTime;
                    // NOTE: unlike CustomPathing.java's PD loops, there is NO
                    // guard here against dT == 0 before dividing. In practice
                    // this is unlikely to hit exactly zero given real loop
                    // timing, but if you ever see a sudden power spike/NaN in
                    // the `rotate` telemetry right when auto-align first
                    // engages, this unguarded division is the first suspect.
                    double dTerm = ((error - lastError) / dT) * kD;

                    rotate = Range.clip(pTerm + dTerm, -0.4, 0.4);

                    lastError = error;
                    lastTime = curTime;
                }
            } else {
                // Trigger held but no valid tag 11 in view: hold rotate at 0
                // (don't spin blindly searching) and reset PD state so that
                // when a tag DOES reappear, the derivative term doesn't
                // compute a spurious jump from stale error/time values.
                lastTime = getRuntime();
                lastError = 0;
                rotate = 0;
            }
        } else {
            // Trigger not held: auto-align fully disengaged, manual rotate
            // input (already read above) passes through untouched. PD state
            // is still reset here so re-engaging later starts clean.
            lastTime = getRuntime();
            lastError = 0;
            rotate = 0;
        }

        // Drive our motors
        drive.setPower(forward, strafe, rotate, 1.0);

        // Update P and D on the fly
        // 'B' button cycles through the different step sizes for tuning precision
        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length; // Modulo wraps the index back to 0.
        }

        // D-pad left/right adjusts the p gain
        if (gamepad1.dpadLeftWasPressed()) {
            kP -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()) {
            kP += stepSizes[stepIndex];
        }

        // D-pad up/down adjusts the d gain
        if (gamepad1.dpadUpWasPressed()) {
            kD += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()) {
            kD -= stepSizes[stepIndex];
        }

        telemetry.addData("tx", atlimelight.getTx());
        telemetry.addData("error", error);
        telemetry.addData("rotate", rotate);
        telemetry.addData("kP", kP);
        telemetry.addData("kD", kD);
        telemetry.addData("step size", stepSizes[stepIndex]);
        // --- ADDED TELEMETRY: state that isn't otherwise visible but is exactly what
        // you'd want on the driver station screen mid-match to sanity check align
        // behavior at a glance ---
        telemetry.addData("left trigger", gamepad1.left_trigger);
        telemetry.addData("auto-align engaged?", gamepad1.left_trigger > 0.3);
        telemetry.addData("tag 11 visible & valid?", atlimelight.hasTag(11) && atlimelight.hasValidResult());
        telemetry.addData("goalX", goalX);
        telemetry.addData("forward (manual)", forward);
        telemetry.addData("strafe (manual)", strafe);
        telemetry.update();
    }
}