package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.mechanism.AprilTagLimelight;

/**
 * ============================================================================
 *  BENCH TEST: RAW APRILTAG DETECTION VERIFICATION
 * ============================================================================
 *
 *  Minimal OpMode whose only job is to prove the Limelight + AprilTag
 *  pipeline (pipeline index 8, same as AprilTagAutoAlignment.java) is
 *  wired up and working: is tag ID 11 visible, and if so, what does the
 *  Limelight report for it (tx/ty/ta and the MegaTag2 field-pose solve).
 *  Run this BEFORE trusting AprilTagAutoAlignment.java at competition --
 *  if tag detection is flaky here (on a static bench test with no PID or
 *  drivetrain involved), it will also be flaky there, and this file is the
 *  simpler place to diagnose why (lighting, camera focus, tag ID mismatch,
 *  wrong pipeline, etc.) without also juggling PID tuning at the same time.
 *
 *  NOTE ON REDUNDANT LIMELIGHT QUERIES: this OpMode (and AprilTagLimelight
 *  itself) calls getResult() / getLatestResult() several times per loop --
 *  once directly below, and again internally inside each of hasTag(),
 *  hasValidResult(), getRobotPose(), getTx(), getTy(), and getTa()  (see
 *  AprilTagLimelight.java, where every one of those methods independently
 *  re-fetches the latest result rather than sharing one cached value). This
 *  is not incorrect -- Limelight results don't change mid-loop-iteration in
 *  practice -- but it is worth knowing about if we're ever chasing a subtle
 *  timing inconsistency or trying to reduce loop-time overhead.
 *
 *  Reference docs:
 *   - Limelight3A AprilTag / MegaTag2 pose estimation (getBotpose_MT2):
 *     https://docs.limelightvision.io/docs/docs-limelight/getting-started/ftc
 * ============================================================================
 */
@Autonomous
public class AprilTagLimelightTest extends OpMode {

    private AprilTagLimelight limelight = new AprilTagLimelight();;

    @Override
    public void init() {
        // Pipeline 8 = AprilTag detection (see AprilTagAutoAlignment.java,
        // which uses the same pipeline index for its live auto-align logic).
        limelight.init(hardwareMap, 8);

        // --- ADDED TELEMETRY: confirm init completed ---
        telemetry.addLine("AprilTagLimelightTest: init complete, pipeline 8 (AprilTag)");
        telemetry.update();
    }

    @Override
    public void start() {
        // Limelight3A requires an explicit start() call to begin actively
        // streaming/processing pipeline results.
        limelight.start();

        // --- ADDED TELEMETRY: confirm start() ran (helps rule out "Limelight never
        // started streaming" as a cause if tag detection appears completely dead) ---
        telemetry.addLine("AprilTagLimelightTest: limelight.start() called");
        telemetry.update();
    }

    @Override
    public void loop() {
        limelight.update();
        LLResult result = limelight.getResult();
        // --- ADDED TELEMETRY: always show whether ANY result/tag is visible, every
        // loop, regardless of which branch below executes. Previously, if tag 11 was
        // never seen, NOTHING would print here at all (the two nested `if`s below only
        // ever add telemetry on the success path), leaving a blank/stale driver station
        // screen with no indication of WHY -- e.g. "no result at all" vs. "result seen,
        // but not tag 11" vs. "tag 11 seen, but not a valid/stable result". ---
        telemetry.addData("LL result present?", result != null);
        telemetry.addData("LL result valid?", result != null && result.isValid());
        telemetry.addData("tag 11 visible?", limelight.hasTag(11));

        if (limelight.hasTag(11)) {
            if (limelight.hasValidResult()) {
                Pose3D botpose = limelight.getRobotPose();
                telemetry.addData("Tx", limelight.getTx());
                telemetry.addData("Ty", limelight.getTy());
                telemetry.addData("Ta", limelight.getTa());
                telemetry.addData("BotPose", botpose.toString());
                telemetry.addData("Yaw", botpose.getOrientation().getYaw());
            }
        }
        // --- ADDED TELEMETRY: flush every loop so the driver station display actually
        // updates continuously (originally this OpMode never called telemetry.update()
        // in loop(), which can leave the screen showing stale data from a much earlier
        // loop depending on FTC SDK telemetry defaults) ---
        telemetry.update();
    }
}