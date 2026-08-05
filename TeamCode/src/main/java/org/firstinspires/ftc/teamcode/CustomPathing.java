package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.mechanism.Drive;

/**
 * ============================================================================
 *  HAND-ROLLED SINGLE-POINT PID PATHING (NO PEDROPATHING)
 * ============================================================================
 *
 *  !!! ARCHITECTURE NOTE, IMPORTANT FOR DEBUGGING !!!
 *  Unlike every other autonomous OpMode in this codebase (CustomPathing's
 *  siblings LimeLightBasic.java and SampleAutoPathing.java), THIS FILE DOES
 *  NOT USE THE PEDROPATHING Follower/PathChain/PathBuilder stack at all. It
 *  talks to the GoBilda Pinpoint odometry computer DIRECTLY
 *  (GoBildaPinpointDriver) and drives the mecanum base DIRECTLY (the
 *  Drive mechanism class), using a small hand-written cascade of
 *  proportional-derivative (PD) controllers to turn-then-drive to a single
 *  hardcoded target pose. There is no curved-path support, no PIDF tuning
 *  file to consult (Constants.java is NOT used here), and no PedroPathing
 *  "holdEnd" behavior -- this is a from-scratch point-to-point mover.
 *
 *  If you're debugging this OpMode, do NOT look in Constants.java or expect
 *  PedroPathing tuning docs to apply -- all the tuning gains (kPX, kDX,
 *  kPY, kDY, kPH, kDH, and the tolerance values) are local to THIS file and
 *  must be tuned by hand/experimentally on this OpMode specifically.
 *
 *  BEHAVIOR: a simple 2-phase state machine --
 *    TURN -> rotate in place until facing endPose's target heading
 *    MOVE -> drive in a straight line (translating in the robot's OWN
 *            frame, re-computed every loop from live odometry, so it's not
 *            a truly straight-line path in the field frame if odometry
 *            noise/drift nudges the estimate around slightly) toward
 *            endPose's X/Y, with a light heading-hold correction mixed in
 *    DONE -> stop
 *
 *  Reference docs:
 *   - GoBilda Pinpoint odometry computer / GoBildaPinpointDriver API:
 *     https://www.gobilda.com/pinpoint-odometry-computer/
 *   - FTC Pose2D / AngleUnit / DistanceUnit (org.firstinspires.ftc...):
 *     standard FTC SDK navigation types, see the FTC SDK docs at
 *     https://ftc-docs.firstinspires.org/
 *
 *  COMPETITION DEBUGGING CHEAT SHEET:
 *    - Robot never leaves TURN -> heading PD gains (kPH/kDH) too weak, or
 *      toleranceH too tight, or the Pinpoint isn't reporting heading
 *      changes (check "pinpoint" hardware config name, same device used by
 *      the PedroPathing localizer in Constants.java).
 *    - Robot overshoots/oscillates while turning or moving -> kPH/kPX/kPY
 *      too high relative to kDH/kDX/kDY; lower P or raise D.
 *    - Robot drives in the wrong direction/strafes instead of driving
 *      forward (or vice versa) -> check the field-to-robot frame rotation
 *      math in the MOVE state below (errorX/errorY), and the sign flips fed
 *      into drive.setPower() at the bottom of MOVE.
 * ============================================================================
 */
@Autonomous
public class CustomPathing extends OpMode {

    private final Drive drive = new Drive();
    // Hardcoded start/target poses in field coordinates (inches, degrees).
    // Unlike LimeLightBasic/SampleAutoPathing (which use PedroPathing's
    // Pose class exclusively), this file mixes PedroPathing's Pose type
    // (unused here except as an import leftover) with the FTC SDK's own
    // Pose2D type for the actual pinpoint/pose math below.
    private final Pose2D startPose = new Pose2D(DistanceUnit.INCH,0,0, AngleUnit.DEGREES, 0);
    private final Pose2D endPose = new Pose2D(DistanceUnit.INCH,30,40, AngleUnit.DEGREES, 0);
    private Pose2D currentPose = new Pose2D(DistanceUnit.INCH,0,0, AngleUnit.DEGREES, 0);
    GoBildaPinpointDriver pinpoint;

    // ---- Heading (rotation) PD gains + state, used in the TURN state ----
    double kPX = 0.1;
    double errorX = 0;
    double lastErrorX = 0;
    double toleranceX = 0.5;
    double kDX = 0.001;
    double curTime = 0;
    double lastTimeX = 0;
    double lastTimeY = 0;
    double lastTimeH = 0;
    double strafe;
    double kPY = 0.1;
    double errorY = 0;
    double lastErrorY = 0;
    double toleranceY = 0.5;
    double kDY = 0.001;
    double forward;
    double kPH = 0.01;
    double errorH = 0;
    double lastErrorH = 0;
    double toleranceH= 2;
    double kDH = 0.001;
    double rotate;
    private enum State {
        TURN,
        MOVE,
        DONE
    }

    private State state;

    @Override
    public void init() {
        // Manual Pinpoint setup -- this duplicates (must be kept consistent
        // with, but is NOT wired to) the PinpointConstants block in
        // Constants.java used by the PedroPathing-based OpModes. If the
        // physical pod offsets/encoder directions/pod type are ever changed
        // for tuning purposes over there, remember this file has its own
        // independent copy of the same setup here that also needs updating.
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(2, -6.5, DistanceUnit.INCH);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.setPosition(startPose);
        drive.init(hardwareMap);
        // If start and end headings already match, skip the TURN phase
        // entirely and go straight to translating.
        if (startPose.getHeading(AngleUnit.DEGREES) != endPose.getHeading(AngleUnit.DEGREES)) {
            state = State.TURN;
        } else {
            state = State.MOVE;
        }

        // --- ADDED TELEMETRY: confirm init completed and show the plan ---
        telemetry.addLine("CustomPathing: init complete (standalone Pinpoint PID pathing, no PedroPathing)");
        telemetry.addData("startPose (x,y,heading deg)", "%.2f, %.2f, %.2f",
                startPose.getX(DistanceUnit.INCH), startPose.getY(DistanceUnit.INCH), startPose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("endPose (x,y,heading deg)", "%.2f, %.2f, %.2f",
                endPose.getX(DistanceUnit.INCH), endPose.getY(DistanceUnit.INCH), endPose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("initial state", state);
        telemetry.update();
    }

    public void start() {
        resetRuntime();
        curTime = getRuntime();
    }

    @Override
    public void loop() {
        if (state == State.TURN) {
            // Pull the latest pose estimate from the Pinpoint. Unlike
            // PedroPathing's Follower.update(), which internally refreshes
            // localization AND drives path-following math, pinpoint.update()
            // ONLY refreshes the raw pose reading -- all control logic below
            // is hand-written in this OpMode.
            pinpoint.update();
            currentPose = pinpoint.getPosition();
            curTime = getRuntime();
            // Signed shortest-path heading error, normalized to (-180, 180]
            // so the robot always turns the short way around rather than
            // potentially spinning almost a full 360 degrees the long way.
            errorH = AngleUnit.normalizeDegrees(endPose.getHeading(AngleUnit.DEGREES)-currentPose.getHeading(AngleUnit.DEGREES));
            if (Math.abs(errorH) > toleranceH) {
                double pTerm = errorH * kPH;
                double dT = curTime - lastTimeH;
                double dTerm;
                if (dT != 0) {
                    // Standard discrete derivative term: rate of change of
                    // error over the time since the last loop. Guarded
                    // against dT == 0 (first loop, or a suspiciously-fast
                    // loop) to avoid a divide-by-zero producing NaN/Infinity
                    // power commands.
                    dTerm = ((errorH - lastErrorH) / dT) * kDH;
                } else {
                    dTerm = 0;
                }
                rotate = Range.clip(pTerm + dTerm, -0.4, 0.4);
                // Only the yaw (3rd) argument is nonzero here -- pure
                // in-place rotation, no translation, during TURN.
                drive.setPower(0, 0, -rotate, 1.0);
            } else {
                // Heading is within tolerance: stop rotating and hand off to
                // MOVE. Reset the X/Y PD state (errors + timestamps) here so
                // MOVE's derivative terms don't compute a bogus dT/dError
                // using stale timestamps from before TURN even started.
                drive.setPower(0, 0, 0, 1.0);
                state = State.MOVE;
                lastErrorX = 0;
                lastErrorY = 0;
                lastTimeX = getRuntime();
                lastTimeY = getRuntime();
                // --- ADDED TELEMETRY: mark the TURN->MOVE handoff explicitly ---
                telemetry.addLine("TURN complete: heading within tolerance, switching to MOVE");
            }
            lastTimeH = curTime;
            lastErrorH = errorH;
        } else if (state == State.MOVE) {
            pinpoint.update();
            currentPose = pinpoint.getPosition();
            curTime = getRuntime();
            // Field-frame position error (target minus current), in inches.
            double fieldErrorX = endPose.getX(DistanceUnit.INCH) - currentPose.getX(DistanceUnit.INCH);
            double fieldErrorY = endPose.getY(DistanceUnit.INCH) - currentPose.getY(DistanceUnit.INCH);
            double headingRad = Math.toRadians(currentPose.getHeading(AngleUnit.DEGREES));
            double cosH = Math.cos(headingRad);
            double sinH = Math.sin(headingRad);
            // Rotate the field-frame error vector by -currentHeading to get
            // it in the ROBOT'S frame (this is the standard 2D rotation
            // matrix for "field frame -> robot frame"). errorX ends up being
            // the component of the remaining distance along the robot's
            // current forward-facing direction; errorY is the component
            // perpendicular to it (i.e. how far to strafe). This is what
            // lets the PD loops below command forward/strafe power directly
            // in terms the Drive mechanism understands, even as the robot's
            // heading continues to be corrected simultaneously.
            errorX =  fieldErrorX * cosH + fieldErrorY * sinH;
            errorY = -fieldErrorX * sinH + fieldErrorY * cosH;

            // Heading correction continues to run WHILE translating (not
            // just during the TURN phase) -- this is a P-ONLY controller
            // here (no derivative term applied, unlike the PD used in TURN),
            // and uses a tighter power clamp (+/-0.3 vs +/-0.4) since it's
            // just a light "keep facing the target heading" trim rather than
            // the primary motion during this phase.
            errorH = AngleUnit.normalizeDegrees(endPose.getHeading(AngleUnit.DEGREES) - currentPose.getHeading(AngleUnit.DEGREES));
            double pTermH = errorH * kPH;
            rotate = Range.clip(pTermH, -0.3, 0.3);

            if (Math.abs(errorY) > toleranceY) {
                double pTerm = errorY * kPY;
                double dT = curTime - lastTimeY;
                double dTerm;
                if (dT != 0) {
                    dTerm = ((errorY - lastErrorY) / dT) * kDY;
                } else {
                    dTerm = 0;
                }
                forward = Range.clip(pTerm + dTerm, -0.4, 0.4);
            } else {
                forward = 0;
            }

            if (Math.abs(errorX) > toleranceX) {
                double pTerm = errorX * kPX;
                double dT = curTime - lastTimeX;
                double dTerm;
                if (dT != 0) {
                    dTerm = ((errorX - lastErrorX) / dT) * kDX;
                } else {
                    dTerm = 0;
                }
                strafe = Range.clip(pTerm + dTerm, -0.4, 0.4);
            } else {
                strafe = 0;
            }

            // Completion check is TRANSLATION-ONLY (errorX/errorY within
            // tolerance) -- it does NOT also require errorH to be within
            // toleranceH. In practice the heading P-trim above generally
            // keeps heading close throughout MOVE, but if you ever see the
            // robot finish MOVE pointed noticeably off the target heading,
            // this is why: DONE can be reached before heading fully settles.
            if (Math.abs(errorX) <= toleranceX &&
                    Math.abs(errorY) <= toleranceY) {
                state = State.DONE;
                // --- ADDED TELEMETRY: mark the MOVE->DONE handoff explicitly ---
                telemetry.addLine("MOVE complete: within X/Y tolerance, switching to DONE");
            }

            // NOTE the sign flips here: forward is passed in NEGATED, and
            // rotate is passed in NEGATED, while strafe is passed straight
            // through. This reconciles this file's error-sign convention
            // (derived from the Pinpoint heading/field-frame rotation math
            // above) with Drive.setPower(axial, lateral, yaw, speed)'s own
            // internal sign convention (see Drive.java). If you ever see the
            // robot drive backward when it should drive forward, or spin the
            // wrong way relative to errorH, this line is the first place to
            // check -- do not "fix" it in isolation without re-deriving
            // both this rotation math AND Drive.setPower()'s convention
            // together, or you'll just move the sign bug somewhere else.
            drive.setPower(strafe, -forward, -rotate, 1.0);
            lastErrorY = errorY;
            lastErrorX = errorX;
            lastTimeY = curTime;
            lastTimeX = curTime;
        } else if (state == State.DONE) {
            drive.setPower(0, 0, 0, 1.0);
        }
        telemetry.addData("State", state);
        telemetry.addData("X", currentPose.getX(DistanceUnit.INCH));
        telemetry.addData("Y", currentPose.getY(DistanceUnit.INCH));
        telemetry.addData("Heading", currentPose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Error X", errorX);
        telemetry.addData("Error Y", errorY);
        telemetry.addData("Error H", errorH);
        // --- ADDED TELEMETRY: the actual PD outputs being sent to the drivetrain
        // (previously not shown -- errors were visible but not the resulting
        // forward/strafe/rotate commands, which makes it hard to tell "is the PID
        // computing a reasonable command that the drivetrain isn't executing" apart
        // from "is the PID itself outputting something wrong") ---
        telemetry.addData("Commanded forward", forward);
        telemetry.addData("Commanded strafe", strafe);
        telemetry.addData("Commanded rotate", rotate);
        telemetry.addData("loop runtime (s)", curTime);
        telemetry.update();
    }
}