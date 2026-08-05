package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * HelloWorld
 * ----------
 * ARCHITECTURE / PURPOSE:
 *   This is the team's very first FTC OpMode of the season ("First FTC code of the season!!!"
 *   per the original telemetry string) -- a minimal smoke-test OpMode used to confirm that:
 *     1. The Robot Controller app / code build / deploy pipeline works end-to-end
 *        (Android Studio -> Robot Controller phone/Control Hub -> Driver Station shows it in
 *        the opmode list).
 *     2. Telemetry actually reaches the Driver Station screen.
 *   It intentionally does NOT touch any hardware (no HardwareMap usage), so it's safe to run on
 *   a robot with no motors/sensors wired up yet, or even with no robot at all connected.
 *
 * WHY @TeleOp (not @Autonomous):
 *   TeleOp OpModes run only after the driver presses PLAY on the Driver Station (no autonomous
 *   timer, no pre-programmed motion) -- appropriate here since this file only prints static
 *   telemetry and never needs to run on a timer.
 *   Reference: FTC SDK docs on OpMode annotations (@TeleOp vs @Autonomous).
 */
@TeleOp
public class HelloWorld extends OpMode{

    @Override
    public void init() {
        // Original behavior (unchanged): pushes a single static telemetry line the moment
        // init() is pressed on the Driver Station, BEFORE the match/practice timer starts.
        // NOTE: addData("label", "value") here is being used with the greeting text AS the
        // "label" argument and "1" as the "value" argument -- i.e. it's really
        // telemetry.addData(<caption>, <data>), so it renders as "Hello World!!! ...: 1" on the
        // Driver Station rather than the caption being separate from a real "1" data point. This
        // is harmless for a smoke test but worth knowing if you ever go looking for where a
        // literal "1" appears on the DS screen.
        telemetry.addData("Hello World!!! First FTC code of the season!!!", "1");
    }

    @Override
    public void loop() {
        // Original behavior (unchanged): intentionally empty. This OpMode only needs to prove
        // that init() telemetry reaches the Driver Station; it does nothing once running.

        // ADDED telemetry: without this, once the driver presses PLAY the Driver Station
        // telemetry screen would go blank (since init()'s telemetry.addData() call is not
        // re-sent automatically, and loop() previously called nothing). This confirms the
        // OpMode is still alive and looping after PLAY is pressed, and gives a live loop
        // timer -- useful even in a smoke-test OpMode to confirm the deploy is truly running
        // and not frozen.
        telemetry.addData("status", "loop() running");
        telemetry.addData("loop time (s)", getRuntime());
    }
}