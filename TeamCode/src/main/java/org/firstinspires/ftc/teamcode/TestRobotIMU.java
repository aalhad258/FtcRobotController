package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechanism.RobotIMU;

/**
 * ============================================================================
 *  BENCH TEST: ROBOTIMU WRAPPER -- CONTAINS SEVERAL LIFECYCLE ISSUES, READ CAREFULLY
 * ============================================================================
 *
 *  Intended purpose: bench-test RobotIMU.java (the standalone hub-IMU
 *  wrapper) by printing its live yaw reading to telemetry. HOWEVER, as
 *  written, this file has several structural issues that will make it
 *  behave very differently from what its name suggests. Per your
 *  instructions NONE of the logic below has been changed -- everything in
 *  this block is flagging existing behavior, not describing a fix that's
 *  been applied.
 *
 *  !!! ISSUE 1: MISSING @TeleOp / @Autonomous ANNOTATION !!!
 *  This class has no @TeleOp or @Autonomous annotation. Every other OpMode
 *  in this codebase has one of these two. Without an annotation, this
 *  OpMode WILL NOT APPEAR in the Driver Station's OpMode selection list at
 *  all -- it cannot currently be run on a robot through the normal FTC
 *  Driver Station app flow. If you're trying to run this and can't find it
 *  on the list, this is why.
 *
 *  !!! ISSUE 2: Imu1.init(hardwareMap) IS CALLED INSIDE loop(), NOT init() !!!
 *  Every other mechanism class in this codebase (RobotIMU included, see its
 *  own init() method) is designed to be initialized ONCE, in the OpMode's
 *  init() method, before the match/test starts. Here, `Imu1.init(hardwareMap)`
 *  is instead called on EVERY SINGLE LOOP ITERATION, inside loop() -- meaning
 *  the hub IMU gets fully re-initialized (including re-applying its mounting
 *  orientation configuration) dozens of times per second for as long as this
 *  OpMode runs, instead of once at the start. This is wasteful at minimum,
 *  and depending on the underlying IMU driver's behavior when
 *  re-initialized rapidly and repeatedly, could also cause the yaw reading
 *  to behave unexpectedly (e.g. resetting or glitching) rather than
 *  smoothly tracking rotation over time the way a bench test would
 *  normally expect.
 *
 *  !!! ISSUE 3: start() READS `Yaw` BEFORE loop() HAS EVER RUN !!!
 *  The FTC OpMode lifecycle calls init() once, then (once the driver
 *  presses PLAY) calls start() once, and only THEN begins repeatedly
 *  calling loop(). The `Yaw` field is only ever assigned a value inside
 *  loop() -- it is never set in init() or start() itself. That means when
 *  start() runs and reads `telemetry.addData("Yaw =", Yaw)`, `Yaw` still
 *  holds its default initial value (0.0, since it's a primitive double
 *  field with no initializer), NOT a real IMU reading -- loop() simply
 *  hasn't executed yet at that point. So the ONLY telemetry line this
 *  OpMode's start() method produces will always read "Yaw = 0.0"
 *  regardless of the robot's actual orientation. Also note start() never
 *  calls telemetry.update(), so depending on FTC SDK telemetry defaults
 *  this line may not even reliably reach the Driver Station screen at all.
 *  The added telemetry inside loop() below (which DOES call update() every
 *  iteration) is where you should actually look for a live-updating yaw
 *  reading with this file as currently structured.
 * ============================================================================
 */
public class TestRobotIMU extends OpMode{
    RobotIMU Imu1 = new RobotIMU();

    private double Yaw;
    @Override
    public void init() {
        // Intentionally empty in the original source -- see Issue 2 above:
        // RobotIMU initialization happens in loop() instead of here.

        // --- ADDED TELEMETRY: make the empty init() visible, since a silent init()
        // can otherwise look identical to a hang/crash from the driver station ---
        telemetry.addLine("TestRobotIMU: init() ran (note: IMU is NOT initialized here -- see Issue 2 in the file header comment; it's (re-)initialized every loop() call instead)");
        telemetry.update();
    }

    @Override
    public void loop() {
        Imu1.init(hardwareMap);
        Yaw = Imu1.getHeading(AngleUnit.DEGREES);

        // --- ADDED TELEMETRY: live yaw reading, updated every loop -- this is the
        // actually-reliable place to read this OpMode's IMU output, since (per Issue 3
        // above) start()'s telemetry line will always show a stale 0.0 ---
        telemetry.addData("Yaw (deg, live, loop())", Yaw);
        telemetry.addData("loop runtime (s)", getRuntime());
        telemetry.update();
    }

    @Override
    public void start() {
        // See Issue 3 in the file header comment: `Yaw` has never been
        // assigned by the time this runs, so this will always print the
        // field's default value (0.0), not a real IMU reading.
        telemetry.addData("Yaw =", Yaw);

        // --- ADDED TELEMETRY: flush this line so it at least reliably reaches the
        // driver station (previously no update() call here), plus an explicit note
        // so it's clear this specific value is expected to be stale/0.0 ---
        telemetry.addLine("(^ NOTE: this Yaw value is from before loop() has ever run -- expect 0.0. See live reading in loop() telemetry instead.)");
        telemetry.update();
    }
}