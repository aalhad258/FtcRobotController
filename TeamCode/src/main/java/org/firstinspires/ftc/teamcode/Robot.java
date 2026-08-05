package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechanism.Drive;

// NOTE (same pattern as AprilTagLimelight.java): Robot is a plain POJO/container class, not an
// OpMode, so it has no built-in `telemetry` field from the FTC SDK. To add telemetry here without
// changing any existing method signatures, fields, or logic, an OPTIONAL Telemetry reference is
// injected via a new setTelemetry() method below. If never set, all telemetry calls in this file
// are no-ops (guarded by a null check) and behavior is identical to the original file.
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Robot
 * -----
 * ARCHITECTURE / PURPOSE:
 *   This is the team's central "robot container" class -- the single top-level object that owns
 *   and wires together every subsystem/mechanism (currently just `drive`, but this is the natural
 *   place future subsystems -- e.g. an arm, intake, lift, or the AprilTagLimelight vision helper --
 *   would be added as additional public fields). The intent is that every OpMode (TeleOp and
 *   Autonomous alike) instantiates ONE Robot object and calls Robot.init(hardwareMap) once from
 *   OpMode.init()/LinearOpMode's setup, rather than each OpMode separately constructing and
 *   initializing each mechanism by hand. This avoids duplicated hardware-map wiring code across
 *   OpModes and gives a single, predictable place to look when a mechanism fails to initialize.
 *
 *   This pattern mirrors the common FTC community "RobotHardware"/"Robot container" convention
 *   seen throughout FTC SDK samples and team codebases (distinct from, but complementary to,
 *   RoadRunner's/PedroPathing's own drivetrain-class conventions -- see the note in Drive.java,
 *   if/when that file is added, for how this project's mecanum drivetrain and any path-following
 *   library integrate with this Robot container).
 *
 * CURRENT SUBSYSTEMS:
 *   - drive (Drive, from org.firstinspires.ftc.teamcode.mechanism) -- the robot's mecanum
 *     drivetrain wrapper. See Drive.java for motor wiring, orientation, and any
 *     PedroPathing/RoadRunner-related configuration details (not included in this pass).
 *
 * COMPETITION DEBUG TIP:
 *   If a match starts and NOTHING on the robot responds, the first thing to check is whether
 *   Robot.init(hardwareMap) was actually called from the active OpMode's init() -- an OpMode that
 *   forgets to call this will compile fine (drive still exists as an object) but every motor
 *   inside `drive` will remain unconfigured/at SDK defaults.
 */
public class Robot {
    // Eagerly constructed at field-declaration time (before init() ever runs), so `drive` is
    // never null -- but note it is NOT yet wired to real hardware until init(hwMap) below runs.
    // Calling any hardware-touching method on `drive` before Robot.init() has been called will
    // fail, since the underlying DcMotor references inside Drive won't be set yet.
    public Drive drive = new Drive();

    // OPTIONAL telemetry sink for competition debugging (ADDED -- not part of original logic).
    // Set this once from your OpMode via robot.setTelemetry(telemetry) if you want Robot-level
    // init status reported to the Driver Station / FTC Dashboard. Safe to leave unset.
    private Telemetry telemetry;

    /**
     * OPTIONAL telemetry injection point (ADDED). Call once from OpMode.init() before/after
     * Robot.init(hardwareMap) if you want this class (and, in the future, any subsystem it owns)
     * to report status here. No effect on existing behavior if never called.
     */
    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    /**
     * One-time hardware setup for every subsystem this Robot owns. Call exactly once from the
     * active OpMode's init() (TeleOp or Autonomous), passing that OpMode's hardwareMap.
     *
     * ARCHITECTURE NOTE: as more subsystems are added to this class (arm, intake, vision, etc.),
     * their own .init(hwMap) calls should be added here, in the same place, so there remains a
     * single "did everything get initialized" checkpoint for the whole robot.
     *
     * @param hwMap the OpMode's hardwareMap, giving access to every device configured on the
     *              Driver Station / Robot/Control Hub's active hardware configuration file.
     */
    public void init(HardwareMap hwMap) {
        // Delegates to Drive's own init(), which is responsible for looking up drive motors by
        // their hardware-config names and configuring them (direction, zero-power behavior, mecanum
        // wheel orientation, etc.). See Drive.java for those specifics.
        drive.init(hwMap);

        // ADDED: single top-level confirmation that Robot-level init completed, so a "nothing is
        // responding" situation at the start of a match can quickly be narrowed down to "Robot.init
        // never ran" vs. "Robot.init ran but a specific subsystem inside it failed silently."
        if (telemetry != null) {
            telemetry.addData("[Robot] init", "complete");
        }
    }
}