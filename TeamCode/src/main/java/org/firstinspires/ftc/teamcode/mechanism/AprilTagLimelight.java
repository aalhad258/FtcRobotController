package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

// NOTE: Telemetry is intentionally NOT imported/used as a hard dependency of this class's
// constructor because this class is a plain "mechanism" helper, not an OpMode. OpMode/LinearOpMode
// are the only classes FTC's SDK gives a live `telemetry` object to. To satisfy the request for
// extra telemetry without changing any existing method signatures or control flow, an OPTIONAL
// telemetry reference is injected via a new setter (see setTelemetry() below). If it is never set,
// every telemetry call in this file is a no-op (guarded by a null check) and behavior is 100%
// identical to the original file.
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * AprilTagLimelight
 * ------------------
 * Thin wrapper around a Limelight 3A smart camera (running in AprilTag/fiducial pipeline mode)
 * fused with the Control/Expansion Hub's built-in IMU.
 *
 * WHY THIS CLASS EXISTS (architecture note):
 *   The Limelight 3A can compute a robot field pose from AprilTags using "MegaTag2" (MT2), but
 *   MT2's accuracy depends on the Limelight knowing the robot's current heading (yaw) ahead of
 *   time -- it uses gyro-assisted disambiguation to reject bad multi-tag solves. That's why every
 *   update() call here pushes the IMU's yaw INTO the Limelight before pulling a pose back OUT.
 *   If you ever see garbage/jumpy pose estimates from getRobotPose(), the #1 suspect is that
 *   update() is not being called every loop (stale yaw) or the IMU hub orientation below no
 *   longer matches how the Control Hub is physically mounted on the robot.
 *
 * DATA FLOW (for competition debugging, read top to bottom):
 *   1. init()   -> configures the Limelight pipeline + IMU axis mapping (one-time, in OpMode.init())
 *   2. start()  -> tells the Limelight to begin streaming/processing frames (call once, after init)
 *   3. update() -> MUST be called every loop() iteration BEFORE reading any pose/tag data below.
 *                  Feeds current robot yaw to the Limelight for MegaTag2 fusion.
 *   4. getResult() / getRobotPose() / hasTag() / getTx() / getTy() / getTa() -> read-only queries
 *      against the most recent frame the Limelight has processed.
 *
 * REFERENCES:
 *   - FTC SDK Limelight3A javadoc / FIRST Tech Challenge SDK samples (SensorLimelight3A.java)
 *   - Limelight official docs: https://docs.limelightvision.io/  (see "MegaTag2 / robot
 *     localization" and "Java quick start for FTC")
 *   - FTC SDK IMU / RevHubOrientationOnRobot docs (control-hub-mounting orientation):
 *     https://ftc-docs.firstinspires.org/ (search "IMU RevHubOrientationOnRobot")
 */
public class AprilTagLimelight {
    // The physical Limelight 3A camera, accessed via the FTC hardware map using the config name
    // "limelight" (must match exactly what's typed in the Robot Controller's hardware config).
    private Limelight3A limelight;

    // The Control/Expansion Hub's onboard IMU (BHI260/BNO055 depending on hub revision), accessed
    // via hardware map name "imu". Used purely to supply yaw to the Limelight's MegaTag2 solver --
    // this class does NOT use the IMU for drivetrain heading control elsewhere.
    private IMU imu;

    // OPTIONAL telemetry sink for competition debugging. Null by default so this class remains
    // usable exactly as before if no one wires it up. Set this from your OpMode via
    // aprilTagLimelight.setTelemetry(telemetry) if you want live dashboard/DS telemetry from here.
    private Telemetry telemetry;

    /**
     * OPTIONAL telemetry injection point (ADDED — not part of original file's logic).
     * Call this once from your OpMode's init() if you want this class to report its own
     * status/values to the Driver Station / FTC Dashboard. Safe to skip entirely.
     */
    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    /**
     * One-time hardware setup. Call from OpMode.init() (or LinearOpMode before waitForStart()).
     *
     * @param hwMap    the OpMode's hardwareMap (gives access to devices configured on the DS/RC)
     * @param pipeline which Limelight pipeline slot (0-9) to switch to. Pipelines are configured
     *                 on the Limelight's own web UI (typically http://limelight.local:5801) and
     *                 each pipeline stores its own AprilTag family/exposure/crop settings. If tags
     *                 aren't being detected, first check this pipeline index actually matches the
     *                 AprilTag pipeline you configured on the Limelight web UI, not a color-detect
     *                 pipeline.
     */
    public void init(HardwareMap hwMap, int pipeline) {
        // Pull the Limelight3A device using its hardware config name. Throws
        // IllegalArgumentException at init if "limelight" isn't in the active hardware config --
        // that's usually the fastest way to catch a typo'd config name during pit troubleshooting.
        limelight = hwMap.get(Limelight3A.class, "limelight");

        // Select which onboard pipeline (vision profile) the Limelight should run. Must match a
        // pipeline you've already built/tuned in the Limelight web UI for AprilTag detection.
        limelight.pipelineSwitch(pipeline);

        // Pull the Control Hub's IMU using its hardware config name "imu" (default name FTC's
        // config tool assigns to the internal IMU on most configs).
        imu = hwMap.get(IMU.class, "imu");

        // DESIGN DECISION: RevHubOrientationOnRobot describes how the Control/Expansion Hub is
        // physically bolted to the robot chassis, so the IMU's internal axes can be remapped to
        // the robot's real-world forward/left/up axes. This MUST match the hub's actual mounting:
        //   - LogoFacingDirection.UP     -> the REV logo printed on the hub's top face points UP
        //   - UsbFacingDirection.FORWARD -> the hub's USB ports point toward the robot's FRONT
        // If the hub is mounted differently on this robot (flipped, sideways, logo facing a side
        // panel, etc.), yaw output will be wrong and MegaTag2 pose fusion above will silently
        // degrade. This is a common competition failure point -- if field-oriented behavior or
        // Limelight pose looks rotated/mirrored, check this orientation FIRST against the hub's
        // actual physical mounting on the current chassis revision.
        // Reference: FTC SDK docs / examples for IMU RevHubOrientationOnRobot.
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);

        // Apply the orientation mapping to the IMU. From this point on, imu.getRobotYawPitchRollAngles()
        // returns angles already corrected for how the hub sits on the robot.
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        // ADDED: confirm setup completed and record which pipeline was selected, so a mid-match
        // "why is vision not working" question can be answered by glancing at telemetry/DS logs
        // instead of re-reading code.
        if (telemetry != null) {
            telemetry.addData("[AprilTagLimelight] init", "complete");
            telemetry.addData("[AprilTagLimelight] pipeline index", pipeline);
        }
    }

    /**
     * Begin active frame processing on the Limelight. Call once, typically right after init()
     * completes (or at the start of the match) -- NOT every loop. Calling start() repeatedly is
     * unnecessary but generally harmless; still, only call it once per OpMode run for clarity.
     */
    public void start() {
        limelight.start();

        // ADDED: cheap breadcrumb -- if a match starts and pose data never appears, telemetry
        // showing "started = true" tells you the problem is downstream (tags not visible, wrong
        // pipeline, bad lighting) rather than the Limelight never having been started at all.
        if (telemetry != null) {
            telemetry.addData("[AprilTagLimelight] started", true);
        }
    }

    /**
     * Must be called every loop() iteration BEFORE reading pose/tag data. Feeds the Limelight the
     * robot's current yaw (from the Control Hub IMU) so its MegaTag2 solver can fuse gyro heading
     * with AprilTag geometry for a more stable/accurate field pose than vision-only solving.
     *
     * COMPETITION DEBUG TIP: if getRobotPose() ever looks "stuck" or wildly incorrect, first check
     * that update() is actually being called every loop -- a missed/late call here means the
     * Limelight is fusing against stale yaw data.
     */
    public void update() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        // Push only yaw (not pitch/roll) into the Limelight -- MegaTag2 on a Limelight 3A expects
        // robot heading about the vertical (Z) axis for its multi-tag solve; pitch/roll aren't
        // part of this API's expected input for a mecanum/ground robot.
        limelight.updateRobotOrientation(orientation.getYaw());

        // ADDED: surface the yaw actually being sent to the Limelight. If field-relative driving
        // or pose estimates seem rotated, comparing this value against your drivetrain's own yaw
        // reading (if it uses a separate IMU reference) helps isolate whether the bug is in this
        // class's orientation config vs. elsewhere in the codebase.
        if (telemetry != null) {
            telemetry.addData("[AprilTagLimelight] yaw sent to LL (deg)", orientation.getYaw());
        }
    }

    /**
     * @return the most recent frame result from the Limelight (raw SDK object), or a result for
     *         which isValid() may be false if no fresh/valid detection exists. Callers should
     *         always null-check AND isValid()-check before trusting fields on this object -- see
     *         every other method in this class for that exact pattern.
     */
    public LLResult getResult() {
        return limelight.getLatestResult();
    }

    /**
     * @return the robot's field pose (as computed by Limelight's MegaTag2 algorithm, which fuses
     *         AprilTag geometry with the yaw pushed in via update()), or null if there is no
     *         result yet or the current result isn't valid (e.g., no tags visible this frame).
     *         CALLERS MUST NULL-CHECK THE RETURN VALUE.
     */
    public Pose3D getRobotPose() {
        LLResult result = getResult();
        if (result == null || !result.isValid()) {
            // ADDED: makes "why is my pose null" immediately visible on the DS instead of a
            // silent null the caller has to chase through a debugger mid-match.
            if (telemetry != null) {
                telemetry.addData("[AprilTagLimelight] getRobotPose", "no valid result this frame");
            }
            return null;
        }

        // getBotpose_MT2() specifically requests the MegaTag2 (gyro-assisted) solve rather than
        // the plain single-tag/multi-tag vision-only botpose -- this is why update() feeding yaw
        // every loop matters so much for this method's accuracy.
        Pose3D pose = result.getBotpose_MT2();

        // ADDED: raw pose telemetry for quick sanity-checking against known field landmarks during
        // pit testing or a match timeout.
        if (telemetry != null && pose != null) {
            telemetry.addData("[AprilTagLimelight] MT2 pose (x,y,z)",
                    "%.2f, %.2f, %.2f", pose.getPosition().x, pose.getPosition().y, pose.getPosition().z);
        }

        return pose;
    }

    /**
     * @param tagId the AprilTag fiducial ID to look for in the current frame (per-season tag IDs
     *              are defined in that year's FTC game manual / into the Limelight's fiducial
     *              family config).
     * @return true if the given tag ID is present among this frame's detected fiducials.
     */
    public boolean hasTag(int tagId) {
        LLResult result = getResult();
        if (result == null || !result.isValid()) {
            return false;
        }
        for (LLResultTypes.FiducialResult tag :
                result.getFiducialResults()) {
            if (tag.getFiducialId() == tagId) {
                // ADDED: confirms exactly which tag ID triggered a true return, useful when
                // auto-alignment behaves unexpectedly (e.g., locking onto the wrong tag ID).
                if (telemetry != null) {
                    telemetry.addData("[AprilTagLimelight] hasTag match", tagId);
                }
                return true;
            }
        }
        return false;
    }

    /**
     * @return horizontal offset (degrees) of the primary detected target from the Limelight's
     *         crosshair, positive/negative sign per Limelight convention (see Limelight docs for
     *         which direction is positive on this camera's mount orientation). Returns 0 if there
     *         is no current result -- NOTE: 0 is also a legitimate "dead-center" reading, so this
     *         return value alone cannot distinguish "no target" from "perfectly centered target".
     *         Use hasValidResult() first if that distinction matters for your logic.
     */
    public double getTx() {
        LLResult result = getResult();
        return result != null ? result.getTx() : 0;
    }

    /**
     * @return vertical offset (degrees) of the primary detected target from the Limelight's
     *         crosshair. Same 0-vs-"no target" ambiguity caveat as getTx() applies here.
     */
    public double getTy() {
        LLResult result = getResult();
        return result != null ? result.getTy() : 0;
    }

    /**
     * @return target area (percentage of the image the primary target occupies, 0-100). Same
     *         0-vs-"no target" ambiguity caveat as getTx()/getTy() applies here.
     */
    public double getTa() {
        LLResult result = getResult();
        return result != null ? result.getTa() : 0;
    }

    /**
     * @return true if the latest Limelight frame has a non-null, valid result (i.e., it is safe to
     *         trust getTx()/getTy()/getTa()/getRobotPose() as "there IS a target" rather than the
     *         ambiguous zero-value case described above).
     */
    public boolean hasValidResult() {
        LLResult result = getResult();
        boolean valid = result != null && result.isValid();

        // ADDED: one-line "is vision currently usable" flag -- the single most useful telemetry
        // line to glance at during a match if auto-align/auto-score behavior seems off.
        if (telemetry != null) {
            telemetry.addData("[AprilTagLimelight] hasValidResult", valid);
        }
        return valid;
    }
}