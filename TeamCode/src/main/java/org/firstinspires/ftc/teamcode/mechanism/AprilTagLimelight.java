package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

// NOTE: Telemetry is intentionally optional.
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * AprilTagLimelight
 * ------------------
 * Wrapper around a Limelight 3A for AprilTag detection.
 *
 * IMU / MegaTag2 is CURRENTLY DISABLED.
 *
 * This class currently uses the Limelight for:
 *   - AprilTag detection
 *   - tx
 *   - ty
 *   - ta
 *   - tag ID detection
 *
 * It does NOT require an IMU.
 *
 * MegaTag2 / getRobotPose() is temporarily disabled because MegaTag2
 * requires robot heading from an IMU.
 */
public class AprilTagLimelight {

    // Limelight hardware
    private Limelight3A limelight;

    // Optional telemetry
    private Telemetry telemetry;

    /**
     * Optional telemetry injection.
     */
    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    /**
     * Initialize the Limelight.
     *
     * @param hwMap hardware map
     * @param pipeline Limelight pipeline number
     */
    public void init(HardwareMap hwMap, int pipeline) {

        // Get Limelight from hardware configuration.
        // The configuration name must be exactly "limelight".
        limelight = hwMap.get(Limelight3A.class, "limelight");

        // Select the requested pipeline.
        limelight.pipelineSwitch(pipeline);

        if (telemetry != null) {
            telemetry.addData("[AprilTagLimelight] init", "complete");
            telemetry.addData("[AprilTagLimelight] pipeline index", pipeline);
            telemetry.addData("[AprilTagLimelight] IMU", "DISABLED");
        }
    }

    /**
     * Start Limelight processing.
     *
     * Call this once after init().
     */
    public void start() {

        limelight.start();

        if (telemetry != null) {
            telemetry.addData("[AprilTagLimelight] started", true);
        }
    }

    /**
     * Update Limelight.
     *
     * IMPORTANT:
     * There is intentionally NO IMU code here.
     *
     * We are currently only using normal Limelight AprilTag
     * detection such as tx/ty/ta.
     */
    public void update() {

        // No IMU.
        // No MegaTag2 orientation update required for tx/ty/ta.

        if (telemetry != null) {
            telemetry.addData("[AprilTagLimelight] update", "running without IMU");
        }
    }

    /**
     * Get the latest Limelight result.
     */
    public LLResult getResult() {
        return limelight.getLatestResult();
    }

    /**
     * Get robot pose from MegaTag2.
     *
     * TEMPORARILY DISABLED because MegaTag2 requires an IMU heading.
     *
     * Returns null for now.
     */
    public Pose3D getRobotPose() {

        if (telemetry != null) {
            telemetry.addData(
                    "[AprilTagLimelight] getRobotPose",
                    "disabled - IMU disabled"
            );
        }

        return null;
    }

    /**
     * Check whether a specific AprilTag ID is visible.
     *
     * @param tagId AprilTag ID
     * @return true if the tag is detected
     */
    public boolean hasTag(int tagId) {

        LLResult result = getResult();

        if (result == null || !result.isValid()) {
            return false;
        }

        for (LLResultTypes.FiducialResult tag :
                result.getFiducialResults()) {

            if (tag.getFiducialId() == tagId) {

                if (telemetry != null) {
                    telemetry.addData(
                            "[AprilTagLimelight] hasTag match",
                            tagId
                    );
                }

                return true;
            }
        }

        return false;
    }

    /**
     * Get horizontal AprilTag offset.
     *
     * tx = horizontal angle from Limelight crosshair.
     */
    public double getTx() {

        LLResult result = getResult();

        return result != null ? result.getTx() : 0;
    }

    /**
     * Get vertical AprilTag offset.
     *
     * ty = vertical angle from Limelight crosshair.
     */
    public double getTy() {

        LLResult result = getResult();

        return result != null ? result.getTy() : 0;
    }

    /**
     * Get AprilTag target area.
     */
    public double getTa() {

        LLResult result = getResult();

        return result != null ? result.getTa() : 0;
    }

    /**
     * Check whether the Limelight currently has a valid result.
     */
    public boolean hasValidResult() {

        LLResult result = getResult();

        boolean valid =
                result != null &&
                        result.isValid();

        if (telemetry != null) {
            telemetry.addData(
                    "[AprilTagLimelight] hasValidResult",
                    valid
            );
        }

        return valid;
    }
}