package org.firstinspires.ftc.teamcode.mechanism;

/**
 * ============================================================================
 *  BLOB: A SINGLE RECORDED VISION DETECTION
 * ============================================================================
 *
 *  Plain data holder (no behavior) representing one detected object from
 *  the Limelight's color/blob-detection pipeline, as collected and
 *  clustered by LimeLightBasic.java. Despite the generic field names (tx,
 *  ty, ta -- matching the raw Limelight terminology), by the time a Blob is
 *  constructed in LimeLightBasic.java, `tx` has ALREADY been converted from
 *  a raw camera-relative reading into an ABSOLUTE FIELD-FRAME bearing (see
 *  LimeLightBasic.SAMPLE's `trueX` calculation) -- it is NOT the same as a
 *  raw Limelight `getTargetXDegrees()` value. Keep this in mind if this
 *  class is ever reused elsewhere: `tx` here means "field bearing in
 *  degrees", not "camera-relative horizontal offset".
 *
 *  Field meanings (as used by LimeLightBasic.java):
 *    tx - absolute field-frame bearing (degrees) to this detection, already
 *         combining the camera's raw horizontal offset with the robot's
 *         heading at the moment of detection.
 *    ty - raw camera-relative VERTICAL offset (degrees) to this detection,
 *         NOT converted to field frame -- used directly in
 *         LimeLightBasic's distance-estimation trig (see the DRIVE state
 *         and findBestCluster()), since vertical angle -> distance doesn't
 *         depend on robot heading the way horizontal bearing does.
 *    ta  - raw Limelight target area (a rough proxy for how large/close the
 *          detected object appears in the camera frame); stored here but
 *          not currently used in any of LimeLightBasic's math -- only
 *          surfaced in telemetry.
 * ============================================================================
 */
public class Blob {
    public double tx;
    public double ty;
    public double ta;

    /**
     * @param tx absolute field-frame bearing in degrees (see class-level
     *           note above -- NOT a raw camera-relative angle)
     * @param ty raw camera-relative vertical offset in degrees
     * @param ta raw Limelight target area
     */
    public Blob(double tx, double ty, double ta) {
        this.tx = tx;
        this.ty = ty;
        this.ta = ta;
    }
}