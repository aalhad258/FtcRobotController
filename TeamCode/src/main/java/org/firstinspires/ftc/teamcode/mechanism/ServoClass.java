package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * ============================================================================
 *  SERVO MECHANISM WRAPPER (ONE POSITIONAL SERVO + ONE CONTINUOUS-ROTATION SERVO)
 * ============================================================================
 *
 *  Thin wrapper around two independent servo-family devices:
 *    - servoPos: a standard POSITIONAL Servo (moves to and holds an
 *      absolute angle, 0.0-1.0 -> some physical angle range).
 *    - servoRot: a CRServo (continuous-rotation servo, behaves like a small
 *      motor -- driven by POWER, -1.0 to 1.0, not position).
 *  Used directly by ServoExamples.java to demo/bench-test both from the
 *  gamepad triggers.
 *
 *  !!! HIGH-PRIORITY COMPETITION-DEBUGGING FLAG !!!
 *  Look closely at the two hardwareMap device names used in init() below:
 *      servoPos = hwMap.get(Servo.class,   "serve_pos")   <- "serve_pos"
 *      servoRot = hwMap.get(CRServo.class, "servo_pos")   <- "servo_pos"
 *  These are NOT the same string -- "serve_pos" (servoPos) vs. "servo_pos"
 *  (servoRot), one letter apart. This is either (a) intentional and matches
 *  two distinctly-named devices in the robot's hardware configuration file
 *  on the Driver Station/Control Hub, or (b) a copy-paste typo. If this
 *  class ever throws an exception on init() (a "device not found" /
 *  IllegalArgumentException from hwMap.get()), or if ServoExamples.java's
 *  position servo doesn't move at all despite the CRServo working, THIS
 *  NAME MISMATCH IS THE FIRST THING TO CHECK against the actual Driver
 *  Station robot configuration -- confirm both "serve_pos" and "servo_pos"
 *  really do exist as two separately-named devices there, not just one.
 *  (Per your instructions this has not been changed -- flagging only.)
 * ============================================================================
 */
public class ServoClass {
    private Servo servoPos;
    private CRServo servoRot;

    /**
     * @param hwMap the OpMode's hardwareMap. See the class-level note above
     *              regarding the "serve_pos" vs "servo_pos" device names
     *              used here -- verify both exist in the robot configuration
     *              before assuming this init() will succeed unmodified.
     */
    public void init(HardwareMap hwMap) {
        servoPos = hwMap.get(Servo.class, "serve_pos");
        servoRot = hwMap.get(CRServo.class, "servo_pos");
        // Remaps the [0.0, 1.0] input range accepted by setServoPos() below
        // onto the PHYSICAL [0.5, 1.0] portion of the servo's raw travel --
        // i.e. restricts usable motion to the servo's midpoint-to-full-travel
        // half, rather than its entire 0.0-1.0 raw range. Combined with the
        // REVERSE direction below, this effectively remaps this mechanism's
        // 0.0-1.0 logical input onto some physical sub-range of the servo's
        // real angular travel (commonly used to protect a mechanism from
        // over-rotating past a safe limit, or to compensate for how the
        // servo horn/linkage is physically mounted).
        servoPos.scaleRange(0.5, 1.0); // set range from midpoint to 180
        servoPos.setDirection(Servo.Direction.REVERSE);
    }

    /**
     * Commands the positional servo to an absolute position.
     *
     * @param angle target position, 0.0 to 1.0, WITHIN the scaled/reversed
     *              range configured in init() above -- i.e. 0.0 here means
     *              "one end of the [0.5, 1.0] physical sub-range", not
     *              "the servo's true zero", because of scaleRange().
     */
    public void setServoPos(double angle) {
        servoPos.setPosition(angle);
    }

    /**
     * Drives the continuous-rotation servo at a given power, like a small
     * motor -- this is NOT a position command (a CRServo has no absolute
     * position feedback at all).
     *
     * @param power -1.0 (full speed one direction) to 1.0 (full speed the
     *              other direction), 0.0 = stopped
     */
    public void setServoRot(double power) {
        servoRot.setPower(power);
    }
}