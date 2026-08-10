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
 * ============================================================================
 */
public class ServoClass {
    private Servo servoPos;
    private CRServo servoRot;

    public void init(HardwareMap hwMap) {
        servoPos = hwMap.get(Servo.class, "servo_pos");
        servoRot = hwMap.get(CRServo.class, "servo_rot");
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