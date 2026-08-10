package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanism.ServoClass;

/**
 * ============================================================================
 *  BENCH TEST: SERVOCLASS DEMO (POSITIONAL SERVO + CONTINUOUS-ROTATION SERVO)
 * ============================================================================
 *
 *  Minimal driver-controlled bench test for ServoClass.java: the left
 *  trigger drives the positional servo (servoPos) to an absolute position
 *  proportional to how far the trigger is pressed, and the right trigger
 *  drives the continuous-rotation servo (servoRot) at a power proportional
 *  to how far IT is pressed. No PID, no autonomous logic -- purely a manual
 *  "does the hardware respond correctly" check.
 *
 *  Trigger values (gamepad1.left_trigger / right_trigger) range 0.0
 *  (untouched) to 1.0 (fully pressed) -- note that because ServoClass's
 *  positional range is scaled to [0.5, 1.0] internally (see
 *  ServoClass.init()'s scaleRange call), a leftTrigger value of 0.0 does
 *  NOT command the servo's true mechanical zero; it commands one end of
 *  that already-restricted physical sub-range.
 * ============================================================================
 */
@TeleOp
public class ServoExamples extends OpMode {
    ServoClass servo = new ServoClass();
    double leftTrigger, rightTrigger;

    @Override
    public void init() {
        servo.init(hardwareMap);

        // --- ADDED TELEMETRY: confirm init completed. ---
        telemetry.addLine("ServoExamples: init complete (servo.init succeeded)");
        telemetry.update();
    }

    @Override
    public void loop() {
        leftTrigger = gamepad1.left_trigger;
        rightTrigger = gamepad1.right_trigger;

        servo.setServoPos(leftTrigger);
        servo.setServoRot(rightTrigger);

        // --- ADDED TELEMETRY: the original OpMode had no telemetry at all -- this
        // makes each trigger's raw value and the resulting commanded servo action
        // visible so it's obvious which physical servo should be responding to which
        // trigger, and by how much, without having to watch the hardware directly ---
        telemetry.addData("left_trigger (-> positional servo)", leftTrigger);
        telemetry.addData("right_trigger (-> continuous-rotation servo)", rightTrigger);
        telemetry.addData("loop runtime (s)", getRuntime());
        telemetry.update();
    }
}