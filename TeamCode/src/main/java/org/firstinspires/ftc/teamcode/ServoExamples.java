package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanism.ServoClass;

@TeleOp
public class ServoExamples extends OpMode {

    ServoClass servo1 = new ServoClass();
    ServoClass servo2 = new ServoClass();

    double lastUp = 0;

    double position1 = 1.0;
    double position2 = 0.0;

    @Override
    public void init() {
        servo1.init(hardwareMap, "servo_pos1");
        servo2.init(hardwareMap, "servo_pos2");

        // Set initial positions
        servo1.setServoPos(position1);
        servo2.setServoPos(position2);

        telemetry.addLine("ServoExamples: init complete");
        telemetry.addData("Servo 1 Position", position1);
        telemetry.addData("Servo 2 Position", position2);
        telemetry.update();
    }

    @Override
    public void loop() {

        // D-pad up: move both servos toward the center
        if (gamepad1.dpad_up && (getRuntime() - lastUp > 0.5)) {
            position1 -= 0.05;
            position2 += 0.05;

            lastUp = getRuntime();
        }

        // D-pad down: move both servos away from the center
        if (gamepad1.dpad_down && (getRuntime() - lastUp > 0.5)) {
            position1 += 0.05;
            position2 -= 0.05;

            lastUp = getRuntime();
        }

        // Keep positions within the valid servo range
        position1 = Math.max(0.0, Math.min(1.0, position1));
        position2 = Math.max(0.0, Math.min(1.0, position2));

        // Apply positions
        servo1.setServoPos(position1);
        servo2.setServoPos(position2);

        // Telemetry
        telemetry.addData("Servo 1 Position", position1);
        telemetry.addData("Servo 2 Position", position2);
        telemetry.addData("Runtime (s)", getRuntime());
        telemetry.update();
    }
}