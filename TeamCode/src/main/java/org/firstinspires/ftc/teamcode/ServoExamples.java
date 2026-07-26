package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanism.ServoClass;

@TeleOp
public class ServoExamples extends OpMode {
    ServoClass servo = new ServoClass();
    double leftTrigger, rightTrigger;

    @Override
    public void init() {
        servo.init(hardwareMap);
    }

    @Override
    public void loop() {
        leftTrigger = gamepad1.left_trigger;
        rightTrigger = gamepad1.right_trigger;

        servo.setServoPos(leftTrigger);
        servo.setServoRot(rightTrigger);
    }
}
