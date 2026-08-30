package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanism.DcMotor;

@TeleOp

public class IntakeTest extends OpMode {
    DcMotor intake = new DcMotor();

    @Override
    public void init() {
        intake.init(hardwareMap, "intake");
    }

    @Override
    public void loop() {
        intake.setMotorSpeed(0.1);
    }
}
