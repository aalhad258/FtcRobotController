package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanism.DcMotorTest;

@TeleOp
public class DcMotorPractice extends OpMode {
    DcMotorTest bench = new DcMotorTest();

    @Override
    public void init() {
        bench.init(hardwareMap);
    }

    @Override
    public void loop() {
        double motorSpeed = gamepad1.left_stick_y;
        bench.setMotorSpeed(motorSpeed);
        telemetry.addData("Ticks per Revolutions", bench.getTicksPerRev());
        telemetry.addData("current pos", bench.currentPosition());
        telemetry.addData("Motor Revs", bench.getMotorRevs());
    }
}
