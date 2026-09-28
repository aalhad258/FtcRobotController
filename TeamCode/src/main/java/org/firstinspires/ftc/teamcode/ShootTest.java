package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanism.DcMotor;

@TeleOp
public class ShootTest extends OpMode {
    DcMotor motor = new DcMotor();
    double speed;

    @Override
    public void init() {
        motor.init(hardwareMap, "shooter");

        telemetry.addLine("TestDcMotor: init complete");
        telemetry.update();

        speed = 1.0;
    }

    @Override
    public void loop() {
        speed = 1.0;
        motor.setMotorSpeed(speed);

        telemetry.addData("commanded motorSpeed", speed);
        telemetry.addData("raw left_stick_y", gamepad1.left_stick_y);
        telemetry.addData("loop runtime (s)", getRuntime());
        telemetry.update();
    }
}