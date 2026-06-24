package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanism.Drive;

@TeleOp
public class TestDrive extends OpMode {
    Drive drive = new Drive();;
    @Override
    public void init() {
        drive.init(hardwareMap);
    }

    @Override
    public void loop() {
        double axial = -gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;
        drive.setPower(axial, lateral, yaw);
    }
}
