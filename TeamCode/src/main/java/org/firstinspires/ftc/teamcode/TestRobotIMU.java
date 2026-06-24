package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechanism.RobotIMU;

public class TestRobotIMU extends OpMode{
    RobotIMU Imu1 = new RobotIMU();

    private double Yaw;
    @Override
    public void init() {
    }

    @Override
    public void loop() {
        Imu1.init(hardwareMap);
        Yaw = Imu1.getHeading(AngleUnit.DEGREES);

    }

    @Override
    public void start() {
        telemetry.addData("Yaw =", Yaw);
    }
}
