package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechanism.Drive;

public class Robot {
    public Drive drive;

    public Robot(HardwareMap hwMap) {
        drive = new Drive(hwMap);

    }
}
