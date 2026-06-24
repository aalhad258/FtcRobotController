package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechanism.Drive;

public class Robot {
    public Drive drive = new Drive();

    public void init(HardwareMap hwMap) {
        drive.init(hwMap);
    }
}
