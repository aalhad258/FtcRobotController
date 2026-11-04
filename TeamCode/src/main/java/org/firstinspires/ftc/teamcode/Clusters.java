package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanism.AprilTagLimelight;

@Disabled
@Autonomous
public class Clusters extends OpMode {

    private AprilTagLimelight limelight = new AprilTagLimelight();

    @Override
    public void init() {
        limelight.init(hardwareMap, 8);
        if (limelight.hasTag(34) || limelight.hasTag(35) || limelight.hasTag(36) || limelight.hasTag(37)) {
            telemetry.addData("shoot yes", "yes");
        } else {
            telemetry.addData("shoot no", "no");
        }
        telemetry.update();
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        limelight.update();
    }
}
