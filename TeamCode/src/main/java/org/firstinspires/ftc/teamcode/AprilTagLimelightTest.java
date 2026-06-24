package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.mechanism.AprilTagLimelight;

@Autonomous
public class AprilTagLimelightTest extends OpMode {

    private AprilTagLimelight limelight = new AprilTagLimelight();;

    @Override
    public void init() {
        limelight.init(hardwareMap, 8);
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        limelight.update();
        LLResult result = limelight.getResult();
        if (limelight.hasTag(11)) {
            if (limelight.hasValidResult()) {
                Pose3D botpose = limelight.getRobotPose();
                telemetry.addData("Tx", limelight.getTx());
                telemetry.addData("Ty", limelight.getTy());
                telemetry.addData("Ta", limelight.getTa());
                telemetry.addData("BotPose", botpose.toString());
                telemetry.addData("Yaw", botpose.getOrientation().getYaw());
            }
        }
    }
}
