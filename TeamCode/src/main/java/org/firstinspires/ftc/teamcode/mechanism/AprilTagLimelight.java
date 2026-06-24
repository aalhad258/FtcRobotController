package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class AprilTagLimelight {
    private Limelight3A limelight;
    private IMU imu;

    public void init(HardwareMap hwMap, int pipeline) {
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(pipeline);
        imu = hwMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
    }

    public void start() {
        limelight.start();
    }

    public void update() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
    }

    public LLResult getResult() {
        return limelight.getLatestResult();
    }

    public Pose3D getRobotPose() {
        LLResult result = getResult();
        if (result == null || !result.isValid()) {
            return null;
        }

        return result.getBotpose_MT2();
    }

    public boolean hasTag(int tagId) {
        LLResult result = getResult();
        if (result == null || !result.isValid()) {
            return false;
        }
        for (LLResultTypes.FiducialResult tag :
                result.getFiducialResults()) {
            if (tag.getFiducialId() == tagId) {
                return true;
            }
        }
        return false;
    }

    public double getTx() {
        LLResult result = getResult();
        return result != null ? result.getTx() : 0;
    }

    public double getTy() {
        LLResult result = getResult();
        return result != null ? result.getTy() : 0;
    }

    public double getTa() {
        LLResult result = getResult();
        return result != null ? result.getTa() : 0;
    }

    public boolean hasValidResult() {
        LLResult result = getResult();
        return result != null && result.isValid();
    }
}
