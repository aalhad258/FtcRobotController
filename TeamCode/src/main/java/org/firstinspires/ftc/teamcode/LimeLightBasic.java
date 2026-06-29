package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanism.Blob;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class LimeLightBasic extends OpMode {
    private Limelight3A limelight3A;
    private List<Blob> blobResults;
    @Override
    public void init() {
        limelight3A= hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(9);
        blobResults = new ArrayList<>();
    }

    @Override
    public void start() {
        limelight3A.start();
    }

    @Override
    public void loop() {

        LLResult llResult = limelight3A.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            List<LLResultTypes.ColorResult> blobs = llResult.getColorResults();
            for (LLResultTypes.ColorResult blob : blobs) {
                blobResults.add(new Blob(
                        blob.getTargetXDegrees(),
                        blob.getTargetYDegrees(),
                        blob.getTargetArea()
                ));
            }

        }

    }
}
