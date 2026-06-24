package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.mechanism.AprilTagLimelight;
import org.firstinspires.ftc.teamcode.mechanism.Drive;

import java.awt.font.NumericShaper;

@TeleOp
public class AprilTagAutoAlignment extends OpMode {
    private final AprilTagLimelight atlimelight = new AprilTagLimelight();
    private final Drive drive = new Drive();

    // ----------- PD Controller --------------
    double kP = 0.002;
    double error = 0;
    double lastError = 0;
    double goalX = 0; //  ADD OFFSET HERE
    double angleTolerance = 0.4; // MAY NEED TO TWEAK THIS
    double kD = 0.0001;
    double curTime = 0;
    double lastTime = 0;

    // ------------- Driving Setup --------------
    double forward, strafe, rotate;

    // ------------- Controller Based PD Tuning ---------------
    double[] stepSizes = {0.01, 0.001, 0.0001};
    int stepIndex = 2;

    @Override
    public void init() {
        atlimelight.init(hardwareMap, 8);
        drive.init(hardwareMap);

        telemetry.addLine("Initialized");
    }

    public void start() {
        resetRuntime();
        curTime = getRuntime();
    }

    @Override
    public void loop() {
    // ------------- Get Drive Inputs ------------
    forward = -gamepad1.left_stick_y;
    strafe = gamepad1.left_stick_x;
    rotate = gamepad1.right_stick_x;

    // ------------- Get April Tag Info --------------
        atlimelight.update();

    // ------------- AutoAlign Rotation Logic -------------
        if (gamepad1.left_trigger > 0.3) {
            if (atlimelight.hasTag(11) && atlimelight.hasValidResult()) {
                error = goalX - atlimelight.getTx();

                if (Math.abs(error) < angleTolerance) {
                    rotate = 0;
                } else {
                    double pTerm = error * kP;
                    curTime = getRuntime();
                    double dT = curTime - lastTime;
                    double dTerm = ((error - lastError) / dT) * kD;

                    rotate = Range.clip(pTerm + dTerm, -0.4, 0.4);

                    lastError = error;
                    lastTime = curTime;
                }
            } else {
                lastTime = getRuntime();
                lastError = 0;
                rotate = 0;
            }
        } else {
            lastTime = getRuntime();
            lastError = 0;
            rotate = 0;
        }

        // Drive our motors
        drive.setPower(forward, strafe, rotate);

        // Update P and D on the fly
        // 'B' button cycles through the different step sizes for tuning precision
        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length; // Modulo wraps the index back to 0.
        }

        // D-pad left/right adjusts the p gain
        if (gamepad1.dpadLeftWasPressed()) {
            kP -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()) {
            kP += stepSizes[stepIndex];
        }

        // D-pad up/down adjusts the d gain
        if (gamepad1.dpadUpWasPressed()) {
            kD += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()) {
            kD -= stepSizes[stepIndex];
        }

        telemetry.addData("tx", atlimelight.getTx());
        telemetry.addData("error", error);
        telemetry.addData("rotate", rotate);
        telemetry.addData("kP", kP);
        telemetry.addData("kD", kD);
        telemetry.addData("step size", stepSizes[stepIndex]);
        telemetry.update();
    }
}
