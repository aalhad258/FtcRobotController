package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drive {
    private DcMotor lfMotor;
    private DcMotor lbMotor;
    private DcMotor rfMotor;
    private DcMotor rbMotor;

    public Drive(HardwareMap hwMap) {
        lfMotor = hwMap.get(DcMotor.class, "leftFrontMotor");
        lbMotor = hwMap.get(DcMotor.class, "leftBackMotor");
        rfMotor = hwMap.get(DcMotor.class, "rightFrontMotor");
        rbMotor = hwMap.get(DcMotor.class, "rightBackMotor");
        lfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lfMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lbMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rfMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rbMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lfMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        lbMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rfMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rbMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setPower(double axial, double lateral, double yaw) {
        // accepts values from -1.0 to 0.1
        double max;

        double lfPower = axial + lateral + yaw;
        double lbPower = axial - lateral + yaw;
        double rfPower = axial - lateral - yaw;
        double rbPower = axial + lateral - yaw;

        max = Math.max(Math.abs(lfPower), Math.abs(rfPower));
        max = Math.max(max, Math.abs(lbPower));
        max = Math.max(max, Math.abs(rbPower));

        if (max > 1.0) {
            lfPower /= max;
            rfPower /= max;
            lbPower /= max;
            rbPower /= max;
        }

        lfMotor.setPower(lfPower);
        lbMotor.setPower(lbPower);
        rfMotor.setPower(rfPower);
        rbMotor.setPower(rbPower);
    }
}