package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drive {
    private DcMotor lfMotor;
    private DcMotor lbMotor;
    private DcMotor rfMotor;
    private DcMotor rbMotor;

    public void init(HardwareMap hwMap) {
        lfMotor = hwMap.get(DcMotor.class, "leftFrontMotor");
        lbMotor = hwMap.get(DcMotor.class, "leftBackMotor");
        rfMotor = hwMap.get(DcMotor.class, "rightFrontMotor");
        rbMotor = hwMap.get(DcMotor.class, "rightBackMotor");
        lfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lbMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rbMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lfMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lbMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rfMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rbMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lfMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        lbMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rfMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rbMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setPower(double axial, double lateral, double yaw, double speed) {
        // accepts values from -1.0 to 0.1
        double max;

        double lfPower = -axial + lateral - yaw;
        double lbPower = -axial - lateral - yaw;
        double rfPower = -axial - lateral + yaw;
        double rbPower = -axial + lateral + yaw;

        max = Math.max(Math.abs(lfPower), Math.abs(rfPower));
        max = Math.max(max, Math.abs(lbPower));
        max = Math.max(max, Math.abs(rbPower));

        if (max > 1.0) {
            lfPower /= max;
            rfPower /= max;
            lbPower /= max;
            rbPower /= max;
        }

        lfMotor.setPower(lfPower*speed);
        lbMotor.setPower(lbPower*speed);
        rfMotor.setPower(rfPower*speed);
        rbMotor.setPower(rbPower*speed);
    }
}