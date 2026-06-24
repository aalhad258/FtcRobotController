package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DcMotor {
    private com.qualcomm.robotcore.hardware.DcMotor motor;
    private double ticksPerRev;

    public void init(HardwareMap hwMap) {
        // DC motor
        motor = hwMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, "motor");
        motor.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER);
        ticksPerRev = motor.getMotorType().getTicksPerRev();
        motor.setZeroPowerBehavior(com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setMotorSpeed(double speed) {
        // accepts values from -1.0 to 0.1
        motor.setPower(speed);
    }

    public double getTicksPerRev() {
        return ticksPerRev;
    }

    public double currentPosition() {
        return motor.getCurrentPosition();
    }

    public double getMotorRevs() {
        return motor.getCurrentPosition() / ticksPerRev;
    }
}
