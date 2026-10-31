package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class TurretMechanism {

    private DcMotorEx turret;

    // Encoder and gearing
    private static final double MOTOR_TICKS_PER_REV = 537.7;
    private static final double TURRET_GEAR_RATIO = 2.75;

    private static final double TICKS_PER_TURRET_REV =
            MOTOR_TICKS_PER_REV * TURRET_GEAR_RATIO;

    // PID constants
    private double kP = 0.005;
    private double kD = 0.0001;

    private double lastError = 0.0;
    private double power = 0.0;

    private static final double MAX_POWER = 0.20;
    private static final double ANGLE_TOLERANCE = 0.5;

    private final ElapsedTime timer = new ElapsedTime();

    public void init(HardwareMap hwMap) {

        turret = hwMap.get(DcMotorEx.class, "turret");

        turret.setDirection(DcMotor.Direction.FORWARD);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // IMPORTANT:
        // Physically point the turret straight forward before initializing.
        // This establishes the encoder's zero position.
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turret.setPower(0);

        lastError = 0;
        timer.reset();
    }

    public void setKP(double newKP) {
        kP = newKP;
    }

    public void setKD(double newKD) {
        kD = newKD;
    }

    public double getKP() {
        return kP;
    }

    public double getKD() {
        return kD;
    }

    // Read encoder and convert ticks into turret degrees
    public double getTurretPositionDegrees() {

        return (turret.getCurrentPosition() / TICKS_PER_TURRET_REV) * 360.0;
    }

    // Continuously aim the turret at a desired angle relative to the robot
    public void update(double desiredAngle) {

        double deltaTime = timer.seconds();
        timer.reset();

        double actualAngle = getTurretPositionDegrees();

        // Difference between desired and actual turret angles
        double error = desiredAngle - actualAngle;

        // Normalize error to [-180, 180]
        error = Math.toDegrees(
                Math.atan2(
                        Math.sin(Math.toRadians(error)),
                        Math.cos(Math.toRadians(error))
                )
        );

        double pTerm = kP * error;

        double dTerm = 0;

        if (deltaTime > 0.001) {
            dTerm = kD * (error - lastError) / deltaTime;
        }

        if (Math.abs(error) < ANGLE_TOLERANCE) {
            power = 0;
        } else {
            power = Range.clip(
                    pTerm + dTerm,
                    -MAX_POWER,
                    MAX_POWER
            );
        }

        turret.setPower(power);

        lastError = error;
    }

    public void stop() {
        turret.setPower(0);
    }
}