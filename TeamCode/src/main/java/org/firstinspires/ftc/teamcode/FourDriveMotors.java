package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechanism.InitDrive;

@TeleOp
public class FourDriveMotors extends OpMode {
    InitDrive motors;

    @Override
    public void init() {
        motors = new InitDrive();
    }

    @Override
    public void loop() {
        // accepts values from -1.0 to 0.1
        double max;
        double axial = -gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

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

        motors.lfMotor.setPower(lfPower);
        motors.lbMotor.setPower(lbPower);
        motors.rfMotor.setPower(rfPower);
        motors.rbMotor.setPower(rbPower);

        telemetry.addData("lfPower", lfPower);
        telemetry.addData("lbPower", lbPower);
        telemetry.addData("rfPower", rfPower);
        telemetry.addData("rbPower", rbPower);

        telemetry.addLine("--- Drivetrain: Encoders (ticks) ---");
        telemetry.addData("lfEncoder", motors.lfMotor.getCurrentPosition());
        telemetry.addData("lbEncoder", motors.lbMotor.getCurrentPosition());
        telemetry.addData("rfEncoder", motors.rfMotor.getCurrentPosition());
        telemetry.addData("rbEncoder", motors.rbMotor.getCurrentPosition());

        telemetry.addLine("--- Drivetrain: Velocity (ticks/sec) ---");
        telemetry.addData("lfVelocity", motors.lfMotor.getVelocity());
        telemetry.addData("lbVelocity", motors.lbMotor.getVelocity());
        telemetry.addData("rfVelocity", motors.rfMotor.getVelocity());
        telemetry.addData("rbVelocity", motors.rbMotor.getVelocity());

        telemetry.addLine("--- Sensors ---");
        telemetry.addData("Heading (deg)", imu.getHeading(AngleUnit.DEGREES));

        telemetry.addLine("--- Inputs ---");
        telemetry.addData("axial", axial);
        telemetry.addData("lateral", lateral);
        telemetry.addData("yaw", yaw);

        telemetry.addLine("--- System ---");
        telemetry.addData("Battery (V)", voltageSensor.getVoltage());
        telemetry.addData("Loop time (ms)", loopTimer.milliseconds());
        telemetry.addData("Frequency (Hz)", 1000.0 / loopTimer.milliseconds());
        telemetry.addData("Runtime (s)", getRuntime());

        telemetry.update();
    }
}
