package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Shooter RPM Test")
public class ShooterRPMTest extends OpMode {

    private DcMotorEx shooter;

    @Override
    public void init() {

        // Get the shooter directly from the hardware map
        shooter = hardwareMap.get(
                DcMotorEx.class,
                "shooter"
        );

        // Use the encoder
        shooter.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER
        );

        telemetry.addLine("Shooter RPM Test: ready");
        telemetry.update();
    }

    @Override
    public void loop() {

        // Hold A to run the shooter
        if (gamepad1.a) {

            // TEMPORARY TEST POWER
            double power = 0.50;

            shooter.setPower(power);

        } else {

            shooter.setPower(0.0);
        }


        // Read encoder velocity
        double ticksPerSecond =
                shooter.getVelocity();


        // Get the motor's configured encoder resolution
        double ticksPerRev =
                shooter.getMotorType().getTicksPerRev();


        // Convert ticks/sec → revolutions/minute
        double rpm =
                (ticksPerSecond / ticksPerRev) * 60.0;


        // Telemetry
        telemetry.addData(
                "Power",
                "%.2f",
                gamepad1.a ? 0.50 : 0.0
        );

        telemetry.addData(
                "Encoder ticks/sec",
                "%.1f",
                ticksPerSecond
        );

        telemetry.addData(
                "Ticks/rev",
                "%.1f",
                ticksPerRev
        );

        telemetry.addData(
                "Shooter RPM",
                "%.1f",
                rpm
        );

        telemetry.update();
    }
}