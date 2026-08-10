package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechanism.RobotIMU;

/**
 * ============================================================================
 *  BENCH TEST: ROBOTIMU WRAPPER
 * ============================================================================
 *
 *  Intended purpose: bench-test RobotIMU.java (the standalone hub-IMU
 *  wrapper) by printing its live yaw reading to telemetry.
 *
 * ============================================================================
 */

@Disabled
@TeleOp
public class TestRobotIMU extends OpMode{
    RobotIMU Imu1 = new RobotIMU();

    private double Yaw;
    @Override
    public void init() {
        Imu1.init(hardwareMap);
        // --- ADDED TELEMETRY: make the empty init() visible, since a silent init()
        // can otherwise look identical to a hang/crash from the driver station ---
        telemetry.addLine("TestRobotIMU: init() ran (note: IMU is NOT initialized here -- see Issue 2 in the file header comment; it's (re-)initialized every loop() call instead)");
        telemetry.update();
    }

    @Override
    public void loop() {
        Yaw = Imu1.getHeading(AngleUnit.DEGREES);

        // --- ADDED TELEMETRY: live yaw reading, updated every loop -- this is the
        // actually-reliable place to read this OpMode's IMU output, since (per Issue 3
        // above) start()'s telemetry line will always show a stale 0.0 ---
        telemetry.addData("Yaw (deg, live, loop())", Yaw);
        telemetry.addData("loop runtime (s)", getRuntime());
        telemetry.update();
    }
}