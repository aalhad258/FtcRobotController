package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp
public class DistanceSensor extends OpMode {

    AnalogInput ranger;

    @Override
    public void init() {
        ranger = hardwareMap.get(AnalogInput.class, "ranger");
    }

    @Override
    public void loop() {
        telemetry.addData("Raw Voltage", ranger.getVoltage());
        //telemetry.addData("Inch 15DEG 0-1 Mode: ", (ranger.getVoltage()*32.5)-2.6);
        telemetry.addData("Inch 20DEG 0-0 Mode: ", (ranger.getVoltage()*48.7)-4.9);
        //telemetry.addData("Inch 17DEG 1-0 Mode: ", (ranger.getVoltage()*78.1)-18.2);

        telemetry.update();
    }
}
