package org.firstinspires.ftc.teamcode.mechanism;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoClass {
    private Servo servoPos;
    private CRServo servoRot;

    public void init(HardwareMap hwMap) {
        servoPos = hwMap.get(Servo.class, "serve_pos");
        servoRot = hwMap.get(CRServo.class, "servo_pos");
        servoPos.scaleRange(0.5, 1.0); // set range from midpoint to 180
        servoPos.setDirection(Servo.Direction.REVERSE);
    }

    public void setServoPos(double angle) {
        servoPos.setPosition(angle);
    }

    public void setServoRot(double power) {
        servoRot.setPower(power);
    }
}
