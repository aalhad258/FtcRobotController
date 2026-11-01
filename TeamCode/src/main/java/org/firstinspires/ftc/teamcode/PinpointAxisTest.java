package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.mechanism.PinpointPathing;

/**
 * Hold a button and watch the Pinpoint numbers. Put the robot on the floor first.
 *
 *  A = Drive.setPower axial   +0.3  (should drive FORWARD)
 *  B = Drive.setPower lateral +0.3  (should strafe RIGHT)
 *  X = Drive.setPower yaw     +0.3  (should spin CLOCKWISE)
 *
 * Expected Pinpoint readings:
 *  A: X goes UP.    B: Y goes DOWN.    X: heading goes DOWN.
 *
 * If the robot physically moves the right way but the numbers go the wrong way,
 * the Pinpoint encoder directions / offsets are the problem.
 * If the numbers are right but the robot moves wrong, the Drive direction flags are.
 *
 * Then set FORWARD_SIGN / STRAFE_SIGN / TURN_SIGN in PinpointPathing so that
 * "forward", "left", and "counter-clockwise" commands move the robot that way.
 */
@TeleOp(name = "Pinpoint Axis Test")
public class PinpointAxisTest extends OpMode {

    private final PinpointPathing pathing = new PinpointPathing();

    @Override
    public void init() {
        pathing.init(
                hardwareMap,
                new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0)
        );
    }

    @Override
    public void init_loop() {
        pathing.updatePose();
        telemetry.addData("Pinpoint Status", pathing.getPinpointStatus());
        telemetry.update();
    }

    @Override
    public void loop() {

        pathing.updatePose();

        double axial = gamepad1.a ? 0.3 : 0;
        double lateral = gamepad1.b ? 0.3 : 0;
        double yaw = gamepad1.x ? 0.3 : 0;

        pathing.rawDrive(axial, lateral, yaw);

        Pose2D p = pathing.getCurrentPose();

        telemetry.addData("Pinpoint Status", pathing.getPinpointStatus());
        telemetry.addData("X (in)", "%.2f", p.getX(DistanceUnit.INCH));
        telemetry.addData("Y (in)", "%.2f", p.getY(DistanceUnit.INCH));
        telemetry.addData("Heading (deg)", "%.2f", p.getHeading(AngleUnit.DEGREES));
        telemetry.addLine("A=axial+  B=lateral+  X=yaw+");
        telemetry.update();
    }

    @Override
    public void stop() {
        pathing.stop();
    }
}