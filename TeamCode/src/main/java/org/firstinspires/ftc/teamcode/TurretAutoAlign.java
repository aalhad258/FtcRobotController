package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.teamcode.mechanism.TurretMechanism;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "Turret Auto Align")
public class TurretAutoAlign extends OpMode {

    private GoBildaPinpointDriver pinpoint;
    private TurretMechanism turret;

    // ==========================================
    // FIELD POSITION CONFIGURATION
    // ==========================================

    // Starting position: CHANGE TO ACTUAL TELEOP STARTING POSITION
    // Starting robot position
    private static final double START_X = 46.0;
    private static final double START_Y = 8.0;
    private static final double START_HEADING = 0.0;

    // Shooting target
    private static final double TARGET_X = 46.0;
    private static final double TARGET_Y = 55.0;


    @Override
    public void init() {

        // Initialize turret
        turret = new TurretMechanism();
        turret.init(hardwareMap);

        // Initialize Pinpoint
        pinpoint = hardwareMap.get(
                GoBildaPinpointDriver.class,
                "pinpoint"
        );

        pinpoint.setOffsets(
                0,
                -4,
                DistanceUnit.INCH
        );

        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );

        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );

        pinpoint.resetPosAndIMU();

        pinpoint.setPosition(
                new Pose2D(
                        DistanceUnit.INCH,
                        START_X,
                        START_Y,
                        AngleUnit.DEGREES,
                        START_HEADING
                )
        );

        telemetry.addLine("Turret Auto Align Initialized");
        telemetry.addLine("Make sure turret is facing forward!");
        telemetry.update();
    }


    @Override
    public void loop() {

        // ==========================================
        // 1. UPDATE ROBOT POSITION
        // ==========================================

        pinpoint.update();

        Pose2D robotPose = pinpoint.getPosition();

        double robotX = robotPose.getX(DistanceUnit.INCH);
        double robotY = robotPose.getY(DistanceUnit.INCH);

        double robotHeading = robotPose.getHeading(AngleUnit.DEGREES);


        // ==========================================
        // 2. CALCULATE ANGLE TO SHOOTING TARGET
        // ==========================================

        double dx = TARGET_X - robotX;
        double dy = TARGET_Y - robotY;

        double targetFieldAngle = Math.toDegrees(
                Math.atan2(dy, dx)
        );


        // ==========================================
        // 3. CALCULATE DESIRED TURRET ANGLE
        // ==========================================

        double desiredTurretAngle = targetFieldAngle - robotHeading;

        // Normalize to [-180, 180]
        desiredTurretAngle = Math.toDegrees(
                Math.atan2(
                        Math.sin(Math.toRadians(desiredTurretAngle)),
                        Math.cos(Math.toRadians(desiredTurretAngle))
                )
        );


        // ==========================================
        // 4. UPDATE TURRET PID
        // ==========================================

        turret.update(desiredTurretAngle);


        // ==========================================
        // 5. TELEMETRY
        // ==========================================

        telemetry.addData("Robot X", robotX);
        telemetry.addData("Robot Y", robotY);
        telemetry.addData("Robot Heading", robotHeading);

        telemetry.addData("Target Field Angle", targetFieldAngle);
        telemetry.addData("Desired Turret Angle", desiredTurretAngle);

        telemetry.addData(
                "Actual Turret Angle",
                turret.getTurretPositionDegrees()
        );

        telemetry.addData("Turret kP", turret.getKP());
        telemetry.addData("Turret kD", turret.getKD());

        telemetry.update();
    }


    @Override
    public void stop() {
        turret.stop();
    }
}