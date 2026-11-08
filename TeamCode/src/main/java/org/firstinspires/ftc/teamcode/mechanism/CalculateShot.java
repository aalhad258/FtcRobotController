package org.firstinspires.ftc.teamcode.mechanism;

public class CalculateShot {

    // =========================================================
    // SHOOTING CONSTANTS
    // =========================================================

    private static final double HIVE_HEIGHT = 63.0;
    private static final double SHOOTER_HEIGHT = 10.0;

    private static final double WHEEL_RADIUS_METERS = 0.036;
    private static final double TICKS_PER_REV = 28.0;

    private static final double SMALL_CONSTANT = 2.5;
    private static final double BIG_CONSTANT = 3.0;

    private static final double GRAVITY = 9.81;


    // =========================================================
    // CALCULATE SHOT
    // =========================================================

    public double calculateShot(
            double robotX,
            double robotY,
            boolean chooseBall,
            double RPM
    ) {

        double vInit =
                WHEEL_RADIUS_METERS * RPM * 2.0 * Math.PI / 60.0;

        // Distance from robot to target, converted to meters
        double dx = 46.0 - robotX;
        double dy = 0.0 - robotY;

        double shootX = 0.0254 * Math.hypot(dx, dy);

        // Height difference between target and shooter
        double deltaY =
                (HIVE_HEIGHT - SHOOTER_HEIGHT) * 0.0254;

        // Calculate initial projectile velocity
        double vInitial = chooseBall
                ? vInit / SMALL_CONSTANT
                : vInit / BIG_CONSTANT;

        // Calculate quadratic coefficients
        double aConstant =
                GRAVITY * shootX * shootX
                        / (2.0 * vInitial * vInitial);

        double bConstant = -shootX;

        double cConstant =
                GRAVITY * shootX * shootX
                        / (2.0 * vInitial * vInitial)
                        + deltaY;

        // Calculate discriminant
        double discriminant =
                bConstant * bConstant
                        - 4.0 * aConstant * cConstant;

        // No real solution
        if (discriminant < 0.0 || aConstant == 0.0) {
            return 0.0;
        }

        // Solve for tangent of launch angle
        double tangent =
                (-bConstant + Math.sqrt(discriminant))
                        / (2.0 * aConstant);

        return Math.atan(tangent);
    }


    // =========================================================
    // GETTERS
    // =========================================================

}