package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class TurretMechanism {

    private DcMotorEx turret;

    private double kP = 0.0001;
    private double kD = 0.0;

    private double lastError = 0.0;
    private double power = 0.0;

    private static final double MAX_POWER = 0.20;
    private static final double SEARCH_POWER = 0.15;
    private static final double ANGLE_TOLERANCE = 0.2;

    private static final double SWEEP_SECONDS = 2.0;

    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime searchTimer = new ElapsedTime();

    // Search states
    private enum SearchState {
        SWEEP_OUT,
        RETURN_HOME,
        FINISHED
    }

    private SearchState searchState = SearchState.SWEEP_OUT;

    private boolean searching = true;

    public void init(HardwareMap hwMap) {
        turret = hwMap.get(DcMotorEx.class, "turret");

        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setPower(0.0);

        searchState = SearchState.SWEEP_OUT;
        searching = true;

        searchTimer.reset();
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

    public void resetTimer() {
        timer.reset();
        searchTimer.reset();
    }

    /**
     * Track a visible AprilTag using its TX angle.
     */
    public void update(double tx) {
        searching = false;

        double deltaTime = timer.seconds();
        timer.reset();

        double error = -tx;

        double pTerm = error * kP;
        double dTerm = 0.0;

        if (deltaTime > 0.0) {
            dTerm = ((error - lastError) / deltaTime) * kD;
        }

        if (Math.abs(error) < ANGLE_TOLERANCE) {
            power = 0.0;
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

    /**
     * Sweep out, return to the starting position, and stop.
     *
     * The OpMode must keep calling this method while searching.
     */
    public void searchForTag() {
        if (!searching) {
            searching = true;
            searchState = SearchState.SWEEP_OUT;
            searchTimer.reset();
        }

        switch (searchState) {

            case SWEEP_OUT:
                if (searchTimer.seconds() >= SWEEP_SECONDS) {
                    turret.setPower(0.0);

                    searchState = SearchState.RETURN_HOME;
                    searchTimer.reset();
                } else {
                    // This sign was selected for your previously
                    // observed correct physical search direction.
                    turret.setPower(-SEARCH_POWER);
                }
                break;

            case RETURN_HOME:
                if (searchTimer.seconds() >= SWEEP_SECONDS) {
                    turret.setPower(0.0);

                    searchState = SearchState.FINISHED;
                    searching = false;
                } else {
                    // Reverse direction to return toward the start.
                    turret.setPower(SEARCH_POWER);
                }
                break;

            case FINISHED:
                turret.setPower(0.0);
                searching = false;
                break;
        }
    }

    public boolean isSearching() {
        return searching;
    }

    public double getTurretPositionDegrees() {
        return turret.getCurrentPosition();
    }

    public void stop() {
        turret.setPower(0.0);
    }
}