package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

/**
 * TEAM ALIGNMENT TELEOP -- LIMELIGHT + ODOMETRY. u drive, the bot aims the turret at the BLUE goal
 * and picks the shooter speed by itself.
 *
 *   LIMELIGHT (pipeline 0) -> sees the AprilTag: tx = how far off the turret is, Full 3D = how far away it is.
 *                             tag in view -> the turret aims straight from the camera, AND the code puts the
 *                             tag's real spot on the field map (robot position + turret angle + camera distance).
 *   ODOMETRY (Pinpoint)    -> where the bot is + how it's moving. tag OUT of view -> the turret keeps aiming at
 *                             the spot the camera found (before the camera ever sees it: the field estimate).
 *                             the shot distance (shooter speed) always comes from this spot.
 *   TURRET ENCODER         -> the real turret angle; the aim angle is turned into motor ticks with it.
 *   SHOOTER ENCODER        -> checks the wheel speed.
 *   LimelightRelay         -> live camera view on the robot WiFi: http://192.168.43.1:5801
 *
 * CONTROLS (gamepad 1):
 *   left stick      = drive / strafe        right stick = spin       (same as Drive Test)
 *   right trigger   = intake IN  (hold, let go = stop)
 *   right bumper    = intake REVERSE / OUT (hold, let go = stop) -- clears a jam
 *   LEFT TRIGGER    = shooter ON / OFF (press once = on, press again = off). stays on while you intake.
 *   (M1 / M2 back buttons: nothing)
 *                     rumbles when the shot is READY -> then hold right trigger to feed
 *   dpad up / down  = shot strength +/- 1% (short? up. long? down). shown as "Trim"
 *   X / SQUARE      = SAVE: "this trim is right at THIS distance". saved on the Control Hub, used every match
 *                     after (TeleOp + auton) and blended between distances -> no more adjusting in matches
 *   dpad left/right = servo uno + dos (hold): move them together, dos 0.01 faster. let go = they STAY there.
 *                     they never move on their own. (turret is auto only now)
 *   Y / TRIANGLE    = switch which tag to track: ALL BLUE -> 38 -> 39 -> ... -> 45 -> ALL BLUE
 *   BACK / SHARE    = reset position -- ONLY with the robot on the START spot
 *
 * FIELD GRID (look down at the field, standing outside at the NEAR wall):
 *   (0,0) = near-LEFT corner.  X = inches from the LEFT wall.  Y = inches from the NEAR wall.
 *   heading 0 = facing right (+X), 90 = facing away from you (+Y).
 *
 * SET-UP: start the robot on the START spot below, turret pointing STRAIGHT AHEAD (that's turret angle 0).
 *   check: hold dpad LEFT -> the turret must turn LEFT. if it turns right, flip TURRET_DIRECTION
 *   (otherwise the camera puts the tag on the wrong side of the map and the out-of-view aim is mirrored).
 *
 * Hardware names: lfm, lbm, rfm, rbm, intake, shooter, odo, turret (DC motor 312 RPM + encoder),
 *                 limelight (Limelight 3A).
 */
@TeleOp(name = "blue 19", group = "Linear OpMode")
public class blue extends LinearOpMode {

    // ================= DRIVE =================
    // directions: forward strafing + wheels fighting on spin = RF and LB flipped -> these are the fixed ones
    static final DcMotor.Direction LF_DIR = DcMotor.Direction.FORWARD;
    static final DcMotor.Direction LB_DIR = DcMotor.Direction.FORWARD;
    static final DcMotor.Direction RF_DIR = DcMotor.Direction.REVERSE;
    static final DcMotor.Direction RB_DIR = DcMotor.Direction.FORWARD;
    static final double DRIVE_SPEED = 1.0;
    // smooth driving (from the old stable TeleOp): ignore tiny stick wobble, gentle near center, smooth speed changes
    static final double STICK_DEADBAND = 0.05;
    static final double ACCEL_PER_SEC = 10.0;            // how fast it can speed up (full speed in 0.1 s)
    static final double DECEL_PER_SEC = 10.0;            // how fast it can slow down / stop

    // ================= INTAKE =================
    static final DcMotorSimple.Direction INTAKE_DIRECTION = DcMotorSimple.Direction.FORWARD;  // + = pull balls IN (flipped: right trigger was pushing OUT on the robot)
    static final double INTAKE_POWER = 1.0;

    // ================= SHOOTER ("shooter", 1:1 6000 RPM, encoder) =================
    static final DcMotorSimple.Direction SHOOTER_DIRECTION = DcMotorSimple.Direction.FORWARD; // Forward if we are using only pollen shooter98
    static final double SHOOTER_TICKS_PER_REV = 28;
    static final double SHOOTER_MAX_RPM = 5000;          // HARD CAP
    static final double SHOOTER_IDLE_RPM = 3600;         // if there's no shot answer (too close etc.)
    static final double RPM_TOLERANCE = 40;              // READY only when within this many RPM of the target...
    static final double READY_HOLD_S = 0.15;             // ...for this long in a row (not while still speeding up)
    // simple speed control: power = target / TOP_RPM x (12 V / battery) + KP x (target - actual). can't run away
    static final double SHOOTER_TOP_RPM_12V = 5400;
    static final double SHOOTER_KP = 0.0008;
    static final double SHOOTER_KI = 0.001;              // slowly adds power until the wheel hits the EXACT target
    static final double SHOOTER_I_MAX = 0.25;
    static final double SPINUP_MAX_S = 1.5;              // encoder reads NOTHING? feeding is still allowed after this long

    // ================= SHOT MATH =================
    static final double LAUNCH_DEG = 60;                 // FIXED launch angle (no angle servo)
    static final double SHOOTER_RPM_BOOST = 10;          // added on top of the calculated shooter speed
    static final double GRAVITY_IN_S2 = 386.09;
    static final double APEX_CLEAR_IN = 3.0;             // ball peaks less than this above the goal = touchy shot (warning only)
    static final double SHOOTER_EXIT_HEIGHT_IN = 17.5;   // floor -> where the ball leaves the wheel
    static final double GOAL_HEIGHT_IN = 54.5;           // MEASURED: floor -> goal
    static final double WHEEL_DIAMETER_IN = 3.78;        // 96 mm
    static final double EXIT_PER_SURFACE = 0.32;         // shots short everywhere? LOWER. long? RAISE.
    static final boolean MOVING_SHOT_COMP = true;        // aim ahead + change speed while driving
    static final double MOVE_SPEED_SIGN = +1;            // moving AWAY from the goal -> MORE speed, moving IN -> LESS
    static final double[] SPEED_TABLE_IN  = {24,  48,  72,  96,  120, 144};
    static final double[] SPEED_FACTOR    = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    static final double TRIM_STEP = 0.01;                // 1% = about 2.5 in of ball travel
    // SHOT CALIBRATION: the physics picks the ball speed from distance + goal height. what it CAN'T know is how much of
    // the wheel's speed the ball really gets (grip, squish), and that changes with distance. so: practice once, fix the
    // trim at a few distances, press X at each -> saved here and blended between them. file survives restarts.
    static final String CAL_FILE = System.getProperty("shotcal", "/sdcard/FIRST/shot_cal.txt");
    static final double CAL_MERGE_IN = 8;               // saving within this many inches of an old point replaces it

    // ================= FIELD (BLUE CELLs, field grid) =================
    // the HIVE frame center is about (67, 72) (your 43 / 52 in + half of 48 x 40). each blue CELL has its OWN cluster
    // of 4 AprilTags on its bottom face (game manual section 9), and the 2 CELLs are ~19 in apart, so EACH CELL gets its
    // own spot. these are ESTIMATES: the camera finds each CELL's real spot the first time it sees one of its tags.
    // telemetry "Cells" shows what the camera found -> type those numbers in here so odometry is right from the start.
    static final String[] CELL_NAMES = {"audience-side CELL", "far-side CELL"};
    static final int[][] CELL_TAGS = {{38, 39, 40, 41}, {42, 43, 44, 45}};
    static final double[][] CELL_FIELD_XY = {{67, 56.6}, {67, 87.4}};   // VERIFY / replace with the "Cells" numbers
    static final int START_CELL = 0;                     // which CELL to aim at before the camera sees anything
    // where the robot CENTER starts + which way it faces. MEASURE it and start there every time.
    // !! if this is wrong, the code thinks the goal is closer/farther than it is -> shots land short/long !!
    // check: the "Field" line shows "__ in away" -- it must match a tape measure from robot center to goal center.
    // default: back against the near wall, centered in front of the goal, facing it (goal 43 in ahead)
    static final double START_FIELD_X = 43, START_FIELD_Y = 9, START_FIELD_HEADING_DEG = 90;

    // ================= TURRET (DC MOTOR, 312 RPM, encoder) -- Teamalignment's turret code =================
    static final DcMotorSimple.Direction TURRET_DIRECTION = DcMotorSimple.Direction.FORWARD; // flip if it turns the wrong way
    static final double TURRET_MOTOR_RPM = 312;               // goBILDA 5203 19.2:1 (312 RPM)
    static final double TURRET_MOTOR_TICKS_PER_REV = 537.7;   // 312 RPM motor: encoder counts per output turn
    static final double TURRET_GEAR_RATIO = 1.0;              // motor turns per 1 turret turn
    static final double TICKS_PER_TURRET_DEG = TURRET_MOTOR_TICKS_PER_REV * TURRET_GEAR_RATIO / 360.0;
    static final double TURRET_TOP_DEG_PER_S = TURRET_MOTOR_RPM * 360.0 / 60.0 / TURRET_GEAR_RATIO;
    static final double MAX_TURRET_DEG_PER_S = 0.8 * TURRET_TOP_DEG_PER_S;   // ~1500 deg/s (was capped at 360 -> fell behind fast spins)
    static final double KP = 10.0;                        // how hard it chases: deg/s of turret speed per degree it's off
    static final double TURN_FF_GAIN = 1.0;
    // the turret motor takes a moment to change speed, so it was always a bit BEHIND the robot's turn and then kept
    // moving after you stopped. this starts the turret speeding up / slowing down that much EARLY (from how fast the
    // robot's turn is speeding up / slowing down). still drifts after a turn? raise a little. wobbles? lower.
    static final double TURRET_LAG_S = 0.05;
    static final double STRAFE_FF_GAIN = 1.0;
    static final boolean LATENCY_COMP = true;            // the cam pic is a lil old by the time we get it, this fixes that
    static final double ON_TARGET_DEG = 1.5;
    static final double MANUAL_DEG_PER_S = 90;

    // ================= SERVO UNO + DOS (manual only, dpad left / right) =================
    // works whether the config says "Servo" or "Continuous Rotation Servo" (the code checks).
    // they NEVER move unless you hold dpad left/right. dos always gets 0.01 more than uno so they stay together.
    static final Servo.Direction UNO_DIR = Servo.Direction.FORWARD;
    static final Servo.Direction DOS_DIR = Servo.Direction.REVERSE;   // mirror of uno. wrong way? flip ONLY this line
    static final double UNO_DOS_START = 0.50;            // normal servos: where they are (their starting point)
    static final double UNO_SPEED = 0.40;                // normal servos: position per second while held
    static final double UNO_CR_POWER = 0.60;             // continuous servos: power while held
    static final double DOS_EXTRA = 0.01;                // dos: +0.01 (speed for normal servos, power for continuous)
    static final double OMEGA_NOISE_DEG_S = 3;
    static final double TURRET_MIN_DEG = -135, TURRET_MAX_DEG = 135;   // (you widened these) VERIFY the wires
    static final double SEARCH_DEG_PER_S = 60;           // how fast it sweeps when it's looking for the tag
    static final double RUNAWAY_CHECK_S = 0.4;           // how long we watch before deciding it's turning the wrong way
    // motor's own speed control (PIDF), units = encoder ticks/s:
    //   F = 32767 / top speed in ticks/s  (312 RPM x 537.7 / 60 = 2796 ticks/s -> F = 11.7)
    //   P = 10, I = 1, D = 0  (what Teamalignment ran)
    static final double TURRET_F = 0;   // lowest possible (was 32767 / top ticks/s = 11.7). P + I do all the work now
    static final double TURRET_P = 10.0, TURRET_I = 1.0, TURRET_D = 0;

    // ================= HARDWARE =================
    private DcMotorEx intake, outtake, turret;
    private Servo uno, dos;                             // manual (dpad left/right), if set up as normal servos
    private CRServo unoCR, dosCR;                       // same, if set up as continuous
    private GoBildaPinpointDriver odo;

    // BLUE goal in odometry terms (0,0 = start spot, X = robot forward, Y = robot left)
    private double goalX = 0, goalY = 0;

    // the shot answer (solveShot fills this in)
    private boolean shotValid = false;
    private double shotDistIn = 0, shotRpm = 0, shotLeadDeg = 0, shotFlightS = 0, shotStillRpm = 0, shotTowardSpeed = 0;
    private String shotNote = "-";
    private double shotRealDist = 0;                    // distance to the goal right now (not the moving-shot one)
    private final TreeMap<Double, Double> cal = new TreeMap<>();   // distance (in) -> speed factor, from CAL_FILE
    private String calNote = "";

    private Limelight3A limelight;

    // ================= LIMELIGHT =================
    static final int PIPELINE = 0;
    // BLUE tags (BioBuzz manual section 9: 38-41 = blue CELL audience side, 42-45 = blue CELL far side)
    static final int[] BLUE_TAG_IDS = {38, 39, 40, 41, 42, 43, 44, 45};
    private int tagChoice = -1;                         // -1 = ANY blue tag, otherwise index into BLUE_TAG_IDS (TRIANGLE switches)
    static final double TX_SIGN = +1;
    static final double MAX_RESULT_AGE_MS = 500;
    static final double SEEN_HOLD_S = 0.20;
    // BioBuzz tags are on the BOTTOM face of the CELL (game manual) -> about the goal height. VERIFY both heights.
    static final double CAM_HEIGHT_IN = 10, TAG_HEIGHT_IN = 54.5, CAM_PITCH_DEG = 20;   // VERIFY: measure these
    static final double TAG_SMOOTHING = 0.2;             // how fast the camera moves the remembered goal spot (0..1)
    static final double MAP_MAX_TURN_DEG_S = 60;         // only put the tag on the map when not spinning fast
    // (the camera picture is ~25 ms old)

    // turret aiming state (camera + odometry)
    private boolean goalFromCamera = false;              // false = goal is the field estimate, true = the camera put it there
    private final double[] cellX = new double[2], cellY = new double[2];   // each CELL's spot (odometry coords)
    private final boolean[] cellFromCamera = new boolean[2];
    private int activeCell = START_CELL;                 // the CELL we aim + shoot at
    private double lastTx = 0, lastTagDist = Double.NaN;
    private int tagId = -1, llTagCount = 0;
    private String llReason = "no result yet";
    private double llLatencyMs = 0;
    private LLResult lastResultObj = null;               // new-frame check (the SDK "staleness" number reads wrong)
    private double lastLLStamp = Double.NaN;
    private long lastHubStamp = Long.MIN_VALUE;
    private final ElapsedTime sinceNewFrame = new ElapsedTime();


    @Override
    public void runOpMode() {
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        // Init Limelight Port Relay
        LimelightRelay.start();

        // ---------- drive: exactly Drive Test ----------
        // the drive cables are plugged into different ports than before (found with Drive Check):
        //   front-left wheel = "rfm", front-right = "lbm", back-left = "rbm", back-right = "lfm"
        // directions below did NOT change. plug the cables back the old way? -> put the old names back
        DcMotor lf = hardwareMap.get(DcMotor.class, "rfm");
        DcMotor lb = hardwareMap.get(DcMotor.class, "rbm");
        DcMotor rf = hardwareMap.get(DcMotor.class, "lbm");
        DcMotor rb = hardwareMap.get(DcMotor.class, "lfm");
        lf.setDirection(LF_DIR);
        lb.setDirection(LB_DIR);
        rf.setDirection(RF_DIR);
        rb.setDirection(RB_DIR);
        DcMotor[] motors = {lf, lb, rf, rb};
        for (DcMotor m : motors) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // ---------- intake / shooter / turret ----------
        intake  = hardwareMap.get(DcMotorEx.class, "intake");
        outtake = hardwareMap.get(DcMotorEx.class, "shooter");
        turret  = hardwareMap.get(DcMotorEx.class, "turret");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // 5. LIMELIGHT
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(PIPELINE);
        limelight.start();
        telemetry.addData("Limelight live view", "http://192.168.43.1:5801  (laptop on ROBOT WiFi)");

        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();
        uno = hardwareMap.tryGet(Servo.class, "servo uno");
        dos = hardwareMap.tryGet(Servo.class, "servo dos");
        if (uno == null) unoCR = hardwareMap.tryGet(CRServo.class, "servo uno");
        if (dos == null) dosCR = hardwareMap.tryGet(CRServo.class, "servo dos");
        if (uno != null) uno.setDirection(UNO_DIR);
        if (dos != null) dos.setDirection(DOS_DIR);
        if (unoCR != null) unoCR.setDirection(UNO_DIR == Servo.Direction.FORWARD ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
        if (dosCR != null) dosCR.setDirection(DOS_DIR == Servo.Direction.FORWARD ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
        // nothing is sent to uno / dos here -> they don't move at INIT or START

        intake.setDirection(INTAKE_DIRECTION);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        outtake.setDirection(SHOOTER_DIRECTION);
        outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);           // our own simple speed control (below)
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        turret.setDirection(TURRET_DIRECTION);
        // Turret: angle 0 = wherever it points at INIT. speed mode + brake so it holds still (same as Teamalignment)
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setVelocityPIDFCoefficients(TURRET_P, TURRET_I, TURRET_D, TURRET_F);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ---------- Pinpoint ----------
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint ");
        odo.setOffsets(0, -5, DistanceUnit.INCH);           // TEAM robot pod offsets (X pod 0 in sideways, Y pod 5 in back)
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();
        placeGoal();
        loadCal();

        telemetry.addData("CODE", "drive v5 | LF %s  LB %s  RF %s  RB %s", LF_DIR, LB_DIR, RF_DIR, RB_DIR);
        telemetry.addData("Status", "Ready. Robot on the START spot, turret STRAIGHT AHEAD.");
        telemetry.update();
        waitForStart();

        boolean shooterOn = false, prevLT = false, prevX = false, prevY = false, prevBack = false, prevUp = false, prevDown = false, wasReady = false;
        double unoPos = UNO_DOS_START, dosPos = UNO_DOS_START;
        boolean unoDosTouched = false;                      // nothing sent to uno/dos until you press dpad left/right
        double trim = 1.0;
        double lastX = Double.NaN, lastY = 0, lastHeading = 0, vx = 0, vy = 0;
        double lastOmegaDeg = 0, alphaDeg = 0;
        double forwardCmd = 0, strafeCmd = 0, spinCmd = 0;      // smoothed stick values
        ElapsedTime loopTimer = new ElapsedTime();
        ElapsedTime spinTimer = new ElapsedTime();
        ElapsedTime atSpeedTimer = new ElapsedTime();
        double shooterI = 0;
        double searchDir = 1;                         // which way it sweeps while searching
        double runawayStartErr = Double.NaN;
        ElapsedTime runawayTimer = new ElapsedTime();
        boolean runaway = false;

        while (opModeIsActive()) {
            double dt = Math.max(1e-3, Math.min(loopTimer.seconds(), 0.1));
            loopTimer.reset();
            for (LynxModule hub : hubs) hub.clearBulkCache();

            // ================= ODOMETRY =================
            odo.update();
            Pose2D pose = odo.getPosition();
            double x = pose.getX(DistanceUnit.INCH);
            double y = pose.getY(DistanceUnit.INCH);
            double heading = pose.getHeading(AngleUnit.RADIANS);
            double omegaDeg = 0;
            if (!Double.isNaN(lastX)) {
                omegaDeg = Math.toDegrees(AngleUnit.normalizeRadians(heading - lastHeading)) / dt;
                vx += 0.5 * ((x - lastX) / dt - vx);
                vy += 0.5 * ((y - lastY) / dt - vy);
            }
            lastX = x; lastY = y; lastHeading = heading;
            if (Math.abs(omegaDeg) < OMEGA_NOISE_DEG_S) omegaDeg = 0;
            double alphaRaw = (omegaDeg - lastOmegaDeg) / dt;   // how fast the turn rate is changing (deg/s per s)
            lastOmegaDeg = omegaDeg;
            alphaDeg += 0.5 * (alphaRaw - alphaDeg);            // smoothed
            double turretDeg = turret.getCurrentPosition() / TICKS_PER_TURRET_DEG;

            // TRIANGLE (Y): next tag. ALL BLUE -> 38 -> ... -> 45 -> ALL BLUE
            if (gamepad1.y && !prevY) {
                tagChoice = (tagChoice + 1 >= BLUE_TAG_IDS.length) ? -1 : tagChoice + 1;
                if (tagChoice >= 0) activeCell = cellOf(BLUE_TAG_IDS[tagChoice]);   // picked one tag -> aim at ITS cell
            }
            prevY = gamepad1.y;

            if (gamepad1.back && !prevBack) {           // robot is back on the START spot
                odo.resetPosAndIMU();
                lastX = Double.NaN; vx = 0; vy = 0;
                placeGoal();
            }
            prevBack = gamepad1.back;

            // ================= DRIVE (Drive Test stick signs + the old TeleOp's smoothing) =================
            // stick up = forward, stick right = strafe right
            forwardCmd = slew(forwardCmd, shape(deadband(-gamepad1.left_stick_y)), dt);
            strafeCmd  = slew(strafeCmd,  shape(deadband( gamepad1.left_stick_x)), dt);
            spinCmd    = slew(spinCmd,    shape(deadband( gamepad1.right_stick_x)), dt);  // stick right = spin right
            double forward = forwardCmd, strafe = strafeCmd, spin = spinCmd;
            double lfP = forward + strafe + spin;
            double rfP = forward - strafe - spin;
            double lbP = forward - strafe + spin;
            double rbP = forward + strafe - spin;
            double max = Math.max(1.0, Math.max(Math.max(Math.abs(lfP), Math.abs(rfP)),
                    Math.max(Math.abs(lbP), Math.abs(rbP))));
            lf.setPower(lfP / max * DRIVE_SPEED);
            rf.setPower(rfP / max * DRIVE_SPEED);
            lb.setPower(lbP / max * DRIVE_SPEED);
            rb.setPower(rbP / max * DRIVE_SPEED);

            // ================= LIMELIGHT: see the tag -> exact goal spot on the map =================
            double turretRateDeg = turret.getVelocity() / TICKS_PER_TURRET_DEG;
            double[] tag = readTag(limelight.getLatestResult());          // {tx deg (+ = right), floor distance in} or null
            boolean tagInView = tag != null;
            if (tagInView) {
                lastTx = tag[0];
                lastTagDist = tag[1];
                // camera direction (robot heading + turret encoder angle - tx) + distance -> tag spot (odometry coords)
                if (!Double.isNaN(tag[1]) && Math.abs(omegaDeg) < MAP_MAX_TURN_DEG_S && Math.abs(turretRateDeg) < MAP_MAX_TURN_DEG_S) {
                    double dirRad = heading + Math.toRadians(turretDeg - tag[0]);
                    double mx = x + tag[1] * Math.cos(dirRad), my = y + tag[1] * Math.sin(dirRad);
                    int c = cellOf(tagId);
                    if (!cellFromCamera[c]) { cellX[c] = mx; cellY[c] = my; cellFromCamera[c] = true; }
                    else { cellX[c] += TAG_SMOOTHING * (mx - cellX[c]); cellY[c] += TAG_SMOOTHING * (my - cellY[c]); }
                }
                activeCell = cellOf(tagId);             // aim + shoot at the CELL we're looking at
            }
            goalX = cellX[activeCell]; goalY = cellY[activeCell]; goalFromCamera = cellFromCamera[activeCell];

            // ================= SHOT =================
            if (gamepad1.dpad_up && !prevUp) trim += TRIM_STEP;
            if (gamepad1.dpad_down && !prevDown) trim = Math.max(0.5, trim - TRIM_STEP);
            prevUp = gamepad1.dpad_up; prevDown = gamepad1.dpad_down;
            solveShot(x, y, vx, vy, trim);
            // X = save "the current trim is right at this distance" -> becomes part of the automatic speed
            if (gamepad1.x && !prevX) {
                if (shotValid) {
                    double d = shotRealDist, f = speedFactor(d) * trim;
                    cal.entrySet().removeIf(e -> Math.abs(e.getKey() - d) < CAL_MERGE_IN);
                    cal.put(d, f);
                    trim = 1.0;                             // it's in the table now
                    calNote = saveCal() ? String.format("SAVED %.0f in -> x%.2f", d, f) : "SAVE FAILED (file)";
                    gamepad1.rumble(400);
                } else calNote = "can't save: " + shotNote;
            }
            prevX = gamepad1.x;

            // ================= TURRET (Teamalignment's turret code) =================
            boolean tagKnown = true;                        // the goal spot always exists here (field estimate until
            // the camera finds the real one), so it aims instead of sweeping
            double turnFfDeg = -(omegaDeg + alphaDeg * (TURRET_LAG_S + dt / 2));   // + where the turn rate is GOING
            double strafeFfDeg = 0;
            if (tagKnown) {
                double rx = goalX - x, ry = goalY - y;
                double r2 = Math.max(rx * rx + ry * ry, 36);
                strafeFfDeg = Math.toDegrees((rx * (-vy) - ry * (-vx)) / r2);
            }
            double ffDegPerS = TURN_FF_GAIN * turnFfDeg + STRAFE_FF_GAIN * strafeFfDeg;

            double lead = (MOVING_SHOT_COMP && shotValid && tagKnown) ? shotLeadDeg : 0;   // aim ahead if we're moving
            double error = Double.NaN;
            String source = "-";
            if (tagInView) {
                error = -lastTx + lead;
                if (LATENCY_COMP) error -= (turretRateDeg - ffDegPerS) * llLatencyMs / 1000.0;
                source = "camera";
            } else if (tagKnown) {
                double want = Math.toDegrees(Math.atan2(goalY - y, goalX - x) - heading) + lead;
                error = wrapDeg(want - turretDeg);
                source = goalFromCamera ? "odometry + encoder (camera spot)" : "odometry + encoder (field estimate)";
            }

            double cmdDegPerS;
            String turretState;
            boolean locked = false;
            boolean manualTurret = false;                   // dpad left/right runs servo uno + dos now
            if (manualTurret) {
                cmdDegPerS = MANUAL_DEG_PER_S;   // + = LEFT (counter-clockwise)
                turretState = "MANUAL";
            } else if (runaway) {
                cmdDegPerS = 0;                       // safety stop bc it was turning AWAY from the tag
                turretState = "STOPPED: TURRET RUNNING AWAY - flip TURRET_DIRECTION (or TX_SIGN)";
            } else if (Double.isNaN(error)) {
                // no tag yet: sweep back and forth till the Limelight spots it
                if (turretDeg >= TURRET_MAX_DEG - 5) searchDir = -1;
                if (turretDeg <= TURRET_MIN_DEG + 5) searchDir = 1;
                cmdDegPerS = searchDir * SEARCH_DEG_PER_S + ffDegPerS;
                turretState = "SEARCHING (sweeping for the tag)";
            } else {
                double correction = Math.abs(error) <= ON_TARGET_DEG ? KP * error * 0.5 : KP * error;
                cmdDegPerS = correction + ffDegPerS;
                locked = Math.abs(error) <= ON_TARGET_DEG;
                turretState = (locked ? "LOCKED" : "TRACKING") + (tagInView ? "" : " (odometry)");
            }
            // runaway check: cam sees the tag, we turn toward it, but the error keeps getting BIGGER -> it's going the wrong way. stop ts
            boolean manual = manualTurret;
            if (!runaway && tagInView && !manual && Math.abs(error) > 8 && Math.abs(omegaDeg) < 90) {
                if (Double.isNaN(runawayStartErr)) { runawayStartErr = Math.abs(error); runawayTimer.reset(); }
                else if (runawayTimer.seconds() > RUNAWAY_CHECK_S) {
                    if (Math.abs(error) > runawayStartErr + 8 && Math.abs(turretRateDeg) > 20
                            && Math.signum(turretRateDeg) == Math.signum(error)) runaway = true;
                    runawayStartErr = Math.abs(error); runawayTimer.reset();
                }
            } else if (Double.isNaN(error) || Math.abs(error) <= 8) {
                runawayStartErr = Double.NaN;
            }
            if (manual) runaway = false;                    // nudging it by hand clears the stop

            cmdDegPerS = Math.max(-MAX_TURRET_DEG_PER_S, Math.min(MAX_TURRET_DEG_PER_S, cmdDegPerS));
            boolean atLimit = false;
            if ((turretDeg >= TURRET_MAX_DEG && cmdDegPerS > 0) || (turretDeg <= TURRET_MIN_DEG && cmdDegPerS < 0)) {
                cmdDegPerS = 0;
                atLimit = true;
            }
            turret.setVelocity(cmdDegPerS * TICKS_PER_TURRET_DEG);

            // ================= SHOOTER =================
            // LEFT TRIGGER = toggle: press once = shooter on, press again = off
            boolean ltDown = gamepad1.left_trigger > 0.5;
            if (ltDown && !prevLT) shooterOn = !shooterOn;
            prevLT = ltDown;
            boolean shooterActive = shooterOn;
            double targetRpm = shotValid ? Math.min(SHOOTER_MAX_RPM, shotRpm + SHOOTER_RPM_BOOST)
                    : (shotNote.startsWith("TOO FAR") ? SHOOTER_MAX_RPM : SHOOTER_IDLE_RPM + SHOOTER_RPM_BOOST);
            double rpm = Math.abs(outtake.getVelocity()) * 60.0 / SHOOTER_TICKS_PER_REV;
            if (shooterActive) {
                double ff = targetRpm / SHOOTER_TOP_RPM_12V * 12.0 / Math.max(battery.getVoltage(), 9);
                double err = targetRpm - rpm;
                double fix = rpm > 100 ? SHOOTER_KP * err : 0;
                // integral: only once the wheel is close (no wind-up during spin-up)
                if (rpm > 100 && Math.abs(err) < 400) shooterI = Math.max(-SHOOTER_I_MAX, Math.min(SHOOTER_I_MAX, shooterI + SHOOTER_KI * err * dt));
                outtake.setPower(Math.max(0, Math.min(1, ff + fix + shooterI)));
            } else {
                spinTimer.reset();
                shooterI = 0;
                outtake.setPower(0);
            }
            // READY = at speed and STEADY (within RPM_TOLERANCE for READY_HOLD_S), not just passing through on the way up
            if (!shooterActive || Math.abs(rpm - targetRpm) > RPM_TOLERANCE) atSpeedTimer.reset();
            boolean encoderDead = rpm < 100 && spinTimer.seconds() > 1.0;
            boolean speedOk = shooterActive && (atSpeedTimer.seconds() >= READY_HOLD_S
                    || (encoderDead && spinTimer.seconds() > SPINUP_MAX_S));
            boolean ready = speedOk && shotValid && locked;

            // ================= SERVO UNO + DOS (dpad right = +, dpad left = -, hold) =================
            int unoCmd = gamepad1.dpad_right && !gamepad1.dpad_left ? 1 : gamepad1.dpad_left && !gamepad1.dpad_right ? -1 : 0;
            if (unoCR != null) unoCR.setPower(unoCmd * UNO_CR_POWER);                     // let go = 0 = stays put
            if (dosCR != null) dosCR.setPower(unoCmd * (UNO_CR_POWER + DOS_EXTRA));
            if (unoCmd != 0) unoDosTouched = true;
            if (unoDosTouched) {                            // normal servos: only once you've used them, then they HOLD
                unoPos = Math.max(0, Math.min(1, unoPos + unoCmd * UNO_SPEED * dt));
                dosPos = Math.max(0, Math.min(1, dosPos + unoCmd * (UNO_SPEED + DOS_EXTRA) * dt));
                if (uno != null) uno.setPosition(unoPos);
                if (dos != null) dos.setPosition(dosPos);
            }

            // ================= INTAKE / FEED =================
            // hold: right trigger IN, right bumper REVERSE (out), let go = stop
            int intakeMode = gamepad1.right_trigger > 0.2 ? 1 : gamepad1.right_bumper ? -1 : 0;   // RT in, RB reverse
            double intakePower;
            intakePower = intakeMode * INTAKE_POWER;                    // shooter keeps spinning while you intake
            intake.setPower(intakePower);

            if (ready && !wasReady) gamepad1.rumble(200);
            wasReady = ready;

            // ================= TELEMETRY =================
            double sh = Math.toRadians(START_FIELD_HEADING_DEG);
            double fx = START_FIELD_X + Math.cos(sh) * x - Math.sin(sh) * y;
            double fy = START_FIELD_Y + Math.sin(sh) * x + Math.cos(sh) * y;
            double volts = battery.getVoltage();
            telemetry.addData("CODE", "drive v5 turret t8 cells, shot cal, fast turret | LF %s  LB %s  RF %s  RB %s", LF_DIR, LB_DIR, RF_DIR, RB_DIR);
            telemetry.addData("FIRE", ready ? ">>> READY - hold RIGHT TRIGGER to feed <<<"
                    : (!shooterActive ? "press LEFT TRIGGER to turn the shooter on" : !shotValid ? shotNote : !locked ? "aiming..." : "speed settling..."));
            telemetry.addData("uno + dos", (uno == null && unoCR == null) ? "no \"servo uno\" in config"
                    : unoCR != null ? String.format("continuous | power uno %.2f dos %.2f (dpad L/R)", unoCmd * UNO_CR_POWER, unoCmd * (UNO_CR_POWER + DOS_EXTRA))
                    : !unoDosTouched ? "not moved (dpad L/R to move)" : String.format("uno %.3f  dos %.3f (dpad L/R)", unoPos, dosPos));
            telemetry.addData("Shot", shotValid
                    ? String.format("dist %.0f in | need %.0f RPM | flight %.2f s | lead %.1f deg%s", shotDistIn, shotRpm, shotFlightS, shotLeadDeg,
                    shotNote.equals("ok") ? "" : " | " + shotNote.replace("ok, but ", ""))
                    : shotNote);
            telemetry.addData("Shooter", "%s  %.0f / %.0f RPM (err %.0f)  Trim %.2f",
                    shooterOn ? "ON (LT to stop)" : "OFF (LT to start)", rpm, targetRpm, rpm - targetRpm, trim);
            telemetry.addData("Tracking", "%s  (TRIANGLE to switch)", tagChoiceName());
            telemetry.addData("Turret", "%s%s", turretState, atLimit ? "  (AT SOFT LIMIT - turn the robot)" : "");
            telemetry.addData("  Turret angle", "%.1f deg (limits %.0f..%.0f)  speed %.0f deg/s, cmd %.0f",
                    turretDeg, TURRET_MIN_DEG, TURRET_MAX_DEG, turretRateDeg, cmdDegPerS);
            telemetry.addData("  Aim error", Double.isNaN(error) ? "-" : String.format("%.1f deg from %s", error, source));
            telemetry.addData("  Tag", tagInView ? String.format("ID %d  tx %.1f deg  %s", tagId, lastTx,
                    Double.isNaN(lastTagDist) ? "no distance (turn on Full 3D)" : String.format("%.0f in", lastTagDist))
                    : String.format("not in view (%s)", llReason));
            double shG = Math.toRadians(START_FIELD_HEADING_DEG);
            telemetry.addData("  Aiming at", "%s | new frame %.0f ms ago", CELL_NAMES[activeCell], sinceNewFrame.milliseconds());
            for (int c = 0; c < 2; c++)
                telemetry.addData("  Cells: " + CELL_NAMES[c], "field (%.1f, %.1f) from %s",
                        START_FIELD_X + Math.cos(shG) * cellX[c] - Math.sin(shG) * cellY[c],
                        START_FIELD_Y + Math.sin(shG) * cellX[c] + Math.cos(shG) * cellY[c],
                        cellFromCamera[c] ? "CAMERA" : "estimate");
            telemetry.addData("  Turret motor", "current %.2f A | encoder %d",
                    turret.getCurrent(CurrentUnit.AMPS), turret.getCurrentPosition());
            double gfx = START_FIELD_X + Math.cos(shG) * goalX - Math.sin(shG) * goalY;
            double gfy = START_FIELD_Y + Math.sin(shG) * goalX + Math.cos(shG) * goalY;
            telemetry.addData("Field", "robot (%.1f, %.1f) facing %.0f deg | BLUE cell (%.0f, %.0f) | %.0f in away",
                    fx, fy, wrapDeg(START_FIELD_HEADING_DEG + Math.toDegrees(heading)), gfx, gfy, Math.hypot(gfx - fx, gfy - fy));
            telemetry.addData("Intake", intakeMode == 1 ? "IN (right trigger)" : intakeMode == -1 ? "OUT (right bumper)" : "OFF");
            StringBuilder cs = new StringBuilder();
            for (Map.Entry<Double, Double> e : cal.entrySet()) cs.append(String.format("%.0fin x%.2f  ", e.getKey(), e.getValue()));
            telemetry.addData("Shot calibration", "%s | %s", cal.isEmpty() ? "none" : cs.toString().trim(), calNote);
            telemetry.addData("Battery", "%.2f V%s", volts, volts < 12.5 ? " - charge it" : "");
            telemetry.update();
        }

        for (DcMotor m : motors) m.setPower(0);
        intake.setPower(0);
        outtake.setPower(0);
        turret.setPower(0);
    }

    /** BLUE goal (field grid) -> odometry terms (0,0 = start spot, X = robot forward, Y = robot left). */
    private void placeGoal() {
        double sh = Math.toRadians(START_FIELD_HEADING_DEG);
        for (int c = 0; c < 2; c++) {                     // every CELL: field estimate -> odometry coords
            double dx = CELL_FIELD_XY[c][0] - START_FIELD_X, dy = CELL_FIELD_XY[c][1] - START_FIELD_Y;
            cellX[c] =  Math.cos(sh) * dx + Math.sin(sh) * dy;
            cellY[c] = -Math.sin(sh) * dx + Math.cos(sh) * dy;
            cellFromCamera[c] = false;                   // odometry restarted -> camera has to find the spots again
        }
        goalX = cellX[activeCell]; goalY = cellY[activeCell];
        goalFromCamera = false;
    }

    /** tag to aim at: {tx deg (+ = right), floor distance in (NaN = none)} or null. picks the tag closest to center. */
    private double[] readTag(LLResult result) {
        if (result == null) { llReason = "no result (Limelight in config? USB?)"; return null; }
        llLatencyMs = Math.max(0, Math.min(100, result.getCaptureLatency() + result.getTargetingLatency()));
        // NEW frame? (new result, or its timestamps changed). don't trust getStaleness(), it climbed on fresh data
        double stamp = result.getTimestamp();
        long hubStamp = result.getControlHubTimeStamp();
        if (result != lastResultObj || stamp != lastLLStamp || hubStamp != lastHubStamp) {
            sinceNewFrame.reset();
            lastResultObj = result; lastLLStamp = stamp; lastHubStamp = hubStamp;
        }
        if (sinceNewFrame.milliseconds() > MAX_RESULT_AGE_MS) { llReason = "no new frames (camera data froze)"; return null; }

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        llTagCount = tags == null ? 0 : tags.size();
        if (llTagCount == 0) { llReason = "no tag in view"; return null; }
        LLResultTypes.FiducialResult best = null, bestSameCell = null;
        for (LLResultTypes.FiducialResult f : tags) {
            if (!isTargetTag(f.getFiducialId())) continue;
            if (best == null || Math.abs(f.getTargetXDegrees()) < Math.abs(best.getTargetXDegrees())) best = f;
            if (cellOf(f.getFiducialId()) == activeCell
                    && (bestSameCell == null || Math.abs(f.getTargetXDegrees()) < Math.abs(bestSameCell.getTargetXDegrees()))) bestSameCell = f;
        }
        if (bestSameCell != null) best = bestSameCell;   // both CELLs in view? stay on the one we're aiming at
        if (best == null) { llReason = "tags seen, but not " + tagChoiceName() + " (TRIANGLE to switch)"; return null; }
        tagId = best.getFiducialId();
        double tx = TX_SIGN * best.getTargetXDegrees();

        // floor distance: Full 3D straight-line distance, minus the height difference (pythagoras). fallback: ty angle
        double dist = 0;
        Pose3D p = best.getTargetPoseCameraSpace();
        if (p != null && p.getPosition() != null) {
            double slant = Math.sqrt(p.getPosition().x * p.getPosition().x + p.getPosition().y * p.getPosition().y
                    + p.getPosition().z * p.getPosition().z) / 0.0254;
            double dh = TAG_HEIGHT_IN - CAM_HEIGHT_IN;
            dist = Math.sqrt(Math.max(slant * slant - dh * dh, 0));
        }
        if (dist < 1) {
            double a = Math.toRadians(CAM_PITCH_DEG + best.getTargetYDegrees());
            dist = Math.abs(Math.tan(a)) > 1e-3 ? (TAG_HEIGHT_IN - CAM_HEIGHT_IN) / Math.tan(a) : 0;
        }
        if (dist < 6 || dist > 200) dist = Double.NaN;  // no believable distance: aim only, don't move the goal spot
        llReason = "OK";
        return new double[]{tx, dist};
    }

    private boolean isTargetTag(int id) {
        if (tagChoice >= 0) return id == BLUE_TAG_IDS[tagChoice];
        for (int t : BLUE_TAG_IDS) if (t == id) return true;
        return false;
    }

    /** which CELL (0 or 1) a tag ID is on */
    private static int cellOf(int id) {
        for (int c = 0; c < 2; c++) for (int t : CELL_TAGS[c]) if (t == id) return c;
        return START_CELL;
    }

    private String tagChoiceName() {
        return tagChoice < 0 ? "ALL BLUE (38-45)" : "tag " + BLUE_TAG_IDS[tagChoice];
    }

    /**
     * shot: distance to the BLUE goal, ball speed for a 60 deg launch, and if we're moving a "fake goal" shifted
     * by (bot speed x flight time). turret lead uses the real motion; the SPEED uses MOVE_SPEED_SIGN.
     */
    private void solveShot(double x, double y, double vx, double vy, double trim) {
        double gx = goalX, gy = goalY;
        double trueBearing = Math.atan2(gy - y, gx - x);
        double realDist = Math.hypot(gx - x, gy - y);
        shotRealDist = realDist;
        shotTowardSpeed = MOVE_SPEED_SIGN * (vx * (gx - x) + vy * (gy - y)) / Math.max(realDist, 1);
        double vStill = launchSpeed(realDist);
        shotStillRpm = Double.isNaN(vStill) ? 0
                : vStill / EXIT_PER_SURFACE * 60.0 / (Math.PI * WHEEL_DIAMETER_IN) * speedFactor(realDist) * trim;

        double flight = 0, dist = realDist, v = Double.NaN;
        double aimX = gx, aimY = gy;
        int iterations = MOVING_SHOT_COMP ? 6 : 1;
        for (int i = 0; i < iterations; i++) {
            aimX = gx - vx * flight;             // turret lead: where the goal "will be"
            aimY = gy - vy * flight;
            dist = Math.hypot(gx - MOVE_SPEED_SIGN * vx * flight - x, gy - MOVE_SPEED_SIGN * vy * flight - y);
            v = launchSpeed(dist);
            if (Double.isNaN(v)) break;
            flight = dist / (v * Math.cos(Math.toRadians(LAUNCH_DEG)));
        }
        if (Double.isNaN(v)) {
            shotValid = false;
            shotNote = String.format("TOO CLOSE (%.0f in): ball would fly over. back up", dist);
            return;
        }
        double rpm = v / EXIT_PER_SURFACE * 60.0 / (Math.PI * WHEEL_DIAMETER_IN) * speedFactor(dist) * trim;
        shotDistIn = dist;
        shotFlightS = flight;
        shotLeadDeg = Math.toDegrees(wrapRad(Math.atan2(aimY - y, aimX - x) - trueBearing));
        if (rpm > SHOOTER_MAX_RPM) {
            shotValid = false;
            shotRpm = SHOOTER_MAX_RPM;
            shotNote = String.format("TOO FAR (%.0f in needs %.0f RPM): move closer", dist, rpm);
            return;
        }
        shotRpm = rpm;
        shotValid = true;
        shotNote = apexAboveGoal(dist) < APEX_CLEAR_IN
                ? String.format("ok, but %.0f in is a touchy shot - %.0f+ in is more reliable", dist, minShotDist()) : "ok";
    }

    /** launch speed (in/s) at LAUNCH_DEG to reach goal height d inches away. NaN = not possible (too close) */
    private static double launchSpeed(double d) {
        double th = Math.toRadians(LAUNCH_DEG);
        double h = GOAL_HEIGHT_IN - SHOOTER_EXIT_HEIGHT_IN;
        double denom = 2 * Math.cos(th) * Math.cos(th) * (d * Math.tan(th) - h);
        if (d < 6 || denom <= 0) return Double.NaN;
        double v = Math.sqrt(GRAVITY_IN_S2 * d * d / denom);
        // the CELL faces UP: the ball has to come DOWN into it. too close and this speed reaches goal height while the
        // ball is still going UP -> it flies over the goal. that's not a real shot.
        double vy = v * Math.sin(th), vx = v * Math.cos(th);
        if (d < vx * vy / GRAVITY_IN_S2) return Double.NaN;
        return v;
    }

    /** how far above the goal the ball peaks at distance d. under APEX_CLEAR_IN = touchy shot (1% speed = big miss) */
    private static double apexAboveGoal(double d) {
        double v = launchSpeed(d), th = Math.toRadians(LAUNCH_DEG);
        if (Double.isNaN(v)) return Double.NaN;
        double vy = v * Math.sin(th);
        return vy * vy / (2 * GRAVITY_IN_S2) - (GOAL_HEIGHT_IN - SHOOTER_EXIT_HEIGHT_IN);
    }

    /** closest distance where the shot is NOT touchy (for telemetry) */
    private static double minShotDist() {
        for (double d = 12; d < 200; d += 1) if (apexAboveGoal(d) >= APEX_CLEAR_IN) return d;
        return Double.NaN;
    }

    /** speed factor at distance d: your saved calibration points (blended in between), or the default table */
    private double speedFactor(double d) {
        if (!cal.isEmpty()) {
            Map.Entry<Double, Double> lo = cal.floorEntry(d), hi = cal.ceilingEntry(d);
            if (lo == null) return hi.getValue();          // closer than every saved point: use the closest one
            if (hi == null) return lo.getValue();          // farther than every saved point
            if (hi.getKey().equals(lo.getKey())) return lo.getValue();
            double f = (d - lo.getKey()) / (hi.getKey() - lo.getKey());
            return lo.getValue() + f * (hi.getValue() - lo.getValue());
        }
        if (d <= SPEED_TABLE_IN[0]) return SPEED_FACTOR[0];
        for (int i = 1; i < SPEED_TABLE_IN.length; i++) {
            if (d <= SPEED_TABLE_IN[i]) {
                double f = (d - SPEED_TABLE_IN[i - 1]) / (SPEED_TABLE_IN[i] - SPEED_TABLE_IN[i - 1]);
                return SPEED_FACTOR[i - 1] + f * (SPEED_FACTOR[i] - SPEED_FACTOR[i - 1]);
            }
        }
        return SPEED_FACTOR[SPEED_FACTOR.length - 1];
    }

    private void loadCal() {
        cal.clear();
        try (BufferedReader r = new BufferedReader(new FileReader(CAL_FILE))) {
            String line;
            while ((line = r.readLine()) != null) {
                String[] v = line.trim().split("\\s+");
                if (v.length == 2) cal.put(Double.parseDouble(v[0]), Double.parseDouble(v[1]));
            }
            calNote = cal.size() + " saved points";
        } catch (Exception e) {
            calNote = "no saved points yet (X saves one)";
        }
    }

    private boolean saveCal() {
        try {
            File f = new File(CAL_FILE);
            if (f.getParentFile() != null) f.getParentFile().mkdirs();
            try (FileWriter w = new FileWriter(f)) {
                for (Map.Entry<Double, Double> e : cal.entrySet()) w.write(String.format("%.1f %.4f%n", e.getKey(), e.getValue()));
            }
            return true;
        } catch (Exception e) {
            return false;
        }
    }

    private static double deadband(double v) { return Math.abs(v) < STICK_DEADBAND ? 0 : v; }

    private static double shape(double v) { return v; }   // straight 1:1: half stick = half speed, full stick = FULL speed

    private static double slew(double current, double target, double dt) {
        boolean speedingUp = Math.abs(target) > Math.abs(current) && Math.signum(target) != -Math.signum(current);
        double step = (speedingUp ? ACCEL_PER_SEC : DECEL_PER_SEC) * dt;
        if (target > current) return Math.min(target, current + step);
        return Math.max(target, current - step);
    }

    private static double wrapDeg(double d) {
        while (d > 180) d -= 360;
        while (d < -180) d += 360;
        return d;
    }

    private static double wrapRad(double r) {
        while (r > Math.PI) r -= 2 * Math.PI;
        while (r < -Math.PI) r += 2 * Math.PI;
        return r;
    }
}
