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
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
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
 * TEAM ALIGNMENT TELEOP -- LIMELIGHT + ODOMETRY. u drive, the bot aims the turret at the RED goal
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
 *   shooter speed   = AUTOMATIC from the distance (pollen: 3384 RPM at 52 in, 2 RPM per inch, MAX 3420;
 *                     nectar: its own physics speed - 10 RPM)
 *   dpad up / down  = servo TRES: up = increase, down = decrease. small step per tap (TRES_DPAD_STEP)
 *   servo tres      = MANUAL with dpad up / down only. it does NOT move at start: it stays put until you tap the dpad.
 *   NECTAR / POLLEN = AUTOMATIC. the color sensor ("color") in the intake path checks every ball while you intake,
 *                     every loop, at the same time as the turret aim, tres angle and shooter speed:
 *                       nectar (any red or blue) -> servo uno + dos open the gap to 76 deg (dos +0.01) AND the shooter
 *                       switches to the NECTAR speed. pollen (yellow) -> pollen gap (0 deg) + pollen speed.
 *                     the gap holds for 4 s of intaking after a ball passes the sensor (nectar + pollen in a row
 *                     don't confuse it). while the gap is moving the intake waits a moment.
 *   A / CROSS       = force POLLEN gap      B / CIRCLE = force NECTAR gap      X / SQUARE = back to AUTO
 *   (dpad left/right: nothing now -- uno + dos are automatic. turret is auto only)
 *   Y / TRIANGLE    = switch which tag to track: ALL RED -> 0 -> 1 -> ... -> 7 -> ALL RED
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
@TeleOp(name = "red 29", group = "Linear OpMode")
public class red extends LinearOpMode {

    // ================= DRIVE =================
    // directions: same as Drive_Test (WIRING 'A'), checked on the robot: all 6 moves right
    static final DcMotor.Direction LF_DIR = DcMotor.Direction.REVERSE;
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
    static final DcMotorSimple.Direction SHOOTER_DIRECTION = DcMotorSimple.Direction.REVERSE;
    static final double SHOOTER_TICKS_PER_REV = 28;
    static final double SHOOTER_MAX_RPM = 5000;          // HARD CAP
    static final double SHOOTER_IDLE_RPM = 3420;         // if there's no shot answer (too close etc.)
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
    static final double SHOOTER_RPM_BOOST = 0;           // (not used now: the speed rule below is the real number)
    // SHOOTER SPEED RULE (pollen): lowered 150 more: FARTHEST shot (70 in) is 3420 -> 52 in = 3384. 2 RPM per inch.
    static final double SPEED_AT_52_RPM = 3384;          // (was 3534, 3564, 3899 before that)
    static final double RPM_PER_INCH = 2.0;              // 5 RPM per 2.5 in
    static final double SPEED_MAX_RPM = 3420;            // HARD CAP for pollen (was 3570)
    static final double TRIM_STEP_PCT = 0.01;            // (not used now: dpad up / down runs servo tres)
    static final double GRAVITY_IN_S2 = 386.09;
    static final double APEX_CLEAR_IN = 3.0;             // ball peaks less than this above the goal = touchy shot (warning only)
    static final double SHOOTER_EXIT_HEIGHT_IN = 17.5;   // floor -> where the ball leaves the wheel
    static final double GOAL_HEIGHT_IN = 54.5;           // MEASURED: floor -> goal
    static final double WHEEL_DIAMETER_IN = 3.78;        // 96 mm
    static final double EXIT_PER_SURFACE = 0.258;        // MEASURED: 3900 RPM makes the shot at 52 in (was 0.32 = too little RPM)
    //   shots short everywhere? LOWER. long? RAISE.
    static final double MAX_SHOT_DIST_IN = 70;           // farthest you can be on YOUR field (43 x 52 in from the goal = 67 in).
    //   odometry / camera says farther? it's wrong -> speed uses this instead
    static final boolean MOVING_SHOT_COMP = true;        // aim ahead + change speed while driving
    static final double MOVE_SPEED_SIGN = +1;            // moving AWAY from the goal -> MORE speed, moving IN -> LESS
    static final double[] SPEED_TABLE_IN  = {24,  48,  72,  96,  120, 144};
    static final double[] SPEED_FACTOR    = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    static final double TRIM_STEP = 0.01;                // 1% = about 2.5 in of ball travel
    // SHOT CALIBRATION: the physics picks the ball speed from distance + goal height. what it CAN'T know is how much of
    // the wheel's speed the ball really gets (grip, squish), and that changes with distance. so: practice once, fix the
    // trim at a few distances, press X at each -> saved here and blended between them. file survives restarts.
    static final String CAL_FILE = System.getProperty("shotcal", "/sdcard/FIRST/shot_cal_v2.txt");   // new file: old saved points were for the old speed math
    static final double CAL_MERGE_IN = 8;               // saving within this many inches of an old point replaces it

    // ================= FIELD (RED CELLs, field grid) =================
    // the HIVE frame center is about (67, 72) (your 43 / 52 in + half of 48 x 40). each red CELL has its OWN cluster
    // of 4 AprilTags on its bottom face (game manual section 9), and the 2 CELLs are ~19 in apart, so EACH CELL gets its
    // own spot. these are ESTIMATES: the camera finds each CELL's real spot the first time it sees one of its tags.
    // telemetry "Cells" shows what the camera found -> type those numbers in here so odometry is right from the start.
    static final String[] CELL_NAMES = {"audience-side CELL", "far-side CELL"};
    static final int[][] CELL_TAGS = {{4, 5, 6, 7}, {0, 1, 2, 3}};
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
    static final double MAX_TURRET_DEG_PER_S = 1.0 * TURRET_TOP_DEG_PER_S;   // FULL speed: 1872 deg/s (312 RPM)
    static final double KP = 10.0;                        // how hard it chases: deg/s of turret speed per degree it's off
    static final double TURN_FF_GAIN = 1.0;
    // the turret motor takes a moment to change speed, so it was always a bit BEHIND the robot's turn and then kept
    // moving after you stopped. this starts the turret speeding up / slowing down that much EARLY (from how fast the
    // robot's turn is speeding up / slowing down). still drifts after a turn? raise a little. wobbles? lower.
    static final double TURRET_LAG_S = 0.05;
    static final double STRAFE_FF_GAIN = 1.0;
    static final boolean LATENCY_COMP = true;            // the cam pic is a lil old by the time we get it, this fixes that
    static final double ON_TARGET_DEG = 1.5;
    static final double TURRET_ACCEL_DEG_S2 = 4000;      // SMOOTH: it slows down as it gets close, so it stops ON the tag
    //   instead of spinning past it. still overshoots? lower. too lazy? raise
    static final double MANUAL_DEG_PER_S = 90;

    // ================= NECTAR / POLLEN: color sensor + servo uno + dos (the shooter gap) =================
    // same logic as Nectar Test RED / BLUE (tested in the sim). config names: "color", "servo uno", "servo dos"
    static final double POLLEN_DIAM_IN = 2.8, NECTAR_DIAM_IN = 3.6;
    static final Servo.Direction UNO_DIR = Servo.Direction.FORWARD;
    static final Servo.Direction DOS_DIR = Servo.Direction.REVERSE;   // mirror of uno
    static final double UNO_POLLEN_POS = 0.0, DOS_POLLEN_POS = 0.0;   // servo angle 0 = right squeeze on POLLEN (the end, not the middle)
    static final double NECTAR_DEG = 76;                 // nectar opening (same as the Nectar Test)
    static final double GAP_SERVO_RANGE_DEG = 300;       // goBILDA servo: 300 deg for position 0 -> 1
    static final int OPEN_SIGN = +1;                     // nectar opens toward a bigger position
    static final double DOS_EXTRA = 0.01;                // dos gets 0.01 more than uno when open
    static final double GAP_SERVO_DEG_PER_S = 300;       // while uno + dos turn, the intake waits (no ball hits a half-open gap)
    static final double GAP_SERVO_SETTLE_S = 0.03;
    static final double COMPRESSION_HOLD_S = 4.0;        // a ball keeps its gap for 4 s of intaking after it leaves the sensor
    // ball detection (gray ramp, bright yellow pollen, medium red + dark blue nectar)
    static final float SENSOR_GAIN = 8;
    static final double BALL_CLOSE_MAX_IN = 3.0, BALL_MARGIN_IN = 0.3, BALL_LEAVE_EXTRA_IN = 0.2;
    static final double BRIGHT_JUMP = 2.0, BRIGHT_MIN = 0.15;
    static final double COLOR_CHANGE = 0.10;             // color mix this different from the gray ramp = a ball
    static final double MIN_COLORFUL = 0.15;
    static final double RED_HUE_MAX = 30, RED_HUE_MIN_WRAP = 300;   // red nectar: 0-30 / 300-360
    static final double POLLEN_HUE_MIN = 38, POLLEN_HUE_MAX = 110;  // pollen: yellow
    static final double BLUE_HUE_MIN = 150, BLUE_HUE_MAX = 300;     // blue nectar
    static final int FRAMES_TO_SWITCH = 3;
    // NECTAR SHOT: bigger + heavier ball. the pollen speed rule above is measured; nectar speed = pollen speed x a factor
    // from physics (air drag on a bigger / heavier ball + how much the flywheel slows down launching a heavier ball).
    static final double POLLEN_WEIGHT_G = 15;            // MEASURE (kitchen scale)
    static final double NECTAR_WEIGHT_G = 30;            // MEASURE
    static final double FLYWHEEL_INERTIA_KGM2 = 2.3e-4;  // shooter wheel(s) + motor, estimate (goBILDA 96 mm wheels)
    static final double NECTAR_SPEED_TUNE = 1.00;        // nectar shots short? raise a little (1.02). long? lower
    static final double NECTAR_MAX_RPM = 3420;           // same cap as pollen
    static final double NECTAR_RPM_OFFSET = -10;         // nectar: 10 RPM less than its physics speed

    // ================= SERVO TRES (launch angle, automatic) =================
    // tres follows the distance to the goal every loop: farther = more angle. 45 deg of servo turn = 3 gear teeth.
    static final Servo.Direction TRES_DIR = Servo.Direction.FORWARD;   // (kept: so the START spot stays where it was)
    static final double TRES_TURN_SIGN = -1;             // -1 = backing up spins tres DOWN / OUTWARD (was +1 = up / inward)
    static final double TRES_RANGE_DEG = 300;            // goBILDA servo: 300 deg for position 0 -> 1
    static final double TRES_DPAD_STEP = 0.01;           // dpad tap = this much servo position (0.01 = 3 deg). too much? 0.005
    static final double TRES_START_POS = 0.0;            // what the code counts from on the FIRST dpad tap (servo position)
    static final double TRES_START_DEG = 0;              // tres starts at servo position 0 and STAYS there (was 45)
    static final double TRES_MAX_DEG = 0;                // = start: tres does not move with distance now (was 90)
    static final double TRES_REF_IN = 52;                // at this distance (or closer) tres sits at the start angle
    static final double TRES_DEG_PER_IN = 4.5;           // each inch farther = 4.5 deg more -> the full 45 deg over 10 in
    static final double TRES_SMOOTH_S = 0.15;            // smooths out odometry / camera jumps (steady, no jitter)
    static final double TRES_STEP_DEG = 0.5;             // only send a new position when it changed this much (no buzzing)
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
    private Servo tres;                                 // launch angle (automatic)
    private CRServo unoCR, dosCR;                       // (not used: the gap needs normal servos)
    private NormalizedColorSensor color;                // nectar / pollen sensor
    private DistanceSensor colorDist;
    private static final double[] nectarFactor = new double[81];   // nectar RPM / pollen RPM, for 0..80 in
    private GoBildaPinpointDriver odo;

    // RED goal in odometry terms (0,0 = start spot, X = robot forward, Y = robot left)
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
    // RED tags (BioBuzz manual section 9: 0-3 = red CELL far side, 4-7 = red CELL audience side)
    static final int[] RED_TAG_IDS = {0, 1, 2, 3, 4, 5, 6, 7};
    private int tagChoice = -1;                         // -1 = ANY red tag, otherwise index into RED_TAG_IDS (TRIANGLE switches)
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
    private final double[] estX = new double[2], estY = new double[2];     // the field estimate (from CELL_FIELD_XY)
    static final double CAM_DIST_TRUST = 0.30;           // camera distance used only if within 30% of the field estimate
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

        // ---------- drive: exactly Drive Test (WIRING 'A') ----------
        // each config name is its own wheel: "lfm" front-left, "rfm" front-right, "lbm" back-left, "rbm" back-right
        DcMotor lf = hardwareMap.get(DcMotor.class, "lfm");
        DcMotor lb = hardwareMap.get(DcMotor.class, "lbm");
        DcMotor rf = hardwareMap.get(DcMotor.class, "rfm");
        DcMotor rb = hardwareMap.get(DcMotor.class, "rbm");
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
        tres = hardwareMap.get(Servo.class, "servo tres");
        tres.setDirection(TRES_DIR);
        uno = hardwareMap.tryGet(Servo.class, "servo uno");
        dos = hardwareMap.tryGet(Servo.class, "servo dos");
        if (uno != null) uno.setDirection(UNO_DIR);
        if (dos != null) dos.setDirection(DOS_DIR);
        color = hardwareMap.tryGet(NormalizedColorSensor.class, "color");
        if (color != null) color.setGain(SENSOR_GAIN);
        colorDist = color instanceof DistanceSensor ? (DistanceSensor) color : null;
        // what "no ball" looks like (gray ramp) -- keep the intake path EMPTY during INIT
        double empty = Double.NaN, emptyBright = 0;
        double[] rampMix = new double[3];
        if (colorDist != null) { double sum = 0; int n = 0; for (int i = 0; i < 10; i++) { double d = colorDist.getDistance(DistanceUnit.INCH); if (!Double.isNaN(d)) { sum += d; n++; } } empty = n > 0 ? sum / n : Double.NaN; }
        if (color != null) for (int i = 0; i < 10; i++) { NormalizedRGBA e = color.getNormalizedColors(); double t = Math.max(1e-6, e.red + e.green + e.blue);
            emptyBright += t / 10; rampMix[0] += e.red / t / 10; rampMix[1] += e.green / t / 10; rampMix[2] += e.blue / t / 10; }
        double ballIn = Double.isNaN(empty) ? BALL_CLOSE_MAX_IN : Math.max(0.4, Math.min(BALL_CLOSE_MAX_IN, empty - BALL_MARGIN_IN));
        double ballOut = ballIn + BALL_LEAVE_EXTRA_IN;
        double brightIn = Math.max(BRIGHT_MIN, emptyBright * BRIGHT_JUMP);
        buildNectarFactor();                                // nectar speed physics, once

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
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.setOffsets(0, -5, DistanceUnit.INCH);           // TEAM robot pod offsets (X pod 0 in sideways, Y pod 5 in back)
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();
        placeGoal();
        // speed is fully automatic now: no saved calibration points are used

        telemetry.addData("CODE", "drive v5 | LF %s  LB %s  RF %s  RB %s", LF_DIR, LB_DIR, RF_DIR, RB_DIR);
        telemetry.addData("Status", "Ready. Robot on the START spot, turret STRAIGHT AHEAD.");
        telemetry.update();
        waitForStart();
        double tresDeg = TRES_START_DEG, tresSent = TRES_START_DEG;
        // START: tres is NOT sent anywhere -> it stays exactly where it is when you press start
        if (uno != null) uno.setPosition(UNO_POLLEN_POS);   // START: uno + dos at the POLLEN gap
        if (dos != null) dos.setPosition(DOS_POLLEN_POS);

        boolean shooterOn = false, prevLT = false, prevX = false, prevY = false, prevBack = false, prevUp = false, prevDown = false, wasReady = false;
        // nectar / pollen tracking (balls between the sensor and the shooter, in order)
        String gapMode = "AUTO";                            // AUTO, POLLEN (A), NECTAR (B)
        boolean wasThere = false, autoNectar = false, prevNectarShot = false;
        double feedClock = 0, feedPower = 0, lastGapDeg = 0, gapBusyUntil = 0, speedWaitUntil = 0;
        double[] cur = null;
        char curKind = '?', streakKind = '?';
        int streak = 0;
        double streakStart = 0;
        java.util.ArrayDeque<double[]> line = new java.util.ArrayDeque<>();   // {1 nectar / 0 pollen / -1 not sure, last seen}
        ElapsedTime runClock = new ElapsedTime();
        String seenName = color == null ? "no \"color\" sensor in config" : "NOTHING";
        double trim = 1.0;                                  // speed trim: fixed (dpad runs tres now)
        double tresPosNow = TRES_START_POS;                 // servo tres position the dpad changes
        boolean tresTapped = false;
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

            // TRIANGLE (Y): next tag. ALL RED -> 0 -> ... -> 7 -> ALL RED
            if (gamepad1.y && !prevY) {
                tagChoice = (tagChoice + 1 >= RED_TAG_IDS.length) ? -1 : tagChoice + 1;
                if (tagChoice >= 0) activeCell = cellOf(RED_TAG_IDS[tagChoice]);   // picked one tag -> aim at ITS cell
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
                // LAST SEEN SPOT: the camera's DIRECTION (robot heading + turret angle - tx) is exact. its distance can be off
                // (camera heights not measured), so: camera distance only if it agrees with the field estimate, else the
                // estimate's distance. tag lost -> odometry aims at this spot (right direction even if the distance is off)
                if (Math.abs(omegaDeg) < MAP_MAX_TURN_DEG_S && Math.abs(turretRateDeg) < MAP_MAX_TURN_DEG_S) {
                    int c = cellOf(tagId);
                    double estD = Math.hypot(estX[c] - x, estY[c] - y);
                    double useD = (!Double.isNaN(tag[1]) && Math.abs(tag[1] - estD) <= CAM_DIST_TRUST * estD) ? tag[1] : estD;
                    double dirRad = heading + Math.toRadians(turretDeg - tag[0]);
                    double mx = x + useD * Math.cos(dirRad), my = y + useD * Math.sin(dirRad);
                    if (!cellFromCamera[c]) { cellX[c] = mx; cellY[c] = my; cellFromCamera[c] = true; }
                    else { cellX[c] += TAG_SMOOTHING * (mx - cellX[c]); cellY[c] += TAG_SMOOTHING * (my - cellY[c]); }
                }
                activeCell = cellOf(tagId);             // aim + shoot at the CELL we're looking at
            }
            goalX = cellX[activeCell]; goalY = cellY[activeCell]; goalFromCamera = cellFromCamera[activeCell];

            // ================= NECTAR / POLLEN: color sensor -> uno + dos gap (every loop, while everything else runs) =================
            if (gamepad1.a) gapMode = "POLLEN";
            if (gamepad1.b) gapMode = "NECTAR";
            if (gamepad1.x) gapMode = "AUTO";
            if (feedPower > 0) feedClock += dt;             // balls only move toward the shooter while the intake pulls IN
            if (feedPower < 0) { line.clear(); wasThere = false; }   // intake reversed: balls go back out, start the line over
            boolean ballThere = false; double hue = 0, colorful = 0;
            if (color != null) {
                NormalizedRGBA c = color.getNormalizedColors();
                double[] hs = hueSat(c.red, c.green, c.blue); hue = hs[0]; colorful = hs[1];
                double inches = colorDist != null ? colorDist.getDistance(DistanceUnit.INCH) : Double.NaN;
                double bright = c.red + c.green + c.blue, tb = Math.max(1e-6, bright);
                double colorDiff = Math.sqrt(Math.pow(c.red / tb - rampMix[0], 2) + Math.pow(c.green / tb - rampMix[1], 2) + Math.pow(c.blue / tb - rampMix[2], 2));
                boolean byDistance = colorDist != null && inches <= (wasThere ? ballOut : ballIn);
                boolean byBrightness = bright >= (wasThere ? brightIn * 0.8 : brightIn);
                boolean byColor = colorDiff >= (wasThere ? COLOR_CHANGE * 0.7 : COLOR_CHANGE) && bright > emptyBright * 0.25;
                ballThere = byDistance || byBrightness || byColor;
            }
            String cls = classifyBall(hue, colorful);
            boolean sure = !cls.equals("UNSURE");
            char kind = !ballThere ? '-' : !sure ? 'U' : cls.equals("YELLOW") ? 'P' : 'N';
            seenName = color == null ? seenName : !ballThere ? "NOTHING" : !sure ? "checking..." : kind == 'P' ? "POLLEN" : "NECTAR (" + cls.toLowerCase() + ")";
            if (kind == streakKind) streak++; else { streakKind = kind; streak = 1; streakStart = feedClock; }
            if (ballThere && !wasThere) { cur = new double[]{-1, feedClock}; line.addLast(cur); curKind = '?'; }
            if (ballThere && (kind == 'N' || kind == 'P') && streak >= FRAMES_TO_SWITCH) {
                if (curKind == '?') { curKind = kind; cur[0] = kind == 'N' ? 1 : 0; }
                else if (kind != curKind) { cur[1] = streakStart; cur = new double[]{kind == 'N' ? 1 : 0, feedClock}; line.addLast(cur); curKind = kind; }
            }
            if (ballThere) cur[1] = feedClock;
            if (!ballThere && wasThere && cur != null && cur[0] < 0) cur[0] = 0;   // left without a clear color -> pollen
            wasThere = ballThere;
            while (!line.isEmpty() && !(ballThere && line.peekFirst() == cur) && feedClock > line.peekFirst()[1] + COMPRESSION_HOLD_S) line.removeFirst();
            if (!line.isEmpty() && line.peekFirst()[0] >= 0) autoNectar = line.peekFirst()[0] == 1;   // next ball at the shooter
            boolean nectarShot = gapMode.equals("NECTAR") || (gapMode.equals("AUTO") && autoNectar);
            double gapDeg = nectarShot ? NECTAR_DEG : 0;
            if (Math.abs(gapDeg - lastGapDeg) > 0.01) {      // gap changing -> intake waits until uno + dos get there
                gapBusyUntil = Math.max(gapBusyUntil, runClock.seconds()) + Math.abs(gapDeg - lastGapDeg) / GAP_SERVO_DEG_PER_S + GAP_SERVO_SETTLE_S;
                lastGapDeg = gapDeg;
            }
            if (nectarShot != prevNectarShot) speedWaitUntil = runClock.seconds() + 1.0;   // ball type changed: let the shooter re-settle
            prevNectarShot = nectarShot;
            double gapMove = OPEN_SIGN * gapDeg / GAP_SERVO_RANGE_DEG;
            double unoPos = Math.max(0, Math.min(1, UNO_POLLEN_POS + gapMove));
            double dosPos = Math.max(0, Math.min(1, DOS_POLLEN_POS + gapMove + (nectarShot ? OPEN_SIGN * DOS_EXTRA : 0)));
            if (uno != null) uno.setPosition(unoPos);
            if (dos != null) dos.setPosition(dosPos);
            boolean gapMoving = runClock.seconds() < gapBusyUntil;

            // ================= SHOT =================
            // speed is picked on its own from the distance
            // dpad up / down = servo TRES (up = increase, down = decrease), one small step per tap
            if (gamepad1.dpad_up && !prevUp)   { tresPosNow = Math.min(1.0, tresPosNow + TRES_DPAD_STEP); tres.setPosition(tresPosNow); tresTapped = true; }
            if (gamepad1.dpad_down && !prevDown) { tresPosNow = Math.max(0.0, tresPosNow - TRES_DPAD_STEP); tres.setPosition(tresPosNow); tresTapped = true; }
            prevUp = gamepad1.dpad_up; prevDown = gamepad1.dpad_down;
            solveShot(x, y, vx, vy, trim);

            // ================= SERVO TRES: manual only (dpad up / down above). no automatic angle =================

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
            } else {
                // can't see the tag -> odometry knows where the goal is: aim there right away (ALWAYS locked on)
                double want = Math.toDegrees(Math.atan2(goalY - y, goalX - x) - heading) + lead;
                double target = turretDeg + wrapDeg(want - turretDeg);              // shortest way...
                if (target > TURRET_MAX_DEG && target - 360 >= TURRET_MIN_DEG) target -= 360;   // ...unless that hits a limit:
                if (target < TURRET_MIN_DEG && target + 360 <= TURRET_MAX_DEG) target += 360;   //    then go the other way around
                error = target - turretDeg;
                source = goalFromCamera ? "odometry + encoder (camera spot)" : "odometry + encoder (field estimate)";
            }

            double cmdDegPerS;
            String turretState;
            boolean locked = false;
            boolean manualTurret = false;                   // dpad left/right runs servo uno + dos now
            // ALWAYS tracking: camera when it sees the tag, odometry when it doesn't. no "running away" stop, no sweeping
            // smooth: never faster than it can stop in (speed <= sqrt(2 x decel x degrees left)), gentle when close
            double correction = Math.signum(error) * Math.min(KP * Math.abs(error),
                    Math.sqrt(2 * TURRET_ACCEL_DEG_S2 * Math.abs(error)));
            cmdDegPerS = correction + ffDegPerS;
            locked = Math.abs(error) <= ON_TARGET_DEG;
            turretState = (locked ? "LOCKED" : "TRACKING") + (tagInView ? " (camera)" : " (odometry - can't see the tag)");

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
            double pollenRpm = shotValid ? Math.min(SHOOTER_MAX_RPM, shotRpm + SHOOTER_RPM_BOOST)
                    : (shotNote.startsWith("TOO FAR") ? SHOOTER_MAX_RPM : SHOOTER_IDLE_RPM + SHOOTER_RPM_BOOST);
            double nectarRpm = Math.min(NECTAR_MAX_RPM, pollenRpm * nectarFactorAt(shotDistIn) * NECTAR_SPEED_TUNE + NECTAR_RPM_OFFSET);
            double targetRpm = nectarShot ? nectarRpm : pollenRpm;  // speed for the ball that's NEXT at the shooter
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
            boolean ready = speedOk && shotValid && locked && !gapMoving;

            // ================= INTAKE / FEED =================
            // hold: right trigger IN, right bumper REVERSE (out), let go = stop
            int intakeMode = gamepad1.right_trigger > 0.2 ? 1 : gamepad1.right_bumper ? -1 : 0;   // RT in, RB reverse
            double intakePower;
            intakePower = intakeMode * INTAKE_POWER;                    // shooter keeps spinning while you intake
            // pulling IN waits while the gap is moving, and (shooter on) up to 1 s for the speed to switch nectar <-> pollen
            boolean speedSwitching = shooterActive && runClock.seconds() < speedWaitUntil && Math.abs(rpm - targetRpm) > 2 * RPM_TOLERANCE;
            if (intakeMode == 1 && (gapMoving || speedSwitching)) intakePower = 0;
            feedPower = intakePower;
            intake.setPower(intakePower);

            if (ready && !wasReady) gamepad1.rumble(200);
            wasReady = ready;

            // ================= TELEMETRY =================
            double sh = Math.toRadians(START_FIELD_HEADING_DEG);
            double fx = START_FIELD_X + Math.cos(sh) * x - Math.sin(sh) * y;
            double fy = START_FIELD_Y + Math.sin(sh) * x + Math.cos(sh) * y;
            double volts = battery.getVoltage();
            telemetry.addData("CODE", "drive v5 turret t8 cells, shot cal, fast turret RED | LF %s  LB %s  RF %s  RB %s", LF_DIR, LB_DIR, RF_DIR, RB_DIR);
            telemetry.addData("FIRE", ready ? ">>> READY - hold RIGHT TRIGGER to feed <<<"
                    : (!shooterActive ? "press LEFT TRIGGER to turn the shooter on" : !shotValid ? shotNote : !locked ? "aiming..." : "speed settling..."));
            telemetry.addData("Servo tres", tresTapped ? String.format("position %.3f (dpad up = increase, down = decrease)", tresPosNow)
                    : "not moved yet (stays where it is) - dpad up / down to adjust");
            StringBuilder q = new StringBuilder();
            for (double[] b : line) q.append(b[0] == 1 ? "NECTAR " : b[0] == 0 ? "pollen " : "? ");
            telemetry.addData("Ball", "sensor sees %s | next at shooter: %s | in line: %s", seenName,
                    nectarShot ? "NECTAR" : "pollen", line.isEmpty() ? "none" : q.toString().trim());
            telemetry.addData("Gap (uno + dos)", "%s %s | uno %.3f dos %.3f%s | hue %.0f colorful %.2f%s",
                    nectarShot ? "NECTAR 76 deg" : "POLLEN 0 deg", gapMode.equals("AUTO") ? "(auto)" : "(forced: X = auto)", unoPos, dosPos,
                    gapMoving ? " MOVING - intake waits" : "", hue, colorful, (uno == null || dos == null) ? " | !! servo uno/dos missing (or set as continuous)" : "");
            telemetry.addData("Speed", "pollen %.0f / nectar %.0f RPM (x%.3f) -> using %s", pollenRpm, nectarRpm,
                    nectarFactorAt(shotDistIn) * NECTAR_SPEED_TUNE, nectarShot ? "NECTAR" : "POLLEN");
            telemetry.addData("Shot", shotValid
                    ? String.format("dist %.0f in | need %.0f RPM | flight %.2f s | lead %.1f deg%s", shotDistIn, shotRpm, shotFlightS, shotLeadDeg,
                    shotNote.equals("ok") ? "" : " | " + shotNote.replace("ok, but ", ""))
                    : shotNote);
            telemetry.addData("Shooter", "%s  %.0f / %.0f RPM (err %.0f)",
                    shooterOn ? "ON (LT to stop)" : "OFF (LT to start)", rpm, targetRpm, rpm - targetRpm);
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
            telemetry.addData("Field", "robot (%.1f, %.1f) facing %.0f deg | RED cell (%.0f, %.0f) | %.0f in away",
                    fx, fy, wrapDeg(START_FIELD_HEADING_DEG + Math.toDegrees(heading)), gfx, gfy, Math.hypot(gfx - fx, gfy - fy));
            telemetry.addData("Intake", intakeMode == 1 ? "IN (right trigger)" : intakeMode == -1 ? "OUT (right bumper)" : "OFF");
            telemetry.addData("Battery", "%.2f V%s", volts, volts < 12.5 ? " - charge it" : "");
            telemetry.update();
        }

        for (DcMotor m : motors) m.setPower(0);
        intake.setPower(0);
        outtake.setPower(0);
        turret.setPower(0);
    }

    /** RED goal (field grid) -> odometry terms (0,0 = start spot, X = robot forward, Y = robot left). */
    private void placeGoal() {
        double sh = Math.toRadians(START_FIELD_HEADING_DEG);
        for (int c = 0; c < 2; c++) {                     // every CELL: field estimate -> odometry coords
            double dx = CELL_FIELD_XY[c][0] - START_FIELD_X, dy = CELL_FIELD_XY[c][1] - START_FIELD_Y;
            cellX[c] =  Math.cos(sh) * dx + Math.sin(sh) * dy;
            cellY[c] = -Math.sin(sh) * dx + Math.cos(sh) * dy;
            estX[c] = cellX[c]; estY[c] = cellY[c];
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
        if (tagChoice >= 0) return id == RED_TAG_IDS[tagChoice];
        for (int t : RED_TAG_IDS) if (t == id) return true;
        return false;
    }

    /** which CELL (0 or 1) a tag ID is on */
    private static int cellOf(int id) {
        for (int c = 0; c < 2; c++) for (int t : CELL_TAGS[c]) if (t == id) return c;
        return START_CELL;
    }

    private String tagChoiceName() {
        return tagChoice < 0 ? "ALL RED (0-7)" : "tag " + RED_TAG_IDS[tagChoice];
    }

    /**
     * shot: distance to the RED goal, ball speed for a 60 deg launch, and if we're moving a "fake goal" shifted
     * by (bot speed x flight time). turret lead uses the real motion; the SPEED uses MOVE_SPEED_SIGN.
     */
    private void solveShot(double x, double y, double vx, double vy, double trim) {
        double gx = goalX, gy = goalY;
        double trueBearing = Math.atan2(gy - y, gx - x);
        double realDist = Math.hypot(gx - x, gy - y);
        shotRealDist = realDist;
        shotTowardSpeed = MOVE_SPEED_SIGN * (vx * (gx - x) + vy * (gy - y)) / Math.max(realDist, 1);
        shotStillRpm = speedRule(Math.min(realDist, MAX_SHOT_DIST_IN), trim);

        // moving: the speed uses a "fake goal" shifted by (bot speed x flight time), the turret aims ahead
        double flight = 0, dist = realDist;
        double aimX = gx, aimY = gy;
        int iterations = MOVING_SHOT_COMP ? 6 : 1;
        for (int i = 0; i < iterations; i++) {
            aimX = gx - vx * flight;             // turret lead: where the goal "will be"
            aimY = gy - vy * flight;
            dist = Math.hypot(gx - MOVE_SPEED_SIGN * vx * flight - x, gy - MOVE_SPEED_SIGN * vy * flight - y);
            double ballInS = speedRule(Math.min(dist, MAX_SHOT_DIST_IN), 1.0) / 60.0 * Math.PI * WHEEL_DIAMETER_IN * EXIT_PER_SURFACE;
            flight = dist / Math.max(1, ballInS * Math.cos(Math.toRadians(LAUNCH_DEG)));
        }
        boolean distTooBig = realDist > MAX_SHOT_DIST_IN + 5;
        dist = Math.min(dist, MAX_SHOT_DIST_IN);
        shotDistIn = dist;
        shotFlightS = flight;
        shotLeadDeg = Math.toDegrees(wrapRad(Math.atan2(aimY - y, aimX - x) - trueBearing));
        shotRpm = speedRule(dist, trim);
        shotValid = true;
        shotNote = distTooBig ? String.format("odometry says %.0f in away - more than the field. check the START spot / Cells (using %.0f in)", realDist, MAX_SHOT_DIST_IN)
                : shotRpm >= SPEED_MAX_RPM - 0.5 ? String.format("at the %.0f RPM cap", SPEED_MAX_RPM) : "ok";
    }

    /** shooter RPM for distance d: 3384 at 52 in, 2 RPM per inch, times your saved calibration + trim, max 3420 */
    private double speedRule(double d, double trim) {
        double rpm = (SPEED_AT_52_RPM + RPM_PER_INCH * (d - 52)) * speedFactor(d) * trim;
        return Math.min(SPEED_MAX_RPM, rpm);
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

    /** tres angle (deg) -> servo position. the START spot never moves; the turn goes the TRES_TURN_SIGN way from it */
    private static double tresPos(double deg) {
        return Math.max(0, Math.min(1, (TRES_START_DEG + TRES_TURN_SIGN * (deg - TRES_START_DEG)) / TRES_RANGE_DEG));
    }

    /** hue 0-360 + how colorful (0-1) from r g b */
    private static double[] hueSat(double r, double g, double b) {
        double mx = Math.max(r, Math.max(g, b)), mn = Math.min(r, Math.min(g, b)), d = mx - mn, h = 0;
        if (d > 1e-9) { if (mx == r) h = 60 * (((g - b) / d) % 6); else if (mx == g) h = 60 * ((b - r) / d + 2); else h = 60 * ((r - g) / d + 4); }
        if (h < 0) h += 360;
        return new double[]{h, mx <= 1e-9 ? 0 : d / mx};
    }

    /** "RED" / "BLUE" (nectar), "YELLOW" (pollen) or "UNSURE" (gray, ball edge, between the bands) */
    private static String classifyBall(double hue, double colorful) {
        if (colorful < MIN_COLORFUL) return "UNSURE";
        if (hue <= RED_HUE_MAX || hue >= RED_HUE_MIN_WRAP) return "RED";
        if (hue >= POLLEN_HUE_MIN && hue <= POLLEN_HUE_MAX) return "YELLOW";
        if (hue >= BLUE_HUE_MIN && hue < BLUE_HUE_MAX) return "BLUE";
        return "UNSURE";
    }

    /** nectar RPM / pollen RPM at distance d (inches), from the table built at INIT */
    private static double nectarFactorAt(double d) {
        int i = (int) Math.max(0, Math.min(nectarFactor.length - 2, Math.floor(d)));
        double f = Math.max(0, Math.min(1, d - i));
        return nectarFactor[i] + f * (nectarFactor[i + 1] - nectarFactor[i]);
    }

    /**
     * NECTAR SHOT MATH (once at INIT): for each distance, the launch speed each ball needs at LAUNCH_DEG to reach the goal
     * (with AIR DRAG: drag slows a ball by area / weight, so size + weight both matter; nectar also leaves 0.4 in higher,
     * its center sits higher), then the FLYWHEEL part: launching a heavier ball takes more energy out of the wheel, so the
     * wheel must start a bit faster (energy: wheel loses 1/2 x 7/5 x mass x v^2). result = how much faster / slower than the
     * pollen RPM (which you measured) the nectar RPM has to be.
     */
    private static void buildNectarFactor() {
        for (int d = 0; d < nectarFactor.length; d++) {
            double dd = Math.max(20, d);
            double vp = launchWithDrag(dd, POLLEN_DIAM_IN, POLLEN_WEIGHT_G, SHOOTER_EXIT_HEIGHT_IN);
            double vn = launchWithDrag(dd, NECTAR_DIAM_IN, NECTAR_WEIGHT_G, SHOOTER_EXIT_HEIGHT_IN + (NECTAR_DIAM_IN - POLLEN_DIAM_IN) / 2);
            double wheelRadS = (SPEED_AT_52_RPM + RPM_PER_INCH * (dd - 52)) * 2 * Math.PI / 60;    // pollen wheel speed here
            double ep = Math.sqrt(1 + 1.4 * POLLEN_WEIGHT_G / 1000 * vp * vp / (FLYWHEEL_INERTIA_KGM2 * wheelRadS * wheelRadS));
            double en = Math.sqrt(1 + 1.4 * NECTAR_WEIGHT_G / 1000 * vn * vn / (FLYWHEEL_INERTIA_KGM2 * wheelRadS * wheelRadS));
            nectarFactor[d] = (vn / vp) * (en / ep);
        }
    }

    /** launch speed (m/s) at LAUNCH_DEG so a ball (diameter in, weight g) with air drag reaches goal height d inches away */
    private static double launchWithDrag(double dIn, double diamIn, double grams, double exitIn) {
        double d = dIn * 0.0254, r = diamIn * 0.0254 / 2, m = grams / 1000.0;
        double k = 1.2 * 0.5 * Math.PI * r * r / (2 * m);   // air density 1.2, sphere drag coefficient 0.5
        double th = Math.toRadians(LAUNCH_DEG), h0 = exitIn * 0.0254, hg = GOAL_HEIGHT_IN * 0.0254;
        double lo = 1, hi = 20;
        for (int it = 0; it < 40; it++) {
            double v = (lo + hi) / 2, x = 0, y = h0, vx = v * Math.cos(th), vy = v * Math.sin(th), dt = 0.001;
            while (x < d && y > -1) { double sp = Math.hypot(vx, vy); vx += -k * sp * vx * dt; vy += (-9.81 - k * sp * vy) * dt; x += vx * dt; y += vy * dt; }
            if (y < hg) lo = v; else hi = v;
        }
        return (lo + hi) / 2;
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
