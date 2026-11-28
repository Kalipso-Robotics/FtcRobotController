package org.firstinspires.ftc.teamcode.kalipsorobotics.test.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.kalipsorobotics.PID.PIDFController;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.drivetrain.FieldOrientedDriveAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.ShootAllAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.ShooterRun;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretAutoAlign;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretRunMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.configs.FieldConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ModuleConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.TurretConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterInterpolationConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.MathFunctions;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.ResetOdometryToPos;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Position;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.IMUModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Turret;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.FlowerMech;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.IntakeModule;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.IntakeSide;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.IShooterPredictor;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.Shooter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.ShooterRunMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KFileWriter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KOpMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;

/**
 * Shot-table rows: the flywheel runs through ShooterRun in SHOOT_USING_TARGET_RPS_HOOD at the set target RPS and hood
 * (the same loop TeleOp uses: goToRPS = kF * target + kP * error, voltage compensated, plus bang-bang and hood
 * compensation), so ShooterConfig.kf / kp are live. Dashboard edits apply next loop.
 * Output: RobotLogs/RPSPredictionData_*.csv with CurrentRPS,CurrentPower,CurrentVoltage,HoodPosition,DistanceToTargetMM,
 * TargetX,TargetY (the Hive Cell), TurretOffsetDeg, AimX,AimY (the point aimed at: the Cell swung around the pivot by the offset),
 * FeedRPS (PushBall's closed-loop transfer target, both sides, IntakeConfig.FEED_RPS), TargetRPS,
 * FeedRpsComp (1 = shooter-RPS compensation on, 0 = the transfer holds FeedRPS), TransferTimedOut (1 = TransferReady gave
 * up after IntakeConfig.FEED_GATE_TIMEOUT_MS and the shot went on with the transfer out of tolerance: suspect row; filled in
 * when the shot ends).
 * CurrentPower is the power the PIDF put out.
 * Distance is from the turret pivot (odometry) to the Hive Cell on the robot's end (SharedData.getTargetPoint).
 * Pattern source: this file's earlier LinearOpMode version, ported to KOpMode + startOdometry.
 *
 * Turret aims at the Hive target the whole time (TurretAutoAlign, ticked by updateActions()).
 * Intake: both sides run while RT is held (not while PushBall is feeding).
 * Shots: LB runs ShootAllAction (TeleOp's shot: ShooterReady + TurretReady, abort after TELEOP_READY_TIMEOUT_MS).
 * PushBall also waits for TransferReady (both transfer sides at IntakeConfig.FEED_RPS within
 * FEED_RPS_TOLERANCE, held against the closed stopper). So the stopper opens only with the shooter (ShooterRun's tolerance
 * + ShooterConfig.timeToStabilize), the turret and the transfer all ready. No Limelight re-zero (odometry stays yours).
 * The row is read when PushBall starts, i.e. the RPS the first ball leaves at. A shot aborted on the timeout records nothing.
 *
 * Controls (GP1):
 *   sticks            field-centric drive (FieldOrientedDriveAction)
 *   dpad up / down    target RPS +/- RPS_STEP_BIG             A / B    target RPS +/- RPS_STEP_SMALL
 *   dpad right / left hood +/- HOOD_STEP (clamped MIN_HOOD..MAX_HOOD)
 *   LT (hold) + A / B hood +/- HOOD_STEP_FINE
 *   X                 toggle the shooter on/off at the set target RPS
 *   RT (hold)         run the intake (both sides) closed loop at the set transfer RPS (same target as the shot feed)
 *   LT (hold) + left stick click   flower down / up (down only while its intake side is down: hold RT first)
 *   GP2 left stick    flower, like TeleOp: up = down, down = up
 *   LB                shoot (ShootAllAction, waits for shooter + turret + transfer); LB again cancels. Records the row
 *   Y                 save the last recorded shot to the file (hits and misses: mark it first)
 *   LT (hold) + RB / Back   made count of the last shot +1 / -1 (first press: RB = 1, Back = 0; saved as Made, -1 = unmarked)
 *   RB                rezero: robot pushed into the loading-zone corner facing +x -> FieldConfig.CORNER_ZERO_POSE
 *   Back              rezero: robot pushed into the far (high x, low y) corner facing +x -> FieldConfig.FAR_CORNER_ZERO_POSE
 *   LT (hold) + dpad left / right   turret offset -/+ TURRET_OFFSET_STEP_DEG (TurretConfig.TURRET_OFFSET_DEG; 0 at start and at the end)
 *   LT (hold) + dpad down           turret offset back to 0
 *   LT (hold) + Y / X               transfer target +/- TRANSFER_RPS_STEP, both sides (IntakeConfig.FEED_RPS; RT and the
 *                                   next feed use it, put back at the end: copy the value into IntakeConfig by hand)
 *   LT (hold) + dpad up             toggle shooter-RPS feed compensation (IntakeConfig.FEED_RPS_COMP_ON; off = the transfer
 *                                   holds the set RPS; put back at the end)
 * All on GP1: Start+A / Start+B to swap gamepads would change the RPS. The offset only turns the turret: the RPS is
 * the set one and DistanceToTargetMM stays pivot to Cell.
 *
 *
 * LUT tune mode (right stick click toggles, starts off): target RPS and hood follow the live table
 * (Shooter.getPrediction at the current pivot-to-Cell distance, i.e. ShooterInterpolationConfig.shooterData interpolated,
 * offsets included) plus a trim you set. The same buttons then step the trim, finely:
 *   dpad up / down    RPS trim +/- LUT_RPS_STEP              A / B    RPS trim +/- LUT_RPS_STEP_FINE
 *   dpad right / left hood trim +/- LUT_HOOD_STEP            LT (hold) + A / B   hood trim +/- LUT_HOOD_STEP_FINE
 *   left stick click  trims back to 0 (pure LUT)
 * The trim stays when you drive to a new distance (reset it with left stick click). Leaving the mode keeps the
 * last LUT + trim values as the manual target, so nothing jumps. A shot uses the RPS / hood from the moment LB is pressed.
 * Extra CSV columns (after TransferTimedOut): LutMode,LutRPS,LutHood,RpsTrim,HoodTrim,BallsLoaded,Made
 * (BallsLoaded = Intake.getBallCount() at LB; Made = your mark, -1 if you saved without marking).
 * Y also writes RobotLogs/LUTRows_*.csv and logs event=lutRow: a row ready to paste into shooterData
 * (distance, rps, hood, FEED_RPS) with rpsOffset / hoodOffset / HOOD_INIT_OFFSET taken back out (adjustShooterRPS assumed 0).
 *
 * Turret debug: KLog tag Test_Shooter, "event=turret" every TURRET_LOG_MS: pose, target, field aim, turret deg,
 * servo position. Positive turretDeg must swing the turret to the robot's RIGHT (clockwise from above).
 * Shooter debug: same tag, "event=shooter" every SHOOTER_LOG_MS: RPS, target RPS, power, voltage, hood, distance, shot state.
 */
@Config
@TeleOp(name = "RPS Prediction Data Collector", group = "Test")
public class RPSPredictionDataCollector extends KOpMode {
    public static final String TAG = "Test_Shooter";
    public static double TURRET_TOLERANCE_DEG = 2;
    private static final double RPS_STEP_BIG = 2;
    private static final double RPS_STEP_SMALL = 0.5;
    private static final double HOOD_STEP = 0.005;
    private static final double HOOD_STEP_FINE = 0.001;
    private static final double TURRET_OFFSET_STEP_DEG = 1;
    private static final double TRANSFER_RPS_STEP = 1; // ~4% of the transfer's ~27 RPS free speed
    private static final long TURRET_LOG_MS = 250;
    private static final long SHOOTER_LOG_MS = 100; // faster than the turret: shows the RPS dip during a feed
    // LUT tune mode steps (Dashboard-editable)
    public static double LUT_RPS_STEP = 0.5;
    public static double LUT_RPS_STEP_FINE = 0.1;
    public static double LUT_HOOD_STEP = 0.002;
    public static double LUT_HOOD_STEP_FINE = 0.0005;
    public static int MAX_MADE = 4; // made-count ceiling (intake holds 4)
    private static final double FLOWER_STICK_DEADBAND = 0.05; // same as TeleOp

    private static final String[] LUT_CONTROLS = {
            "LUT MODE (R-stick click: back to manual)   target = LUT(distance) + trim",
            "dpad up/down: RPS trim +/-" + LUT_RPS_STEP + "   A/B: RPS trim +/-" + LUT_RPS_STEP_FINE,
            "dpad right/left: hood trim +/-" + LUT_HOOD_STEP + "   LT hold + A/B: hood trim +/-" + LUT_HOOD_STEP_FINE,
            "L-stick click: trims to 0   X: shooter on/off   RT hold: intake",
            "LB: shoot / cancel   LT hold + RB/Back: made +1/-1   Y: save last shot (CSV + paste-ready LUT row)",
            "RB / Back: rezero   LT hold + dpad: turret offset / feed comp   LT hold + Y/X: transfer RPS",
            "LT hold + L-stick click: flower down/up (RT first)   GP2 left stick: flower",
    };

    private static final String[] CONTROLS = {
            "sticks: field-centric drive   turret auto-aims at the Hive",
            "dpad up/down: target RPS +/-" + RPS_STEP_BIG + "   A/B: target RPS +/-" + RPS_STEP_SMALL,
            "dpad right/left: hood +/-" + HOOD_STEP + "   LT hold + A/B: hood +/-" + HOOD_STEP_FINE,
            "X: shooter on/off   RT hold: intake",
            "LB: shoot / cancel (ShootAllAction: shooter + turret + transfer ready)   Y: save last shot",
            "LT hold + RB / Back: last shot made +1 / -1 (mark before Y)",
            "RB: rezero odometry (loading-zone corner, facing +x)   Back: far corner, facing +x",
            "LT hold + dpad left/right: turret offset -/+" + TURRET_OFFSET_STEP_DEG + " deg   LT hold + dpad down: offset 0",
            "LT hold + Y/X: transfer RPS +/-" + TRANSFER_RPS_STEP + "   LT hold + dpad up: feed RPS comp on/off",
            "R-stick click: LUT tune mode (track the table + fine trim)",
            "LT hold + L-stick click: flower down/up (intake side must be down: RT)   GP2 left stick: flower",
    };

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();
        DriveTrain.setInstanceNull();
        DriveTrain driveTrain = DriveTrain.getInstance(opModeUtilities);
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);
        Shooter shooter = new Shooter(opModeUtilities);
        PIDFController pidf = shooter.getShooterMotor().getPIDFController();
        Intake intake = new Intake(opModeUtilities); // same as TeleOp: both sides + the flower (flowerPivot) on ModuleConfig.FLOWER_SIDE
        FlowerMech flower = intake.getFlower();
        Stopper stopper = new Stopper(opModeUtilities);
        Turret turret = Turret.getInstance(opModeUtilities);
        FieldOrientedDriveAction driveAction = new FieldOrientedDriveAction(driveTrain, imuModule);
        OpModeUtilities.startOdometry(opModeUtilities, odoExecutorService, driveTrain, imuModule, null); // keeps the pose from auto, else the corner

        TurretConfig.TURRET_OFFSET_DEG = 0;
        double feedRpsAtStart = IntakeConfig.FEED_RPS;
        boolean feedCompAtStart = IntakeConfig.FEED_RPS_COMP_ON;
        turretAutoAlignTeleOp = new TurretAutoAlign(opModeUtilities, turret); // follows SharedData.getTargetPoint(); updateActions() ticks it
        turretAutoAlignTeleOp.setTurretRunMode(TurretRunMode.RUN_USING_ODOMETRY);
        turretAutoAlignTeleOp.setToleranceDeg(TURRET_TOLERANCE_DEG);
        stopper.setPosition(ModuleConfig.STOPPER_SERVO_CLOSED_POS);
        flower.getServo().setPosition(FlowerMech.UP_POS); // start up: the interlock only lets it down once its intake side is down
        KLog.d(TAG, () -> String.format("event=turretConfig degPerServoDeg=%.4f servoRangeDeg=%.1f zeroPos=%.3f dir=%d limitsDeg=%.1f..%.1f pivot=%s startPose=%s",
                TurretConfig.TURRET_DEG_PER_SERVO_DEG, TurretConfig.SERVO_RANGE_DEG, TurretConfig.SERVO_ZERO_POS,
                TurretConfig.SERVO_DIRECTION, TurretConfig.MIN_TURRET_DEG, TurretConfig.MAX_TURRET_DEG, TurretConfig.ROBOT_REL_TURRET_POINT,
                SharedData.getOdometryWheelIMUPosition()));
        long lastTurretLogMs = 0;
        long lastShooterLogMs = 0;

        KFileWriter fileWriter = new KFileWriter("RPSPredictionData", opModeUtilities);
        fileWriter.writeLine("CurrentRPS,CurrentPower,CurrentVoltage,HoodPosition,DistanceToTargetMM,TargetX,TargetY,TurretOffsetDeg,AimX,AimY,FeedRPS,TargetRPS,FeedRpsComp,TransferTimedOut,LutMode,LutRPS,LutHood,RpsTrim,HoodTrim,BallsLoaded,Made");
        flush(fileWriter);
        KFileWriter lutRowWriter = new KFileWriter("LUTRows", opModeUtilities);
        lutRowWriter.writeLine("// paste into ShooterInterpolationConfig.shooterData (distanceMM, rps, hood, intakeRPS); keep sorted by distance");
        flush(lutRowWriter);
        // Every shot, automatically (no Y): one line when it ends, one per LT + RB / Back mark (the last line per n wins).
        // Survives logcat rotation, unlike the KLog lines.
        KFileWriter shotLogWriter = new KFileWriter("ShotLog", opModeUtilities);
        shotLogWriter.writeLine("n,event,ShooterTimedOut,CurrentRPS,CurrentPower,CurrentVoltage,HoodPosition,DistanceToTargetMM,TargetX,TargetY,TurretOffsetDeg,AimX,AimY,FeedRPS,TargetRPS,FeedRpsComp,TransferTimedOut,LutMode,LutRPS,LutHood,RpsTrim,HoodTrim,RobotX,RobotY,HeadingDeg,TurretDeg,BallsLoaded,Made,LutRow");
        flush(shotLogWriter);
        boolean lastShooterTimedOut = false;

        double targetRPS = 38;
        double hoodPosition = 0.25;
        shooter.getHood().setPosition(hoodPosition);
        // ShootAllAction needs a ShooterRun; updateActions() ticks it (KOpMode.shooterRun). STOP until X turns it on.
        ShooterRun rpsLoop = new ShooterRun(opModeUtilities, shooter, 0, hoodPosition);
        rpsLoop.stop();
        shooterRun = rpsLoop;
        Point hiveTarget = null;
        String lastShot = null; // complete row, ready for Y
        String pendingShot = null; // row of the shot being fed: TransferTimedOut is added when the shot ends
        String lastTransferGate = "none yet";
        boolean flowerDown = false;
        String message = "";
        boolean shooterOn = false;
        ShootAllAction shot = null;
        boolean wasShooting = false;
        // LUT tune mode
        boolean lutMode = true; // start on the table; manual 0.25 hood shot low into the HIVE bottom everywhere (2026-11-28)
        double rpsTrim = 0;
        double hoodTrim = 0;
        // snapshot at LB: what the shot was fired with (the live values keep tracking the LUT during the shot)
        double shotTargetRPS = 0, shotHood = 0, shotLutRPS = 0, shotLutHood = 0, shotRpsTrim = 0, shotHoodTrim = 0;
        boolean shotLutMode = false;
        int shotBallsLoaded = 0; // Intake.getBallCount() at LB (beam breaks; 0 if they aren't publishing)
        String pendingLutCols = null; // LUT columns of the shot being fed
        String lastLutRow = null; // paste-ready shooterData row for lastShot
        int madeCount = -1; // balls of lastShot that went in, set with LT + RB / Back; -1 = not marked
        int shotNumber = 0; // n= in event=shotDone / shotMarked, ties a mark to its shot

        for (String line : CONTROLS) {
            telemetry.addLine(line);
        }
        telemetry.addLine("Press PLAY to start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            opModeUtilities.clearBulkCache();
            hiveTarget = SharedData.getTargetPoint();
            double distanceMM = ShooterRun.getDistanceToTargetFromCurrentPos(hiveTarget); // turret pivot to Cell, same call that reads the table
            boolean ltHeld = gamepad1.left_trigger > 0.1;
            boolean rightBumper = kGamePad1.isRightBumperFirstPressed();
            boolean backButton = kGamePad1.isBackButtonFirstPressed();
            // LT + RB / Back: mark how many balls of the last shot went in (made). Without LT they rezero.
            if (ltHeld && (rightBumper || backButton)) {
                if (isPending(shot)) {
                    message = "Shot still running: mark it after it ends";
                } else if (lastShot == null) {
                    message = "No shot to mark: shoot with LB first";
                } else {
                    if (rightBumper) {
                        madeCount = madeCount < 0 ? 1 : Math.min(MAX_MADE, madeCount + 1);
                    } else {
                        madeCount = madeCount < 0 ? 0 : Math.max(0, madeCount - 1);
                    }
                    message = "Marked last shot: " + madeCount + " made (Y to save)";
                    int m = madeCount;
                    int n = shotNumber;
                    String row = lastShot + "," + shotBallsLoaded + "," + madeCount; // the latest shotMarked per n wins
                    KLog.d(TAG, () -> "event=shotMarked n=" + n + " made=" + m + " row=" + row);
                    shotLogWriter.writeLine(n + ",marked," + (lastShooterTimedOut ? 1 : 0) + "," + row + ",\"" + lastLutRow + "\"");
                    flush(shotLogWriter);
                }
            }
            if (!ltHeld && rightBumper) {
                setLastZeroAction(new ResetOdometryToPos(new Position(FieldConfig.CORNER_ZERO_POSITION))); // updateActions() applies it
                message = "Rezeroed to corner " + FieldConfig.CORNER_ZERO_POSITION;
                KLog.d(TAG, () -> "event=rezero from=" + SharedData.getOdometryWheelIMUPosition() + " to=" + FieldConfig.CORNER_ZERO_POSITION);
            }
            if (!ltHeld && backButton) {
                setLastZeroAction(new ResetOdometryToPos(new Position(FieldConfig.FAR_CORNER_ZERO_POSITION)));
                message = "Rezeroed to far corner " + FieldConfig.FAR_CORNER_ZERO_POSITION;
                KLog.d(TAG, () -> "event=rezero from=" + SharedData.getOdometryWheelIMUPosition() + " to=" + FieldConfig.FAR_CORNER_ZERO_POSITION);
            }
            // Read each once per loop: FirstPressed clears itself. LT held = the dpad trims the turret (dpad up toggles feed comp), A/B step the hood finely and Y/X step the transfer RPS; their normal jobs are off.
            boolean trimTurret = ltHeld;
            boolean dpadUp = kGamePad1.isDpadUpFirstPressed();
            boolean dpadDown = kGamePad1.isDpadDownFirstPressed();
            boolean dpadLeft = kGamePad1.isDpadLeftFirstPressed();
            boolean dpadRight = kGamePad1.isDpadRightFirstPressed();
            boolean buttonA = kGamePad1.isButtonAFirstPressed();
            boolean buttonB = kGamePad1.isButtonBFirstPressed();
            boolean buttonX = kGamePad1.isButtonXFirstPressed();
            boolean buttonY = kGamePad1.isButtonYFirstPressed();
            boolean toggleLut = kGamePad1.isRightStickButtonFirstPressed();
            boolean leftStickClick = kGamePad1.isLeftStickButtonFirstPressed();
            boolean resetTrim = leftStickClick && !ltHeld; // LT + left stick click toggles the flower instead
            // Flower: GP1 LT + left stick click toggles down/up; GP2 left stick like TeleOp (up = down, down = up).
            // Interlock (same as TeleOp): down only while the flower's intake side is down (hold RT to lower the intake).
            flowerDown = flower.getServo().getTargetPosition() == FlowerMech.DOWN_POS; // follows an intake raise that lifted it
            double flowerStick = -gamepad2.left_stick_y;
            Boolean flowerWantDown = null;
            if (ltHeld && leftStickClick) {
                flowerWantDown = !flowerDown;
            } else if (flowerStick > FLOWER_STICK_DEADBAND) {
                flowerWantDown = true;
            } else if (flowerStick < -FLOWER_STICK_DEADBAND) {
                flowerWantDown = false;
            }
            if (flowerWantDown != null && flowerWantDown != flowerDown) {
                if (flowerWantDown && !intake.getModule(ModuleConfig.FLOWER_SIDE).isDown()) {
                    if (ltHeld && leftStickClick) {
                        message = "Flower not lowered: intake side " + ModuleConfig.FLOWER_SIDE + " is not down (hold RT first)";
                        KLog.d(TAG, "event=flowerRefused reason=intakeSideNotDown");
                    }
                } else {
                    flowerDown = flowerWantDown;
                    flower.getServo().setTargetPosition(flowerDown ? FlowerMech.DOWN_POS : FlowerMech.UP_POS);
                    boolean d = flowerDown;
                    KLog.d(TAG, () -> "event=flowerSet down=" + d + " pos=" + (d ? FlowerMech.DOWN_POS : FlowerMech.UP_POS));
                }
            }
            IShooterPredictor.ShooterParams lut = shooter.getPrediction(distanceMM); // same table + offsets TeleOp uses
            if (toggleLut) {
                lutMode = !lutMode;
                if (lutMode) {
                    rpsTrim = 0;
                    hoodTrim = 0;
                    message = "LUT mode ON: tracking the table, trims 0";
                } else {
                    message = "LUT mode off: manual, holding " + String.format("%.2f RPS / %.4f hood", targetRPS, hoodPosition);
                }
                boolean on = lutMode;
                KLog.d(TAG, () -> String.format("event=lutModeToggle on=%b distanceMm=%.0f lutRps=%.2f lutHood=%.4f",
                        on, distanceMM, lut.shooterRPS, lut.hoodPosition));
            }
            double offsetBefore = TurretConfig.TURRET_OFFSET_DEG;
            if (trimTurret && dpadLeft) {
                TurretConfig.TURRET_OFFSET_DEG -= TURRET_OFFSET_STEP_DEG;
            }
            if (trimTurret && dpadRight) {
                TurretConfig.TURRET_OFFSET_DEG += TURRET_OFFSET_STEP_DEG;
            }
            if (trimTurret && dpadDown) {
                TurretConfig.TURRET_OFFSET_DEG = 0;
            }
            if (TurretConfig.TURRET_OFFSET_DEG != offsetBefore) {
                KLog.d(TAG, () -> "event=turretOffsetSet offsetDeg=" + TurretConfig.TURRET_OFFSET_DEG);
            }
            if (trimTurret && (buttonX || buttonY)) {
                // floor at one step: <= 0 would drop PushBall back to open loop
                IntakeConfig.FEED_RPS = Math.max(TRANSFER_RPS_STEP,
                        IntakeConfig.FEED_RPS + (buttonY ? TRANSFER_RPS_STEP : -TRANSFER_RPS_STEP));
                KLog.d(TAG, () -> "event=transferRpsSet rps=" + IntakeConfig.FEED_RPS);
            }
            if (trimTurret && dpadUp) {
                IntakeConfig.FEED_RPS_COMP_ON = !IntakeConfig.FEED_RPS_COMP_ON;
                KLog.d(TAG, () -> "event=feedRpsCompSet on=" + IntakeConfig.FEED_RPS_COMP_ON);
            }
            driveAction.move(gamepad1);
            updateActions();

            long nowMs = System.currentTimeMillis();
            if (nowMs - lastTurretLogMs >= TURRET_LOG_MS) {
                lastTurretLogMs = nowMs;
                Position pose = SharedData.getOdometryWheelIMUPosition();
                Point launcher = Turret.launcherPoint(pose);
                Point target = turretAutoAlignTeleOp.getTargetPoint();
                double fieldAimDeg = Math.toDegrees(Math.atan2(target.getY() - launcher.getY(), target.getX() - launcher.getX()));
                KLog.d(TAG, () -> String.format("event=turret x=%.0f y=%.0f headingDeg=%.1f target=(%.0f,%.0f) fieldAimDeg=%.1f wantDeg=%.1f turretDeg=%.1f servoPos=%.4f clamped=%b moving=%b aligned=%b",
                        pose.getX(), pose.getY(), Math.toDegrees(pose.getTheta()), target.getX(), target.getY(), fieldAimDeg,
                        Math.toDegrees(turretAutoAlignTeleOp.getTargetAngleRad()), turret.getAngleDeg(),
                        Turret.servoPosFromTurretDeg(turret.getAngleDeg()), turret.isClamped(), turret.isMoving(),
                        turretAutoAlignTeleOp.isWithinRange()));
            }

            if (lutMode) {
                double rpsTrimBefore = rpsTrim;
                double hoodTrimBefore = hoodTrim;
                if (!trimTurret && dpadUp) {
                    rpsTrim += LUT_RPS_STEP;
                }
                if (!trimTurret && dpadDown) {
                    rpsTrim -= LUT_RPS_STEP;
                }
                if (!trimTurret && buttonA) {
                    rpsTrim += LUT_RPS_STEP_FINE;
                }
                if (!trimTurret && buttonB) {
                    rpsTrim -= LUT_RPS_STEP_FINE;
                }
                if (!trimTurret && dpadRight) {
                    hoodTrim += LUT_HOOD_STEP;
                }
                if (!trimTurret && dpadLeft) {
                    hoodTrim -= LUT_HOOD_STEP;
                }
                if (trimTurret && buttonA) {
                    hoodTrim += LUT_HOOD_STEP_FINE;
                }
                if (trimTurret && buttonB) {
                    hoodTrim -= LUT_HOOD_STEP_FINE;
                }
                if (resetTrim) {
                    rpsTrim = 0;
                    hoodTrim = 0;
                }
                // round away float drift so 0.1 steps read as 0.1 (4 dp for hood steps of 0.0005)
                rpsTrim = Math.round(rpsTrim * 100) / 100.0;
                hoodTrim = Math.round(hoodTrim * 10000) / 10000.0;
                targetRPS = MathFunctions.clamp(lut.shooterRPS + rpsTrim, 0, ShooterInterpolationConfig.MAX_RPS);
                hoodPosition = MathFunctions.clamp(lut.hoodPosition + hoodTrim, ShooterInterpolationConfig.MIN_HOOD, ShooterInterpolationConfig.MAX_HOOD);
                if (rpsTrim != rpsTrimBefore || hoodTrim != hoodTrimBefore) {
                    double rt = rpsTrim, ht = hoodTrim, r = targetRPS, h = hoodPosition;
                    KLog.d(TAG, () -> String.format("event=lutTrimSet distanceMm=%.0f lutRps=%.2f rpsTrim=%.2f targetRps=%.2f lutHood=%.4f hoodTrim=%.4f hood=%.4f",
                            distanceMM, lut.shooterRPS, rt, r, lut.hoodPosition, ht, h));
                }
            } else {
                double rpsBefore = targetRPS;
                if (!trimTurret && dpadUp) {
                    targetRPS += RPS_STEP_BIG;
                }
                if (!trimTurret && dpadDown) {
                    targetRPS -= RPS_STEP_BIG;
                }
                if (!trimTurret && buttonA) {
                    targetRPS += RPS_STEP_SMALL;
                }
                if (!trimTurret && buttonB) {
                    targetRPS -= RPS_STEP_SMALL;
                }
                targetRPS = MathFunctions.clamp(targetRPS, 0, ShooterInterpolationConfig.MAX_RPS);
                if (targetRPS != rpsBefore) {
                    double r = targetRPS;
                    KLog.d(TAG, () -> "event=targetRpsSet targetRps=" + r);
                }
            }
            if (!trimTurret && buttonX) {
                shooterOn = !shooterOn;
                if (!shooterOn && isPending(shot)) {
                    shot = cancelShot(shot, stopper, intake);
                    message = "Shot cancelled: shooter off";
                }
                boolean on = shooterOn;
                KLog.d(TAG, () -> "event=shooterToggle on=" + on);
            }

            if (!lutMode) { // LUT mode set the hood above
                double hoodBefore = hoodPosition;
                if (!trimTurret && dpadRight) {
                    hoodPosition += HOOD_STEP;
                }
                if (!trimTurret && dpadLeft) {
                    hoodPosition -= HOOD_STEP;
                }
                if (trimTurret && buttonA) {
                    hoodPosition += HOOD_STEP_FINE;
                }
                if (trimTurret && buttonB) {
                    hoodPosition -= HOOD_STEP_FINE;
                }
                hoodPosition = MathFunctions.clamp(hoodPosition, ShooterInterpolationConfig.MIN_HOOD, ShooterInterpolationConfig.MAX_HOOD);
                if (hoodPosition != hoodBefore) {
                    double h = hoodPosition;
                    KLog.d(TAG, () -> "event=hoodSet hood=" + h);
                }
            }
            if (!shooterOn) {
                shooter.getHood().setPosition(hoodPosition); // when on, ShooterRun drives the hood (with its RPS compensation)
            }

            boolean shooting = isPending(shot);
            // Record the row the moment the stopper starts opening: shooter, turret and transfer were all ready.
            if (shooting && pendingShot == null && shot.getOpenStopper().getHasStarted()) {
                Point aim = TurretAutoAlign.offsetTargetPoint(hiveTarget, SharedData.getOdometryWheelIMUPosition(), TurretConfig.TURRET_OFFSET_DEG);
                pendingShot = String.format("%.2f,%.3f,%.2f,%.4f,%.2f,%.1f,%.1f,%.1f,%.1f,%.1f,%.2f,%.2f,%d",
                        shooter.getRPS(), shooter.getShooterMotor().getPower(), getBatteryVoltage(), shotHood, distanceMM,
                        hiveTarget.getX(), hiveTarget.getY(), TurretConfig.TURRET_OFFSET_DEG, aim.getX(), aim.getY(), IntakeConfig.FEED_RPS, shotTargetRPS,
                        IntakeConfig.FEED_RPS_COMP_ON ? 1 : 0);
                Position pose = SharedData.getOdometryWheelIMUPosition(); // pose + turret: separate odometry drift from turret calibration on aim misses
                pendingLutCols = String.format("%d,%.2f,%.4f,%.2f,%.4f,%.0f,%.0f,%.1f,%.1f", shotLutMode ? 1 : 0, shotLutRPS, shotLutHood, shotRpsTrim, shotHoodTrim,
                        pose.getX(), pose.getY(), Math.toDegrees(pose.getTheta()), turretAutoAlignTeleOp.getTurret().getAngleDeg());
                // shooterData row that reproduces this shot: Shooter.getPrediction adds these offsets back on
                lastLutRow = String.format("%.0f, %.2f, %.4f, FEED_RPS,", distanceMM,
                        shotTargetRPS - ShooterInterpolationConfig.rpsOffset,
                        shotHood - ShooterInterpolationConfig.hoodOffset - ShooterInterpolationConfig.HOOD_INIT_OFFSET);
                String s = pendingShot + " lut=" + pendingLutCols;
                KLog.d(TAG, () -> "event=shotRecorded row=" + s);
            }
            if (wasShooting && !shooting) {
                if (pendingShot == null) {
                    message = "Not fed: shooter or turret not ready in time (ShootAllAction abort), nothing recorded";
                    KLog.d(TAG, "event=shotAborted");
                } else {
                    boolean transferTimedOut = shot.intakeTimedOut();
                    lastShot = pendingShot + "," + (transferTimedOut ? 1 : 0) + "," + pendingLutCols;
                    pendingShot = null;
                    pendingLutCols = null;
                    lastTransferGate = transferTimedOut ? "TIMED OUT: fed with the transfer out of tolerance (suspect row)" : "ok, in tolerance";
                    shotNumber++;
                    // Every shot goes to KLog, no Y needed: same columns as the CSV, Made=-1 until marked (event=shotMarked)
                    String row = lastShot + "," + shotBallsLoaded + ",-1";
                    int n = shotNumber;
                    String lr = lastLutRow;
                    lastShooterTimedOut = shot.getShooterReady().timedOut(); // fired after SHOOTER_READY_TIMEOUT_MS: suspect row
                    boolean sto = lastShooterTimedOut;
                    KLog.d(TAG, () -> "event=shotDone n=" + n + " shooterTimedOut=" + sto + " transferTimedOut=" + transferTimedOut + " row=" + row + " lutRow=" + lr);
                    shotLogWriter.writeLine(n + ",done," + (sto ? 1 : 0) + "," + row + ",\"" + lr + "\"");
                    flush(shotLogWriter);
                }
                turretAutoAlignTeleOp.setTurretRunMode(TurretRunMode.RUN_USING_ODOMETRY); // ShootAllAction stops it once the stopper opens
            }
            wasShooting = shooting;

            if (kGamePad1.isLeftBumperFirstPressed()) {
                if (shooting) {
                    shot = cancelShot(shot, stopper, intake);
                    shooting = false;
                    wasShooting = false;
                    message = "Shot cancelled";
                } else if (!shooterOn) {
                    message = "Shooter is off: press X first";
                } else {
                    shot = new ShootAllAction(turret, stopper, intake, shooter, rpsLoop, turretAutoAlignTeleOp, targetRPS, hoodPosition);
                    shot.setEnableLimelightZeroing(false);
                    shot.setName("collectorShot");
                    setLastShooterAction(shot); // updateActions() ticks it; it owns the intake + stopper until done
                    shotTargetRPS = targetRPS;
                    shotHood = hoodPosition;
                    shotLutMode = lutMode;
                    shotLutRPS = lut.shooterRPS;
                    shotLutHood = lut.hoodPosition;
                    shotRpsTrim = lutMode ? rpsTrim : 0;
                    shotHoodTrim = lutMode ? hoodTrim : 0;
                    lastShot = null; // this shot's row is complete once it ends
                    lastLutRow = null;
                    madeCount = -1;
                    shotBallsLoaded = intake.getBallCount();
                    pendingShot = null;
                    pendingLutCols = null;
                    shooting = true;
                    wasShooting = true;
                    message = "";
                }
                boolean on = shooting;
                KLog.d(TAG, () -> "event=shotButton shooting=" + on);
            }

            if (!shooting) {
                if (gamepad1.right_trigger > 0.1) {
                    intake.lower();
                    if (false/*IntakeConfig.FEED_RPS > 0*/) {
                        // same closed loop and target as PushBall's feed, so LT + Y/X changes this too
                        intake.getModule(IntakeSide.FRONT).runRPS(IntakeConfig.FEED_RPS);
                        intake.getModule(IntakeSide.BACK).runRPS(IntakeConfig.FEED_RPS);
                    } else {
                        intake.run(1.0); // closed loop off in IntakeConfig
                    }
                } else {
                    intake.stop();
                }
            }

            pidf.setKp(ShooterConfig.kp); // every loop: Dashboard edits apply live
            pidf.setKf(ShooterConfig.kf);
            if (!shooterOn) {
                if (rpsLoop.getShooterRunMode() != ShooterRunMode.STOP) {
                    rpsLoop.stop();
                }
            } else if (!shooting) { // a running ShootAllAction sets the mode itself (and leaves CURRENT_POINT when done)
                rpsLoop.setTargetRPS(targetRPS);
                rpsLoop.setTargetHoodPosition(hoodPosition);
                if (rpsLoop.getShooterRunMode() != ShooterRunMode.SHOOT_USING_TARGET_RPS_HOOD) {
                    rpsLoop.setShooterRunMode(ShooterRunMode.SHOOT_USING_TARGET_RPS_HOOD);
                }
            }

            if (nowMs - lastShooterLogMs >= SHOOTER_LOG_MS) {
                lastShooterLogMs = nowMs;
                boolean on = shooterOn;
                boolean shootingNow = shooting;
                double h = hoodPosition;
                double tr = targetRPS;
                boolean lm = lutMode;
                double rt = rpsTrim, ht = hoodTrim;
                KLog.d(TAG, () -> String.format("event=shooter rps=%.2f targetRps=%.2f on=%b power=%.3f voltage=%.2f hood=%.4f distanceMm=%.0f shooterInRange=%b aligned=%b shooting=%b rpsA=%.2f rpsB=%.2f balls=%d lutMode=%b lutRps=%.2f lutHood=%.4f rpsTrim=%.2f hoodTrim=%.4f",
                        shooter.getRPS(), tr, on, shooter.getShooterMotor().getPower(), SharedData.getVoltage(), h, distanceMM,
                        rpsLoop.isWithinRange(), turretAutoAlignTeleOp.isWithinRange(), shootingNow,
                        intake.getModule(IntakeSide.FRONT).getRPS(), intake.getModule(IntakeSide.BACK).getRPS(), intake.getBallCount(),
                        lm, lut.shooterRPS, lut.hoodPosition, rt, ht));
            }

            if (!trimTurret && buttonY) {
                if (shooting) {
                    message = "Shot still running: press Y after it ends";
                    KLog.d(TAG, "event=saveRefused reason=shooting");
                } else if (lastShot != null) {
                    String row = lastShot + "," + shotBallsLoaded + "," + madeCount;
                    String lutLine = lastLutRow + " // made " + (madeCount < 0 ? "?" : String.valueOf(madeCount))
                            + (shotBallsLoaded > 0 ? "/" + shotBallsLoaded : "");
                    fileWriter.writeLine(row);
                    flush(fileWriter);
                    lutRowWriter.writeLine(lutLine);
                    flush(lutRowWriter);
                    message = (madeCount < 0 ? "SAVED UNMARKED (Made=-1): " : "SAVED: ") + row + "\nLUT row: " + lutLine;
                    String s = row;
                    String lr = lutLine;
                    KLog.d(TAG, () -> "event=saved row=" + s);
                    KLog.d(TAG, () -> "event=lutRow row=" + lr);
                } else {
                    message = "Nothing to save: shoot with LB first";
                    KLog.d(TAG, "event=saveRefused reason=noShot");
                }
            }

            for (String line : lutMode ? LUT_CONTROLS : CONTROLS) {
                telemetry.addLine(line);
            }
            if (!message.isEmpty()) {
                telemetry.addLine(message);
            }
            telemetry.addData("Mode", lutMode ? "LUT TUNE" : "manual");
            telemetry.addData("LUT @ distance", "%.0f mm -> %.2f RPS / %.4f hood", distanceMM, lut.shooterRPS, lut.hoodPosition);
            if (lutMode) {
                telemetry.addData("Trim RPS / hood", "%+.2f / %+.4f", rpsTrim, hoodTrim);
                telemetry.addData("Target (LUT + trim)", "%.2f RPS / %.4f hood", targetRPS, hoodPosition);
            }
            telemetry.addData("Last LUT row (saved with Y)", lastLutRow == null ? "none" : lastLutRow);
            telemetry.addData("Last shot made (LT+RB +1 / LT+Back -1)", lastShot == null ? "-" :
                    (madeCount < 0 ? "NOT MARKED" : String.valueOf(madeCount)) + (shotBallsLoaded > 0 ? " of " + shotBallsLoaded + " loaded" : ""));
            if (!shooting) {
                telemetry.addData("Shot", "idle");
            } else if (shot.getOpenStopper().getHasStarted()) {
                telemetry.addData("Shot", "FEEDING");
            } else {
                telemetry.addData("Shot", "waiting: shooter %s  turret %s  transfer %s",
                        shot.getShooterReady().getIsDone() ? "READY" : "no", turretAutoAlignTeleOp.isWithinRange() ? "READY" : "no",
                        shot.getIntakeReady().getIsDone() ? (shot.intakeTimedOut() ? "TIMED OUT" : "READY") : "no");
            }
            telemetry.addData("Shooter in range (ShooterRun)", "%s  error %.2f RPS, tol %.2f, hold %.0f ms",
                    rpsLoop.isWithinRange() ? "IN TOL" : "out", targetRPS - shooter.getRPS(), rpsLoop.calculateRPSTolerance(), ShooterConfig.timeToStabilize);
            telemetry.addData("Shooter on / target RPS / power", "%b / %.1f / %.3f", shooterOn, targetRPS, shooter.getShooterMotor().getPower());
            telemetry.addData("kF / kP", "%.5f / %.4f", ShooterConfig.kf, ShooterConfig.kp);
            telemetry.addData("Transfer set RPS / tol", "%.1f / %.1f   timeout %.0f ms", IntakeConfig.FEED_RPS,
                    IntakeConfig.FEED_RPS_TOLERANCE, IntakeConfig.FEED_GATE_TIMEOUT_MS);
            telemetry.addData("Transfer in tol A / B", "%s / %s",
                    intake.getModule(IntakeSide.FRONT).isAtRPS(IntakeConfig.FEED_RPS, IntakeConfig.FEED_RPS_TOLERANCE) ? "IN TOL" : "out",
                    intake.getModule(IntakeSide.BACK).isAtRPS(IntakeConfig.FEED_RPS, IntakeConfig.FEED_RPS_TOLERANCE) ? "IN TOL" : "out");
            telemetry.addData("Last shot transfer gate", lastTransferGate);
            IntakeModule transferA = intake.getModule(IntakeSide.FRONT);
            IntakeModule transferB = intake.getModule(IntakeSide.BACK);
            telemetry.addData("Transfer actual RPS A / B", "%.2f / %.2f", transferA.getRPS(), transferB.getRPS());
            telemetry.addData("Transfer power A / B", "%.2f / %.2f%s", transferA.getPower(), transferB.getPower(),
                    Math.max(transferA.getPower(), transferB.getPower()) >= 0.99 ? "  MAXED: target above top speed, lower it" : "");
            telemetry.addData("Balls", intake.getBallCount());
            telemetry.addData("Flower", "%s  pos %.3f  (LT + L-stick click / GP2 left stick)", flowerDown ? "DOWN" : "UP",
                    flower.getServo().getPosition());
            telemetry.addData("Hood Position", "%.4f", hoodPosition);
            telemetry.addData("Current RPS", "%.2f", shooter.getRPS());
            telemetry.addData("Feed RPS comp (IntakeConfig)", "%s, %.2f transfer RPS per shooter RPS", IntakeConfig.FEED_RPS_COMP_ON ? "ON" : "off", IntakeConfig.FEED_RPS_COMP_GAIN);
            telemetry.addData("Turret target / delta", "%.1f / %.1f deg",
                    Math.toDegrees(turretAutoAlignTeleOp.getTargetAngleRad()), turretAutoAlignTeleOp.getDeltaAngleDeg());
            telemetry.addData("Turret offset", "%.1f deg", TurretConfig.TURRET_OFFSET_DEG);
            telemetry.addData("Turret cmd / servo / clamped", "%.1f deg / %.4f / %b",
                    turret.getAngleDeg(), Turret.servoPosFromTurretDeg(turret.getAngleDeg()), turret.isClamped());
            telemetry.addData("Odometry pose", SharedData.getOdometryWheelIMUPosition());
            telemetry.addData("Target Cell", "%s (%.0f, %.0f)", hiveTarget == ShooterConfig.TARGET_POINT_1 ? "A (high y)" : "B (low y)",
                    hiveTarget.getX(), hiveTarget.getY());
            telemetry.addData("Turret to target (odometry)", "%.0f mm", distanceMM);
            telemetry.addData("Last shot (not saved until Y)", shooting ? "shooting..." : lastShot == null ? "none" : lastShot);
            telemetry.update();
        }

        TurretConfig.TURRET_OFFSET_DEG = 0; // static: don't leak the trim (or the feed target / comp) into TeleOp
        IntakeConfig.FEED_RPS = feedRpsAtStart;
        IntakeConfig.FEED_RPS_COMP_ON = feedCompAtStart;
        if (isPending(shot)) {
            cancelShot(shot, stopper, intake);
        }
        rpsLoop.stop();
        shooterRun = null;
        shooter.stop();
        intake.stop();
        driveTrain.setPower(0);
        fileWriter.close();
        lutRowWriter.close();
        shotLogWriter.close();
        KLog.d(TAG, "event=done file=RobotLogs/ pull: adb pull /sdcard/Android/data/com.qualcomm.ftcrobotcontroller/files/RobotLogs/");
        cleanupRobot();
    }

    /** Like TeleOp's stop, plus stopper closed and intake stopped: a cancel can land mid-feed. Returns null. */
    private ShootAllAction cancelShot(ShootAllAction shot, Stopper stopper, Intake intake) {
        shot.setIsDone(true);
        setLastShooterAction(null);
        stopper.setPosition(ModuleConfig.STOPPER_SERVO_CLOSED_POS);
        intake.stop();
        turretAutoAlignTeleOp.setTurretRunMode(TurretRunMode.RUN_USING_ODOMETRY);
        KLog.d(TAG, "event=shotCancelled");
        return null;
    }

    private double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    private static void flush(KFileWriter writer) {
        try {
            writer.flush();
        } catch (java.io.IOException e) {
            KLog.e(TAG, "event=flushFailed", e);
        }
    }
}
