package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter;

import static org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.ShooterRunMode.SHOOT_USING_CURRENT_POINT;
import static org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.ShooterRunMode.SHOOT_USING_TARGET_RPS_HOOD;

import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.SOTM;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KActionSet;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.WaitAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeRun;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeStop;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeReady;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.stopper.CloseStopperAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.stopper.OpenStopperAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretAutoAlign;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretReady;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretRunMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.ResetOdometryToLimelight;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.IntakeRunMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Turret;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.Shooter;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.ShooterRunMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;


public class ShootAllAction extends KActionSet {
    public static final String TAG = "Shooting_ShootAction";

    private final Turret turret;
    private final Stopper stopper;
    private final Intake intake;
    private final Shooter shooter;
    private final ShooterRun shooterRun;
    private final TurretAutoAlign turretAutoAlign;

    private ShooterReady shooterReady;
    private ShooterRunMode shooterRunMode;

    private TurretReady turretReady;

    private IntakeRun intakeRun;
    private IntakeReady intakeReady;

    private OpenStopperAction openStopper;
    private WaitAction shootTimeWait;
    private CloseStopperAction closeStopper;
    private IntakeStop intakeStop;

    private ResetOdometryToLimelight resetOdometryToPosition;

    // Turret mode when the shot started: the turret is stopped during the feed and put back to this once the
    // stopper closes (or the shot is cancelled), so callers that never turn it back on (autos) keep aiming.
    private TurretRunMode turretModeBeforeShot = null;
    private boolean turretRestored = false;

    // Hive Cell this shot aims at (e.g. ShooterConfig.TARGET_POINT_1 = Cell A, TARGET_POINT_2 = Cell B); null = leave
    // the shooter's / turret's targets as they are.
    private Point targetPoint = null;

    public void setEnableLimelightZeroing(boolean enableLimelightZeroing) {
        this.enableLimelightZeroing = enableLimelightZeroing;
    }

    private boolean enableLimelightZeroing = false; // TeleOp's auto-rezero toggle sets this; autos never rezero


    public ShootAllAction(Turret turret, Stopper stopper, Intake intake, Shooter shooter, ShooterRun shooterRun, TurretAutoAlign turretAutoAlign, double targetRPS, double targetHoodPos) {
        this.stopper = stopper;
        this.intake = intake;
        this.shooter = shooter;
        this.shooterRun = shooterRun;
        this.turretAutoAlign = turretAutoAlign;
        this.turret = turret;

        shooterRunMode = SHOOT_USING_TARGET_RPS_HOOD;
        shooterRun.setTargetRPS(targetRPS);
        shooterRun.setTargetHoodPosition(targetHoodPos);

        // Fixed shooter RPS (pass / data collection): no distance to go by, so the intake runs at IntakeConfig.FEED_RPS.
        generateBasicAction(shooterRun, stopper, intake, turretAutoAlign, new IntakeRun(intake, IntakeConfig.FEED_RPS));
    }

    public ShootAllAction(Turret turret, Stopper stopper, Intake intake, Shooter shooter, ShooterRun shooterRun, TurretAutoAlign turretAutoAlign) {
        this.stopper = stopper;
        this.intake = intake;
        this.shooter = shooter;
        this.shooterRun = shooterRun;
        this.turretAutoAlign = turretAutoAlign;
        this.turret = turret;

        shooterRunMode = SHOOT_USING_CURRENT_POINT;

        // Intake RPS from the distance to the shooter's goal (Intake.getIntakeRPSPrediction); FEED_RPS if there is no goal.
        IntakeRun intakeRun = shooterRun.getTargetPoint() != null
                ? new IntakeRun(intake, shooterRun.getTargetPoint())
                : new IntakeRun(intake, IntakeConfig.FEED_RPS);
        generateBasicAction(shooterRun, stopper, intake, turretAutoAlign, intakeRun);

        KLog.d("Intake Target RPS", () -> String.format("%.2f", this.intakeRun.getTargetRPS()));
    }

    /**
     * Picks the Hive Cell this shot aims at (ShooterConfig.TARGET_POINT_1 = Cell A, TARGET_POINT_2 = Cell B).
     * When the shot starts, the shared ShooterRun (RPS / hood), the TurretAutoAlign and this shot's IntakeRun all
     * switch to it, and they keep it after the shot until another shot picks a different one.
     * Not set (null) = the shooter keeps its own target and the turret keeps whatever it was following.
     */
    public void setTargetPoint(Point targetPoint) {
        this.targetPoint = targetPoint;
    }

    public Point getTargetPoint() {
        return targetPoint;
    }

    public ShooterRun getShooterRun() {
        return shooterRun;
    }

    public ShooterReady getShooterReady() {
        return shooterReady;
    }

    public IntakeRun getIntakeRun() {
        return intakeRun;
    }

    public IntakeReady getIntakeReady() {
        return intakeReady;
    }

    /** Started = the feed has begun (shooter, turret and intake all ready). */
    public OpenStopperAction getOpenStopper() {
        return openStopper;
    }

    /** The intake never got in tolerance and the shot went on after IntakeConfig.FEED_GATE_TIMEOUT_MS. */
    public boolean intakeTimedOut() {
        return intakeReady.timedOut();
    }

    /** Cancel / abort: IntakeRun keeps the intake spinning until IntakeStop, so stop it here if IntakeStop never ran. */
    @Override
    public void setIsDone(boolean isDone) {
        if (isDone && !this.isDone && !intakeStop.getIsDone()) {
            intakeRun.stop();
        }
        if (isDone && !this.isDone) {
            restoreTurretMode();
        }
        super.setIsDone(isDone);
    }

    public void setShooterReadyTimeoutMs(double ms) {
        shooterReady.setTimeOutMs(ms);
    }



    private void generateBasicAction(ShooterRun shooterRun, Stopper stopper, Intake intake, TurretAutoAlign turretAutoAlign, IntakeRun intakeRun) {

        shooterReady = new ShooterReady(shooterRun);
        shooterReady.setName("shooterReady");
        this.addAction(shooterReady);

        turretReady = new TurretReady(turretAutoAlign);
        turretReady.setName("turretReady");
        this.addAction(turretReady);


        // Spins the intake from the start, against the closed stopper; keeps running through the feed until intakeStop.
        this.intakeRun = intakeRun;
        intakeRun.setName("intakeRun");
        this.addAction(intakeRun);

        // Added after intakeRun so it sees this loop's RPS check. Done when either side is at intakeRun's target RPS.
        intakeReady = new IntakeReady(intakeRun);
        intakeReady.setName("intakeReady");
        this.addAction(intakeReady);

        openStopper = new OpenStopperAction(stopper);
        openStopper.setName("openStopper");
        openStopper.setDependentActions(shooterReady, turretReady, intakeReady);
        this.addAction(openStopper);

        // Same dependencies as openStopper so it starts the same loop the stopper starts opening: the feed time.
        shootTimeWait = new WaitAction(IntakeConfig.shootTimeMS);
        shootTimeWait.setName("shootTimeWait");
        shootTimeWait.setDependentActions(openStopper);
        this.addAction(shootTimeWait);

        closeStopper = new CloseStopperAction(stopper);
        closeStopper.setName("closeStopper");
        closeStopper.setDependentActions(shootTimeWait);
        this.addAction(closeStopper);

        intakeStop = new IntakeStop(intakeRun);
        intakeStop.setName("intakeStop");
        intakeStop.setDependentActions(shootTimeWait);
        this.addAction(intakeStop);

        // Re-zero x/y to the Limelight once the balls are gone (turret still). Off unless TeleOp turns it on; autos never do.
        resetOdometryToPosition = new ResetOdometryToLimelight(turret);
        resetOdometryToPosition.setName("resetOdometryToPosition");
        resetOdometryToPosition.setDependentActions(closeStopper, intakeStop);
        this.addAction(resetOdometryToPosition);
    }

    @Override
    public void beforeUpdate() {
        if (isDone) {
            return;
        }
        if (!hasStarted) {
            if (targetPoint != null) {
                shooterRun.setTargetPoint(targetPoint);
                turretAutoAlign.setTargetPoint(targetPoint);
                if (intakeRun.getIntakeRunMode() == IntakeRunMode.RUN_USING_CURRENT_POINT) {
                    intakeRun.setTargetPoint(targetPoint); // fixed-RPS shots keep FEED_RPS
                }
                KLog.d(TAG, () -> "event=target name=" + getName() + " target=" + targetPoint);
            }
            //Refresh targetRPS
            shooterRun.setShooterRunMode(shooterRunMode);
            shooterRun.update();
            hasStarted = true;
            turretModeBeforeShot = turretAutoAlign.getTurretRunMode();
            stopper.close(); // IntakeRun spins the intake from the start: an open stopper would feed early
            if (!enableLimelightZeroing) {
                resetOdometryToPosition.setIsDone(true);
            }
        }

        KLog.d(TAG, () -> "event=status shooterReady=" + shooterReady.getIsDone() + " turretReady=" + turretReady.getIsDone()
                + " intakeReady=" + intakeReady.getIsDone() + " stopperOpened=" + openStopper.getHasStarted()
                + " stopperClosed=" + closeStopper.getIsDone() + " intakeStopped=" + intakeStop.getIsDone());
    }

    @Override
    public void afterUpdate() {
        // Once only: setShooterRunMode also runs update(), so calling it every loop doubles the shooter update.
        if (closeStopper.getIsDone() && shooterRun.getShooterRunMode() != SHOOT_USING_CURRENT_POINT) {
//            double maintainRPSValue = shooterRun.getTargetRPS() * 0.925;
//            shooterRun.setTargetRPS(maintainRPSValue);
            shooterRun.setShooterRunMode(SHOOT_USING_CURRENT_POINT);
            //KLog.d("ShooterRun", "Maintaining " + maintainRPSValue + " RPS after Running ShootAllAction");
        }
        KLog.d(TAG, () -> String.format("event=update stopperOpen=%s turretDeltaDeg=%.2f turretWithinRange=%s deltaRps=%.2f",
                (openStopper.getIsDone()),
                (turretAutoAlign.getDeltaAngleDeg()),
                (turretAutoAlign.isWithinRange()),
                (shooterRun.getTargetRPS() - shooter.getRPS())));
        if (openStopper.getIsDone() && !closeStopper.getIsDone()) {
            if (!SOTM.isActive()) {
                turretAutoAlign.stop(); // hold still while the balls feed
            }
        }
        if (closeStopper.getIsDone()) {
            restoreTurretMode();
        }
    }

    /** Once per shot: put the turret back in the mode it had before the shot (stop() above left it in STOP). */
    private void restoreTurretMode() {
        if (turretRestored || turretModeBeforeShot == null) {
            return;
        }
        turretRestored = true;
        turretAutoAlign.setTurretRunMode(turretModeBeforeShot);
        KLog.d(TAG, () -> "event=turretRestored mode=" + turretModeBeforeShot);
    }

    public void setTurretReady(boolean isDone) {
        turretReady.setIsDone(isDone);
    }



}
