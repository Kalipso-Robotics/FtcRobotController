package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter;

import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.SOTM;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KActionSet;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeStop;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeReady;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.pusher.PushBall;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretAutoAlign;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretReady;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.localization.ResetOdometryToLimelight;
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

    private PushBall pushBall;

    private TurretReady turretReady;

    private IntakeReady intakeReady;

    private ResetOdometryToLimelight resetOdometryToPosition;
    private IntakeStop intakeStop;

    public void setEnableLimelightZeroing(boolean enableLimelightZeroing) {
        this.enableLimelightZeroing = enableLimelightZeroing;
    }

    private boolean enableLimelightZeroing = ShooterConfig.REZERO_AFTER_EACH_SHOT;
    private final boolean abortOnTimeout;

    public ShootAllAction(Turret turret, Stopper stopper, Intake intake, Shooter shooter, ShooterRun shooterRun, TurretAutoAlign turretAutoAlign, double targetRPS, double targetHoodPos) {
        this(turret, stopper, intake, shooter, shooterRun, turretAutoAlign, targetRPS, targetHoodPos, false);
    }

    /** abortOnTimeout: TeleOp gate. If RPS or turret is not ready in TELEOP_READY_TIMEOUT_MS, finish without feeding. */
    public ShootAllAction(Turret turret, Stopper stopper, Intake intake, Shooter shooter, ShooterRun shooterRun, TurretAutoAlign turretAutoAlign, double targetRPS, double targetHoodPos, boolean abortOnTimeout) {
        this.abortOnTimeout = abortOnTimeout;
        this.stopper = stopper;
        this.intake = intake;
        this.shooter = shooter;
        this.shooterRun = shooterRun;
        this.turretAutoAlign = turretAutoAlign;
        this.turret = turret;

        shooterRun.setShooterRunMode(ShooterRunMode.SHOOT_USING_TARGET_RPS_HOOD);
        shooterRun.setTargetRPS(targetRPS);
        shooterRun.setTargetHoodPosition(targetHoodPos);

        generateBasicAction(shooterRun, stopper, intake, turretAutoAlign);
    }

    public ShootAllAction(Turret turret, Stopper stopper, Intake intake, Shooter shooter, ShooterRun shooterRun, TurretAutoAlign turretAutoAlign) {
        this(turret, stopper, intake, shooter, shooterRun, turretAutoAlign, false);
    }

    public ShootAllAction(Turret turret, Stopper stopper, Intake intake, Shooter shooter, ShooterRun shooterRun, TurretAutoAlign turretAutoAlign, boolean abortOnTimeout) {
        this.abortOnTimeout = abortOnTimeout;
        this.stopper = stopper;
        this.intake = intake;
        this.shooter = shooter;
        this.shooterRun = shooterRun;
        this.turretAutoAlign = turretAutoAlign;
        this.turret = turret;

        shooterRun.setShooterRunMode(ShooterRunMode.SHOOT_USING_CURRENT_POINT);


        generateBasicAction(shooterRun, stopper, intake, turretAutoAlign);
    }

    public ShooterRun getShooterRun() {
        return shooterRun;
    }

    public ShooterReady getShooterReady() {
        return shooterReady;
    }

    public PushBall getPushBall() {
        return pushBall;
    }

    public IntakeReady getTransferReady() {
        return intakeReady;
    }

    /** The transfer never got in tolerance and the shot went on after IntakeConfig.FEED_GATE_TIMEOUT_MS. */
    public boolean transferTimedOut() {
        return intakeReady.timedOut();
    }

    /** Cancel / abort before the feed: TransferReady has the transfer spinning and nothing else would stop it. */
    @Override
    public void setIsDone(boolean isDone) {
        if (isDone && !this.isDone && intakeReady.getHasStarted() && !pushBall.getHasStarted()) {
            intake.stop();
        }
        super.setIsDone(isDone);
    }



    private void generateBasicAction(ShooterRun shooterRun, Stopper stopper, Intake intake, TurretAutoAlign turretAutoAlign) {

        shooterReady = abortOnTimeout
                ? new ShooterReady(shooterRun, ShooterConfig.TELEOP_READY_TIMEOUT_MS)
                : new ShooterReady(shooterRun);
        shooterReady.setName("shooterReady");
        this.addAction(shooterReady);

        turretReady = abortOnTimeout
                ? new TurretReady(turretAutoAlign, ShooterConfig.TELEOP_READY_TIMEOUT_MS, true)
                : new TurretReady(turretAutoAlign);
        turretReady.setName("turretReady");
        this.addAction(turretReady);


        // Holds both transfer sides at IntakeConfig.FEED_RPS against the closed stopper; PushBall (which opens it) waits for this.
        intakeReady = new IntakeReady(intake);
        intakeReady.setName("transferReady");
        this.addAction(intakeReady);

        // Shooter error feeds PushBall's IntakeConfig.FEED_RPS_COMP_ON compensation; with it off (default) this is the plain feed.
        pushBall = new PushBall(stopper, intake, () -> shooterRun.getTargetRPS() - shooter.getRPS());
        pushBall.setName("pushAllBalls");
        pushBall.setDependentActions(shooterReady, turretReady, intakeReady);
        this.addAction(pushBall);


//        this.addAction(pushBall.getOpenStopper());
//

        resetOdometryToPosition = new ResetOdometryToLimelight(turret);
        resetOdometryToPosition.setName("resetOdometryToPosition");
        resetOdometryToPosition.setDependentActions(pushBall);
        this.addAction(resetOdometryToPosition);
    }

    /** Abort only when a gate timed out and the two gates are not both ready (both ready = a legit feed). */
    public static boolean shouldAbortOnTimeout(boolean abortOnTimeout, boolean shooterTimedOut, boolean turretTimedOut,
                                               boolean shooterDone, boolean turretDone) {
        return abortOnTimeout && (shooterTimedOut || turretTimedOut) && !(shooterDone && turretDone);
    }

    @Override
    public void update() {
        // Checked before super.update(): KActionSet runs PushBall in the same pass as beforeUpdate(), so aborting there can leak a feed.
        if (!isDone && shouldAbortOnTimeout(abortOnTimeout, shooterReady.timedOut(), turretReady.timedOut(),
                shooterReady.getIsDone(), turretReady.getIsDone())) {
            KLog.e(TAG, "event=abort reason=timeout name=" + getName() + " shooterTimedOut=" + shooterReady.timedOut()
                    + " turretTimedOut=" + turretReady.timedOut());
            setIsDone(true);
            return;
        }
        super.update();
    }

    @Override
    public void beforeUpdate() {
        if (isDone) {
            return;
        }
        if (!hasStarted) {
            //Refresh targetRPS
            shooterRun.update();
            hasStarted = true;
            stopper.close(); // TransferReady spins the transfer: an open stopper would feed early
            if (!enableLimelightZeroing) {
                resetOdometryToPosition.setIsDone(true);
            }
        }

        KLog.d(TAG, () -> "event=status shooterReady=" + shooterReady.getIsDone() + " turretReady=" + turretReady.getIsDone()
                + " transferReady=" + intakeReady.getIsDone() + " pushBall=" + pushBall.getIsDone());
    }

    @Override
    public void afterUpdate() {
        // TransferReady stops ticking once done: keep the transfer at speed while shooter / turret finish, until PushBall takes over.
        if (intakeReady.getIsDone() && !pushBall.getHasStarted()) {
            intakeReady.holdFeedRPS();
        }
        // Once only: setShooterRunMode also runs update(), so calling it every loop doubles the shooter update.
        if (pushBall.getIsDone() && shooterRun.getShooterRunMode() != ShooterRunMode.SHOOT_USING_CURRENT_POINT) {
//            double maintainRPSValue = shooterRun.getTargetRPS() * 0.925;
//            shooterRun.setTargetRPS(maintainRPSValue);
            shooterRun.setShooterRunMode(ShooterRunMode.SHOOT_USING_CURRENT_POINT);
            //KLog.d("ShooterRun", "Maintaining " + maintainRPSValue + " RPS after Running ShootAllAction");
        }
        KLog.d(TAG, () -> String.format("event=update stopperOpen=%s turretDeltaDeg=%.2f turretWithinRange=%s deltaRps=%.2f",
                (pushBall.getOpenStopper().getIsDone()),
                (turretAutoAlign.getDeltaAngleDeg()),
                (turretAutoAlign.isWithinRange()),
                (shooterRun.getTargetRPS() - shooter.getRPS())));
        if (pushBall.getOpenStopper().getIsDone()) {
            if (!SOTM.isActive()) {
                turretAutoAlign.stop();
            }
        }
    }

    public void setTurretReady(boolean isDone) {
        turretReady.setIsDone(isDone);
    }



}
