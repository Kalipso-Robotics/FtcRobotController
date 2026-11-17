package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter;

import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter.SOTM;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KActionSet;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.intake.IntakeStop;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.shooter.pusher.PushBall;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretAutoAlignTeleOp;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretReadyTeleOp;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.turret.TurretStop;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.ShooterConfig;
import org.firstinspires.ftc.teamcode.kalipsorobotics.decode.configs.TurretConfig;
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
    private final TurretAutoAlignTeleOp turretAutoAlignTeleop;
    private double targetRPS;
    private double targetHoodPos;

    private final boolean hasTurnedLimelightOn = false;

    private ShooterReady shooterReady;

    private PushBall pushBall;

    private TurretReadyTeleOp turretReadyTeleOp;

    private ResetOdometryToLimelight resetOdometryToPosition;
    private IntakeStop intakeStop;

    public void setEnableLimelightZeroing(boolean enableLimelightZeroing) {
        this.enableLimelightZeroing = enableLimelightZeroing;
    }

    private boolean enableLimelightZeroing = true;
    private final boolean abortOnTimeout;

    public ShootAllAction(Turret turret, Stopper stopper, Intake intake, Shooter shooter, ShooterRun shooterRun, TurretAutoAlignTeleOp turretAutoAlignTeleop, double targetRPS, double targetHoodPos) {
        this(turret, stopper, intake, shooter, shooterRun, turretAutoAlignTeleop, targetRPS, targetHoodPos, false);
    }

    /** abortOnTimeout: TeleOp gate. If RPS or turret is not ready in TELEOP_READY_TIMEOUT_MS, finish without feeding. */
    public ShootAllAction(Turret turret, Stopper stopper, Intake intake, Shooter shooter, ShooterRun shooterRun, TurretAutoAlignTeleOp turretAutoAlignTeleop, double targetRPS, double targetHoodPos, boolean abortOnTimeout) {
        this.abortOnTimeout = abortOnTimeout;
        this.stopper = stopper;
        this.intake = intake;
        this.shooter = shooter;
        this.shooterRun = shooterRun;
        this.turretAutoAlignTeleop = turretAutoAlignTeleop;
        this.targetRPS = targetRPS;
        this.targetHoodPos = targetHoodPos;
        this.turret = turret;

        shooterRun.setShooterRunMode(ShooterRunMode.SHOOT_USING_TARGET_RPS_HOOD);
        shooterRun.setTargetRPS(targetRPS);
        shooterRun.setTargetHoodPosition(targetHoodPos);

        generateBasicAction(shooterRun, stopper, intake, turretAutoAlignTeleop);
    }

    public ShootAllAction(Turret turret, Stopper stopper, Intake intake, Shooter shooter, ShooterRun shooterRun, TurretAutoAlignTeleOp turretAutoAlignTeleop) {
        this(turret, stopper, intake, shooter, shooterRun, turretAutoAlignTeleop, false);
    }

    public ShootAllAction(Turret turret, Stopper stopper, Intake intake, Shooter shooter, ShooterRun shooterRun, TurretAutoAlignTeleOp turretAutoAlignTeleop, boolean abortOnTimeout) {
        this.abortOnTimeout = abortOnTimeout;
        this.stopper = stopper;
        this.intake = intake;
        this.shooter = shooter;
        this.shooterRun = shooterRun;
        this.turretAutoAlignTeleop = turretAutoAlignTeleop;
        this.turret = turret;

        shooterRun.setShooterRunMode(ShooterRunMode.SHOOT_USING_CURRENT_POINT);


        generateBasicAction(shooterRun, stopper, intake, turretAutoAlignTeleop);
    }

    public ShooterRun getShooterRun() {
        return shooterRun;
    }

    public ShooterReady getShooterReady() {
        return shooterReady;
    }



    private void generateBasicAction(ShooterRun shooterRun, Stopper stopper, Intake intake, TurretAutoAlignTeleOp turretAutoAlignTeleop) {

        shooterReady = abortOnTimeout
                ? new ShooterReady(shooterRun, ShooterConfig.TELEOP_READY_TIMEOUT_MS)
                : new ShooterReady(shooterRun);
        shooterReady.setName("shooterReady");
        this.addAction(shooterReady);

        turretReadyTeleOp = abortOnTimeout
                ? new TurretReadyTeleOp(turretAutoAlignTeleop, ShooterConfig.TELEOP_READY_TIMEOUT_MS)
                : new TurretReadyTeleOp(turretAutoAlignTeleop);
        turretReadyTeleOp.setName("turretReady");
        this.addAction(turretReadyTeleOp);

//        TurretStop turretStop = new TurretStop(turretAutoAlignTeleop);
//        turretStop.setName("turretStop");
//        turretStop.setDependentActions(turretReady);
//        this.addAction(turretStop);

        pushBall = new PushBall(stopper, intake);
        pushBall.setName("pushAllBalls");
        pushBall.setDependentActions(shooterReady, turretReadyTeleOp);
        this.addAction(pushBall);


//        this.addAction(pushBall.getOpenStopper());
//
//        TurretStop turretStop = new TurretStop();
//        turretStop.setDependentActions(pushBall.getOpenStopper());

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
        if (!isDone && shouldAbortOnTimeout(abortOnTimeout, shooterReady.timedOut(), turretReadyTeleOp.timedOut(),
                shooterReady.getIsDone(), turretReadyTeleOp.getIsDone())) {
            KLog.e(TAG, "event=abort reason=timeout name=" + getName() + " shooterTimedOut=" + shooterReady.timedOut()
                    + " turretTimedOut=" + turretReadyTeleOp.timedOut());
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
            if (!enableLimelightZeroing) {
                resetOdometryToPosition.setIsDone(true);
            }
        }

        KLog.d(TAG, () -> "event=status shooterReady=" + shooterReady.getIsDone() + " turretReady=" + turretReadyTeleOp.getIsDone() + " pushBall=" + pushBall.getIsDone());
    }

    @Override
    public void afterUpdate() {
        if (pushBall.getIsDone()) {
//            double maintainRPSValue = shooterRun.getTargetRPS() * 0.925;
//            shooterRun.setTargetRPS(maintainRPSValue);
            shooterRun.setShooterRunMode(ShooterRunMode.SHOOT_USING_CURRENT_POINT);
            //KLog.d("ShooterRun", "Maintaining " + maintainRPSValue + " RPS after Running ShootAllAction");
        }
        KLog.d(TAG, () -> String.format("event=update stopperOpen=%s turretDeltaDeg=%.2f turretWithinRange=%s deltaRps=%.2f",
                (pushBall.getOpenStopper().getIsDone()),
                (turretAutoAlignTeleop.getDeltaAngleDeg()),
                (turretAutoAlignTeleop.isWithinRange()),
                (shooterRun.getTargetRPS() - shooter.getRPS())));
        if (pushBall.getOpenStopper().getIsDone()) {
            if (!SOTM.isActive()) {
                turretAutoAlignTeleop.stop();
            }
        }
    }

    public void setTurretReady(boolean isDone) {
        turretReadyTeleOp.setIsDone(isDone);
    }



}
