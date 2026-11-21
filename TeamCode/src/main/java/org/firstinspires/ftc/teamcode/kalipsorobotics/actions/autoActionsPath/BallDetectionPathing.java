package org.firstinspires.ftc.teamcode.kalipsorobotics.actions.autoActionsPath;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.Action;
import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.actionUtilities.KActionSet;
import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.ClusterDetectionAction;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.DriveTrain;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.Stopper;
import org.firstinspires.ftc.teamcode.kalipsorobotics.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.SharedData;
import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.apriltag.AllianceColor;

/**
 * Look at the balls the Arducam sees, pick the best cluster, drive there and intake it.
 * Needs BallVision.start(...) running. Usage in a new auto:
 *   auto.addAction(new BallDetectionPathing(driveTrain, stopper, intake, allianceColor));
 * If no cluster is worth picking up, getFailed() is true and nothing drives.
 */
public class BallDetectionPathing extends KActionSet {
    /** How many distinct camera frames the look waits for. */
    public static int LOOK_FRAMES = 3; // TODO-measure: tune how many frames give a stable reading
    /** The look gives up waiting after this long and uses whatever the camera has. */
    public static double LOOK_TIMEOUT_MS = 500; // TODO-measure: tune how many frames give a stable reading

    private final AutoToBallsPathingAction pathing;

    public BallDetectionPathing(DriveTrain driveTrain, Stopper stopper, Intake intake, AllianceColor allianceColor) {
        Action look = new Action() {
            private long startMs;
            private long lastFrameMs;
            private int frames = 0;
            private boolean sawBalls = false;

            @Override
            protected void update() {
                if (!hasStarted) {
                    hasStarted = true;
                    startMs = System.currentTimeMillis();
                }
                long frameMs = SharedData.getBallInformationTimeMs();
                if (frameMs > startMs && frameMs != lastFrameMs) { // a new frame, taken after we started looking
                    lastFrameMs = frameMs;
                    frames++;
                    sawBalls |= !SharedData.getBallInformation().isEmpty();
                }
                boolean enoughFrames = frames >= LOOK_FRAMES && sawBalls;
                if (enoughFrames || System.currentTimeMillis() - startMs >= LOOK_TIMEOUT_MS) {
                    isDone = true;
                }
            }
        };
        look.setName("ballLook");

        // ponytail: ClusterDetectionAction uses only the latest frame, not a union of the looked frames.
        // Union them (dedupe with BallInformation.isSameBall) if a single frame turns out too patchy.
        ClusterDetectionAction cluster = new ClusterDetectionAction(allianceColor);
        cluster.setName("ballCluster");
        cluster.setDependentActions(look);

        // No cluster => ClusterDetectionAction clears the target and this fails cleanly without driving.
        pathing = new AutoToBallsPathingAction(driveTrain, stopper, intake, true);
        pathing.setName("ballPathing");
        pathing.setDependentActions(cluster);

        addAction(look, cluster, pathing);
    }

    /** True if no cluster was worth picking up (or the planner said no), so nothing was driven. */
    public boolean getFailed() {
        return pathing.getFailed();
    }
}
