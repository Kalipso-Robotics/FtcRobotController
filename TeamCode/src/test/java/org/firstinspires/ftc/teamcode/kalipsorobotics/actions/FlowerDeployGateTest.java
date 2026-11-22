package org.firstinspires.ftc.teamcode.kalipsorobotics.actions;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.kalipsorobotics.actions.autoActionsPath.AutoToBallsPathingAction;
import org.junit.Test;

public class FlowerDeployGateTest {
    private static final double CLEAR = AutoToBallsPathingAction.FLOWER_DEPLOY_CLEARANCE_MM;

    @Test
    public void alreadyDown_doesNotRedeploy() {
        assertFalse(AutoToBallsPathingAction.shouldLowerFlower(true, true, CLEAR + 100));
    }

    @Test
    public void sideUp_waits() {
        assertFalse(AutoToBallsPathingAction.shouldLowerFlower(false, false, CLEAR + 100));
    }

    @Test
    public void tooClose_doesNotDeploy() {
        assertFalse(AutoToBallsPathingAction.shouldLowerFlower(false, true, CLEAR - 1));
    }

    @Test
    public void clearAndReady_deploys() {
        assertTrue(AutoToBallsPathingAction.shouldLowerFlower(false, true, CLEAR));
    }
}
