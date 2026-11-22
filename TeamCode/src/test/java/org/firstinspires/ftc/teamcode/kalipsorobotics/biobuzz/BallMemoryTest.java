package org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz;

import static org.junit.Assert.assertEquals;

import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.junit.Test;

import java.util.Arrays;
import java.util.Collections;

public class BallMemoryTest {
    private static BallInformation ball(double x, double y) {
        return new BallInformation(x, y, BallInformation.Type.YELLOW);
    }

    @Test
    public void ballOutOfViewIsStillRemembered() {
        BallMemory memory = new BallMemory();
        memory.add(Arrays.asList(ball(500, 500), ball(1000, 1000)), 0);
        memory.add(Collections.singletonList(ball(500, 500)), 100); // the second ball left the view
        assertEquals(2, memory.getBalls().size());
    }

    @Test
    public void nearbySightingIsTheSameBall() {
        BallMemory memory = new BallMemory();
        memory.add(Collections.singletonList(ball(500, 500)), 0);
        memory.add(Collections.singletonList(ball(530, 520)), 100);
        assertEquals(1, memory.getBalls().size());
        assertEquals(530, memory.getBalls().get(0).x, 1e-9); // newest sighting wins
    }

    @Test
    public void oldBallsAreForgottenAndRefreshedOnesKept() {
        BallMemory memory = new BallMemory();
        memory.add(Arrays.asList(ball(500, 500), ball(1000, 1000)), 0);
        memory.add(Collections.singletonList(ball(500, 500)), BallMemory.MEMORY_MS);
        memory.forgetOld(BallMemory.MEMORY_MS + 1);
        assertEquals(1, memory.getBalls().size());
        assertEquals(500, memory.getBalls().get(0).x, 1e-9);
    }

    @Test
    public void forgetNearRemovesPickedUpBalls() {
        BallMemory memory = new BallMemory();
        memory.add(Arrays.asList(ball(500, 500), ball(1000, 1000)), 0);
        memory.forgetNear(new Point(510, 500), BallMemory.MERGE_MM);
        assertEquals(1, memory.getBalls().size());
    }
}
