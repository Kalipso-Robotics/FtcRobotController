package org.firstinspires.ftc.teamcode.kalipsorobotics.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.BallInformation;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

public class SimCameraTest {

    @Before
    public void setUp() {
        SimCamera.on = true;
        SimCamera.fovDiagonalDeg = 100;
        SimCamera.maxRangeMm = 2000;
        SimCamera.minRangeMm = 300;
        SimCamera.missPercent = 0;
        SimCamera.errorMm = 20;
    }

    @After
    public void tearDown() {
        setUp();
    }

    /** {id, type, x, y, z, held} for a pollen ball lying on the floor. */
    private static double[] floorBall(double x, double y) {
        return new double[]{0, 0, x, y, 35.56, 0};
    }

    private static List<double[]> balls(double[]... rows) {
        List<double[]> list = new ArrayList<>();
        for (double[] row : rows) {
            list.add(row);
        }
        return list;
    }

    @Test
    public void halfWidthComesFromTheDiagonalOfA16By9Image() {
        // tan(50 deg) = 1.1918, times 1280 / 1468.6 = 1.0388, atan -> 46.1 deg
        assertEquals(46.1, Math.toDegrees(SimCamera.halfWidthRad()), 0.1);
    }

    @Test
    public void aBallStraightAheadIsSeenWithinTheErrorBox() {
        List<BallInformation> seen = new SimCamera().see(balls(floorBall(1000, 0)), 0, 0, 0);
        assertEquals(1, seen.size());
        assertEquals(1000, seen.get(0).x, 20.0001);
        assertEquals(0, seen.get(0).y, 20.0001);
        assertEquals(BallInformation.Type.YELLOW, seen.get(0).type);
    }

    @Test
    public void theViewFollowsTheRobotsHeading() {
        // Robot at the origin facing +y (theta = 90 deg): a ball at +y is ahead, a ball at +x is 90 deg to the side.
        List<BallInformation> seen = new SimCamera().see(balls(floorBall(0, 1000), floorBall(1000, 0)), 0, 0, Math.PI / 2);
        assertEquals(1, seen.size());
        assertEquals(1000, seen.get(0).y, 20.0001);
    }

    @Test
    public void ballsOutsideTheAngleBehindTooFarTooCloseHeldOrOffTheFloorAreNotSeen() {
        double[] inside = floorBall(1000, 600);                 // 31 deg: seen
        double[] outsideAngle = floorBall(1000, 1200);          // 50 deg: not seen
        double[] behind = floorBall(-1000, 0);
        double[] tooFar = floorBall(2500, 0);
        double[] tooClose = floorBall(200, 0);
        double[] held = {0, 0, 1000, 0, 35.56, 1};
        double[] inACell = {0, 0, 1000, 0, 900, 0};
        List<BallInformation> seen = new SimCamera().see(balls(inside, outsideAngle, behind, tooFar, tooClose, held, inACell), 0, 0, 0);
        assertEquals(1, seen.size());
        assertEquals(600, seen.get(0).y, 20.0001);
    }

    @Test
    public void aboutTwoPercentOfDetectionsAreMissed() {
        SimCamera.missPercent = 2;
        SimCamera camera = new SimCamera();
        int frames = 20000;
        int seenCount = 0;
        for (int i = 0; i < frames; i++) {
            seenCount += camera.see(balls(floorBall(1000, 0)), 0, 0, 0).size();
        }
        double missed = 100.0 * (frames - seenCount) / frames;
        assertTrue("missed " + missed + " %", missed > 1.5 && missed < 2.5);
    }

    @Test
    public void theSameSeedGivesTheSameFrames() {
        SimCamera.missPercent = 2;
        List<BallInformation> a = new ArrayList<>();
        List<BallInformation> b = new ArrayList<>();
        SimCamera first = new SimCamera();
        SimCamera second = new SimCamera();
        for (int i = 0; i < 200; i++) {
            a.addAll(first.see(balls(floorBall(1000, 0)), 0, 0, 0));
            b.addAll(second.see(balls(floorBall(1000, 0)), 0, 0, 0));
        }
        assertEquals(a.size(), b.size());
        for (int i = 0; i < a.size(); i++) {
            assertEquals(a.get(i).x, b.get(i).x, 0);
            assertEquals(a.get(i).y, b.get(i).y, 0);
        }
    }

    @Test
    public void errorNeverExceedsTheSetting() {
        SimCamera camera = new SimCamera();
        double worst = 0;
        for (int i = 0; i < 5000; i++) {
            for (BallInformation ball : camera.see(balls(floorBall(1000, 0)), 0, 0, 0)) {
                worst = Math.max(worst, Math.max(Math.abs(ball.x - 1000), Math.abs(ball.y)));
            }
        }
        assertTrue("worst error " + worst, worst <= 20 && worst > 15);
    }
}
