package org.firstinspires.ftc.teamcode.kalipsorobotics.sim;

import org.firstinspires.ftc.teamcode.kalipsorobotics.vision.BallInformation;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * The balls the camera has seen, kept for a while so a ball that leaves the view is still known.
 * A new sighting near a remembered ball replaces it (same ball). A ball is forgotten when it is old,
 * or with forgetNear (we picked it up or drove over its spot). Same rules as RedDecideAuto's memory.
 *
 * ponytail: forgets by age only; V2 = forget only when the camera should see the spot and does not (field-of-view check).
 */
public class BallMemory {
    /** A new sighting this close to a remembered ball is the same ball, mm. */
    public static double MERGE_MM = 100;
    /** A ball not seen again for this long is forgotten (balls get bumped and roll). GUESS. */
    public static long MEMORY_MS = 5000;

    private final List<BallInformation> balls = new ArrayList<>();
    private final List<Long> seenAtMs = new ArrayList<>();

    /** Adds one frame's sightings. */
    public void add(List<BallInformation> seen, long nowMs) {
        for (BallInformation ball : seen) {
            if (ball == null) {
                continue;
            }
            int same = indexNear(ball.toPoint(), MERGE_MM);
            if (same >= 0) {
                balls.set(same, ball);
                seenAtMs.set(same, nowMs);
            } else {
                balls.add(ball);
                seenAtMs.add(nowMs);
            }
        }
    }

    /** Forgets balls not seen for MEMORY_MS. */
    public void forgetOld(long nowMs) {
        for (int i = balls.size() - 1; i >= 0; i--) {
            if (nowMs - seenAtMs.get(i) > MEMORY_MS) {
                remove(i);
            }
        }
    }

    /** Forgets balls within mm of point. */
    public void forgetNear(Point point, double mm) {
        for (int i = balls.size() - 1; i >= 0; i--) {
            if (Point.distance(balls.get(i).toPoint(), point) < mm) {
                remove(i);
            }
        }
    }

    public List<BallInformation> getBalls() {
        return Collections.unmodifiableList(balls);
    }

    public void clear() {
        balls.clear();
        seenAtMs.clear();
    }

    private int indexNear(Point point, double mm) {
        for (int i = 0; i < balls.size(); i++) {
            if (Point.distance(balls.get(i).toPoint(), point) < mm) {
                return i;
            }
        }
        return -1;
    }

    private void remove(int i) {
        balls.remove(i);
        seenAtMs.remove(i);
    }
}
