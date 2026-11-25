package org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertArrayEquals;

import org.firstinspires.ftc.teamcode.kalipsorobotics.sim.PathFile;
import org.junit.Test;

import java.util.List;

public class PathFileTest {
    private static final String JSON = "{\"name\":\"t\",\"alliance\":\"red\",\"start\":[100,200,90],\"steps\":["
            + "{\"drive\":[1,2,3]},{\"drive\":[4,5,6],\"speedMmS\":300},{\"intake\":true},{\"drive\":[7,8,9]},"
            + "{\"intake\":false},{\"shoot\":3},{\"wait\":1.5}]}";

    @Test
    public void consecutiveDrivesBecomeOneSegmentAndOtherStepsKeepOrder() {
        PathFile f = PathFile.fromJson(JSON);
        assertArrayEquals(new double[]{100, 200, 90}, f.start, 0);
        List<PathFile.Segment> s = f.segments();
        assertEquals(6, s.size());
        assertEquals(PathFile.Kind.DRIVE, s.get(0).kind);
        assertEquals(2, s.get(0).points.size());
        assertArrayEquals(new double[]{4, 5, 6}, s.get(0).points.get(1), 0);
        assertEquals(PathFile.Kind.INTAKE_ON, s.get(1).kind);
        assertEquals(PathFile.Kind.DRIVE, s.get(2).kind);
        assertEquals(1, s.get(2).points.size());
        assertEquals(PathFile.Kind.INTAKE_OFF, s.get(3).kind);
        assertEquals(PathFile.Kind.SHOOT, s.get(4).kind);
        assertEquals(PathFile.Kind.WAIT, s.get(5).kind);
        assertEquals(1500, s.get(5).value, 1e-9);
    }

    @Test(expected = IllegalArgumentException.class)
    public void aStepWithNothingInItIsRejected() {
        PathFile.fromJson("{\"start\":[0,0,0],\"steps\":[{}]}").segments();
    }
}
