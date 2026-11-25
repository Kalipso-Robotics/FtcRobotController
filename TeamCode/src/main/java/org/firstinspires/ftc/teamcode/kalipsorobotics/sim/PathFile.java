package org.firstinspires.ftc.teamcode.kalipsorobotics.sim;

import com.google.gson.Gson;

import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.Reader;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;

/**
 * A hand-drawn auto: {name, alliance, start:[x,y,deg], steps:[{drive:[x,y,deg]} | {intake:bool} | {shoot:n} | {wait:seconds}]}.
 * mm, alliance frame (like FieldConfig). Loaded from -Dsim.pathFile=file.json, else the resource paths/AutoName.json.
 * Written by the sim's #/path editor (sim/paths/r1/*.json).
 */
public class PathFile {
    public String name;
    public String alliance;
    public double[] start = {0, 0, 0};
    public List<Step> steps = new ArrayList<>();

    public static class Step {
        public double[] drive;
        public Double speedMmS;   // ponytail: stored for the editor, not applied yet (the pure pursuit has one speed profile)
        public Boolean intake;
        public Integer shoot;     // ponytail: one shoot step empties the robot (PushBall); the count is only a label
        public Double wait;       // seconds
    }

    public enum Kind { DRIVE, INTAKE_ON, INTAKE_OFF, SHOOT, WAIT }

    /** One thing to run: a drive (all its points) or one other step. */
    public static class Segment {
        public final Kind kind;
        public final List<double[]> points = new ArrayList<>();
        public double value;   // wait: ms

        Segment(Kind kind) {
            this.kind = kind;
        }
    }

    /** Consecutive drive steps become one Segment; every other step is its own Segment, in order. */
    public List<Segment> segments() {
        List<Segment> out = new ArrayList<>();
        for (Step s : steps) {
            if (s.drive != null) {
                Segment last = out.isEmpty() ? null : out.get(out.size() - 1);
                if (last == null || last.kind != Kind.DRIVE) {
                    last = new Segment(Kind.DRIVE);
                    out.add(last);
                }
                last.points.add(s.drive);
            } else if (s.intake != null) {
                out.add(new Segment(s.intake ? Kind.INTAKE_ON : Kind.INTAKE_OFF));
            } else if (s.shoot != null) {
                out.add(new Segment(Kind.SHOOT));
            } else if (s.wait != null) {
                Segment w = new Segment(Kind.WAIT);
                w.value = s.wait * 1000;
                out.add(w);
            } else {
                throw new IllegalArgumentException("step has none of drive/intake/shoot/wait");
            }
        }
        return out;
    }

    public static PathFile fromJson(String json) {
        PathFile f = new Gson().fromJson(json, PathFile.class);
        if (f == null || f.start == null || f.start.length != 3 || f.steps == null) {
            throw new IllegalArgumentException("path file needs start:[x,y,deg] and steps");
        }
        return f;
    }

    /** -Dsim.pathFile, else resources paths/autoName.json. */
    public static PathFile load(String autoName) {
        try {
            String path = System.getProperty("sim.pathFile");
            if (path != null) {
                try (Reader r = new FileReader(new File(path))) {
                    return fromJson(read(r));
                }
            }
            InputStream in = PathFile.class.getClassLoader().getResourceAsStream("paths/" + autoName + ".json");
            if (in == null) {
                throw new IllegalStateException("no path file: set -Dsim.pathFile or add resources paths/" + autoName + ".json");
            }
            try (Reader r = new InputStreamReader(in, StandardCharsets.UTF_8)) {
                return fromJson(read(r));
            }
        } catch (IOException e) {
            throw new IllegalStateException("cannot read path file for " + autoName, e);
        }
    }

    private static String read(Reader reader) throws IOException {
        StringBuilder sb = new StringBuilder();
        char[] buf = new char[4096];
        for (int n; (n = reader.read(buf)) > 0; ) {
            sb.append(buf, 0, n);
        }
        return sb.toString();
    }
}
