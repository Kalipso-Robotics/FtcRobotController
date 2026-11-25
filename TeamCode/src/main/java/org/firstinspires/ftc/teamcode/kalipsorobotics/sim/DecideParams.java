package org.firstinspires.ftc.teamcode.kalipsorobotics.sim;

import com.google.gson.Gson;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;

import org.firstinspires.ftc.teamcode.kalipsorobotics.sim.RedTippingAuto.PartnerMode;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;

import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.Reader;
import java.lang.reflect.Field;
import java.nio.charset.StandardCharsets;
import java.util.Map;

/**
 * The tuned and GUESS numbers RedDecideAuto (and its subclasses) decide with. Public fields so a JSON file can
 * override any of them: -Dsim.autoParams=file.json, else the resource autoparams/AutoName.json, else the defaults.
 * Measured values (manual numbers) stay static final in the OpMode.
 */
public class DecideParams {
    public PartnerMode PARTNER_MODE = PartnerMode.SOLO_NOTHING;
    public double SPEED_MM_PER_SECOND = 1000;        // GUESS: how fast the robot drives
    public double SECONDS_TO_PICK_UP = 2;            // GUESS
    public double SECONDS_TO_SHOOT = 2;              // GUESS
    public double WAIT_MS = 500;                     // how long one WAIT lasts
    public double FLOWER_BALL_OFFSET_MM = 150;       // GUESS: the pickup target is this far from the Flower's center
    public double GARDEN_BALLS_CENTER_X_MM = 142;    // GUESS from the sim field: the 4 Garden balls sit at x = 36 to 249
    public double OBSTACLE_BUFFER_MM = 262.5;          // same as DecideSimulation / AutoToBallsPathingAction
    public double AT_SHOOT_POINT_MM = 150;           // closer than this to the launch point = "at the shoot point"
    public double PARK_WALL_GAP_MM = 15;             // GUESS: room left between the body and the walls at the park spot
    public int BIG_CLUSTER_BALLS = 3;   // a seen cluster this big is taken before the next pile
    public double VIEW_PLUS_Y_X = 700;
    public double VIEW_PLUS_Y_Y = 2500;
    public double VIEW_PLUS_Y_HEADING_DEG = 90;
    public double VIEW_MINUS_Y_X = 700;
    public double VIEW_MINUS_Y_Y = 1100;
    public double VIEW_MINUS_Y_HEADING_DEG = -90;
    public double SWEEP_OUT_X_MM = 750;              // out lane, along the Hive's side of the corridor
    public double SWEEP_BACK_X_MM = 300;             // back lane, along the wall
    public double SWEEP_FAR_WALL_GAP_MM = 450;       // the far end of the loop stops this far from the spill wall
    public double SWEEP_NEAR_INSET_MM = 450;         // the near end is this far inward from the up Cell's launch point
    public double SWEEP_DETOUR_MM = 400;             // a seen cluster this close to the leg we are driving is worth a detour
    public long SWEEP_CHECK_MS = 250;                // how often the sweep looks for a cluster beside the lane
    public double LOOK_SETTLE_MS = 700;           // after arriving, let the rolling balls settle and the camera look
    public double COUNT_SETTLE_SECONDS = 0.4;        // after a pickup the beam-break count lags a moment: wait before deciding
    public double BALL_NEAR_CENTER_MM = 100;        // a cluster's middle counts as "on a ball" when one is this close
    public long SIGHTING_MAX_AGE_MS = 500;           // a camera list older than this is ignored
    public long BALL_MEMORY_MS = 5000;               // a ball seen while driving is remembered this long (spilled balls roll away)
    public double COMMIT_MAX_SECONDS = 8;             // a chosen cluster is kept this long (the target does not change every decision)
    public double COMMIT_REACHED_MM = 250;            // within this of the chosen cluster = we are there
    public int COMMIT_SWITCH_EXTRA_BALLS = 2;         // a new cluster must have this many more balls to replace the chosen one...
    public double COMMIT_SWITCH_CLOSER = 0.6;         // ...or be this fraction of its distance
    public double LONE_BALL_MAX_MM = 1500;            // a single ball is only worth a trip when it is this close
    public double SWEEP_MIN_SECONDS_LEFT = 5;         // with less time than this left, no new sweep: park
    public double SWEEP_CORNER_REACHED_MM = 300;      // passing this close to a corner counts the corner as done
    public double NEAR_FIRST_MM = 1200;              // seen balls this close are taken before any farther ones
    public double BALL_MERGE_MM = 100;               // a new sighting this close to a remembered ball is the same ball
    public double BALL_FORGET_NEAR_ROBOT_MM = 250;   // we drove over this spot: it was picked up (or the camera cannot see it)
    public double SEEN_CLUSTER_REPEAT_MM = 300;      // balls this close to a cluster we already visited are ignored
    public double PICKUP_GIVE_UP_SECONDS = 6;
    public boolean SOTM = false;                    // shoot on the move: fire while still driving
    public double SOTM_RELEASE_MM = 300;            // GUESS: with SOTM, release the shot this far before the shoot point

    /** Defaults, overridden by -Dsim.autoParams or autoparams/autoName.json. Unknown keys throw in the sim, warn on the robot. */
    public static DecideParams load(String autoName) {
        try {
            String path = System.getProperty("sim.autoParams");
            if (path != null) {
                try (Reader reader = new FileReader(new File(path))) {
                    return fromJson(read(reader), isSim());
                }
            }
            InputStream in = DecideParams.class.getClassLoader().getResourceAsStream("autoparams/" + autoName + ".json");
            if (in == null) {
                return new DecideParams();
            }
            try (Reader reader = new InputStreamReader(in, StandardCharsets.UTF_8)) {
                return fromJson(read(reader), isSim());
            }
        } catch (IOException e) {
            throw new IllegalStateException("cannot read auto params for " + autoName, e);
        }
    }

    /** ponytail: the robot's VM is Dalvik/ART (java.vm.name = "Dalvik"); anything else is the sim or a unit test. */
    private static boolean isSim() {
        return !"Dalvik".equals(System.getProperty("java.vm.name"));
    }

    private static String read(Reader reader) throws IOException {
        StringBuilder sb = new StringBuilder();
        char[] buf = new char[4096];
        for (int n; (n = reader.read(buf)) > 0; ) {
            sb.append(buf, 0, n);
        }
        return sb.toString();
    }

    static DecideParams fromJson(String json, boolean strict) {
        DecideParams p = new DecideParams();
        Gson gson = new Gson();
        JsonObject object = gson.fromJson(json, JsonObject.class);
        for (Map.Entry<String, JsonElement> entry : object.entrySet()) {
            try {
                Field field = DecideParams.class.getField(entry.getKey());
                field.set(p, gson.fromJson(entry.getValue(), field.getGenericType()));
            } catch (NoSuchFieldException | IllegalAccessException e) {
                String message = "unknown auto param " + entry.getKey();
                if (strict) {
                    throw new IllegalArgumentException(message, e);
                }
                KLog.d("DecideParams", () -> message + " (ignored)");
            }
        }
        return p;
    }
}
