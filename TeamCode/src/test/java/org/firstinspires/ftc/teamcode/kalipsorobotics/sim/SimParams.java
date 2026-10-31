package org.firstinspires.ftc.teamcode.kalipsorobotics.sim;

import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.RobotFootprint;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * The numbers the user can change on the Setup screen. Each one is passed to the run as a system
 * property, -Dsim.param.<id>=<number>, and applied here before the OpMode starts.
 *
 * The robot size is applied to RobotFootprint, which BOTH the physics and the auto's path planner
 * read, so a bigger robot is bigger for both, the same as measuring the real robot would be.
 *
 * To add a parameter: add its id to defaults() and set(), and describe it in sim/params.json
 * (label, range, unit). The defaults come from here, so there is no second copy to keep in step.
 */
final class SimParams {

    static final String PREFIX = "sim.param.";

    private SimParams() {
    }

    /** The values the code has when nothing is changed. Read this BEFORE apply(). */
    static Map<String, Double> defaults() {
        Map<String, Double> values = new LinkedHashMap<>();
        values.put("robotFront", RobotFootprint.FRONT_MM);
        values.put("robotBack", RobotFootprint.BACK_MM);
        values.put("robotWidth", RobotFootprint.WIDTH_MM);
        values.put("cameraOn", SimCamera.on ? 1.0 : 0.0);
        values.put("cameraFovDiagonalDeg", SimCamera.fovDiagonalDeg);
        values.put("cameraMaxRangeMm", SimCamera.maxRangeMm);
        values.put("cameraMinRangeMm", SimCamera.minRangeMm);
        values.put("cameraMissPercent", SimCamera.missPercent);
        values.put("cameraErrorMm", SimCamera.errorMm);
        return values;
    }

    /** Applies every -Dsim.param.<id> property and returns the values now in force. An unknown id throws. */
    static Map<String, Double> applyFromSystemProperties() {
        Map<String, Double> values = defaults();
        for (String key : System.getProperties().stringPropertyNames()) {
            if (!key.startsWith(PREFIX)) {
                continue;
            }
            String id = key.substring(PREFIX.length());
            if (!values.containsKey(id)) {
                throw new IllegalArgumentException("Unknown simulator parameter \"" + id + "\". Known: " + values.keySet());
            }
            double value = Double.parseDouble(System.getProperty(key));
            set(id, value);
            values.put(id, value);
        }
        // The grab points sit inside the body, so they cannot be beyond a tip that was made shorter.
        RobotFootprint.FRONT_GRAB_MM = Math.min(RobotFootprint.FRONT_GRAB_MM, RobotFootprint.FRONT_MM);
        RobotFootprint.BACK_GRAB_MM = Math.min(RobotFootprint.BACK_GRAB_MM, RobotFootprint.BACK_MM);
        return values;
    }

    private static void set(String id, double value) {
        switch (id) {
            case "robotFront":
                RobotFootprint.FRONT_MM = value;
                break;
            case "robotBack":
                RobotFootprint.BACK_MM = value;
                break;
            case "robotWidth":
                RobotFootprint.WIDTH_MM = value;
                break;
            case "cameraOn":
                SimCamera.on = value != 0;
                break;
            case "cameraFovDiagonalDeg":
                SimCamera.fovDiagonalDeg = value;
                break;
            case "cameraMaxRangeMm":
                SimCamera.maxRangeMm = value;
                break;
            case "cameraMinRangeMm":
                SimCamera.minRangeMm = value;
                break;
            case "cameraMissPercent":
                SimCamera.missPercent = value;
                break;
            case "cameraErrorMm":
                SimCamera.errorMm = value;
                break;
            default:
                throw new IllegalArgumentException("No setter for simulator parameter " + id);
        }
    }
}
