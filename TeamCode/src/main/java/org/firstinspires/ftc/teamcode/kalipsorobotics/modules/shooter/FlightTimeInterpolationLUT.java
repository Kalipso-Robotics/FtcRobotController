package org.firstinspires.ftc.teamcode.kalipsorobotics.modules.shooter;

import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;

import java.util.ArrayList;

public class FlightTimeInterpolationLUT {

    public static final ArrayList<double[]> distanceMMToFlightTimeMS;

    static {
        distanceMMToFlightTimeMS = new ArrayList<>();

        distanceMMToFlightTimeMS.add(new double[]{877.0, 660.0});
        distanceMMToFlightTimeMS.add(new double[]{1150.0, 580.0});
        distanceMMToFlightTimeMS.add(new double[]{1327.0, 700.0});
        distanceMMToFlightTimeMS.add(new double[]{1494.0, 480.0});
        distanceMMToFlightTimeMS.add(new double[]{1555.0, 590.0});
        distanceMMToFlightTimeMS.add(new double[]{1686.0, 610.0});
        distanceMMToFlightTimeMS.add(new double[]{1827.0, 580.0});
        distanceMMToFlightTimeMS.add(new double[]{2193.0, 680.0});
        distanceMMToFlightTimeMS.add(new double[]{2367.0, 750.0});
        distanceMMToFlightTimeMS.add(new double[]{2704.0, 940.0});
        distanceMMToFlightTimeMS.add(new double[]{2680.0, 900.0});
        distanceMMToFlightTimeMS.add(new double[]{2591.0, 730.0});
        distanceMMToFlightTimeMS.add(new double[]{2976.0, 740.0});
        distanceMMToFlightTimeMS.add(new double[]{3289.0, 1000.0});

    }

    public static double calculate(double distance) {

        if (distance <= distanceMMToFlightTimeMS.get(0)[0]) {
            return distanceMMToFlightTimeMS.get(0)[1];
        }
        if (distance >= distanceMMToFlightTimeMS.get(distanceMMToFlightTimeMS.size() - 1)[0]) {
            return distanceMMToFlightTimeMS.get(distanceMMToFlightTimeMS.size() - 1)[1];
        }

        for (int i = 0; i < distanceMMToFlightTimeMS.size() -1; i++) {
            double[] lower = distanceMMToFlightTimeMS.get(i);
            double[] upper = distanceMMToFlightTimeMS.get(i+1);

            if (distance >= lower[0] && distance <= upper[0]) {
                // Linear interpolation
                double t = (distance - lower[0]) / (upper[0] - lower[0]);

                double interpolatedFlightTime = lower[1] + t * (upper[1] - lower[1]);

                return interpolatedFlightTime;
            }
        }

        KLog.d("FlightTimeMS", "INVALID DISTANCE RETURNING FALLBACK TO avg flight time");
        return distanceMMToFlightTimeMS.get(distanceMMToFlightTimeMS.size() / 2)[1];
    }
}
