package org.firstinspires.ftc.teamcode.kalipsorobotics.test.shooter;

public class RampUpTimeInterpolation {
    public static double[] rampUpData = {
            26, 2957,
            26.5, 3015,
            27, 2955,
            27.5, 3115,
            28, 2816,
            28.5, 3083,
            29, 3157,
            29.5, 3104,
            30, 3324,
            30.5, 2986,
            31, 3156,
            31.5, 3064,
            32, 3065,
            32.5, 3144,
            33, 3147,
            33.5, 3383,
            34, 3300,
            34.5, 3217,
            35, 3152,
            35.5, 3070,
            36, 3297,
            36.5, 3299,
            37, 3235,
            37.5, 3532,
            38, 3545,
            38.5, 3203,
            39, 3157,
            39.5, 3370,
            40, 3385,
            40.5, 3298,
            41, 3705,
            41.5, 3586,
            42, 3600,
            42.5, 3587,
            43, 3608,
            43.5, 3506,
            44, 3590,
            44.5, 3754,
            45, 3612,
            45.5, 3441,
            46, 3676,
            46.5, 3834,
            47, 3608,
            47.5, 4132,
            48, 3980,
            48.5, 3667,
            49, 3544,
            49.5, 4064,
            50, 3983,
    };

    public static double lookupRampUpTime(double targetRPS) {
        double underRPS = 0;
        double overRPS = 0;
        double deltaOver = 0;
        double deltaUnder = 0;
        double deltaTotal = 0;
        double underRatio = 0;
        double overRatio = 0;

        // check for out of bounds + divide by zero
        for (int i = 0; i < rampUpData.length - 3; i+=2) {
            underRPS = rampUpData[i];
            overRPS = rampUpData[i+2];

            if (targetRPS >= underRPS && targetRPS <= overRPS) {
                deltaOver = Math.abs(overRPS - targetRPS);
                deltaUnder = Math.abs(underRPS - targetRPS);
                deltaTotal = deltaOver + deltaUnder;

                if (deltaTotal == 0) {
                    return rampUpData[i+1];
                }

                underRatio = deltaUnder / deltaTotal;
                overRatio = deltaOver / deltaTotal;

                return rampUpData[i+1] * overRatio + rampUpData[i+3] * underRatio;
            }
        }

        // fallbacks

        // if too low then get lowest
        if (targetRPS < rampUpData[0]) {
            return rampUpData[1];
        }

        // if too high then get highest
        return rampUpData[rampUpData.length - 1];
    }
}
