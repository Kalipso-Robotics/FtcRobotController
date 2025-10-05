package com.kalipsorobotics.math;

/*
calculating hood angle degree from robot position using interpolated tables
 */
public class ShooterLUTCornerGrid {

    // field grid
    public static final double TILE_IN = 24.0;
    public static final int N = 7; // 0..6

    // find actual values
    private static final double THETA_MIN_DEG = 0.0;
    private static final double THETA_MAX_DEG = 35.0;
    private static final double RPM_MIN = 400.0;
    private static final double RPM_MAX = 6000.0;

    // PUT DATA HERE (NaN for unknown points).
    // Indexing: theta[i][j] where i is X corner (0..6), j is Y corner (0..6).
    // Coordinates for a corner (i,j): x = i*24 in, y = j*24 in.


    //hood angle (degrees)
    private static final double[][] THETA = new double[][]{
            {Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN},
            {Double.NaN, 0,          0,          0,          0,          0,       Double.NaN},
            {Double.NaN, 0,          0,          0,          0,          0,       Double.NaN},
            {Double.NaN, 0,          0,          0,          0,          0,       Double.NaN},
            {Double.NaN, 0,          0,          0,          0,          0,       Double.NaN},
            {Double.NaN, 0,          0,          0,          0,          0,       Double.NaN},
            {Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN},
    };

    //motor rpm
    private static final double[][] RPM = new double[][]{
            {Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN},
            {Double.NaN, 0,          0,          0,          0,          0,       Double.NaN},
            {Double.NaN, 0,          0,          0,          0,          0,       Double.NaN},
            {Double.NaN, 0,          0,          0,          0,          0,       Double.NaN},
            {Double.NaN, 0,          0,          0,          0,          0,       Double.NaN},
            {Double.NaN, 0,          0,          0,          0,          0,       Double.NaN},
            {Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN},
    };


    public static class ShotParams {
        public final double thetaDeg;
        public final double rpm;
        public ShotParams(double thetaDeg, double rpm){ this.thetaDeg = thetaDeg; this.rpm = rpm; }
    }

    //query odo data to b able to use tile unit
    public ShotParams queryInches(double xIn, double yIn){
        double u = xIn / TILE_IN;   // 0..6
        double v = yIn / TILE_IN;   // 0..6
        int i = (int)Math.floor(u);
        int j = (int)Math.floor(v);

        i = clampInt(i, 0, N-2);
        j = clampInt(j, 0, N-2);
        double du = u - i;
        double dv = v - j;

        Corner c00 = corner(i,   j  );
        Corner c10 = corner(i+1, j  );
        Corner c01 = corner(i,   j+1);
        Corner c11 = corner(i+1, j+1);

        if (!(c00.ok() && c10.ok() && c01.ok() && c11.ok())) {
            Corner nn = nearestAvailable(u, v);
            return clamp(nn.theta, nn.rpm);
        }

        double th =
                (1-du)*(1-dv)*c00.theta + du*(1-dv)*c10.theta +
                        (1-du)*dv*c01.theta     + du*dv*c11.theta;

        double rp =
                (1-du)*(1-dv)*c00.rpm + du*(1-dv)*c10.rpm +
                        (1-du)*dv*c01.rpm     + du*dv*c11.rpm;

        return clamp(th, rp);
    }
    //for odo values that use meters/cm/mm
    public ShotParams queryMeters(double xM, double yM){
        double xIn = xM * 39.37007874;
        double yIn = yM * 39.37007874;
        return queryInches(xIn, yIn);
    }

    //helper methods
    private static class Corner {
        final double theta, rpm; Corner(double t, double r){ theta=t; rpm=r; }
        boolean ok(){ return !Double.isNaN(theta) && !Double.isNaN(rpm); }
    }
    private static int clampInt(int x, int lo, int hi){ return Math.max(lo, Math.min(hi, x)); }
    private static double clamp(double x, double lo, double hi){ return Math.max(lo, Math.min(hi, x)); }

    private static Corner corner(int i, int j){
        return new Corner(THETA[i][j], RPM[i][j]);
    }

    private static Corner nearestAvailable(double u, double v){
        int i0 = clampInt((int)Math.round(u), 0, N-1);
        int j0 = clampInt((int)Math.round(v), 0, N-1);
        for (int r=0; r<N; r++){
            for (int di=-r; di<=r; di++){
                for (int dj=-r; dj<=r; dj++){
                    int ii = i0+di, jj = j0+dj;
                    if (ii<0||ii>=N||jj<0||jj>=N) continue;
                    Corner c = corner(ii, jj);
                    if (c.ok()) return c;
                }
            }
        }
        return new Corner(20.0, 1200.0); // safe default
    }

    private static ShotParams clamp(double th, double rp){
        th = clamp(th, THETA_MIN_DEG, THETA_MAX_DEG);
        rp = clamp(rp, RPM_MIN, RPM_MAX);
        return new ShotParams(th, rp);
    }
}
