package org.firstinspires.ftc.teamcode.utils;

/** Pure math; no FTC hardware required. */
public final class TurretMath {
    private TurretMath() { }

    /** Choose a physically reachable equivalent angle closest to the current angle.
     * Position error must NOT be wrapped: +179 to -179 must travel through zero.
     */
    public static double reachableDegrees(double requested, double current,
                                           double min, double max) {
        if (!Double.isFinite(requested) || !Double.isFinite(current)
                || !Double.isFinite(min) || !Double.isFinite(max) || min > max) {
            return Double.NaN;
        }
        double base = requested % 360.0;
        double best = Double.NaN;
        double distance = Double.POSITIVE_INFINITY;
        for (double turn = Math.ceil((min - base) / 360.0);
             turn <= Math.floor((max - base) / 360.0); turn++) {
            double candidate = base + turn * 360.0;
            if (Math.abs(candidate - current) < distance) {
                best = candidate;
                distance = Math.abs(candidate - current);
            }
        }
        return best;
    }
}
