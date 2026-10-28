package org.firstinspires.ftc.teamcode.subsystems.turret;

import org.firstinspires.ftc.teamcode.subsystems.vision.AimingInput.Cell;

/** Field configuration shared by future red/blue OpModes. */
public final class AimingConstants {
    private AimingConstants() { }
    public enum Alliance { RED, BLUE }

    // Set explicitly from your future red/blue OpModes before aiming starts.
    public static Alliance alliance = Alliance.BLUE;

    // Raised CELL opening centers projected onto the floor, in the documented frame.
    // A is audience-side; B is the opposite side. NaN means not calibrated.
    public static double redAX = Double.NaN, redAY = Double.NaN;
    public static double redBX = Double.NaN, redBY = Double.NaN;
    public static double blueAX = Double.NaN, blueAY = Double.NaN;
    public static double blueBX = Double.NaN, blueBY = Double.NaN;
    public static long maxObservationAgeMs = 500;
    // Origin at audience-side corner, +Y away from audience. Adjust to measured field.
    public static double halfFieldBoundaryYmm = 3657.6 / 2.0;

    /** On the dividing line, choose A deterministically. */
    public static Cell cellForPosition(double yMm) {
        if (!Double.isFinite(yMm)) return Cell.UNKNOWN;
        return yMm <= halfFieldBoundaryYmm ? Cell.A : Cell.B;
    }

    public static double[] target(Alliance side, Cell cell) {
        if (side == null || cell == null || cell == Cell.UNKNOWN) return null;
        if (side == Alliance.RED) {
            return cell == Cell.A ? new double[]{redAX, redAY} : new double[]{redBX, redBY};
        }
        return cell == Cell.A ? new double[]{blueAX, blueAY} : new double[]{blueBX, blueBY};
    }
}
