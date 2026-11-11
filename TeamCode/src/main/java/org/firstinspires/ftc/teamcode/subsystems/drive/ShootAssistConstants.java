package org.firstinspires.ftc.teamcode.subsystems.drive;

/**
 * Parameters for the Overtune ShootAssist/AutoShoot port. Names follow Overtune's
 * DriveConstants / VisionConstants / TransitConstants / ShooterConstants fields so the ported
 * logic reads line-for-line. "Spinup" values come from existing Spinup code; "OT" values had no
 * Spinup equivalent and still need on-robot calibration.
 */
public final class ShootAssistConstants {
    private ShootAssistConstants() { }

    /* ---- AutoAimController (OT DriveConstants) ---- */
    // Spinup ChassisAimShootCommand gain magnitude (Pedro heading P). Negative sign = OT convention:
    // Spinup now uses OT's Blue mixer + corrected wheel directions, so positive turn is clockwise.
    public static volatile double autoAimKp = -4.734866377156585;   // Spinup
    public static volatile double autoAimKi = 0.0;                  // Spinup
    public static volatile double autoAimKd = 0.0;                  // Spinup (OT -0.02)
    public static volatile double autoAimKf = 0.0;                  // Spinup
    public static volatile double autoAimKs = 0.06;                 // Spinup
    public static volatile double autoAimYawToTurnSign = 1.0;       // Spinup; flip if aim turns away
    public static volatile double autoAimMaxTurnPower = 0.65;       // Spinup
    public static volatile double autoAimTestMaxTurnPower = 0.65;   // Spinup
    public static volatile double autoAimAcquisitionTurnPower = 0.65; // OT default
    public static volatile double aimLockToleranceDegrees = 2.0;    // Spinup
    public static volatile double aimUnlockToleranceDegrees = 3.0;  // Spinup
    public static volatile double aimLockStableTimeSeconds = 0.1;   // Spinup

    /* ---- PositionHoldController (OT DriveConstants; Spinup ChassisAimShootCommand in mm) ---- */
    public static boolean positionHoldCalibrated = true;
    public static double positionHoldKp = 0.12;      // = Spinup 0.0047 /mm
    public static double positionHoldKi = 0.0;
    public static double positionHoldKd = 0.007;     // = Spinup 0.00028 /mm
    public static double positionHoldMaxPower = 0.6;
    public static double positionLockEnterToleranceInches = 0.75; // = Spinup 19.05 mm
    public static double positionLockExitToleranceInches = 0.75;
    public static double positionLockMaxSpeedInchesPerSecond = 2.0; // = Spinup 50.8 mm/s
    public static double positionLockStableTimeSeconds = 0.15;

    /* ---- Vision -> Pinpoint overwrite gates (OT defaults; Spinup has none) ---- */
    public static boolean pinpointConfigurationConfirmed = true;
    public static double maximumVisionPositionDifferenceInches = 1.0;
    public static double maximumVisionHeadingDifferenceDegrees = 5.0;
    public static int visionRejectedFramesBeforeOverride = 3;

    /* ---- Aim geometry ---- */
    /** Shooter exit relative to chassis front, CCW degrees. Spinup shoots forward (OT: 180, rear). */
    public static double shooterYawDegrees = 0.0;
    /** OT AutoHiveTarget: Pedro (0..144 in) to field-center frame. */
    public static double pedroFieldCenterInches = 72.0;
    public static double highHiveTiltThresholdDegrees = 23.0; // OT default

    /* ---- Shooter readiness (Spinup Solo: +/-30 TPS, 0 ms hold) ---- */
    public static double readyEnterToleranceTicksPerSecond = 30.0;
    public static double readyExitToleranceTicksPerSecond = 30.0;
    public static double readyStableTimeSeconds = 0.0;
    public static double targetChangeResetThresholdTicksPerSecond = 30.0; // OT default
    public static double pitchSettleTimeSeconds = 0.2;       // OT default
    public static double pitchChangeResetThreshold = 0.005;  // OT default

    /* ---- Transit firing sequence (OT TransitConstants) ---- */
    /** Brief reverse after the gate opens, before feeding. OT default; Spinup motor uses REVERSED. */
    public static double feedReverseTime = 0.1;

    /* ---- AutoShootCommand ---- */
    public static long preShotDelayMs = 7_000; // OT default
}
