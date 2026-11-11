package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.acmerobotics.dashboard.config.Config;

/**
 * Parameters for the Overtune ShootAssist/AutoShoot port. Names follow Overtune's
 * DriveConstants / VisionConstants / TransitConstants / ShooterConstants fields so the ported
 * logic reads line-for-line. "Spinup" values come from existing Spinup code; "OT" values had no
 * Spinup equivalent and still need on-robot calibration.
 *
 * <p>@Config: every field can be edited live in FTC Dashboard (Config -> ShootAssistConstants).
 * Dashboard edits are lost on restart; copy tuned values back into this file.</p>
 */
@Config
public class ShootAssistConstants {
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

    /* ---- Distance-adaptive shot table (Overtune ShooterConstants near/farShotPoints) ---- */
    // Rows are {distance cm (robot center to CELL point), flywheel ticks/s, pitch servo position}.
    // Overtune's algorithm needs exactly 5 near + 5 far rows with strictly increasing distance.
    // Spinup has no recorded distance data yet, so every row keeps Spinup's current values
    // (Solo 900 TPS, pitch preset 0.50): behavior is unchanged until these rows are measured.
    //
    // ======================= 距离表标定步骤（Spinup solo） =======================
    // 算法（照抄 Overtune）：
    //   距离 d = Limelight 校准后的 Pinpoint 底盘中心 到 当前 CELL 点的水平距离（cm）。
    //   在 near+far 共 10 行里找 d 两侧最近的两行，按距离线性插值出 转速 和 俯仰；
    //   d 小于第一行 / 大于最后一行时直接用端点那一行（不外推）。
    //   不按 RT（预转）：只设转速，不动俯仰。按住 RT：转速和俯仰都按表。
    //   D-pad 左右设了手动转速时，只替换转速，俯仰仍按表；按 Y 回到查表。
    //
    // 0. 准备（每次标定前都要满足，否则距离是错的）：
    //    a. Limelight 网页里的安装位姿与 VisionConstants.cameraWebPose 一致（含倒置 roll=180）。
    //    b. INIT 前把云台手动对正车头（云台锁在启动角，视觉按这个角度解算）。
    //    c. INIT 时选对红蓝方（X 切换，遥测 "Alliance / CELL"），Pinpoint 显示 READY。
    //    d. 电脑连机器人 Wi-Fi，浏览器开 FTC Dashboard（192.168.43.1:8080/dash），
    //       在 Config -> ShootAssistConstants 里能看到 nearShotPoints / farShotPoints。
    //    e. 标定期间不要用手柄 2 的十字键切俯仰预设（会改俯仰起点，干扰判断）。
    //
    // 1. 读距离：START 后把车开到想标定的位置停稳，让 Limelight 看到场地 AprilTag。
    //    不按 RT 时视觉会持续校准位置；遥测 "Localization" 出现 "Vision correction applied"
    //    或 "Initialized..." 后，"Shot distance cm (robot center to CELL)" 才是可信的。
    //    记下这个数（停稳后看 2~3 秒，取稳定值）。若显示 "waiting for vision-calibrated position"，
    //    说明还没定位，先让相机看到标签。
    //
    // 2. 选点：近表 5 行覆盖常用近射位，远表 5 行覆盖远射位，10 个距离必须严格递增、>0。
    //    建议从实际会射击的最近位置到最远位置大致均匀取 10 个点；近表最后一行 < 远表第一行。
    //    先在 Dashboard 里把该行第 1 列改成第 1 步读到的距离。
    //
    // 3. 调转速（第 2 列，单位 ticks/s，右飞轮编码器）：
    //    在该位置按住 RT 连射。偏近/打框下沿 -> 加转速，偏远/飞过 -> 减转速。
    //    可以直接在 Dashboard 改第 2 列，也可以用手柄 1 D-pad 左右临时覆盖（每步 100 RPM ≈ 46.7 TPS，
    //    遥测 "Speed source" 显示当前覆盖值），找到最好的值后写回第 2 列，再按 Y 清除覆盖。
    //    注意达速窗口只有 ±readyEnterToleranceTicksPerSecond（30 TPS）：若遥测 "Shoot allowed"
    //    长期卡在 "flywheel not at speed"，说明该转速飞轮跟不住，需要先调射手 PID。
    //
    // 4. 调俯仰（第 3 列，舵机位置，范围 pitchMinPosition..pitchMaxPosition）：
    //    俯仰只在按住 RT 时按表移动。先确认方向：把某行第 3 列改大 0.05 按住 RT，观察斜坡是抬高还是放低，
    //    并在此注释里记下（目前未确认）。每次改 0.01~0.02，配合第 3 步一起调：
    //    先固定俯仰找能进的转速，再微调俯仰让落点最稳。改俯仰后要等 pitchSettleTimeSeconds 才会放行。
    //
    // 5. 确认：每个点至少连续 5 球满意再换下一个距离。10 行都填完后，在任意两行中间的位置再射一次，
    //    检查插值结果也能进（不进就在该段中间补点：把相邻某行移到这里重标）。
    //
    // 6. 保存：Dashboard 修改断电即丢失。把最终 10 行抄回下面两个数组，编译上传。
    //
    // 校验：行数不是 5+5、距离不递增、转速<=0、俯仰超出范围时整张表无效，飞轮不转，
    //       遥测 "Speed source" 显示 "distance table INVALID"。
    public static double[][] nearShotPoints = {
            {55.8, 900, 0.50},
            {64.4, 900, 0.50},
            {96.0, 900, 0.50},
            {113.2, 900, 0.50},
            {120.5, 900, 0.50}
    };
    public static double[][] farShotPoints = {
            {130.9, 900, 0.50},
            {152.1, 900, 0.50},
            {163.0, 900, 0.50},
            {183.4, 900, 0.50},
            {215.7, 900, 0.50}
    };
    // Spinup pitch servos accept the full 0..1 range (Shooter.adjustPitch clamps to 0..1).
    public static double pitchMinPosition = 0.0;
    public static double pitchMaxPosition = 1.0;

    /* ---- Transit firing sequence (OT TransitConstants) ---- */
    /** Brief reverse after the gate opens, before feeding. OT default; Spinup motor uses REVERSED. */
    public static double feedReverseTime = 0.1;

    /* ---- AutoShootCommand ---- */
    public static long preShotDelayMs = 7_000; // OT default
}
