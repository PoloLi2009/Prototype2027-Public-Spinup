package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/** Killerqueen AutoFormal field geometry, expressed in inches and degrees. */
final class AutoFormalRoute {
    private static final PoseFactory POSES = PoseFactory.degrees();
    static final Pose START = POSES.of(54.2726, 7.9672, 90);

    private AutoFormalRoute() { }

    static Path[] paths() {
        Pose point1 = POSES.of(57.1091, 21.6998, 90);
        Pose point2 = POSES.of(7.4165, 23.4434, 270);
        Pose point3 = POSES.of(7.0525, 6.141, 270);
        Pose point4 = POSES.of(57.1357, 121.0297, 270);
        Pose point5Start = POSES.of(57.1357, 121.0297, 90);
        Pose point5 = POSES.of(46.9313, 132.8969, 90);
        Pose point6 = POSES.of(47.19, 117.4016, 90);
        Pose point7 = POSES.of(56.9942, 120.9401, 270);
        Pose point8 = POSES.of(8.6715, 107.8308, 0);

        return new Path[] {
                Paths.line(START, point1).linear(START, point1),
                Paths.line(point1, point2).linear(point1, point2),
                Paths.line(point2, point3).constant(point3),
                Paths.curve(point3, POSES.of(34.9821, 80.8158, 0),
                        POSES.of(38.2982, 96.3828, 0), POSES.of(16.5318, 84.5853, 0), point4)
                        .linear(point3, point4),
                Paths.curve(point5Start, POSES.of(44.6111, 116.5979, 0),
                        POSES.of(45.8742, 115.0938, 0), point5).linear(point5Start, point5),
                Paths.line(point5, point6).constant(point6),
                Paths.line(point6, point7).linear(point6, point7),
                Paths.line(point7, point8).linear(point7, point8)
        };
    }
}
