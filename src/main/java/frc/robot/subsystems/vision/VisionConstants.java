// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import java.nio.file.Path;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Filesystem;

public class VisionConstants {
    public static final double kLargeVariance = 1e6;

    // AprilTag layout
    public static Path path = Filesystem.getDeployDirectory()
            .toPath()
            .resolve("apriltags/2026-rebuilt-andymark.json"); // Turkey uses andymark version of layouts
    public static AprilTagFieldLayout aprilTagLayout;
    static {
        try {
            aprilTagLayout = new AprilTagFieldLayout(path);
        } catch (java.io.IOException e) {
            throw new RuntimeException("Failed to load AprilTagFieldLayout from " + path, e);
        }
    }

    // Camera names, must match names configured on coprocessor
    public static String camera0Name = "camera_0";
    public static String camera1Name = "camera_1";

    // Robot to camera transforms
    // (Not used by Limelight, configure in web UI instead)
    public static Transform3d robotToCamera0 = new Transform3d(-0.315, -0.01, 0.1, new Rotation3d(0.0, -0.4, -Math.PI/2));
    public static Transform3d robotToCamera1 = new Transform3d(0, 0.3, 0.10, new Rotation3d(0.0, 0.0, 0));

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3; // -29,5
    public static double maxZError = 0.75;
    //public static double maxAmbiguity = 1; // -29,5
   // public static double maxZError = 999;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors = new double[] {
            1.0, // Camera 0
            1.0 // Camera 1
    };

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation data available
}
