package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class PointInPolygon {

  public static final Pose2d[] blueUpperTrench =
      new Pose2d[] {
        new Pose2d(4.02, 8.07, new Rotation2d()),
        new Pose2d(4.02, 6.84, new Rotation2d()),
        new Pose2d(5.17, 6.84, new Rotation2d()),
        new Pose2d(5.17, 8.07, new Rotation2d()),
      };

  public static final Pose2d[] blueLowerTrench =
      new Pose2d[] {
        new Pose2d(4.02, 1.26, new Rotation2d()),
        new Pose2d(4.02, 0.0, new Rotation2d()),
        new Pose2d(5.17, 0.0, new Rotation2d()),
        new Pose2d(5.17, 1.26, new Rotation2d()),
      };

  public static final Pose2d[] redUpperTrench =
      new Pose2d[] {
        new Pose2d(11.27, 8.07, new Rotation2d()),
        new Pose2d(11.27, 6.84, new Rotation2d()),
        new Pose2d(12.43, 6.84, new Rotation2d()),
        new Pose2d(12.43, 8.07, new Rotation2d()),
      };

  public static final Pose2d[] redLowerTrench =
      new Pose2d[] {
        new Pose2d(11.27, 1.26, new Rotation2d()),
        new Pose2d(11.27, 0.0, new Rotation2d()),
        new Pose2d(12.43, 0.0, new Rotation2d()),
        new Pose2d(12.43, 1.26, new Rotation2d()),
      };

  public static boolean pointInPolygon(Pose2d point, Pose2d[] polygon) {
    boolean inside = false;
    int n = polygon.length;
    int j = n - 1; // The last vertex is the previous one to the first

    for (int i = 0; i < n; i++) {
      double xi = polygon[i].getX();
      double yi = polygon[i].getY();
      double xj = polygon[j].getX();
      double yj = polygon[j].getY();

      // Check if the point is inside the polygon using the ray-casting algorithm
      if ((yi > point.getY()) != (yj > point.getY())
          && (point.getX() < (xj - xi) * (point.getY() - yi) / (yj - yi + 1e-10) + xi)) {
        inside = !inside; // Toggle the inside status
      }
      j = i; // Move to the next vertex
    }

    return inside;
  }
}