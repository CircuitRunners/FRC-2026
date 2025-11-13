package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.List;

public class VisionConstants {

  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  public static String camera0Name = "camera_0";
  public static String camera1Name = "camera_1";

  public static Transform3d robotToCamera0 =
      new Transform3d(
          Units.inchesToMeters(12.564),
          Units.inchesToMeters(8.0),
          Units.inchesToMeters(7.523),
          new Rotation3d(0.0, Units.degreesToRadians(-20.0), 0.0));
  public static Transform3d robotToCamera1 =
      new Transform3d(
          Units.inchesToMeters(-11.564),
          Units.inchesToMeters(0.0),
          Units.inchesToMeters(7.523),
          new Rotation3d(0.0, Units.degreesToRadians(-20.0), Units.degreesToRadians(180.0)));


  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;
  public static double maxSingleTagDistanceMeters = 3.0;

  public static double linearStdDevBaseline = 0.02;
  public static double angularStdDevBaseline = 0.06;

  public static boolean useVisionRotation = true;
  public static boolean useVisionRotationSingleTag = false;

  public static double[] cameraStdDevFactors =
      new double[] {
        1.0,
        1.0
      };

  public static final List<Integer> singleTagIdsToReject =
      new ArrayList<>() {
        {

          add(1);
          add(2);
          add(3);
          add(4);
          add(5);
          add(12);
          add(13);
          add(14);
          add(15);
          add(16);
        }
      };

  public static double linearStdDevMegatag2Factor = 0.5;
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY;
}