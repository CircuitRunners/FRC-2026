// package frc.robot.subsystems.vision;

// import frc.lib.vision.Fiducial;
// import frc.lib.vision.PhotonVisionCamera;
// import frc.lib.vision.PipelineConfig;
// import frc.lib.vision.PipelineVisionPacket;
// import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
// import edu.wpi.first.math.Matrix;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.math.numbers.N1;
// import edu.wpi.first.math.numbers.N3;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import org.ejml.simple.SimpleMatrix;

// import com.ctre.phoenix6.swerve.SwerveDrivetrain;

// import java.util.Optional;

// /**
//  * VisionSubsystem — manages a PhotonVision camera and integrates its pose data.
//  */
// public class VisionSubsystem {

//     // The PhotonVision camera wrapper from frc.lib
//     private final PhotonVisionCamera camera;

//     // Debug field visualization (for simulation or SmartDashboard)
//     private final Field2d debugField = new Field2d();

//     // Vision measurement noise covariance (standard deviations)
//     private final Matrix<N3, N1> visionStdDevs = new Matrix<>(N3.instance, N1.instance);

//     // Camera transform relative to robot center (meters)
//     private static final Transform3d robotToCam =
//         new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d());

//     public VisionSubsystem() {
//         // Define the camera pipeline configuration(s)
//         PipelineConfig[] configs = new PipelineConfig[] {
//             new PipelineConfig(
//                 Fiducial.Type.APRILTAG,
//                 640, // width
//                 480, // height
//                 null, // camera intrinsics if used in sim
//                 null  // distortion coefficients if used in sim
//             )
//         };

//         // Create the PhotonVision camera
//         camera = new PhotonVisionCamera("photonvision", robotToCam, configs);

//         // Default standard deviations (x, y, theta)
//         visionStdDevs.set(0, 0, 0.5);
//         visionStdDevs.set(1, 0, 0.5);
//         visionStdDevs.set(2, 0, 1.0);
//     }

//     /**
//      * Updates the vision system and returns the latest pose estimate, if available.
//      */
//     // public Optional<Pose3d> updateVisionPose(Pose2d currentEstimatedPose) {
//     //     // Pull in the latest vision data
//     //     camera.updateInputs();
//     //     PipelineVisionPacket packet = camera.getLatestMeasurement();

//     //     // If there are no targets, return empty
//     //     if (!packet.hasTargets()) return Optional.empty();

//     //     // Process the vision data to get a pose estimate
//     //     Optional<Pose3d> estimatedPose = processVision(packet);

//     //     // Optionally draw to a Field2d for debugging
//     //     estimatedPose.ifPresent(pose -> debugField.getObject("VisionPose").setPose(pose.toPose2d()));

//     //     return estimatedPose;
//     // }

//     public Optional<Pose3d> updateVisionPose(CommandSwerveDrivetrain drivetrain) {
//     camera.updateInputs();
//     PipelineVisionPacket packet = camera.getLatestMeasurement();

//     if (!packet.hasTargets()) return Optional.empty();

//     // Convert camera data into a robot pose (you might do AprilTag solving here)
//     Optional<Pose3d> visionEst = processVision(packet);

//     // ✅ Fuse vision into CTRE’s internal pose estimator
//     visionEst.ifPresent(visionPose ->
//         drivetrain.addVisionMeasurement(
//             visionPose.toPose2d(),
//             packet.getCaptureTimestamp(),
//             visionStdDevs
//         )
//     );

//     return visionEst;
// }


//     /**
//      * Example of using your vision packet data to compute a robot pose.
//      * In practice, this might feed a pose estimator or fusion algorithm.
//      */
//     private Optional<Pose3d> processVision(PipelineVisionPacket packet) {
//         // TODO: Integrate with your Pose Estimator logic if you have one in frc.lib
//         // For now, this simply demonstrates structure.
//         // Example: if your best target gives a field-relative pose
//         var bestTarget = packet.getBestTarget();

//         if (bestTarget == null) {
//             return Optional.empty();
//         }

//         // Convert best target to an estimated pose (placeholder example)
//         // Replace with your real vision-based pose computation
//         Pose3d estimatedPose = new Pose3d(
//             bestTarget.getBestCameraToTarget().getTranslation(),
//             bestTarget.getBestCameraToTarget().getRotation()
//         );

//         return Optional.of(estimatedPose);
//     }

//     public Field2d getDebugField() {
//         return debugField;
//     }

//     public Matrix<N3, N1> getVisionStdDevs() {
//         return visionStdDevs;
//     }
// }
