// package frc.robot.auto.autos;

// import java.util.ArrayList;
// import java.util.List;

// import choreo.auto.AutoFactory;
// import choreo.auto.AutoTrajectory;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.units.Units;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import frc.lib.drive.PIDToPoseCommand;
// import frc.lib.util.FieldLayout;
// import frc.robot.subsystems.drive.Drive;
// import frc.robot.Robot;
// import frc.robot.RobotConstants;
// import frc.robot.RobotContainer;
// import frc.robot.auto.AutoConstants;
// import frc.robot.auto.AutoHelpers;
// import frc.robot.auto.AutoModeBase;
// import frc.robot.subsystems.superstructure.Superstructure;
// import frc.robot.subsystems.vision.objectdetection.ObjectPoseEstimator;


// public class CenterPreloadClimbLeft extends AutoModeBase {
// 	public CenterPreloadClimbLeft(Drive drive, Superstructure superstructure, AutoFactory factory) {
// 		super(drive, superstructure, factory, "Center Preload + Left Climb");
// 		AutoTrajectory centerPreloadClimb = trajectory("centerPreloadClimb");

// 		Pose2d startPose = centerPreloadClimb.getInitialPose().get();
		


// 		// prepRoutine(
// 		// 	AutoHelpers.resetPoseIfWithoutEstimate(startPose, drive),
// 		// 	Commands.parallel(cmdWithAccuracy(centerPreloadClimb), superstructure.runIntakeIfDeployed()),
// 		// 	superstructure.shootWhenReady().withTimeout(AutoConstants.shootAllFuelTime),
// 		// 	superstructure.climbLeft()
//         // );

// 		prepRoutine(
// 			AutoHelpers.resetPoseIfWithoutEstimate(startPose, drive),
// 		cmdWithAccuracy(centerPreloadClimb),
// 		superstructure.deployIntake(),
// 		Commands.sequence(
// 							Commands.parallel(superstructure.shootRun(),
// 							superstructure.hoodRun(),
// 							superstructure.shootWhenReadyAuto())).withTimeout(AutoConstants.shootAllFuelTime));

// 	}
// }