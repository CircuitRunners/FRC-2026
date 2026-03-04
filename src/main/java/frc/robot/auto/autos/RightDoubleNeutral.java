package frc.robot.auto.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.drive.PIDToPoseCommand;
import frc.lib.util.FieldLayout;
import frc.robot.RobotConstants;
import frc.robot.auto.AutoConstants;
import frc.robot.auto.AutoHelpers;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.vision.objectdetection.ObjectPoseEstimator;
import frc.robot.auto.AutoModeBase;

public class RightDoubleNeutral extends AutoModeBase {

	public RightDoubleNeutral(Drive drive, Superstructure superstructure, AutoFactory factory) {
		super(drive, superstructure, factory, "Right Neutral Cycle + Climb");

		AutoTrajectory rightTrenchToNeutral = trajectory("rightTrenchToNeutral");
		AutoTrajectory rightNeutralToFuel = trajectory("rightNuetralToFuel");
		AutoTrajectory rightNeutralToTrench = trajectory("rightNeutralToTrench");
		AutoTrajectory rightTrenchToShoot = trajectory("rightTrenchToShoot");
		AutoTrajectory rightShootToTrench = trajectory("rightShootToTrench");

		Pose2d startPose = rightTrenchToNeutral.getInitialPose().get();

		superstructure.updateSide(ObjectPoseEstimator.INTAKE_SIDE.RIGHT);


		prepRoutine(
			AutoHelpers.resetPoseIfWithoutEstimate(startPose, drive),
			Commands.parallel(rightTrenchToNeutral.cmd(), superstructure.deployIntake()),
			Commands.deadline(
			Commands.sequence(
				//superstructure.collectFuel(rightNeutralToTrench.getInitialPose().get()).withTimeout(7),
				// rightNeutralToFuel.cmd(),
				new PIDToPoseCommand(drive, superstructure, FieldLayout.handleAllianceFlip(new Pose2d(new Translation2d(
					7.620832920074463, 1.069169521331787), Rotation2d.fromDegrees(90)), RobotConstants.isRedAlliance),
					Units.Inches.of(10.0), Units.Degrees.of(20.0)
				),
				new PIDToPoseCommand(drive, superstructure, FieldLayout.handleAllianceFlip(new Pose2d(new Translation2d(
					7.620832920074463, 2.8354833126068115), Rotation2d.fromDegrees(90)), RobotConstants.isRedAlliance),
					Units.Inches.of(10.0), Units.Degrees.of(20.0)
				),

				new PIDToPoseCommand(drive, superstructure, rightNeutralToTrench.getInitialPose().get(), Units.Inches.of(10.0), Units.Degrees.of(20.0))
			), superstructure.runIntakeIfDeployed()),
			rightNeutralToTrench.cmd(),
			cmdWithAccuracy(rightTrenchToShoot),
			Commands.sequence(
							Commands.parallel(superstructure.shootRun(),
							superstructure.hoodRun(),
							superstructure.shootWhenReady())).withTimeout(AutoConstants.shootAllFuelTime),
			rightShootToTrench.cmd(),
			Commands.parallel(rightTrenchToNeutral.cmd()),
			Commands.deadline(
			Commands.sequence(
				//superstructure.collectFuel(rightNeutralToTrench.getInitialPose().get()).withTimeout(7),
				// rightNeutralToFuel.cmd(),
				new PIDToPoseCommand(drive, superstructure, FieldLayout.handleAllianceFlip(new Pose2d(new Translation2d(
					7.620832920074463, 1.069169521331787), Rotation2d.fromDegrees(90)), RobotConstants.isRedAlliance),
					Units.Inches.of(10.0), Units.Degrees.of(20.0)
				),
				new PIDToPoseCommand(drive, superstructure, FieldLayout.handleAllianceFlip(new Pose2d(new Translation2d(
					7.620832920074463, 2.8354833126068115), Rotation2d.fromDegrees(90)), RobotConstants.isRedAlliance),
					Units.Inches.of(10.0), Units.Degrees.of(20.0)
				),

				new PIDToPoseCommand(drive, superstructure, rightNeutralToTrench.getInitialPose().get(), Units.Inches.of(10.0), Units.Degrees.of(20.0))
			), superstructure.runIntakeIfDeployed()),
			rightNeutralToTrench.cmd(),
			cmdWithAccuracy(rightTrenchToShoot),
			Commands.sequence(
							Commands.parallel(superstructure.shootRun(),
							superstructure.hoodRun(),
							superstructure.shootWhenReady())).withTimeout(AutoConstants.shootAllFuelTime)
			//superstructure.climb()
        );


	}
}