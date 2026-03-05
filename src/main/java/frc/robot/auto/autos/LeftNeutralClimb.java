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
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.vision.objectdetection.ObjectPoseEstimator;
import frc.robot.auto.AutoModeBase;

public class LeftNeutralClimb extends AutoModeBase {

	public LeftNeutralClimb(Drive drive, Superstructure superstructure, AutoFactory factory) {
		super(drive, superstructure, factory, "Left Neutral Cycle + Climb");

		AutoTrajectory leftTrenchToNeutral = trajectory("leftTrenchToNeutral");
		AutoTrajectory leftNeutralToFuel = trajectory("leftNuetralToFuel");
		AutoTrajectory leftNeutralToTrench = trajectory("leftNeutralToTrench");
		AutoTrajectory leftTrenchToShoot = trajectory("leftTrenchToShoot");

		Pose2d startPose = leftTrenchToNeutral.getInitialPose().get();

		//superstructure.updateSide(ObjectPoseEstimator.INTAKE_SIDE.LEFT);


		prepRoutine(
			AutoHelpers.resetPoseIfWithoutEstimate(startPose, drive),
			Commands.parallel(leftTrenchToNeutral.cmd(), superstructure.deployIntake()),
			Commands.deadline(
			Commands.sequence(
				//superstructure.collectFuel(leftNeutralToTrench.getInitialPose().get()).withTimeout(7),
				// leftNeutralToFuel.cmd(),
				new PIDToPoseCommand(drive, superstructure, FieldLayout.handleAllianceFlip(new Pose2d(new Translation2d(
					7.620832920074463, FieldLayout.kFieldWidth.in(Units.Meters) - 1.069169521331787), Rotation2d.fromDegrees(-90)), RobotConstants.isRedAlliance),
					Units.Inches.of(5.0), Units.Degrees.of(20.0)
				),
				new PIDToPoseCommand(drive, superstructure, FieldLayout.handleAllianceFlip(new Pose2d(new Translation2d(
					7.620832920074463, FieldLayout.kFieldWidth.in(Units.Meters) - 2.8354833126068115), Rotation2d.fromDegrees(-90)), RobotConstants.isRedAlliance),
					Units.Inches.of(10.0), Units.Degrees.of(20.0),
					DriveConstants.getIntakeAutoAlignTranslationController()
				),

				new PIDToPoseCommand(drive, superstructure, leftNeutralToTrench.getInitialPose().get(), Units.Inches.of(10.0), Units.Degrees.of(20.0))
			), superstructure.runIntakeIfDeployed()),
			leftNeutralToTrench.cmd(),
			cmdWithAccuracy(leftTrenchToShoot),
			Commands.sequence(
							Commands.parallel(superstructure.shootRun(),
							superstructure.hoodRun(),
							superstructure.shootWhenReady())).withTimeout(AutoConstants.shootAllFuelTime)
			//superstructure.climb()
        );


	}
}