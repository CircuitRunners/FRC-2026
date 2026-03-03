package frc.robot.auto.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

public class LeftNeutralClimb extends AutoModeBase {

	public LeftNeutralClimb(Drive drive, Superstructure superstructure, AutoFactory factory) {
		super(drive, superstructure, factory, "Left Neutral Cycle + Climb");

		AutoTrajectory leftTrenchToNeutral = trajectory("leftTrenchToNeutral");
		AutoTrajectory leftNeutralToTrench = trajectory("leftNeutralToTrench");
		AutoTrajectory leftTrenchToShoot = trajectory("leftTrenchToShoot");

		Pose2d startPose = leftTrenchToNeutral.getInitialPose().get();

		superstructure.updateSide(ObjectPoseEstimator.INTAKE_SIDE.LEFT);


		prepRoutine(
			AutoHelpers.resetPoseIfWithoutEstimate(startPose, drive),
			Commands.parallel(leftTrenchToNeutral.cmd(), superstructure.deployIntake()),
			Commands.deadline(
				superstructure.collectFuel(leftNeutralToTrench.getInitialPose().get()).withTimeout(7),
				superstructure.runIntakeIfDeployed()),
			leftNeutralToTrench.cmd(),
			cmdWithAccuracy(leftTrenchToShoot),
			Commands.sequence(
							Commands.parallel(superstructure.shootRun(),
							superstructure.hoodRun(),
							superstructure.shootWhenReady())).withTimeout(AutoConstants.shootAllFuelTime),
			superstructure.climb()
        );


	}
}