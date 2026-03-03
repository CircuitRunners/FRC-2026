package frc.robot.auto.autos;

import java.util.ArrayList;
import java.util.List;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.drive.PIDToPoseCommand;
import frc.lib.util.FieldLayout;
import frc.robot.subsystems.drive.Drive;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.auto.AutoConstants;
import frc.robot.auto.AutoHelpers;
import frc.robot.auto.AutoModeBase;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.vision.objectdetection.ObjectPoseEstimator;


public class RightNeutralClimb extends AutoModeBase {
	public RightNeutralClimb(Drive drive, Superstructure superstructure, AutoFactory factory) {
		super(drive, superstructure, factory, "Right Neutral Cycle + Climb");
		AutoTrajectory rightTrenchToNeutral = trajectory("rightTrenchToNeutral");
		AutoTrajectory rightNeutralToTrench = trajectory("rightNeutralToTrench");
		AutoTrajectory rightTrenchToShoot = trajectory("rightTrenchToShoot");
		Pose2d startPose = rightTrenchToNeutral.getInitialPose().get();
		superstructure.updateSide(ObjectPoseEstimator.INTAKE_SIDE.RIGHT);


		prepRoutine(
			AutoHelpers.resetPoseIfWithoutEstimate(startPose, drive),
			Commands.parallel(rightTrenchToNeutral.cmd(), superstructure.deployIntake()),
			Commands.deadline(
				superstructure.collectFuel(rightNeutralToTrench.getInitialPose().get()).withTimeout(7),
				superstructure.runIntakeIfDeployed()),
			rightNeutralToTrench.cmd(),
			cmdWithAccuracy(rightTrenchToShoot),
			Commands.sequence(
							Commands.parallel(superstructure.shootRun(),
							superstructure.hoodRun(),
							superstructure.shootWhenReady())).withTimeout(AutoConstants.shootAllFuelTime),
			superstructure.climb()
        );


	}
}