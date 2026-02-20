package frc.robot.auto;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.drive.PIDToPoseCommand;
import frc.lib.util.FieldLayout;
import frc.robot.subsystems.drive.Drive;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.auto.AutoHelpers;
import frc.robot.auto.AutoModeBase;
import frc.robot.subsystems.superstructure.Superstructure;


public class RightNeutralCycle_Climb extends AutoModeBase {
	RobotContainer r;
	public RightNeutralCycle_Climb(Drive drive, Superstructure superstructure, AutoFactory factory) {
		super(factory, "Right Neutral Cycle + Climb");
		this.r = r;
		AutoTrajectory rightTrenchToNeutral = trajectory("rightTrenchToNeutral");
		AutoTrajectory rightNeutralToTrench = trajectory("rightNeutralToTrench");
		AutoTrajectory rightTrenchToShoot = trajectory("rightTrenchToShoot");
		Pose2d startPose = FieldLayout.handleAllianceFlip(
				new Pose2d(4.622, .652, Rotation2d.kZero),
				RobotConstants.isRedAlliance);


		prepRoutine(
			AutoHelpers.resetPoseIfWithoutEstimate(startPose, drive),
			Commands.parallel(rightTrenchToNeutral.cmd(), superstructure.deployIntake()),
            Commands.deadline(superstructure.collectFuelCommand().withTimeout(5), superstructure.runIntakeIfDeployed()),
			superstructure.driveToNeutralTrajectory(),
			rightNeutralToTrench.cmd(),
			rightTrenchToShoot.cmd(),
			Commands.parallel(superstructure.shoot(), drive.brake()).withTimeout(5),
			superstructure.climb()
        );
	}
}