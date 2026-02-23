package frc.robot.auto.autos;

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
import frc.robot.auto.AutoConstants;
import frc.robot.auto.AutoHelpers;
import frc.robot.auto.AutoModeBase;
import frc.robot.subsystems.superstructure.Superstructure;


public class RightNeutralClimb extends AutoModeBase {
	public RightNeutralClimb(Drive drive, Superstructure superstructure, AutoFactory factory) {
		super(drive, superstructure, factory, "Right Neutral Cycle + Climb");
		AutoTrajectory rightTrenchToNeutral = trajectory("rightTrenchToNeutral");
		AutoTrajectory rightNeutralToTrench = trajectory("rightNeutralToTrench");
		AutoTrajectory rightTrenchToShoot = trajectory("rightTrenchToShoot");
		Pose2d startPose = FieldLayout.handleAllianceFlip(
				new Pose2d(4.622, .652, Rotation2d.kZero),
				RobotConstants.isRedAlliance);


		prepRoutine(
			AutoHelpers.resetPoseIfWithoutEstimate(startPose, drive),
			Commands.parallel(rightTrenchToNeutral.cmd(), superstructure.deployIntake()),
            //Commands.deadline(superstructure.collectFuelCommand().withTimeout(5), superstructure.runIntakeIfDeployed()),
			new PIDToPoseCommand(drive, superstructure, rightNeutralToTrench.getInitialPose().get(), Units.Inches.of(50.0), Units.Degrees.of(10.0)),
			rightNeutralToTrench.cmd(),
			cmdWithAccuracy(rightTrenchToShoot),
			superstructure.shootWhenReady().withTimeout(AutoConstants.shootAllFuelTime),
			superstructure.climb()
        );
	}
}