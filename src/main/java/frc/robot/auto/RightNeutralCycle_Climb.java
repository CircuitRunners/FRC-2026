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
		AutoTrajectory rightSideToRightNeutral = trajectory("Right_Side_to_Right_Neutral");
		AutoTrajectory rightNeutralToRightShoot = trajectory("Right_Neutral_to_Right_Shoot");


		prepRoutine(
			Commands.parallel(rightSideToRightNeutral.cmd(), superstructure.deployIntake()),
            Commands.deadline(superstructure.collectFuelCommand(), superstructure.runIntakeIfDeployed()),
			rightNeutralToRightShoot.cmd(),
			Commands.deadline(Commands.waitSeconds(AutoConstants.shootAllFuelTime), Commands.parallel(superstructure.shootWhenReady(), drive.brake())),
			superstructure.climb()
        );
	}
}