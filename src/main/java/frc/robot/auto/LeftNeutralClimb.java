package frc.robot.auto;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.drive.PIDToPoseCommand;
import frc.lib.util.FieldLayout;
import frc.robot.RobotConstants;
import frc.robot.auto.AutoHelpers;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.auto.AutoModeBase;

public class LeftNeutralClimb extends AutoModeBase {

	public LeftNeutralClimb(Drive drive, Superstructure superstructure, AutoFactory factory) {
		super(drive, superstructure, factory, "left neutral climb auto");

		AutoTrajectory leftTrenchToNeutral = trajectory("leftTrenchToNeutral");
		AutoTrajectory leftNeutralToTrench = trajectory("leftNeutralToTrench");
		AutoTrajectory leftTrenchToShoot = trajectory("leftTrenchToShoot");

		Pose2d startPose =
				FieldLayout.handleAllianceFlip(new Pose2d(4.64, 7.44, Rotation2d.kZero), RobotConstants.isRedAlliance);

		prepRoutine(
				AutoHelpers.resetPoseIfWithoutEstimate(startPose, drive),
				leftTrenchToNeutral.cmd(),
				new PIDToPoseCommand(drive, superstructure, new Pose2d(7.6, 5, new Rotation2d(-1)), Units.Inches.of(12.0), Units.Degrees.of(10.0)),
				new PIDToPoseCommand(drive, superstructure, leftNeutralToTrench.getInitialPose().get(), Units.Inches.of(36.0), Units.Degrees.of(10.0)),
				leftNeutralToTrench.cmd(),
				cmdWithAccuracy(leftTrenchToShoot),
				superstructure.shootWhenReady().withTimeout(3),
				superstructure.climb());


	}
}