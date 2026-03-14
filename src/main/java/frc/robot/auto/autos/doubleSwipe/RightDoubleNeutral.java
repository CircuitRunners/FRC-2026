package frc.robot.auto.autos.doubleSwipe;

import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

public class RightDoubleNeutral extends AutoModeBase {

	public RightDoubleNeutral(Drive drive, Superstructure superstructure, AutoFactory factory) {
		super(drive, superstructure, factory, "Right Double Neutral");

		AutoTrajectory rightIntakeToShoot = trajectory("rightIntakeToShoot");
		AutoTrajectory rightIntakeToShoot2 = trajectory("rightIntakeToShoot2");


		AutoTrajectory rightTrenchToNeutralIntake = trajectory("rightTrenchToNeutralIntake");
		AutoTrajectory rightShootToNeutralIntake = trajectory("rightShootToNeutralIntake");


		Pose2d startPose = rightTrenchToNeutralIntake.getInitialPose().get();

		//superstructure.updateSide(ObjectPoseEstimator.INTAKE_SIDE.RIGHT);


		prepRoutine(
			AutoHelpers.resetPoseIfWithoutEstimate(startPose, drive),
			Commands.deadline(
				rightTrenchToNeutralIntake.cmd(),
				Commands.sequence(
					superstructure.deployIntake(),
					superstructure.runIntakeIfDeployed()
				)
			),
			cmdWithAccuracy(rightIntakeToShoot),
			drive.stopDrivetrain(),
			superstructure.shootWhenReadyTeleop().withTimeout(AutoConstants.shootAllFuelTime),
			
			Commands.deadline(
				cmdWithAccuracy(rightShootToNeutralIntake),//.cmd(),
				Commands.sequence(
					superstructure.runIntakeIfDeployed()
				)
			),
			cmdWithAccuracy(rightIntakeToShoot2),
			drive.stopDrivetrain(),
			superstructure.shootWhenReadyTeleop().withTimeout(AutoConstants.shootAllFuelTime)
			
		);


	}
}