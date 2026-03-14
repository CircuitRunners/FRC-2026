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

public class LeftDoubleNeutral extends AutoModeBase {

	public LeftDoubleNeutral(Drive drive, Superstructure superstructure, AutoFactory factory) {
		super(drive, superstructure, factory, "Left Double Neutral");

		AutoTrajectory leftIntakeToShoot = trajectory("leftIntakeToShoot");
		AutoTrajectory leftIntakeToShoot2 = trajectory("leftIntakeToShoot2");


		AutoTrajectory leftTrenchToNeutralIntake = trajectory("leftTrenchToNeutralIntake");
		AutoTrajectory leftShootToNeutralIntake = trajectory("leftShootToNeutralIntake");


		Pose2d startPose = leftTrenchToNeutralIntake.getInitialPose().get();

		//superstructure.updateSide(ObjectPoseEstimator.INTAKE_SIDE.left);


		prepRoutine(
			AutoHelpers.resetPoseIfWithoutEstimate(startPose, drive),
			Commands.deadline(
				leftTrenchToNeutralIntake.cmd(),
				Commands.sequence(
					superstructure.deployIntake(),
					superstructure.runIntakeIfDeployed()
				)
			),
			cmdWithAccuracy(leftIntakeToShoot),
			drive.stopDrivetrain(),
			superstructure.shootWhenReadyTeleop().withTimeout(AutoConstants.shootAllFuelTime),
			
			Commands.deadline(
				leftShootToNeutralIntake.cmd(),
				Commands.sequence(
					superstructure.runIntakeIfDeployed()
				)
			),
			cmdWithAccuracy(leftIntakeToShoot2),
			drive.stopDrivetrain(),
			superstructure.shootWhenReadyTeleop().withTimeout(AutoConstants.shootAllFuelTime)
			
		);


	}
}