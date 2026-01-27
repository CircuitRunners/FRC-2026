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
import frc.robot.auto.AutoHelpers;
import frc.robot.auto.AutoModeBase;
import frc.robot.subsystems.superstructure.Superstructure;


public class NetGHIJKL extends AutoModeBase {
	public NetGHIJKL(Drive drive, Superstructure superstructure, AutoFactory factory) {
		super(factory, "Net GH IJ KL");


		Pose2d startPose = FieldLayout.handleAllianceFlip(
				// new Pose2d(10.17, 7.17, Rotation2d.kZero),
                drive.getPose(),
				RobotConstants.isRedAlliance);
		Pose2d firstAlignPose = FieldLayout.handleAllianceFlip(
				new Pose2d(
						FieldLayout.kFieldLength
								.div(2.0)
								.minus(Units.Inches.of(29.5))
								.in(Units.Meters),
						5.0,
						Rotation2d.kZero),
				RobotConstants.isRedAlliance);
		Pose2d secondAlignPose = FieldLayout.handleAllianceFlip(
				new Pose2d(
						FieldLayout.kFieldLength
								.div(2.0)
								.minus(Units.Inches.of(28.0))
								.in(Units.Meters),
						6.4,
						Rotation2d.k180deg),
				RobotConstants.isRedAlliance);
		Pose2d thirdAlignPose = FieldLayout.handleAllianceFlip(
				new Pose2d(
						FieldLayout.kFieldLength
								.div(2.0)
								.minus(Units.Inches.of(26.0))
								.in(Units.Meters),
						6.6,
						Rotation2d.kZero),
				RobotConstants.isRedAlliance);

		Pose2d endingPose = new Pose2d(
				thirdAlignPose.getMeasureX().plus(Units.Inches.of(RobotConstants.isRedAlliance ? -4.0 : 4.0)),
				thirdAlignPose.getMeasureY(),
				thirdAlignPose.getRotation());

		AutoTrajectory ghAlgaeToNet = trajectory("ghAlgaeToNet");
		AutoTrajectory netToIJAlgae = trajectory("netToIJAlgae");
		AutoTrajectory ijAlgaeToNet = trajectory("ijAlgaeToCenterNet");
		AutoTrajectory netToKLAlgae = trajectory("netToKLAlgae");
		AutoTrajectory klAlgaeToNet = trajectory("klAlgaeToNet");

		prepRoutine(
            Commands.sequence(
				AutoHelpers.resetPoseIfWithoutEstimate(startPose, drive),
				ghAlgaeToNet
						.cmd()
						.deadlineFor(Commands.waitSeconds(0.5)
								)
						.andThen(netAutoScoreWithPrep(firstAlignPose, drive, superstructure))
				)

        );
	}
}