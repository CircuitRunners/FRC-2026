package frc.lib.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.lib.util.FieldLayout.Level;
import frc.lib.util.SynchronousPIDF;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.superstructure.Superstructure;

public class FollowSyncedPIDToPose extends PIDToPoseCommand {
    private final SynchronousPIDF tippyTranslationController;
	private final SynchronousPIDF tippyHeadingController;
	private final Distance distanceToStartSlowing;
	private final Distance distanceToRaiseElevator;
	private final LinearVelocity velToRaiseElevator;

    public FollowSyncedPIDToPose(
			Drive drive, Superstructure superstructure, Pose2d finalPose, Level level, SynchronousPIDF translationController, SynchronousPIDF headingController) {
		super(drive, superstructure, finalPose, level, translationController, headingController);

		getSuperstructure().setSuperstructureDone(false);
		getSuperstructure().setDriveReady(false);
		getSuperstructure().setReadyToRaiseElevator(false);

		tippyTranslationController = DriveConstants.getTippyTranslationControllerForLevel(level);
		tippyHeadingController = DriveConstants.getTippyHeadingControllerForLevel(level);
		distanceToStartSlowing = DriveConstants.getDistanceToStartSlowingForLevel(level);
		distanceToRaiseElevator = DriveConstants.getDistanceToRaiseElevatorForLevel(level);
		velToRaiseElevator = DriveConstants.getVelocityToRaiseForLevel(level);
	}

    public FollowSyncedPIDToPose(Drive drive, Superstructure superstructure, Pose2d finalPose, Level level, SynchronousPIDF translationController) {
		super(drive, superstructure, finalPose, level, translationController);

		getSuperstructure().setSuperstructureDone(false);
		getSuperstructure().setDriveReady(false);
		getSuperstructure().setReadyToRaiseElevator(false);

		tippyTranslationController = DriveConstants.getTippyTranslationControllerForLevel(level);
		tippyHeadingController = DriveConstants.getTippyHeadingControllerForLevel(level);
		distanceToStartSlowing = DriveConstants.getDistanceToStartSlowingForLevel(level);
		distanceToRaiseElevator = DriveConstants.getDistanceToRaiseElevatorForLevel(level);
		velToRaiseElevator = DriveConstants.getVelocityToRaiseForLevel(level);
	}

    public FollowSyncedPIDToPose(Drive drive, Superstructure superstructure, Pose2d finalPose, Level level) {
		super(drive, superstructure, finalPose, level);

		getSuperstructure().setSuperstructureDone(false);
		getSuperstructure().setDriveReady(false);
		getSuperstructure().setReadyToRaiseElevator(false);

		tippyTranslationController = DriveConstants.getTippyTranslationControllerForLevel(level);
		tippyHeadingController = DriveConstants.getTippyHeadingControllerForLevel(level);
		distanceToStartSlowing = DriveConstants.getDistanceToStartSlowingForLevel(level);
		distanceToRaiseElevator = DriveConstants.getDistanceToRaiseElevatorForLevel(level);
		velToRaiseElevator = DriveConstants.getVelocityToRaiseForLevel(level);
	}

    public FollowSyncedPIDToPose(Drive drive, Superstructure superstructure, Pose2d rawEndPose, Level level, boolean diffParam) {
		super(drive, superstructure, rawEndPose, level, diffParam);

		getSuperstructure().setSuperstructureDone(false);
		getSuperstructure().setDriveReady(false);
		getSuperstructure().setReadyToRaiseElevator(false);

		tippyTranslationController = DriveConstants.getTippyTranslationControllerForLevel(level);
		tippyHeadingController = DriveConstants.getTippyHeadingControllerForLevel(level);
		distanceToStartSlowing = DriveConstants.getDistanceToStartSlowingForLevel(level);
		distanceToRaiseElevator = DriveConstants.getDistanceToRaiseElevatorForLevel(level);
		velToRaiseElevator = DriveConstants.getVelocityToRaiseForLevel(level);
	}

    public boolean driveDone() {
		return super.isFinished();
	}

	public boolean closeEnoughToRaiseElevator() {
		return distanceFromEnd().lte(distanceToRaiseElevator);
	}

	public boolean slowEnoughToRaiseElevator() {
		return Math.hypot(
						drive.getState().Speeds.vxMetersPerSecond,
						drive.getState().Speeds.vxMetersPerSecond)
				< velToRaiseElevator.in(Units.MetersPerSecond);
	}

    @Override
	public void execute() {
		boolean shouldStartSlowing = distanceFromEnd().lte(distanceToStartSlowing);
		if (shouldStartSlowing) {
			drive.getDrivetrain().setControl(DriveConstants.getPIDToPoseRequestUpdater(drive,
							finalPose, tippyTranslationController, tippyHeadingController)
					.apply(DriveConstants.PIDToPoseRequest));
		} else {
			super.execute();
		}
		getSuperstructure().setDriveReady(driveDone());
		boolean closeEnoughToRaiseElevator = closeEnoughToRaiseElevator();
		boolean slowEnoughToRaiseElevator = slowEnoughToRaiseElevator();
		getSuperstructure().setReadyToRaiseElevator(closeEnoughToRaiseElevator && slowEnoughToRaiseElevator);
	}

	@Override
	public boolean isFinished() {
		return getSuperstructure().getSuperstructureDone();
	}
}
