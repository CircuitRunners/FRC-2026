package frc.lib.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.lib.util.SynchronousPIDF;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.superstructure.Superstructure;

public class FollowSyncedPIDToPose extends PIDToPoseCommand {
    private final SynchronousPIDF translationController = DriveConstants.getAutoAlignTranslationController();
	private final SynchronousPIDF headingController = DriveConstants.getAutoAlignHeadingController();

    public FollowSyncedPIDToPose(
			Drive drive, Superstructure superstructure, Pose2d finalPose, SynchronousPIDF translationController, SynchronousPIDF headingController) {
		super(drive, superstructure, finalPose, translationController, headingController);

		getSuperstructure().setSuperstructureDone(false);
		getSuperstructure().setDriveReady(false);
		getSuperstructure().setReadyToRaiseElevator(false);
	}

    public FollowSyncedPIDToPose(Drive drive, Superstructure superstructure, Pose2d finalPose, SynchronousPIDF translationController) {
		super(drive, superstructure, finalPose, translationController);

		getSuperstructure().setSuperstructureDone(false);
		getSuperstructure().setDriveReady(false);
		getSuperstructure().setReadyToRaiseElevator(false);
	}

    public FollowSyncedPIDToPose(Drive drive, Superstructure superstructure, Pose2d finalPose) {
		super(drive, superstructure, finalPose);

		getSuperstructure().setSuperstructureDone(false);
		getSuperstructure().setDriveReady(false);
		getSuperstructure().setReadyToRaiseElevator(false);
	}

    public FollowSyncedPIDToPose(Drive drive, Superstructure superstructure, Pose2d rawEndPose, boolean diffParam) {
		super(drive, superstructure, rawEndPose, diffParam);

		getSuperstructure().setSuperstructureDone(false);
		getSuperstructure().setDriveReady(false);
		getSuperstructure().setReadyToRaiseElevator(false);
	}

    public boolean driveDone() {
		return super.isFinished();
	}

    @Override
	public void execute() {
		super.execute();
		getSuperstructure().setDriveReady(driveDone());
	}

	@Override
	public boolean isFinished() {
		return getSuperstructure().getSuperstructureDone();
	}
}
