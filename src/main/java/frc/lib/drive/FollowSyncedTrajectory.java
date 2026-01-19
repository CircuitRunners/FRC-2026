package frc.lib.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.lib.util.SynchronousPIDF;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.superstructure.Superstructure;

public class FollowSyncedTrajectory extends FollowTrajectoryCommand{
    private Superstructure superstructure;
    public FollowSyncedTrajectory(
            Drive drive,
            Superstructure superstructure,
			Trajectory trajectory,
			Distance epsilonDist,
			Angle epsilonAngle,
			Time delayTime,
			Time lookaheadTime,
			Rotation2d targetRotation,
			boolean isAuto) {
		super(drive, trajectory, epsilonDist, epsilonAngle, delayTime, lookaheadTime, targetRotation);
        this.superstructure = superstructure;
		this.superstructure.setSuperstructureDone(false);
		this.superstructure.setDriveReady(false);
	}

    public FollowSyncedTrajectory(Drive drive, Superstructure superstructure, Trajectory trajectory) {
		super(drive, trajectory);
        this.superstructure = superstructure;
		superstructure.setSuperstructureDone(false);
		superstructure.setDriveReady(false);
	}

	public FollowSyncedTrajectory(Drive drive, Superstructure superstructure, Trajectory trajectory, Rotation2d targetRotation) {
		super(drive, trajectory, targetRotation);
        this.superstructure = superstructure;
		superstructure.setSuperstructureDone(false);
		superstructure.setDriveReady(false);
	}

	public FollowSyncedTrajectory(Drive drive, Superstructure superstructure, Trajectory trajectory, SynchronousPIDF translationController) {
		super(drive, trajectory, translationController);
        this.superstructure = superstructure;
		superstructure.setSuperstructureDone(false);
		superstructure.setDriveReady(false);
	}

    // public FollowSyncedTrajectory(Drive drive, Superstructure superstructure, Trajectory trajectory) {
	// 	super(drive,
	// 			trajectory,
	// 			trajectory
	// 					.getStates()
	// 					.get(trajectory.getStates().size() - 1)
	// 					.poseMeters
	// 					.getRotation());
    //     this.superstructure = superstructure;
	// 	superstructure.setSuperstructureDone(false);
	// 	superstructure.setDriveReady(false);
	// }

    public boolean driveDone() {
		return super.isFinished();
	}

    @Override
	public void execute() {
		super.execute();
		superstructure.setDriveReady(driveDone());
	}

	@Override
	public boolean isFinished() {
		return superstructure.getSuperstructureDone();
	}

    
}
