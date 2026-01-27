package frc.robot.auto;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.Stopwatch;
import frc.robot.auto.AutoConstants.AutoType;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;

import frc.lib.drive.PIDToPoseCommand;
public class AutoModeBase {
    private static AutoRoutine routine;
	private static Stopwatch stopwatch = new Stopwatch();
	private static AutoType side;

	public AutoModeBase(AutoFactory factory, String name) {
		routine = factory.newRoutine(name);
	}

	public AutoModeBase(AutoFactory factory, String name, AutoType side) {
		this(factory, name);
		AutoModeBase.side = side;
	}

	/**
	 * @return Trajectory from choreo
	 */
	public AutoTrajectory trajectory(String name) {
		return routine.trajectory(name);
	}

	public AutoTrajectory trajectory(String name, int index) {
		return routine.trajectory(name, index);
	}

	public Command netAutoScoreWithPrep(Pose2d pose, Drive drive, Superstructure superstructure) {
		return Commands.sequence(
			new PIDToPoseCommand(
							drive, superstructure, pose)
			);

	}

	public void prepRoutine(Command... sequence) {
		routine.active()
				.onTrue(Commands.sequence(sequence)
						.withName("Auto Routine Sequential Command Group"));
	}

	public AutoRoutine getRoutine() {
		return routine;
	}
}
