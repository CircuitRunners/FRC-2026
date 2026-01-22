package frc.robot.auto;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import frc.lib.util.Stopwatch;
import frc.robot.auto.AutoConstants.AutoType;

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
}
