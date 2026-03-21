package frc.robot.auto;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.robot.subsystems.drive.DriveConstants;

public class AutoConstants {
    public static final double kPThetaController = 5.0;
    public static final double kPXYController = 5.0;
	public static final Distance kAutoLinearEpsilon = Units.Centimeters.of(4.0);
	public static final Angle kAutoAngleEpsilon = Units.Degrees.of(1.0);
	public static final Time kDelayTime = Units.Milliseconds.of(80);
	public static final Time kDefaultTrajectoryTimeout = Units.Seconds.of(1.0);

    public static enum AutoType {
	}

	public static enum AutoEndBehavior {
	}
	public static double shootAllFuelTime = 4;

	public static TrajectoryConfig intakeConfig = new TrajectoryConfig(DriveConstants.kIntakeMaxSpeed, DriveConstants.kMaxAcceleration)
		.setEndVelocity(DriveConstants.kIntakeMaxSpeed);
	public static TrajectoryConfig regularConfig = new TrajectoryConfig(DriveConstants.kMaxSpeed, DriveConstants.kMaxAcceleration)
		.setEndVelocity(DriveConstants.kIntakeMaxSpeed);


}