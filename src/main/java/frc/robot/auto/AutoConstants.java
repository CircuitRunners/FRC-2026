package frc.robot.auto;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

public class AutoConstants {
    public static final double kPThetaController = 5.0;
    public static final double kPXYController = 5.0;
	public static final Distance kAutoLinearEpsilon = Units.Centimeters.of(4.0);
	public static final Angle kAutoAngleEpsilon = Units.Degrees.of(1.0);
	public static final Time kDelayTime = Units.Milliseconds.of(80);
	public static final Time kDefaultTrajectoryTimeout = Units.Seconds.of(1.0);

    public static enum AutoType {
		LEFT,
		RIGHT,
		MARK
	}

	public static enum AutoEndBehavior {
		ALGAE_GRAB,
		ALGAE_DRIVE
	}
}