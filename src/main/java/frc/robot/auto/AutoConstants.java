package frc.robot.auto;

public class AutoConstants {
    public static final double kPThetaController = 5.0;
    public static final double kPXYController = 5.0;

    public static enum AutoType {
		LEFT,
		RIGHT,
		MARK
	}

	public static enum AutoEndBehavior {
		ALGAE_GRAB,
		ALGAE_DRIVE
	}
	public static double shootAllFuelTime = 3;
}