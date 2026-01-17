package frc.robot.shooting;

import frc.lib.util.team254_2022.InterpolatingDouble;
import frc.lib.util.team254_2022.InterpolatingTreeMap;

public class RegressionMaps {
	public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kHoodAutoAimMap =
			new InterpolatingTreeMap<>();

	static {
		for (double[] pair : HubRegression.kHoodManualAngle) {
			kHoodAutoAimMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
		}
	}

	public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kFlywheelAutoAimMap =
			new InterpolatingTreeMap<>();

	static {
		for (double[] pair : HubRegression.kFlywheelManualRPM) {
			kFlywheelAutoAimMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
		}
	}

	public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kShooterCurveOffsetMap =
			new InterpolatingTreeMap<>();

}