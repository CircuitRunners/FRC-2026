package frc.robot.shooting;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;

public class RegressionMaps {
	public static InterpolatingTreeMap<Double, Double> kHoodAutoAimMap =
			new InterpolatingDoubleTreeMap();

	static {
		for (double[] pair : HubRegression.kHoodManualAngle) {
			kHoodAutoAimMap.put(pair[0], pair[1]);
		}
	}

	public static InterpolatingTreeMap<Double, Double> kFlywheelAutoAimMap =
			new InterpolatingDoubleTreeMap();

	static {
		for (double[] pair : HubRegression.kFlywheelManualRPM) {
			kFlywheelAutoAimMap.put(pair[0], pair[1]);
		}
	}

	public static InterpolatingTreeMap<Double, Double> kTimeOfFlightMap =
			new InterpolatingDoubleTreeMap();

	static {
		for (double[] pair : HubRegression.ktimeOfFlightMap) {
			kTimeOfFlightMap.put(pair[0], pair[1]);
		}
	}
}