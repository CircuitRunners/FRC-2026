package frc.robot;

import com.ctre.phoenix6.CANBus;

import choreo.auto.AutoFactory;

//import choreo.auto.AutoFactory;

public class RobotConstants {
	public static String kSerial;
	public static boolean isComp;
	public static boolean isOmega;

	static {
		RobotConstants.isRedAlliance = false;
	}

	public static boolean isRedAlliance;
	public static AutoFactory mAutoFactory;
	public static CANBus superstructureBus = new CANBus("superstructure");
}