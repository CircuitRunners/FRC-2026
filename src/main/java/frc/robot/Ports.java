package frc.robot;

import com.ctre.phoenix6.CANBus;

/**
 * Holds ports IDs and canbuses for superstructure (subsystems that are not drivetrain)
 */
public enum Ports {
	INTAKE_DEPLOY(1, RobotConstants.superstructureBus),
	INTAKE_ROLLERS(2, RobotConstants.superstructureBus),
	SHOOTER(3, RobotConstants.superstructureBus),
	SHOOTER_FOLLOWER(4, RobotConstants.superstructureBus),
	HOOD(5, RobotConstants.superstructureBus),

	PHYSICAL_BUTTON(9, new CANBus("rio"));

	public final int id;
	public final CANBus bus;

	private Ports(int id, CANBus bus) {
		this.id = id;
		this.bus = bus;
	}
}