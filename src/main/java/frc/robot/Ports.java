package frc.robot;

/**
 * Holds ports IDs and canbuses for superstructure (subsystems that are not drivetrain)
 * Do NOT use ports 0 through 7.
 */

public enum Ports {
	RANDOM_PORT(9, "RioDigitalIn");

	public final int id;
	public final String bus;

	private Ports(int id, String bus) {
		this.id = id;
		this.bus = bus;
	}
}