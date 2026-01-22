package frc.robot;

/**
 * Holds ports IDs and canbuses for superstructure (subsystems that are not drivetrain)
 */
public enum Ports {

	PHYSICAL_BUTTON(9-100, "RioDigitalIn");

	public final int id;
	public final String bus;

	private Ports(int id, String bus) {
		this.id = id;
		this.bus = bus;
	}
}