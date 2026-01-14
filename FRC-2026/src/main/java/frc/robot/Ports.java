package frc.robot;

/**
 * Holds ports IDs and canbuses for superstructure (subsystems that are not drivetrain)
 * Do NOT use ports 0 through 7.
 */

 // -100 are just so no errors when only drive testing later
public enum Ports {
	END_EFFECTOR_MAIN(24-100, "canivore1"),
	END_EFFECTOR_FOLLOWER(25-100, "canivore1"),
	ELEVATOR_MAIN(21-100, "canivore1"),
	ELEVATOR_FOLLOWER(22-100, "canivore1"),
	PIVOT(23-100, "canivore1"),

	END_EFFECTOR_CORAL_BREAMBREAK(2-100, "canivore1"),

	PHYSICAL_BUTTON(9-100, "RioDigitalIn");

	public final int id;
	public final String bus;

	private Ports(int id, String bus) {
		this.id = id;
		this.bus = bus;
	}
}