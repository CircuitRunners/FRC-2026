package frc.robot;

import com.ctre.phoenix6.CANBus;

/**
 * Holds ports IDs and canbuses for superstructure (subsystems that are not drivetrain)
 */
public enum Ports {
	INTAKE_DEPLOY(1, Bus.CANIVORE_1),
	INTAKE_ROLLERS(2, Bus.CANIVORE_1),
	SHOOTER(3, Bus.CANIVORE_1),

	PHYSICAL_BUTTON(9, Bus.RIO);

	public final int id;
	public final CANBus bus;

	private static final CANBus canivore1 = new CANBus("canivore 1");

	private Ports(int id, Bus bus) {
		this.id = id;
		this.bus = bus.canBus;
	}

	public enum Bus {
        CANIVORE_1("canivore 1"),
        RIO("rio");

        public final CANBus canBus;

        Bus(String name) {
            this.canBus = new CANBus(name);
        }
    }
}