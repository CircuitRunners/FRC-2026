package frc.robot.subsystems.intakeRollers;

import frc.lib.bases.MotorSubsystem;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIO.Setpoint;

public class IntakeRollers extends MotorSubsystem<MotorIOTalonFX> {
    public static final Setpoint IDLE = Setpoint.withNeutralSetpoint();
    public static final Setpoint INTAKE = Setpoint.withVoltageSetpoint(IntakeRollerConstants.kIntakeVoltage);
	public static final Setpoint EXHAUST = Setpoint.withVoltageSetpoint(IntakeRollerConstants.kOuttakeVoltage);
	public static final Setpoint PELICAN = Setpoint.withVoltageSetpoint(IntakeRollerConstants.kPeliVoltage);
	public static final Setpoint START = Setpoint.withVoltageSetpoint(IntakeRollerConstants.kStartVoltage);

    public IntakeRollers() {
        super(IntakeRollerConstants.getMotorIO(), "Intake Rollers");
    }
}
