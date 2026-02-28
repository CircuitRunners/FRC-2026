package frc.lib.bases;

import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.lib.io.MotorIO;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.util.Util;

/**
 * Base subsystem for any subsystem that uses motors and requires precise velocity control.
 */
public class FlywheelMotorSubsystem<IO extends MotorIO> extends MotorSubsystem<IO> {
	protected final AngularVelocity epsilonThreshold;

	/**
	 * Creates a FlywheelMotorSubsystem with a MotorIO, name for telemetry, and threshold for differences in measurement.
	 *
	 * @param io MotorIO for the subsystem.
	 * @param name Name for telemetry.
	 * @param epsilonThreshold Acceptable error range for velocity.
	 */
	public FlywheelMotorSubsystem(IO io, String name, AngularVelocity epsilonThreshold) {
		super(io, name);
		this.epsilonThreshold = epsilonThreshold;
	}

	public void periodic() {
		super.periodic();
	}

	/**
	 * Gets whether or not the subsystem is within an acceptable threshold of a provided velocity.
	 *
	 * @param velocity Velocity to check proximity to.
	 * @return Whether the subsystem is acceptably near the given velocity.
	 */
	public boolean nearVelocity(AngularVelocity velocity) {
		return Util.epsilonEquals(
				velocity.baseUnitMagnitude(), getVelocity().baseUnitMagnitude(), epsilonThreshold.baseUnitMagnitude());
	}

	/**
	 * Gets whether or not the subsystem is within an acceptable threshold of it's velocity setpoint.
	 *
	 * @return Whether the subsystem is acceptably near it's setpoint's velocity. Returns false if not in velcity coontrol mode.
	 */
	public boolean spunUp() {
		return nearVelocity(BaseUnits.AngleUnit.per(BaseUnits.TimeUnit).of(io.getSetpoint().baseUnits))
				&& io.getSetpoint().mode.isVelocityControl();
	}

	/**
	 * Gets the current setpoint value of the MotorIO using units of the MotorIO.
	 *
	 * @return Setpoint in mechanism units.
	 */
	public double getSetpointDoubleInUnits() {
		return io.getSetpointDoubleInUnits();
	}

	@Override
	public void applySetpoint(Setpoint setpoint) {
		super.applySetpoint(setpoint);
	}

		/**
	 * Enables or disables soft limits.
	 *
	 * @param enable True to enable, soft to disable.
	 */
	public void useSoftLimits(boolean enable) {
		io.useSoftLimits(enable);
	}

	/**
	 * Sets neutral mode to brake or coast.
	 *
	 * @param wantsBrake True for brake, false for coast.
	 */
	public void setNeutralBrake(boolean wantsBrake) {
		io.setNeutralBrake(wantsBrake);
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		super.initSendable(builder);
		io.initSendable(builder);
	}

}