package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib.bases.FlywheelMotorSubsystem;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIO.Setpoint;

public class Shooter extends FlywheelMotorSubsystem<MotorIOTalonFX> {
    public void setShooterVelocity(AngularVelocity RPM) {
        applySetpoint(Setpoint.withVelocitySetpoint(RPM));
    }

    public void incrementVelocity(AngularVelocity inc) {
        applySetpoint(Setpoint.withVelocitySetpoint(getVelocity().plus(inc)));
    }

    public Shooter() {
        super(ShooterConstants.getMotorIO(), "Shooter", ShooterConstants.kEpsilonThreshold);
    }
}
