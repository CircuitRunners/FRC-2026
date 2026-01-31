package frc.robot.subsystems.shooter;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.bases.FlywheelMotorSubsystem;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIO.Setpoint;
import frc.robot.shooting.ShotCalculator;
import frc.robot.subsystems.drive.Drive;

public class Shooter extends FlywheelMotorSubsystem<MotorIOTalonFX> {
    private Drive drive;

    public static final Setpoint IDLE = Setpoint.withCoastSetpoint();
    public static final Setpoint STOP = Setpoint.withVelocitySetpoint(Units.RotationsPerSecond.of(0));

    public void setShooterVelocity(double RPM) {
        applySetpoint(Setpoint.withVelocitySetpoint(Units.RotationsPerSecond.of(RPM)));
    }

    public void incrementVelocity(double inc) {
        applySetpoint(Setpoint.withVelocitySetpoint(getVelocity().plus(Units.RotationsPerSecond.of(inc))));
    }

    public Command trackTargetCommand(Setpoint setpoint) {
        return followSetpointCommand(() -> setpoint);
    }

    private void stop() {
        applySetpoint(STOP);
    }

    public Command stopCommand() {
        return runOnce(this::stop);
    }

    public Shooter() {
        super(ShooterConstants.getMotorIO(),
            "Shooter", 
            ShooterConstants.kEpsilonThreshold
        );
    }
}
