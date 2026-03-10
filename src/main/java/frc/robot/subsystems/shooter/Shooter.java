package frc.robot.subsystems.shooter;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.bases.FlywheelMotorSubsystem;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIO.Setpoint;
import frc.robot.subsystems.drive.Drive;

public class Shooter extends FlywheelMotorSubsystem<MotorIOTalonFX> {

    public static final Setpoint IDLE = Setpoint.withCoastSetpoint();
    public static final Setpoint STOP = Setpoint.withVelocitySetpoint(Units.RotationsPerSecond.of(0));
    public static final Setpoint KITBOT = Setpoint.withVelocitySetpoint(Units.RotationsPerSecond.of(10.0));

    public Shooter() {
        super(ShooterConstants.getMotorIO(),
            "Shooter", 
            ShooterConstants.kEpsilonThreshold
        );
    }
}
