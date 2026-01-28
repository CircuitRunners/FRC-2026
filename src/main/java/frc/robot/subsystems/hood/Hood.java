package frc.robot.subsystems.hood;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.bases.ServoMotorSubsystem;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIO.Setpoint;
import frc.robot.shooting.ShotCalculator;
import frc.robot.subsystems.drive.Drive;

public class Hood extends ServoMotorSubsystem<MotorIOTalonFX>{
    private Drive drive;

    public static final Setpoint KITBOT = Setpoint.withMotionMagicSetpoint(HoodConstants.kKitbotPosition);
    public static final Setpoint ZERO = Setpoint.withMotionMagicSetpoint(HoodConstants.kMinAngle);

    public void setHoodAngle(double degrees) {
        applySetpoint(Setpoint.withMotionMagicSetpoint(Units.Degrees.of(degrees)));
    }

    public void incrementHoodAngle(double inc) {
        applySetpoint(Setpoint.withMotionMagicSetpoint(Units.Degrees.of(getPosition().in(Units.Degrees) + inc)));
    }

    public Command trackTargetCommand() {
        return followSetpointCommand(() -> Setpoint.withMotionMagicSetpoint(Units.Degrees.of(ShotCalculator.getInstance(drive).getParameters().hoodAngle())));
    }

    public Hood() {
        super(
            HoodConstants.getMotorIO(),
            "Hood",
            HoodConstants.kEpsilonThreshold,
            HoodConstants.getServoHomingConfig()
        );
        setCurrentPosition(HoodConstants.kMinAngle);
    }
}
