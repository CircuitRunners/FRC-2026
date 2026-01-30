package frc.robot.subsystems.Kicker;

    import frc.lib.bases.MotorSubsystem;
    import frc.lib.io.MotorIOTalonFX;
    import frc.lib.io.MotorIO.Setpoint;
    import frc.robot.subsystems.intakeRollers.IntakeRollerConstants;

public class Kicker extends MotorSubsystem<MotorIOTalonFX>{

        public static final Setpoint IDLE = Setpoint.withNeutralSetpoint();
        public static final Setpoint FEED_FORWARD = Setpoint.withVoltageSetpoint(IntakeRollerConstants.kIntakeVoltage);
        public static final Setpoint FEED_BACKWARDS = Setpoint.withVoltageSetpoint(IntakeRollerConstants.kExhaustVoltage);
        public static final Setpoint HOLD_PIECE = Setpoint.withVoltageSetpoint(IntakeRollerConstants.kPelicanVoltage);
        public static final Setpoint START = Setpoint.withVoltageSetpoint(IntakeRollerConstants.kStartVoltage);
    
public Kicker() {
    super(KickerConstants.getMotorIO(), "Kicker");
        }
    }

