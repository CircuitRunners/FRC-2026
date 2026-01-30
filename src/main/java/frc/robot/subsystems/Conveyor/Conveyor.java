package frc.robot.subsystems.Conveyor;

    import frc.lib.bases.MotorSubsystem;
    import frc.lib.io.MotorIOTalonFX;
    import frc.lib.io.MotorIO.Setpoint;
    import frc.robot.subsystems.intakeRollers.IntakeRollerConstants;

public class Conveyor extends MotorSubsystem<MotorIOTalonFX>{

        public static final Setpoint IDLE = Setpoint.withNeutralSetpoint();
        public static final Setpoint FEED_FORWARD = Setpoint.withVoltageSetpoint(IntakeRollerConstants.kIntakeVoltage);
        public static final Setpoint FEED_BACKWARDS = Setpoint.withVoltageSetpoint(IntakeRollerConstants.kExhaustVoltage);
        public static final Setpoint HOLD_PIECE = Setpoint.withVoltageSetpoint(IntakeRollerConstants.kPelicanVoltage);
        public static final Setpoint START = Setpoint.withVoltageSetpoint(IntakeRollerConstants.kStartVoltage);
    
public Conveyor() {
    super(ConveyorConstants.getMotorIO(), "Conveyor");
        }
    }

