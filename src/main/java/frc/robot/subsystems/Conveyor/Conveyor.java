package frc.robot.subsystems.Conveyor;

    import frc.lib.bases.MotorSubsystem;
    import frc.lib.io.MotorIOTalonFX;
    import frc.lib.io.MotorIO.Setpoint;

public class Conveyor extends MotorSubsystem<MotorIOTalonFX>{

        public static final Setpoint IDLE = Setpoint.withNeutralSetpoint();
        public static final Setpoint FEED_FORWARD = Setpoint.withVoltageSetpoint(ConveyorConstants.kFeedForwardVoltage);
        public static final Setpoint FEED_BACKWARDS = Setpoint.withVoltageSetpoint(ConveyorConstants.kFeedBackwardVoltage);
        public static final Setpoint HOLD_PIECE = Setpoint.withVoltageSetpoint(ConveyorConstants.kHoldVoltage);
        public static final Setpoint START = Setpoint.withVoltageSetpoint(ConveyorConstants.kStartVoltage);
    
public Conveyor() {
    super(ConveyorConstants.getMotorIO(), "Conveyor");
        }
    }

