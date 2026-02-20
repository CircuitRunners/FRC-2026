package frc.robot.subsystems.conveyor;

    import frc.lib.bases.MotorSubsystem;
    import frc.lib.io.MotorIOTalonFX;
    import frc.lib.io.MotorIO.Setpoint;

public class Conveyor extends MotorSubsystem<MotorIOTalonFX>{

        public static final Setpoint IDLE = Setpoint.withNeutralSetpoint();
        public static final Setpoint FEED_FORWARD = Setpoint.withVoltageSetpoint(ConveyorConstants.kFeedForwardVoltage);
        public static final Setpoint FEED_BACKWARDS = Setpoint.withVoltageSetpoint(ConveyorConstants.kFeedBackwardVoltage);
    
public Conveyor() {
    super(ConveyorConstants.getMotorIO(), "Conveyor");
        }
    }

