package frc.robot.subsystems.kicker;

    import frc.lib.bases.MotorSubsystem;
    import frc.lib.io.MotorIOTalonFX;
    import frc.lib.io.MotorIO.Setpoint;

public class Kicker extends MotorSubsystem<MotorIOTalonFX>{

    public static final Setpoint IDLE = Setpoint.withNeutralSetpoint();
    public static final Setpoint FEED_FORWARD = Setpoint.withVoltageSetpoint(KickerConstants.kFeedForwardVoltage);
    public static final Setpoint FEED_BACKWARDS = Setpoint.withVoltageSetpoint(KickerConstants.kFeedBackwardVoltage);
    
        public Kicker() {
        super(KickerConstants.getMotorIO(), "Kicker");
            }
    }

