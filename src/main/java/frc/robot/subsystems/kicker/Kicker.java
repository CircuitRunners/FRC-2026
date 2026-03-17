package frc.robot.subsystems.kicker;

import frc.lib.bases.FlywheelMotorSubsystem;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIO.Setpoint;

public class Kicker extends FlywheelMotorSubsystem<MotorIOTalonFX>{

    public static final Setpoint IDLE = Setpoint.withNeutralSetpoint();
    public static final Setpoint FEED_FORWARD = Setpoint.withVoltageSetpoint(KickerConstants.kFeedForwardVoltage);
    public static final Setpoint FEED_BACKWARDS = Setpoint.withVoltageSetpoint(KickerConstants.kFeedBackwardVoltage);

    public static final Setpoint VELOCITY_FORWARD = Setpoint.withVelocitySetpoint(KickerConstants.kFeedForwardVelocity);
    public static final Setpoint VELOCITY_BACKWARD = Setpoint.withVelocitySetpoint(KickerConstants.kFeedBackwardVelocity);

    public static final Setpoint JUGGLE = Setpoint.withVoltageSetpoint(KickerConstants.kJuggleVoltage);
    
        public Kicker() {
        super(
            KickerConstants.getMotorIO(),
            "Kicker",
            KickerConstants.kEpsilonThreshold);
        }
    }

