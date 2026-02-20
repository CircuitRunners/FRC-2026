package frc.robot.subsystems.intakeDeploy;

import frc.lib.bases.ServoMotorSubsystem;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.io.MotorIOTalonFX;

public class IntakeDeploy extends ServoMotorSubsystem<MotorIOTalonFX> {
    
    public static final Setpoint STOW = Setpoint.withMotionMagicSetpoint(IntakeDeployConstants.kStowPosition);
	public static final Setpoint DEPLOY = Setpoint.withMotionMagicSetpoint(IntakeDeployConstants.kDeployPosition);
	public static final Setpoint EXHAUST = Setpoint.withMotionMagicSetpoint(IntakeDeployConstants.kExhaustPosition);

    public IntakeDeploy() {
        super(
            IntakeDeployConstants.getMotorIO(),
            "Intake Deploy",
            IntakeDeployConstants.kEpsilonThreshold,
            IntakeDeployConstants.getServoHomingConfig()
        );
        setCurrentPosition(IntakeDeployConstants.kStowPosition);
        applySetpoint(STOW);
    }

    @Override
    public void outputTelemetry() {

    }
}
