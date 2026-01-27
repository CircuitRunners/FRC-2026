package frc.robot.subsystems.intakeDeploy;

import frc.lib.bases.ServoMotorSubsystem;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.io.MotorIOTalonFX;

public class IntakeDeploy extends ServoMotorSubsystem<MotorIOTalonFX> {
    
    public static final Setpoint STOW_FULL = Setpoint.withMotionMagicSetpoint(IntakeDeployConstants.kFullStowPosition);
	public static final Setpoint STOW_CLEAR = Setpoint.withMotionMagicSetpoint(IntakeDeployConstants.kStowClearPosition);
	public static final Setpoint DEPLOY = Setpoint.withMotionMagicSetpoint(IntakeDeployConstants.kDeployPosition);
	public static final Setpoint EXHAUST = Setpoint.withMotionMagicSetpoint(IntakeDeployConstants.kExhaustPosition);
	public static final Setpoint INDEXERHOLD = Setpoint.withMotionMagicSetpoint(IntakeDeployConstants.kIndexerHold);

    public IntakeDeploy() {
        super(
            IntakeDeployConstants.getMotorIO(),
            "Intake Deploy",
            IntakeDeployConstants.kEpsilonThreshold,
            IntakeDeployConstants.getServoHomingConfig()
        );
        setCurrentPosition(IntakeDeployConstants.kFullStowPosition);
        applySetpoint(STOW_CLEAR);
    }

    @Override
    public void outputTelemetry() {

    }
}
