package frc.robot.subsystems.intakeDeploy;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import frc.lib.bases.ServoMotorSubsystem;
import frc.lib.io.MotorIO;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.io.MotorIOTalonFX;

public class IntakeDeploy extends ServoMotorSubsystem<MotorIOTalonFX> {
    
    public static final Setpoint FULL_STORE = Setpoint.withMotionMagicSetpoint(IntakeDeployConstants.kFullStoragePos);
    public static final Setpoint EMPTY_STORE = Setpoint.withMotionMagicSetpoint(IntakeDeployConstants.kEmptyStoragePos);
    public static final Setpoint DEPLOY = Setpoint.withMotionMagicSetpoint(IntakeDeployConstants.kDeployPos);
    public static final Setpoint OUTTAKE = Setpoint.withMotionMagicSetpoint(IntakeDeployConstants.kOuttakePos);
    public static final Setpoint HOLD = Setpoint.withMotionMagicSetpoint(IntakeDeployConstants.kHoldPos);

    public static final IntakeDeploy mInstance = new IntakeDeploy();

    private final StructPublisher<Pose3d> poweredBarPublisher = NetworkTableInstance.getDefault()
			.getStructTopic("Mechanisms/Coral Deploy Powered Bar", Pose3d.struct)
			.publish();

	private final StructPublisher<Pose3d> unpoweredBarPublisher = NetworkTableInstance.getDefault()
			.getStructTopic("Mechanisms/Coral Deploy Unpowered Bar", Pose3d.struct)
			.publish();

	private final StructPublisher<Pose3d> mainIntakePublisher = NetworkTableInstance.getDefault()
			.getStructTopic("Mechanisms/Coral Deploy Main", Pose3d.struct)
			.publish();

    public IntakeDeploy() {
        super(
            IntakeDeployConstants.getMotorIO(),
            "Intake Deploy",
            IntakeDeployConstants.kEpsilonThreshold,
            IntakeDeployConstants.getServoHomingConfig()
        );
        setCurrentPosition(IntakeDeployConstants.kFullStoragePos);
        applySetpoint(EMPTY_STORE);
    }

    @Override
    public void outputTelemetry() {

    }
}
