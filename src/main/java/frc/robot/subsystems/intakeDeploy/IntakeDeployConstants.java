package frc.robot.subsystems.intakeDeploy;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.lib.bases.ServoMotorSubsystem.ServoHomingConfig;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig;
import frc.lib.io.MotorIOTalonFXSim;
import frc.lib.sim.PivotSim;
import frc.lib.sim.PivotSim.PivotSimConstants;
import frc.robot.Ports;
import frc.robot.Robot;
import frc.robot.RobotConstants;

// PH -> PlaceHolder
public class IntakeDeployConstants {
    public static final double kGearing = 40.0;

	public static final Angle kDeployPosition = Units.Degrees.of(3.0);
	public static final Angle kStowClearPosition = Units.Degrees.of(55.0);
	public static final Angle kFullStowPosition = Units.Degrees.of(83.0);
	public static final Angle kIndexerHold = Units.Degrees.of(10.0);

	public static final Angle kExhaustPosition = kDeployPosition;
	public static final Distance kArmLength = Units.Inches.of(14.0);

	public static final Angle kEpsilonThreshold = Units.Degrees.of(6.0);

    public static TalonFXConfiguration getFXConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 180.0; // PH
        config.Slot0.kD = 0.0; // PH
        config.Slot0.kS = 0.0; // PH
        config.Slot0.kG = 0.0; // PH

        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        config.MotionMagic.MotionMagicCruiseVelocity = 5; // PH
        config.MotionMagic.MotionMagicAcceleration = 7; // PH

        config.Voltage.PeakForwardVoltage = 6; // PH
        config.Voltage.PeakReverseVoltage = -6; // PH

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40; // PH

        config.Feedback.SensorToMechanismRatio = kGearing;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kFullStowPosition.in(Units.Rotations);

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kDeployPosition.in(Units.Rotations);

        return config;

    }

    public static MotorIOTalonFXConfig getIOConfig() {
        MotorIOTalonFXConfig config = new MotorIOTalonFXConfig();
        config.mainConfig = getFXConfig();
        config.mainID = Ports.INTAKE_DEPLOY.id; // PH
        config.mainBus = Ports.INTAKE_DEPLOY.bus; // PH
        config.time = Seconds;
        config.unit = Degrees;
        return config;
    }

    public static MotorIOTalonFX getMotorIO() {
        if (Robot.isReal()) {
            return new MotorIOTalonFX(getIOConfig());
        } else {
            return new MotorIOTalonFXSim(getIOConfig(), new PivotSim(getSimConstants()));
        }
    }

    public static PivotSimConstants getSimConstants() {
		PivotSimConstants simConstants = new PivotSimConstants();
		simConstants.gearing = kGearing;
		simConstants.armLength = kArmLength;
		simConstants.momentOfInertia = Units.KilogramSquareMeters.of(0.6046665376);
		simConstants.motor = DCMotor.getKrakenX60Foc(1);
		simConstants.mechanismMaxHardStop = kFullStowPosition;
		simConstants.mechanismMinHardStop = kDeployPosition;
		simConstants.simGravity = false;
		simConstants.mechanismStartPos = kFullStowPosition;

		return simConstants;
	}

    public static ServoHomingConfig getServoHomingConfig() {
		ServoHomingConfig config = new ServoHomingConfig();
		config.kHomePosition = kDeployPosition;
		config.kHomingTimeout = Seconds.of(0.2); 
		config.kHomingVoltage = Units.Volts.of(-1); // PH
		config.kSetHomedVelocity = Units.DegreesPerSecond.of(1.0); // PH

		return config;
	}
}
