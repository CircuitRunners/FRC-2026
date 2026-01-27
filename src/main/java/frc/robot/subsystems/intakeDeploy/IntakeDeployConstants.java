package frc.robot.subsystems.intakeDeploy;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.lib.bases.ServoMotorSubsystem.ServoHomingConfig;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIOTalonFXSim;
import frc.lib.sim.PivotSim;
import frc.lib.sim.PivotSim.PivotSimConstants;
import frc.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig;
import frc.robot.Ports;
import frc.robot.Robot;

// PH -> PlaceHolder
public class IntakeDeployConstants {
    public static final double kGearing = 40.0;
    
    public static final Angle kDeployPos = Degrees.of(10); // PH
    public static final Angle kFullStoragePos = Degrees.of(10); // PH
    public static final Angle kEmptyStoragePos = Degrees.of(10); // PH
    public static final Angle kHoldPos = Degrees.of(10); // PH
    public static final Angle kOuttakePos = kDeployPos;

    public static final Distance kArmLength = Inches.of(6); // PH

    public static final Angle kEpsilonThreshold = Degrees.of(6); // PH

    public static TalonFXConfiguration getFXConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 1; // PH
        config.Slot0.kD = 1; // PH
        config.Slot0.kS = 1; // PH
        config.Slot0.kG = 1; // PH

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
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kFullStoragePos.in(Rotations);

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kDeployPos.in(Rotations);

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
		simConstants.momentOfInertia = KilogramSquareMeters.of(0.6046665376);
		simConstants.motor = DCMotor.getKrakenX60Foc(1);
		simConstants.mechanismMaxHardStop = kFullStoragePos;
		simConstants.mechanismMinHardStop = kDeployPos;
		simConstants.simGravity = false;
		simConstants.mechanismStartPos = kFullStoragePos;

		return simConstants;
	}

    public static ServoHomingConfig getServoHomingConfig() {
		ServoHomingConfig config = new ServoHomingConfig();
		config.kHomePosition = kDeployPos;
		config.kHomingTimeout = Seconds.of(0.2); 
		config.kHomingVoltage = Volts.of(-1); // PH
		config.kSetHomedVelocity = DegreesPerSecond.of(1.0); // PH

		return config;
	}
}
