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

    public static TalonFXConfiguration getTlnFXConfig() {
        TalonFXConfiguration nConfig = new TalonFXConfiguration();
        nConfig.Slot0.kP = 1; // PH
        nConfig.Slot0.kD = 1; // PH
        nConfig.Slot0.kS = 1; // PH
        nConfig.Slot0.kG = 1; // PH

        nConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        nConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        nConfig.MotionMagic.MotionMagicCruiseVelocity = 5; // PH
        nConfig.MotionMagic.MotionMagicAcceleration = 7; // PH

        nConfig.Voltage.PeakForwardVoltage = 6; // PH
        nConfig.Voltage.PeakReverseVoltage = -6; // PH

        nConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        nConfig.CurrentLimits.SupplyCurrentLimit = 40; // PH

        nConfig.Feedback.SensorToMechanismRatio = kGearing;

        nConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        nConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        nConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kFullStoragePos.in(Rotations);

        nConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        nConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kDeployPos.in(Rotations);

        return nConfig;

    }

    public static MotorIOTalonFXConfig getIOConfig() {
        MotorIOTalonFXConfig nConfig = new MotorIOTalonFXConfig();
        nConfig.mainConfig = getTlnFXConfig();
        nConfig.mainID = 0; // PH
        nConfig.mainBus = ""; // PH
        nConfig.time = Seconds;
        nConfig.unit = Degrees;
        return nConfig;
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
		ServoHomingConfig nConfig = new ServoHomingConfig();
		nConfig.kHomePosition = kDeployPos;
		nConfig.kHomingTimeout = Seconds.of(0.2); 
		nConfig.kHomingVoltage = Volts.of(-1); // PH
		nConfig.kSetHomedVelocity = DegreesPerSecond.of(1.0); // PH

		return nConfig;
	}
}
