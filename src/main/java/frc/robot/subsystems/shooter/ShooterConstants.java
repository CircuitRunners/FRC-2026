package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig;
import frc.lib.io.MotorIOTalonFXSim;
import frc.lib.sim.RollerSim;
import frc.lib.sim.RollerSim.RollerSimConstants;
import frc.lib.util.TunableNumber;
import frc.robot.Ports;
import frc.robot.Robot;

public class ShooterConstants {
    public static final double kGearing = 1.0 / 1.0;
    public static Transform2d robotToShooter = new Transform2d(Units.Inches.of(0), Units.Inches.of(6.881), Rotation2d.kZero);

    public static final AngularVelocity kMinVelocity = Units.RotationsPerSecond.of(2000);

    public static final AngularVelocity kEpsilonThreshold = Units.RotationsPerSecond.of(100);

    public static TalonFXConfiguration getFXConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot1.kS = new TunableNumber("Shooter kS", 0.0, true).get(); // default
        config.Slot1.kA = new TunableNumber("Shooter kA", 0.0, true).get(); // default
        config.Slot1.kP = new TunableNumber("Shooter kP", 0.0, true).get(); // default
        config.Slot1.kI = new TunableNumber("Shooter kI", 0.0, true).get(); // default
        config.Slot1.kD = new TunableNumber("Shooter kD", 0.0, true).get(); // default

        config.CurrentLimits.StatorCurrentLimitEnable = Robot.isReal();
        config.CurrentLimits.StatorCurrentLimit = 120.0; // default

		config.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
		config.CurrentLimits.SupplyCurrentLimit = 60.0;
		config.CurrentLimits.SupplyCurrentLowerLimit = 60.0;
		config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

        config.TorqueCurrent.PeakForwardTorqueCurrent = 800; // default
        config.TorqueCurrent.PeakReverseTorqueCurrent = 800; // default
        config.TorqueCurrent.TorqueNeutralDeadband = 0; // default

        config.Feedback.SensorToMechanismRatio = kGearing;

		config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

		return config;
    }

    public static MotorIOTalonFXConfig getIOConfig() {
		MotorIOTalonFXConfig config = new MotorIOTalonFXConfig();
		config.mainConfig = getFXConfig();
		config.time = Units.Second;
		config.unit = Units.Rotations;
		config.mainID = Ports.SHOOTER.id;
		config.mainBus = Ports.SHOOTER.bus;
		config.followerConfig = getFXConfig();
		config.followerMotorAlignment = new MotorAlignmentValue[] {MotorAlignmentValue.Opposed};
		config.followerBuses = new CANBus[] {Ports.SHOOTER_FOLLOWER.bus};
		return config;
	}

    public static MotorIOTalonFX getMotorIO() {
		//if (Robot.isReal()) {
			return new MotorIOTalonFX(getIOConfig());
		//} else {
			//return new MotorIOTalonFXSim(getIOConfig(), new RollerSim(getSimConstants()));
		//}
	}

    public static RollerSimConstants getSimConstants() {
		RollerSimConstants simConstants = new RollerSimConstants();

		simConstants.motor = DCMotor.getKrakenX60Foc(4);
		simConstants.gearing = kGearing;
		simConstants.momentOfInertia = 0.000000001;

		return simConstants;
	}
}