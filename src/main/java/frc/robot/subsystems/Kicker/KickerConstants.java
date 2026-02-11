package frc.robot.subsystems.kicker;

import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig;
import frc.lib.io.MotorIOTalonFXSim;
import frc.lib.sim.RollerSim;
import frc.lib.sim.RollerSim.RollerSimConstants;
import frc.robot.Ports;
import frc.robot.Robot;

//PH -> PlaceHolder
public class KickerConstants {
    private static final double kGearing = (6.0 / 9.0);

    public static final Voltage kStartVoltage = Volts.of(3.0); // PH
    public static final Voltage kFeedForwardVoltage = Volts.of(12.0); // PH
    public static final Voltage kHoldVoltage = Volts.of(2.0); // PH
    public static final Voltage kFeedBackwardVoltage = Volts.of(12); // PH

    public static TalonFXConfiguration getFXConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimitEnable = Robot.isReal();
        config.CurrentLimits.StatorCurrentLimit = 120; // PH?

        config.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
		    config.CurrentLimits.SupplyCurrentLimit = 60.0; // PH
		    config.CurrentLimits.SupplyCurrentLowerLimit = 60.0; // PH
		    config.CurrentLimits.SupplyCurrentLowerTime = 0.1; // PH

		    config.Voltage.PeakForwardVoltage = 12.0; // PH
		    config.Voltage.PeakReverseVoltage = -12.0; // PH

		    config.Feedback.SensorToMechanismRatio = kGearing;

		    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

		    return config;
    }

    public static MotorIOTalonFXConfig getIOConfig() {
        MotorIOTalonFXConfig config = new MotorIOTalonFXConfig();
        config.mainConfig = getFXConfig();
        config.time = Minute;
        config.unit = Rotations; 
        config.mainID = Ports.INTAKE_ROLLERS.id; // PH
        config.mainBus = Ports.INTAKE_ROLLERS.bus; // PH
        return config;
    }

    public static MotorIOTalonFX getMotorIO() {
		//if (Robot.isReal()) {
			return new MotorIOTalonFX(getIOConfig());
		// } else {
		// 	return new MotorIOTalonFXSim(getIOConfig(), new RollerSim(getSimConstants()));
		// }
	}

    public static RollerSimConstants getSimConstants() {
		RollerSimConstants simConstants = new RollerSimConstants();

		simConstants.motor = new DCMotor(
				12, 4.05, 275, 1.4, edu.wpi.first.math.util.Units.rotationsPerMinuteToRadiansPerSecond(7530), 1);
		simConstants.gearing = kGearing;
		simConstants.momentOfInertia = 0.000000001;

		return simConstants;
	}
}
