package frc.robot.subsystems.intakeRollers;

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
import frc.robot.Robot;

//PH -> PlaceHolder
public class IntakeRollerConstants {
    private static final double kGearing = (6 / 9);

    public static final Voltage kStartVoltage = Volts.of(3); // PH
    public static final Voltage kIntakeVoltage = Volts.of(2); // PH
    public static final Voltage kPeliVoltage = Volts.of(2); // PH
    public static final Voltage kOuttakeVoltage = Volts.of(12); // PH

    public static TalonFXConfiguration getTlnFXConfig() {
        TalonFXConfiguration nConfig = new TalonFXConfiguration();

        nConfig.CurrentLimits.StatorCurrentLimitEnable = Robot.isReal();
        nConfig.CurrentLimits.StatorCurrentLimit = 120; // PH?

        nConfig.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
		nConfig.CurrentLimits.SupplyCurrentLimit = 60.0; // PH
		nConfig.CurrentLimits.SupplyCurrentLowerLimit = 60.0; // PH
		nConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1; // PH

		nConfig.Voltage.PeakForwardVoltage = 12.0; // PH
		nConfig.Voltage.PeakReverseVoltage = -12.0; // PH

		nConfig.Feedback.SensorToMechanismRatio = kGearing;

		nConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

		return nConfig;
    }

    public static MotorIOTalonFXConfig getIOConfig() {
        MotorIOTalonFXConfig nConfig = new MotorIOTalonFXConfig();
        nConfig.mainConfig = getTlnFXConfig();
        nConfig.time = Minute;
        nConfig.unit = Rotations;
        nConfig.mainID = 0; // PH
        nConfig.mainBus = ""; // PH
        return nConfig;
    }

    public static MotorIOTalonFX getMotorIO() {
		if (Robot.isReal()) {
			return new MotorIOTalonFX(getIOConfig());
		} else {
			return new MotorIOTalonFXSim(getIOConfig(), new RollerSim(getSimConstants()));
		}
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
