package frc.robot.subsystems.superstructure;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.CANrangeConfiguration;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.io.BeamBreakIO;
import frc.lib.io.BeamBreakIOCANRange;
import frc.lib.io.BeamBreakIOSim;

import frc.robot.Ports;
import frc.robot.Robot;
import frc.robot.controlboard.ControlBoardConstants;

public class SuperstructureConstants {
    public static Distance getAutoAlignScoringDistanceEpsilon() {
		return Units.Inches.of(0);
	}

	public static Angle getAutoAlignScoringAngleEpsilon() {
		return Units.Degrees.of(0);
	}

	public static Time getAutoAlignScoringDelay() {
		return Units.Seconds.of(0);
	}
}
