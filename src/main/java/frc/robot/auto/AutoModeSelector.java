package frc.robot.auto;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;
public class AutoModeSelector {
	private AutoChooser mAutoChooser = new AutoChooser();

	public AutoModeSelector(Drive drive, Superstructure superstructure, AutoFactory factory) {
		mAutoChooser.addRoutine("[CENTER] Net GH IJ KL", () -> new NetGHIJKL(drive, superstructure, factory).getRoutine());
    }

	public Command getSelectedCommand() {
		return mAutoChooser.selectedCommandScheduler();
	}

	public AutoChooser getAutoChooser() {
		return mAutoChooser;
	}
}