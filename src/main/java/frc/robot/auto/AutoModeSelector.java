package frc.robot.auto;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;
public class AutoModeSelector {
	private AutoChooser mAutoChooser = new AutoChooser();

	public AutoModeSelector(RobotContainer r, Drive drive, Superstructure superstructure, AutoFactory factory) {
		mAutoChooser.addRoutine("Right Neutral Cycle + Climb", () -> new RightNeutralCycle_Climb(r, drive, superstructure, factory).getRoutine());
		mAutoChooser.addRoutine("Left Neutral Cycle + Climb", () -> new LeftNeutralClimb(drive, superstructure, factory).getRoutine());
    }

	public Command getSelectedCommand() {
		return mAutoChooser.selectedCommandScheduler();
	}

	public AutoChooser getAutoChooser() {
		return mAutoChooser;
	}
}