package frc.robot.auto;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.auto.autos.LeftNeutralClimb;
import frc.robot.auto.autos.RightNeutralClimb;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;
public class AutoModeSelector {
	private AutoChooser mAutoChooser = new AutoChooser();

	public static SendableChooser<Boolean> useObjectDetections = new SendableChooser<>();
	
	
	public AutoModeSelector(Drive drive, Superstructure superstructure, AutoFactory factory) {
		mAutoChooser.addRoutine("Right Neutral Cycle + Climb", () -> new RightNeutralClimb(drive, superstructure, factory).getRoutine());
		mAutoChooser.addRoutine("Left Neutral Cycle + Climb", () -> new LeftNeutralClimb(drive, superstructure, factory).getRoutine());

		//useObjectDetections.addOption("hell yea!", true);
		useObjectDetections.addOption("no :(", false);
		useObjectDetections.setDefaultOption("hell yea!", true);
		SmartDashboard.putData("object detection", useObjectDetections);
    }



	public Command getSelectedCommand() {
		return mAutoChooser.selectedCommandScheduler();
	}

	public AutoChooser getAutoChooser() {
		return mAutoChooser;
	}

}