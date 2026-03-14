package frc.robot.auto;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.auto.autos.centerPreload.CenterPreloadClimbLeft;
import frc.robot.auto.autos.centerPreload.CenterPreloadClimbRight;
import frc.robot.auto.autos.doubleSwipe.LeftDoubleNeutral;
import frc.robot.auto.autos.doubleSwipe.RightDoubleNeutral;
import frc.robot.auto.singleSwipe.LeftNeutralClimb;
import frc.robot.auto.singleSwipe.RightNeutralClimb;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;
public class AutoModeSelector {
	private AutoChooser mAutoChooser = new AutoChooser();

	public static SendableChooser<Boolean> useObjectDetections = new SendableChooser<>();
	
	
	public AutoModeSelector(Drive drive, Superstructure superstructure, AutoFactory factory) {
		mAutoChooser.addRoutine("Left Neutral Cycle", () -> new LeftNeutralClimb(drive, superstructure, factory).getRoutine());
		mAutoChooser.addRoutine("Right Neutral Cycle", () -> new RightNeutralClimb(drive, superstructure, factory).getRoutine());
		mAutoChooser.addRoutine("Center Preload + Left Climb", () -> new CenterPreloadClimbLeft(drive, superstructure, factory).getRoutine());
		mAutoChooser.addRoutine("Center Preload + Right Climb", () -> new CenterPreloadClimbRight(drive, superstructure, factory).getRoutine());
		mAutoChooser.addRoutine("Left Double Neutral", () -> new LeftDoubleNeutral(drive, superstructure, factory).getRoutine());
		mAutoChooser.addRoutine("Right Double Neutral", () -> new RightDoubleNeutral(drive, superstructure, factory).getRoutine());

		

		//
		useObjectDetections.setDefaultOption("no :(", false);
		useObjectDetections.addOption("hell yea!", true);
		SmartDashboard.putData("object detection", useObjectDetections);
    }



	public Command getSelectedCommand() {
		return mAutoChooser.selectedCommandScheduler();
	}

	public AutoChooser getAutoChooser() {
		return mAutoChooser;
	}

}