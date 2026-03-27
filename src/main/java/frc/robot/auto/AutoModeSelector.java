package frc.robot.auto;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.auto.autos.LeftDoubleNeutralSilly;
import frc.robot.auto.autos.RightDoubleNeutralSilly;
import frc.robot.auto.autos.centerPreload.CenterPreload;
// import frc.robot.auto.autos.centerPreload.CenterPreload;
import frc.robot.auto.autos.doubleSwipe.LeftDoubleNeutral;
import frc.robot.auto.autos.doubleSwipe.RightDoubleNeutral;
import frc.robot.auto.autos.singleSwipe.LeftNeutralClimb;
import frc.robot.auto.autos.singleSwipe.RightNeutralClimb;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;
public class AutoModeSelector {
	private AutoChooser mAutoChooser = new AutoChooser();

	public static SendableChooser<Boolean> useObjectDetections = new SendableChooser<>();
	

	public AutoModeSelector(Drive drive, Superstructure superstructure, AutoFactory factory) {
		mAutoChooser.addRoutine("Left Neutral Cycle", () -> new LeftNeutralClimb(drive, superstructure, factory).getRoutine());
		mAutoChooser.addRoutine("Right Neutral Cycle", () -> new RightNeutralClimb(drive, superstructure, factory).getRoutine());
		mAutoChooser.addRoutine("Center Preload", () -> new CenterPreload(drive, superstructure, factory).getRoutine());
		// mAutoChooser.addRoutine("Center Preload + Right Climb", () -> new CenterPreload(drive, superstructure, factory).getRoutine());
		mAutoChooser.addRoutine("Left Double Neutral", () -> new LeftDoubleNeutral(drive, superstructure, factory).getRoutine());
		mAutoChooser.addRoutine("Right Double Neutral", () -> new RightDoubleNeutral(drive, superstructure, factory).getRoutine());
		mAutoChooser.addRoutine("right silly", () -> new RightDoubleNeutralSilly(drive, superstructure, factory).getRoutine());
		mAutoChooser.addRoutine("left silly", () -> new LeftDoubleNeutralSilly(drive, superstructure, factory).getRoutine());


		

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