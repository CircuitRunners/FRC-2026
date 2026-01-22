package frc.robot.auto;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.FieldLayout;
import frc.robot.RobotConstants;
import frc.robot.auto.AutoModeBase;
import frc.robot.subsystems.drive.Drive;
public class AutoSingle extends AutoModeBase{
    public AutoSingle(AutoFactory factory, Drive drive) {
		super(factory, "BranchHAuto");
		AutoTrajectory hToStation = trajectory("hToStation");
	}
}
