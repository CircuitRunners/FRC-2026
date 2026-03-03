package frc.robot.auto.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.drive.PIDToPoseCommand;
import frc.lib.util.FieldLayout;
import frc.robot.RobotConstants;
import frc.robot.auto.AutoConstants;
import frc.robot.auto.AutoHelpers;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.auto.AutoModeBase;

public class RightPreloadClimb extends AutoModeBase{
    public RightPreloadClimb(Drive drive, Superstructure superstructure, AutoFactory autoFactory){
        super(drive, superstructure, autoFactory, "Right Preload + Climb");
        AutoTrajectory RightPreloadToClimb = trajectory("RightPreloadClimb");
        Pose2d startPose = FieldLayout.handleAllianceFlip(new Pose2d(4.64, 7.44, Rotation2d.kZero), RobotConstants.isRedAlliance);


        prepRoutine(
            AutoHelpers.resetPoseIfWithoutEstimate(startPose, drive),
            RightPreloadToClimb.cmd(),
            superstructure.shootWhenReady(),
            superstructure.climb()
        );
    }
}
