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

public class LeftDepotClimb extends AutoModeBase{
    public LeftDepotClimb(Drive drive, Superstructure superstructure, AutoFactory autoFactory){
        super(drive,superstructure,autoFactory,"Left Depot + Climb");

        AutoTrajectory leftTrenchDepot = trajectory("leftTrenchToDepot");
        AutoTrajectory leftDepotClimb = trajectory("leftDepotToClimb");

        Pose2d startPose = FieldLayout.handleAllianceFlip(new Pose2d(4.64, 7.44, Rotation2d.kZero), RobotConstants.isRedAlliance);

        prepRoutine(
            AutoHelpers.resetPoseIfWithoutEstimate(startPose, drive),
            leftTrenchDepot.cmd(),
            new PIDToPoseCommand(drive, superstructure, FieldLayout.handleAllianceFlip(new Pose2d(7.6, 5, new Rotation2d(-1)), RobotConstants.isRedAlliance), Units.Inches.of(12.0), Units.Degrees.of(10.0)),
			new PIDToPoseCommand(drive, superstructure, leftTrenchDepot.getInitialPose().get(), Units.Inches.of(36.0), Units.Degrees.of(10.0))

        );
    }
}