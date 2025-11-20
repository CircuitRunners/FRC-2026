package frc.lib.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.FieldLayout;
import frc.lib.util.FieldLayout.Branch;
import frc.robot.RobotConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;

public class AutoAlignToReefCommand extends Command {
    private final Drive drive;
    private final Superstructure superstructure;
    private final boolean isLeft;

    private Command pidCommand;

    public AutoAlignToReefCommand(Drive drive, Superstructure superstructure, boolean isLeft) {
        this.drive = drive;
        this.superstructure = superstructure;
        this.isLeft = isLeft;
        addRequirements(drive);
        SmartDashboard.putNumber("Autoalign reef offset", 0.5);
    }

    @Override
    public void initialize() {
        Pose2d target = computeTargetPose(isLeft);
        pidCommand = new PIDToPoseCommand(drive, superstructure, target);

        pidCommand.initialize();
    }

    @Override
    public void execute() {
        if (pidCommand != null) {
            pidCommand.execute();
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (pidCommand != null) {
            pidCommand.end(interrupted);
        }
    }

    @Override
    public boolean isFinished() {
        return pidCommand == null || pidCommand.isFinished();
    }

    private Pose2d computeTargetPose(boolean left) {
        Pose2d robotPose = drive.getPose();
        Branch branch;
        if (left) {
            branch = Branch.getClosestLeftRightBranch(true, robotPose, RobotConstants.isRedAlliance);
        } else {
            branch = Branch.getClosestLeftRightBranch(false, robotPose, RobotConstants.isRedAlliance);
        }

        Pose2d branchPose = Branch.handleAllianceFlip(Branch.getCoralScoringPose(branch), RobotConstants.isRedAlliance);

        if (RobotConstants.isRedAlliance) {
            branchPose = Branch.handleAllianceFlip(Branch.getCoralScoringPose(branch), RobotConstants.isRedAlliance);
        }

        double offset = SmartDashboard.getNumber("Autoalign reef offset", 0);
        if (!RobotConstants.isRedAlliance) offset*=-1;
        Rotation2d rot = Branch.getBranchFaceRotation(branch);

        double dx = rot.getCos() * offset;
        double dy = rot.getSin() * offset;
        
        //branchPose = new Pose2d(branchPose.getTranslation(), new Rotation2d(branchPose.getRotation().getDegrees()+ 180));
        return new Pose2d(branchPose.getX() + dx, branchPose.getY() + dy, branchPose.getRotation());
    }
}
