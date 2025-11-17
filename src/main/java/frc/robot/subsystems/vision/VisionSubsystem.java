package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.drive.Drive;

public class VisionSubsystem {
    private VisionIO leftCam;
    private VisionIO rightCam;
    private Vision vision;

    public VisionSubsystem(Drive drive) {
        if (RobotBase.isSimulation()) {
            leftCam = new VisionIOPhotonVisionSim(VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose);
            rightCam = new VisionIOPhotonVisionSim(VisionConstants.camera1Name, VisionConstants.robotToCamera1, drive::getPose);
        } else {

            leftCam = new VisionIOPhotonVision(VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose);
            rightCam = new VisionIOPhotonVision(VisionConstants.camera1Name, VisionConstants.robotToCamera1, drive::getPose);
        }
        vision = new Vision(
            drive.getDrivetrain().getVisionConsumer(),
            leftCam,
            rightCam);
    }
}
