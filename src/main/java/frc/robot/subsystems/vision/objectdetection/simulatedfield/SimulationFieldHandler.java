package frc.robot.subsystems.vision.objectdetection.simulatedfield;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.RobotContainer;


import java.util.ArrayList;
import java.util.List;

/**
 * Handles the simulation of game pieces.
 */
public class SimulationFieldHandler {
    public static Superstructure superstructure;
    public static Drive drive;


    public static void update() {
        SimulatedGamePiece.logAll();
    }

    /**
     * Updates the state of all game pieces.
     */
    

    


    /**
     * Gets the fuel object that is being collected.
     *
     * @param collectionPosition the pose of the collection mechanism
     * @return the fuel object that is being collected
     */
    private static ArrayList<SimulatedGamePiece> getCollectedFuel(Translation3d collectionPosition) {
        final ArrayList<SimulatedGamePiece> collectedFuel = new ArrayList<>();
        for (SimulatedGamePiece gamePiece : SimulatedGamePiece.getUnheldGamePieces())
            if (gamePiece.getDistanceFromPositionMeters(collectionPosition) <= SimulatedGamePieceConstants.INTAKE_TOLERANCE_METERS)
                collectedFuel.add(gamePiece);
        return collectedFuel;
    }

    private static boolean isCollectingFuel() {
        return superstructure.getState().equals(Superstructure.State.INTAKING);
    }

    private static Translation3d getFuelLoaderFieldRelativePose() {
        final Translation3d loaderPose = SimulatedGamePieceConstants.LOADER_CHECK_POSITION;
        return robotRelativeToFieldRelative(loaderPose);
    }







    /**
     * Converts a robot relative pose into a field relative pose.
     *
     * @param robotRelativePose the robot relative pose to convert
     * @return the field relative pose
     */
    private static Translation3d robotRelativeToFieldRelative(Translation3d robotRelativePose) {
        final Pose3d robotPose = new Pose3d(drive.getPose());
        return robotPose.plus(new Transform3d(robotRelativePose, new Rotation3d())).getTranslation();
    }
}