package frc.robot.subsystems.vision.objectdetection;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.objectdetection.simulatedfield.SimulatedGamePieceConstants;

public class ObjectDetectionConstants {
    public static final int NUMBER_OF_GAME_PIECE_TYPES = SimulatedGamePieceConstants.GamePieceType.values().length;
    static final double TRACKED_OBJECT_TOLERANCE_METERS = 0.12;
    public static final double OBJECT_POSE_ESTIMATOR_DELETION_THRESHOLD_SECONDS = 0.5;
    public static final Transform3d cameraTransform =  new Transform3d();
}