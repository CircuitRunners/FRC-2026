package frc.robot.subsystems.vision.objectdetection.objectdetectioncamera;

import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.subsystems.vision.objectdetection.ObjectDetectionConstants;


public class ObjectDetectionCameraIO {
    protected ObjectDetectionCameraIO() {
    }

    protected void updateInputs(ObjectDetectionCameraInputs inputs) {
    }

    public static class ObjectDetectionCameraInputs {
        /**
         * Whether the camera sees at least one object or none of each game piece, by game piece index (type).
         */
        public boolean[] hasObject = new boolean[ObjectDetectionConstants.NUMBER_OF_GAME_PIECE_TYPES];
        /**
         * Stores the Rotation3d of all visible objects.
         * The first index is the game piece ID (type).
         * The second index is the index of the game piece's Rotation3d, with the closest object placed first (index 0).
         */
        public Rotation3d[][] visibleObjectRotations = new Rotation3d[ObjectDetectionConstants.NUMBER_OF_GAME_PIECE_TYPES][0];
        public double latestResultTimestamp = 0;
    }
}