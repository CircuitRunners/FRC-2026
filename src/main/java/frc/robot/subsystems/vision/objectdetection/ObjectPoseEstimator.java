
package frc.robot.subsystems.vision.objectdetection;

import java.lang.System.Logger;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.MathHelpers;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.intakeRollers.IntakeRollerConstants;
import frc.robot.subsystems.vision.objectdetection.BestClusterFinder.ClusterResult;
import frc.robot.subsystems.vision.objectdetection.objectdetectioncamera.ObjectDetectionCamera;
import frc.robot.subsystems.vision.objectdetection.objectdetectioncamera.io.SimulationObjectDetectionCameraIO;
import frc.robot.subsystems.vision.objectdetection.simulatedfield.SimulatedGamePiece;
import frc.robot.subsystems.vision.objectdetection.simulatedfield.SimulatedGamePieceConstants;
import frc.robot.subsystems.vision.objectdetection.simulatedfield.SimulatedGamePieceConstants.GamePieceType;

public class ObjectPoseEstimator extends SubsystemBase {
    private final double deletionThresholdSeconds;
    private final SimulatedGamePieceConstants.GamePieceType gamePieceType;
    private final ObjectDetectionCamera camera;
    private final Drive drive;
    public static final Field2d field = new Field2d();

    /**
     * Stores the position of each detected object along with the timestamp of when it was detected.
     */
    private final HashMap<Translation2d, Double> objectPositionsToDetectionTimestamp;

    /**
     * Constructs an ObjectPoseEstimator for estimating the positions of objects detected by camera.
     *
     * @param deletionThresholdSeconds the time in seconds after which an object is considered old and removed
     * @param gamePieceType            the type of game piece to track
     * @param camera                   the camera used for detecting objects
     */
    public ObjectPoseEstimator(Drive drive, double deletionThresholdSeconds,
                               SimulatedGamePieceConstants.GamePieceType gamePieceType,
                               ObjectDetectionCamera camera) {
        this.drive = drive;
        this.deletionThresholdSeconds = deletionThresholdSeconds;
        this.gamePieceType = gamePieceType;
        this.camera = camera;
        this.objectPositionsToDetectionTimestamp = new HashMap<>();
        SimulatedGamePieceConstants.initializeFuel();
        SmartDashboard.putData("ObjectDetectionField", field);
    }

    /**
     * Updates the object positions based on the camera detected objects.
     * Removes objects that have not been detected for a certain time frame, defined in {@link ObjectPoseEstimator#deletionThresholdSeconds}.
     */
    @Override
    public void periodic() {
        updateTrackedObjectsPositions();
        removeOldObjects();
        
        try {
            getOrderedClusters();
            removeIntakedFuel();
        } catch(Exception e) {

        }

        
    }

    /**
     * Gets the position of all known objects on the field.
     *
     * @return a list of Translation2d representing the positions of objects on the field
     */
    public ArrayList<Translation2d> getObjectsOnField() {
        return new ArrayList<>(objectPositionsToDetectionTimestamp.keySet());
    }

    /**
     * Removes the closest object to the robot from the list of objects in the pose estimator.
     */
    public void removeClosestObjectToRobot() {
        final Translation2d closestObject = getClosestObjectToRobot();
        if (closestObject == null)
            return;
        removeObject(closestObject);
    }

    /**
     * Removes the closest object to the intake from the list of objects in the pose estimator.
     *
     * @param intakeTransform the transform of the intake relative to the robot
     */
    public void removeClosestObjectToIntake(Transform2d intakeTransform) {
        final Pose2d robotPose = drive.getPose();
        final Translation2d closestObjectToIntake = getClosestTrackedObjectToPosition(robotPose.transformBy(intakeTransform).getTranslation());
        if (closestObjectToIntake == null)
            return;
        removeObject(closestObjectToIntake);
    }

    /**
     * Removes the closest object to a given pose from the list of objects in the pose estimator.
     *
     * @param fieldRelativePose the pose to which the removed object is closest
     */
    public void removeClosestObjectToPose(Pose2d fieldRelativePose) {
        removeClosestObjectToPosition(fieldRelativePose.getTranslation());
    }

    /**
     * Removes the closest object to a given position from the stored objects in the pose estimator.
     *
     * @param position the position to which the removed object is closest
     */
    public void removeClosestObjectToPosition(Translation2d position) {
        final Translation2d closestObject = getClosestTrackedObjectToPosition(position);
        if (closestObject == null)
            return;
        removeObject(closestObject);
    }

    /**
     * Removes a specific object from the stored objects in the pose estimator.
     * Unlike {@link #removeClosestObjectToPosition} which removes the closest object to a given position.
     *
     * @param objectPosition the position of the object to be removed. Must be the precise position as stored in the pose estimator.
     */
    public void removeObject(Translation2d objectPosition) {
        objectPositionsToDetectionTimestamp.remove(objectPosition);
    }

    /**
     * Returns whether any objects are stored in the pose estimator.
     *
     * @return if there are objects stored in the pose estimator
     */
    public boolean hasObjects() {
        return !objectPositionsToDetectionTimestamp.isEmpty();
    }

    /**
     * Gets the position of the closest object to the robot.
     *
     * @return the closest object to the robot
     */
    public Translation2d getClosestObjectToRobot() {
        return getClosestTrackedObjectToPosition(drive.getPose().getTranslation());

    }

    private void updateTrackedObjectsPositions() {
        final Translation2d[] visibleObjects = camera.getObjectPositionsOnField(gamePieceType);
        final HashMap<Translation2d, Translation2d> trackedObjectsToUpdatedPositions = new HashMap<>();

        for (Translation2d visibleObject : visibleObjects)
            updateObjectPosition(visibleObject, trackedObjectsToUpdatedPositions);

        applyObjectUpdates(trackedObjectsToUpdatedPositions);
    }

    private void applyObjectUpdates(HashMap<Translation2d, Translation2d> currentToNewObjectPositions) {
        final double currentTimestamp = Timer.getTimestamp();

        objectPositionsToDetectionTimestamp.keySet().removeAll(currentToNewObjectPositions.keySet());
        currentToNewObjectPositions.values().forEach(object -> objectPositionsToDetectionTimestamp.put(object, currentTimestamp));
    }

    private void updateObjectPosition(Translation2d objectUpdate, HashMap<Translation2d, Translation2d> trackedObjectsToUpdatedPositions) {
        final Translation2d closestAvailableTrackedObjectToVisibleObject = getClosestAvailableObjectToUpdate(objectUpdate, trackedObjectsToUpdatedPositions);
        if (closestAvailableTrackedObjectToVisibleObject == null) {
            trackedObjectsToUpdatedPositions.put(objectUpdate, objectUpdate);
            return;
        }
        final Translation2d previousUpdate = trackedObjectsToUpdatedPositions.get(closestAvailableTrackedObjectToVisibleObject);
        trackedObjectsToUpdatedPositions.put(closestAvailableTrackedObjectToVisibleObject, objectUpdate);
        if (previousUpdate != null)
            updateObjectPosition(previousUpdate, trackedObjectsToUpdatedPositions);
    }

    private Translation2d getClosestAvailableObjectToUpdate(Translation2d update, HashMap<Translation2d, Translation2d> objectsWithUpdates) {
        final Set<Translation2d> availableObjectsToUpdate = getAvailableObjectsToUpdate(update, objectsWithUpdates);
        if (availableObjectsToUpdate == null || availableObjectsToUpdate.isEmpty())
            return null;
        return getClosestObjectFromSetToPosition(update, availableObjectsToUpdate);
    }

    private Set<Translation2d> getAvailableObjectsToUpdate(Translation2d update, HashMap<Translation2d, Translation2d> objectsWithUpdates) {
        if (objectPositionsToDetectionTimestamp.isEmpty())
            return null;
        final Set<Translation2d> availableObjects = new HashSet<>();
        for (Translation2d currentObject : objectPositionsToDetectionTimestamp.keySet()) {
            final double updateDistanceFromCurrentObject = update.getDistance(currentObject);
            if (updateDistanceFromCurrentObject > ObjectDetectionConstants.TRACKED_OBJECT_TOLERANCE_METERS)
                continue;
            if (!objectsWithUpdates.containsKey(currentObject)) {
                availableObjects.add(currentObject);
                continue;
            }
            if (isNewUpdateCloserThanPreviousUpdate(update, objectsWithUpdates.get(currentObject), currentObject))
                availableObjects.add(currentObject);
        }
        return availableObjects;
    }

    private boolean isNewUpdateCloserThanPreviousUpdate(Translation2d newUpdate, Translation2d previousUpdate, Translation2d object) {
        return newUpdate.getDistance(object) < previousUpdate.getDistance(object);
    }

    private Translation2d getClosestTrackedObjectToPosition(Translation2d position) {
        return getClosestObjectFromSetToPosition(position, objectPositionsToDetectionTimestamp.keySet());
    }

    private Translation2d getClosestObjectFromSetToPosition(Translation2d position, Set<Translation2d> objects) {
        if (objects.isEmpty())
            return null;
        Translation2d closestObjectTranslation = null;
        double closestObjectDistance = Double.MAX_VALUE;

        for (Translation2d object : objects) {
            final double currentObjectDistance = position.getDistance(object);
            if (currentObjectDistance < closestObjectDistance) {
                closestObjectDistance = currentObjectDistance;
                closestObjectTranslation = object;
            }
        }
        return closestObjectTranslation;
    }

    private void removeOldObjects() {
        objectPositionsToDetectionTimestamp.entrySet().removeIf(entry -> hasObjectExpired(entry.getValue()));
    }

    private boolean hasObjectExpired(double timestamp) {
        return Timer.getTimestamp() - timestamp > deletionThresholdSeconds;
    }

    public List<Translation2d> getOrderedClusters() {
        ArrayList<ClusterResult> w = new ArrayList<>();
        List<Translation2d> l = new ArrayList<Translation2d>();
        List<Pose2d> d = new ArrayList<>();

        
        for (SimulatedGamePiece s : SimulatedGamePiece.getSimulatedGamePieces()) {
            l.add(new Translation2d(s.getPosition().getX(), s.getPosition().getY()));
            d.add(MathHelpers.pose2dFromTranslation(new Translation2d(s.getPosition().getX(), s.getPosition().getY())));
        }
        field.getObject("Fuel").setPoses(d);
        double radius = ObjectDetectionConstants.maxClusterRadius;
        boolean cont = true;
        
        while (cont == true) {
            BestClusterFinder.ClusterResult result =
                    BestClusterFinder.findBestCluster(l, radius);

            for (Translation2d p : result.points) {
                for (int j = 0; j < l.size(); j++) {
                    if (l.get(j).getDistance(p) < 0.005) {
                        l.remove(l.get(j));
                    }
                }
            }
            
            if (result.points.size() < 3) cont = false;
            else w.add(result);
        }
        ArrayList<Pose2d> results = new ArrayList<>();
        for (ClusterResult c : w) {
            results.add(MathHelpers.pose2dFromTranslation(c.center));
        }

        // field.getObject("Centroid").setPoses(results);
        field.setRobotPose(drive.getPose());

        List<Pose2d> ordered = new ArrayList<>();
        List<Pose2d> remaining = new ArrayList<>(results);


        Pose2d currentPose = drive.getPose();


        while (!remaining.isEmpty()) {

            Pose2d closest = remaining.get(0);
            double closestDistance =
                    currentPose.getTranslation()
                            .getDistance(closest.getTranslation());

            for (Pose2d pose : remaining) {

                double distance =
                        currentPose.getTranslation()
                                .getDistance(pose.getTranslation());

                if (distance < closestDistance) {
                    closestDistance = distance;
                    closest = pose;
                }
            }

            ordered.add(closest);
            currentPose = closest;
            remaining.remove(closest);
        }

        boolean improved = true;

        while (improved) {
            improved = false;

            for (int i = 0; i < ordered.size() - 2; i++) {
                for (int j = i + 2; j < ordered.size() - 1; j++) {

                    Pose2d A = ordered.get(i);
                    Pose2d B = ordered.get(i + 1);
                    Pose2d C = ordered.get(j);
                    Pose2d D = ordered.get(j + 1);

                    double currentDist =
                            A.getTranslation().getDistance(B.getTranslation())
                        + C.getTranslation().getDistance(D.getTranslation());

                    double swappedDist =
                            A.getTranslation().getDistance(C.getTranslation())
                        + B.getTranslation().getDistance(D.getTranslation());

                    if (swappedDist < currentDist) {
                        Collections.reverse(ordered.subList(i + 1, j + 1));
                        improved = true;
                    }
                }
            }
        }


        List<Translation2d> interiorWaypoints = new ArrayList<>();

        for (int i = 0; i < ordered.size() - 1; i++) {
            interiorWaypoints.add(ordered.get(i).getTranslation());
        }

        if (ordered.isEmpty()) {
            return new ArrayList<>();
        }
        Pose2d endPose = ordered.get(ordered.size() - 1);

        interiorWaypoints.add(endPose.getTranslation());

        return interiorWaypoints;
    }



    // public void removeIntakedFuel() {
    //     Rectangle2d intake = new Rectangle2d(
    //         (new Translation2d(0.6408928, IntakeRollerConstants.intakeWidth / 2))
    //             .plus(drive.getPose().getTranslation()),
    //         (new Translation2d(0.6408928 - 0.2524, -IntakeRollerConstants.intakeWidth / 2))
    //             .plus(drive.getPose().getTranslation())
    //     );

    //     Iterator<SimulatedGamePiece> iterator =
    //         SimulatedGamePiece.getSimulatedGamePieces().iterator();

    //     while (iterator.hasNext()) {
    //         SimulatedGamePiece s = iterator.next();

    //         Translation2d piecePos =
    //             new Translation2d(s.getPosition().getX(),
    //                             s.getPosition().getY());

    //         if (intake.contains(piecePos)) {
    //             iterator.remove();
    //         }
    //     }
    // }

    private Pose2d lastPose = null;

    public void removeIntakedFuel() {

        Pose2d currentPose = drive.getPose();

        if (lastPose == null) {
            lastPose = currentPose;
            return;
        }

        Iterator<SimulatedGamePiece> iterator =
            SimulatedGamePiece.getSimulatedGamePieces().iterator();

        while (iterator.hasNext()) {
            SimulatedGamePiece s = iterator.next();

            Translation2d pieceField =
                new Translation2d(
                    s.getPosition().getX(),
                    s.getPosition().getY());

            // Check multiple interpolation steps between lastPose and currentPose
            int steps = 5;  // increase if still unreliable

            boolean remove = false;

            for (int i = 0; i <= steps; i++) {

                double t = (double) i / steps;

                Pose2d interpPose = lastPose.interpolate(currentPose, t);

                // Convert piece into ROBOT frame at this interpolated pose
                Translation2d pieceRobot =
                    pieceField.minus(interpPose.getTranslation())
                            .rotateBy(interpPose.getRotation().unaryMinus());

                double x = pieceRobot.getX();
                double y = pieceRobot.getY();

                boolean inside =
                    x <= 0.6408928 &&
                    x >= (0.6408928 - 0.2524) &&
                    Math.abs(y) <= IntakeRollerConstants.intakeWidth / 2;

                if (inside) {
                    remove = true;
                    break;
                }
            }

            if (remove) {
                iterator.remove();
            }
        }

        lastPose = currentPose;
    }
}