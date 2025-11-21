// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;


public class VisionIOPhotonVision implements VisionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;

  private final Supplier<Pose2d> robotPoseSupplier;
  private final PhotonPoseEstimator poseEstimator;

  public VisionIOPhotonVision(
      String name, Transform3d robotToCamera, Supplier<Pose2d> robotPoseSupplier) {
    camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
    this.robotPoseSupplier = robotPoseSupplier;

    poseEstimator =
        new PhotonPoseEstimator(aprilTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
    poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();

    poseEstimator.setReferencePose(robotPoseSupplier.get());

    Set<Short> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    for (var result : camera.getAllUnreadResults()) {

      if (result.hasTargets()) {
        inputs.latestTargetObservation =
            new TargetObservation(
                Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
      } else {
        inputs.latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
      }

      Optional<EstimatedRobotPose> estimate = poseEstimator.update(result);

      estimate.ifPresent(
          (poseEstimate) -> {

            double totalTagDistance = 0.0;
            for (var target : poseEstimate.targetsUsed) {

              tagIds.add((short) target.fiducialId);
              totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
            }


            poseObservations.add(
                new PoseObservation(
                    poseEstimate.timestampSeconds,
                    poseEstimate.estimatedPose,
                    poseEstimate.targetsUsed.get(0).poseAmbiguity,
                    poseEstimate.targetsUsed.size(),
                    totalTagDistance / poseEstimate.targetsUsed.size(),
                    PoseObservationType.PHOTONVISION));
          });
    }

    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }
}