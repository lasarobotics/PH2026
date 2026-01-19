package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

public class VisionSubsystem extends SubsystemBase {
  public static record BoundingBox(double widthPixels, double heightPixels) {}

  public static record TargetObservation(
      PhotonTrackedTarget target,
      BoundingBox boundingBox,
      double distanceMeters,
      Translation2d robotTranslation) {}

  private final PhotonCamera m_camera;
  private PhotonPipelineResult m_lastResult = new PhotonPipelineResult();
  private Optional<TargetObservation> m_bestObservation = Optional.empty();
  private boolean m_objectDetectionActive = false;

  public VisionSubsystem() {
    this(Constants.Vision.CAMERA_NAME);
  }

  public VisionSubsystem(String cameraName) {
    m_camera = new PhotonCamera(cameraName);
  }

  @Override
  public void periodic() {
    var unreadResults = m_camera.getAllUnreadResults();
    if (unreadResults.isEmpty()) {
      return;
    }

    m_lastResult = unreadResults.get(unreadResults.size() - 1);
    processResult(m_lastResult);
  }

  private void processResult(PhotonPipelineResult result) {
    m_objectDetectionActive =
        m_camera.getPipelineIndex() == Constants.Vision.OBJECT_DETECTION_PIPELINE_INDEX;
    if (!m_objectDetectionActive || !result.hasTargets()) {
      m_bestObservation = Optional.empty();
      return;
    }

    List<PhotonTrackedTarget> targets = result.getTargets();
    int limit = Constants.Vision.MAX_TARGETS_TO_PROCESS;
    List<TargetObservation> observations = new ArrayList<>(Math.min(limit, targets.size()));
    int processed = 0;

    for (PhotonTrackedTarget target : targets) {
      if (processed >= limit) {
        break;
      }

      if (!isObjectDetectionTarget(target)) {
        continue;
      }

      BoundingBox box = computeBoundingBox(target);
      double distance = estimateDistance(target, box);
      Translation2d robotTranslation = toRobotTranslation(target, distance);

      observations.add(new TargetObservation(target, box, distance, robotTranslation));
      processed++;
    }

    observations.sort(Comparator.comparingDouble(TargetObservation::distanceMeters));
    m_bestObservation = observations.isEmpty() ? Optional.empty() : Optional.of(observations.get(0));

    m_bestObservation.ifPresent(
        obs -> {
          Logger.recordOutput(getName() + "/TargetDistanceMeters", obs.distanceMeters());
          Logger.recordOutput(getName() + "/TargetYawDegrees", obs.target().getYaw());
          Logger.recordOutput(getName() + "/RobotRelativeTargetX", obs.robotTranslation().getX());
          Logger.recordOutput(getName() + "/RobotRelativeTargetY", obs.robotTranslation().getY());
        });
  }

  private boolean isObjectDetectionTarget(PhotonTrackedTarget target) {
    if (target.getDetectedObjectClassID() < 0) {
      return false;
    }

    return target.getDetectedObjectConfidence() >= Constants.Vision.MIN_CONFIDENCE;
  }

  private BoundingBox computeBoundingBox(PhotonTrackedTarget target) {
    List<TargetCorner> corners = target.getMinAreaRectCorners();
    if (corners == null || corners.isEmpty()) {
      // If corners are missing, fall back to a zero-size box to keep processing predictable.
      return new BoundingBox(0, 0);
    }

    double minX = Double.POSITIVE_INFINITY;
    double minY = Double.POSITIVE_INFINITY;
    double maxX = Double.NEGATIVE_INFINITY;
    double maxY = Double.NEGATIVE_INFINITY;

    for (TargetCorner corner : corners) {
      minX = Math.min(minX, corner.x);
      minY = Math.min(minY, corner.y);
      maxX = Math.max(maxX, corner.x);
      maxY = Math.max(maxY, corner.y);
    }

    return new BoundingBox(maxX - minX, maxY - minY);
  }

  private double estimateDistance(PhotonTrackedTarget target, BoundingBox box) {
    double areaPercent = Math.max(target.getArea(), Constants.Vision.MIN_TARGET_AREA_PERCENT);
    double areaScale = Math.sqrt(Constants.Vision.AREA_PERCENT_AT_ONE_METER / areaPercent);

    double boxArea = box.widthPixels() * box.heightPixels();
    double boxScale =
        boxArea > 0
            ? Math.sqrt(Constants.Vision.BOUNDING_BOX_AREA_AT_ONE_METER / boxArea)
            : areaScale;

    double distance = (areaScale + boxScale) * 0.5;
    return Math.max(distance, Constants.Vision.MIN_DISTANCE_METERS);
  }

  private Translation2d toRobotTranslation(PhotonTrackedTarget target, double distanceMeters) {
    double yawRadians = Math.toRadians(target.getYaw());
    double forward = distanceMeters * Math.cos(yawRadians);
    double left = distanceMeters * Math.sin(yawRadians);
    return new Translation2d(forward, left);
  }

  public boolean hasValidTarget() {
    return m_objectDetectionActive && m_bestObservation.isPresent();
  }

  public double getTargetYaw() {
    return m_bestObservation.map(obs -> obs.target().getYaw()).orElse(0.0);
  }

  public double getTargetDistance() {
    return m_bestObservation.map(TargetObservation::distanceMeters).orElse(Double.POSITIVE_INFINITY);
  }

  public Translation2d getRobotRelativeTranslation() {
    return m_bestObservation.map(TargetObservation::robotTranslation).orElse(new Translation2d());
  }

  public Optional<PhotonTrackedTarget> getRawTarget() {
    return m_bestObservation.map(TargetObservation::target);
  }

  public boolean isObjectDetectionPipelineActive() {
    return m_objectDetectionActive;
  }
}
