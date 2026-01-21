package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.ArrayList;
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
  private Optional<TargetObservation> m_currentTarget = Optional.empty();
  private int m_framesSinceSeen = 0;
  private static final int MAX_MISSED_FRAMES = 3;

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
      if (!m_objectDetectionActive) {
        m_currentTarget = Optional.empty();
      }
      m_bestObservation = Optional.empty();
      m_framesSinceSeen++;
      if (m_framesSinceSeen > MAX_MISSED_FRAMES) {
        m_currentTarget = Optional.empty();
      }
      return;
    }

    // Targets present → reset missed-frame counter
    m_framesSinceSeen = 0;

    List<PhotonTrackedTarget> targets = result.getTargets();
    int limit = Constants.Vision.MAX_TARGETS_TO_PROCESS;
    List<TargetObservation> observations = new ArrayList<>(Math.min(limit, targets.size()));
    int processed = 0;

    for (int i = 0; i < targets.size() && processed < limit; i++) {
      PhotonTrackedTarget target = targets.get(i);
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

    m_bestObservation = selectBestTarget(observations);
    updateCurrentTarget(observations);

    m_currentTarget.ifPresent(
        obs -> {
          Logger.recordOutput(getName() + "/TargetDistanceMeters", obs.distanceMeters());
          Logger.recordOutput(getName() + "/TargetYawDegrees", obs.target().getYaw());
          Logger.recordOutput(getName() + "/RobotRelativeTargetX", obs.robotTranslation().getX());
          Logger.recordOutput(getName() + "/RobotRelativeTargetY", obs.robotTranslation().getY());
        });
    Logger.recordOutput(getName() + "/HasTarget", hasValidTarget());
    Logger.recordOutput(getName() + "/FramesSinceSeen", m_framesSinceSeen);
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
      // If corners are missing it shouldfall back to a zero-size box to keep processing predictable.
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
    return m_objectDetectionActive && m_currentTarget.isPresent();
  }

  public double getTargetYaw() {
    return m_currentTarget.map(obs -> obs.target().getYaw()).orElse(0.0);
  }

  public double getTargetDistance() {
    return m_currentTarget
        .map(TargetObservation::distanceMeters)
        .orElse(Double.POSITIVE_INFINITY);
  }

  public Translation2d getRobotRelativeTranslation() {
    return m_currentTarget
        .map(TargetObservation::robotTranslation)
        .orElse(new Translation2d());
  }

  public Optional<PhotonTrackedTarget> getRawTarget() {
    return m_currentTarget.map(TargetObservation::target);
  }

  public boolean isObjectDetectionPipelineActive() {
    return m_objectDetectionActive;
  }

  public boolean hasLockedTarget() {
    return m_currentTarget.isPresent();
  }

  public void clearTargetLock() {
    m_currentTarget = Optional.empty();
    m_framesSinceSeen = 0;
  }

  private Optional<TargetObservation> selectBestTarget(List<TargetObservation> observations) {
    if (observations.isEmpty()) {
      return Optional.empty();
    }

    double[] neighborBonuses = new double[observations.size()];
    for (int i = 0; i < observations.size(); i++) {
      Translation2d position = observations.get(i).robotTranslation();
      for (int j = 0; j < observations.size(); j++) {
        if (i == j) continue;
        double neighborDistance = position.getDistance(observations.get(j).robotTranslation());
        if (neighborDistance < 1.0) {
          neighborBonuses[i] += 0.05;
        }
      }
    }

    for (int i = 0; i < neighborBonuses.length; i++) {
      neighborBonuses[i] = Math.min(neighborBonuses[i], 0.3);
    }

    TargetObservation bestObservation = observations.get(0);
    double bestScore = computeBallWeightScore(observations.get(0), neighborBonuses[0]);

    for (int i = 1; i < observations.size(); i++) {
      double score = computeBallWeightScore(observations.get(i), neighborBonuses[i]);
      if (score > bestScore) {
        bestScore = score;
        bestObservation = observations.get(i);
      }
    }

    return Optional.of(bestObservation);
  }

  private double computeBallWeightScore(TargetObservation observation, double neighborBonus) {
    double distanceToBall = observation.distanceMeters();
    double baseScore = 1.0 / Math.max(distanceToBall, Constants.Vision.MIN_DISTANCE_METERS);
    return baseScore + neighborBonus;
  }

  private void updateCurrentTarget(List<TargetObservation> observations) {
    if (m_bestObservation.isEmpty()) {
      m_framesSinceSeen++;
      if (m_framesSinceSeen > MAX_MISSED_FRAMES) {
        m_currentTarget = Optional.empty();
      }
      return;
    }

    TargetObservation best = m_bestObservation.get();
    if (m_currentTarget.isPresent()) {
      Translation2d currentPos = m_currentTarget.get().robotTranslation();
      double distanceBetween = currentPos.getDistance(best.robotTranslation());
      double yawDiff =
          Math.abs(m_currentTarget.get().target().getYaw() - best.target().getYaw());
      double distanceDiff = Math.abs(m_currentTarget.get().distanceMeters() - best.distanceMeters());
      if (yawDiff <= 7.0 && distanceDiff <= 0.4) {
        m_framesSinceSeen = 0;
        return;
      }
      if (scoreBetter(best, m_currentTarget.get())) {
        m_currentTarget = Optional.of(best);
        m_framesSinceSeen = 0;
        return;
      }
      m_framesSinceSeen++;
      if (m_framesSinceSeen > MAX_MISSED_FRAMES) {
        m_currentTarget = Optional.of(best);
        m_framesSinceSeen = 0;
      }
      return;
    }

    m_currentTarget = Optional.of(best);
    m_framesSinceSeen = 0;
  }

  private boolean scoreBetter(TargetObservation candidate, TargetObservation current) {
    double candidateScore = computeBallWeightScore(candidate, 0);
    double currentScore = computeBallWeightScore(current, 0) + 0.2; // bias to keep lock
    return candidateScore > currentScore;
  }
}
