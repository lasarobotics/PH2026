package frc.robot;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.drive.DriveSubsystem;

public class VisionUtil {
  private static final String CAMERA_NAME = "418";
  private static final String LOG_PATH = "VisionUtil";
  private static final int INVALID_OBJECT_ID = -1;
  private static final double INVALID_CONFIDENCE = -1.0;
  private static final double SAME_OBJECT_DISTANCE_METERS = 0.75; // if a pose of a new object with same ID is within this distance just keep it as the same object.
  private static final Translation3d CLEARED_TRANSLATION = new Translation3d();
  private static final Pose3d CLEARED_POSE = new Pose3d();
  private static final Transform3d ROBOT_TO_CAMERA = new Transform3d();
  private static final double IntakeWidth = 24.5; // INCHES
  private static final double FuelDiamter = 5.91; // INCHES

  private static final PhotonCamera s_camera = new PhotonCamera(CAMERA_NAME);
  private static PhotonPipelineResult s_latestResult = new PhotonPipelineResult();
  private static final List<TrackedObjectSlot> s_objectSlots = new ArrayList<>();

  private VisionUtil() {}

  private static class VisibleObject {
    private final PhotonTrackedTarget target;
    private final Transform3d cameraToObject;
    private final Pose3d fieldPose;

    private VisibleObject(
      PhotonTrackedTarget target,
      Transform3d cameraToObject,
      Pose3d fieldPose
    ) {
      this.target = target;
      this.cameraToObject = cameraToObject;
      this.fieldPose = fieldPose;
    }
  }

  private static class TrackedObjectSlot {
    private boolean visible;
    private int classId = INVALID_OBJECT_ID;
    private double confidence = INVALID_CONFIDENCE;
    private double yaw;
    private double pitch;
    private double area;
    private Transform3d cameraToObject = new Transform3d();
    private Pose3d fieldPose = CLEARED_POSE;

    private void update(VisibleObject visibleObject) {
      visible = true;
      classId = visibleObject.target.getDetectedObjectClassID();
      confidence = visibleObject.target.getDetectedObjectConfidence();
      yaw = visibleObject.target.getYaw();
      pitch = visibleObject.target.getPitch();
      area = visibleObject.target.getArea();
      cameraToObject = visibleObject.cameraToObject;
      fieldPose = visibleObject.fieldPose;
    }

    private void clear() {
      visible = false;
      classId = INVALID_OBJECT_ID;
      confidence = INVALID_CONFIDENCE;
      yaw = 0.0;
      pitch = 0.0;
      area = 0.0;
      cameraToObject = new Transform3d();
      fieldPose = CLEARED_POSE;
    }
  }

  public static boolean hasObjects() {
    return s_latestResult != null && s_latestResult.hasTargets();
  }

  public static int getObjectCount() {
    return s_latestResult == null ? 0 : s_latestResult.getTargets().size();
  }

  public static void updateVisionData() {
    List<PhotonPipelineResult> unreadResults = s_camera.getAllUnreadResults();
    if (!unreadResults.isEmpty()) {
      s_latestResult = unreadResults.get(unreadResults.size() - 1);
    }

    Pose3d cameraFieldPose = getCameraFieldPose();
    List<PhotonTrackedTarget> rawObjects = s_latestResult.getTargets();
    List<VisibleObject> visibleObjects = buildVisibleObjects(rawObjects, cameraFieldPose);

    updateTrackedObjects(visibleObjects);

    Logger.recordOutput(LOG_PATH + "/cameraName", CAMERA_NAME);
    Logger.recordOutput(LOG_PATH + "/cameraConnected", s_camera.isConnected());
    Logger.recordOutput(LOG_PATH + "/cameraFieldPose", cameraFieldPose);
    Logger.recordOutput(LOG_PATH + "/hasObjects", !visibleObjects.isEmpty());
    Logger.recordOutput(LOG_PATH + "/objectCount", visibleObjects.size());

    logBestObject(cameraFieldPose);

    for (int i = 0; i < s_objectSlots.size(); i++) {
      logObjectSlot(i, s_objectSlots.get(i));
    }

    trimTrailingEmptySlots();
  }

  private static Pose3d getCameraFieldPose() {
    return new Pose3d(DriveSubsystem.getDrivetrain().getState().Pose).transformBy(ROBOT_TO_CAMERA);
  }

  private static List<VisibleObject> buildVisibleObjects(
    List<PhotonTrackedTarget> rawObjects,
    Pose3d cameraFieldPose
  ) {
    ArrayList<VisibleObject> visibleObjects = new ArrayList<>(rawObjects.size());

    for (PhotonTrackedTarget object : rawObjects) {
      Transform3d cameraToObject = object.getBestCameraToTarget();
      Pose3d fieldPose = cameraFieldPose.transformBy(cameraToObject);
      if (!isFinite(cameraToObject.getTranslation()) || !isFinite(fieldPose.getTranslation())) {
        continue;
      }

      visibleObjects.add(new VisibleObject(object, cameraToObject, fieldPose));
    }

    visibleObjects.sort(
      Comparator
        .comparingDouble((VisibleObject object) -> object.fieldPose.getY())
        .thenComparingDouble(object -> object.fieldPose.getX())
    );

    return visibleObjects;
  }

  private static void updateTrackedObjects(List<VisibleObject> visibleObjects) {
    ArrayList<Integer> unmatchedVisibleSlots = new ArrayList<>();
    for (int i = 0; i < s_objectSlots.size(); i++) {
      if (s_objectSlots.get(i).visible) {
        unmatchedVisibleSlots.add(i);
      }
    }

    for (VisibleObject visibleObject : visibleObjects) {
      int slotIndex = findBestSlot(visibleObject, unmatchedVisibleSlots);
      if (slotIndex >= 0) {
        unmatchedVisibleSlots.remove(Integer.valueOf(slotIndex));
      } else {
        slotIndex = findEmptySlot();
      }

      s_objectSlots.get(slotIndex).update(visibleObject);
    }

    for (int slotIndex : unmatchedVisibleSlots) {
      s_objectSlots.get(slotIndex).clear();
    }
  }

  private static int findBestSlot(VisibleObject visibleObject, List<Integer> candidateSlots) {
    int bestSlot = -1;
    double bestDistance = SAME_OBJECT_DISTANCE_METERS;
    Translation2d currentTranslation = visibleObject.fieldPose.getTranslation().toTranslation2d();

    for (int slotIndex : candidateSlots) {
      TrackedObjectSlot slot = s_objectSlots.get(slotIndex);
      if (slot.classId != visibleObject.target.getDetectedObjectClassID()) {
        continue;
      }

      double distance =
        slot.fieldPose.getTranslation().toTranslation2d().getDistance(currentTranslation);
      if (distance < bestDistance) {
        bestDistance = distance;
        bestSlot = slotIndex;
      }
    }

    return bestSlot;
  }

  private static int findEmptySlot() {
    for (int i = 0; i < s_objectSlots.size(); i++) {
      if (!s_objectSlots.get(i).visible) {
        return i;
      }
    }

    s_objectSlots.add(new TrackedObjectSlot());
    return s_objectSlots.size() - 1;
  }

  private static void logBestObject(Pose3d cameraFieldPose) {
    if (!s_latestResult.hasTargets()) {
      Logger.recordOutput(LOG_PATH + "/BestObject/visible", false);
      Logger.recordOutput(LOG_PATH + "/BestObject/id", INVALID_OBJECT_ID);
      Logger.recordOutput(LOG_PATH + "/BestObject/confidence", INVALID_CONFIDENCE);
      Logger.recordOutput(LOG_PATH + "/BestObject/yaw", 0.0);
      Logger.recordOutput(LOG_PATH + "/BestObject/pitch", 0.0);
      Logger.recordOutput(LOG_PATH + "/BestObject/area", 0.0);
      Logger.recordOutput(LOG_PATH + "/BestObject/cameraTranslation", CLEARED_TRANSLATION);
      Logger.recordOutput(LOG_PATH + "/BestObject/fieldPose", CLEARED_POSE);
      return;
    }

    PhotonTrackedTarget bestObject = s_latestResult.getBestTarget();
    Transform3d cameraToObject = bestObject.getBestCameraToTarget();
    Pose3d fieldPose = cameraFieldPose.transformBy(cameraToObject);

    Logger.recordOutput(LOG_PATH + "/BestObject/visible", true);
    Logger.recordOutput(LOG_PATH + "/BestObject/id", bestObject.getDetectedObjectClassID());
    Logger.recordOutput(LOG_PATH + "/BestObject/confidence", bestObject.getDetectedObjectConfidence());
    Logger.recordOutput(LOG_PATH + "/BestObject/yaw", bestObject.getYaw());
    Logger.recordOutput(LOG_PATH + "/BestObject/pitch", bestObject.getPitch());
    Logger.recordOutput(LOG_PATH + "/BestObject/area", bestObject.getArea());
    Logger.recordOutput(LOG_PATH + "/BestObject/cameraTranslation", cameraToObject.getTranslation());
    Logger.recordOutput(LOG_PATH + "/BestObject/fieldPose", fieldPose);
  }

  private static void logObjectSlot(int slotIndex, TrackedObjectSlot slot) {
    String objectPath = LOG_PATH + "/" + (slotIndex + 1) + "_Object";

    Logger.recordOutput(objectPath + "/visible", slot.visible);
    Logger.recordOutput(objectPath + "/id", slot.classId);
    Logger.recordOutput(objectPath + "/confidence", slot.confidence);
    Logger.recordOutput(objectPath + "/yaw", slot.yaw);
    Logger.recordOutput(objectPath + "/pitch", slot.pitch);
    Logger.recordOutput(objectPath + "/area", slot.area);
    Logger.recordOutput(objectPath + "/cameraTranslation", slot.cameraToObject.getTranslation());
    Logger.recordOutput(objectPath + "/fieldPose", slot.fieldPose);
  }

  private static void trimTrailingEmptySlots() {
    while (!s_objectSlots.isEmpty() && !s_objectSlots.get(s_objectSlots.size() - 1).visible) {
      s_objectSlots.remove(s_objectSlots.size() - 1);
    }
  }

  private static boolean isFinite(Translation3d translation) {
    return Double.isFinite(translation.getX())
      && Double.isFinite(translation.getY())
      && Double.isFinite(translation.getZ());
  }
}
