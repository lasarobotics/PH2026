package frc.robot;

import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionUtil {
  // TODO: Change to real camera name in photon vision UI
  private static final String CAMERA_NAME = "Arducam_OV9782_USB_CAMERA";
  private static final String LOG_PATH = "VisionUtil";
  private static final int INVALID_FIDUCIAL_ID = -1;
  private static final double CLEARED_DOUBLE_VALUE = 0.0;

  private static final PhotonCamera s_camera = new PhotonCamera(CAMERA_NAME);
  private static PhotonPipelineResult s_latestResult = s_camera.getLatestResult();
  private static int s_lastObjectCount = 0;

  private VisionUtil() {}

  public static boolean hasObjects() {
    return s_latestResult != null && s_latestResult.hasTargets();
  }

  public static int getObjectCount() {
    return s_latestResult == null ? 0 : s_latestResult.getTargets().size();
  }

  public static double getBestObjectYaw() {
    if (!hasObjects()) {
      return CLEARED_DOUBLE_VALUE;
    }

    return s_latestResult.getBestTarget().getYaw();
  }

  public static double getBestObjectPitch() {
    if (!hasObjects()) {
      return CLEARED_DOUBLE_VALUE;
    }

    return s_latestResult.getBestTarget().getPitch();
  }

  public static double getBestObjectArea() {
    if (!hasObjects()) {
      return CLEARED_DOUBLE_VALUE;
    }

    return s_latestResult.getBestTarget().getArea();
  }

  public static void updateVisionData() {
    s_latestResult = s_camera.getLatestResult();

    boolean hasObjects = hasObjects();
    List<PhotonTrackedTarget> objects = s_latestResult.getTargets();

    // basic camera and pipeline status logs
    Logger.recordOutput(LOG_PATH + "/cameraName", CAMERA_NAME);
    Logger.recordOutput(LOG_PATH + "/cameraConnected", s_camera.isConnected());
    Logger.recordOutput(LOG_PATH + "/hasObjects", hasObjects);
    Logger.recordOutput(LOG_PATH + "/objectCount", objects.size());

    if (hasObjects) {
      // logs best object according to Photon Vision, basicaly biggest bounding box
      PhotonTrackedTarget bestObject = s_latestResult.getBestTarget();
      Logger.recordOutput(LOG_PATH + "/BestObject/id", bestObject.getFiducialId());
      Logger.recordOutput(LOG_PATH + "/BestObject/yaw", bestObject.getYaw());
      Logger.recordOutput(LOG_PATH + "/BestObject/pitch", bestObject.getPitch());
      Logger.recordOutput(LOG_PATH + "/BestObject/area", bestObject.getArea());
    } else {
      // clears best object logs when nothing is visible
      Logger.recordOutput(LOG_PATH + "/BestObject/id", INVALID_FIDUCIAL_ID);
      Logger.recordOutput(LOG_PATH + "/BestObject/yaw", CLEARED_DOUBLE_VALUE);
      Logger.recordOutput(LOG_PATH + "/BestObject/pitch", CLEARED_DOUBLE_VALUE);
      Logger.recordOutput(LOG_PATH + "/BestObject/area", CLEARED_DOUBLE_VALUE);
    }

    // logs every object currently seen
    for (int i = 0; i < objects.size(); i++) {
      PhotonTrackedTarget object = objects.get(i);
      String objectPath = LOG_PATH + "/" + (i + 1) + "_Object";
      Logger.recordOutput(objectPath + "/id", object.getFiducialId());
      Logger.recordOutput(objectPath + "/yaw", object.getYaw());
      Logger.recordOutput(objectPath + "/pitch", object.getPitch());
      Logger.recordOutput(objectPath + "/area", object.getArea());
      Logger.recordOutput(objectPath + "/skew", object.getSkew());
    }

    // clears old objects if those objects were lost since last loop
    for (int i = objects.size(); i < s_lastObjectCount; i++) {
      String objectPath = LOG_PATH + "/" + (i + 1) + "_Object";
      Logger.recordOutput(objectPath + "/id", INVALID_FIDUCIAL_ID);
      Logger.recordOutput(objectPath + "/yaw", CLEARED_DOUBLE_VALUE);
      Logger.recordOutput(objectPath + "/pitch", CLEARED_DOUBLE_VALUE);
      Logger.recordOutput(objectPath + "/area", CLEARED_DOUBLE_VALUE);
      Logger.recordOutput(objectPath + "/skew", CLEARED_DOUBLE_VALUE);
    }

    s_lastObjectCount = objects.size();
  }
}
