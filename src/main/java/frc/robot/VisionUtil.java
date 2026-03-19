package frc.robot;

import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Translation3d;

public class VisionUtil {
  // TODO: Change to real camera name in photon vision UI
  private static final String CAMERA_NAME = "418";
  private static final String LOG_PATH = "VisionUtil";
  private static final int INVALID_FIDUCIAL_ID = -1;
  private static final double CLEARED_DOUBLE_VALUE = 0.0;
  private static final Translation3d CLEARED_POSE = new Translation3d();

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
      Logger.recordOutput(LOG_PATH + "/BestObject/id", bestObject.getDetectedObjectClassID());
      Logger.recordOutput(LOG_PATH + "/BestObject/pose", bestObject.getBestCameraToTarget().getTranslation());
    } else {
      // clears best object logs when nothing is visible
      Logger.recordOutput(LOG_PATH + "/BestObject/id", INVALID_FIDUCIAL_ID);
      Logger.recordOutput(LOG_PATH + "/BestObject/pose", CLEARED_POSE);
    }

    // logs every object currently seen
    for (int i = 0; i < objects.size(); i++) {
      PhotonTrackedTarget object = objects.get(i);
      String objectPath = LOG_PATH + "/" + (i + 1) + "_Object";
      Logger.recordOutput(objectPath + "/id", object.getDetectedObjectClassID());
      Logger.recordOutput(objectPath + "/pose", object.getAlternateCameraToTarget().getTranslation());
    }

    // clears old objects if those objects were lost since last loop
    for (int i = objects.size(); i < s_lastObjectCount; i++) {
      String objectPath = LOG_PATH + "/" + (i + 1) + "_Object";
      Logger.recordOutput(objectPath + "/id", INVALID_FIDUCIAL_ID);
      Logger.recordOutput(objectPath + "/pose", CLEARED_POSE);
    }

    s_lastObjectCount = objects.size();
  }
}
