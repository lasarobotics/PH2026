package frc.robot.subsystems.vision;

import frc.robot.fsm.StateMachine;
import frc.robot.fsm.SystemState;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.littletonrobotics.junction.Logger;

import java.util.List;

public class VisionSubsystem extends StateMachine implements AutoCloseable {
  public enum VisionStates implements SystemState {
    DRIVER_CONTROL {
      @Override
      public SystemState nextState() {
        if (s_requestedVisionState == VisionStates.AUTO_INTAKE) {
          return AUTO_INTAKE;
        }

        return this;
      }
    },
    AUTO_INTAKE {
      @Override
      public SystemState nextState() {
        if (s_requestedVisionState == VisionStates.DRIVER_CONTROL) {
          return DRIVER_CONTROL;
        }

        return this;
      }
    }
  }

  // TODO: Change to real camera name in photon vision UI
  private static final String CAMERA_NAME = "Arducam_OV9782_USB_CAMERA";
  private static VisionSubsystem s_visionSubsystem;
  private static VisionStates s_requestedVisionState;

  private final PhotonCamera m_camera;
  private int m_lastTargetCount = 0;

  public static VisionSubsystem getInstance() {
    if (s_visionSubsystem == null) {
      s_visionSubsystem = new VisionSubsystem();
    }
    return s_visionSubsystem;
  }

  private VisionSubsystem() {
    super(VisionStates.DRIVER_CONTROL);
    s_requestedVisionState = VisionStates.DRIVER_CONTROL;
    m_camera = new PhotonCamera(CAMERA_NAME);
  }

  public void autoIntake() {
    s_requestedVisionState = VisionStates.AUTO_INTAKE;
  }

  public void driverControl() {
    s_requestedVisionState = VisionStates.DRIVER_CONTROL;
  }

  @Override
  public void periodic() {
    String logPath = getName();
    var result = m_camera.getLatestResult();
    boolean hasTargets = result.hasTargets();
    List<PhotonTrackedTarget> targets = result.getTargets();

    Logger.recordOutput(logPath + "/subsystemState", getState().toString());
    Logger.recordOutput(logPath + "/cameraName", CAMERA_NAME);
    Logger.recordOutput(logPath + "/cameraConnected", m_camera.isConnected());
    Logger.recordOutput(logPath + "/hasTargets", hasTargets);
    Logger.recordOutput(logPath + "/targetCount", targets.size());
    Logger.recordOutput(logPath + "/AUTO_INTAKE/enabled", getState().equals(VisionStates.AUTO_INTAKE));

    if (hasTargets) {
      PhotonTrackedTarget bestTarget = result.getBestTarget();
      Logger.recordOutput(logPath + "/BestTarget/id", bestTarget.getFiducialId());
      Logger.recordOutput(logPath + "/BestTarget/yaw", bestTarget.getYaw());
      Logger.recordOutput(logPath + "/BestTarget/pitch", bestTarget.getPitch());
      Logger.recordOutput(logPath + "/BestTarget/area", bestTarget.getArea());
      Logger.recordOutput(logPath + "/AUTO_INTAKE/targetVisible", true);
      Logger.recordOutput(logPath + "/AUTO_INTAKE/targetYaw", bestTarget.getYaw());
    } else {
      Logger.recordOutput(logPath + "/BestTarget/id", -1);
      Logger.recordOutput(logPath + "/BestTarget/yaw", 0.0);
      Logger.recordOutput(logPath + "/BestTarget/pitch", 0.0);
      Logger.recordOutput(logPath + "/BestTarget/area", 0.0);
      Logger.recordOutput(logPath + "/AUTO_INTAKE/targetVisible", false);
      Logger.recordOutput(logPath + "/AUTO_INTAKE/targetYaw", 0.0);
    }

    // does for however many targets are seen
    for (int i = 0; i < targets.size(); i++) {
      PhotonTrackedTarget target = targets.get(i);
      String targetPath = logPath + "/" + (i + 1) + "_Target";
      Logger.recordOutput(targetPath + "/id", target.getFiducialId());
      Logger.recordOutput(targetPath + "/yaw", target.getYaw());
      Logger.recordOutput(targetPath + "/pitch", target.getPitch());
      Logger.recordOutput(targetPath + "/area", target.getArea());
      Logger.recordOutput(targetPath + "/skew", target.getSkew());
    }

    // Clear bad target slots if fewer targets are seen this cycle
    for (int i = targets.size(); i < m_lastTargetCount; i++) {
      String targetPath = logPath + "/" + (i + 1) + "_Target";
      Logger.recordOutput(targetPath + "/id", -1);
      Logger.recordOutput(targetPath + "/yaw", 0.0);
      Logger.recordOutput(targetPath + "/pitch", 0.0);
      Logger.recordOutput(targetPath + "/area", 0.0);
      Logger.recordOutput(targetPath + "/skew", 0.0);
    }

    m_lastTargetCount = targets.size();
  }

  @Override
  public void close() {
    s_visionSubsystem = null;
  }
}
