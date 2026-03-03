package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;

public class AutoPositionConfig {
  public enum Quadrant {
    BLUE_LEFT,
    BLUE_RIGHT,
    RED_LEFT,
    RED_RIGHT
  }

  private Pose2d AllianceZoneSide;
  private Pose2d AllianceZoneOppositeSide;
  private Pose2d AllianceZoneCenter;
  private Pose2d DepotEnterPose;
  private Pose2d DepotExitPose;

  public AutoPositionConfig(
    Quadrant quadrant
  ) {
    switch (quadrant) {
      case BLUE_LEFT:
        this.AllianceZoneSide =
          Constants.Auto.BLUE_LEFT_POSE;
        this.AllianceZoneOppositeSide =
          Constants.Auto.BLUE_RIGHT_POSE;
        this.AllianceZoneCenter =
          Constants.Auto.BLUE_CENTER_POSE;
        this.DepotEnterPose =
          Constants.Auto.BLUE_DEPOT_ENTER_POSE;
        this.DepotExitPose =
          Constants.Auto.BLUE_DEPOT_EXIT_POSE;
        break;

      case BLUE_RIGHT:
        this.AllianceZoneSide =
          Constants.Auto.BLUE_RIGHT_POSE;
        this.AllianceZoneOppositeSide =
          Constants.Auto.BLUE_LEFT_POSE;
        this.AllianceZoneCenter =
          Constants.Auto.BLUE_CENTER_POSE;
        this.DepotEnterPose =
          Constants.Auto.BLUE_DEPOT_ENTER_POSE;
        this.DepotExitPose =
          Constants.Auto.BLUE_DEPOT_EXIT_POSE;
        break;

      case RED_LEFT:
        this.AllianceZoneSide =
          Constants.Auto.RED_LEFT_POSE;
        this.AllianceZoneOppositeSide =
          Constants.Auto.RED_RIGHT_POSE;
        this.AllianceZoneCenter =
          Constants.Auto.RED_CENTER_POSE;
        this.DepotEnterPose =
          Constants.Auto.RED_DEPOT_ENTER_POSE;
        this.DepotExitPose =
          Constants.Auto.RED_DEPOT_EXIT_POSE;
        break;

      case RED_RIGHT:
        this.AllianceZoneSide =
          Constants.Auto.RED_RIGHT_POSE;
        this.AllianceZoneOppositeSide =
          Constants.Auto.RED_LEFT_POSE;
        this.AllianceZoneCenter =
          Constants.Auto.RED_CENTER_POSE;
        this.DepotEnterPose =
          Constants.Auto.RED_DEPOT_ENTER_POSE;
        this.DepotExitPose =
          Constants.Auto.RED_DEPOT_EXIT_POSE;
        break;
    }
  }

  public Pose2d AllianceZoneSide() {
    return AllianceZoneSide;
  }

  public Pose2d AllianceZoneOppositeSide() {
    return AllianceZoneOppositeSide;
  }

  public Pose2d AllianceZoneCenter() {
    return AllianceZoneCenter;
  }
  
  public Pose2d DepotEnterPose() {
    return DepotEnterPose;
  }
  
  public Pose2d DepotExitPose() {
    return DepotExitPose;
  }
}
