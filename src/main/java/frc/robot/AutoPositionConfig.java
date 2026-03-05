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
  private Pose2d TowerShootingPose;
  private Pose2d AllianceZoneCenter;
  private Pose2d DepotEnterPose;
  private Pose2d DepotExitPose;
  
  private Pose2d AcrossBumpNZ;
  private Pose2d AcrossBumpAZ;
  private Pose2d AcrossBumpAZOpposite;
  private Pose2d NZStart;
  private Pose2d NZEnd;
  private Pose2d NZEndFull;

  public AutoPositionConfig(
    Quadrant quadrant
  ) {
    switch (quadrant) {
      case BLUE_LEFT:
        this.AllianceZoneSide =
          Constants.Auto.BLUE_LEFT_POSE;
        this.AllianceZoneOppositeSide =
          Constants.Auto.BLUE_RIGHT_POSE;
        this.TowerShootingPose =
          Constants.Auto.BLUE_TOWER_SHOOTING_POSE;
        this.AllianceZoneCenter =
          Constants.Auto.BLUE_CENTER_POSE;
        this.DepotEnterPose =
          Constants.Auto.BLUE_DEPOT_ENTER_POSE;
        this.DepotExitPose =
          Constants.Auto.BLUE_DEPOT_EXIT_POSE;

        this.AcrossBumpNZ =
          Constants.Auto.BLUE_LEFT_ABUMP_NZ_POSE;
        this.AcrossBumpAZ =
          Constants.Auto.BLUE_LEFT_ABUMP_AZ_POSE;
        this.NZStart =
          Constants.Auto.BLUE_LEFT_DEPOT_NZ_POSE;
        this.NZEnd =
          Constants.Auto.BLUE_LEFT_HUB_NZ_POSE;
        this.NZEndFull =
          Constants.Auto.BLUE_LEFT_OUTPOST_NZ_POSE;
        this.AcrossBumpAZOpposite =
          Constants.Auto.BLUE_RIGHT_ABUMP_AZ_POSE;
        break;

      case BLUE_RIGHT:
        this.AllianceZoneSide =
          Constants.Auto.BLUE_RIGHT_POSE;
        this.AllianceZoneOppositeSide =
          Constants.Auto.BLUE_LEFT_POSE;
        this.TowerShootingPose =
          Constants.Auto.BLUE_TOWER_SHOOTING_POSE;
        this.AllianceZoneCenter =
          Constants.Auto.BLUE_CENTER_POSE;
        this.DepotEnterPose =
          Constants.Auto.BLUE_DEPOT_ENTER_POSE;
        this.DepotExitPose =
          Constants.Auto.BLUE_DEPOT_EXIT_POSE;

        this.AcrossBumpNZ =
          Constants.Auto.BLUE_RIGHT_ABUMP_NZ_POSE;
        this.AcrossBumpAZ =
          Constants.Auto.BLUE_RIGHT_ABUMP_AZ_POSE;
        this.NZStart =
          Constants.Auto.BLUE_RIGHT_OUTPOST_NZ_POSE;
        this.NZEnd =
          Constants.Auto.BLUE_RIGHT_HUB_NZ_POSE;
        this.NZEndFull =
          Constants.Auto.BLUE_RIGHT_DEPOT_NZ_POSE;
        this.AcrossBumpAZOpposite =
          Constants.Auto.BLUE_LEFT_ABUMP_AZ_POSE;
        break;

      case RED_LEFT:
        this.AllianceZoneSide =
          Constants.Auto.RED_LEFT_POSE;
        this.AllianceZoneOppositeSide =
          Constants.Auto.RED_RIGHT_POSE;
        this.TowerShootingPose =
          Constants.Auto.RED_TOWER_SHOOTING_POSE;
        this.AllianceZoneCenter =
          Constants.Auto.RED_CENTER_POSE;
        this.DepotEnterPose =
          Constants.Auto.RED_DEPOT_ENTER_POSE;
        this.DepotExitPose =
          Constants.Auto.RED_DEPOT_EXIT_POSE;

        this.AcrossBumpNZ =
          Constants.Auto.RED_LEFT_ABUMP_NZ_POSE;
        this.AcrossBumpAZ =
          Constants.Auto.RED_LEFT_ABUMP_AZ_POSE;
        this.NZStart =
          Constants.Auto.RED_LEFT_DEPOT_NZ_POSE;
        this.NZEnd =
          Constants.Auto.RED_LEFT_HUB_NZ_POSE;
        this.NZEndFull =
          Constants.Auto.RED_LEFT_OUTPOST_NZ_POSE;
        this.AcrossBumpAZOpposite =
          Constants.Auto.RED_RIGHT_ABUMP_AZ_POSE;
        break;

      case RED_RIGHT:
        this.AllianceZoneSide =
          Constants.Auto.RED_RIGHT_POSE;
        this.AllianceZoneOppositeSide =
          Constants.Auto.RED_LEFT_POSE;
        this.TowerShootingPose =
          Constants.Auto.RED_TOWER_SHOOTING_POSE;
        this.AllianceZoneCenter =
          Constants.Auto.RED_CENTER_POSE;
        this.DepotEnterPose =
          Constants.Auto.RED_DEPOT_ENTER_POSE;
        this.DepotExitPose =
          Constants.Auto.RED_DEPOT_EXIT_POSE;

        this.AcrossBumpNZ =
          Constants.Auto.RED_RIGHT_ABUMP_NZ_POSE;
        this.AcrossBumpAZ =
          Constants.Auto.RED_RIGHT_ABUMP_AZ_POSE;
        this.NZStart =
          Constants.Auto.RED_RIGHT_OUTPOST_NZ_POSE;
        this.NZEnd =
          Constants.Auto.RED_RIGHT_HUB_NZ_POSE;
        this.NZEndFull =
          Constants.Auto.RED_RIGHT_DEPOT_NZ_POSE;
        this.AcrossBumpAZOpposite =
          Constants.Auto.RED_LEFT_ABUMP_AZ_POSE;
        break;
    }
  }

  public Pose2d AllianceZoneSide() {
    return AllianceZoneSide;
  }

  public Pose2d AllianceZoneOppositeSide() {
    return AllianceZoneOppositeSide;
  }

  public Pose2d TowerShootingPose() {
    return TowerShootingPose;
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

  // Position to go to to get across bump initially from AZ to NZ
  public Pose2d AcrossBumpNZPosition() {
    return AcrossBumpNZ;
  }

  // Position to go to to get across bump from NZ to AZ
  public Pose2d AcrossBumpAZPosition() {
    return AcrossBumpAZ;
  }

  // Position to go to to get across bump from NZ to AZ (uses ramp on side that didn't start on)
  public Pose2d AcrossBumpAZOppositePosition() {
    return AcrossBumpAZOpposite;
  }

  // Position to start plow
  public Pose2d NeutralZoneStartPosition() {
    return NZStart;
  }

  // Position to end plow (for short)
  public Pose2d NeutralZoneEndPosition() {
    return NZEnd;
  }

  // Position to end plow (goes across full row of balls)
  public Pose2d NeutralZoneEndPositionFull() {
    return NZEndFull;
  }

}
