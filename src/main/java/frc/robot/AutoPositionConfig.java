package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.drive.DriveSubsystem;

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
  private Pose2d TowerPose;

  public AutoPositionConfig(
    Quadrant quadrant
  ) {
    this.TowerPose = DriveSubsystem.s_climbPosition;

    switch (quadrant) {
      case BLUE_LEFT:
        this.AllianceZoneSide =
          Constants.Auto.BLUE_LEFT_POSE;
        this.AllianceZoneOppositeSide =
          Constants.Auto.BLUE_RIGHT_POSE;
        this.AllianceZoneCenter =
          Constants.Auto.BLUE_CENTER_POSE;
      case BLUE_RIGHT:
        this.AllianceZoneSide =
          Constants.Auto.BLUE_RIGHT_POSE;
        this.AllianceZoneOppositeSide =
          Constants.Auto.BLUE_LEFT_POSE;
        this.AllianceZoneCenter =
          Constants.Auto.BLUE_CENTER_POSE;
      case RED_LEFT:
        this.AllianceZoneSide =
          Constants.Auto.RED_LEFT_POSE;
        this.AllianceZoneOppositeSide =
          Constants.Auto.RED_RIGHT_POSE;
        this.AllianceZoneCenter =
          Constants.Auto.RED_CENTER_POSE;
      case RED_RIGHT:
        this.AllianceZoneSide =
          Constants.Auto.RED_RIGHT_POSE;
        this.AllianceZoneOppositeSide =
          Constants.Auto.RED_LEFT_POSE;
        this.AllianceZoneCenter =
          Constants.Auto.RED_CENTER_POSE;
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

  public Pose2d TowerPose() {
    return TowerPose;
  }
}

// blue right middle side:
// targetPose = new Pose2d(
//   new Translation2d(
//     2.5,
//     1.5
//   ),
//   new Rotation2d(
//     Degrees.of(45)
//   )
// );