package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;

public record AutoPositionConfig(
  Pose2d AllianceZoneMiddleSide,
  Pose2d AllianceZoneMiddleOppositeSide,
  Pose2d AllianceZoneCenter,
  Pose2d TowerPose
) {}

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