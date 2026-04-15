package frc.robot;

import static edu.wpi.first.math.geometry.Rotation2d.k180deg;
import static frc.robot.Constants.Auto.BLUE_CENTER_POSE;
import static frc.robot.Constants.Auto.BLUE_DEPOT_ENTER_POSE;
import static frc.robot.Constants.Auto.BLUE_DEPOT_EXIT_POSE;
import static frc.robot.Constants.Auto.BLUE_LEFT_ABUMP_AZ_POSE;
import static frc.robot.Constants.Auto.BLUE_LEFT_ABUMP_NZ_POSE;
import static frc.robot.Constants.Auto.BLUE_LEFT_DEPOT_NZ_POSE;
import static frc.robot.Constants.Auto.BLUE_LEFT_HUB_NZ_POSE;
import static frc.robot.Constants.Auto.BLUE_LEFT_OUTPOST_NZ_POSE;
import static frc.robot.Constants.Auto.BLUE_LEFT_PLOW_1_NZ_POSE;
import static frc.robot.Constants.Auto.BLUE_LEFT_DEPOT_CLOSE_NZ_POSE;
import static frc.robot.Constants.Auto.BLUE_LEFT_HUB_CLOSE_NZ_POSE;
import static frc.robot.Constants.Auto.BLUE_LEFT_OUTPOST_CLOSE_NZ_POSE;
import static frc.robot.Constants.Auto.BLUE_LEFT_POSE;
import static frc.robot.Constants.Auto.BLUE_RIGHT_ABUMP_AZ_POSE;
import static frc.robot.Constants.Auto.BLUE_RIGHT_ABUMP_NZ_POSE;
import static frc.robot.Constants.Auto.BLUE_RIGHT_DEPOT_NZ_POSE;
import static frc.robot.Constants.Auto.BLUE_RIGHT_HUB_NZ_POSE;
import static frc.robot.Constants.Auto.BLUE_RIGHT_OUTPOST_NZ_POSE;
import static frc.robot.Constants.Auto.BLUE_RIGHT_PLOW_1_NZ_POSE;
import static frc.robot.Constants.Auto.BLUE_RIGHT_DEPOT_CLOSE_NZ_POSE;
import static frc.robot.Constants.Auto.BLUE_RIGHT_HUB_CLOSE_NZ_POSE;
import static frc.robot.Constants.Auto.BLUE_RIGHT_OUTPOST_CLOSE_NZ_POSE;
import static frc.robot.Constants.Auto.BLUE_RIGHT_POSE;
import static frc.robot.Constants.Auto.rotate180;
import static frc.robot.Constants.Field.FIELD_CENTER;

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
  
  private Pose2d AcrossBumpNZ;
  private Pose2d AcrossBumpNZOpposite;
  private Pose2d AcrossBumpAZ;
  private Pose2d AcrossBumpNZHeadingFlipped;
  private Pose2d AcrossBumpAZOpposite;
  private Pose2d NZStart;
  private Pose2d Plow1;
  private Pose2d NZEnd;
  private Pose2d NZEndFull;
  private Pose2d NZDoubleTapStart;
  private Pose2d NZDoubleTapEnd;

  public AutoPositionConfig(
    Quadrant quadrant
  ) {
    switch (quadrant) {
      case BLUE_LEFT:
        this.AllianceZoneSide =
          BLUE_LEFT_POSE;
        this.AllianceZoneOppositeSide =
          BLUE_RIGHT_POSE;
        this.AllianceZoneCenter =
          BLUE_CENTER_POSE;
        this.DepotEnterPose =
          BLUE_DEPOT_ENTER_POSE;
        this.DepotExitPose =
          BLUE_DEPOT_EXIT_POSE;

        this.AcrossBumpNZ =
          BLUE_LEFT_ABUMP_NZ_POSE;
        this.AcrossBumpNZHeadingFlipped =
          rotate180(BLUE_LEFT_ABUMP_NZ_POSE);
        this.AcrossBumpNZOpposite =
          BLUE_RIGHT_ABUMP_NZ_POSE;
        this.AcrossBumpAZ =
          BLUE_LEFT_ABUMP_AZ_POSE;
        this.Plow1 = 
          BLUE_LEFT_PLOW_1_NZ_POSE;
        this.NZStart =
          BLUE_LEFT_OUTPOST_NZ_POSE;
        this.NZEnd =
          BLUE_LEFT_HUB_NZ_POSE;
        this.Plow1 =
          BLUE_LEFT_PLOW_1_NZ_POSE;
        this.NZEndFull =
          BLUE_LEFT_OUTPOST_NZ_POSE;
        this.AcrossBumpAZOpposite =
          BLUE_RIGHT_ABUMP_AZ_POSE;
        this.NZDoubleTapStart =
          BLUE_LEFT_DEPOT_CLOSE_NZ_POSE;
        this.NZDoubleTapEnd =
          BLUE_LEFT_HUB_CLOSE_NZ_POSE;
        break;

      case BLUE_RIGHT:
        this.AllianceZoneSide =
          BLUE_RIGHT_POSE;
        this.AllianceZoneOppositeSide =
          BLUE_LEFT_POSE;
        this.AllianceZoneCenter =
          BLUE_CENTER_POSE;
        this.DepotEnterPose =
          BLUE_DEPOT_ENTER_POSE;
        this.DepotExitPose =
          BLUE_DEPOT_EXIT_POSE;

        this.AcrossBumpNZ =
          BLUE_RIGHT_ABUMP_NZ_POSE;
        this.AcrossBumpNZHeadingFlipped =
          rotate180(BLUE_RIGHT_ABUMP_NZ_POSE);
        this.AcrossBumpNZOpposite =
          BLUE_LEFT_ABUMP_NZ_POSE;
        this.AcrossBumpAZ =
          BLUE_RIGHT_ABUMP_AZ_POSE;
        this.Plow1 = 
          BLUE_RIGHT_PLOW_1_NZ_POSE;
        this.NZStart =
          BLUE_RIGHT_OUTPOST_NZ_POSE;
        this.NZEnd =
          BLUE_RIGHT_HUB_NZ_POSE;
        this.NZEndFull =
          BLUE_RIGHT_DEPOT_NZ_POSE;
        this.AcrossBumpAZOpposite =
          BLUE_LEFT_ABUMP_AZ_POSE;
        this.NZDoubleTapStart =
          BLUE_RIGHT_OUTPOST_CLOSE_NZ_POSE;
        this.NZDoubleTapEnd =
          BLUE_RIGHT_HUB_CLOSE_NZ_POSE;
        break;

      case RED_LEFT:
        this.AllianceZoneSide =
          BLUE_LEFT_POSE.rotateAround(FIELD_CENTER, k180deg);
        this.AllianceZoneOppositeSide =
          BLUE_RIGHT_POSE.rotateAround(FIELD_CENTER, k180deg);
        this.AllianceZoneCenter =
          BLUE_CENTER_POSE.rotateAround(FIELD_CENTER, k180deg);
        this.DepotEnterPose =
          BLUE_DEPOT_ENTER_POSE.rotateAround(FIELD_CENTER, k180deg);
        this.DepotExitPose =
          BLUE_DEPOT_EXIT_POSE.rotateAround(FIELD_CENTER, k180deg);

        this.AcrossBumpNZ =
          BLUE_LEFT_ABUMP_NZ_POSE.rotateAround(FIELD_CENTER, k180deg);
        this.AcrossBumpNZHeadingFlipped =
          rotate180(BLUE_LEFT_ABUMP_NZ_POSE.rotateAround(FIELD_CENTER, k180deg));
        this.AcrossBumpNZOpposite =
          BLUE_RIGHT_ABUMP_NZ_POSE.rotateAround(FIELD_CENTER, k180deg);
        this.AcrossBumpAZ =
          BLUE_LEFT_ABUMP_AZ_POSE.rotateAround(FIELD_CENTER, k180deg);
        this.Plow1 = 
          BLUE_LEFT_PLOW_1_NZ_POSE.rotateAround(FIELD_CENTER, k180deg);
        this.NZStart =
          BLUE_LEFT_OUTPOST_NZ_POSE.rotateAround(FIELD_CENTER, k180deg);
        this.NZEnd =
          BLUE_LEFT_HUB_NZ_POSE.rotateAround(FIELD_CENTER, k180deg);
        this.NZEndFull =
          BLUE_LEFT_OUTPOST_NZ_POSE.rotateAround(FIELD_CENTER, k180deg);
        this.AcrossBumpAZOpposite =
          BLUE_RIGHT_ABUMP_AZ_POSE.rotateAround(FIELD_CENTER, k180deg);
        this.NZDoubleTapStart =
          BLUE_LEFT_DEPOT_CLOSE_NZ_POSE.rotateAround(FIELD_CENTER, k180deg);
        this.NZDoubleTapEnd =
          BLUE_LEFT_HUB_CLOSE_NZ_POSE.rotateAround(FIELD_CENTER, k180deg);
        break;

      case RED_RIGHT:
        this.AllianceZoneSide =
          BLUE_RIGHT_POSE.rotateAround(FIELD_CENTER, k180deg);
        this.AllianceZoneOppositeSide =
          BLUE_LEFT_POSE.rotateAround(FIELD_CENTER, k180deg);
        this.AllianceZoneCenter =
          BLUE_CENTER_POSE.rotateAround(FIELD_CENTER, k180deg);
        this.DepotEnterPose =
          BLUE_DEPOT_ENTER_POSE.rotateAround(FIELD_CENTER, k180deg);
        this.DepotExitPose =
          BLUE_DEPOT_EXIT_POSE.rotateAround(FIELD_CENTER, k180deg);

        this.AcrossBumpNZ =
          BLUE_RIGHT_ABUMP_NZ_POSE.rotateAround(FIELD_CENTER, k180deg);
        this.AcrossBumpNZHeadingFlipped =
          rotate180(BLUE_RIGHT_ABUMP_NZ_POSE.rotateAround(FIELD_CENTER, k180deg));
        this.AcrossBumpNZOpposite =
          BLUE_LEFT_ABUMP_NZ_POSE.rotateAround(FIELD_CENTER, k180deg);
        this.AcrossBumpAZ =
          BLUE_RIGHT_ABUMP_AZ_POSE.rotateAround(FIELD_CENTER, k180deg);
        this.Plow1 = 
          BLUE_RIGHT_PLOW_1_NZ_POSE.rotateAround(FIELD_CENTER, k180deg);
        this.NZStart =
          BLUE_RIGHT_OUTPOST_NZ_POSE.rotateAround(FIELD_CENTER, k180deg);
        this.NZEnd =
          BLUE_RIGHT_HUB_NZ_POSE.rotateAround(FIELD_CENTER, k180deg);
        this.NZEndFull =
          BLUE_RIGHT_DEPOT_NZ_POSE.rotateAround(FIELD_CENTER, k180deg);
        this.AcrossBumpAZOpposite =
          BLUE_LEFT_ABUMP_AZ_POSE.rotateAround(FIELD_CENTER, k180deg);
        this.NZDoubleTapStart =
          BLUE_RIGHT_OUTPOST_CLOSE_NZ_POSE.rotateAround(FIELD_CENTER, k180deg);
        this.NZDoubleTapEnd =
          BLUE_RIGHT_HUB_CLOSE_NZ_POSE.rotateAround(FIELD_CENTER, k180deg);
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

  // Position to go to to get across bump initially from AZ to NZ
  public Pose2d AcrossBumpNZ() {
    return AcrossBumpNZ;
  }

  // Position to go to to get across bump initially from AZ to NZ
  public Pose2d AcrossBumpNZHeadingFlipped() {
    return AcrossBumpNZHeadingFlipped;
  }

  // Position to go to to get across bump initially from AZ to NZ
  public Pose2d AcrossBumpNZOpposite() {
    return AcrossBumpNZOpposite;
  }

  // Position to go to to get across bump from NZ to AZ
  public Pose2d AcrossBumpAZ() {
    return AcrossBumpAZ;
  }

  // Position to go to to get across bump from NZ to AZ (uses ramp on side that didn't start on)
  public Pose2d AcrossBumpAZOpposite() {
    return AcrossBumpAZOpposite;
  }

  //Go to the PLOW_1 position before you plow in auto
  public Pose2d Plow1() {
    return Plow1;
  }

  // Position to start plow
  public Pose2d NeutralZoneStart() {
    return NZStart;
  }


  // Position to end plow (for short)
  public Pose2d NeutralZoneEnd() {
    return NZEnd;
  }

  // Position to end plow (goes across full row of balls)
  public Pose2d NeutralZoneEndFull() {
    return NZEndFull;
  }

  public Pose2d NeutralZoneDoubleTapStart() {
    return NZDoubleTapStart;
  }

  public Pose2d NeutralZoneDoubleTapEnd() {
    return NZDoubleTapEnd;
  }
}
