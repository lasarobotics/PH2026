package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class VisonPatherSystem extends SubsystemBase {

  private final VisionSubsystem m_vision;
  private final Supplier<Pose2d> m_robotPoseSupplier;
  private List<Translation2d> m_obstacles = new ArrayList<>();

  public VisonPatherSystem(VisionSubsystem vision, Supplier<Pose2d> robotPoseSupplier) {
    m_vision = vision;
    m_robotPoseSupplier = robotPoseSupplier;
  }

  @Override
  public void periodic() {
    Pose2d robotPose = m_robotPoseSupplier.get();
    Logger.recordOutput("VisionPath/RobotPose", robotPose);

    if (!m_vision.hasValidTarget()) {
      Logger.recordOutput("VisionPath/HasTarget", false);
      Logger.recordOutput("VisionPath/PathX", new double[0]);
      Logger.recordOutput("VisionPath/PathY", new double[0]);
      Logger.recordOutput("VisionPath/LineEquation", "none");
      return;
    }

    Logger.recordOutput("VisionPath/HasTarget", true);
    Translation2d targetRobot = m_vision.getRobotRelativeTranslation();
    Translation2d targetField =
        robotPose.getTranslation().plus(targetRobot.rotateBy(robotPose.getRotation()));

    List<Translation2d> path = runSimpleAStar(robotPose.getTranslation(), targetField);
    logPath(path);
    logLineEquation(robotPose.getTranslation(), targetField);
    logObstacles();
  }

  private List<Translation2d> runSimpleAStar(Translation2d start, Translation2d goal) {
    // quick straight-line fallback, no obsticless modeled
    List<Translation2d> path = new ArrayList<>();
    path.add(start);

    Translation2d detour = findDetour(start, goal);
    if (detour != null) {
      path.add(detour);
    }

    path.add(goal);
    return path;
  }

  private void logPath(List<Translation2d> path) {
    double[] xs = new double[path.size()];
    double[] ys = new double[path.size()];
    for (int i = 0; i < path.size(); i++) {
      xs[i] = path.get(i).getX();
      ys[i] = path.get(i).getY();
    }
    Logger.recordOutput("VisionPath/PathX", xs);
    Logger.recordOutput("VisionPath/PathY", ys);
  }

  private void logLineEquation(Translation2d start, Translation2d goal) {
    double dx = goal.getX() - start.getX();
    double dy = goal.getY() - start.getY();
    if (Math.abs(dx) < 1e-6) {
      Logger.recordOutput("VisionPath/LineEquation", "x = " + start.getX());
      return;
    }
    double slope = dy / dx;
    double intercept = start.getY() - slope * start.getX();
    Logger.recordOutput(
        "VisionPath/LineEquation", "y = " + slope + "x + " + intercept);
  }

  public void setObstacles(List<Translation2d> obstacles) {
    m_obstacles = new ArrayList<>(obstacles);
  }

  private Translation2d findDetour(Translation2d start, Translation2d goal) {
    double clearance = 0.3;
    for (Translation2d obstacle : m_obstacles) {
      double distToLine = distancePointToSegment(obstacle, start, goal);
      if (distToLine < clearance) {
        Translation2d dir = goal.minus(start);
        Translation2d perp = new Translation2d(-dir.getY(), dir.getX());
        if (perp.getNorm() < 1e-6) {
          continue;
        }
        Translation2d offset = perp.div(perp.getNorm()).times(clearance * 2.0);
        return obstacle.plus(offset);
      }
    }
    return null;
  }

  private double distancePointToSegment(Translation2d p, Translation2d a, Translation2d b) {
    Translation2d ab = b.minus(a);
    double abLenSq = ab.getNorm() * ab.getNorm();
    if (abLenSq < 1e-9) {
      return p.getDistance(a);
    }
    Translation2d ap = p.minus(a);
    double t = (ap.getX() * ab.getX() + ap.getY() * ab.getY()) / abLenSq;
    t = Math.max(0, Math.min(1, t));
    Translation2d closest = new Translation2d(a.getX() + ab.getX() * t, a.getY() + ab.getY() * t);
    return p.getDistance(closest);
  }

  private void logObstacles() {
    double[] xs = new double[m_obstacles.size()];
    double[] ys = new double[m_obstacles.size()];
    for (int i = 0; i < m_obstacles.size(); i++) {
      xs[i] = m_obstacles.get(i).getX();
      ys[i] = m_obstacles.get(i).getY();
    }
    Logger.recordOutput("VisionPath/ObstaclesX", xs);
    Logger.recordOutput("VisionPath/ObstaclesY", ys);
  }
}
