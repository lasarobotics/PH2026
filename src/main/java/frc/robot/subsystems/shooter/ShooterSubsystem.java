package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AimUtil;
import frc.robot.Constants;
import frc.robot.GameHelpers;
import frc.robot.HeadHoncho;
import frc.robot.subsystems.drive.DriveSubsystem;

public class ShooterSubsystem extends SubsystemBase implements AutoCloseable {

  private static ShooterSubsystem s_shooterSubsystem;
  private TalonFX m_shooterMotorLeader;
  private TalonFX m_shooterMotorFollowerOne;
  private TalonFX m_shooterMotorFollowerTwo;
  private TalonFX m_indexerMotor;
  private TalonFX m_hoodMotor;

  private final MotionMagicVelocityVoltage m_shooterRequest;
  private final MotionMagicVelocityVoltage m_indexerRequest;
  private final MotionMagicVoltage m_hoodRequest;

  private boolean m_zeroingHood = false;

  public static ShooterSubsystem getInstance() {
    if (s_shooterSubsystem == null) {
      s_shooterSubsystem = new ShooterSubsystem();
    }
    return s_shooterSubsystem;
  }

  private ShooterSubsystem() {
    m_shooterMotorLeader = new TalonFX(Constants.Shooter.LEADER_SHOOTER_MOTOR_ID);
    m_shooterMotorFollowerOne = new TalonFX(Constants.Shooter.FOLLOWER_SHOOTER_ONE_MOTOR_ID);
    m_shooterMotorFollowerTwo = new TalonFX(Constants.Shooter.FOLLOWER_SHOOTER_TWO_MOTOR_ID);
    m_indexerMotor = new TalonFX(Constants.Shooter.INDEXER_MOTOR_ID);
    m_hoodMotor = new TalonFX(Constants.Shooter.HOOD_MOTOR_ID);

    m_shooterRequest = new MotionMagicVelocityVoltage(0);
    m_indexerRequest = new MotionMagicVelocityVoltage(0);
    m_hoodRequest = new MotionMagicVoltage(0);

    // TODO set up configs
    // for reference:
    // https://github.com/lasarobotics/PH2025/blob/master/src/main/java/frc/robot/subsystems/lift/LiftSubsystem.java#L1359-L1422
    TalonFXConfiguration shooterOneConfig = new TalonFXConfiguration();
    TalonFXConfiguration shooterTwoConfig = new TalonFXConfiguration();
    TalonFXConfiguration shooterThreeConfig = new TalonFXConfiguration();

    TalonFXConfiguration indexerConfig = new TalonFXConfiguration();

    TalonFXConfiguration hoodConfig = new TalonFXConfiguration();

    m_shooterMotorLeader.getConfigurator().apply(shooterOneConfig);
    m_shooterMotorFollowerOne.getConfigurator().apply(shooterTwoConfig);
    m_shooterMotorFollowerTwo.getConfigurator().apply(shooterThreeConfig);
    m_indexerMotor.getConfigurator().apply(indexerConfig);
    m_hoodMotor.getConfigurator().apply(hoodConfig);

    // Master motor should be the one that goes in a different
    // direction than the other two
    m_shooterMotorFollowerOne.setControl(
      new Follower(m_shooterMotorLeader.getDeviceID(), MotorAlignmentValue.Opposed)
    );    
    m_shooterMotorFollowerTwo.setControl(
      new Follower(m_shooterMotorLeader.getDeviceID(), MotorAlignmentValue.Opposed)
    );
  }

  /**
   * Stops all motors.
   */
  public void stopEverything() {
    stopShooter();
    stopIndexer();
    stopHood();
  }

  /**
   * Stop the {@link #m_indexerMotor indexer motor}.
   */
  private void stopIndexer() {
    m_indexerMotor.setControl(
      m_indexerRequest.withVelocity(0)
    );
  }

  /**
   * Set the speed of the {@link #m_indexerMotor indexer motor}
   * to the (constant)
   * {@link Constants.Shooter#indexerHoldSpeed indexer hold speed}.
   */
  private void runIndexer() {
    m_indexerMotor.setControl(
      m_indexerRequest.withVelocity(Constants.Shooter.INDEXER_MOTOR_SPEED)
    );
  }

  /**
   * Stop the {@link #m_shooterMotorLeader shooter motors} (coast to 0).
   */
  private void stopShooter() {
    m_shooterMotorLeader.setVoltage(0);
  }

  /**
   * Set the speed of one {@link #m_shooterMotorLeader shooter motor}
   * to the (constant)
   * {@link Constants.Shooter#SHOOTER_HOLD_SPEED shooter hold speed}
   * and let the others coast.
   */
  private void holdShooter() {
    m_shooterMotorLeader.setControl(
      m_shooterRequest.withVelocity(Constants.Shooter.SHOOTER_HOLD_SPEED)
    );
  }

  /**
   * Set the speed of the {@link #m_shooterMotorLeader shooter motors}
   * to the desired shooting speed
   * according to {@link #wantedShooterSpeed()}
   */
  private void runShooter() {
    m_shooterMotorLeader.setControl(
      m_shooterRequest.withVelocity(wantedShooterSpeed())
    );
  }

  /**
   * 
   * @return If the master shooter motor is {@link Constants.Shooter#SHOOTER_SPEED_TOLERANCE near}
   * the desired speed according to {@link #wantedShooterSpeed()}
   */
  private boolean atShootSpeed() {
    return m_shooterMotorLeader.getVelocity().isNear(
      wantedShooterSpeed(),
      Constants.Shooter.SHOOTER_SPEED_TOLERANCE
    );
  }

  /**
   * Sets the target position of the hood to the current hood position
   * (i.e. stop the hood). Does not do anything if hood is currently
   * in the process of zeroing.
   */
  private void stopHood() {
    if (m_zeroingHood) {
      return;
    }
    double pos = m_hoodMotor.getPosition().getValueAsDouble();
    m_hoodMotor.setControl(m_hoodRequest.withPosition(pos));
  }

  /**
   * Set the setpoint of the {@link #m_hoodMotor hood motor}
   * to the desired hood position according to
   * {@link frc.robot.AimUtil#getExitAngle() getExitAngle()}.
   * Does not do anything if hood is currently
   * in the process of zeroing.
   */
  private void adjustHood() {
    if (m_zeroingHood) {
      return;
    }
    m_hoodMotor.setControl(
      m_hoodRequest.withPosition(AimUtil.getExitAngle())
    );
  }

  /**
   * Starts the hood zeroing process. Other operations to
   * hood cannot be made until this process is done. It
   * comprises setting a constant voltage, and then
   * stopping and zeroing once a hard stop is detected.
   */
  public void zeroHood() {
    m_zeroingHood = true;
    m_hoodMotor.setVoltage(Constants.Shooter.HOOD_ZERO_VOLTAGE);
  }

  /**
   * Checks if the hood position is near the exit angle
   * provided by getExitAngle(). Tolerance is determined by
   * {@link frc.robot.Constants.Shooter#HOOD_POSITION_TOLERANCE
   * a constant}.
   * @return If the check succeeds
   */
  private boolean atHoodPosition() {
    return m_hoodMotor.getPosition().isNear(
      AimUtil.getExitAngle(),
      Constants.Shooter.HOOD_POSITION_TOLERANCE
    );
  }

  /**
   * Checks that the robot can make it in if it shoots right now
   * and that the drivetrain is at the wanted rotation
   * & within the alliance zone
   * @return If robot is in a good position to shoot
   */
  private boolean readyToShoot() {
    return (
      (
        GameHelpers.scoringTimeLeft() - Constants.Field.HANG_TIME
        >= Constants.Shooter.SHOOTER_TIME_MARGIN
      ) &&
      atShootSpeed() &&
      atHoodPosition() &&
      DriveSubsystem.getInstance().atWantedRotation() &&
      DriveSubsystem.getInstance().inAllianceZone()
    );
  }

  /**
   * Checks that the robot can make it in the alliance zone if it shoots right now
   * and that the drivetrain is at the wanted rotation & not in the
   * alliance zone. Doesn't check if hub is active.
   * @return If robot is in a good position to shoot
   */
  private boolean readyToPass() {
    return (
      atShootSpeed() &&
      atHoodPosition() &&
      DriveSubsystem.getInstance().atWantedRotation() &&
      !DriveSubsystem.getInstance().inAllianceZone()
    );
  }

  /**
   * Get the current wanted ball velocity from AimUtil
   * and convert to rotations per second.
   * @return The wanted rotations per second
   */
  private double wantedShooterSpeed() {
    LinearVelocity ballVelocity = AimUtil.getBallVelocity();
    double radiansPerSecond =
      2 * ballVelocity.in(MetersPerSecond)
      / Constants.Shooter.SHOOTER_RADIUS.in(Meters);
    double rotationsPerSecond =
      radiansPerSecond / (2 * Math.PI);
    return rotationsPerSecond;
  }

  @Override
  public void periodic() {
    super.periodic();
    Logger.recordOutput(getName() + "/shooterSpeed",
      m_shooterMotorLeader.get());
    Logger.recordOutput(getName() + "/indexerSpeed",
      m_indexerMotor.get());
    Logger.recordOutput(getName() + "/hoodPosition",
      m_hoodMotor.getPosition().getValueAsDouble());
    Logger.recordOutput(getName() + "/atShootSpeed",
      atShootSpeed());
    Logger.recordOutput(getName() + "/atHoodPosition",
      atHoodPosition());

    // Periodic shooter logic. Basically:
    // Always adjust hood
    // If holding shoot/pass button:
    //    - Set shoot motor to shoot speed
    //    - Toggle indexer motor based on if ready to shoot/pass
    // If not holding button, set shoot motor to hold speed
    // and stop indexer
    boolean shooting = HeadHoncho.getInstance().wantToShoot();
    boolean passing = HeadHoncho.getInstance().wantToPass();
    Logger.recordOutput(getName() + "/wantToShoot", shooting);
    Logger.recordOutput(getName() + "/wantToPass", passing);

    if (m_zeroingHood) {
      // If the current is near the stall current
      // and the speed is near zero,
      // finish zeroing 
      if (
        m_hoodMotor.getTorqueCurrent().isNear(
          Constants.Shooter.HOOD_STALL_CURRENT,
          Constants.Shooter.HOOD_ZERO_CURRENT_TOLERANCE
        ) &&
        m_hoodMotor.getVelocity().isNear(
          0,
          Constants.Shooter.HOOD_ZERO_SPEED_TOLERANCE
        )
      ) {
        m_hoodMotor.setPosition(0.0);
        m_hoodMotor.setControl(
          m_hoodRequest.withPosition(0)
        );
        m_zeroingHood = false;
      }
    } else {
      adjustHood();
    }

    if (shooting || passing) {
      runShooter();
      boolean shootReady = readyToShoot();
      boolean passReady = readyToPass();
      Logger.recordOutput(getName() + "/readyToShoot", shootReady);
      Logger.recordOutput(getName() + "/readyToPass", passReady);

      if ((shootReady && shooting) ||
          (passReady && passing)) {
        runIndexer();
      } else {
        stopIndexer();
      }
    } else {
      holdShooter();
      stopIndexer();
    }
  }

  @Override
  public void close() {
    m_shooterMotorLeader.close();
    m_shooterMotorFollowerOne.close();
    m_shooterMotorFollowerTwo.close();
    m_indexerMotor.close();
    m_hoodMotor.close();
  }
}