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

import edu.wpi.first.units.measure.Angle;
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
  private final MotionMagicVoltage m_hoodRequest;

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
    m_indexerMotor.setVoltage(0);
  }

  /**
   * Set the speed of the {@link #m_indexerMotor indexer motor}
   * to the (constant)
   * {@link Constants.Shooter#indexerHoldSpeed indexer hold speed}.
   */
  private void runIndexer() {
    m_indexerMotor.setVoltage(Constants.Shooter.INDEXER_MOTOR_SPEED);
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
   * Sets the target position of the hood to the
   * current hood position (i.e. stop the hood).
   */
  private void stopHood() {
    double pos = m_hoodMotor.getPosition().getValueAsDouble();
    m_hoodMotor.setControl(m_hoodRequest.withPosition(pos));
  }

  /**
   * Set the setpoint of the {@link #m_hoodMotor hood motor}
   * to the desired hood position according to
   * {@link frc.robot.AimUtil#getExitAngle() getExitAngle()}.
   */
  private void adjustHood() {
    m_hoodMotor.setControl(
      m_hoodRequest.withPosition(wantedHoodPosition())
    );
  }

  /**
   * Checks the shooter speed, hood position, and robot orientation to see if they
   * match the ones wanted by AimUtil.
   * @return True if all the checks pass.
   */
  private boolean shooterReady() {
    return (
      atShootSpeed() &&
      atHoodPosition() &&
      DriveSubsystem.getInstance().atWantedRotation()
    );
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
      wantedHoodPosition(),
      Constants.Shooter.HOOD_POSITION_TOLERANCE
    );
  }

  /**
   * Get the current wanted ball velocity from AimUtil
   * and convert to rotations per second. If the
   * dumb shoot button is being held, return the
   * constant dumb speed instead.
   * @return The wanted rotations per second
   */
  private double wantedShooterSpeed() {
    if (HeadHoncho.getInstance().wantToDumbShoot()) {
      return Constants.Shooter.DUMB_SHOOTER_SPEED;
    }

    LinearVelocity ballVelocity = AimUtil.getBallVelocity();
    double radiansPerSecond =
      2 * ballVelocity.in(MetersPerSecond)
      / Constants.Shooter.SHOOTER_RADIUS.in(Meters);
    double rotationsPerSecond =
      radiansPerSecond / (2 * Math.PI);
    return rotationsPerSecond;
  }

  /**
   * Returns the exit angle wanted by AimUtil, unless
   * the driver wants to dumb shoot, in which case this
   * method returns the constant dumb hood position.
   * @return The angle that the hood should be at.
   */
  private Angle wantedHoodPosition() {
    if (HeadHoncho.getInstance().wantToDumbShoot()) {
      return Constants.Shooter.DUMB_HOOD_POSITION;
    }

    return AimUtil.getExitAngle();
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
    Logger.recordOutput(getName() + "/wantedShooterSpeed",
      wantedShooterSpeed());
    Logger.recordOutput(getName() + "/wantedHoodPosition",
      wantedHoodPosition());

    // Periodic shooter logic. Basically:
    // Always adjust hood
    // If holding shoot button:
    // (pass/shoot/forceshoot/dumbshoot)
    //    - Set shoot motor to shoot speed
    //    - Toggle indexer motor based on if ready to shoot/pass
    //    - Indexer is always turned on if forceshoot is held
    // If not holding button, set shoot motor to hold speed
    // and stop indexer
    boolean shooting = HeadHoncho.getInstance().wantToShoot();
    boolean passing = HeadHoncho.getInstance().wantToPass();
    boolean dumbShooting = HeadHoncho.getInstance().wantToDumbShoot();
    boolean forceShooting = HeadHoncho.getInstance().wantToForceShoot();
    Logger.recordOutput(getName() + "/wantToShoot", shooting);
    Logger.recordOutput(getName() + "/wantToPass", passing);
    Logger.recordOutput(getName() + "/wantToDumbShoot", dumbShooting);
    Logger.recordOutput(getName() + "/wantToForceShoot", forceShooting);

    adjustHood();

    if (shooting || passing || forceShooting || dumbShooting) {
      runShooter();
      // TODO: Is this the right way to do the ready to shoot checks?
      // It feels kinda messy, because this is basically unpacking
      // two different methods
      boolean shooterIsReady = shooterReady();
      boolean readyToShoot = (
        shooterIsReady &&
        DriveSubsystem.getInstance().inAllianceZone() &&
        (
          // TODO maybe unpack this (time) into its own method?
          GameHelpers.scoringTimeLeft() - Constants.Field.HANG_TIME
          >= Constants.Shooter.SHOOTER_TIME_MARGIN
        )
      );
      boolean readyToPass = (
        shooterIsReady && 
        !DriveSubsystem.getInstance().inAllianceZone()
      );

      Logger.recordOutput(getName() + "/shooterIsReady", shooterIsReady);
      Logger.recordOutput(getName() + "/readyToShoot", readyToShoot);
      Logger.recordOutput(getName() + "/readyToPass", readyToPass);

      if ((readyToShoot && shooting) ||
          (readyToPass && passing) ||
          forceShooting ||
          // TODO do we want dumbshoot to force indexing
          // or do we want it to also check shootReady
          // BTW: wantedHoodPosition & wantedShooterSpeed
          // return the dumb constants if the dumb shooting
          // button is held
          dumbShooting) {
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