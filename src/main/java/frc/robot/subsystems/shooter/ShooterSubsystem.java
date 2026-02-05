package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;

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
import frc.robot.subsystems.drive.DriveSubsystem;

public class ShooterSubsystem extends SubsystemBase implements AutoCloseable {

  private static ShooterSubsystem s_shooterSubsystem;
  private TalonFX m_shooterMotorMaster;
  private TalonFX m_shooterMotorSlaveOne;
  private TalonFX m_shooterMotorSlaveTwo;
  private TalonFX m_indexerMotor;
  private TalonFX m_hoodMotor;

  private final MotionMagicVelocityVoltage m_shooterRequest;
  private final MotionMagicVelocityVoltage m_indexerRequest;
  private final MotionMagicVoltage m_hoodRequest;

  public static ShooterSubsystem getInstance() {
    if (s_shooterSubsystem == null) {
      s_shooterSubsystem = new ShooterSubsystem();
    }
    return s_shooterSubsystem;
  }

  private ShooterSubsystem() {
    m_shooterMotorMaster = new TalonFX(Constants.Shooter.MASTER_SHOOTER_MOTOR_ID);
    m_shooterMotorSlaveOne = new TalonFX(Constants.Shooter.SLAVE_SHOOTER_ONE_MOTOR_ID);
    m_shooterMotorSlaveTwo = new TalonFX(Constants.Shooter.SLAVE_SHOOTER_TWO_MOTOR_ID);
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

    m_shooterMotorMaster.getConfigurator().apply(shooterOneConfig);
    m_shooterMotorSlaveOne.getConfigurator().apply(shooterTwoConfig);
    m_shooterMotorSlaveTwo.getConfigurator().apply(shooterThreeConfig);
    m_indexerMotor.getConfigurator().apply(indexerConfig);
    m_hoodMotor.getConfigurator().apply(hoodConfig);
  }

  /**
   * Periodic shooter logic. Basically:
   * <ul>
   * <li>Always adjust hood
   * <li>If holding shoot/pass button:
   * <ul>
     * <li>Set shoot motor to shoot speed
     * <li>Toggle indexer motor based on if ready to shoot/pass
   * </ul>
   * <li>If not holding button, set shoot motor to hold speed
   * and stop indexer
   * </ul>
   * @param shooting If shoot button is held
   * @param passing If pass button is held
   */
  public void shooterPeriodic(boolean shooting, boolean passing) {
    Logger.recordOutput(getName() + "/wantToShoot", shooting);
    Logger.recordOutput(getName() + "/wantToPass", passing);
    
    adjustHood();

    if (shooting || passing) {
      shoot();
      boolean shootReady = readyToShoot();
      boolean passReady = readyToPass();
      Logger.recordOutput(getName() + "/readyToShoot", shootReady);
      Logger.recordOutput(getName() + "/readyToPass", passReady);

      if ((shootReady && shooting) ||
          (passReady && passing)) {
        index();
      } else {
        stopIndexer();
      }
    } else {
      holdShooter();
      stopIndexer();
    }
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
  public void stopIndexer() {
    m_indexerMotor.setControl(
      m_indexerRequest.withVelocity(0)
    );
  }

  /**
   * Set the speed of the {@link #m_indexerMotor indexer motor}
   * to the (constant)
   * {@link Constants.Shooter#indexerHoldSpeed indexer hold speed}.
   */
  public void index() {
    m_indexerMotor.setControl(
      m_indexerRequest.withVelocity(Constants.Shooter.INDEXER_MOTOR_SPEED)
    );
  }

  /**
   * Stop the {@link #m_shooterMotorMaster shooter motors} (coast to 0).
   */
  public void stopShooter() {
    m_shooterMotorMaster.setVoltage(0);
    m_shooterMotorSlaveOne.setVoltage(0);
    m_shooterMotorSlaveTwo.setVoltage(0);
  }

  /**
   * Set the speed of one {@link #m_shooterMotorMaster shooter motor}
   * to the (constant)
   * {@link Constants.Shooter#SHOOTER_HOLD_SPEED shooter hold speed}
   * and let the others coast.
   */
  public void holdShooter() {
    m_shooterMotorMaster.setControl(
      m_shooterRequest.withVelocity(Constants.Shooter.SHOOTER_HOLD_SPEED)
    );

    //TODO: figure out the alignment of the slave motors with respect to the master
    m_shooterMotorSlaveOne.setControl(
      new Follower(m_shooterMotorMaster.getDeviceID(), MotorAlignmentValue.Aligned)
    );
    m_shooterMotorSlaveTwo.setControl(
      new Follower(m_shooterMotorMaster.getDeviceID(), MotorAlignmentValue.Aligned)
    );
  }

  /**
   * Set the speed of the {@link #m_shooterMotorMaster shooter motors}
   * to the desired shooting speed
   * according to {@link #wantedShooterSpeed()}
   */
  public void shoot() {
    m_shooterMotorMaster.setControl(
      m_shooterRequest.withVelocity(wantedShooterSpeed())
    );

    //TODO: figure out the alignment of the slave motors with respect to the master
    m_shooterMotorSlaveOne.setControl(
      new Follower(m_shooterMotorMaster.getDeviceID(), MotorAlignmentValue.Aligned)
    );
    m_shooterMotorSlaveTwo.setControl(
      new Follower(m_shooterMotorMaster.getDeviceID(), MotorAlignmentValue.Aligned)
    );
  }

  /**
   * 
   * @return If the master shooter motor is {@link Constants.Shooter#SHOOTER_SPEED_TOLERANCE near}
   * the desired speed according to {@link #wantedShooterSpeed()}
   */
  public boolean atShootSpeed() {
    return m_shooterMotorMaster.getVelocity().isNear(
      wantedShooterSpeed(),
      Constants.Shooter.SHOOTER_SPEED_TOLERANCE
    );
  }

  /**
   * Sets the target position of the hood to the current hood position
   * (i.e. stop the hood)
   */
  public void stopHood() {
    double pos = m_hoodMotor.getPosition().getValueAsDouble();
    m_hoodMotor.setControl(m_hoodRequest.withPosition(pos));
  }

  /**
   * Set the setpoint of the {@link #m_hoodMotor hood motor}
   * to the desired hood position
   * according to {@link #wantedHoodPosition()}
   */
  public void adjustHood() {
    m_hoodMotor.setControl(
      m_hoodRequest.withPosition(wantedHoodPosition())
    );
  }

  public boolean atHoodPosition() {
    return m_hoodMotor.getPosition().isNear(
      wantedHoodPosition(),
      Constants.Shooter.HOOD_POSITION_TOLERANCE
    );
  }

  /**
   * Checks that the robot can make it in if it shoots right now
   * and that the drivetrain is at the wanted rotation
   * & within the alliance zone
   * @return If robot is in a good position to shoot
   */
  public boolean readyToShoot() {
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

  // TODO do we want to disallow passing from the alliance zone?
  /**
   * Checks that the robot can make it in the alliance zone if it shoots right now
   * and that the drivetrain is at the wanted rotation
   * (doesn't check if hub is active)
   * @return If robot is in a good position to shoot
   */
  public boolean readyToPass() {
    return (
      atShootSpeed() &&
      atHoodPosition() &&
      DriveSubsystem.getInstance().atWantedRotation()
    );
  }

  /**
   * Get the current wanted ball velocity from AimUtil
   * and convert to rotations per second.
   * @return The wanted rotations per second
   */
  public double wantedShooterSpeed() {
    LinearVelocity ballVelocity = AimUtil.getBallVelocity();
    double radiansPerSecond =
      2 * ballVelocity.in(MetersPerSecond)
      / Constants.Shooter.SHOOTER_RADIUS.in(Meters);
    double rotationsPerSecond =
      radiansPerSecond / (2 * Math.PI);
    return rotationsPerSecond;
  }

  // TODO implement
  // return a number of rotations
  public double wantedHoodPosition() {
    Angle theta = AimUtil.getExitAngle();
    return 0;
  }

  /**
   * Converts a given angle to the number of rotations
   * the hood motor would be at at that angle
   * @param angle The angle to convert
   * @return The number of rotations
   */
  public double hoodAngleToRotations(Angle angle) {
    double base = angle.in(Rotations);
    double rotations = base * Constants.Shooter.HOOD_TO_MOTOR_RATIO;
    return rotations;
  }

  @Override
  public void periodic() {
    super.periodic();
    Logger.recordOutput(getName() + "/shooterSpeed",
      m_shooterMotorMaster.get());
    Logger.recordOutput(getName() + "/indexerSpeed",
      m_indexerMotor.get());
    Logger.recordOutput(getName() + "/hoodPosition",
      m_hoodMotor.getPosition().getValueAsDouble());
    Logger.recordOutput(getName() + "/atShootSpeed",
      atShootSpeed());
    Logger.recordOutput(getName() + "/atHoodPosition",
      atHoodPosition());
  }

  @Override
  public void close() {
    m_shooterMotorMaster.close();
    m_shooterMotorSlaveOne.close();
    m_shooterMotorSlaveTwo.close();
    m_indexerMotor.close();
    m_hoodMotor.close();
  }
}