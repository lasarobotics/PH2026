package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AimUtil;
import frc.robot.Constants;
import frc.robot.GameHelpers;
import frc.robot.HeadHoncho;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class ShooterSubsystem extends SubsystemBase implements AutoCloseable {

  private static ShooterSubsystem s_shooterSubsystem;
  private TalonFX m_shooterMotorLeader;
  private TalonFX m_shooterMotorFollowerOne;
  private TalonFX m_shooterMotorFollowerTwo;
  private TalonFX m_indexerMotor;
  private TalonFX m_hoodMotor;
  // private TalonFX m_agitatorMotor;
  private CANcoder m_hoodCanCoder;

  private final VelocityVoltage m_shooterVoltageRequest;
  private final VelocityDutyCycle m_shooterDutyCycleRequest;
  private final MotionMagicVoltage m_hoodRequest;
  private final DutyCycleOut m_indexerRequest;

  private boolean m_isRunning = true;

  /**
   * Get an instance of ShooterSubsystem
   * @return Subsystem instance
   */
  public static ShooterSubsystem getInstance() {
    if (s_shooterSubsystem == null) {
      s_shooterSubsystem = new ShooterSubsystem();
    }
    return s_shooterSubsystem;
  }

  private ShooterSubsystem() {
    m_shooterMotorLeader = new TalonFX(Constants.Shooter.LEADER_SHOOTER_MOTOR_ID, "canivore");
    m_shooterMotorFollowerOne = new TalonFX(Constants.Shooter.FOLLOWER_SHOOTER_ONE_MOTOR_ID, "canivore");
    m_shooterMotorFollowerTwo = new TalonFX(Constants.Shooter.FOLLOWER_SHOOTER_TWO_MOTOR_ID, "canivore");
    m_indexerMotor = new TalonFX(Constants.Shooter.INDEXER_MOTOR_ID);
    m_hoodMotor = new TalonFX(Constants.Shooter.HOOD_MOTOR_ID);
    m_hoodCanCoder = new CANcoder(Constants.Shooter.HOOD_CANCODER_ID);

    m_shooterVoltageRequest = new VelocityVoltage(0);
    m_shooterDutyCycleRequest = new VelocityDutyCycle(0);
    m_hoodRequest = new MotionMagicVoltage(0);
    m_indexerRequest = new DutyCycleOut(0);

    TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
    shooterConfig
      .Feedback
        .withSensorToMechanismRatio(36.0/48.0);
    shooterConfig
      .Slot0
        .withKP(999999.0);
    shooterConfig
      .MotorOutput
        .withInverted(InvertedValue.Clockwise_Positive)
        .withPeakForwardDutyCycle(1.0)
        .withPeakReverseDutyCycle(0.0);
    // https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/api-usage/status-signals.html
    m_shooterMotorLeader.getDutyCycle().setUpdateFrequency(1000);
    m_shooterMotorLeader.getMotorVoltage().setUpdateFrequency(100);
    m_shooterMotorLeader.getTorqueCurrent().setUpdateFrequency(100);
    ParentDevice.optimizeBusUtilizationForAll(
      m_shooterMotorFollowerOne,
      m_shooterMotorFollowerTwo
    );

    TalonFXConfiguration indexerConfig = new TalonFXConfiguration();

    TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
    hoodConfig
      .MotorOutput
        .withInverted(InvertedValue.CounterClockwise_Positive);
    hoodConfig
      .Feedback
        .withRotorToSensorRatio(42.0 / 8.0)
        .withSensorToMechanismRatio(127.0 / 13.0)
        .withRemoteCANcoder(m_hoodCanCoder);

    hoodConfig
      .SoftwareLimitSwitch
        .withForwardSoftLimitEnable(true)
        .withReverseSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(-0.012207) // TODO remove once bolt is gone
        .withReverseSoftLimitThreshold(-0.09668);
    hoodConfig
      .Slot0
        .withKP(300)
        .withKD(3)
        .withKS(0.515);
    hoodConfig
      .MotionMagic
        .withMotionMagicCruiseVelocity(100)
        .withMotionMagicAcceleration(15);

    CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
    canCoderConfig.MagnetSensor
      .withAbsoluteSensorDiscontinuityPoint(0.05)
      .withMagnetOffset(0.799560546875)
      .withSensorDirection(SensorDirectionValue.Clockwise_Positive);
      
    m_shooterMotorLeader.getConfigurator().apply(shooterConfig);
    m_shooterMotorFollowerOne.getConfigurator().apply(shooterConfig);
    m_shooterMotorFollowerTwo.getConfigurator().apply(shooterConfig);
    m_indexerMotor.getConfigurator().apply(indexerConfig);
    m_hoodMotor.getConfigurator().apply(hoodConfig);
    m_hoodCanCoder.getConfigurator().apply(canCoderConfig);

    m_shooterMotorFollowerOne.setControl(
      new Follower(m_shooterMotorLeader.getDeviceID(), MotorAlignmentValue.Aligned)
    );
    m_shooterMotorFollowerTwo.setControl(
      new Follower(m_shooterMotorLeader.getDeviceID(), MotorAlignmentValue.Aligned)
    );
  }

  /**
   * Allows the shooter subsystem to operate as normal.
   */
  public void startOperation() {
    m_isRunning = true;
  }

  /**
   * Force the shooter subsystem to stop operation and
   * stop motors.
   */
  public void stopOperation() {
    m_isRunning = false;
  }

  /**
   * Stops all motors.
   */
  private void stopEverything() {
    stopShooter();
    stopIndexer();
    stopHood();
  }

  /**
   * Stop the {@link #m_indexerMotor indexer motor}.
   */
  private void stopIndexer() {
    m_indexerMotor.setVoltage(0);
    // IntakeSubsystem.getInstance().jiggleOff();
  }

  /**
   * Set the speed of the {@link #m_indexerMotor indexer motor}
   * to the (constant)
   * {@link Constants.Shooter#INDEXER_MOTOR_SPEED indexer run speed}.
   */
  private void runIndexer() {
    m_indexerMotor.setControl(
      m_indexerRequest.withOutput(Constants.Shooter.INDEXER_MOTOR_SPEED)
    );
    // IntakeSubsystem.getInstance().jiggleOn();
  }
  
  /**
   * Stop the {@link #m_shooterMotorLeader shooter motors} (coast to 0).
   */
  private void stopShooter() {
    m_shooterMotorLeader.setVoltage(0);
  }

  /**
   * Set the speed of the {@link #m_shooterMotorLeader shooter motors}
   * to the desired shooting speed according to {@link #wantedShooterSpeed()}.
   * If we're too far above the wanted shooter speed, run with a
   * velocity (non-BangBang) controller. This is because BangBang can only
   * drive up, and we don't want to be waiting for multiple seconds for the
   * flywheel to spin down on its own. At a tolerance of 2 rotations per second,
   * it takes somewhere between 0.1 and 0.2 seconds to spin down back to the
   * target velocity.
   */
  private void runShooter() {
    if (
      m_shooterMotorLeader.getVelocity().getValue().in(RotationsPerSecond)
      > wantedShooterSpeed() + Constants.Shooter.SHOOTER_SPEED_ABOVE_TOLERANCE
    ) {
      // too high above - run voltage request
      m_shooterMotorLeader.setControl(
        m_shooterVoltageRequest.withVelocity(wantedShooterSpeed())
      );
    } else {
      // below or within tolerance - run bang bang
      m_shooterMotorLeader.setControl(
        m_shooterDutyCycleRequest.withVelocity(wantedShooterSpeed())
      );
    }
  }

  /**
   * Determine whether robot is at speed where it is able to shoot
   * @return If the master shooter motor is {@link Constants.Shooter#SHOOTER_SPEED_TOLERANCE near}
   * the desired speed according to {@link #wantedShooterSpeed()}
   */
  private boolean atShootSpeed() {
    return (
      m_shooterMotorLeader.getVelocity().getValue().in(RotationsPerSecond) >=
        wantedShooterSpeed()
    );
  }

  /**
   * Sets the target position of the hood to the
   * current hood position (i.e. stop the hood).
   */
  private void stopHood() {
    //double pos = m_hoodMotor.getPosition().getValueAsDouble();
    //m_hoodMotor.setControl(m_hoodRequest.withPosition(pos));
  }

  /**
   * Set the setpoint of the {@link #m_hoodMotor hood motor}
   * to the desired hood position according to
   * {@link frc.robot.AimUtil#getExitAngle() getExitAngle()}.
   */
  private void adjustHood() {
   Angle positionAngle = wantedHoodPosition();
    m_hoodMotor.setControl(
      m_hoodRequest.withPosition(positionAngle)
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
      // dumb shoot shouldn't care about robot orientation
      // but we still want to check shooter speed &
      // hood position before indexing
      (
        HeadHoncho.getInstance().wantToDumbShoot() ||
        DriveSubsystem.getInstance().atWantedRotation()
      )
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
    return m_hoodMotor.getPosition().getValue().isNear(
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
      // return Constants.Shooter.DUMB_SHOOTER_SPEED;
      return Constants.Shooter.DUMB_SHOOTER_SPEED.get();
    }

    LinearVelocity ballVelocity = AimUtil.getBallVelocity();
    double rotationsPerSecond =
      ballVelocity.in(MetersPerSecond) /
      (2 * Math.PI * Constants.Shooter.SHOOTER_RADIUS.in(Meters));
    return rotationsPerSecond;
  }

  /**
   * Returns the exit angle wanted by AimUtil, unless
   * the driver wants to dumb shoot, in which case this
   * method returns the constant dumb hood position.
   * @return The angle that the hood should be at. This
   * value accounts for the offset of the hood relative
   * to the exit angle.
   */
  private Angle wantedHoodPosition() {
    if (HeadHoncho.getInstance().wantToDumbShoot()) {
      // return Constants.Shooter.DUMB_HOOD_POSITION;
      return Degrees.of(Constants.Shooter.DUMB_HOOD_POSITION.get())
        .minus(Degrees.of(80));
    }

    return AimUtil.getExitAngle().minus(Degrees.of(80.0));
  }

  @Override
  public void periodic() {
    super.periodic();

    // Periodic shooter logic. Basically (if in operation):
    // Always adjust hood
    // If holding shoot button:
    // (shoot/forceshoot/dumbshoot)
    //    - Set shoot motor to shoot speed
    //    - Toggle indexer motor based on if ready to shoot
    //    - Indexer is always turned on if forceshoot is held
    // If not holding button, set shoot motor to hold speed
    // and stop indexer
    // If not in operation, just stop everything
    boolean shooting = HeadHoncho.getInstance().wantToShoot();
    boolean dumbShooting = HeadHoncho.getInstance().wantToDumbShoot();
    boolean forceShooting = HeadHoncho.getInstance().wantToForceShoot();
    Logger.recordOutput(getName() + "/wantToShoot", shooting);
    Logger.recordOutput(getName() + "/wantToDumbShoot", dumbShooting);
    Logger.recordOutput(getName() + "/wantToForceShoot", forceShooting);
    Logger.recordOutput(getName() + "/inAllianceZone", DriveSubsystem.inAllianceZone());
    

    if (m_isRunning) {
      adjustHood();

      if (shooting || forceShooting || dumbShooting) {
        runShooter();
        IntakeSubsystem.getInstance().jiggleOn();
        // IntakeSubsystem.getInstance().deployIntake();

        // If the shooter is ready (rpm, position, hood) and
        // the time/pass check succeeds, then ready to shoot
        // If we're not in the alliance zone, we're passing
        // and we can always pass, so succeed
        // If we're in the alliance zone, we're shooting, and
        // we only want to shoot during active shift

        boolean readyToShoot = (
          shooterReady() &&
          (
            !DriveSubsystem.inAllianceZone() ||
            (
              GameHelpers.scoringTimeLeft() - AimUtil.getHangTime().in(Seconds)
              >= Constants.Shooter.SHOOTER_TIME_MARGIN
            )
          )
        );

        Logger.recordOutput(getName() + "/readyToShoot", readyToShoot);

        if ((readyToShoot && shooting) ||
            // BTW: wantedHoodPosition & wantedShooterSpeed
            // return the dumb constants and readyToShoot
            // doesn't check orientation if the dumb
            // shooting button is held
            (readyToShoot && dumbShooting) ||
            forceShooting
        ) {
          runIndexer();
        } else {
          stopIndexer();
        }
      } else {
        // holdShooter();
        IntakeSubsystem.getInstance().jiggleOff();
        stopShooter();
        stopIndexer();
      }
    } else {
      IntakeSubsystem.getInstance().jiggleOff();
      stopEverything();
    }
    
    Logger.recordOutput(getName() + "/shooterSpeed",
      m_shooterMotorLeader.getVelocity().getValue().in(RotationsPerSecond));
    Logger.recordOutput(getName() + "/indexerSpeed",
      m_indexerMotor.get());
    Logger.recordOutput(getName() + "/hoodPosition",
      m_hoodMotor.getPosition().getValue().in(Degrees));
    Logger.recordOutput(getName() + "/atShootSpeed",
      atShootSpeed());
    Logger.recordOutput(getName() + "/shooterReady",
      shooterReady());
    Logger.recordOutput(getName() + "/atHoodPosition",
      atHoodPosition());
    Logger.recordOutput(getName() + "/wantedShooterSpeed",
      wantedShooterSpeed());
    Logger.recordOutput(getName() + "/wantedHoodPosition",
      wantedHoodPosition().in(Degrees));
    Logger.recordOutput(getName() + "/isRunning",
      m_isRunning);
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