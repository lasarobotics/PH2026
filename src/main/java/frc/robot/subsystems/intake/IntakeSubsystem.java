package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Value;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase implements AutoCloseable {

  private static IntakeSubsystem s_intakeInstance;
  private final TalonFX m_intakeMotor;
  private final TalonFX m_armMotor;
  private final MotionMagicVoltage m_armPositionSetter;
  private CANcoder m_intakeEncoder;

  private boolean m_isIntaking;
  private boolean m_isIntakeRunning;

  /** Creates a new IntakeSubsystem */
  private IntakeSubsystem() {
    this.m_intakeMotor = new TalonFX(Constants.Intake.INTAKE_MOTOR_ID);
    this.m_armMotor = new TalonFX(Constants.Intake.ARM_MOTOR_ID);
    this.m_intakeEncoder = new CANcoder(Constants.Intake.ARM_ENCODER_ID);

    m_armPositionSetter = new MotionMagicVoltage(Radians.zero());

    m_isIntaking = false;
    m_isIntakeRunning = false;

    // Create configs for TalonFX motors
    // TODO: set the configs for these motors
    // for reference:
    // https://github.com/lasarobotics/PH2025/blob/master/src/main/java/frc/robot/subsystems/lift/LiftSubsystem.java#L1359-L1422
    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
    TalonFXConfiguration armConfig = new TalonFXConfiguration();

    // TODO: set the config for the cancoder
    CANcoderConfiguration intakeEncoderConfig = new CANcoderConfiguration();

    // Apply configs for TalonFX motors
    m_intakeMotor.getConfigurator().apply(intakeConfig);
    m_armMotor.getConfigurator().apply(armConfig);
    m_intakeEncoder.getConfigurator().apply(intakeEncoderConfig);
  }

  /**
   * Get an instance of IntakeSubsystem
   * @return Subsystem instance
   */
  public static IntakeSubsystem getInstance() {
    if (s_intakeInstance == null) {
      s_intakeInstance = new IntakeSubsystem();
    }
    return s_intakeInstance;
  }

  /**
   * If the intake is started, stop it (and vice versa).
   */
  public void toggleIntake() {
    if (m_isIntaking) {
      stopIntake();
    } else {
      startIntake();
    }
  }

  /**
   *  Deploy the intake arm and start the intake motor
   */
  public void startIntake() {
    deployIntake();
    startIntakeMotor();
    m_isIntaking = true;
  }

  /**
   *  Stow the intake arm and stop the intake motor
   */
  public void stopIntake() {
    stowIntake();
    stopIntakeMotor();
    m_isIntaking = false;
  }

  /**
   * Check if the hopper is fully extended.
   * @return True if arm motor is at fully
   * extended setpoint within a certain tolerance.
   */
  public boolean hopperDeployed() {
    return m_armMotor.getPosition().getValue().isNear(
      Constants.Intake.DEPLOY_ANGLE,
      Constants.Intake.DEPLOY_TOLERANCE
    );
  }

  /**
   * Start intake of fuel using intake motor
   */
  private void startIntakeMotor() {
    m_intakeMotor.set(Constants.Intake.INTAKE_SPEED.in(Value));
    m_isIntakeRunning = true;
  }

  /**
   * Sets intake motor running in reverse
   */
  private void reverseIntakeMotor() {
    m_intakeMotor.set(-Constants.Intake.INTAKE_SPEED.in(Value));
  }

  /**
   * Stop intake of fuel using intake motor
   */
  private void stopIntakeMotor() {
    m_intakeMotor.stopMotor();
    m_isIntakeRunning = false;
  }

  /**
   *  Move the intake arm to the extended out position
   */
  private void deployIntake() {
    m_armMotor.setControl(
      m_armPositionSetter.withPosition(Constants.Intake.DEPLOY_ANGLE)
    );
  }

  /**
   *  Move the intake arm to the stowed position
   */
  private void stowIntake() {
    m_armMotor.setControl(
      m_armPositionSetter.withPosition(Constants.Intake.STOW_ANGLE)
    );
  }

  @Override
  public void periodic() {
    super.periodic();
    Logger.recordOutput(getName() + "/intakeMotor", m_isIntakeRunning);
    Logger.recordOutput(getName() + "/isInIntake", m_isIntaking);
    Logger.recordOutput(getName() + "/intakeEncoder", m_intakeEncoder.getAbsolutePosition().getValue());
  }
  
  /**
   * Closes all the motors, makes intake instance null
   */
  @Override
  public void close() {
    m_intakeMotor.close();
    m_armMotor.close();
    m_intakeEncoder.close();
    s_intakeInstance = null;
  }
}