package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Value;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.measure.Angle;

import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase implements AutoCloseable {

  // TODO maybe these should be in constants instead? -mw
  // TODO: find values of intake positions
  static final Dimensionless INTAKE_SPEED = Value.of(0.6);
  static final Angle STOW_ANGLE = Degrees.of(0);
  static final Angle DEPLOY_ANGLE = Degrees.of(0);

  private static IntakeSubsystem s_intakeInstance;
  private final TalonFX m_intakeMotor;
  private final TalonFX m_armMotor;
  private final MotionMagicVoltage m_armPositionSetter;
  private CANcoder m_armEncoder;

  private boolean m_isIntaking;
  private boolean m_isIntakeRunning;

  /** Creates a new IntakeSubsystem */
  private IntakeSubsystem() {
    this.m_intakeMotor = new TalonFX(Constants.Intake.INTAKE_MOTOR_ID);
    this.m_armMotor = new TalonFX(Constants.Intake.ARM_MOTOR_ID);
    this.m_armEncoder = new CANcoder(Constants.Intake.ARM_ENCODER_ID);

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
    CANcoderConfiguration armEncoderConfig = new CANcoderConfiguration();

    // Apply configs for TalonFX motors
    m_intakeMotor.getConfigurator().apply(intakeConfig);
    m_armMotor.getConfigurator().apply(armConfig);
    m_armEncoder.getConfigurator().apply(armEncoderConfig);
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
    deployArm();
    startIntakeMotor();
    m_isIntaking = true;
  }

  /**
   *  Stow the intake arm and stop the intake motor
   */
  public void stopIntake() {
    stowArm();
    stopIntakeMotor();
    m_isIntaking = false;
  }

  /**
   * Start intake of fuel using intake motor
   */
  private void startIntakeMotor() {
    m_intakeMotor.set(INTAKE_SPEED.in(Value));
    m_isIntakeRunning = true;
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
  private void deployArm() {
    m_armMotor.setControl(m_armPositionSetter.withPosition(DEPLOY_ANGLE));
  }

  /**
   *  Move the intake arm to the stowed position
   */
  private void stowArm() {
    m_armMotor.setControl(m_armPositionSetter.withPosition(STOW_ANGLE));
  }

  @Override
  public void periodic() {
    super.periodic();
    Logger.recordOutput(getName() + "/intakeMotor", m_isIntakeRunning);
    Logger.recordOutput(getName() + "/isInIntake", m_isIntaking);
    Logger.recordOutput(getName() + "/armEncoder", m_armEncoder.getAbsolutePosition().getValue());
  }
  
  /**
   * Closes all the motors, makes intake instance null
   */
  @Override
  public void close() {
    m_intakeMotor.close();
    m_armMotor.close();
    m_armEncoder.close();
    s_intakeInstance = null;
  }
}