package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase implements AutoCloseable {

  private static IntakeSubsystem s_intakeInstance;
  private final TalonFX m_intakeMotorLeader;
  private final TalonFX m_intakeMotorFollower;
  private final TalonFX m_armMotor;
  private final MotionMagicVoltage m_armPositionSetter;
  private CANdi m_intakeEncoder;

  private boolean m_isIntaking;
  private boolean m_isJiggling;
  private boolean m_isReversing;
  private boolean m_isIntakeRunning;

  /** Creates a new IntakeSubsystem */
  private IntakeSubsystem() {
    this.m_intakeMotorLeader = new TalonFX(Constants.Intake.INTAKE_MOTOR_LEADER_ID);
    this.m_intakeMotorFollower = new TalonFX(Constants.Intake.INTAKE_MOTOR_FOLLOWER_ID);
    this.m_armMotor = new TalonFX(Constants.Intake.ARM_MOTOR_ID);
    this.m_intakeEncoder = new CANdi(Constants.Intake.ARM_ENCODER_ID);

    m_armPositionSetter = new MotionMagicVoltage(Radians.zero());

    m_isIntaking = false;
    m_isJiggling = false;
    m_isReversing = false;
    m_isIntakeRunning = false;

    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
    TalonFXConfiguration armConfig = new TalonFXConfiguration();

    CANdiConfiguration intakeEncoderConfig = new CANdiConfiguration();

    armConfig
      .MotorOutput
        .withInverted(InvertedValue.Clockwise_Positive);
    armConfig
      .Slot0
        .withKP(40)
        .withKS(.185)
        .withKG(.4)
        .withGravityType(GravityTypeValue.Arm_Cosine)
        .withGravityArmPositionOffset(.25);
    armConfig
      .Feedback
        .withRemoteCANdiPWM1(m_intakeEncoder)
        // gearbox is 9:1, sprockets are 40:28, sprockets are 2:1 again
        .withRotorToSensorRatio((40.0 / 28.0) * 9.0 * 2);
    armConfig
      .SoftwareLimitSwitch
        .withForwardSoftLimitEnable(true)
        .withReverseSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(0.2685) // measured value
        .withReverseSoftLimitThreshold(-0.075); // zero position
    armConfig
      .MotionMagic
        .withMotionMagicCruiseVelocity(10) // measured value
        .withMotionMagicAcceleration(8); // measured value
    armConfig
      .MotorOutput
        .withNeutralMode(NeutralModeValue.Coast);

    intakeEncoderConfig
      .PWM1
        .withAbsoluteSensorOffset(0.174805)
        .withAbsoluteSensorDiscontinuityPoint(0.5);
    m_intakeMotorLeader.getConfigurator().apply(intakeConfig);
    m_intakeMotorFollower.getConfigurator().apply(intakeConfig);
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
    m_isIntaking = true;
  }

  /**
   *  Stow the intake arm and stop the intake motor[]\
   * 
   */
  public void stopIntake() {
    m_isIntaking = false;
  }

  /**
   * Enables jiggling of intake
   */
  public void jiggleOn() {
    m_isJiggling = true;
  }

  /**
   * Disables jiggling of intake. Deploys intake if intaking, stows it if otherwise.
   */
  public void jiggleOff() {
    if (!m_isJiggling) {
      return;
    }
    m_isJiggling = false;
    if (m_isIntaking) {
      deployIntake();
    } else {
      stowIntake();
    }
  }

  /**
   *  Reverse the intaking direction
   */
  public void reverseIntake(boolean shouldReverse) {
    m_isReversing = shouldReverse;
  }

  /**
   * Check if the intake is fully deployed with DEPLOY_TOLERANCE
   * @return True if arm motor is at fully
   * extended setpoint within a certain tolerance.
   */
  public boolean intakeDeployed() {
    return m_armMotor.getPosition().getValue().isNear(
      Constants.Intake.DEPLOY_ANGLE,
      Constants.Intake.DEPLOY_TOLERANCE
    );
  }
  
  /**
   * Check if intake is at jiggle position or not with DEPLOY_TOLERANCE
   * @return {@code true} if intake is at jiggle position, {@code false} if otherwise
   */
  public boolean intakeAtJigglePosition() {
    return m_armMotor.getPosition().getValue().isNear(
      Constants.Intake.JIGGLE_ANGLE,
      Constants.Intake.DEPLOY_TOLERANCE
    );
  }
  
  /**
   * Check if intake is stowed or not with DEPLOY_TOLERANCE
   * @return {@code true} if intake is at stow position, {@code false} if otherwise
   */
  public boolean intakeStowed() {
    return m_armMotor.getPosition().getValue().isNear(
      Constants.Intake.STOW_ANGLE,
      Constants.Intake.DEPLOY_TOLERANCE
    );
  }

  /**
   * Start intake of fuel using intake motor
   */
  private void runIntakeMotor() {
    m_intakeMotorLeader.set(Constants.Intake.INTAKE_SPEED);
    m_intakeMotorFollower.set(Constants.Intake.INTAKE_SPEED);
    m_isIntakeRunning = true;
  }

  /**
   * Sets intake motor running in reverse
   */
  private void reverseIntakeMotor() {
    m_intakeMotorLeader.set(-Constants.Intake.INTAKE_SPEED);
    m_intakeMotorFollower.set(-Constants.Intake.INTAKE_SPEED);
    m_isIntakeRunning = true;
  }

  /**
   * Stop intake of fuel using intake motor
   */
  private void stopIntakeMotor() {
    m_intakeMotorLeader.stopMotor();
    m_intakeMotorFollower.stopMotor();
    m_isIntakeRunning = false;
  }

  /**
   *  Move the intake arm to the extended out position
   */
  public void deployIntake() {
    m_armMotor.setControl(
      m_armPositionSetter.withPosition(Constants.Intake.DEPLOY_ANGLE)
    );
  }

  /**
   *  Move the intake arm to the stowed position
   */
  public void stowIntake() {
    m_armMotor.setControl(
      m_armPositionSetter.withPosition(Constants.Intake.STOW_ANGLE)
    );
  }

  private void intakeToJigglePosition() {
    m_armMotor.setControl(
      m_armPositionSetter.withPosition(Constants.Intake.JIGGLE_ANGLE)
    );
  }

  /**
   * Run periodically to control intake motor and intake arm and log useful data
   */
  @Override
  public void periodic() {
    super.periodic();

    boolean intakeOverheated =
      m_intakeMotorLeader.getDeviceTemp().getValue().gte(Constants.Intake.OVERHEATING_TEMP) ||
      m_intakeMotorFollower.getDeviceTemp().getValue().gte(Constants.Intake.OVERHEATING_TEMP);

    if (m_isIntaking && intakeDeployed() && !intakeOverheated) {
      if (m_isReversing) {
        reverseIntakeMotor();
      } else {
        runIntakeMotor();
      }
    } else {
      stopIntakeMotor();
    }

    if (m_isJiggling) {
      if (intakeDeployed()) {
        intakeToJigglePosition();
      } else if (intakeAtJigglePosition()) {
        deployIntake();
      } else {
        deployIntake();
      }
    }

    Logger.recordOutput(getName() + "/intakeDeployed", intakeDeployed());
    Logger.recordOutput(getName() + "/intakeAtJiggle", intakeAtJigglePosition());
    Logger.recordOutput(getName() + "/intakeAtStow", intakeStowed());
    Logger.recordOutput(getName() + "/intakeMotor", m_isIntakeRunning);
    Logger.recordOutput(getName() + "/isJiggling", m_isJiggling);
    Logger.recordOutput(getName() + "/isIntaking", m_isIntaking);
    Logger.recordOutput(getName() + "/intakeEncoder", m_intakeEncoder.getPWM1Position().getValue());
    Logger.recordOutput(getName() + "/intakeLeaderMotorTemperature", m_intakeMotorLeader.getDeviceTemp().getValue().in(Celsius));
    Logger.recordOutput(getName() + "/intakeFollowerMotorTemperature", m_intakeMotorFollower.getDeviceTemp().getValue().in(Celsius));
    Logger.recordOutput(getName() + "/intakeOverheated", intakeOverheated);
  }

  /**
   * Closes all the motors, makes intake instance null
   */
  @Override
  public void close() {
    m_intakeMotorLeader.close();
    m_intakeMotorFollower.close();
    m_armMotor.close();
    m_intakeEncoder.close();
    s_intakeInstance = null;
  }
}