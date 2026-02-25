// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Degrees;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase implements AutoCloseable {

  private static ClimbSubsystem s_climbInstance;
  private final CANcoder m_climbEncoder;
  private final TalonFX m_climbMotor1;
  private final Servo m_climbServo;

  public static ClimbSubsystem getInstance() {
    if (s_climbInstance == null){
      s_climbInstance = new ClimbSubsystem();
    }
    return s_climbInstance;
  }

  /** Creates a new ClimbSubsystem. */
  private ClimbSubsystem() {
    this.m_climbMotor1 = new TalonFX(Constants.Climb.CLIMB_MOTOR_1_ID);
    this.m_climbEncoder = new CANcoder(Constants.Climb.ARM_ENCODER_ID);
    this.m_climbServo = new Servo(Constants.Climb.SERVO_CHANNEL);

    // TODO: Verify motor configs
    TalonFXConfiguration motorOneConfig = new TalonFXConfiguration();
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    
    motorOneConfig.MotorOutput
      .withInverted(InvertedValue.Clockwise_Positive)
      .withNeutralMode(NeutralModeValue.Brake);
    motorOneConfig.Feedback
      .withRotorToSensorRatio(125.0)
      .withSensorToMechanismRatio(1.66666666667)
      .withFeedbackRemoteSensorID(m_climbEncoder.getDeviceID())
      .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder);

    motorOneConfig.SoftwareLimitSwitch
      .withForwardSoftLimitEnable(true)
      .withReverseSoftLimitEnable(true)
      .withForwardSoftLimitThreshold(0.26)
      .withReverseSoftLimitThreshold(0.0);

    encoderConfig.MagnetSensor
      .withMagnetOffset(-0.92236328125)
      .withAbsoluteSensorDiscontinuityPoint(0.95)
      .withSensorDirection(SensorDirectionValue.Clockwise_Positive);


    this.m_climbMotor1.getConfigurator().apply(motorOneConfig);
    this.m_climbEncoder.getConfigurator().apply(encoderConfig);
  }


  /**
   * Stow the climber so it is inside the frame perimeter. If in climb position, the climber will deploy.
   * @return Command to move the climb motor to stow position
   */
  public Command stow() {
    if (!inClimbPosition()) {
      return this.runOnce(() -> m_climbMotor1.setControl(new MotionMagicVoltage(Constants.Climb.STOW_ANGLE)));
    } else {
      return deploy();
    }
  }

  /**
   * Deploy the climber and retract or stow servo
   * @return Command to deploy the climb motor and move servo appropriately
   */
  public Command deploy() {
    if (inClimbPosition()) {
      return Commands.sequence(
        deployClimbMotor(),
        runOnce(() -> m_climbServo.setAngle(Constants.Climb.SERVO_STOW_ANGLE.magnitude()))
      );
    } else {
      return Commands.sequence(
        runOnce(() -> m_climbServo.setAngle(Constants.Climb.SERVO_RETRACT_ANGLE.magnitude())),
        deployClimbMotor()
      );
    }
  }

  /**
   * Move the climber to CLIMB_ANGLE if climber is in deployed position. Otherwise deploy climber.
   * @return Command to move the climb motor to climb position
   */
  public Command climb() {
    if (inDeployPosition()) {
      return this.runOnce(() -> m_climbMotor1.setControl(new MotionMagicVoltage(Constants.Climb.CLIMB_ANGLE)));
    } else {
      return deploy();
    }
  }

  /**
   * Stops all climber motors
   */
  public void stopMotor() {
    m_climbMotor1.stopMotor();
  }

  /**
   * Check to see if climber is in stow position
   * @return A boolean for whether or not the climber is in the stow position
   */
  public boolean inStowPosition() {
    return s_climbInstance.getClimberAngle().isNear(Constants.Climb.STOW_ANGLE, Constants.Climb.CLIMB_TOLERANCE);
  }

  /**
   * Check to see if climber is in climb position
   * @return A boolean for whether or not the climber is in the climb position
   */
  public boolean inClimbPosition() {
    return s_climbInstance.getClimberAngle().isNear(Constants.Climb.CLIMB_ANGLE, Constants.Climb.CLIMB_TOLERANCE);
  }

  /**
   * Check to see if climber is in deploy position
   * @return A boolean for whether or not the climber is in the deploy position
   */
  public boolean inDeployPosition() {
    return s_climbInstance.getClimberAngle().isNear(Constants.Climb.DEPLOY_ANGLE, Constants.Climb.CLIMB_TOLERANCE)
      && getServoAngle().equals(Constants.Climb.SERVO_RETRACT_ANGLE);
  }

  /**
   * Gets the climb's encoders values
   */
  private Angle getClimberAngle() {
    return this.m_climbEncoder.getAbsolutePosition().getValue();
  }

  /**
   * Get the climb servo's angle
   * @return An Angle containing the climb servo's position
   */
  private Angle getServoAngle() {
    // TODO: Figure out this angle conversion math
    return Degrees.of(0);
  }

  /**
   * Move the climb motor to deploy position
   */
  private Command deployClimbMotor() {
    return this.runOnce(() -> m_climbMotor1.setControl(new MotionMagicVoltage(Constants.Climb.DEPLOY_ANGLE)));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput(getName() + "/encoderAngle", getClimberAngle());
    Logger.recordOutput(getName() + "/inStowPosition", inStowPosition());
    Logger.recordOutput(getName() + "/inClimbPosition", inClimbPosition());
    Logger.recordOutput(getName() + "/inDeployPosition", inDeployPosition());
  }

  @Override
  public void close() {
    m_climbMotor1.close();
    m_climbEncoder.close();
    s_climbInstance = null;
  }
}