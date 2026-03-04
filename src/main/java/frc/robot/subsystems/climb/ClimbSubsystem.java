// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Servo;
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

    TalonFXConfiguration motorOneConfig = new TalonFXConfiguration();
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    
    motorOneConfig
      .MotorOutput
        .withInverted(InvertedValue.Clockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);
    motorOneConfig
      .Feedback
        .withRotorToSensorRatio(125.0)
        .withSensorToMechanismRatio(1.66666666667)
        .withFeedbackRemoteSensorID(m_climbEncoder.getDeviceID())
        .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder);
    motorOneConfig
      .Slot0
        .withKP(1000);
    motorOneConfig
      .Voltage
        .withPeakReverseVoltage(-3);
    motorOneConfig
      .SoftwareLimitSwitch
        .withForwardSoftLimitEnable(true)
        .withReverseSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(0.26)
        .withReverseSoftLimitThreshold(0.0);
    encoderConfig
      .MagnetSensor
        .withMagnetOffset(-0.762)
        .withAbsoluteSensorDiscontinuityPoint(0.95)
        .withSensorDirection(SensorDirectionValue.Clockwise_Positive);

    // this.m_climbMotor1.getConfigurator().apply(motorOneConfig);
    this.m_climbEncoder.getConfigurator().apply(encoderConfig);

     m_climbMotor1.setControl(new PositionVoltage(Constants.Climb.STOW_ANGLE));
  }


  /**
   * Stow the climber so it is inside the frame perimeter. If in climb position, the climber will deploy.
   * @return Command to move the climb motor to stow position
   */
  public void stow() {
    Logger.recordOutput(getName() + "/commanded", "stow");
     m_climbMotor1.setControl(new PositionVoltage(Constants.Climb.STOW_ANGLE));
  }

  /**
   * Move the climber to CLIMB_ANGLE if climber is in deployed position. Otherwise deploy climber.
   * @return Command to move the climb motor to climb position
   */
  public void climb() {
    Logger.recordOutput(getName() + "/commanded", "climb");
     m_climbMotor1.setControl(new PositionVoltage(Constants.Climb.CLIMB_ANGLE));
  }

  /**
   * Stops all climber motors
   */
  public void stopMotor() {
    Logger.recordOutput(getName() + "/commanded", "stop");
    // m_climbMotor1.stopMotor();
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
    return s_climbInstance.getClimberAngle().isNear(Constants.Climb.DEPLOY_ANGLE, Constants.Climb.CLIMB_TOLERANCE);
  }

  /**
   * Gets the climb's encoders values
   */
  private Angle getClimberAngle() {
    return this.m_climbMotor1.getPosition().getValue();
  }
  
  public void setClimbServo() {
    this.m_climbServo.set(1);
  }

  public void setClimbServoZero() {
    this.m_climbServo.set(0);
  }

  public void setClimbServoPointFive() {
    this.m_climbServo.set(0.5);
  }

  /**
   * Move the climb motor to deploy position
   */
  public void deployArm() {
    Logger.recordOutput(getName() + "/commanded", "deploy");
    m_climbMotor1.setControl(new PositionVoltage(Constants.Climb.DEPLOY_ANGLE));
  }

  /**
   * Move climb servo to the retract position
   */
  public void retractServo() {
    m_climbServo.set(Constants.Climb.SERVO_RETRACT_ANGLE);
  }

  /**
   * Move climb servo to the stow position
   */
  public void stowServo() {
    m_climbServo.set(Constants.Climb.SERVO_STOW_ANGLE);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput(getName() + "/encoderAngle", getClimberAngle());
    Logger.recordOutput(getName() + "/inStowPosition", inStowPosition());
    Logger.recordOutput(getName() + "/inClimbPosition", inClimbPosition());
    Logger.recordOutput(getName() + "/inDeployPosition", inDeployPosition());
    Logger.recordOutput(getName() + "/servoAngle", m_climbServo.get());
  }

  @Override
  public void close() {
    m_climbMotor1.close();
    m_climbEncoder.close();
    s_climbInstance = null;
  }
}