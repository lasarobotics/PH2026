// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase implements AutoCloseable {

  private static ClimbSubsystem s_climbInstance;
  private final CANcoder m_climbEncoder;
  private final TalonFX m_climbMotor1;
  private final TalonFX m_climbMotor2;

  public static ClimbSubsystem getInstance() {
    if (s_climbInstance == null){
      s_climbInstance = new ClimbSubsystem();
    }
    return s_climbInstance;
  }

  /** Creates a new ClimbSubsystem. */
  private ClimbSubsystem() {

    this.m_climbEncoder = new CANcoder(Constants.Climb.ARM_ENCODER_ID);
    this.m_climbMotor1 = new TalonFX(Constants.Climb.CLIMB_MOTOR_1_ID);
    this.m_climbMotor2 = new TalonFX(Constants.Climb.CLIMB_MOTOR_2_ID);

    // TODO: Verify motor configs
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    this.m_climbMotor1.getConfigurator().apply(motorConfig);
    this.m_climbMotor2.getConfigurator().apply(motorConfig);

    // Make climbMotor2 follow climbMotor1
    this.m_climbMotor2.setControl(new Follower(this.m_climbMotor1.getDeviceID(), MotorAlignmentValue.Aligned));
  }

  /**
   * Stow the climber so it is inside the frame perimeter
   */
  public void stow() {
    while (!inStowPosition()) {
      m_climbMotor1.set(-Constants.Climb.CLIMB_SPEED_SLOW);
    }
  }

  /**
   * Stops all climber motors
   */
  public void stopMotor() {
    m_climbMotor1.stopMotor();
  }

  /**
   * Sets the motor output for climbing
   */
  public void climb() {
    while (!inClimbPosition()) {
      m_climbMotor1.set(Constants.Climb.CLIMB_SPEED);
    }
  }

  /**
   * Check to see if climber is in stow position
   * @return A boolean for whether or not the climber is in the stow position
   */
  public boolean inStowPosition() {
    return s_climbInstance.getClimberAngle().lte(Constants.Climb.STOW_ANGLE);
  }

  /**
   * Check to see if climber is in climb position
   * @return A boolean for whether or not the climber is in the climb position
   */
  public boolean inClimbPosition() {
    return s_climbInstance.getClimberAngle().gte(Constants.Climb.CLIMB_ANGLE);
  }

  /**
   * Gets the climb's encoders values
   */
  private Angle getClimberAngle() {
    return this.m_climbEncoder.getAbsolutePosition().getValue();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput(getName() + "/encoderAngle", s_climbInstance.getClimberAngle());
    Logger.recordOutput(getName() + "/inStowPosition", s_climbInstance.inStowPosition());
    Logger.recordOutput(getName() + "/inClimbPosition", s_climbInstance.inClimbPosition());
  }

  @Override
  public void close() {
    m_climbMotor1.close();
    m_climbMotor2.close();
    m_climbEncoder.close();
    s_climbInstance = null;
  }
}