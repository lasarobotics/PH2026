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
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase implements AutoCloseable {

  private static ClimbSubsystem s_climbInstance;
  private final CANcoder m_climbEncoder;
  private final TalonFX m_climbMotor1;
  private final TalonFX m_climbMotor2;
  private boolean m_isClimbing;
  private boolean m_wantToClimb;
  private boolean m_wantToDeploy;
  private boolean m_isDeployed;
  private boolean m_isClimbed;

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

    this.m_isClimbing = false;
    this.m_wantToClimb = false;
    this.m_isClimbed = false;
  }

  /**
   * Stow the climber so it is inside the frame perimeter
   */
  public void stowClimber() {
    m_isClimbing = false;
  }

  /**
   * Stops all climber motors
   */
  public void stopMotor() {
    m_climbMotor1.stopMotor();
  }

  /**
   * Set the motor output for climbing
   */
  public void extendClimber() {
    m_isClimbing = true;
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
    return s_climbInstance.getClimberAngle().lte(Constants.Climb.CLIMB_ANGLE);
  }

  /**
   * Check to see if climber is in deploy position
   * @return A boolean for whether or not the climber is in the deploy position
   */
  public boolean inDeployPosition() {
    return s_climbInstance.getClimberAngle().gte(Constants.Climb.DEPLOY_ANGLE);
  }

  /**
   * Gets the climb's encoders values
   */
  private Angle getClimberAngle() {
    return this.m_climbEncoder.getAbsolutePosition().getValue();
  }

  /**
   *  Set the motor to move to climb position
   */
  private void moveClimbUp() {
    s_climbInstance.m_climbMotor1.setControl(new MotionMagicVelocityVoltage(Constants.Climb.CLIMB_SPEED_RPS));
  }

  /**
   *  Set the motor to move to stow position
   */
  private void moveClimbDown() {
    s_climbInstance.m_climbMotor1.setControl(new MotionMagicVelocityVoltage(-Constants.Climb.CLIMB_SPEED_RPS));
  }

  @Override
  public void periodic() {
    // Handle climbing logic
    // TODO: Make sure this follows the actual climbing logic
    if (m_wantToDeploy) {
      if (!s_climbInstance.inDeployPosition()) {
        s_climbInstance.moveClimbUp();
      } else {
        m_isDeployed = true;
        // Wait for confirmation
        if (m_wantToClimb && !s_climbInstance.inClimbPosition()) {
          s_climbInstance.moveClimbDown();
          m_isClimbing = true;
        } else {
          s_climbInstance.m_climbMotor1.stopMotor();
          m_isClimbed = true;
          m_isClimbing = false;
        }
      }
    } else {
      if (!s_climbInstance.inStowPosition()) {
        s_climbInstance.moveClimbDown();
      } else {
        s_climbInstance.m_climbMotor1.stopMotor();
        m_isClimbed = false;
        m_isDeployed = false;
      }
    }

    // This method will be called once per scheduler run
    Logger.recordOutput(getName() + "/encoderAngle", getClimberAngle());
    Logger.recordOutput(getName() + "/inStowPosition", inStowPosition());
    Logger.recordOutput(getName() + "/inClimbPosition", inClimbPosition());
    Logger.recordOutput(getName() + "/isClimbing", m_isClimbing);
    Logger.recordOutput(getName() + "/isClimbed", m_isClimbed);
    Logger.recordOutput(getName() + "/wantToClimb", m_wantToClimb);
    Logger.recordOutput(getName() + "/wantToDeploy", m_wantToDeploy);
    Logger.recordOutput(getName() + "/isDeployed", m_isDeployed);
  }

  @Override
  public void close() {
    m_climbMotor1.close();
    m_climbMotor2.close();
    m_climbEncoder.close();
    s_climbInstance = null;
  }
}