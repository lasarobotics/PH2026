// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Degrees;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;

import frc.robot.Constants;

public class ClimbSubsystem extends StateMachine implements AutoCloseable {

  // TODO: Find the values of climb constants
  static final double CLIMB_SPEED = 0.0;
  static final double CLIMB_SPEED_SLOW = CLIMB_SPEED * 0.2;
  static final Angle CLIMB_ANGLE = Degrees.of(0);
  static final Angle STOW_ANGLE = Degrees.of(0);

  public enum ClimbStates implements SystemState {
    NOTHING {
      @Override
      public ClimbStates nextState() {
        return ClimbStates.NOTHING;
      }
    },
    STOW {
      @Override
      public void initialize() { 
        s_climbInstance.stow();
      }

      @Override
      public void execute() {
        if (!s_climbInstance.inStowPosition()) {
          s_climbInstance.stow();
        } else {
          s_climbInstance.stopMotor();
        }
      }

      @Override
      public ClimbStates nextState() {
        return s_climbInstance.nextState;
      }
    },
    CLIMB {
      @Override
      public void initialize() {
        s_climbInstance.climb();
      }

      @Override
      public void execute() {
        if(!s_climbInstance.inClimbPosition()){
          s_climbInstance.climb();
        } else {
          s_climbInstance.stopMotor();
        }
      }

      @Override
      public ClimbStates nextState() {
        return s_climbInstance.nextState;
      }
    }
  }

  private static ClimbSubsystem s_climbInstance;
  private final CANcoder m_climbEncoder;
  private final TalonFX m_climbMotor1;
  private final TalonFX m_climbMotor2;
  private ClimbStates nextState;

  /** Creates a new ClimbSubsystem. */
  private ClimbSubsystem() {
    super(ClimbStates.STOW);
    nextState =  ClimbStates.STOW;

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
   * Gets new instance of Climb Subsystem
   * @param ClimbHardware contains hardware for climb subsystem
   * @return ClimbSubsystem object
   */
  public static ClimbSubsystem getInstance() {
    if (s_climbInstance == null){
      s_climbInstance = new ClimbSubsystem();
    }
    return s_climbInstance;
  }

  /**
   * Sets {@code nextState} instance variable to given {@code ClimbState}
   * @param targetState The new state for {@code nextState}
   */
  public void setNextState(ClimbStates targetState) {
    this.nextState = targetState;
  }

  /**
   * Stow the climber so it is inside the frame perimeter
   */
  public void stow() {
    m_climbMotor1.set(-CLIMB_SPEED_SLOW);
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
    m_climbMotor1.set(CLIMB_SPEED);
  }

  /**
   * Check to see if climber is in stow position
   * @return A boolean for whether or not the climber is in the stow position
   */
  public boolean inStowPosition() {
    return s_climbInstance.getClimberAngle().lte(STOW_ANGLE);
  }

  /**
   * Check to see if climber is in climb position
   * @return A boolean for whether or not the climber is in the climb position
   */
  public boolean inClimbPosition() {
    return s_climbInstance.getClimberAngle().gte(CLIMB_ANGLE);
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
    Logger.recordOutput(getName() + "/state", getState().toString());
  }

  @Override
  public void close() {
    m_climbMotor1.close();
    m_climbMotor2.close();
    m_climbEncoder.close();
    s_climbInstance = null;
  }
}