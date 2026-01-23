package frc.robot.subsystems.shooter;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;

public class ShooterSubsystem extends StateMachine implements AutoCloseable {

    public enum ShooterSubsystemStates implements SystemState {
        NOTHING {
            @Override
            public SystemState nextState() {
                return this;
            }
        },
        REST {
            @Override
            public void initialize() {
                // stop indexer and shooter motor
                getInstance().m_indexerMotor.set(0);
                getInstance().m_shooterMotor.set(0);
            }

            @Override
            public SystemState nextState() {
                return nextState;
            }
        },
        AUTO_ADJUST {
            @Override
            public void initialize() {
                // stop indexer motor
                getInstance().m_indexerMotor.set(0);
                getInstance().m_shooterMotor.set(
                    Constants.ShooterSubsystem.shooterHoldSpeed
                );
            }

            @Override
            public SystemState nextState() {
                return nextState;
            }
        },
        SHOOTING {
            @Override
            public void initialize() {
                getInstance().m_indexerMotor.set(
                    Constants.ShooterSubsystem.indexerMotorSpeed);
            }

            @Override
            public void execute() {
                // constantly set shooter to desired speed
                getInstance().m_shooterMotor.set(
                    getInstance().wantedShooterSpeed()
                );
            }

            @Override
            public SystemState nextState() {
                return nextState;
            }
        }
    }

    public static void setState(ShooterSubsystemStates state) {
        nextState = state;
    }

    private static ShooterSubsystem s_shooterSubsystem;
    private static ShooterSubsystemStates nextState;
    private TalonFX m_shooterMotor;
    private TalonFX m_indexerMotor;

    public static ShooterSubsystem getInstance() {
        if (s_shooterSubsystem == null) {
            s_shooterSubsystem = new ShooterSubsystem();
        }
        return s_shooterSubsystem;
    }

    private ShooterSubsystem() {
        super(ShooterSubsystemStates.REST);
        
        m_shooterMotor = new TalonFX(Constants.ShooterSubsystem.shooterMotorId);
        m_indexerMotor = new TalonFX(Constants.ShooterSubsystem.indexerMotorId);
        // TODO config these guys

    }

    // TODO implement
    // return between -1 and 1
    public double wantedShooterSpeed() {
        return 0;
    }

    public void close() {
        m_indexerMotor.close();
        m_shooterMotor.close();
    }
}