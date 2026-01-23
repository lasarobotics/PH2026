package frc.robot.subsystems.shooter;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
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
                s_shooterSubsystem.stopShooter();
                s_shooterSubsystem.stopIndexer();
                s_shooterSubsystem.stopHood();
            }

            @Override
            public SystemState nextState() {
                return nextState;
            }
        },
        AUTO_ADJUST {
            @Override
            public void initialize() {
                // set shooter motor to hold speed
                s_shooterSubsystem.holdShooter();
                // stop indexer motor
                s_shooterSubsystem.stopIndexer();
            }

            @Override
            public void execute() {
                // set the hood to the optimal position
                s_shooterSubsystem.adjustHood();
            }

            @Override
            public SystemState nextState() {
                return nextState;
            }
        },
        SHOOTING {
            @Override
            public void initialize() {
                s_shooterSubsystem.index();
            }

            @Override
            public void execute() {
                // set shooter to desired speed
                s_shooterSubsystem.shoot();
                // set the hood to the optimal position
                s_shooterSubsystem.adjustHood();
            }

            @Override
            public SystemState nextState() {
                return nextState;
            }
        }
    }

    /**
     * Sets the next state of the {@link #ShooterSubsystem shooter subsystem}
     * @param state The state to be set.
     */
    public static void setState(ShooterSubsystemStates state) {
        nextState = state;
    }

    private static ShooterSubsystem s_shooterSubsystem;
    private static ShooterSubsystemStates nextState;
    private TalonFX m_shooterMotor;
    private TalonFX m_indexerMotor;
    private TalonFX m_hoodMotor;

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
        m_hoodMotor = new TalonFX(Constants.ShooterSubsystem.hoodMotorId);

        // TODO set up configs
        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
        TalonFXConfiguration indexerConfig = new TalonFXConfiguration();
        TalonFXConfiguration hoodConfig = new TalonFXConfiguration();

        m_shooterMotor.getConfigurator().apply(shooterConfig);        m_indexerMotor.getConfigurator().apply(indexerConfig);
        m_indexerMotor.getConfigurator().apply(indexerConfig);
        m_hoodMotor.getConfigurator().apply(hoodConfig);

    }

    /**
     * Stop the {@link #m_indexerMotor indexer motor}.
     */
    public void stopIndexer() {
        m_indexerMotor.set(0);
    }

    /**
     * Set the speed of the {@link #m_indexerMotor indexer motor}
     * to the (constant)
     * {@link Constants.ShooterSubsystem#indexerHoldSpeed indexer hold speed}.
     */
    public void index() {
        m_indexerMotor.set(Constants.ShooterSubsystem.indexerMotorSpeed);
    }

    /**
     * Stop the {@link #m_shooterMotor shooter motor}.
     */
    public void stopShooter() {
        m_shooterMotor.set(0);
    }

    /**
     * Set the speed of the {@link #m_shooterMotor shooter motor}
     * to the (constant)
     * {@link Constants.ShooterSubsystem#shooterHoldSpeed shooter hold speed}.
     */
    public void holdShooter() {
        m_shooterMotor.set(Constants.ShooterSubsystem.shooterHoldSpeed);
    }

    /**
     * Set the speed of the {@link #m_shooterMotor shooter motor}
     * to the desired shooting speed
     * according to {@link #wantedShooterSpeed()}
     */
    public void shoot() {
        m_shooterMotor.set(wantedShooterSpeed());
    }

    /**
     * Sets the target position of the hood to the current hood position
     * (i.e. stop the hood)
     */
    public void stopHood() {
        PositionVoltage control = new PositionVoltage(
            m_hoodMotor.getPosition().getValue()
        );
        m_hoodMotor.setControl(control);
    }

    /**
     * Set the setpoint of the {@link #m_hoodMotor hood motor}
     * to the desired hood position
     * according to {@link #wantedHoodPosition()}
     */
    public void adjustHood() {
        PositionVoltage control = new PositionVoltage(
            wantedHoodPosition()
        );
        m_hoodMotor.setControl(control);
    }

    // TODO implement
    // return between -1 and 1
    public double wantedShooterSpeed() {
        return 0;
    }

    // TODO implement
    // return a number of rotations
    public double wantedHoodPosition() {
        return 0;
    }

    public void close() {
        m_indexerMotor.close();
        m_shooterMotor.close();
        m_hoodMotor.close();
    }
}