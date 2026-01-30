package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Rotations;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;
import frc.robot.GameHelpers;

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
            public void execute() {
                // set shooter to desired speed
                s_shooterSubsystem.shoot();
                // set the hood to the optimal position
                s_shooterSubsystem.adjustHood();
                if (s_shooterSubsystem.readyToShoot()) {
                    s_shooterSubsystem.index();
                } else {
                    s_shooterSubsystem.stopIndexer();
                }
            }

            @Override
            public SystemState nextState() {
                return nextState;
            }
        },
        PASSING {
            @Override
            public void execute() {
                // set shooter to desired speed
                s_shooterSubsystem.shoot();
                // set the hood to the optimal position
                s_shooterSubsystem.adjustHood();
                if (s_shooterSubsystem.readyToPass()) {
                    s_shooterSubsystem.index();
                } else {
                    s_shooterSubsystem.stopIndexer();
                }
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

    private final MotionMagicVelocityVoltage m_shooterRequest;
    private final MotionMagicVelocityVoltage m_indexerRequest;
    private final MotionMagicVoltage m_hoodRequest;

    public static ShooterSubsystem getInstance() {
        if (s_shooterSubsystem == null) {
            s_shooterSubsystem = new ShooterSubsystem();
        }
        return s_shooterSubsystem;
    }

    private ShooterSubsystem() {
        super(ShooterSubsystemStates.REST);
        nextState = ShooterSubsystemStates.REST;
        
        m_shooterMotor = new TalonFX(Constants.ShooterSubsystem.shooterMotorId);
        m_indexerMotor = new TalonFX(Constants.ShooterSubsystem.indexerMotorId);
        m_hoodMotor = new TalonFX(Constants.ShooterSubsystem.hoodMotorId);

        m_shooterRequest = new MotionMagicVelocityVoltage(0);
        m_indexerRequest = new MotionMagicVelocityVoltage(0);
        m_hoodRequest = new MotionMagicVoltage(0);

        // TODO set up configs
        // for reference:
        // https://github.com/lasarobotics/PH2025/blob/master/src/main/java/frc/robot/subsystems/lift/LiftSubsystem.java#L1359-L1422
        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();

        TalonFXConfiguration indexerConfig = new TalonFXConfiguration();

        TalonFXConfiguration hoodConfig = new TalonFXConfiguration();

        m_shooterMotor.getConfigurator().apply(shooterConfig);
        m_indexerMotor.getConfigurator().apply(indexerConfig);
        m_hoodMotor.getConfigurator().apply(hoodConfig);
    }

    /**
     * Stop the {@link #m_indexerMotor indexer motor}.
     */
    public void stopIndexer() {
        m_indexerMotor.setControl(
            m_indexerRequest.withVelocity(0)
        );
    }

    /**
     * Set the speed of the {@link #m_indexerMotor indexer motor}
     * to the (constant)
     * {@link Constants.ShooterSubsystem#indexerHoldSpeed indexer hold speed}.
     */
    public void index() {
        m_indexerMotor.setControl(
            m_indexerRequest.withVelocity(Constants.ShooterSubsystem.indexerMotorSpeed)
        );
    }

    /**
     * Stop the {@link #m_shooterMotor shooter motor} (coast to 0).
     */
    public void stopShooter() {
        m_shooterMotor.setVoltage(0);
    }

    /**
     * Set the speed of the {@link #m_shooterMotor shooter motor}
     * to the (constant)
     * {@link Constants.ShooterSubsystem#shooterHoldSpeed shooter hold speed}.
     */
    public void holdShooter() {
        m_shooterMotor.setControl(
            m_shooterRequest.withVelocity(Constants.ShooterSubsystem.shooterHoldSpeed)
        );
    }

    /**
     * Set the speed of the {@link #m_shooterMotor shooter motor}
     * to the desired shooting speed
     * according to {@link #wantedShooterSpeed()}
     */
    public void shoot() {
        m_shooterMotor.setControl(
            m_shooterRequest.withVelocity(wantedShooterSpeed())
        );
    }

    // TODO maybe use getClosedLoopError() instead?
    // getClosedLoop error would need to be configured for a higher refresh rate though
    // https://www.chiefdelphi.com/t/phoenix6-kraken-is-up-to-speed/512695/3
    // "the default [status signal for getClosedLoopError()] is 4 Hz"
    /**
     * 
     * @return If the shooter motor is {@link Constants.ShooterSubsystem#shooterSpeedTolerance near}
     * the desired speed according to {@link #wantedShooterSpeed()}
     */
    public boolean atShootSpeed() {
        return m_shooterMotor.getVelocity().isNear(
            wantedShooterSpeed(),
            Constants.ShooterSubsystem.shooterSpeedTolerance
        );
    }

    /**
     * Sets the target position of the hood to the current hood position
     * (i.e. stop the hood)
     */
    public void stopHood() {
        double pos = m_hoodMotor.getPosition().getValueAsDouble();
        m_hoodMotor.setControl(m_hoodRequest.withPosition(pos));
    }

    /**
     * Set the setpoint of the {@link #m_hoodMotor hood motor}
     * to the desired hood position
     * according to {@link #wantedHoodPosition()}
     */
    public void adjustHood() {
        m_hoodMotor.setControl(
            m_hoodRequest.withPosition(wantedHoodPosition())
        );
    }

    // TODO also check rotation/position
    /**
     * Checks that the robot can make it in if it shoots right now
     * @return If robot is in a good position to shoot
     */
    public boolean readyToShoot() {
        return (
            (
                GameHelpers.scoringTimeLeft() - getShotTime() >= Constants.ShooterSubsystem.shooterTimeMargin
            ) &&
            atShootSpeed()
        );
    }

    // TODO also check rotation/position
    /**
     * Checks that the robot can make it in the alliance zone if it shoots right now
     * (doesn't check if hub is active)
     * @return If robot is in a good position to shoot
     */
    public boolean readyToPass() {
        return atShootSpeed();
    }

    // TODO implement
    // return rotations per second
    public double wantedShooterSpeed() {
        return 0;
    }

    // TODO implement
    // return a number of rotations
    public double wantedHoodPosition() {
        return 0;
    }

    /**
     * Converts a given angle to the number of rotations
     * the hood motor would be at at that angle
     * @param angle The angle to convert
     * @return The number of rotations
     */
    public double hoodAngleToRotations(Angle angle) {
        double base = angle.in(Rotations);
        double rotations = base * Constants.ShooterSubsystem.hoodToMotorRatio;
        return rotations;
    }

    // TODO implement
    public double getShotTime() {
        return 0;
    }

    public void close() {
        m_indexerMotor.close();
        m_shooterMotor.close();
        m_hoodMotor.close();
    }
}