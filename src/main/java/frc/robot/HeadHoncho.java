package frc.robot;

import java.util.function.BooleanSupplier;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;

import frc.robot.subsystems.shooter.ShooterSubsystem;

public class HeadHoncho extends StateMachine implements AutoCloseable {
    
    public enum HeadHonchoStates implements SystemState {
        REST {
            @Override
            public void initialize() {
                ShooterSubsystem.getInstance().stopShooter();
                ShooterSubsystem.getInstance().stopIndexer();
                ShooterSubsystem.getInstance().stopHood();
            }

            @Override
            public SystemState nextState() {
                return this;
            }
        },
        NORMAL {
            @Override
            public void initialize() {
                // set shooter motor to hold speed
                ShooterSubsystem.getInstance().holdShooter();
                // stop indexer motor
                ShooterSubsystem.getInstance().stopIndexer();
            }

            @Override
            public void execute() {

                // Shooter logic
                // Basically:
                // Always adjust hood
                // If holding shoot/pass button:
                    // Set shoot motor to shoot speed
                    // Toggle indexer motor based on if ready to shoot/pass
                // If not holding button, set shoot motor to hold speed
                // and stop indexer
                boolean shootB = s_headHoncho.m_shootButton.getAsBoolean();
                boolean passB = s_headHoncho.m_passButton.getAsBoolean();
                ShooterSubsystem.getInstance().adjustHood();
                if (shootB || passB) {
                    ShooterSubsystem.getInstance().shoot();
                    if ((ShooterSubsystem.getInstance().readyToShoot() &&
                         shootB) ||
                        (ShooterSubsystem.getInstance().readyToPass() &&
                         passB)) {
                        ShooterSubsystem.getInstance().index();
                    } else {
                        ShooterSubsystem.getInstance().stopIndexer();
                    }
                } else {
                    ShooterSubsystem.getInstance().holdShooter();
                    ShooterSubsystem.getInstance().stopIndexer();
                }
            }

            @Override
            public SystemState nextState() {
                return this;
            }
        },
        AUTO_INTAKE {
            @Override
            public SystemState nextState() {
                return this;
            }
        },
        CLIMB {
            @Override
            public SystemState nextState() {
                return this;
            }
        }
    }

    private static HeadHoncho s_headHoncho;
    private BooleanSupplier m_shootButton;
    private BooleanSupplier m_passButton;
    private BooleanSupplier m_intakeButton;

    public static HeadHoncho getInstance() {
        if (s_headHoncho == null) {
            s_headHoncho = new HeadHoncho();
        }
        return s_headHoncho;
    }

    private HeadHoncho() {
        super(HeadHonchoStates.REST);
    }

    public void configureBindings(
        BooleanSupplier shootButton,
        BooleanSupplier passButton,
        BooleanSupplier intakeButton
    ) {
        m_shootButton = shootButton;
        m_passButton = passButton;
        m_intakeButton = intakeButton;
    }

    // TODO impl
    public void close() {}
}
