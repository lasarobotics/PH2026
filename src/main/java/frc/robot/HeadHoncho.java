package frc.robot;

import java.util.function.BooleanSupplier;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;

import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem.HopperSubsystemStates;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class HeadHoncho extends StateMachine implements AutoCloseable {
    
    public enum HeadHonchoStates implements SystemState {
        REST {
            @Override
            public void initialize() {
                ShooterSubsystem.getInstance().stopEverything();
                HopperSubsystem.setState(HopperSubsystemStates.REST);
            }

            @Override
            public SystemState nextState() {
                return this;
            }
        },
        NORMAL {
            @Override
            public void execute() {
                boolean shootButton = s_headHoncho.m_shootButton.getAsBoolean();
                boolean passButton = s_headHoncho.m_passButton.getAsBoolean();

                ShooterSubsystem.getInstance().shooterPeriodic(
                    shootButton, passButton
                );

                if (shootButton || passButton) {
                    HopperSubsystem.setState(HopperSubsystemStates.LOADING);
                } else {
                    HopperSubsystem.setState(HopperSubsystemStates.REST);
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
