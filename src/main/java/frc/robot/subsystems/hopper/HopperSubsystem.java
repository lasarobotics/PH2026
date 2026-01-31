package frc.robot.subsystems.hopper;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;

public class HopperSubsystem extends StateMachine implements AutoCloseable {
    
    public enum HopperSubsystemStates implements SystemState {
        REST {
            @Override
            public SystemState nextState() {
                // TODO Auto-generated method stub
                return nextState;
            }
        },
        LOADING {
            @Override
            public SystemState nextState() {
                // TODO Auto-generated method stub
                return nextState;
            }
        }
    }

    public static void setState(HopperSubsystemStates state) {
        nextState = state;
    }

    private static HopperSubsystem s_hopperSubsystem;
    private static HopperSubsystemStates nextState;

    public static HopperSubsystem getInstance() {
        if (s_hopperSubsystem == null) {
            s_hopperSubsystem = new HopperSubsystem();
        }
        return s_hopperSubsystem;
    }

    private HopperSubsystem() {
        super(HopperSubsystemStates.REST);
    }

    // TODO
    public void close() {}
}
