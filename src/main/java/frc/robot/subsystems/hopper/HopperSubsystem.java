package frc.robot.subsystems.hopper;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;

public class HopperSubsystem extends StateMachine implements AutoCloseable {
    
    enum HopperSubsystemStates implements SystemState {
        REST {
            @Override
            public SystemState nextState() {
                // TODO Auto-generated method stub
                return null;
            }
        },
        FEEDING {
            @Override
            public SystemState nextState() {
                // TODO Auto-generated method stub
                return null;
            }
        }
    }

    private static HopperSubsystem s_hopperSubsystem;

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
