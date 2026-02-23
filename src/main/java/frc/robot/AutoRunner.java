package frc.robot;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;

public class AutoRunner extends StateMachine implements AutoCloseable {
  public AutoRunner(
    SystemState startingState,
    AutoPositionConfig positions
  ) {
    super(startingState);
    AutoHoncho.positionConfig = positions;
  }

  @Override
  public void close() {}
}