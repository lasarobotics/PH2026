package frc.robot;

import frc.robot.fsm.StateMachine;
import frc.robot.fsm.SystemState;

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