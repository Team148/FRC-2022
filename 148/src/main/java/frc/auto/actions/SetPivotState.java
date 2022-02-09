package frc.auto.actions;

import frc.Robot;
import frc.subsystems.IntakePivot.PivotState;
public class SetPivotState extends RunOnceAction{
    PivotState desiredState;

    public SetPivotState(PivotState state){
        desiredState = state;
    }

    @Override
    public void runOnce() {
        Robot.getIntakePivot().setState(desiredState);
    }
}
