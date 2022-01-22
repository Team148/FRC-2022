package frc.auto.actions;

import frc.subsystems.Superstructure;

public class SuperStructureAction extends RunOnceAction{
    Superstructure s;
	
	public SuperStructureAction(){
		s = Superstructure.getInstance();
	}
	
	@Override
	public void runOnce() {
	s.firingVision();
	}
}
