package frc.auto;

// import com.team1323.frc2020.auto.modes.OppoTrenchAndThreeMode;
// import com.team1323.frc2020.auto.modes.StandStillMode;
// import com.team1323.frc2020.auto.modes.TrenchRunMode;
// import com.team1323.frc2020.auto.modes.TrenchRunPoachedMode;
// import com.team1323.frc2020.auto.modes.TwoAndTrenchRunMode;
// import com.team1323.frc2020.auto.modes.TwoAndTrenchRunPoachedMode;
import frc.auto.modes.ThiefAndRendezvous;
import frc.auto.modes.RendezvousAndTrenchMode;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardInteractions {
    private static final String SELECTED_AUTO_MODE = "selected_auto_mode";
    
    private static final AutoOption DEFAULT_MODE = AutoOption.THIEF_AND_RENDEZVOUS;

    private SendableChooser<AutoOption> modeChooser;

    
    public void initWithDefaults(){
    	modeChooser = new SendableChooser<AutoOption>();
        modeChooser.setDefaultOption(DEFAULT_MODE.name, DEFAULT_MODE);
        //modeChooser.addOption(AutoOption.TRENCH_RUN_MODE.name, AutoOption.TRENCH_RUN_MODE);
        //modeChooser.addOption(AutoOption.TRENCH_RUN_POACHED_MODE.name, AutoOption.TRENCH_RUN_POACHED_MODE);
        // modeChooser.addOption(AutoOption.TWO_AND_TRENCH_RUN.name, AutoOption.TWO_AND_TRENCH_RUN);
        // modeChooser.addOption(AutoOption.TWO_AND_TRENCH_POACHED_MODE.name, AutoOption.TWO_AND_TRENCH_POACHED_MODE);
        // modeChooser.addOption(AutoOption.OPPO_TRENCH_AND_THREE_MODE.name, AutoOption.OPPO_TRENCH_AND_THREE_MODE);
        modeChooser.addOption(AutoOption.THIEF_AND_RENDEZVOUS.name, AutoOption.THIEF_AND_RENDEZVOUS);
        modeChooser.addOption(AutoOption.RENDEZVOUS_AND_TRENCH.name, AutoOption.RENDEZVOUS_AND_TRENCH);

        SmartDashboard.putData("Mode Chooser", modeChooser);
    	SmartDashboard.putString(SELECTED_AUTO_MODE, DEFAULT_MODE.name);
    }
    
    public AutoModeBase getSelectedAutoMode(){
        AutoOption selectedOption = (AutoOption) modeChooser.getSelected();                
        return createAutoMode(selectedOption);
    }
    
    public String getSelectedMode(){
    	AutoOption option = (AutoOption) modeChooser.getSelected();
    	return option.name;
    }

    enum AutoOption{
        // STAND_STILL("Stand Still"),
        // TRENCH_RUN_MODE("Trench Run"),
        // TWO_AND_TRENCH_RUN("Two Balls + Trench Run"),
        // TRENCH_RUN_POACHED_MODE("Trench Poached Run"),
        // TWO_AND_TRENCH_POACHED_MODE("Two Balls + Trench Poached Run"),
        // OPPO_TRENCH_AND_THREE_MODE("Oppo Trench Run + Three Balls");
        THIEF_AND_RENDEZVOUS("Opponent Trench and 5 RZone Balls"),
    	RENDEZVOUS_AND_TRENCH("Trench Three and 5 Rzone Balls");
    	public final String name;
    	
    	AutoOption(String name){
    		this.name = name;
    	}
    }

    private AutoModeBase createAutoMode(AutoOption option){
    	switch(option){
            // case STAND_STILL:
            //     return new StandStillMode();
            // case TRENCH_RUN_MODE:
            //     return new TrenchRunMode();
            // case TWO_AND_TRENCH_RUN:
            //     return new TwoAndTrenchRunMode();
            // case TRENCH_RUN_POACHED_MODE:
            //     return new TrenchRunPoachedMode();
            // case TWO_AND_TRENCH_POACHED_MODE:
            //     return new TwoAndTrenchRunPoachedMode();
            // case OPPO_TRENCH_AND_THREE_MODE:
            //     return new OppoTrenchAndThreeMode();\
            case THIEF_AND_RENDEZVOUS:
                return new ThiefAndRendezvous();
            case RENDEZVOUS_AND_TRENCH:
                return new RendezvousAndTrenchMode();
            default:
                System.out.println("ERROR: unexpected auto mode: " + option);
                return new ThiefAndRendezvous();
    	}
    }
    
    public void output(){
    	SmartDashboard.putString(SELECTED_AUTO_MODE, getSelectedMode());
    }
}
