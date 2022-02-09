package frc.auto;

import frc.auto.modes.TwoBallAndTerminal;
import frc.auto.modes.OneBallAndDefend;
import frc.auto.modes.OneBallAndTerminal;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardInteractions {
    private static final String SELECTED_AUTO_MODE = "selected_auto_mode";
    
    private static final AutoOption DEFAULT_MODE = AutoOption.TWO_BALL_AND_TERMINAL;

    private SendableChooser<AutoOption> modeChooser;

    
    public void initWithDefaults(){
    	modeChooser = new SendableChooser<AutoOption>();
        modeChooser.setDefaultOption(DEFAULT_MODE.name, DEFAULT_MODE);
        modeChooser.addOption(AutoOption.TWO_BALL_AND_TERMINAL.name, AutoOption.TWO_BALL_AND_TERMINAL);
        modeChooser.addOption(AutoOption.ONE_BALL_AND_DEFEND.name, AutoOption.ONE_BALL_AND_DEFEND);
        modeChooser.addOption(AutoOption.ONE_BALL_AND_TERMINAL.name, AutoOption.ONE_BALL_AND_TERMINAL);

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
        TWO_BALL_AND_TERMINAL("Two Balls and Terminal Shot"),
        ONE_BALL_AND_DEFEND("One ball and defend"),
        ONE_BALL_AND_TERMINAL("One ball and Terminal Shot");
    	public final String name;
    	
    	AutoOption(String name){
    		this.name = name;
    	}
    }

    private AutoModeBase createAutoMode(AutoOption option){
    	switch(option){
            case ONE_BALL_AND_TERMINAL:
                return new OneBallAndTerminal();
            case ONE_BALL_AND_DEFEND:
                return new OneBallAndDefend();
            case TWO_BALL_AND_TERMINAL:
                return new TwoBallAndTerminal();
            default:
                System.out.println("ERROR: unexpected auto mode: " + option);
                return new TwoBallAndTerminal();
    	}
    }
    
    public void output(){
    	SmartDashboard.putString(SELECTED_AUTO_MODE, getSelectedMode());
    }
}
