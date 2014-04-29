package daalma;

import robocode.AdvancedRobot;

public class DummyRobot extends AdvancedRobot{
	
	
	@Override
	public void run() {
		while(true){
			turnLeft(360);
		}
	}
	

}
