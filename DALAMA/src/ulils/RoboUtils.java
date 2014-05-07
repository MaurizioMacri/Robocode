package ulils;

import robocode.AdvancedRobot;
import robocode.Rules;

public class RoboUtils {

	public static double bulletVelocity(double power) {
		// CHECK ROBOCODE PHISICS
		return (20.0 - (3.0 * power));
	}


	public static double getMaxEscapeAngle(double power){
		return Math.asin(Rules.MAX_VELOCITY/(bulletVelocity(power)));
	}

	public static double normaliseBearing(double ang) {
		if (ang > Math.PI)
			ang -= 2*Math.PI;
		if (ang < -Math.PI)
			ang += 2*Math.PI;
		return ang;

	}

	public static double getRange( double x1,double y1, double x2,double y2 )
	{
		/**returns the distance between two x,y coordinates*/
		double xo = x2-x1;
		double yo = y2-y1;
		double h = Math.sqrt( xo*xo + yo*yo );
		return h;	
	}
	public static double absbearing( double x1,double y1, double x2,double y2 )
	{
		/**gets the absolute bearing between to x,y coordinates*/
		double xo = x2-x1;
		double yo = y2-y1;
		double h = getRange( x1,y1, x2,y2 );
		if( xo > 0 && yo > 0 )
		{
			return Math.asin( xo / h );
		}
		if( xo > 0 && yo < 0 )
		{
			return Math.PI - Math.asin( xo / h );
		}
		if( xo < 0 && yo < 0 )
		{
			return Math.PI + Math.asin( -xo / h );
		}
		if( xo < 0 && yo > 0 )
		{
			return 2.0*Math.PI - Math.asin( -xo / h );
		}
		return 0;
	}
	
	public static void goTo(double x, double y,AdvancedRobot robot) {
		/**Move towards an x and y coordinate**/
		double dist = 20; 
		double angle = Math.toDegrees(RoboUtils.absbearing(robot.getX(),robot.getY(),x,y));
		double r = turnTo(angle,robot);
		robot.setAhead(dist * r);
	}
	
	
	/**Turns the shortest angle possible to come to a heading, then returns the direction the
	the bot needs to move in.**/
	public static int turnTo(double angle,AdvancedRobot robot) {
		double ang;
		int dir;
		ang = RoboUtils.normaliseBearing(robot.getHeading() - angle);
		if (ang > 90) {
			ang -= 180;
			dir = -1;
		}
		else if (ang < -90) {
			ang += 180;
			dir = -1;
		}
		else {
			dir = 1;
		}
		robot.setTurnLeft(ang);
		return dir;
	}
	
}
