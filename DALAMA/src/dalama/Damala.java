package dalama;

import java.awt.Color;
import java.util.Enumeration;
import java.util.Hashtable;

import robocode.AdvancedRobot;
import robocode.Rules;
import robocode.ScannedRobotEvent;
import robocode.util.Utils;




public class Damala extends AdvancedRobot {
	final double PI = Math.PI;		//just a constant

	private Enemy target;
	private int direction;


	private double firePower;

	double midpointstrength = 0;	//The strength of the gravity point in the middle of the field
	int midpointcount = 0;			//Number of turns since that strength was changed.



	private Hashtable<String, Enemy> targets;

	private void initializeStuff() {
		setAdjustGunForRobotTurn(true);
		setAdjustRadarForGunTurn(true);
		firePower=0.0;
		targets = new Hashtable<String, Enemy>();
		target = new Enemy();
		target.setDistance(Double.MAX_VALUE);
	}

	@Override
	public void run() {
		initializeStuff();
		decayAppereance();

		do {
			//printDebug();
			// ...
			// Turn the radar if we have no more turn, starts it if it stops and at the start of round
			doMove();
			doFirePower();
			doScanner(null);
			doGun();
			fire(firePower);
			execute();
		} while ( true );



	}


	private void printDebug(){
		if(targets !=null){
			Enumeration e = targets.elements();
			while (e.hasMoreElements()) {
				Enemy en = (Enemy)e.nextElement();
				out.println(en.getName());
			}
		}
	}






	private void decayAppereance() {
		setColors(Constants.bodyColor,Constants.gunColor,Constants.radarColor);

	}

	private void doGun() {
		// TODO Auto-generated method stub

	}




	private void doFirePower() {
		/**
		 * selects a bullet power based on our distance away from the target
		 * */
		firePower = 400/target.getDistance();
		if (firePower > 3) {
			firePower = 3;
		}

	}




	private void doMove() {
		double xforce = 0;
	    double yforce = 0;
	    double force;
	    double ang;
	    GravityPoint p;
		Enemy en;
    	Enumeration e = targets.elements();
	    //cycle through all the enemies.  If they are alive, they are repulsive.  Calculate the force on us
		while (e.hasMoreElements()) {
    	    en = (Enemy)e.nextElement();
			if (en.isLive()) {
				p = new GravityPoint(en.getX(),en.getY(), -1000);
		        force = p.getPower()/Math.pow(getRange(getX(),getY(),p.getX(),p.getY()),Constants.forcePower);
		        //Find the bearing from the point to us
		        ang = normaliseBearing(Math.PI/2 - Math.atan2(getY() - p.getY(), getX() - p.getX())); 
		        //Add the components of this force to the total force in their respective directions
		        xforce += Math.sin(ang) * force;
		        yforce += Math.cos(ang) * force;
			}
	    }
	    
		/**The next section adds a middle point with a random (positive or negative) strength.
		The strength changes every 5 turns, and goes between -1000 and 1000.  This gives a better
		overall movement.**/
		midpointcount++;
		if (midpointcount > 5) {
			midpointcount = 0;
			midpointstrength = (Math.random() * 2000) - 1000;
		}
		p = new GravityPoint(getBattleFieldWidth()/2, getBattleFieldHeight()/2, midpointstrength);
		force = p.getPower()/Math.pow(getRange(getX(),getY(),p.getX(),p.getY()),1.5);
	    ang = normaliseBearing(Math.PI/2 - Math.atan2(getY() - p.getY(), getX() - p.getX())); 
	    xforce += Math.sin(ang) * force;
	    yforce += Math.cos(ang) * force;
	   
	    /**The following four lines add wall avoidance.  They will only affect us if the bot is close 
	    to the walls due to the force from the walls decreasing at a power 3.**/
	    xforce += 5000/Math.pow(getRange(getX(), getY(), getBattleFieldWidth(), getY()), 3);
	    xforce -= 5000/Math.pow(getRange(getX(), getY(), 0, getY()), 3);
	    yforce += 5000/Math.pow(getRange(getX(), getY(), getX(), getBattleFieldHeight()), 3);
	    yforce -= 5000/Math.pow(getRange(getX(), getY(), getX(), 0), 3);
	    
	    //Move in the direction of our resolved force.
	    goTo(getX()-xforce,getY()-yforce);

	}

	
	//gets the absolute bearing between to x,y coordinates
	public double absbearing( double x1,double y1, double x2,double y2 )
	{
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
	/**Move towards an x and y coordinate**/
	void goTo(double x, double y) {
	    double dist = 20; 
	    double angle = Math.toDegrees(absbearing(getX(),getY(),x,y));
	    double r = turnTo(angle);
	    setAhead(dist * r);
	}

	
	//returns the distance between two x,y coordinates
	public double getRange( double x1,double y1, double x2,double y2 )
	{
		double xo = x2-x1;
		double yo = y2-y1;
		double h = Math.sqrt( xo*xo + yo*yo );
		return h;	
	}

	
	/**Turns the shortest angle possible to come to a heading, then returns the direction the
	the bot needs to move in.**/
	int turnTo(double angle) {
	    double ang;
    	int dir;
	    ang = normaliseBearing(getHeading() - angle);
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
	    setTurnLeft(ang);
	    return dir;
	}

	@Override
	public void onScannedRobot(ScannedRobotEvent e) {
		doScanner(e);
		robotScanned(e);

	}


	private void robotScanned(ScannedRobotEvent e) {
		Enemy en;
		if (targets.containsKey(e.getName())) {
			en = (Enemy)targets.get(e.getName());
		} else {
			en = new Enemy();
			targets.put(e.getName(),en);
		}

		//the next line gets the absolute bearing to the point where the bot is
		double absbearing_rad = (getHeadingRadians()+e.getBearingRadians())%(2*PI);
		//this section sets all the information about our target
		en.setName(e.getName());
		double h = normaliseBearing(e.getHeadingRadians() - en.getHeading());
		h = h/(getTime() - en.getCtime());
		en.setChangehead(h);
		en.setX(getX()+Math.sin(absbearing_rad)*e.getDistance()); //works out the x coordinate of where the target is
		en.setY(getY()+Math.cos(absbearing_rad)*e.getDistance()); //works out the y coordinate of where the target is
		en.setBearing(e.getBearingRadians());
		en.setHeading(e.getHeadingRadians());
		en.setCtime(getTime());				//game time at which this scan was produced
		en.setSpeed(e.getVelocity());
		en.setDistance( e.getDistance());	
		en.setLive(true);
		if ((en.getDistance()< target.getDistance()) || (!target.isLive())) {
			target = en;
		}


	}

	private void doScanner(ScannedRobotEvent e){
		/**
		 * If i got the target i keep the eyes on 
		 * it by scanning by WIDTH on both right and left side.
		 * It's a perfect lock scan.
		 * if e is NULL -> just a 2PI rotation scan is performed.
		 * This metod should be called in 
		 * 		run() -> doScanner(null)
		 *  	onScannedRobot() -> doScanner(e) e not NULL 
		 */
		if ( getRadarTurnRemaining() == 0.0 && e==null){
			setTurnRadarRightRadians( Double.POSITIVE_INFINITY );
		}
		else if(e!=null){
			double angleToEnemy = getHeadingRadians() + e.getBearingRadians();
			// Subtract current radar heading to get the turn required to face the enemy, be sure it is normalized
			double radarTurn = Utils.normalRelativeAngle( angleToEnemy - getRadarHeadingRadians() );
			// Distance we want to scan from middle of enemy to either side
			// The 36.0 is how many units from the center of the enemy robot it scans.
			double extraTurn = Math.min( Math.atan( 30.0 / e.getDistance() ), Rules.RADAR_TURN_RATE_RADIANS );
			// Adjust the radar turn so it goes that much further in the direction it is going to turn
			// Basically if we were going to turn it left, turn it even more left, if right, turn more right.
			// This allows us to overshoot our enemy so that we get a good sweep that will not slip.
			radarTurn += (radarTurn < 0 ? -extraTurn : extraTurn);
			double gunTurn=0;
			//Turn the radar
			double absoluteBearing = getHeadingRadians() + e.getBearingRadians();
			setTurnGunRightRadians(
					robocode.util.Utils.normalRelativeAngle(absoluteBearing - 
							getGunHeadingRadians()));
			gunTurn= (getHeading()-getGunHeading()+e.getBearing())%360;

			setTurnRadarRightRadians(radarTurn);
		}
	}

	//if a bearing is not within the -pi to pi range, alters it to provide the shortest angle
	double normaliseBearing(double ang) {
		if (ang > PI)
			ang -= 2*PI;
		if (ang < -PI)
			ang += 2*PI;
		return ang;
		
	}


}
