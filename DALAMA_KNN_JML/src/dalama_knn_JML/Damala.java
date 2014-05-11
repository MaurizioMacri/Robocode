package dalama_knn_JML;

import java.awt.Color;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.Enumeration;
import java.util.Hashtable;
import java.util.List;
import java.util.Random;
import java.util.Vector;

import net.sf.javaml.classification.Classifier;
import net.sf.javaml.classification.KNearestNeighbors;
import net.sf.javaml.core.Dataset;
import net.sf.javaml.core.DefaultDataset;
import net.sf.javaml.core.DenseInstance;
import net.sf.javaml.core.Instance;
import robocode.AdvancedRobot;
import robocode.HitByBulletEvent;
import robocode.Rules;
import robocode.ScannedRobotEvent;
import robocode.util.Utils;

 


public class Damala extends AdvancedRobot {

	/*WavSurfer variables*/

	private static int STATS_DIM = 47;
	private static double WALL_STICK = 120;
	private static Rectangle2D.Double fieldRect = new java.awt.geom.Rectangle2D.Double(18, 18, 764, 564);

	private Point2D.Double enemyPos;
	private Point2D.Double myPos;
	private static ArrayList<Wave> waves_mov;
	private double enemyEnergy;
	private int dir;
	private double orbitAngle;
	private static double STAT[];

	/*END WAVESURFER VARIABLES*/



	final double PI = Math.PI;		//just a constant


	double absBearing;
	private Enemy target;
	private int direction;


	private double firePower;

	private Dataset situationsDataset= new DefaultDataset();
	
//	private static KdTree<Integer> situationsKDTree = new KdTree<>(3);

	double midpointstrength = 0;	//The strength of the gravity point in the middle of the field
	int midpointcount = 0;			//Number of turns since that strength was changed.



	private Hashtable<String, Enemy> targets;

	private static List<WaveBullet> waves= new Vector<>();
	private static List<EnemySituation> situations= new Vector<>();


	private void initializeStuff() {
		setAdjustGunForRobotTurn(true);
		setAdjustRadarForGunTurn(true);
		firePower=0.0;
		targets = new Hashtable<String, Enemy>();
		target = new Enemy();
		target.setDistance(Double.MAX_VALUE);

		//initialize waveSurfer
		enemyEnergy = 0;
		orbitAngle = 0;
		enemyPos = new Point2D.Double();
		waves_mov = new ArrayList<Wave>();
		dir = 1;
		STAT = new double[STATS_DIM];

		
		
	}

	private int fromGuessFactorToIndex(double guessFactor, int slices){
		//range GF [0,2]
		
		return (int)((guessFactor)*slices)/(2);
	}
	
	private double fromIndexToGuessFactor(int index,int slices){
		double sliceSize=2.0/(double)slices;
		double sliceStart=index*sliceSize;
		return (sliceStart+sliceSize/2-1);//center of class
		
	}
	
	
	
	@Override
	public void run() {
		enemyEnergy = getEnergy();//surfer

		initializeStuff();
		decayAppereance();

		do {
			//printDebug();
			// ...
			// Turn the radar if we have no more turn, starts it if it stops and at the start of round
			updateWavesSurfer();
			surfNearestWave();

			//doMove();
			doFirePower();
			doScanner(null);
			doGun();
			//fire(firePower);
			execute();
		} while ( true );



	}

	
	@Override
	public void onHitByBullet(HitByBulletEvent event) {

		// find the wave associated to the bullet
		if (!waves_mov.isEmpty()) {
			Wave hitWave = null;
			for (Wave wave : waves_mov) {
				if (Math.abs(wave.getDistanceTraveled() - myPos.distance(wave.getFireLocation())) > 50 && Math.abs(wave.getBulletVelocity() - bulletVelocity(event.getBullet().getPower())) < 0.001) {
					hitWave = wave;
					break;
				}
			}

			// Update STAT
			if (hitWave != null)
				updateStats(hitWave, myPos);

		}

	}
	
	// update STAT array when the robot is hitted by a bullet
	public void updateStats(Wave wave, Point2D.Double pos) {

		int index = getStatsIndex(wave, pos);

		for (int i = 0; i < STATS_DIM; i++) {
			STAT[i] += 1.0 / (Math.pow(index - i, 2) + 1);
		}

	}
	
	public static double bulletVelocity(double power) {
		// CHECK ROBOCODE PHISICS
		return (20.0 - (3.0 * power));
	}
	public static Point2D.Double project(Point2D.Double sourceLocation, double angle, double length) {
		angle = Math.toRadians(angle);
		Point2D.Double endPoint = new Point2D.Double();
		endPoint.x = sourceLocation.x + Math.sin(angle) * length;
		endPoint.y = sourceLocation.y + Math.cos(angle) * length;
		return endPoint;
	}
	
	public void surfNearestWave() {

		// drawing debug stuff
		Point2D.Double pointForward = new Point2D.Double(1, 1);
		Point2D.Double pointBackward = new Point2D.Double(1, 1);
		getGraphics().fillRect((int) pointBackward.x, (int) pointBackward.y, 10, 10);
		getGraphics().fillRect((int) pointForward.x, (int) pointForward.y, 10, 10);

		// if there is no wave the robot stay. What should we do?
		if (waves_mov.size() == 0)
			return;

		// find the nearest wave
		Wave nearestWave = waves_mov.get(0);
		double absMinDistance = Math.abs(myPos.distance(waves_mov.get(0).getFireLocation()) - waves_mov.get(0).getDistanceTraveled());
		// it make one unusefull iteration
		for (Wave wave : waves_mov) {
			double absDistance = Math.abs(myPos.distance(wave.getFireLocation()) - wave.getDistanceTraveled());
			if (absDistance < absMinDistance) {
				nearestWave = wave;
				absMinDistance = absDistance;
			}
		}

		// compute the orbit angle to surf the wave

		// angle between the wave origin and my current position
		double currentBearingRespectToWave = Math.atan2(myPos.x - nearestWave.getFireLocation().x, myPos.y - nearestWave.getFireLocation().y);

		orbitAngle = getHeading() - Math.toDegrees(currentBearingRespectToWave) - 90;

		orbitAngle = Utils.normalRelativeAngleDegrees(wallSmoothing(myPos, orbitAngle, dir));

		// predict the robot postition at the time the wave will hit him
		// THIS NEED TESTING
		double tickUntilHit = absMinDistance / nearestWave.getBulletVelocity();
		double maxCoverDistance = tickUntilHit * Rules.MAX_VELOCITY;

		pointBackward = project(myPos, getHeading(), maxCoverDistance);
		pointForward = project(myPos, getHeading(), -maxCoverDistance);

		// move to the less dangerous position according to the STAT array
		if (STAT[getStatsIndex(nearestWave, pointBackward)] > STAT[getStatsIndex(nearestWave, pointForward)]) {
			dir = -1;
		} else if (STAT[getStatsIndex(nearestWave, pointBackward)] < STAT[getStatsIndex(nearestWave, pointForward)]) {
			dir = 1;
		}

		orbitAngle = Utils.normalRelativeAngleDegrees(wallSmoothing(myPos, orbitAngle, dir));

		setTurnLeft(orbitAngle);
		setAhead(100 * dir);

		// drawing debug stuff

		getGraphics().setColor(new Color(0.0f, 0.0f, 1.0f, 0.5f));

		// drawing maximum escape angle
		double maxEscapeAngle = Math.asin(Rules.MAX_VELOCITY / nearestWave.getBulletVelocity());
		maxEscapeAngle = Math.toDegrees(maxEscapeAngle);
		Point2D.Double tmp = project(nearestWave.getMyPosAtFireTime(), -maxEscapeAngle / 2 + nearestWave.getAbsWaveBearing(), nearestWave.getMyPosAtFireTime().distance(nearestWave.getFireLocation()));
		Point2D.Double tmp1 = project(nearestWave.getMyPosAtFireTime(), maxEscapeAngle / 2 + nearestWave.getAbsWaveBearing(), nearestWave.getMyPosAtFireTime().distance(nearestWave.getFireLocation()));
		int px1[] = { (int) nearestWave.getFireLocation().x, (int) tmp1.x, (int) tmp.x };
		int py1[] = { (int) nearestWave.getFireLocation().y, (int) tmp1.y, (int) tmp.y };
		getGraphics().fillPolygon(px1, py1, 3);

		getGraphics().setColor(new Color(0.0f, 1.0f, 0.0f, 0.5f));

		Point2D.Double stat = new Point2D.Double();

		for (int i = 0; i < STATS_DIM; i++) {
			getGraphics().setColor(new Color((float) (1 / (STAT[i] + 1)), 0, 1 - (float) (1 / (STAT[i] + 1)), 0.5f));
			stat = project(nearestWave.getFireLocation(), -maxEscapeAngle / 2 + nearestWave.getAbsWaveBearing() + i * (maxEscapeAngle / STATS_DIM), nearestWave.getDistanceTraveled());
			getGraphics().fillRect((int) stat.x, (int) stat.y, 4, 4);
		}

	}

	// update waves position and delete unusefull waves
	public void updateWavesSurfer() {
		myPos = new Point2D.Double(getX(), getY());
		for (int i = 0; i < waves_mov.size(); i++) {
			Wave wave = waves_mov.get(i);
			wave.setDistanceTraveled((getTime() - wave.getFireTime()) * wave.getBulletVelocity());
			if (wave.getDistanceTraveled() > myPos.distance(wave.getFireLocation()) + 50) {
				waves_mov.remove(i);
				i--;
			}
		}
	}


	@Override
	public void onScannedRobot(ScannedRobotEvent e) {
doScannedRobotSurfer(e);

		
		doScanner(e);
		robotScanned(e);



		//this method process already flying waves
		processWaves(e);
		collectData(e);

	}


	private void doScannedRobotSurfer(ScannedRobotEvent e) {
		// absolute Bearing is needed to compute enemy position
		double absoluteBearing = getHeading() + e.getBearing();

		double bulletPower = enemyEnergy - e.getEnergy();

		if (bulletPower >= Rules.MIN_BULLET_POWER && bulletPower <= Rules.MAX_BULLET_POWER) {
			Wave wave = new Wave();
			wave.setBulletVelocity(WaveBullet.getBulletSpeed(bulletPower));
			wave.setDistanceTraveled(WaveBullet.getBulletSpeed(bulletPower));
			// -1 because the bullet was fired in the previous tick
			wave.setFireTime(getTime() - 1);
			// enemyPos refers the enemy position in the previous tick when the
			// bullet was fired
			wave.setFireLocation((Point2D.Double) enemyPos.clone());

			// needed to compute the guessing factor during the surfing
			// i don't know why + 180
			wave.setAbsWaveBearing(Utils.normalRelativeAngleDegrees(absoluteBearing + 180));
			// needed to compute the guessing factor during the surfing
			wave.setMyDirAtFireTime((int) Math.signum(getVelocity() * Math.sin(e.getBearingRadians())));

			wave.setMyPosAtFireTime(myPos);

			waves_mov.add(wave);
		}
		enemyEnergy = e.getEnergy();
		enemyPos = project(myPos, absoluteBearing, e.getDistance());
		
	}

	//only when situazions.size() > 0
	private void collectData(ScannedRobotEvent e) {
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


	}




	private void doFirePower() {
		/**
		 * selects a bullet power based on our distance away from the target
		 * */
		firePower = 600/target.getDistance();
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
		force = p.getPower()/Math.pow(getRange(getX(),getY(),p.getX(),p.getY()),4.5);
		ang = normaliseBearing(Math.PI/2 - Math.atan2(getY() - p.getY(), getX() - p.getX())); 
		xforce += Math.sin(ang) * force*2;
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




	private void processWaves(ScannedRobotEvent e) {
		double absBearing = getHeadingRadians() + e.getBearingRadians();

		// find our enemy's location:
		double ex = getX() + Math.sin(absBearing) * e.getDistance();
		double ey = getY() + Math.cos(absBearing) * e.getDistance();

		// Let's process the waves now:
		for (int i=0; i < waves.size(); i++)
		{
			WaveBullet currentWave = (WaveBullet)waves.get(i);
			if (currentWave.checkHit(ex, ey, getTime()))
			{
				currentWave.getRelatedSituation().setRightGessFsctor(currentWave.getRightGuessFactor());
				Instance tmpInstance = new DenseInstance(currentWave.getRelatedSituation().toKD_Key(),fromGuessFactorToIndex(currentWave.getRightGuessFactor()+1,Constants.numSliceGF));
				situationsDataset.add(tmpInstance);
				waves.remove(currentWave);
				i--;
			}
		}

//		out.println("size tree "+situationsKDTree.size());
		EnemySituation newSituation = new EnemySituation();
		newSituation.setDistance(e.getDistance());
		newSituation.setBulletFlightTime(e.getDistance()/WaveBullet.getBulletSpeed(firePower));
		newSituation.setTime(getTime());
		newSituation.setVelocity(e.getVelocity());
		newSituation.setAdvancingVelocity(e.getVelocity() * -1 * Math.cos(e.getHeadingRadians() - (e.getBearingRadians() + getHeadingRadians())));
		newSituation.setLateralVelocity(e.getVelocity() * Math.sin(e.getHeadingRadians() - (e.getBearingRadians() + getHeadingRadians())));
		newSituation.setPower(firePower);
		if(situations.size()>0){	
			double oldLatVel = situations.get(situations.size()-1).getLateralVelocity();
			double oldAdvVel = situations.get(situations.size()-1).getAdvancingVelocity();
			double deltaTime = getTime()-situations.get(situations.size()-1).getTime();//currTime - old time
			newSituation.setLateralAcceleration(Enemy.returnAcceleration(oldLatVel, newSituation.getLateralVelocity(), deltaTime)); 
			newSituation.setAdvancingAcceleration(Enemy.returnAcceleration(oldAdvVel, newSituation.getAdvancingVelocity(), deltaTime));
		}

		situations.add(newSituation);


		newSituation.printEnemySituation();
		out.println("no situations "+situations.size());

		if (e.getVelocity() != 0)
		{
			if (Math.sin(e.getHeadingRadians()-absBearing)*e.getVelocity() < 0)
				direction = -1;
			else
				direction = 1;
		}

		WaveBullet newWave = new WaveBullet(getX(), getY(), absBearing, firePower,
				direction, getTime(),newSituation);



		double dist = Double.MAX_VALUE;

		EnemySituation best=newSituation;
		double guessfactor = best.getRightGessFsctor();
		if(situationsDataset.size() > Constants.KNNThreshold ) {
//			MaxHeap<Integer> maxHeap = situationsKDTree.findNearestNeighbors(newSituation.toKD_Key(), 3,new SquareEuclideanDistanceFunction());
//			MaxHeap<Integer> maxHeap = situationsKDTree.findNearestNeighbors(newSituation.toKD_Key(), 3,new WeightedSquareEuclideanDistance(Constants.dimensions, Constants.weights));
			Classifier knn = new KNearestNeighbors(3);
//			System.out.println("AAAAAAAAAAAAAAAAAAAA");
			knn.buildClassifier(situationsDataset);
//			System.out.println(knn.classDistribution(new DenseInstance(newSituation.toKD_Key())));
			guessfactor=fromIndexToGuessFactor((Integer )knn.classify(new DenseInstance(newSituation.toKD_Key())),Constants.numSliceGF);
//		out.println("MAX HEAP ->"+maxHeap.getMax());
		}
		// this should do the opposite of the math in the WaveBullet:

		double angleOffset = direction * guessfactor * (best.getMaxEscapeAngle());
		double gunAdjust = Utils.normalRelativeAngle(
				absBearing- getGunHeadingRadians() + angleOffset);
		setTurnGunRightRadians(gunAdjust);
		out.println(""+angleOffset+","+direction+","+guessfactor+","+best.getMaxEscapeAngle());

		waves.add(newWave);
		if (getGunHeat() == 0 && gunAdjust < Math.atan2(9, e.getDistance()) )
			setFireBullet(firePower) ;

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

			//			double gunTurn=0;
			//			//Turn the radar
			//			double absoluteBearing = getHeadingRadians() + e.getBearingRadians();
			//			setTurnGunRightRadians(
			//					robocode.util.Utils.normalRelativeAngle(absoluteBearing - 
			//							getGunHeadingRadians()));
			//			gunTurn= (getHeading()-getGunHeading()+e.getBearing())%360;

			setTurnRadarRightRadians(radarTurn);
			absBearing = getHeadingRadians() + e.getBearingRadians();
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
	
	// compute the guess factor and convert it to an index on the STAT array
		public int getStatsIndex(Wave wave, Point2D.Double pos) {

			// Maximum escape angle computation
			double maxEscapeAngle = Math.asin(Rules.MAX_VELOCITY / wave.getBulletVelocity());
			// angle between the wave origin and pos
			double currentBearingRespectToWave = Math.atan2(pos.x - wave.getFireLocation().x, pos.y - wave.getFireLocation().y);

			double offset = currentBearingRespectToWave - Math.toRadians(wave.getAbsWaveBearing());

			// guess factor represent the possible gun bearing represented as a
			// fraction of the maximum escape angle
			double guessFactor = wave.getMyDirAtFireTime() * Utils.normalRelativeAngle(offset) / maxEscapeAngle;

			if (guessFactor > 1)
				guessFactor = 1;
			else if (guessFactor < -1)
				guessFactor = -1;

			guessFactor++;

			return (int) (STATS_DIM / 2 * guessFactor);
		}
		
		public double wallSmoothing(Point2D.Double botLocation, double angle, int orientation) {
			int fix = 0;
			getGraphics().fillRect((int) project(botLocation, getHeading() + fix, -WALL_STICK).x, (int) project(botLocation, getHeading() + fix, -WALL_STICK).y, 10, 10);
			if (!fieldRect.contains(project(botLocation, getHeading() + fix, WALL_STICK * dir)))
				dir *= -1;
			// out.println("ange "+angle+" orienation "+orientation);
			// while (!fieldRect.contains(project(botLocation, getHeading() + fix,
			// -WALL_STICK))) {
			// fix -= orientation * 1;
			// }

			return Utils.normalAbsoluteAngleDegrees(angle + fix);
		}


}
