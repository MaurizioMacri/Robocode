package dalama;

import java.awt.Color;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.Hashtable;
import java.util.List;
import java.util.Vector;

import kdtree.KdTree;
import kdtree.MaxHeap;
import robocode.AdvancedRobot;
import robocode.BulletHitEvent;
import robocode.HitByBulletEvent;
import robocode.Rules;
import robocode.ScannedRobotEvent;
import robocode.util.Utils;
import surfing.MovementsManager;
import ulils.RoboUtils;
import ulils.RobotSituation;
import ulils.Wave;

public class Damala extends AdvancedRobot {

	/* WavSurfer variables */

	private static int STATS_DIM = 47;
	private static double WALL_STICK = 120;
	private static Rectangle2D.Double fieldRect = new java.awt.geom.Rectangle2D.Double(18, 18, 764, 564);

	private Point2D.Double enemyPos;
	private static ArrayList<Wave> waves_mov;
	private double enemyEnergy;
	private int dir;
	private double orbitAngle;
	private static double STAT[];

	ArrayList<RobotSituation> situations = new ArrayList<>();
	ArrayList<Integer> relatedClasses = new ArrayList<>();
	/* END WAVESURFER VARIABLES */

	final double PI = Math.PI; // just a constant

	double absBearing;
	private int direction;

	private double firePower;

	private static KdTree<Integer> situationsKDTree = new KdTree<>(Constants.dimensions);

	double midpointstrength = 0; // The strength of the gravity point in the
	// middle of the field
	int midpointcount = 0; // Number of turns since that strength was changed.


	private static List<Wave> waves = new Vector<>();



	private static MovementsManager movManager;
	private RobotSituation mySituation;


	private void initializeStuff() {
		movManager = new MovementsManager();


		setAdjustGunForRobotTurn(true);
		setAdjustRadarForGunTurn(true);
		firePower = 0.0;

		// initialize waveSurfer
		enemyEnergy = 0;
		orbitAngle = 0;
		enemyPos = new Point2D.Double();
		waves_mov = new ArrayList<Wave>();
		dir = 1;
		STAT = new double[STATS_DIM];

	}

	private int fromGuessFactorToIndex(double guessFactor, int slices) {
		// range GF [0,2]
		return (int) ((guessFactor) * slices) / (2);
	}

	private double fromIndexToGuessFactor(int index, int slices) {
		double sliceSize = 2.0 / (double) slices;
		double sliceStart = index * sliceSize;
		return (sliceStart + sliceSize / 2 - 1);// center of class

	}

	@Override
	public void run() {
		enemyEnergy = getEnergy();// surfer

		initializeStuff();
		decayAppereance();

		do {

			// Turn the radar if we have no more turn, starts it if it stops and
			// at the start of round
			updateWavesSurfer();
			surfNearestWave();

			// doMove();
			doFirePower();
			doScanner(null);
			doGun();
			// fire(firePower);
			execute();
				
		} while (true);

	}

	@Override
	public void onHitByBullet(HitByBulletEvent event) {
		Point2D.Double myCurrPos = new Point2D.Double(getX(), getY());
		// find the wave associated to the bullet
		if (!waves_mov.isEmpty()) {
			Wave hitWave = null;
			for (Wave wave : waves_mov) {
				if (wave.checkHit_Surfing(myCurrPos, event.getBullet().getPower(), getTime())) {
					hitWave = wave;
					break;
				}
			}

			// Update STAT
			if (hitWave != null){
				movManager.addSituation(hitWave.getRelatedSituation(),hitWave.GF_UsedByEnemy(myCurrPos) );
				
				out.println("AGGIUNTO e colpito ");
			}

		}

	}

	// update STAT array when the robot is hitted by a bullet
	public void updateStats(Wave wave, Point2D.Double pos) {

		int index = getStatsIndex(wave, pos);

		for (int i = 0; i < STATS_DIM; i++) {
			STAT[i] += 1.0 / (Math.pow(index - i, 2) + 1);
		}

	}

	public static Point2D.Double project(Point2D.Double sourceLocation, double angle, double length) {
		angle = Math.toRadians(angle);
		Point2D.Double endPoint = new Point2D.Double();
		endPoint.x = sourceLocation.x + Math.sin(angle) * length;
		endPoint.y = sourceLocation.y + Math.cos(angle) * length;
		return endPoint;
	}

	public void surfNearestWave() {
		Point2D.Double myPos = new Point2D.Double(getX(), getY());
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
		double absMinDistance = Math.abs(myPos.distance(waves_mov.get(0).getStartPoint())
				- waves_mov.get(0).getDistanceTraveled(getTime()));
		// it make one unusefull iteration
		for (Wave wave : waves_mov) {
			double absDistance = Math.abs(myPos.distance(wave.getStartPoint()) - wave.getDistanceTraveled(getTime()));
			if (absDistance < absMinDistance) {
				nearestWave = wave;
				absMinDistance = absDistance;
			}
		}

		// compute the orbit angle to surf the wave

		// angle between the wave origin and my current position
		double currentBearingRespectToWave = Math.atan2(myPos.x - nearestWave.getStartPoint().x,
				myPos.y - nearestWave.getStartPoint().y);

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
		Point2D.Double tmp = project(nearestWave.getMyPosAtFireTime(),
				-maxEscapeAngle / 2 + nearestWave.getStartAbsBearing(),
				nearestWave.getMyPosAtFireTime().distance(nearestWave.getStartPoint()));
		Point2D.Double tmp1 = project(nearestWave.getMyPosAtFireTime(),
				maxEscapeAngle / 2 + nearestWave.getStartAbsBearing(),
				nearestWave.getMyPosAtFireTime().distance(nearestWave.getStartPoint()));
		int px1[] = { (int) nearestWave.getStartPoint().x, (int) tmp1.x, (int) tmp.x };
		int py1[] = { (int) nearestWave.getStartPoint().y, (int) tmp1.y, (int) tmp.y };
		getGraphics().fillPolygon(px1, py1, 3);

		getGraphics().setColor(new Color(0.0f, 1.0f, 0.0f, 0.5f));

		Point2D.Double stat = new Point2D.Double();

		for (int i = 0; i < STATS_DIM; i++) {
			getGraphics().setColor(new Color((float) (1 / (STAT[i] + 1)), 0, 1 - (float) (1 / (STAT[i] + 1)), 0.5f));
			stat = project(nearestWave.getStartPoint(), -maxEscapeAngle / 2 + nearestWave.getStartAbsBearing() + i
					* (maxEscapeAngle / STATS_DIM), nearestWave.getDistanceTraveled(getTime()));
			getGraphics().fillRect((int) stat.x, (int) stat.y, 4, 4);
		}

	}

	// update waves position and delete unusefull waves
	public void updateWavesSurfer() {
		// myPos = new Point2D.Double(getX(), getY());
		// for (int i = 0; i < waves_mov.size(); i++) {
		// Wave wave = waves_mov.get(i);
		// wave.setDistanceTraveled((getTime() - wave.getFireTime())
		// * wave.getBulletVelocity());
		// if (wave.getDistanceTraveled() > myPos.distance(wave
		// .getStartPoint()) + 50) {
		// waves_mov.remove(i);
		// i--;
		// }
		// }
	}

	@Override
	public void onScannedRobot(ScannedRobotEvent e) {
		doScannedRobotSurfer(e);

		doScanner(e);
		// this method process already flying waves
		processWaves(e);

	}


	private RobotSituation getMySituation(double bearing){
		Point2D.Double myPos = new Point2D.Double(getX(),getY());
		mySituation= new RobotSituation();
		mySituation.setDistance(myPos.distance(enemyPos));
		mySituation.setTime(getTime());
		mySituation.setAdvancingVelocity(getVelocity() * -1
				* Math.cos(getHeadingRadians() - (bearing + getHeadingRadians())));
		mySituation.setLateralVelocity(getVelocity()
				* Math.sin(getHeadingRadians() - (bearing + getHeadingRadians())));
		mySituation.setPower(firePower);
		
		
		mySituation.printRobotSituation();

		return mySituation;


	}

	private void doScannedRobotSurfer(ScannedRobotEvent e) {
		Point2D.Double myPos = new Point2D.Double(getX(),getY());


		// absolute Bearing is needed to compute enemy position
		double absoluteBearing = getHeading() + e.getBearing();

		double bulletPower = enemyEnergy - e.getEnergy();

		if (bulletPower >= Rules.MIN_BULLET_POWER && bulletPower <= Rules.MAX_BULLET_POWER) {
			Wave wave = new Wave();
			wave.setPower(bulletPower);
			// -1 because the bullet was fired in the previous tick
			wave.setFireTime(getTime() - 1);
			// enemyPos refers the enemy position in the previous tick when the
			// bullet was fired
			wave.setStartPoint((Point2D.Double) enemyPos.clone());

			// needed to compute the guessing factor during the surfing
			// i don't know why + 180
			wave.setStartAbsBearing(Utils.normalRelativeAngleDegrees(absoluteBearing + 180));
			// needed to compute the guessing factor during the surfing
			wave.setDirection((int) Math.signum(getVelocity() * Math.sin(e.getBearingRadians())));

			wave.setMyPosAtFireTime(myPos);
			wave.setRelatedSituation(new RobotSituation(mySituation));

			waves_mov.add(wave);
		}
		enemyEnergy = e.getEnergy();
		mySituation= getMySituation(e.getBearingRadians());
		enemyPos = project(myPos, absoluteBearing, e.getDistance());

	}

	private void decayAppereance() {
		setColors(Constants.bodyColor, Constants.gunColor, Constants.radarColor);

	}

	private void doGun() {

	}

	private void doFirePower() {
		/**
		 * selects a bullet power based on our distance away from the target
		 * */
		Point2D.Double myCurrPos = new Point2D.Double(getX(), getY());
		firePower = 600 / myCurrPos.distance(enemyPos);
		if (firePower > 3) {
			firePower = 3;
		}

	}

	@Override
	public void onBulletHit(BulletHitEvent event) {
		// situationsKDTree = new KdTree<>(Constants.dimensions);

	}

	private void processWaves(ScannedRobotEvent e) {
		absBearing = getHeadingRadians() + e.getBearingRadians();

		// find our enemy's location:
		double ex = getX() + Math.sin(absBearing) * e.getDistance();
		double ey = getY() + Math.cos(absBearing) * e.getDistance();

		// Let's process the waves now:
		for (int i = 0; i < waves.size(); i++) {
			Wave currentWave = (Wave) waves.get(i);
			if (currentWave.checkHit(ex, ey, getTime())) {
				currentWave.getRelatedSituation().setRightGessFsctor(currentWave.getRightGuessFactor());
				situationsKDTree.addPoint(currentWave.getRelatedSituation().toKD_Key(),
						fromGuessFactorToIndex(currentWave.getRightGuessFactor() + 1, Constants.numSliceGF));

				situations.add(currentWave.getRelatedSituation());
				relatedClasses.add(fromGuessFactorToIndex(currentWave.getRightGuessFactor() + 1, Constants.numSliceGF));

				waves.remove(currentWave);
				i--;
			}
		}

		out.println("size tree " + situationsKDTree.size());

		RobotSituation newSituation = new RobotSituation();

		newSituation.setDistance(e.getDistance());
		newSituation.setBulletFlightTime(e.getDistance() / RoboUtils.bulletVelocity(firePower));
		newSituation.setTime(getTime());
		newSituation.setVelocity(e.getVelocity());
		newSituation.setAdvancingVelocity(e.getVelocity() * -1
				* Math.cos(e.getHeadingRadians() - (e.getBearingRadians() + getHeadingRadians())));
		newSituation.setLateralVelocity(e.getVelocity()
				* Math.sin(e.getHeadingRadians() - (e.getBearingRadians() + getHeadingRadians())));
		newSituation.setPower(firePower);
		if (situations.size() > 0) {
			double oldLatVel = situations.get(situations.size() - 1).getLateralVelocity();
			double oldAdvVel = situations.get(situations.size() - 1).getAdvancingVelocity();
			double deltaTime = getTime() - situations.get(situations.size() - 1).getTime();// currTime
			// - old
			// time
			newSituation.setLateralAcceleration(Enemy.returnAcceleration(oldLatVel, newSituation.getLateralVelocity(),
					deltaTime));
			newSituation.setAdvancingAcceleration(Enemy.returnAcceleration(oldAdvVel,
					newSituation.getAdvancingVelocity(), deltaTime));
		}

		situations.add(newSituation);

		newSituation.printRobotSituation();
		out.println("no situations " + situations.size());

		if (e.getVelocity() != 0) {
			if (Math.sin(e.getHeadingRadians() - absBearing) * e.getVelocity() < 0)
				direction = -1;
			else
				direction = 1;
		}

		Wave newWave = new Wave(getX(), getY(), absBearing, firePower, direction, getTime(), newSituation);

		RobotSituation best = newSituation;
		double guessfactor = best.getRightGessFsctor();
		// kernel density estimation
		double[] GF = new double[Constants.KNN_NUM_NEIGHBORS];
		double[] firingAngles = new double[Constants.KNN_NUM_NEIGHBORS];
		if (situationsKDTree.size() > Constants.KNNThreshold) {
			// MaxHeap<Integer> maxHeap =
			// situationsKDTree.findNearestNeighbors(newSituation.toKD_Key(),
			// 3,new SquareEuclideanDistanceFunction());
			MaxHeap<Integer> maxHeap = situationsKDTree.findNearestNeighbors(newSituation.toKD_Key(),
					Constants.KNN_NUM_NEIGHBORS, new WeightedSquareEuclideanDistance(Constants.dimensions,
							Constants.weights));

			// prenditi tutti i guessFactors -> trasformali in fireingAngles
			// corrispondenti
			// applica un kernel density estimator per calcolare il bestFiring
			// angle
			guessfactor = fromIndexToGuessFactor(maxHeap.getMax(), Constants.numSliceGF);
			out.println("MAX HEAP ->" + maxHeap.getMax());

			int cont = 0;
			while (maxHeap.size() > 0) {
				GF[cont] = fromIndexToGuessFactor(maxHeap.getMax(), Constants.numSliceGF);
				firingAngles[cont] = fromGuessFactorToFiringAngle(GF[cont], firePower);
				maxHeap.removeMax();
				out.println("gf " + GF[cont]);
				cont++;
			}
		}

		double bestFiringAngle = getBestFiringAngle(firingAngles, Constants.KNN_NUM_NEIGHBORS, best.getDistance());

		// double gunAdjust = fromGuessFactorToFiringAngle(guessfactor,
		// firePower);
		double gunAdjust = bestFiringAngle;
		out.println(" BEST " + bestFiringAngle + " NORMAL " + gunAdjust);

		setTurnGunRightRadians(gunAdjust);
		out.println(" " + direction + "," + guessfactor + "," + RoboUtils.getMaxEscapeAngle(firePower));

		waves.add(newWave);
		if (getGunHeat() == 0 && gunAdjust < Math.atan2(9, e.getDistance()))
			setFireBullet(firePower);

	}

	private double getBestFiringAngle(double[] firingAngles, int knnNumNeighbors, double distance) {
		double bestAngle = 0;
		double bestDensity = 0;
		double botWidthAngle = Math.abs(36 / distance);
		for (int i = 0; i < knnNumNeighbors; i++) {
			double density = 0;
			double currAngle_i = firingAngles[i];
			for (int j = 0; j < knnNumNeighbors; j++) {
				if (i != j) {
					double currAngle_j = firingAngles[j];
					double ux = (currAngle_i - currAngle_j) / botWidthAngle;
					if (Math.abs(ux) <= 1)//uniform
						density++;

					// gaussian
					//density = (1 / Math.sqrt(2 * PI)) * Math.exp(-0.5 * Math.pow(ux, 2));
				}

				if (density > bestDensity)
					bestAngle = currAngle_i;
				bestDensity = density;
			}
		}
		return bestAngle;
	}

	private double fromGuessFactorToFiringAngle(double GF, double bullettPower) {

		// this should do the opposite of the math in the WaveBullet:

		// double angleOffset = direction * guessfactor *
		// (EnemySituation.getMaxEscapeAngle(firePower));
		// double gunAdjust = Utils.normalRelativeAngle(
		// absBearing- getGunHeadingRadians() + angleOffset);

		double angleOffset = direction * GF * (RoboUtils.getMaxEscapeAngle(bullettPower));
		double gunAdjust = Utils.normalRelativeAngle(absBearing - getGunHeadingRadians() + angleOffset);
		return gunAdjust;
	}



	private void doScanner(ScannedRobotEvent e) {
		/**
		 * If i got the target i keep the eyes on it by scanning by WIDTH on
		 * both right and left side. It's a perfect lock scan. if e is NULL ->
		 * just a 2PI rotation scan is performed. This metod should be called in
		 * run() -> doScanner(null) onScannedRobot() -> doScanner(e) e not NULL
		 */
		if (getRadarTurnRemaining() == 0.0 && e == null) {
			setTurnRadarRightRadians(Double.POSITIVE_INFINITY);
		} else if (e != null) {
			double angleToEnemy = getHeadingRadians() + e.getBearingRadians();
			// Subtract current radar heading to get the turn required to face
			// the enemy, be sure it is normalized
			double radarTurn = Utils.normalRelativeAngle(angleToEnemy - getRadarHeadingRadians());
			// Distance we want to scan from middle of enemy to either side
			// The 36.0 is how many units from the center of the enemy robot it
			// scans.
			double extraTurn = Math.min(Math.atan(30.0 / e.getDistance()), Rules.RADAR_TURN_RATE_RADIANS);
			// Adjust the radar turn so it goes that much further in the
			// direction it is going to turn
			// Basically if we were going to turn it left, turn it even more
			// left, if right, turn more right.
			// This allows us to overshoot our enemy so that we get a good sweep
			// that will not slip.
			radarTurn += (radarTurn < 0 ? -extraTurn : extraTurn);

			// double gunTurn=0;
			// //Turn the radar
			// double absoluteBearing = getHeadingRadians() +
			// e.getBearingRadians();
			// setTurnGunRightRadians(
			// robocode.util.Utils.normalRelativeAngle(absoluteBearing -
			// getGunHeadingRadians()));
			// gunTurn= (getHeading()-getGunHeading()+e.getBearing())%360;

			setTurnRadarRightRadians(radarTurn);
			absBearing = getHeadingRadians() + e.getBearingRadians();
		}

	}

	// if a bearing is not within the -pi to pi range, alters it to provide the
	// shortest angle

	// compute the guess factor and convert it to an index on the STAT array
	public int getStatsIndex(Wave wave, Point2D.Double pos) {

		// Maximum escape angle computation
		double maxEscapeAngle = Math.asin(Rules.MAX_VELOCITY / wave.getBulletVelocity());
		// angle between the wave origin and pos
		double currentBearingRespectToWave = Math.atan2(pos.x - wave.getStartPoint().x, pos.y - wave.getStartPoint().y);

		double offset = currentBearingRespectToWave - Math.toRadians(wave.getStartAbsBearing());

		// guess factor represent the possible gun bearing represented as a
		// fraction of the maximum escape angle
		double guessFactor = wave.getDirection() * Utils.normalRelativeAngle(offset) / maxEscapeAngle;

		if (guessFactor > 1)
			guessFactor = 1;
		else if (guessFactor < -1)
			guessFactor = -1;

		guessFactor++;

		return (int) (STATS_DIM / 2 * guessFactor);
	}

	public double wallSmoothing(Point2D.Double botLocation, double angle, int orientation) {
		int fix = 0;
		getGraphics().fillRect((int) project(botLocation, getHeading() + fix, -WALL_STICK).x,
				(int) project(botLocation, getHeading() + fix, -WALL_STICK).y, 10, 10);
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
