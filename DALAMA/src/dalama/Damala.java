package dalama;

import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.List;
import java.util.Vector;

import kdtree.KdTree;
import kdtree.MaxHeap;
import robocode.AdvancedRobot;
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

	private static double WALL_STICK = 80;
	private static Rectangle2D.Double fieldRect = new java.awt.geom.Rectangle2D.Double(18, 18, 764, 564);
	private static ArrayList<Wave> waves_mov;

	private Point2D.Double enemyPos;
	private double enemyEnergy;
	private int dir;
	private double orbitAngle;

	private ArrayList<RobotSituation> situations = new ArrayList<>();
	private ArrayList<Integer> relatedClasses = new ArrayList<>();
	/* END WAVESURFER VARIABLES */

	private static KdTree<Integer> situationsKDTree = new KdTree<>(Constants.dimensions);
	private static List<Wave> waves = new Vector<>();
	private static MovementsManager movManager = new MovementsManager();;

	private double absBearing;
	private int direction;
	private double firePower;
	// middle of the field
	private RobotSituation mySituation;

	@Override
	public void run() {
		enemyEnergy = getEnergy();// surfer

		initializeStuff();
		decayAppereance();

		do {

			// Turn the radar if we have no more turn, starts it if it stops and
			// at the start of round
			surfNearestWave();

			// doMove();
			doFirePower();
			doScanner(null);
			// fire(firePower);
			execute();
//			out.println("size = " + movManager.sizeHistory());
		} while (true);

	}

	private void initializeStuff() {

		setAdjustGunForRobotTurn(true);
		setAdjustRadarForGunTurn(true);
		firePower = 0.0;

		// initialize waveSurfer
		enemyEnergy = 0;
		orbitAngle = 0;
		enemyPos = new Point2D.Double();
		waves_mov = new ArrayList<Wave>();
		dir = 1;

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
			if (hitWave != null)
				movManager.addSituation(hitWave.getRelatedSituation(), hitWave.GF_UsedByEnemy());

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
		long currTime = getTime();

		if (waves_mov.size() == 0)
			return;

		// find the nearest wave
		Wave nearestWave = waves_mov.get(0);
		double absMinDistance = Math.abs(myPos.distance(waves_mov.get(0).getStartPoint()) - waves_mov.get(0).getDistanceTraveled(currTime));
		// it make one unusefull iteration
		for (Wave wave : waves_mov) {
			double absDistance = Math.abs(myPos.distance(wave.getStartPoint()) - wave.getDistanceTraveled(currTime));
			if (absDistance < absMinDistance) {
				nearestWave = wave;
				absMinDistance = absDistance;
			}
		}

		double currentBearingRespectToWave = Math.atan2(myPos.x - nearestWave.getStartPoint().x, myPos.y - nearestWave.getStartPoint().y);

		if (movManager.sizeHistory() > Constants.KNN_NUM_NEIGHBORS) {
			double GFPredicted = movManager.mostProbableGFEnemyFireTo(nearestWave.getRelatedSituation());
			if (GFPredicted < 0) {
				// destra
				dir = 1;
			} else if (GFPredicted > 0) {
				dir = -1;
			}
		}

		orbitAngle = Math.toDegrees(currentBearingRespectToWave);
		orbitAngle = wallSmoothing(myPos, orbitAngle + (90) * dir, dir);

		setBackAsFront(this, orbitAngle);
	}

	@Override
	public void onScannedRobot(ScannedRobotEvent e) {
		doScannedRobotSurfer(e);

		doScanner(e);
		// this method process already flying waves
		processWaves(e);

	}

	private RobotSituation getMySituation(double bearing) {
		Point2D.Double myPos = new Point2D.Double(getX(), getY());
		mySituation = new RobotSituation();
		mySituation.setDistance(myPos.distance(enemyPos));
		mySituation.setTime(getTime());
		mySituation.setAdvancingVelocity(getVelocity() * -1 * Math.cos(getHeadingRadians() - (bearing + getHeadingRadians())));
		mySituation.setLateralVelocity(getVelocity() * Math.sin(getHeadingRadians() - (bearing + getHeadingRadians())));
		mySituation.setPower(firePower);

		// mySituation.printRobotSituation();

		return mySituation;

	}

	private void doScannedRobotSurfer(ScannedRobotEvent e) {
		Point2D.Double myPos = new Point2D.Double(getX(), getY());

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
		mySituation = getMySituation(e.getBearingRadians());
		enemyPos = project(myPos, absoluteBearing, e.getDistance());

	}

	private void decayAppereance() {
		setColors(Constants.bodyColor, Constants.gunColor, Constants.radarColor);
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
				situationsKDTree.addPoint(currentWave.getRelatedSituation().toKD_Key(), fromGuessFactorToIndex(currentWave.getRightGuessFactor() + 1, Constants.numSliceGF));

				situations.add(currentWave.getRelatedSituation());
				relatedClasses.add(fromGuessFactorToIndex(currentWave.getRightGuessFactor() + 1, Constants.numSliceGF));

				waves.remove(currentWave);
				i--;
			}
		}

		// out.println("size tree " + situationsKDTree.size());

		RobotSituation newSituation = new RobotSituation();

		newSituation.setDistance(e.getDistance());
		newSituation.setBulletFlightTime(e.getDistance() / RoboUtils.bulletVelocity(firePower));
		newSituation.setTime(getTime());
		newSituation.setVelocity(e.getVelocity());
		newSituation.setAdvancingVelocity(e.getVelocity() * -1 * Math.cos(e.getHeadingRadians() - (e.getBearingRadians() + getHeadingRadians())));
		newSituation.setLateralVelocity(e.getVelocity() * Math.sin(e.getHeadingRadians() - (e.getBearingRadians() + getHeadingRadians())));
		newSituation.setPower(firePower);
		if (situations.size() > 0) {
			double oldLatVel = situations.get(situations.size() - 1).getLateralVelocity();
			double oldAdvVel = situations.get(situations.size() - 1).getAdvancingVelocity();
			double deltaTime = getTime() - situations.get(situations.size() - 1).getTime();// currTime
			// - old
			// time
			newSituation.setLateralAcceleration(Enemy.returnAcceleration(oldLatVel, newSituation.getLateralVelocity(), deltaTime));
			newSituation.setAdvancingAcceleration(Enemy.returnAcceleration(oldAdvVel, newSituation.getAdvancingVelocity(), deltaTime));
		}

		situations.add(newSituation);

		// newSituation.printRobotSituation();
		// out.println("no situations " + situations.size());

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
			MaxHeap<Integer> maxHeap = situationsKDTree.findNearestNeighbors(newSituation.toKD_Key(), Constants.KNN_NUM_NEIGHBORS, new WeightedSquareEuclideanDistance(Constants.dimensions, Constants.weights));

			// prenditi tutti i guessFactors -> trasformali in fireingAngles
			// corrispondenti
			// applica un kernel density estimator per calcolare il bestFiring
			// angle
			guessfactor = fromIndexToGuessFactor(maxHeap.getMax(), Constants.numSliceGF);
			// out.println("MAX HEAP ->" + maxHeap.getMax());

			int cont = 0;
			while (maxHeap.size() > 0) {
				GF[cont] = fromIndexToGuessFactor(maxHeap.getMax(), Constants.numSliceGF);
				firingAngles[cont] = fromGuessFactorToFiringAngle(GF[cont], firePower);
				maxHeap.removeMax();
				// out.println("gf " + GF[cont]);
				cont++;
			}
		}

		double bestFiringAngle = getBestFiringAngle(firingAngles, Constants.KNN_NUM_NEIGHBORS, best.getDistance());

		double gunAdjust = bestFiringAngle;
		// out.println(" BEST " + bestFiringAngle + " NORMAL " + gunAdjust);

		setTurnGunRightRadians(gunAdjust);
		// out.println(" " + direction + "," + guessfactor + "," +
		// RoboUtils.getMaxEscapeAngle(firePower));

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
					if (Math.abs(ux) <= 1)// uniform
						density++;

					// gaussian
					// density = (1 / Math.sqrt(2 * PI)) * Math.exp(-0.5 *
					// Math.pow(ux, 2));
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

	public static Point2D.Double projectRadians(Point2D.Double sourceLocation, double angle, double length) {
		Point2D.Double endPoint = new Point2D.Double();
		endPoint.x = sourceLocation.x + Math.sin(angle) * length;
		endPoint.y = sourceLocation.y + Math.cos(angle) * length;
		return endPoint;
	}

	public double wallSmoothing(Point2D.Double botLocation, double angle, int orientation) {
		while (!fieldRect.contains(project(botLocation, angle, WALL_STICK))) {
			angle += orientation * 0.5;
		}
		return angle;
	}

	public static void setBackAsFront(AdvancedRobot robot, double goAngle) {
		double angle = Utils.normalRelativeAngleDegrees(goAngle - robot.getHeading());
		if (Math.abs(angle) > (90)) {
			if (angle < 0) {
				robot.setTurnRight(90 + angle);
			} else {
				robot.setTurnLeft(90 - angle);
			}
			robot.setBack(100);
		} else {
			if (angle < 0) {
				robot.setTurnLeft(-1 * angle);
			} else {
				robot.setTurnRight(angle);
			}
			robot.setAhead(100);
		}
	}

}
