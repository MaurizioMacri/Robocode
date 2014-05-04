package wave;

import java.awt.Color;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;

import robocode.AdvancedRobot;
import robocode.HitByBulletEvent;
import robocode.Rules;
import robocode.ScannedRobotEvent;
import robocode.util.Utils;

public class WaveSurfingDamaal extends AdvancedRobot {

	private static int STATS_DIM = 47;
	private static double WALL_STICK = 100;
	private static Rectangle2D.Double fieldRect = new java.awt.geom.Rectangle2D.Double(18, 18, 764, 564);

	private Point2D.Double enemyPos;
	private Point2D.Double myPos;
	private ArrayList<Wave> waves_mov;
	private double enemyEnergy;
	private int dir;
	private double orbitAngle;
	private double STAT[];
	

	public WaveSurfingDamaal() {
		enemyEnergy = 0;
		orbitAngle = 0;
		enemyPos = new Point2D.Double();
		waves_mov = new ArrayList<Wave>();
		dir = 1;
		STAT = new double[STATS_DIM];
	}

	@Override
	public void run() {
		enemyEnergy = getEnergy();
		setAdjustGunForRobotTurn(true);
		setAdjustRadarForGunTurn(true);

		while (true) {
			if (getRadarTurnRemaining() == 0) {
				setTurnRadarLeft(360);
			}

			updateWaves();

			surfNearestWave();

			drawWaves();

			execute();
		}

	}

	@Override
	public void onScannedRobot(ScannedRobotEvent event) {

		// absolute Bearing is needed to compute enemy position
		double absoluteBearing = getHeading() + event.getBearing();

		double bulletPower = enemyEnergy - event.getEnergy();

		if (bulletPower >= Rules.MIN_BULLET_POWER && bulletPower <= Rules.MAX_BULLET_POWER) {
			Wave wave = new Wave();
			wave.setBulletVelocity(bulletVelocity(bulletPower));
			wave.setDistanceTraveled(bulletVelocity(bulletPower));
			// -1 because the bullet was fired in the previous tick
			wave.setFireTime(getTime() - 1);
			// enemyPos refers the enemy position in the previous tick when the
			// bullet was fired
			wave.setFireLocation((Point2D.Double) enemyPos.clone());

			// needed to compute the guessing factor during the surfing
			// i don't know why + 180
			wave.setAbsWaveBearing(Utils.normalRelativeAngleDegrees(absoluteBearing + 180));
			// needed to compute the guessing factor during the surfing
			wave.setMyDirAtFireTime((int) Math.signum(getVelocity() * Math.sin(event.getBearingRadians())));

			wave.setMyPosAtFireTime(myPos);

			waves_mov.add(wave);
		}
		enemyEnergy = event.getEnergy();
		enemyPos = project(myPos, absoluteBearing, event.getDistance());
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

	// update STAT array when the robot is hitted by a bullet
	public void updateStats(Wave wave, Point2D.Double pos) {

		int index = getStatsIndex(wave, pos);

		for (int i = 0; i < STATS_DIM; i++) {
			STAT[i] += 1.0 / (Math.pow(index - i, 2) + 1);
		}

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
	public void updateWaves() {
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

	public static double bulletVelocity(double power) {
		// CHECK ROBOCODE PHISICS
		return (20.0 - (3.0 * power));
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

	public static Point2D.Double project(Point2D.Double sourceLocation, double angle, double length) {
		angle = Math.toRadians(angle);
		Point2D.Double endPoint = new Point2D.Double();
		endPoint.x = sourceLocation.x + Math.sin(angle) * length;
		endPoint.y = sourceLocation.y + Math.cos(angle) * length;
		return endPoint;
	}

	private void drawWaves() {
		getGraphics().setColor(Color.blue);
		for (Wave w : waves_mov) {
			getGraphics().drawArc((int) w.getFireLocation().x - (int) w.getDistanceTraveled() / 2, (int) w.getFireLocation().y - (int) w.getDistanceTraveled() / 2, (int) w.getDistanceTraveled(), (int) w.getDistanceTraveled(), 0, 360);
		}
	}

}
