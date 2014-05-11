package ulils;

import java.awt.geom.Point2D;
import java.awt.geom.Point2D.Double;

import dalama.Constants;

import robocode.util.Utils;

public class Wave {

	private Point2D.Double startPoint;
	private Point2D.Double myPosAtFireTime;
	private double startAbsBearing;
	private double power;
	private long fireTime;
	private int direction;

	private double rightGuessFactor;

	private RobotSituation relatedSituation;

	public Wave() {
		// TODO Auto-generated constructor stub
	}

	public Wave(double x, double y, double bearing, double power,
			int direction, long time, RobotSituation situation) {
		startPoint = new Point2D.Double();
		myPosAtFireTime = new Point2D.Double();

		startPoint.x = x;
		startPoint.y = y;
		startAbsBearing = bearing;
		this.power = power;
		this.direction = direction;
		fireTime = time;
		relatedSituation=situation;

	}

	public double getDistanceTraveled(long currentTime) {
		return (currentTime - fireTime) * RoboUtils.bulletVelocity(power);
	}

	/*Va chiamata in onhitByBullet del robot*/
	public boolean checkHit_Surfing(Point2D.Double myCurrPosition,	double bulletPower, long currentTime) {
		//maurizio aveva messo maggiore TESTA BENE SE MINORE VA BENE!!!!!!
		if (Math.abs(myCurrPosition.distance(startPoint)- getDistanceTraveled(currentTime)) < Constants.waveVisibilityOffset
				&&
				Math.abs(RoboUtils.bulletVelocity(bulletPower)-RoboUtils.bulletVelocity(power))< Constants.EPISILON) {
			//quest'onda ha colpito il robot!
			return true;

		}
		return false;
	}

	public double GF_UsedByEnemy(){
		
		double maxEscapeAngle = RoboUtils.getMaxEscapeAngle(power);
		double currentBearingRespectToWave = Math.atan2(myPosAtFireTime.x - getStartPoint().x, myPosAtFireTime.y - getStartPoint().y);
		double offset = currentBearingRespectToWave - Math.toRadians(getStartAbsBearing());
		double guessFactor = getDirection() * Utils.normalRelativeAngle(offset) / maxEscapeAngle;
		if (guessFactor > 1)
			guessFactor = 1;
		else if (guessFactor < -1)
			guessFactor = -1;
		return guessFactor;
	}
	
	public boolean checkHit(double enemyX, double enemyY, long currentTime) {
		// if the distance from the wave origin to our enemy has passed
		// the distance the bullet would have traveled...
		if (Point2D.distance(startPoint.x, startPoint.y, enemyX, enemyY) <= getDistanceTraveled(currentTime)) {
			
			double desiredDirection = Math.atan2(enemyX - startPoint.x, enemyY
					- startPoint.y);
			double angleOffset = Utils.normalRelativeAngle(desiredDirection
					- startAbsBearing);
			double guessFactor = Math.max(
					-1,
					Math.min(1,
							angleOffset / RoboUtils.getMaxEscapeAngle(power)))
							* direction;

			rightGuessFactor = guessFactor;

			return true;
		}
		return false;
	}

	public double getBulletVelocity(){
		return RoboUtils.bulletVelocity(power);
	}

	public Point2D.Double getStartPoint() {
		return startPoint;
	}

	public void setStartPoint(Point2D.Double startPoint) {
		this.startPoint = startPoint;
	}

	public Point2D.Double getMyPosAtFireTime() {
		return myPosAtFireTime;
	}

	public void setMyPosAtFireTime(Point2D.Double myPosAtFireTime) {
		this.myPosAtFireTime = myPosAtFireTime;
	}



	public double getStartAbsBearing() {
		return startAbsBearing;
	}



	public void setStartAbsBearing(double startAbsBearing) {
		this.startAbsBearing = startAbsBearing;
	}

	public double getPower() {
		return power;
	}

	public void setPower(double power) {
		this.power = power;
	}

	public long getFireTime() {
		return fireTime;
	}

	public void setFireTime(long fireTime) {
		this.fireTime = fireTime;
	}

	public int getDirection() {
		return direction;
	}

	public void setDirection(int direction) {
		this.direction = direction;
	}

	public double getRightGuessFactor() {
		return rightGuessFactor;
	}

	public RobotSituation getRelatedSituation() {
		return relatedSituation;
	}

	public void setRelatedSituation(RobotSituation relatedSituation) {
		this.relatedSituation = relatedSituation;
	}




}
