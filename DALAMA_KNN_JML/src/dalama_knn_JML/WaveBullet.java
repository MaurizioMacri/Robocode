package dalama_knn_JML;

import java.awt.geom.Point2D;

import robocode.Rules;
import robocode.util.Utils;

public class WaveBullet
{
	private double startX, startY, startBearing, power;
	private long   fireTime;
	private int    direction;
	
	private double rightGuessFactor;
	
	private EnemySituation relatedSituation;
	

	public WaveBullet(double x, double y, double bearing, double power,
			int direction, long time, EnemySituation situation)
	{
		startX         = x;
		startY         = y;
		startBearing   = bearing;
		this.power     = power;
		this.direction = direction;
		fireTime       = time;
		rightGuessFactor=0.0d;
		relatedSituation=situation;
		}


	public static double getBulletSpeed(double power)
	{
		return (20 - (power * 3));
	}

	
	public double maxEscapeAngle()
	{
		return Math.asin(Rules.MAX_VELOCITY / getBulletSpeed(power));
	}
	public boolean checkHit(double enemyX, double enemyY, long currentTime)
	{
		// if the distance from the wave origin to our enemy has passed
		// the distance the bullet would have traveled...
		if (Point2D.distance(startX, startY, enemyX, enemyY) <= 
				(currentTime - fireTime) * getBulletSpeed(power))
		{
			double desiredDirection = Math.atan2(enemyX - startX, enemyY - startY);
			double angleOffset = Utils.normalRelativeAngle(desiredDirection - startBearing);
			double guessFactor =
					Math.max(-1, Math.min(1, angleOffset / maxEscapeAngle())) * direction;
			
			
			rightGuessFactor=guessFactor;
			
			return true;
		}
		return false;
	}


	public double getRightGuessFactor() {
		return rightGuessFactor;
	}


	public EnemySituation getRelatedSituation() {
		return relatedSituation;
	}


	public void setRelatedSituation(EnemySituation relatedSituation) {
		this.relatedSituation = relatedSituation;
	}



	
	
	
	
} // end CLASS
