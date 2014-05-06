package dalama;

import java.awt.geom.Point2D;

public class Wave {

	private Point2D.Double fireLocation;
	private Point2D.Double myPosAtFireTime;

	private long fireTime;
	private double bulletVelocity;
	private double distanceTraveled;
	private double absWaveBearing;
	private int myDirAtFireTime;

	public Wave() {
		// TODO Auto-generated constructor stub
	}

	public Point2D.Double getMyPosAtFireTime() {
		return myPosAtFireTime;
	}

	public void setMyPosAtFireTime(Point2D.Double myPosAtFireTime) {
		this.myPosAtFireTime = myPosAtFireTime;
	}

	public int getMyDirAtFireTime() {
		return myDirAtFireTime;
	}

	public void setMyDirAtFireTime(int myDirAtFireTime) {
		this.myDirAtFireTime = myDirAtFireTime;
	}

	public double getAbsWaveBearing() {
		return absWaveBearing;
	}

	public void setAbsWaveBearing(double absWaveBearing) {
		this.absWaveBearing = absWaveBearing;
	}

	public Point2D.Double getFireLocation() {
		return fireLocation;
	}

	public void setFireLocation(Point2D.Double fireLocation) {
		this.fireLocation = fireLocation;
	}

	public long getFireTime() {
		return fireTime;
	}

	public void setFireTime(long fireTime) {
		this.fireTime = fireTime;
	}

	public double getBulletVelocity() {
		return bulletVelocity;
	}

	public void setBulletVelocity(double bulletVelocity) {
		this.bulletVelocity = bulletVelocity;
	}

	public double getDistanceTraveled() {
		return distanceTraveled;
	}

	public void setDistanceTraveled(double distanceTraveled) {
		this.distanceTraveled = distanceTraveled;
	}

}
