package dalama;

import java.awt.Color;

public class Constants {
	/* Robot Decay/Appereance constants*/
	
	
	public static final Color bodyColor = Color.RED;
	public static final Color gunColor = Color.BLUE;
	public static final Color radarColor = Color.GREEN;
	
	public static   int forcePower = 3; 
	public static final int KNNThreshold = 7;
	public static final int KNN_NUM_NEIGHBORS= 5 ;
	
	
	public static final int numSliceGF=1000;
	public static final int dimensions = 6;
	public static final double[] weights={0.5,1.0,1.0,0.6,0.6,1.0};//same size as dimensions
	public static final int limitSituations = 200;
	
	
	public static final int waveVisibilityOffset = 50;//in pixels
	public static final double EPISILON = 0.001;
	
	public static double getBulletDamage(double energy) {
		
        double damage= 4 * energy;

        if (energy > 1)
            damage += 2 * (energy - 1);

        return damage;
    }

    public static double bulletDamageToEnergy(double damage) {
        double energy;

        if (damage <= 4) {

            energy= damage / 4;
        } else {
            energy= (damage + 2) / 6;
        }

        return energy;
    }


}
