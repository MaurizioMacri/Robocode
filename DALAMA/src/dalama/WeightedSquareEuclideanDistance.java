package dalama;
import kdtree.DistanceFunction;


public class WeightedSquareEuclideanDistance implements DistanceFunction{

	int dim;
	double [] weights;
	public WeightedSquareEuclideanDistance(int dimWeights,double[] weights) {
		this.dim=dimWeights;
		this.weights=weights;
	}
	
	
	
	  @Override
	    public double distance(double[] p1, double[] p2) {
	        double d = 0;

	        for (int i = 0; i < p1.length; i++) {
	            double diff = (p1[i] - p2[i])*weights[i];
	            d += diff * diff;
	        }

	        return d;
	    }

	    @Override
	    public double distanceToRect(double[] point, double[] min, double[] max) {
	        double d = 0;

	        for (int i = 0; i < point.length; i++) {
	            double diff = 0;
	            if (point[i] > max[i]) {
	                diff = (point[i] - max[i]);
	            }
	            else if (point[i] < min[i]) {
	                diff = (point[i] - min[i]);
	            }
	            diff*=weights[i];
	            d += diff * diff;
	        }

	        return d;
	    }
}
