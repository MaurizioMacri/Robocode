package surfing;

import kdtree.DistanceFunction;
import kdtree.KdTree;
import kdtree.MaxHeap;
import kdtree.SquareEuclideanDistanceFunction;
import ulils.RobotSituation;
import dalama.Constants;

public class MovementsManager {

	private static KdTree<Double> history;

	private DistanceFunction distanceFunction;

	public MovementsManager() {
		history = new KdTree<Double>(Constants.dimensions);
		distanceFunction = new SquareEuclideanDistanceFunction();
	}

	public void addSituation(RobotSituation newSituation, double GF) {
		history.addPoint(newSituation.toKD_Key(), GF);
	}

	public Double mostProbableGFEnemyFireTo(RobotSituation currSituation) {
		MaxHeap<Double> kNeighs = history.findNearestNeighbors(currSituation.toKD_Key(), Constants.KNN_NUM_NEIGHBORS, distanceFunction);

		double[] GF = new double[Constants.KNN_NUM_NEIGHBORS];

		int cont = 0;
		while (kNeighs.size() > 0) {
			GF[cont] = kNeighs.getMax();
			kNeighs.removeMax();
			cont++;
		}

		double bestGF = 0;
		double bestDensity = 0;
		for (int i = 0; i < Constants.KNN_NUM_NEIGHBORS; i++) {
			double density = 0;
			double currGF_i = GF[i];
			for (int j = 0; j < Constants.KNN_NUM_NEIGHBORS; j++) {
				if (i != j) {
					double currGF_j = GF[j];
					double ux = (currGF_i - currGF_j) / 2;// due perchè range
															// GF
					// if (Math.abs(ux) <= 1)//uniform
					// density++;
					// gaussian
					density = (1 / Math.sqrt(2 * Math.PI)) * Math.exp(-0.5 * Math.pow(ux, 2));
				}

				if (density > bestDensity)
					bestGF = currGF_i;
				bestDensity = density;
			}
		}
		return bestGF;

	}

	public int sizeHistory() {
		return history.size();
	}

}
