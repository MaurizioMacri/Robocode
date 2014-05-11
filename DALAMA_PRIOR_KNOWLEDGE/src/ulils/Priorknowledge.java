package ulils;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

import robocode.RobocodeFileOutputStream;
import robocode.RobocodeFileWriter;
import kdtree.KdTree;

public class Priorknowledge {

	private String path_file;

	public void setPath_file(String path_file) {
		this.path_file = path_file;
	}

	public Priorknowledge(String path_file) {

		this.path_file = path_file;
	}

	public Priorknowledge() {
	}

	// serve per poter riempire il file come se fosse un addestramento
	public void addPriorKnowledge(double[] tmp, double a) {
		
		try (RobocodeFileWriter filewriter =new RobocodeFileWriter(path_file+"/prior.dat",true)) {
			
			for (int i = 0; i < tmp.length; i++) {
				double d = tmp[i];
				filewriter.write(d + " ");
			}
			filewriter.write(a+"");
			filewriter.write(System.getProperty("line.separator"));
			filewriter.close();
		} catch (IOException e) {
			System.out.println(e);
			// exception handling left as an exercise for the reader
		}
	}

	//serve per caricare dal file
	public void loadPriorKnowledge(KdTree<Integer> tmp) {
		try {
			 BufferedReader br = new BufferedReader(new FileReader(path_file+"/prior.dat"));
			    try {
			        StringBuilder sb = new StringBuilder();
			        String line = br.readLine();
			        String[] a = line.split(" ");
		        while (line != null) {
			        	  double[] part1 = new double[6];
					        for (int i = 0; i < a.length-1; i++) {
					        	part1[i] = Double.parseDouble(a[i]);					        	
							}
					        int part2= (int) Double.parseDouble(a[a.length-1]);
					        tmp.addPoint(part1, part2);
			            sb.append(line);
			            sb.append(System.lineSeparator());
			            line = br.readLine();
			        }
			    } finally {
			        br.close();
			    }
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

}
