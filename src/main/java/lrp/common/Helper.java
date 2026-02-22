package lrp.common;

import lrp.data.Instance;
import java.io.*;
import java.text.DecimalFormat;
import java.util.*;

public class Helper {

	public Instance inst;

	public DecimalFormat df = new DecimalFormat("#.####");

	// Constructor
	public Helper(Instance _inst) {
		inst = _inst;
	}

	/****************************************************** Heuristic initial *****************************************
	 =================================================================================================================*/

	// Get one route cost
	public double get_route_cost(ArrayList<Integer> route) {
		double cost = 0;
		if(route.size() == 2){
			return 0;
		}
		for(int i = 1; i < route.size(); ++i) {
			cost += inst.t[route.get(i-1)][route.get(i)] + inst.s[route.get(i-1)];
		}
		return cost;
	}

	// Get one route capacity
	public double get_route_capacity(ArrayList<Integer> route) {
		double ele = 0;
		if(route.size() == 2){
			return 0;
		}
		for(int i = 1; i < route.size()-1; ++i) {
			ele += inst.q[route.get(i)];
		}
		return ele;
	}

	// Get one route cost
	public double get_route_cost_group(ArrayList<Integer> route) {
		double cost = 0;
		if(route.size() == 2){
			return 0;
		}
		for(int i = 1; i < route.size(); ++i) {
			cost += inst.t[inst.map[route.get(i-1)]][inst.map[route.get(i)]] + inst.s[inst.map[route.get(i)]];
		}
		return cost;
	}

	// check feasibility
	public void check_solution(ArrayList<ArrayList<Integer>> solution){
		// elementary & capacity
		int[] visit = new int[inst.N];
		for(int i = 0; i < solution.size(); i++){
			double cap = 0;
			for(int j = 1; j < solution.get(i).size() - 1; j++){
				int id = solution.get(i).get(j);
				cap += inst.q[id];
				int temp = visit[id]++;
				if(temp > 1){
					System.out.println("multiple visit!!!" + id);
					System.exit(0);
				}
			}
			if(cap > inst.Q){
				System.out.println("capacity error !!!" + cap + " " + inst.Q);
			}
		}
	}

	// print solution
	public void print_heuristic_solution(ArrayList<ArrayList<Integer>> solution, double objective, double time){
		System.out.println("\nObjective: " + objective + " \tTime: " + time);

		for (int j = 0; j < solution.size(); j++){
			System.out.println(solution.get(j) + "  cost:" + get_route_cost(solution.get(j)) + " cap:" + get_route_capacity(solution.get(j)));
		}

	}

	/********************************************************* Benders *************************************************
	 ==================================================================================================================*/

	// Benders check
	public void CheckSolution(ArrayList<ArrayList<ArrayList<Integer>>> solution, double obj) {

		double totalCost = 0;
		Set<Integer> depots = new HashSet<>();
		int[] visit = new int[inst.N + 2 * inst.M + 1];
		for (int i = 0; i < solution.size(); i++) {
			ArrayList<ArrayList<Integer>> routes = solution.get(i);
			if(routes.size() == 0)
				continue;
			int depot_id = routes.get(0).get(0);
			depots.add(depot_id);
			if(routes.size() > inst.para.MaxVehicleNumber){
				System.out.println("Vehicle Number Exceed !!! " + i + " " + routes.size() + " " + inst.para.MaxVehicleNumber);
				System.exit(0);
			}

			if (routes.size() == 0) {
				continue;
			}

			double duration = 0;
			for (int j = 0; j < routes.size(); j++) {
				ArrayList<Integer> route = routes.get(j);

				totalCost += get_route_cost(route);

				// 电量合法
				double ele = get_route_capacity(route);
				if (ele > inst.Q) {
					System.out.println("Electric err : " + ele + " " + inst.Q);
				}

				// duration 合法
				duration += get_route_capacity(route);
				if (duration > inst.D[depot_id]) {
					System.out.println("Duration err : " + duration + " " + inst.D[depot_id]);
				}

				// 每个点必须被访问一次
				for (int c = 1; c < route.size()-1; c++) {
					visit[route.get(c)]++;
				}
			}
		}
		for(Integer k : depots){
			totalCost += inst.fCost[k];
		}

		// 每个点必须被访问一次
		for(int c = 1; c < inst.N + 1; c++){
			if(visit[c] < 1){
				System.out.println("visit err: " + c + " is not be  visited");
			}
			if (visit[c] > 1) {
				System.out.println("visit err : " + c + " is visited more than one times");
			}
		}

		// 路径成本是否与目标函数一致;
		if( Math.abs(totalCost - obj) > 1e-9){
			System.out.println("objective is not match solutions, hand_cal:" + totalCost + " cplex_cal:" + obj);
		}
	}

	// Check feasibility
	public boolean check_cplex_solution(ArrayList<Integer> location, Map<Integer, ArrayList<Integer>> assign, ArrayList<ArrayList<ArrayList<Integer>>> routes, double obj){

		for(int i = 0; i < location.size(); i++){
			int id = location.get(i);
			if(id <= inst.N || id >= inst.N + inst.M + 1){
				System.out.println("location id error !!!");
				return false;
			}
		}

		// assignment feasibility
		int[] a = new int[inst.N+1];
		for (Map.Entry<Integer, ArrayList<Integer>> entry : assign.entrySet()) {

			// associate to location
			if(!location.contains(entry.getKey())){
				System.out.println("assign to wrong parking lot !!!");
				return false;
			}

			// assign all base station once
			ArrayList<Integer> assi = entry.getValue();
			for(int i = 0; i < assi.size(); i++){
				a[assi.get(i)]++;
				if(a[i] > 1) {
					System.out.println("a station is assigned more than once !!!" + a[i] + "times , " + i + " " + assi);
					for (Map.Entry<Integer, ArrayList<Integer>> entry1 : assign.entrySet()) {
						System.out.println(entry1);
					}
					return false;
				}
			}
		}

		for(int i = 1; i < a.length; i++){
			if (a[i] < 1){
				System.out.println("a station is not assigned !!!" + a[i] + "times , " + i);
				return false;
			}
		}

		// solution 的合法性
		CheckSolution(routes,obj);

		return true;
	}

	/****************************************************** Printer ****************************************************
	 ==================================================================================================================*/

	public void printInstance(){
		System.out.println("Name: " + inst.name);
		System.out.println("M: " + inst.M);
		System.out.println("N: " + inst.N);
		System.out.println("D: " + inst.D);
		System.out.println("FixCost: " + inst.fCost);
		System.out.println("E: " + inst.Q);

		System.out.println("iter. " + "\t" + "lat. " + "\t" + "lng.");
		for(int i = 0; i < inst.lat.length; i++){
			System.out.println(i + "\t" + inst.lat[i] +  "\t" + inst.lng[i]);
		}
		System.out.println("s:");
		for(int i = 0; i < inst.s.length; i++){
			System.out.print("["+i + "," + inst.s[i] + "] ");
		}
		System.out.println();

		System.out.println("se:");
		for(int i = 0; i < inst.q.length; i++){
			System.out.print(inst.q[i] + " ");
		}
		System.out.println();

		System.out.println("t:");
		for(int i = 0; i < inst.t.length; i++){
			for(int j = 0; j < inst.t[i].length; j++){
				System.out.print(inst.t[i][j] + " ");
			}
			System.out.println();
		}

	}

	// for benchmark barreto
	public void printBenchmarkInstance(){
		System.out.println("N: " + inst.N);
		System.out.println("M: " + inst.M);

		System.out.print("D: ");
		for(int i = inst.N + 1; i < inst.fCost.length; i++){
			System.out.print(inst.D[i] + " ");
		}
		System.out.println();

		System.out.print("FixCost: ");
		for(int i = inst.N + 1; i < inst.fCost.length; i++){
			System.out.print(inst.fCost[i] + " ");
		}
		System.out.println();
		System.out.println("E: " + inst.Q);

		System.out.println("lat." + "\t" + "lng.");
		for(int i = 0; i < inst.lat.length; i++){
			System.out.println(i + "\t" + inst.lat[i] +  "\t" + inst.lng[i]);
		}
		System.out.println("s:");
		for(int i = 0; i < inst.s.length; i++){
			System.out.print("["+i + "," + inst.s[i] + "] ");
		}
		System.out.println();

		System.out.println("se:");
		for(int i = 0; i < inst.q.length; i++){
			System.out.print("["+i + "," + inst.q[i] + "] ");
		}
		System.out.println();

		System.out.println("t:");
		for(int i = 0; i < inst.t.length; i++){
			System.out.print(i + ":");
			for(int j = 0; j < inst.t[i].length; j++){
				System.out.print(inst.t[i][j] + " ");
			}
			System.out.println();
		}

	}

	public void printBestBPSolution(ArrayList<ArrayList<Integer>> solution,
								  double objective,
								  double time){
		System.out.println("\nObjective: " + objective + " \tTime: " + time);

		for (int j = 0; j < solution.size(); j++){
			System.out.println(solution.get(j) + " : " + get_route_cost_group(solution.get(j)));
		}

	}

	// Print Solution information
	public void printCplexSolution(double ub, double gap, double lb,  int nodeN, double time, ArrayList<ArrayList<ArrayList<Integer>>> solution) {

		System.out.println("\nUb: " + ub + "\tGap: " + gap + "\tLb: " +lb + "\tnodeN: " + nodeN  + "\ttime: " + time);

		int iter = 0;
		for(int i = 0;i < solution.size(); i++){
			ArrayList<ArrayList<Integer>> routes = solution.get(i);
			for(int j = 0; j < routes.size(); j++){
				ArrayList<Integer> route = routes.get(j);
				double rouCost = get_route_cost(route);
				double cap = get_route_capacity(route);
				System.out.println("Rid: " + (iter++) + " Cap " + cap + " Cost " + rouCost + ": " + route);
			}
		}
	}

	public void printBestSolution(ArrayList<ArrayList<ArrayList<Integer>>> solution,
								  double objective,
								  double time,
								  String status,
								  int nodeN){
		System.out.println("\nObjective: " + objective + " \tTime: " + time + " \tStatus: " + status + " \tnodeN: " + nodeN);
		int iter = 0;
		for (int j = 0; j < solution.size(); j++){
			for (int i = 0; i < solution.get(j).size(); i++){
				System.out.println("Rid: " + (iter++) + " Cap " + get_route_capacity(solution.get(j).get(i)) + " Cost " + get_route_cost(solution.get(j).get(i)) + ": " + solution.get(j).get(i));
			}
		}
	}

	public void printMas(int iter,
						 double obj,
						 double fixCost,
						 double totalBendersVV,
						 double[] bendersVV,
						 double[] ff,
						 double[][] zz) {
		if(iter == 0){
			System.out.println("\n----------------------master Problem " + iter);
			System.out.println("Obj: " + obj + "\tFC: " + fixCost + "\tBenders: " + totalBendersVV + "\tbendersVV: " + Arrays.toString(bendersVV));
		}else{
			System.out.println("Obj: " + obj + "\tFC: " + fixCost + "\tBenders: " + totalBendersVV + "\tbendersVV: " + Arrays.toString(bendersVV));
		}
		if(inst.para.PrintBendersAssign){
			for (int j = inst.N+1; j < inst.N + inst.M +1; j++){
				if(ff[j] > 0.5){
					System.out.print(j + " : ");
				}else{
					continue;
				}
				double capacity = 0;
				for(int i = 1; i < inst.N + 1; i++){
					if(zz[j][i] > 0.5) {
						System.out.print(i + "  ");
						capacity += inst.q[i];
					}
				}
				System.out.println(" cap:" + capacity );
			}
		}
	}

	public void printSub(int iter,
						 double obj,
						 ArrayList<ArrayList<ArrayList<Integer>>> subRoutes ) {
		System.out.println("\n----------------------sub Problem " + iter);
		System.out.println("Obj: " + obj);
		if(subRoutes == null)
			return;
		for (int j = 0; j < subRoutes.size(); j++){
			for (int i = 0; i < subRoutes.get(j).size(); i++){
				System.out.println(subRoutes.get(j).get(i) + " " + get_route_cost(subRoutes.get(j).get(i)));
			}
		}
	}

	public void printIter(int iter,
						  double LB,
						  double UB,
						  double tol) {
		System.out.println("\n----------------------iteration " + iter );
		System.out.println("LB = " + LB + ", UB = " + UB + ", tol = " + tol);
	}

	public void printBPIter(int _nodeId, int _LeftNodeN, double _obj, int _iinf, double _bestInteger, double _bestBound, double _itCnt, double _gap){
		System.out.println( _nodeId + "\t\t" + _LeftNodeN + "\t\t" + df.format(_obj) + "\t\t" + _iinf + "\t\t\t" + df.format(_bestInteger) + "\t\t" + df.format(_bestBound) + "\t\t" + _itCnt + "\t\t" + df.format(_gap) + "\t");
	}

	/********************************************************** Cplex **************************************************
	 ==================================================================================================================*/

	// Write Benders decomposition Solution
	public void writeBDSolution(String path,
								String status,
								double time,
								double lb,
								double ub,
								ArrayList<ArrayList<ArrayList<Integer>>> routes,
								int iter,
								int nodeN,
								ArrayList<Double> ublist
								) throws IOException {

		File directory = new File(path).getParentFile();
		if (!directory.exists()) {
			directory.mkdirs(); // 创建目录及其所有父目录
		}
		File writename = new File(path);
		writename.createNewFile();
		BufferedWriter out = new BufferedWriter(new FileWriter(writename));
		out.write(status + "\t" + time + "\t" + lb + "\t" + ub + "\t" + nodeN + "\n");
		if(routes == null){
			out.flush();
			out.close();
			return;
		}
		for(int i = 0;i < routes.size(); i++){
			ArrayList<ArrayList<Integer>> route = routes.get(i);

			if(route.size() == 0){
				continue;
			}
			for(int j = 0; j < route.size(); j++){
				ArrayList<Integer> rou = route.get(j);
				double rouCost = get_route_cost(rou);
				for(int c = 0; c < rou.size(); c++){
					if(c != rou.size()-1){
						out.write(rou.get(c) + " -> ");
					}else{
						out.write(rou.get(c) + "  cost:" + rouCost +" \n");
					}
				}
			}
		}
		out.write(iter + "\n");
		for(int i = 0; i < ublist.size(); i++){
			out.write(ublist.get(i) + " ");
		}
		out.flush();
		out.close();
	}
	// Write cplex Solution
	public void writeCplexSolution(String path,
										  String status,
										  double time,
										  double upBound,
										  double gap,
										  double lowBound,
										  ArrayList<ArrayList<ArrayList<Integer>>> routes) throws IOException {
		File writename = new File(path);
		writename.createNewFile();
		BufferedWriter out = new BufferedWriter(new FileWriter(writename));
		out.write(status + "\t" + time + "\t" + upBound  + "\t" + gap  + "\t" + lowBound + "\n");
		if(routes == null){
			out.flush();
			out.close();
			return;
		}
		for(int i = 0;i < routes.size(); i++){
			ArrayList<ArrayList<Integer>> route = routes.get(i);

			if(route.size() == 0){
				continue;
			}
			for(int j = 0; j < route.size(); j++){
				ArrayList<Integer> rou = route.get(j);
				for(int c = 0; c < rou.size(); c++){
					out.write(rou.get(c) + " ");
				}
				double routeCost = get_route_cost_group(rou);
				out.write(routeCost + "\n");
			}
		}
		out.flush();
		out.close();
	}
}
