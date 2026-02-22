package lrp.branch;

import ilog.concert.IloException;
import ilog.concert.IloNumVar;
import lrp.algorithm.SCSolve;
import lrp.model.Node;
import lrp.model.SetCovering;
import lrp.data.Instance;
import java.util.ArrayList;

public final class Ba {

	// --------------------------------- instance information
	/**
	 * 数据
	 */
	public Instance inst;
	/**
	 * 找离 0.5 最近的
	 */
	public double thresh_hold;

	// --------------------------------- 基于边分支的 information
	/**
	 * 分支边的取值
	 */
	public double av;
	/**
	 * 强分支，选择对目标函数提升最大的变量作为分支
	 */
	public double max_value;
	/**
	 * 左子树
	 */
	public Node left_node;
	/**
	 * 右子树
	 */
	public Node right_node;

	/**
	 * 分支边的 start node
	 */
	public int id1;
	/**
	 * 分支边的 end node
	 */
	public int id2;
	/**
	 * 左子树的目标函数
	 */
	public double left_obj;
	/**
	 * 右子树的目标函数
	 */
	public double right_obj;
	/**
	 * left node 的列集合
	 */
	public ArrayList<ArrayList<Integer>> left_column_set;
	/**
	 * left node 列 对应的 cost 集合
	 */
	public ArrayList<Double> left_column_cost;
	/**
	 * left node 列 对应的 capacity 集合
	 */
	public ArrayList<Double> left_column_capacity;
	/**
	 *  right node 的列集合
	 */
	public ArrayList<ArrayList<Integer>> right_column_set;
	/**
	 * right node 列 对应的 cost 集合
	 */
	public ArrayList<Double> right_column_cost;
	/**
	 * right node 列 对应的 capacity 集合
	 */
	public ArrayList<Double> right_column_capacity;

	/**
	 * 构造函数
	 * @param _inst
	 */
	public Ba(Instance _inst){
		inst = _inst;
	}

	/**
	 * 主函数
	 * @param model
	 * @param node
	 * @param feasible_arc
	 * @return
	 * @throws IloException
	 */
	public boolean branch(SetCovering model, Node node, int[][] feasible_arc) throws IloException{
		// 计算 每个边的 变量值
		double[][] map = new double[inst.N+1][inst.N+1];
		for(int i = 0; i < model.active_set.size(); i++){
			ArrayList<Integer> route = model.column_set.get(model.active_set.get(i));
			double value = model.active_value.get(i);
			for(int j = 0; j < route.size() - 1; j++){
				map[route.get(j)][route.get(j + 1)] += value;
			}
		}

		// 选择一个效果最好的边进行分支
		if(select(model, feasible_arc, map) == false){
			return false;
		}

		left_node = node;
		right_node = node.duplicate();
		left_node.sudo_cost = model.lpc;
		right_node.sudo_cost = model.lpc;
		left_node.depth ++;
		right_node.depth = left_node.depth;
		left_node.sr_cuts.clear();
		right_node.sr_cuts.clear();
		left_node.cap_cuts.clear();
		right_node.cap_cuts.clear();
		if(model.caps != null){
			for(int i = 0; i < model.caps.cap_lefts.size(); i++){
				left_node.cap_cuts.add(new ArrayList<>(model.caps.cap_lefts.get(i)));
				right_node.cap_cuts.add(new ArrayList<>(model.caps.cap_lefts.get(i)));
			}
		}
		if(model.srs != null){
			for(int i = 0; i < model.srs.sr_cuts.size(); i++){
				left_node.sr_cuts.add(new ArrayList<>(model.srs.sr_cuts.get(i)));
				right_node.sr_cuts.add(new ArrayList<>(model.srs.sr_cuts.get(i)));
			}
		}

		// -------------------------- branch on root

		left_node.fid1.add(id1);
		left_node.fid2.add(id2);
		if(id1 < inst.N && id1 > 0){
			for(int i = 0; i < inst.N; i++){
				if(i != id2){
					right_node.fid1.add(id1);
					right_node.fid2.add(i);
				}
			}
		}
		if(id2 < inst.N && id2 > 0){
			for(int i = 0; i < inst.N; i++){
				if(i != id1){
					right_node.fid1.add(i);
					right_node.fid2.add(id2);
				}
			}
		}

		// -------------------------- get the left route set and the right route set
		left_node.column_set = left_column_set;
		left_node.column_cost = left_column_cost;
		left_node.column_capacity = left_column_capacity;
		right_node.column_set = right_column_set;
		right_node.column_cost = right_column_cost;
		right_node.column_capacity = right_column_capacity;
		return true;
	}

	/**
	 * 选择一个最好的分支
	 * @param model
	 * @param feasible_arc
	 * @param map
	 * @return
	 * @throws IloException
	 */
	public boolean select(SetCovering model, int[][] feasible_arc, double[][] map) throws IloException{
		id1 = -1;
		id2 = -1;
		SCSolve scSolve = new SCSolve(inst);

		double o_cost = model.lpc;
		int[][] save = new int[inst.N][inst.N];

//		System.out.println("o_cost: " + o_cost);
//		System.out.println("active_column:");
//		for (int i = 0;i < model.active_set.size(); i++){
//			int id = model.active_set.get(i);
//			System.out.println(model.column_set.get(id)  + " " + model.column_cost.get(id) + " " + model.active_value.get(i) + " " + model.column_capacity.get(i));
//		}

		// compute change cost
		double[][] left_cost = new double[inst.N][inst.N];
		double[][] right_cost = new double[inst.N][inst.N];
		ArrayList<ArrayList<ArrayList<ArrayList<Integer>>>> left_matrix = new ArrayList<>();
		ArrayList<ArrayList<ArrayList<Double>>> l_costs = new ArrayList<>();
		ArrayList<ArrayList<ArrayList<Double>>> l_capacities = new ArrayList<>();
		ArrayList<ArrayList<ArrayList<ArrayList<Integer>>>> right_matrix = new ArrayList<>();
		ArrayList<ArrayList<ArrayList<Double>>> r_costs = new ArrayList<>();
		ArrayList<ArrayList<ArrayList<Double>>> r_capacities = new ArrayList<>();
		for(int i = 0; i < inst.N; i++){
			left_matrix.add(new ArrayList<>());
			right_matrix.add(new ArrayList<>());
			l_costs.add(new ArrayList<>());
			r_costs.add(new ArrayList<>());
			l_capacities.add(new ArrayList<>());
			r_capacities.add(new ArrayList<>());
			for(int j = 0; j < inst.N; j++){
				left_matrix.get(i).add(new ArrayList<>());
				right_matrix.get(i).add(new ArrayList<>());
				l_costs.get(i).add(new ArrayList<>());
				r_costs.get(i).add(new ArrayList<>());
				l_capacities.get(i).add(new ArrayList<>());
				r_capacities.get(i).add(new ArrayList<>());
			}
		}

		for(int i = 0; i < inst.N; i++){
			for(int j = 0; j < inst.N; j++){
				if(i == j || frac_cost(map[i][j]) > 0.4)
					continue;

				// modify the lp
				double lobj;
				double robj;
				ArrayList<ArrayList<Integer>> l_column_set = new ArrayList<>();
				ArrayList<Double> l_column_cost = new ArrayList<>();
				ArrayList<Double> l_column_capacity = new ArrayList<>();
				ArrayList<ArrayList<Integer>> r_column_set = new ArrayList<>();
				ArrayList<Double> r_column_cost = new ArrayList<>();
				ArrayList<Double> r_column_capacity = new ArrayList<>();

				// ---------------------------- 更新左子树
				left_modify(model, feasible_arc, i, j);
				scSolve.solve(model, feasible_arc, false);
				model.set_value();
				lobj = model.lpc;
				// debug
//				System.out.println("left : " + lobj);
//				System.out.println("active_column:" + " ["+i+","+j+"]");
//				for (int k = 0;k < model.active_set.size(); k++){
//					int id = model.active_set.get(k);
//					System.out.println(model.column_set.get(id)  + " " + model.column_cost.get(id) + " " + model.active_value.get(k) + " " + model.column_capacity.get(k));
//				}

				model.get_columns(l_column_set, l_column_cost, l_column_capacity);
				left_restore(model, feasible_arc, i, j);


				// ---------------------------- 更新右子树
				right_modify(model, feasible_arc, save, i, j);
				scSolve.solve(model, feasible_arc, false);
				model.set_value();
				robj = model.lpc;
				//debug
//				System.out.println("right : " + robj);
//				System.out.println("active_column:"+ " ["+i+","+j+"]");
//				for (int k = 0;k < model.active_set.size(); k++){
//					int id = model.active_set.get(k);
//					System.out.println(model.column_set.get(id)  + " " + model.column_cost.get(id) + " " + model.active_value.get(k) + " " + model.column_capacity.get(k));
//				}

				model.get_columns(r_column_set, r_column_cost, r_column_capacity);
				right_restore(model, feasible_arc, save, i, j);

				left_cost[i][j] = lobj;
				left_matrix.get(i).set(j, l_column_set);
				l_costs.get(i).set(j, l_column_cost);
				l_capacities.get(i).set(j, l_column_capacity);

				right_cost[i][j] = robj;
				right_matrix.get(i).set(j, r_column_set);
				r_costs.get(i).set(j, r_column_cost);
				r_capacities.get(i).set(j, r_column_capacity);
			}
		}

		// select the arcs
		max_value = 0;
		thresh_hold = 0.1;
		while(max_value < 1e-1 && thresh_hold > -1e-6){
			for(int i = 0; i < inst.N; i++){
				for(int j = 0; j < inst.N; j++){
					if(i == j || frac_cost(map[i][j]) > 0.4){
//						System.out.println( " frac_cost(map[i][j])" +  frac_cost(map[i][j]) );
						continue;
					}
					if(left_cost[i][j] - o_cost < thresh_hold){
//						System.out.println((left_cost[i][j] - o_cost) + " -- " + thresh_hold);
						continue;
					}
					if(right_cost[i][j] - o_cost < thresh_hold){
//						System.out.println((right_cost[i][j] - o_cost) + " + " + thresh_hold);
						continue;
					}
					if(inst.para.isPrintBranchArc){
//						System.out.println(i + "," + j + "," + left_cost[i][j] + "," + right_cost[i][j] + "," + map[i][j]);
					}
					if(left_cost[i][j] + right_cost[i][j] > max_value){
						max_value = left_cost[i][j] + right_cost[i][j];
						id1 = i;
						id2 = j;
						av = map[i][j];
						left_obj = left_cost[i][j];
						right_obj = right_cost[i][j];
						left_column_set = left_matrix.get(i).get(j);
						left_column_cost = l_costs.get(i).get(j);
						left_column_capacity = l_capacities.get(i).get(j);
						right_column_set = right_matrix.get(i).get(j);
						right_column_cost = r_costs.get(i).get(j);
						right_column_capacity = r_capacities.get(i).get(j);
					}
				}
			}
			thresh_hold -= 0.02;
			if(inst.para.isPrintBranchArc){
				System.out.println("thresh_hold value * " + thresh_hold);
			}
		}
		model.lpc = o_cost;

		if(max_value < 1e-1)
			return false;
		else
			return true;
	}

	/**
	 * 更新左子树：将包含 边（id1 id2）的变量 置为 0
	 * 更新 feasible_arc[id1][id2] = -1;
	 * @param model
	 * @param feasible_arc
	 * @param id1
	 * @param id2
	 * @throws IloException
	 */
	public void left_modify(SetCovering model, int[][] feasible_arc, int id1, int id2) throws IloException{
		ArrayList<Integer> zero_set = model.ij2ri[id1][id2];
		for(int i = 0; i < zero_set.size(); i++){
			IloNumVar var = model.column_vars.get(zero_set.get(i));
			var.setUB(0.0);
		}
		feasible_arc[id1][id2] = -1;
	}

	/**
	 * 将 left node 禁止的边 复原
	 * @param model
	 * @param feasible_arc
	 * @param id1
	 * @param id2
	 * @throws IloException
	 */
	public void left_restore(SetCovering model, int[][] feasible_arc, int id1, int id2) throws IloException{
		ArrayList<Integer> zero_set = model.ij2ri[id1][id2];
		for(int i = 0; i < zero_set.size(); i++){
			IloNumVar var = model.column_vars.get(zero_set.get(i));
			var.setUB(1e10);
		}
		feasible_arc[id1][id2] = 0;
	}

	/**
	 * 更新右子树：将只包含 点 id1 和 id2 但是不是边（id1 id2）的变量 置为 0
	 * @param model
	 * @param feasible_arc
	 * @param save
	 * @param id1
	 * @param id2
	 * @throws IloException
	 */
	public void right_modify(SetCovering model, int[][] feasible_arc, int[][] save, int id1, int id2) throws IloException{
		if(id1 < inst.N && id1 > 0){
			for(int i = 0; i < inst.N; i++){
				if(i != id2){
					save[id1][i] = feasible_arc[id1][i];
					feasible_arc[id1][i] = -1;
					ArrayList<Integer> zero_set = model.ij2ri[id1][i];
					for(int j = 0; j < zero_set.size(); j++){
						IloNumVar var = model.column_vars.get(zero_set.get(j));
						var.setUB(0.0);
					}
				}
			}
		}
		if(id2 < inst.N && id2 > 0){
			for(int i = 0; i < inst.N; i++){
				if(i != id1){
					save[i][id2] = feasible_arc[i][id2];
					feasible_arc[i][id2] = -1;
					ArrayList<Integer> zero_set = model.ij2ri[i][id2];
					for(int j = 0; j < zero_set.size(); j++){
						IloNumVar var = model.column_vars.get(zero_set.get(j));
						var.setUB(0.0);
					}
				}
			}
		}
	}

	/**
	 * 将 right node 禁止的边 复原
	 * @param model
	 * @param feasible_arc
	 * @param save
	 * @param id1
	 * @param id2
	 * @throws IloException
	 */
	public void right_restore(SetCovering model, int[][] feasible_arc, int[][] save, int id1, int id2) throws IloException{
		if(id1 < inst.N && id1 > 0){
			for(int i = 0; i < inst.N; i++){
				if(i != id2){
					feasible_arc[id1][i] = save[id1][i];
					if(feasible_arc[id1][i] >= 0){
						ArrayList<Integer> one_set = model.ij2ri[id1][i];
						for(int j = 0; j < one_set.size(); j++){
							IloNumVar var = model.column_vars.get(one_set.get(j));
							var.setUB(1e10);
						}
					}
				}
			}
		}
		if(id2 < inst.N && id2 > 0){
			for(int i = 0; i < inst.N; i++){
				if(i != id1){
					feasible_arc[i][id2] = save[i][id2];
					if(feasible_arc[i][id2] >= 0){
						ArrayList<Integer> one_set = model.ij2ri[i][id2];
						for(int j = 0; j < one_set.size(); j++){
							IloNumVar var = model.column_vars.get(one_set.get(j));
							var.setUB(1e10);
						}
					}
				}
			}
		}
	}

	/**
	 * 距离 0.5 的距离
	 * @param v
	 * @return
	 */
	public static double frac_cost(double v){
		double line = (int)(v) + 0.5;
		if(v >= line)
			return v - line;
		else
			return line - v;
	}
}
