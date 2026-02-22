package lrp.algorithm;

import ilog.concert.IloException;
import lrp.cut.CapSep1;
import lrp.cut.CapSep2;
import lrp.labelsetting.LabelSetting;
import lrp.model.SetCovering;
import lrp.data.Instance;
import java.util.ArrayList;
import java.util.Arrays;

public final class SCSolve {

	public Instance inst;

	public SCSolve(Instance _inst){
		inst = _inst;
	}

	/**
	 * sub set row cut generation
	 * @param model
	 * @param feasible_arc
	 * @param ub
	 * @throws IloException
	 */
	public void cut_solve(SetCovering model, int[][] feasible_arc, double ub) throws IloException{

		while(true){

			solve(model, feasible_arc, true);

			// 剪枝
			if((System.nanoTime() - inst.para.start_time)/1e9 > inst.para.time_limit || model.is_feasible() == false || model.lpc > ub - 1e-3)
				break;

			boolean cut_add = false;
			ArrayList<ArrayList<Integer>> routes = new ArrayList<>();
			ArrayList<Double> values = new ArrayList<>();
			for(int i = 0; i < model.active_set.size(); i++){
				routes.add(model.column_set.get(model.active_set.get(i)));
				values.add(model.active_value.get(i));
			}
			double[][] map = new double[inst.N][inst.N];    // 边 对应的变量的 值的加和
			for(int i = 0; i < routes.size(); i++){
				ArrayList<Integer> route = routes.get(i);
				for(int j = 0; j < route.size() - 1; j++){
					map[route.get(j)][route.get(j + 1)] += values.get(i);
				}
			}

			CapSep1 cap_sep1 = new CapSep1(inst.N, inst.q, inst.Q, map, routes, values);
			cap_sep1.solve();
			if(cap_sep1.retS.size() > 0){
				model.add_capcut(cap_sep1.retS, cap_sep1.rightS);
				System.out.println("cap sep1>> " + cap_sep1.retS.size());
				// debug
//				System.out.println("active_routes:");
//				for(int i = 0; i < routes.size(); i++){
//					System.out.println(routes.get(i) + "  " + values.get(i));
//				}
				System.out.println(cap_sep1.retS);
				cut_add = true;
			} else{
				CapSep2 cap_sep2 = new CapSep2(inst.N, inst.q, inst.Q, map);
				cap_sep2.solve();
				if(cap_sep2.retS.size() > 0){
					model.add_capcut(cap_sep2.retS, cap_sep2.rightS);
					System.out.println("cap sep2>> " + cap_sep2.retS.size());
					System.out.println(cap_sep2.retS);
					cut_add = true;
				}
			}

			if(cut_add)
				continue;

//			SrSep sep = new SrSep(inst.N, routes, values);
//			sep.solve(20);
//			if(sep.retS.size() > 0){
//				model.add_srcut(sep.retS);
//				System.out.println("sr cuts >>" + sep.retS.size());
//				System.out.println(sep.retS);
//				cut_add = true;
//			}

			if(cut_add == false)
				break;
		}

		/**
		 * debug
		 * // 获取根节点 的线性松弛解
		 * 		if(inst.lpc_cost < 1e-6){
		 * 			inst.lpc_cost = model.cost;
		 * 			inst.lpc_time = (System.nanoTime() - inst.start_time) / 1e9;
		 *                }
		 *
		 * 		// 获取根节点中 subset row 的数量
		 * 		if(inst.root_sr == -1){
		 * 			if(model.srs != null){
		 * 				inst.root_sr = model.srs.sr_cuts.size();
		 *            }
		 * 			else{
		 * 				inst.root_sr = 0;
		 *            }
		 *        }
		 *
		 * 		// 总的subset row 的数量
		 * 		if(model.srs != null){
		 * 			inst.total_sr += model.srs.sr_cuts.size();
		 *        }
		 */

	}

	/**
	 * column generation without dual stabilization
	 * @param model
	 * @param feasible_arc
	 * @param is_exact
	 * @return
	 * @throws IloException
	 */
	public void solve(SetCovering model, int[][] feasible_arc, boolean is_exact) throws IloException{

		double[][] amu = new double[inst.N][inst.N];

		// 列生成算法
		LabelSetting labelsetting = new LabelSetting(inst);

		model.solve();
		model.set_value();
		model.set_dual();

		int iter = 0;
		while(true){

			if((System.nanoTime() - inst.para.start_time) / 1e9 > inst.para.time_limit){
				System.out.println("column generation time out of limit !!!");
				return;
			}

			if(model.caps != null){
				for(int i = 0; i < model.caps.dual_arc.length; i++){
					amu[i] = model.caps.dual_arc[i].clone();
				}
			}

			ArrayList<ArrayList<Integer>> neg_routes = new ArrayList<>();
			ArrayList<Double> neg_costs = new ArrayList<>();
			ArrayList<Double> neg_rcs = new ArrayList<>();
			ArrayList<Double> neg_capacities = new ArrayList<>();

			// 是否含有 subset row cut
			boolean is_sr = model.is_sr;
			int[][] sr_node_set = null;
			double[][] sr_comp = null;
			if(is_sr){
				sr_node_set = model.srs.sr_node_set;
				sr_comp = model.srs.sr_comp;
			}

			// is_relax = true, 松弛版本的 labelsetting
//			System.out.println("Start solve relax...");
			labelsetting.solve(true, model.dur_mu, model.mu, amu, feasible_arc, neg_routes, neg_rcs, neg_costs, neg_capacities, model.column_hash,is_sr,sr_node_set,sr_comp);
			boolean is_all = false;
//
			if(neg_routes.size() == 0 && is_exact) {
				is_all = true;
//				System.out.println("Start solve exact...");
				// is_relax = true, 精确版本的 labelsetting
				labelsetting.solve(false, model.dur_mu, model.mu, amu, feasible_arc, neg_routes, neg_rcs, neg_costs, neg_capacities, model.column_hash,is_sr,sr_node_set,sr_comp);
			}

			// debug
//			System.out.printf("%-5s %-20s %-20s %-20s%n", "iter", "slack_value", "mu", "slack_bound");
//			for(int j = 0; j < model.slack_values.length; j++){
//				System.out.printf("%-5d %-20.10f %-20.10f %-20.10f%n", j, model.slack_values[j], model.mu[j], model.slack_bound[j]);
//			}
//			for (int j = 0; j < neg_routes.size()/1000; j++) {
//				System.out.println(neg_routes.get(j) + " " + neg_costs.get(j) + " " + neg_rcs.get(j));
//			}

			if(inst.para.isPrintLabelsetting){
				System.out.println(iter ++ + " lp cost----------------------------------------------->> " + model.lpc + "\t" + is_all + "\t" + neg_routes.size());
			}

			// debug
//			if(neg_routes.size() == 0){
//				System.out.println("active_column:");
//				for (int i = 0;i < model.active_set.size(); i++){
//					int id = model.active_set.get(i);
//					System.out.println(model.column_set.get(id)  + " " + model.column_cost.get(id) + " " + model.active_value.get(i) + " " + model.column_capacity.get(i) + " " + model.cplex.getReducedCost(model.column_vars.get(i)));
//				}
//			}

//			ArrayList<Integer> r = new ArrayList<>(Arrays.asList(0, 6, 4, 1, 2, 3, 5, 8, 0));
//			double rc = 0;
//			for(int i = 0; i < r.size() - 1; i++){
//				int id1 = r.get(i);
//				int id2 = r.get(i+1);
//				rc += inst.t[id1][id2] - model.mu[id1];
//			}
//			System.out.println(rc + "++");

			// no dual stabilization
			if(neg_routes.size() == 0)
				break;
			model.add_column(neg_routes, neg_costs, neg_capacities);
			model.solve();
			model.set_value();
			model.set_dual();

			// For dual stabilization
//			if(neg_routes.size() > 0) {
//				model.add_column(neg_routes, neg_costs, neg_capacities);
//				model.solve();
//				model.set_value();
//				model.set_dual();
//			} else if(model.is_feasible() == false){
//				boolean isFeasible = true;
//				for(int i = 0; i < model.slack_bound.length; i++){
//					if(model.slack_bound[i] > inst.para.dualStabilization){
//						isFeasible = false;
//					}
//				}
//				if(!isFeasible){
//					break;
//				}
//				model.update_slacks();
//				model.solve();
//				model.set_value();
//				model.set_dual();
//
//			} else{
//				break;
//			}

		}
	}

	// 打印原始点
	public ArrayList<Integer> getRoute(ArrayList<Integer> column){
		ArrayList<Integer> ans = new ArrayList<>();
		for(int i = 0; i < column.size(); i++){
			ans.add(inst.map[column.get(i)]);
		}
		return ans;
	}

}
