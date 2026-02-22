package lrp.model;

import ilog.concert.*;
import ilog.cplex.IloCplex;
import java.util.ArrayList;
import java.util.HashMap;

public class CapModel {

	// --------------------------- instance information & parameter
	/**
	 * 客户点数量
	 */
	public int n;

	// --------------------------- model information
	/**
	 * 需要将 cut 加入 setCovering 模型
	 */
	public IloCplex cplex;
	/**
	 * 列变量
	 */
	public ArrayList<IloNumVar> cap_column_vars;
	/**
	 * 边 对应的 columns id 集合
	 */
	public ArrayList<Integer>[][] ij2ri;
	/**
	 * capacity cut
	 */
	public ArrayList<IloRange> cap_ranges;
	/**
	 * 每个range 包含有一个slack variable
	 */
	public ArrayList<IloNumVar> cap_slack;
	/**
	 * 每个range 包含有一个slack value
	 */
	public ArrayList<Double> cap_slack_value;
	/**
	 * 记录 S 集合
	 */
	public ArrayList<ArrayList<Integer>> cap_lefts;
	/**
	 * 记录 S 对应 的 demand 的加和
	 */
	public ArrayList<Integer> cap_rights;
	/**
	 * 边（i，j）\in S^+ 对应的 capacity cut 的集合
	 */
	public ArrayList<Integer>[][] ij2cap;
	/**
	 * capacity cut 对应的对偶值
	 */
	public ArrayList<Double> cap_duals;
	/**
	 * 边 对应的 capacity cut 的对偶值
	 */
	public double[][] dual_arc;

	public CapModel(IloCplex _cplex, ArrayList<IloNumVar> _cap_columns_var, ArrayList<Integer>[][] _ij2ri, int _n){
		cplex = _cplex;
		cap_column_vars = _cap_columns_var;
		ij2ri =_ij2ri;
		n = _n;
		cap_ranges = new ArrayList<>();
		cap_slack = new ArrayList<>();
		cap_slack_value = new ArrayList<>();
		cap_lefts = new ArrayList<>();
		cap_rights = new ArrayList<>();
		ij2cap = new ArrayList[n][n];
		for(int i = 0; i < n; i++){
			for(int j = 0; j < n; j++){
				if(i == j)
					continue;
				ij2cap[i][j] = new ArrayList<>();
			}
		}
		cap_duals = new ArrayList<>();
		dual_arc = new double[n][n];
	}

	/**
	 * 根据 新列中 每条边 对应的 capacity cut 来添加
	 * @param col
	 * @param column
	 * @return
	 * @throws IloException
	 */
	public IloColumn add_column(IloColumn col, ArrayList<Integer> column) throws IloException{
		ArrayList<Integer> cc = new ArrayList<>();
		ArrayList<Integer> ccv = new ArrayList<>();
		HashMap<Integer, Integer> cc_map = new HashMap<>();
		for(int j = 0; j < column.size() - 1; j++){
			int id1 = column.get(j);
			int id2 = column.get(j+1);
			ArrayList<Integer> cuts = ij2cap[id1][id2];
			for(int k = 0; k < cuts.size(); k++){
				int cindex = cuts.get(k);
				if(cc_map.containsKey(cindex)){
					int p = cc_map.get(cindex);
					ccv.set(p, ccv.get(p) + 1);
				} else{
					cc.add(cindex);
					ccv.add(1);
					cc_map.put(cindex, cc.size() - 1);
				}
			}
		}
		for(int j = 0; j < cc.size(); j++){
			col = col.and(cplex.column(cap_ranges.get(cc.get(j)), ccv.get(j)));
		}
		return col;
	}

	/**
	 * 将 找到的 cut 加入模型
	 * @param cut
	 * @param right
	 * @throws IloException
	 */
	public void add_cut(ArrayList<Integer> cut, int right) throws IloException{
		cap_lefts.add(cut);
		cap_rights.add(right);
		int place = cap_lefts.size() - 1;
		IloRange range = cplex.addRange(right, 1e15, "CUT," + place);

		ArrayList<Integer> relative_route = new ArrayList<>();   // 记录所有存在边 （i，j）\in S^+ 的路径;
		HashMap<Integer, Integer> map = new HashMap<>();         // key: 路径id; value: 路径从集合S中出来的次数
		for(int j = 0; j < cut.size(); j++){
			for(int k = 0; k < n; k++){
				if(cut.contains(k) == false){
					ArrayList<Integer> route_set = ij2ri[cut.get(j)][k];
					for(int h = 0; h < route_set.size(); h++){
						int ri = route_set.get(h);
						if(map.containsKey(ri) == true){
							int v = map.get(ri);
							map.put(ri, v + 1);
						} else{
							relative_route.add(ri);
							map.put(ri, 1);
						}
					}
				}
			}
		}

		// 每条路径 ri 中 边 （i，j）\in S^+ 的个数 * (ri 对应的变量)
		IloNumExpr expr = cplex.numExpr(); 
		for(int j = 0; j < relative_route.size(); j++){
			int ri = relative_route.get(j);
			IloNumExpr sub_expr = cplex.numExpr(); 
			sub_expr = cplex.sum(sub_expr, cap_column_vars.get(ri));
			sub_expr = cplex.prod(sub_expr, map.get(ri));
			expr = cplex.sum(expr, sub_expr);
		}

		// add a slack
		IloNumVar slack = cplex.numVar(0.0, 1e10, IloNumVarType.Float);
		IloLinearNumExpr obj = (IloLinearNumExpr)cplex.getObjective().getExpr();
		obj.addTerm(1e6, slack);
		cplex.getObjective().setExpr(obj);
		expr = cplex.sum(expr, slack);

		range.setExpr(expr);
		cap_ranges.add(range);
		cap_slack.add(slack);

		for(int j = 0; j < cut.size(); j++){
			for(int k = 0; k < n; k++){
				if(cut.contains(k) == false){
					ij2cap[cut.get(j)][k].add(place);
				}
			}
		}

	}

	/**
	 * 1 提取每个 cut 的对偶值
	 * 2 跟新相关的边的对偶值
	 * @throws IloException
	 */
	public void get_duals() throws IloException{
		cap_duals.clear();
		for(int i = 0; i < cap_lefts.size(); i++){
			cap_duals.add(cplex.getDual(cap_ranges.get(i)));
		}
		for(int i = 0; i < n; i++){
			for(int j = 0; j < n; j++){
				dual_arc[i][j] = 0.0;
				if(i == j)
					continue;
				ArrayList<Integer> cut_index = ij2cap[i][j];
				for(int k = 0; k < cut_index.size(); k++){
					dual_arc[i][j] += cap_duals.get(cut_index.get(k));
				}
			}
		}
	}

	/**
	 * set slack variables
	 */
	public boolean set_value() throws IloException {
		cap_slack_value.clear();
		for(int i = 0; i < cap_slack.size(); i++) {
			double v = cplex.getValue(cap_slack.get(i));
			cap_slack_value.add(v);
		}

		for(int i = 0; i < cap_slack_value.size(); i++){
			if(cap_slack_value.get(i) > 1e-6){
				return false;
			}
		}
		return true;
	}

	/**
	 * 清空该类的所有存储器
	 * @throws IloException
	 */
	public void clear() throws IloException{
		for(int i = 0; i < cap_ranges.size(); i++){
			cplex.delete(cap_ranges.get(i));
		}
		cap_ranges.clear();
		cap_lefts.clear();
		cap_rights.clear();
		cap_duals.clear();
		ij2cap = null;
	}

	/**
	 * 将对偶值 小于 0 的 cut 删掉
	 * @throws IloException
	 */
	public void remove_cuts() throws IloException{
		for(int i = 0; i < cap_duals.size(); i++){
			if(cap_duals.get(i) <= 0.0){
				cplex.delete(cap_ranges.get(i));
				cap_ranges.remove(i);
				cap_rights.remove(i);
				cap_lefts.remove(i);
				cap_duals.remove(i);
				if(i < cap_slack.size()){
					cap_slack.remove(i);
				}
				if(i < cap_slack_value.size()){
					cap_slack_value.remove(i);
				}
				i--;
			}
		}
		for(int i = 0; i < n; i++){
			for(int j = 0; j < n; j++){
				if(ij2cap[i][j] != null)
					ij2cap[i][j].clear();
			}
		}
		for(int i = 0; i < cap_lefts.size(); i++){
			ArrayList<Integer> cut = cap_lefts.get(i);
			for(int j = 0; j < cut.size(); j++){
				for(int k = 0; k < n; k++){
					if(cut.contains(k) == false){
						ij2cap[cut.get(j)][k].add(i);
					}
				}
			}
		}
	}
}
