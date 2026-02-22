package lrp.algorithm;

import ilog.concert.IloException;
import lrp.model.Node;
import lrp.model.SetCovering;
import lrp.data.Instance;
import java.util.ArrayList;
import java.util.Arrays;

public final class Tree_root {

    // --------------------------------- instance information
    /**
     * instance
     */
	public Instance inst;

	// --------------------------------- Column generation results informations
	/**
	 * is feasible
	 */
	public boolean is_fea;
	/**
	 * 目标函数
	 */
	public double obj;

	public double[] mu;
	public double dur_mu;
	public double kub;

	/**
	 * 用上界信息构造一棵树
	 * @param _inst
	 */
	public Tree_root(Instance _inst){
		inst = _inst;
	}

	/**
	 * 列生成
	 */
	public void solve() throws IloException{

		// -------------------------------- 构建模型
		int[][] feasible_arc = new int[inst.N][inst.N];
		Node node = initial_node();
		SetCovering model = solve_node(node, feasible_arc);
		model.set_value();
		model.set_dual();

		// -------------------------------- 提取模型中的数据，用于生成 benders cut
		if(model.is_feasible()){
			is_fea = true;
			obj = model.lpc;
		}else{
			is_fea = false;
			obj = Double.MAX_VALUE;
		}
		mu = model.mu.clone();
		dur_mu = model.dur_mu;
		kub = node.kub;

		model.clear();
	}

	/**
	 * 初始化一个 node
	 * @return
	 */
	public Node initial_node(){
		Node node = new Node();
		node.inst = inst;
		node.klb = 0;
		node.kub = inst.para.MaxVehicleNumber;
		node.column_set = new ArrayList<>();
		node.column_cost = new ArrayList<>();
		node.column_capacity = new ArrayList<>();
		node.sudo_cost = 0;
		node.depth = 0;
		node.fid1 = new ArrayList<>();
		node.fid2 = new ArrayList<>();
		node.sr_cuts = new ArrayList<>();

		// 使用启发式算法寻在初始列
//		HSolve hSolve = new HSolve(inst);
//		hSolve.solve();
//		node.set_columns(hSolve.columns_set);

		return node;
	}

	/**
	 * 求解这个 node 对应的 RMP
	 * @param node
	 * @param feasible_arc
	 * @return
	 * @throws IloException
	 */
	public SetCovering solve_node(Node node, int[][] feasible_arc) throws IloException{

		// 所有边都可访问
		for(int i = 0; i < feasible_arc.length; i++)
			Arrays.fill(feasible_arc[i], 0);

		// 构建模型
		SetCovering model = new SetCovering();
		model.construct(inst.N, inst.D[inst.map[0]], node.klb, node.kub,false);
		model.add_column(node.column_set, node.column_cost, node.column_capacity);

		// 构建求解器
		SCSolve solver = new SCSolve(inst);

		// 更新可访问边集合
		for(int i = 0; i < node.fid1.size(); i++){
			feasible_arc[node.fid1.get(i)][node.fid2.get(i)] = -1;
		}

		solver.solve(model, feasible_arc,true);

		return model;
	}

}
