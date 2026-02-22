package lrp.algorithm;

import ilog.concert.IloException;
import lrp.branch.Ba;
import lrp.branch.Bv;
import lrp.model.Node;
import lrp.model.SetCovering;
import lrp.common.Helper;
import lrp.data.Instance;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.PriorityQueue;

public final class Tree {

    // --------------------------------- instance information
    /**
     * instance
     */
	public Instance inst;

    // --------------------------------- Branch and bound information
	/**
	 * 上界
	 */
	public double ub;
	/**
	 * 上界对应的解
	 */
	public ArrayList<ArrayList<Integer>> ub_routes;
	public boolean hasStrictFeasibleInteger;

	/**
	 * 用上界信息构造一棵树
	 * @param _inst
	 */
	public Tree(Instance _inst){
		inst = _inst;
		ub = 1e10;
		ub_routes = new ArrayList<>();
		hasStrictFeasibleInteger = false;
	}

	/**
	 * 分支定界框架
	 */
	public void solve() throws IloException{

		// 生成一个优先队列
		PriorityQueue<Node> queue = new PriorityQueue<>();

		// 初始化一个 空 node 加入优先队列
		queue.add(initial_node());

		// label 数量;
		int node_num = 0;

		// 存储边可访问情况
		int[][] feasible_arc = new int[inst.N][inst.N];

		// 最优树搜索框架
		while(queue.isEmpty() == false){

			// 取出第 1 个 node
			Node node = queue.poll();

			// 剪枝
			double sudo_cost = node.sudo_cost;
			if(node.sudo_cost >= ub && node.fid != -1){
				node.clear();
				continue;
			}

			// 求解该 node 对应的 RMP
			SetCovering model = solve_node(node, feasible_arc, ub);

			// debug
//			find_node(node,model.lpc);

			// 当求解时间超过最大时间限制，则清除该模型
			if( (System.nanoTime() - inst.para.start_time) / 1e9 > inst.para.time_limit){
   				break;
			}
			node_num++;

			if(inst.para.isPrintPBiter){
				double gap = 100 * (ub - sudo_cost) / ub;
				System.out.println( node_num + "\t" + String.format("%.2f", model.lpc) + "\t" + String.format("%.2f", sudo_cost) + "\t" + String.format("%.2f", ub) + " " + String.format("%.2f", gap) + "%\t" + queue.size());
			}

			// 剪枝
			if(model.lpc >= ub){
				model.clear();
				node.clear();
				continue;
			}

			model.set_value();

			// 更新上界，或 分支
			boolean integral = model.is_integral();
			boolean strictFeasible = model.is_feasible();

			if(integral && strictFeasible){
				hasStrictFeasibleInteger = true;
				ub = model.lpc;
				ub_routes.clear();
				for(int i = 0; i < model.active_set.size(); i++){
					ArrayList<Integer> suRoute = model.column_set.get(model.active_set.get(i));
					ArrayList<Integer> route = new ArrayList<>();
					for(int r = 0; r < suRoute.size(); r++){
						int id = inst.map[suRoute.get(r)];
						route.add(id);
					}
					route.set(route.size()-1,inst.map[inst.N]);   // TODO锟斤拷路锟斤拷 id 转锟斤拷
					ub_routes.add(route);
				}
				if(inst.para.isPrintPBiter){
					System.out.println("\t**** " + ub);
				}
			} else if(!integral) {
				Node[] nodes = branch(model, node, feasible_arc);
				if(nodes[0] != null){
					nodes[0].fid = node_num;
					nodes[0].check(true);
					queue.add(nodes[0]);
				}
				if(nodes[1] != null){
					nodes[1].fid = node_num;
					nodes[1].check(true);
					queue.add(nodes[1]);
				}
			}
			model.clear();
		}
		inst.para.node_num_bp = node_num;
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
		node.depth = 0;
		node.fid1 = new ArrayList<>();
		node.fid2 = new ArrayList<>();
		node.sr_cuts = new ArrayList<>();
		node.cap_cuts = new ArrayList<>();

		// 使用启发式算法寻在初始列
//		HSolve hSolve = new HSolve(inst);
//		hSolve.solve();
//		node.sudo_cost = hSolve.obj;
//		node.set_columns(hSolve.columns_set);
//
//		ub = node.sudo_cost;
//		for(int i = 0; i < hSolve.columns_set.size(); i++){
//			ArrayList<Integer> suRoute = hSolve.columns_set.get(i);
//			ArrayList<Integer> route = new ArrayList<>();
//			for(int r = 0; r < suRoute.size(); r++){
//				int id = inst.map[suRoute.get(r)];
//				route.add(id);
//			}
//			route.set(route.size()-1,inst.map[inst.N]);   // TODO：路径 id 转换
//			ub_routes.add(route);
//		}

		// debug
//		System.out.println(node.sudo_cost);
//		for(int i = 0; i < hSolve.columns_set.size(); i++){
//			System.out.println(hSolve.columns_set.get(i));
//		}
//
//		// check 启发式穿过来的初始列成本是否正确（下标问题）
//		Helper helper = new Helper(inst);
//		double tcost = 0;
//		for(int i = 0; i < hSolve.columns_set.size(); i++){
//			tcost += helper.get_route_cost(hSolve.columns_set.get(i));
//		}
//		System.out.println(tcost);
//		System.exit(0);

		return node;
	}

	/**
	 * 分支过程
	 * @param model
	 * @param node
	 * @param feasible_arc
	 * @return
	 * @throws IloException
	 */
	public Node[] branch(SetCovering model, Node node, int[][] feasible_arc) throws IloException{
		Node[] ret = new Node[2];
		Bv bv = new Bv(inst);
		if(inst.para.isPrintPBiter){
			System.out.println("branching on vehicle >>>");
		}
		if(bv.branch(model, node, feasible_arc)){
			if(inst.para.isPrintPBiter){
				System.out.println("branching on vehicle >>>" + bv.fv);
			}
			ret[0] = bv.left_node;
			ret[1] = bv.right_node;
			return ret;
		}
		if(inst.para.isPrintPBiter){
			System.out.println("branching on arc >>>");
		}
		Ba ba = new Ba(inst);
		ba.branch(model, node, feasible_arc);
		ret[0] = ba.left_node;
		ret[1] = ba.right_node;
		if(inst.para.isPrintPBiter){
			System.out.println("branching on arc>>>>" + ba.id1 + " " + ba.id2 + " " + ba.av);
		}
		return ret;
	}

	/**
	 * 求解这个 node 对应的 RMP
	 * @param node
	 * @param feasible_arc
	 * @param ub
	 * @return
	 * @throws IloException
	 */
	public SetCovering solve_node(Node node, int[][] feasible_arc, double ub) throws IloException{

		// 所有边都可访问
		for(int i = 0; i < feasible_arc.length; i++)
			Arrays.fill(feasible_arc[i], 0);

		// 构建模型
		SetCovering model = new SetCovering();
		model.construct(inst.N, inst.D[inst.map[0]], node.klb, node.kub,false);
		model.add_column(node.column_set, node.column_cost, node.column_capacity);

		if(node.cap_cuts.size() > 0){
			ArrayList<Integer> rights = new ArrayList<>();
			for(int i = 0; i < node.cap_cuts.size(); i++){
				ArrayList<Integer> cut = node.cap_cuts.get(i);
				double sq = 0;
				for(int k = 0; k < cut.size(); k++){
					int id = cut.get(k);
					sq += inst.q[id];
				}
				rights.add((int)Math.ceil(sq / inst.Q));
			}
			model.add_capcut(node.cap_cuts, rights);
		}

		if(node.sr_cuts.size() > 0){
			model.add_srcut(node.sr_cuts);
		}

		// 构建求解器
		SCSolve solver = new SCSolve(inst);

		// 更新可访问边集合
		for(int i = 0; i < node.fid1.size(); i++){
			feasible_arc[node.fid1.get(i)][node.fid2.get(i)] = -1;
		}

		// 求解
		if(node.fid == -1)
			solver.solve(model, feasible_arc, true);
		else
			solver.cut_solve(model, feasible_arc, ub);
		return model;
	}

	/**
	 * 判断该 node 是否满足
	 */
	public void find_node(Node node, double _lpc){

		int[][] solution = new int[][]{
				{0, 9, 7, 5, 2, 1, 6, 0},
				{0, 10, 8, 3, 4, 11, 13, 0},
				{0, 14, 12, 15, 0}
		};

		double cple_obj = 36888;

		boolean ans = true;
		for(int i = 0; i < node.fid1.size(); i++){
			int id1 = node.fid1.get(i);
			int id2 = node.fid2.get(i);
			for(int j = 0; j < solution.length; j++){
				for(int k = 0; k < solution[j].length - 1; k++){
					if(solution[j][k] == id1 && solution[j][k+1] == id2){
						ans = false;
					}
				}
			}
		}

		if(ans){
				if(_lpc > cple_obj){
					System.out.println(" node error !! ");
					System.out.println(node.fid1);
					System.out.println(node.fid2);
					throw new IllegalStateException("debug node bound check failed");
				}
			}
		}

}
