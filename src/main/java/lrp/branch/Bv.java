package lrp.branch;

import ilog.concert.IloException;
import lrp.algorithm.SCSolve;
import lrp.model.Node;
import lrp.model.SetCovering;
import lrp.data.Instance;
import java.util.ArrayList;

public final class Bv {

	// --------------------------------- instance information
	/**
	 * 算例数据
	 */
	public Instance inst;
	/**
	 * 精确度，判断是否为整数
	 */
	public static double tolerance = 1e-5;

	// --------------------------------- 基于车辆数分支 information
	/**
	 * 是用的车辆数，每个列（变量）当前取值的加和
	 */
	public double fv;
	/**
	 * 左子树
	 */
	public Node left_node;
	/**
	 * 右子树
	 */
	public Node right_node;

	/**
	 * 按照车辆数分支的构造函数
	 * @param _inst
	 */
	public Bv(Instance _inst){
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

		// 车辆数为整数
		if(fraction_v(model) == false)
			return false;

		left_node = node;
		right_node = node.duplicate();

		left_node.depth ++;
		right_node.depth = left_node.depth;

		left_node.sudo_cost = model.lpc;
		right_node.sudo_cost = model.lpc;

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
				ArrayList<Integer> cut = model.srs.sr_cuts.get(i);
				left_node.sr_cuts.add((ArrayList<Integer>) cut.clone());
				right_node.sr_cuts.add((ArrayList<Integer>) cut.clone());
			}
		}

		//------------------------- branch on root

		// change the direction of the branching arc
		left_node.kub = (int) Math.floor(fv);
		right_node.klb = (int) Math.ceil(fv);

		System.out.println(left_node.kub + " " + right_node.klb);

		//------------------------- get the left route set and the right route set
		left_node.column_set = new ArrayList<>();
		left_node.column_cost = new ArrayList<>();
		left_node.column_capacity = new ArrayList<>();

		right_node.column_set = new ArrayList<>();
		right_node.column_cost = new ArrayList<>();
		right_node.column_capacity = new ArrayList<>();

		boolean ret = get_columns(model, feasible_arc, left_node.klb, left_node.kub, left_node.column_set, left_node.column_cost, left_node.column_capacity);
		if(ret == false){
			left_node = null;
		}

		ret = get_columns(model, feasible_arc, right_node.klb, right_node.kub, right_node.column_set, right_node.column_cost,right_node.column_capacity);
		if(ret == false){
			right_node = null;
		}

		return true;
	}

	/**
	 * 获取列，当松弛变量存在的时候，返回false，说明不可行
	 * @param model
	 * @param feasible_arc
	 * @param klb
	 * @param kub
	 * @param column_set
	 * @param column_cost
	 * @return
	 * @throws IloException
	 */
	public boolean get_columns(SetCovering model, int[][] feasible_arc, int klb, int kub, ArrayList<ArrayList<Integer>> column_set, ArrayList<Double> column_cost, ArrayList<Double> column_capacity) throws IloException{
		double lb = model.ranges[0].getLB();
		double ub = model.ranges[0].getUB();
		SCSolve solve = new SCSolve(inst);
		model.ranges[0].setBounds(klb, kub);
		solve.solve(model, feasible_arc, false); // TODO: false 会不会导致不可行，它属于强分支中的一个可能
		model.set_value();
		boolean ret = model.is_feasible();
		model.get_columns(column_set, column_cost,column_capacity);
		model.ranges[0].setBounds(lb, ub);
		return ret;
	}

	/**
	 * 当车辆数为整数或者接近整数的时候跳过
	 * @param model
	 * @return
	 */
	public boolean fraction_v(SetCovering model){
		fv = 0;
		for(int i = 0; i < model.active_value.size(); i++){
			fv += model.active_value.get(i);
		}

		if(is_fractional(fv) == false)
			return false;
		if(frac_cost(fv) > 0.4)
			return false;
		return true;
	}

	/**
	 * 判断 v 是否为分数
	 * @param v
	 * @return
	 */
	public static boolean is_fractional(double v){
		if( v > (int) v + tolerance && v < (int) v + 1 - tolerance)
			return true;
		else
			return false;
	}

	/**
	 * 到 x.5 的距离
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
