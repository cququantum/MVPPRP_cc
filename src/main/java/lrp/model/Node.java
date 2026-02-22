package lrp.model;

import ilog.concert.IloException;
import lrp.data.Instance;
import java.util.ArrayList;

public class Node implements Comparable<Node>{

    // --------------------------------- instance information
    /**
     * instance 数据
     */
    public Instance inst;

    // --------------------------------- node information
    /**
     * node id 编号
     */
    public int fid = -1;
    /**
     * 松弛解目标函数
     */
    public double sudo_cost;
    /**
     * node 的深度
     */
    public int depth;
    /**
     * 车辆数下界
     */
    public int klb;
    /**
     * 车辆数上界
     */
    public int kub;
    /**
     * Routes that feasible Solution used in branch
     */
    public ArrayList<ArrayList<Integer>> column_set;
    /**
     * Routes cost
     */
    public ArrayList<Double> column_cost;
    /**
     * Routes capacity
     */
    public ArrayList<Double> column_capacity;
    /**
     * 禁止边 start vertex
     */
    public ArrayList<Integer> fid1;
    /**
     * 禁止边 end vertex
     */
    public ArrayList<Integer> fid2;
    /**
     * sub set row cuts 集合
     */
    public ArrayList<ArrayList<Integer>> sr_cuts;

    /**
     * capacity cut 集合
     */
    public ArrayList<ArrayList<Integer>> cap_cuts;

    public Node(){}

    public Node duplicate() {
        Node new_node = new Node();
        new_node.inst = inst;

        new_node.fid = fid;
        new_node.depth = depth;
        new_node.sudo_cost = sudo_cost;
        new_node.klb = klb;
        new_node.kub = kub;

        new_node.column_set = null;
        new_node.column_cost = null;
        new_node.column_capacity = null;
        new_node.fid1 = new ArrayList<>();
        new_node.fid2 = new ArrayList<>();
        for(int i = 0; i < fid1.size(); i++){
            new_node.fid1.add(fid1.get(i));
            new_node.fid2.add(fid2.get(i));
        }

        new_node.sr_cuts = new ArrayList<>();
        for(int i = 0; i < sr_cuts.size(); i++){
            new_node.sr_cuts.add((ArrayList<Integer>) sr_cuts.get(i).clone());
        }

        new_node.cap_cuts = new ArrayList<>();
        for(int i = 0; i < cap_cuts.size(); i++){
            new_node.cap_cuts.add((ArrayList<Integer>) cap_cuts.get(i).clone());
        }

        return new_node;
    }

    /**
     * 1，车辆数上下限；
     * 2，被禁止的边是否仍然被使用；
     * 3，判断使用当前的列是否是可解的；
     */
    public void check(boolean solve) throws IloException {

        // check klb & lub
        if(klb > kub){
            System.out.println("incompatible vehicle bound>>>" + klb + " " + kub);
            System.exit(0);
        }

		// check infeasible arc
		boolean[][] forbid = new boolean[inst.N+1][inst.N+1];
		for(int i = 0; i < fid1.size(); i++){
			forbid[fid1.get(i)][fid2.get(i)] = true;
		}
		for(int i = 0; i < column_set.size(); i++){
			ArrayList<Integer> column = column_set.get(i);
			for(int j = 0; j < column.size() - 1; j++){
				int id1 = column.get(j);
				int id2 = column.get(j + 1);
				if(forbid[id1][id2]){
					System.out.println("forbid arc>>>" + id1 + " " + id2 + " " + column);
					System.exit(0);
				}
			}
		}

		// check the solvability
		if(solve){
			SetCovering model = new SetCovering();
			model.construct(inst.N, inst.D[inst.map[0]], klb, kub,false);
			model.add_column(column_set, column_cost, column_capacity);
			if(model.cplex.solve() == false){
				System.out.println("node solvable error");
				System.exit(0);
			}
		}
    }

    public void clear() {
        // basic information
        if(column_set != null)
            column_set.clear();
        if(column_cost != null)
            column_cost.clear();
        if(column_capacity != null)
            column_capacity.clear();
        fid1.clear();
        fid2.clear();
        sr_cuts.clear();
        cap_cuts.clear();
    }

    // 给 column 赋值
    public void set_columns(ArrayList< ArrayList<Integer> > _column_set){
        column_set = new ArrayList<>();
        column_cost = new ArrayList<>();
        column_capacity = new ArrayList<>();
        for(int i = 0 ; i < _column_set.size(); i++){

            ArrayList<Integer> tempR = (ArrayList<Integer>) _column_set.get(i).clone();
            column_set.add(tempR);

            double tempC = get_route_cost(tempR);
            column_cost.add(tempC);

            double temp_capacity = get_route_capacity(tempR);
            column_capacity.add(temp_capacity);
        }
    }

    // Get one route cost
    public double get_route_cost(ArrayList<Integer> route) {
        double cost = 0;
        if(route.size() == 2){
            return 0;
        }
        for(int i = 1; i < route.size(); ++i) {
            cost += inst.t[route.get(i-1)][route.get(i)] + inst.s[route.get(i)];
        }
        return cost;
    }

    // Get one route capacity
    public double get_route_capacity(ArrayList<Integer> route) {
        double cost = 0;
        if(route.size() == 2){
            return 0;
        }
        for(int i = 1; i < route.size() - 1; ++i) {
            cost += inst.q[route.get(i)];
        }
        return cost;
    }

    @Override
    public int compareTo(Node node) {
        if(sudo_cost < node.sudo_cost) {
            return -1;
        } else if(sudo_cost > node.sudo_cost) {
            return 1;
        }
        return 0;
    }

}

