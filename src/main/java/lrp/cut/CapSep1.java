package lrp.cut;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;

public class CapSep1 {

	/**
	 * 从大到小排列
	 */
	public class SortObject implements Comparable{
		public int id;
		public double val;

		public int compareTo(Object o){
			SortObject obj = (SortObject) o;
			if(val < obj.val)
				return 1;
			else if(val == obj.val)
				return 0;
			else
				return -1;
		}
	}

	// ------------------------------- instance information
	/**
	 * 客户点数量包含 0
	 */
	public int n;
	/**
	 * 客户需求量
	 */
	public int[] q;
	/**
	 * 车辆容量
	 */
	public int cap;
	/**
	 * 边上的流量
	 */
	public double[][] map;
	/**
	 * setCovering 中起作用的 column
	 */
	public ArrayList<ArrayList<Integer>> routes;
	/**
	 * column 对应的变量值
	 */
	public ArrayList<Double> route_v;

	// ------------------------------ parameter
	public double violate_threshhold = 0.05;
	public double shrink_threshhold = 0.5;
	public int component_remove_size = 5;

	// ------------------------------- return results
	/**
	 * 存储 S 的集合
	 */
	public ArrayList<ArrayList<Integer>> retS;
	/**
	 * 存储 S 的右端项
	 */
	public ArrayList<Integer> rightS;

	// ------------------------------- temporary data
	public int[] rec_S;
	public int[] left_comp;
	
	public CapSep1(){};

	/**
	 * 构造函数
	 * @param _n
	 * @param _q
	 * @param _cap
	 * @param _map
	 * @param _routes
	 * @param _route_v
	 */
	public CapSep1(int _n, int[] _q, int _cap, double[][] _map, ArrayList<ArrayList<Integer>> _routes, ArrayList<Double> _route_v){
		n = _n;
		q = _q;
		cap = _cap;
		map = _map;
		routes = _routes;
		route_v = _route_v;
	}

	/**
	 * 主函数
	 */
	public void solve(){
		retS = new ArrayList<>();
		rightS = new ArrayList<>();
		if(routes != null)
			route_base();
		shrink(map);
	}

	/**
	 * 1 ------------------------------------- 基于逐个合并的思想 生成 capacity cut
	 * @param y
	 */
	public void shrink(double[][] y){
		// ------------------------------- copy the map
		ArrayList<ArrayList<Double>> map = new ArrayList<>(); // 第1维是 i 点，第2维是 j 点，每个元素存储边 「i，j」对应的变量值
		ArrayList<Integer> demand = new ArrayList<>();        // 每个元素存储的是 位置 i 的需求量
		for(int i = 0; i < n; i++){
			ArrayList<Double> row = new ArrayList<>();
			for(int j = 0; j < n; j++){
				row.add(y[i][j]);
			}
			map.add(row);
			demand.add(q[i]);
		}

		ArrayList<ArrayList<Integer>> node = new ArrayList<>();
		for(int i = 0; i < n; i++){
			ArrayList<Integer> s = new ArrayList<>();
			s.add(i);
			node.add(s);
		}

		while(node.size() > 3){
			double max_y = 0.0;
			int i_id = -1;
			int j_id = -1;

			// -------------------------------- initialize the edge set
			for(int i = 1; i < node.size(); i++){
				for(int j = i + 1; j < node.size() - 1; j++){
					double v = map.get(i).get(j) + map.get(j).get(i);  // 将最大的边放进一个 S 可以 更容易的找到 S，因为将较小的 边 * 变量值 留给了做左端项
					if(v > max_y){
						max_y = v;
						i_id = i;
						j_id = j;
					}
				}
			}
			if(max_y <= shrink_threshhold)
				break;

			// ------------------------------- 将 j 放进 i 的集合
			/**
			 * 更新：map
			 * 加入：将其他点 到 j 的流量并入其他点到 i 的流量
			 * 删除：将 j 从 map 中删掉
 			 */
			for(int k = 0; k < node.size(); k++){
				if(k != i_id && k != j_id){
					map.get(k).set(i_id, map.get(k).get(i_id) + map.get(k).get(j_id));
					map.get(i_id).set(k, map.get(i_id).get(k) + map.get(j_id).get(k));
				}
			}
			for(int k = 0; k < node.size(); k++){
				map.get(k).remove(j_id);
			}
			map.remove(j_id);

			/**
			 * 更新：node set
			 * 加入：将 j 点 并入 i 的 node set
			 * 删除：将 j 从 node 中删掉
			 */
			for(int k = 0; k < node.get(j_id).size(); k++){
				node.get(i_id).add(node.get(j_id).get(k));
			}
			node.remove(j_id);

			/**
			 * 更新：demand
			 * 加入：将 j 的容量 并入 到 i 的容量中
			 * 删除：j 对应的容量
			 */
			demand.set(i_id, demand.get(i_id) + demand.get(j_id));
			demand.remove(j_id);

			// -------------------------------- check whether node i is valid cut
			int right = (int) Math.ceil( ((double)demand.get(i_id))/cap );
			double left = 0;
			for(int k = 0; k < node.size(); k++){
				if(k != i_id){
					left += map.get(k).get(i_id);   // 进入 S 和出 S 是一样的
				}
			}
			if(left + violate_threshhold < right){
				retS.add((ArrayList<Integer>) node.get(i_id).clone());
				rightS.add(right);
			}
		}
	}

	/**
	 * 2 ------------------------------------- 基于路径生成 capacity cut
	 * 基于一些可行路径，将 S 去掉 该路径中的点，加速寻找 cut
	 * 可行路径中的点 不好，好似因为一辆车就可以访问这些点，导致左侧增加的多，而右侧增加的少
	 */
	public void route_base(){

		// ------------------- get the maximum components
		ArrayList<ArrayList<Integer>> max_comps = max_comp(map);

		while(max_comps.size() > 0){
			HashSet<ArrayList<Integer>> hash = new HashSet<>();   // 避免重复加入 cut 集合
			ArrayList<Integer> comp = max_comps.get(0);
			max_comps.remove(0);

			// ------------------- find the best route
			ArrayList<SortObject> col_list = new ArrayList<>();
			for(int i = 0; i < route_v.size(); i++){
				ArrayList<Integer> route = routes.get(i);
				int in_count = 0;
				double sq = 0;
				for(int j = 1; j < route.size() - 1; j++){
					if(comp.contains(route.get(j)) == true){
						sq += q[route.get(j)];
						in_count++;
					}
				}
				if(comp.size() - in_count < 2)   // 说明：如果仅这一条路径中的点就能几乎覆盖这个集合，那么大概率这个点没用（我们想找左端项 < 右端项的）但是现在右端项 == 1，很难再小于了。
					continue;
				SortObject obj = new SortObject();
				obj.id = i;
				obj.val = route_v.get(i) - sq / cap;
				col_list.add(obj);
			}
			Collections.sort(col_list);

			// ------------------- analyze the components
			for(int k = 0; k < col_list.size(); k++){
				// tabu customer
				int[] tabu = new int[n];
				int remove_size = 0;
				ArrayList<Integer> route = routes.get(col_list.get(k).id);
				for(int i = 1; i < route.size() - 1; i++){
					tabu[route.get(i)] = 1;
				}

				// test whether comp is a valid cut
				double sq = 0;
				for(int i = 0; i < comp.size(); i++){
					if(tabu[comp.get(i)] == 0)
						sq += q[comp.get(i)];
					else
						remove_size++;
				}
				if(remove_size > component_remove_size)   // 如果路径 与 该 S 重合的点 超过 5 个 就跳出；
					break;

				int right = (int)Math.ceil(sq / cap);
				double left = 0;
				for(int i = 0; i < n; i++){
					if(tabu[i] == 1 || comp.contains(i) == false){
						for(int j = 0; j < comp.size(); j++){
							if(tabu[comp.get(j)] == 0){
								left += map[i][comp.get(j)];
							}
						}
					}
				}

				if(left + violate_threshhold < right){
					ArrayList<Integer> cut = new ArrayList<>();
					for(int i = 0; i < comp.size(); i++){
						if(tabu[comp.get(i)] == 0) {
							cut.add(comp.get(i));
						}
					}
					if(hash.contains(cut) == false){
						hash.add(cut);
						if(find_cut(cut)){
							System.out.println("++" + cut);
						}
						retS.add(cut);
						rightS.add(right);
					}
				}
			}
		}
	}

	/**
	 * 寻找最大的互相连接的点集
	 * @param y
	 * @return
	 */
	public ArrayList<ArrayList<Integer>> max_comp(double[][] y){
		ArrayList<ArrayList<Integer>> comps = new ArrayList<>();
		rec_S = new int[n];
		left_comp = new int[n];

		while(true){

			//---------------------------- initialize the set
			int start_point = -1;

			//---------------------------- find the start point
			for(int i = 1; i < n; i++){
				if(left_comp[i] == 0){
					start_point = i;
					break;
				}
			}

			if(start_point == -1)
				break;

			//---------------------------- search
			recursive(y, start_point);

			//---------------------------- add the result
			ArrayList<Integer> comp = new ArrayList<>();
			for(int i = 1; i < n; i++){
				if(rec_S[i] == 1){
					left_comp[i] = 1;
					rec_S[i] = 0;
					comp.add(i);
				}
			}
			comps.add(comp);
		}
		return comps;
	}

	/**
	 * 深度优先搜索；
	 * 将以 id 为起点的 所有的相连的点 的 rec_id[i] 置为 1
	 * @param y
	 * @param id
	 */
	public void recursive(double[][] y, int id){
		rec_S[id] = 1;
		for(int i = 1; i < n; i++){
			if( (y[id][i] > 0 || y[i][id] > 0) && rec_S[i] == 0 && left_comp[i] == 0){
				recursive(y, i);
			}
		}
	}

	// 定位cut
	public boolean find_cut(ArrayList<Integer> cut){
		ArrayList<Integer> list = new ArrayList<>(Arrays.asList(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 13));
		if(cut.equals(list)){
			return true;
		}
		return false;
	}
}
