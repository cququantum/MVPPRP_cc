package lrp.labelsetting;

import lrp.data.Instance;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;

public class LabelSetting {

	// --------------------------------- instance information
	public Instance inst;
	public int target_size;
	public int ng_size;
	public boolean is_relax;

	/**
	 * 位置索引对应客户；
	 * 每个元素是一个64位的long型变量；
	 * 这里将这个long型的变量中，二进制形式下，该客户点的位置变成 1
	 * 当客户数量超过，64位的时候，就需要用两个数组（mkII）
	 */
	public long[] maskI;
	public long[] maskII;
	/**
	 * capVI[i]: 存储了 i 客户点 容量允许的 可到达的点的集合
	 */
	public long[] capVI;
	public long[] capVII;

	//--------------------------------------------- 模型传递过来的数据
	/**
	 * columns in the SetCovering model
	 */
	public HashSet<ArrayList<Integer>> extra_hash;
	/**
	 * dur_mu
	 */
	public double dur_mu;
	/**
	 * 对偶变量的值
	 */
	public double[] mu;
	/**
	 * 边的对偶变量
	 */
	public double[][] amu;
	/**
	 * 禁止的边
	 */
	public int[][] fc;
	/**
	 * 是否包含 subset-row inequalities
	 */
	public boolean is_sr;
	/**
	 * SrModel参数
	 * 第一维：客户点id;
	 * 第二维：segment_id;
	 * 存储的值是一个整数，这个整数对应的二进制形式记录了所有 该客户点 所在的subset row cut id;
	 */
	public int[][] sr_node_set;
	/**
	 * SrModel参数
	 * 第一维：segment_id;
	 * 第二维：每个位置就是该位置的下标值；
	 * sr_comp[0][1] 记录的是第 0 个 Segment 中
	 * "1"这个整数对应的2进制形式，包含的所有的subset_row cut 对偶值的和
	 */
	public double[][] sr_comp;

	// --------------------------------- algorithm changing data
	/**
	 * ng set
	 */
	public long[] ngVI;
	public long[] ngVII;
	/**
	 *  forward un_extended labels
	 */
	public ArrayList< ArrayList<Label>> FUL;
	/**
	 * forward extended labels
	 */
	public ArrayList< ArrayList<Label>> FTL;
	/**
	 * backward un_extended labels
	 */
	public ArrayList< ArrayList<Label>> BUL;
	/**
	 * backward extended labels
	 */
	public ArrayList< ArrayList<Label>> BTL;

	// --------------------------------- bounding
	public boolean is_bound_active = true;
	/**
	 * 当路径是ng route的时候，计算bound
	 */
	public CapTwoCycleFree bound;
	/**
	 * 当 路径是 elementary 的时候，计算bound
	 */
//	public ExactBounding e_bound;

	public LabelSetting(Instance _inst){
		inst  = _inst;
		target_size = 500;
		ng_size = 5;

		maskI = new long[inst.N];
		maskII = new long[inst.N];
		long mask = 0x01;
		for(int i = 0; i < inst.N; i++){
			if(i < 64){
				maskI[i] = mask;
			} else{
				maskII[i] = mask;
			}
			if(i == 63)
				mask = 0x01;
			else
				mask *= 2;
		}
		capVI = new long[inst.Q + 1];
		capVII = new long[inst.Q + 1];
		for(int i = 0; i <= inst.Q; i++){
			for(int j = 0; j < inst.N; j++){
				if(i + inst.q[j] > inst.Q){
					capVI[i] = capVI[i] | maskI[j];
					capVII[i] = capVII[i] | maskII[j];
				}
			}
		}

		ngVI = new long[inst.N];
		ngVII = new long[inst.N];
		if(ng_size > inst.N-1)
			ng_size = inst.N-1;
	}

	public void solve(boolean _is_relax, double _dur_mu, double[] _mu, double[][] _amu, int[][] _fc,ArrayList<ArrayList<Integer>> columns, ArrayList<Double> reduce_costs, ArrayList<Double> costs, ArrayList<Double> capacities, HashSet<ArrayList<Integer>> _extra_hash, boolean _is_sr, int[][] _sr_node_set, double[][] _sr_comp){
		is_relax = _is_relax;
		dur_mu = _dur_mu;
		mu = _mu;
		amu = _amu;
		fc = _fc;
		extra_hash = _extra_hash;
		is_sr = _is_sr;
		sr_node_set = _sr_node_set;
		sr_comp = _sr_comp;

		// --------------------------------- bounding
		if(is_bound_active){
			bound = new CapTwoCycleFree(inst,fc);
//			e_bound = new ExactBounding(inst,fc);
		}

		HashSet<ArrayList<Integer>> route_hash = new HashSet<>();
		ArrayList<ArrayList<Integer>> gen_routes = new ArrayList<>();
		ArrayList<Double> gen_rcs = new ArrayList<>();
		ArrayList<Double> gen_costs = new ArrayList<>();
		ArrayList<Double> gen_capacities = new ArrayList<>();

		initialize_algorithm();
		while( (System.nanoTime() - inst.para.start_time) / 1e9 < inst.para.time_limit){

			double step_size;
			double f_limit;
			double b_limit;

			if(is_relax)
				step_size = inst.Q / 6.0;
			else
				step_size = inst.Q / 15.0;
			f_limit = step_size;
			b_limit = step_size;

			ArrayList<Integer> best_route = null;

			initialize_labels();

			while(f_limit + b_limit < inst.Q + 1e-6){
				//System.out.println("limit: " + flimit + " " + blimit);
//				System.out.println("forward start..");
				f_extend(inst.para.time_limit - (System.nanoTime() - inst.para.start_time) / 1e9, f_limit, gen_routes, gen_rcs, gen_costs, gen_capacities, route_hash);
				if(gen_routes.size() >= target_size || (System.nanoTime() - inst.para.start_time) / 1e9 > inst.para.time_limit)
					break;
//				System.out.println("backward start..");
				b_extend( inst.para.time_limit - (System.nanoTime() - inst.para.start_time) / 1e9, b_limit, gen_routes, gen_rcs, gen_costs, gen_capacities, route_hash);
				if(gen_routes.size() >= target_size || (System.nanoTime() - inst.para.start_time) / 1e9 > inst.para.time_limit)
					break;

				if(f_limit + b_limit > inst.Q - 1e-6){
					for(int i = 0; i < inst.N; i++){
						FTL.get(i).addAll(FUL.get(i));
						BTL.get(i).addAll(BUL.get(i));
					}
				}

				int ftl = 0;
				for(int i = 0; i < inst.N; i++)
					ftl += FTL.get(i).size();
				int ful = 0;
				for(int i = 0; i < inst.N; i++)
					ful += FUL.get(i).size();
				int btl = 0;
				for(int i = 0; i < inst.N; i++)
					btl += BTL.get(i).size();
				int bul = 0;
				for(int i = 0; i < inst.N; i++)
					bul += BUL.get(i).size();
//				System.out.println("labels number:\t" + ful + "\t" + ftl + "\t" + bul + "\t" + btl);
//				System.out.println("join start..");
				best_route = join( inst.para.time_limit - (System.nanoTime() - inst.para.start_time) / 1e9, gen_routes, gen_rcs, gen_costs, gen_capacities, route_hash);

				if(ful < bul)
					f_limit += step_size;
				else
					b_limit += step_size;
			}

			if(best_route == null)
				break;
			if(gen_routes.size() >= target_size || (System.nanoTime() - inst.para.start_time) / 1e9 > inst.para.time_limit)
				break;
			ArrayList<ArrayList<Integer>> circles = get_circles(best_route);
			if(circles.size() == 0)
				break;
			for(int i = 0; i < circles.size(); i++){
				update_ng_set(circles.get(i));
			}
			if(is_relax)
				break;
		}

		// select the best route as the return value
		while(columns.size() < target_size && gen_routes.size() > 0){
			double min_cost = 0;
			int min_id = -1;
			for(int i = 0; i < gen_routes.size(); i++){
				if(gen_rcs.get(i) < min_cost){
					min_cost = gen_rcs.get(i);
					min_id = i;
				}
			}
			columns.add(gen_routes.get(min_id));
			reduce_costs.add(gen_rcs.get(min_id));
			costs.add(gen_costs.get(min_id));
			capacities.add(gen_capacities.get(min_id));

			gen_routes.remove(min_id);
			gen_rcs.remove(min_id);
			gen_costs.remove(min_id);
			gen_capacities.remove(min_id);
		}
	}

	public void initialize_labels(){
		// initial labels
		FUL = new ArrayList<>();
		FTL = new ArrayList<>();
		for(int i = 0; i < inst.N; i++){
			FUL.add(new ArrayList<>(10000));
			FTL.add(new ArrayList<>(10000));
		}

		BUL = new ArrayList<>();
		BTL = new ArrayList<>();
		for(int i = 0; i < inst.N; i++){
			BUL.add(new ArrayList<>(10000));
			BTL.add(new ArrayList<>(10000));
		}

		Label flabel = new Label();
		flabel.id = 0;
		flabel.rc = -mu[0] - dur_mu + inst.s[0];
		flabel.t = inst.s[0];
		flabel.q = 0;
		flabel.VI = 0x00;
		flabel.VII = 0x00;
		flabel.UI = 0x00;
		flabel.UII = 0x00;
		flabel.WI = 0x00;
		flabel.WII = 0x00;
		if(is_sr){
			flabel.sr_set = new int[sr_node_set[1].length];
			flabel.sr_cov = new int[sr_node_set[1].length];
			flabel.src = 0;
		}
		flabel.father = null;
		FUL.get(0).add(flabel);

		Label blabel = new Label();
		blabel.id = 0;
		blabel.rc = -mu[0] - dur_mu + inst.s[0];
		blabel.t = inst.s[0];
		blabel.q = 0;
		blabel.VI = 0x00;
		blabel.VII = 0x00;
		blabel.UI = 0x00;
		blabel.UII = 0x00;
		blabel.WI = 0x00;
		blabel.WII = 0x00;
		if(is_sr){
			blabel.sr_set = new int[sr_node_set[1].length];
			blabel.sr_cov = new int[sr_node_set[1].length];
			blabel.src = 0;
		}
		blabel.father = null;
		BUL.get(0).add(blabel);
	}

	public void initialize_algorithm(){

		// ------------------------------ initial ng-set
		for(int i = 0; i < inst.N; i++){
			ngVI[i] = 0x00;
			ngVII[i] = 0x00;
		}
		for(int i = 0; i < inst.N; i++){
			boolean[] select = new boolean[inst.N];
			int count = 0;
			while(count++ < ng_size - 1){
				double min_cost = 1e10;
				int min_id = -1;
				for(int j = 0; j < inst.N; j++){
					if(select[j] || i == j)
						continue;
					if(inst.t[i][j] < min_cost){
						min_cost = inst.t[i][j];
						min_id = j;
					}
				}
				ngVI[i] = ngVI[i] | maskI[min_id];
				ngVII[i] = ngVII[i] | maskII[min_id];
				select[min_id] = true;
			}
			ngVI[i] = ngVI[i] | maskI[i];
			ngVII[i] = ngVII[i] | maskII[i];
		}
		if(is_relax){
			for(int i = 0; i < inst.N; i++){
				ngVI[0] = ngVI[0] | maskI[i];
				ngVII[0] = ngVII[0] | maskII[i];
			}
			for(int i = 1; i < inst.N; i++){
				ngVI[i] = ngVI[0];
				ngVII[i] = ngVII[0];
			}
		}

		// ------------------------------ bound
		if(is_bound_active){
			bound.compute_bound(mu,amu);
//			if(!is_relax && is_sr){
//				e_bound.solve(mu,amu,sr_node_set,sr_comp);
//			}
		}

	}

	public void f_extend(double time_limit, double limit, ArrayList<ArrayList<Integer>> routes, ArrayList<Double> rcs, ArrayList<Double> costs, ArrayList<Double> capacities, HashSet<ArrayList<Integer>> route_hash){
		while((System.nanoTime() - inst.para.start_time) / 1e9 < time_limit){
			Label label = null;
			for(int i = 0; i < inst.N; i++){
				for(int j = 0; j < FUL.get(i).size(); j++){
					Label temp = FUL.get(i).get(j);
					if(temp.q <= limit ){
						label = temp;
						FUL.get(i).remove(j);
						break;
					}
				}
				if(label != null)
					break;
			}
			if(label == null)
				break;
			FTL.get(label.id).add(label);

			for(int i = 1; i < inst.N; i++){
				if(i == label.id || fc != null && fc[label.id][i] == -1)
					continue;
				if(label.q + inst.q[i] > inst.Q)
					continue;
				if(maskI[i] != 0 && (label.VI & maskI[i]) == maskI[i] || maskII[i] != 0 && (label.VII & maskII[i]) == maskII[i])
					continue;
				Label new_label = new Label();
				new_label.id = i;
				new_label.rc = label.rc + inst.t[label.id][new_label.id] + inst.s[new_label.id] - amu[label.id][new_label.id] - mu[new_label.id];
				new_label.q = label.q + inst.q[i];
				new_label.t = label.t + inst.t[label.id][new_label.id] + inst.s[new_label.id];
				if(is_bound_active){
					if(new_label.rc + bound.fcb[new_label.id][new_label.q] > -inst.para.Tolerance){
						continue;
					}
//					if(!is_relax && is_sr){
//						if(new_label.rc + e_bound.fcb[i][new_label.q] > -inst.para.Tolerance)
//							continue;
//					}
				}

				binary_handle(label, new_label);
				sr_handle(label, new_label);
				new_label.father = label;

				if(dominate(FUL, FTL, new_label) == false){
					FUL.get(new_label.id).add(new_label);
				}
				// get target route
				double r_cost = new_label.rc + inst.t[new_label.id][0] - amu[new_label.id][0];
				if(r_cost < -inst.para.Tolerance
						&& new_label.WI == 0x00 && new_label.WII == 0x00
						&& (fc == null || fc[new_label.id][0] != -1)){
					ArrayList<Integer> route = new ArrayList<>();
					Label l = new_label;
					while(l != null){
						route.add(0, l.id);
						l = l.father;
					}
					route.add(0);

					if(route_hash.contains(route) == false && extra_hash.contains(route) == false){
						routes.add(route);
						costs.add(new_label.t + inst.t[new_label.id][0]);
						rcs.add(new_label.rc + inst.t[new_label.id][0] - amu[new_label.id][0]);
						capacities.add((double)new_label.q);
						route_hash.add(route);
						if(check_route(route) == false){
							System.out.println("get set: " + get_set(label.VI, label.VII));
							System.exit(0);
						}
						if(routes.size() >= target_size)
							return;
					}
				}
			}
		}
	}

	public void b_extend(double time_limit, double limit, ArrayList<ArrayList<Integer>> routes, ArrayList<Double> rcs, ArrayList<Double> costs, ArrayList<Double> capacities, HashSet<ArrayList<Integer>> route_hash){
		while((System.nanoTime() - inst.para.start_time) / 1e9 < time_limit){
			Label label = null;
			for(int i = 0; i < inst.N; i++){
				for(int j = 0; j < BUL.get(i).size(); j++){
					Label temp = BUL.get(i).get(j);
					if(temp.q <= limit ){
						label = temp;
						BUL.get(i).remove(j);
						break;
					}
				}
				if(label != null)
					break;
			}
			if(label == null)
				break;

			BTL.get(label.id).add(label);
			for(int i = 1; i < inst.N; i++){

				if(i == label.id || fc != null && fc[i][label.id] == -1)
					continue;
				if(label.q + inst.q[i] > inst.Q)
					continue;
				if(maskI[i] != 0 && (label.VI & maskI[i]) == maskI[i] || maskII[i] != 0 && (label.VII & maskII[i]) == maskII[i])
					continue;
				Label new_label = new Label();
				new_label.id = i;
				new_label.rc = label.rc + inst.t[new_label.id][label.id] + inst.s[new_label.id] - amu[new_label.id][label.id] - mu[new_label.id];
				new_label.q = label.q + inst.q[i];
				new_label.t =  label.t + inst.t[new_label.id][label.id] + inst.s[new_label.id];
				if(is_bound_active){
					if(new_label.rc + bound.bcb[new_label.id][new_label.q] > -inst.para.Tolerance) {
						continue;
					}
//					if(!is_relax && is_sr){
//						if(new_label.rc + e_bound.bcb[i][new_label.q] > -inst.para.Tolerance)
//							continue;
//					}
				}

				binary_handle(label, new_label);
				sr_handle(label, new_label);
				new_label.father = label;

				if(dominate(BUL, BTL, new_label) == false){
					BUL.get(new_label.id).add(new_label);
				}

				// get target route
				double r_cost = new_label.rc + inst.t[0][new_label.id] - amu[0][new_label.id];
				if(r_cost < -inst.para.Tolerance
						&& new_label.WI == 0x00 && new_label.WII == 0x00
						&& (fc == null || fc[0][new_label.id] != -1)){

					ArrayList<Integer> route = new ArrayList<>();
					route.add(0);
					Label l = new_label;
					while(l != null){
						route.add(l.id);
						l = l.father;
					}
					if(route_hash.contains(route) == false && extra_hash.contains(route) == false){
						routes.add(route);
						costs.add(new_label.t + inst.t[0][new_label.id]);
						rcs.add(new_label.rc + inst.t[0][new_label.id] - amu[0][new_label.id]);
						capacities.add((double)new_label.q);
						route_hash.add(route);
						boolean ret = check_route(route);
						if(ret == false){
							System.out.println("get set: " + get_set(label.VI, label.VII));
							System.exit(0);
						}
						if(routes.size() >= target_size)
							return;
					}
				}
			}
		}
	}

	public ArrayList<Integer> join(double time_limit, ArrayList<ArrayList<Integer>> routes, ArrayList<Double> rcs, ArrayList<Double> costs,ArrayList<Double> capacities, HashSet<ArrayList<Integer>> route_hash){
		double min_cost = 0;
		ArrayList<Integer> min_route = null;
		for(int i = 1; i < inst.N; i++){
			ArrayList<Label> f_set = FTL.get(i);
			ArrayList<Label> b_set = BTL.get(i);
			for(int j = 0; j < f_set.size(); j++){
				for(int k = 0; k < b_set.size(); k++){
					Label f_label = f_set.get(j);
					Label b_label = b_set.get(k).father;
					// debug
//					if(indicate_flabel(f_label) && indicate_blabel(b_label)){
//						System.out.println();
//					}
					// check the feasibility
					if(fc != null && fc[f_label.id][b_label.id] == -1)
						continue;
					if(f_label.q + b_label.q > inst.Q)
						continue;
					if((f_label.VI & b_label.UI) != 0x00 || (f_label.VII & b_label.UII) != 0x00
							|| (f_label.UI & b_label.VI) != 0x00 || (f_label.UII & b_label.VII) != 0x00)
						continue;
					double rc = f_label.rc + b_label.rc + inst.t[f_label.id][b_label.id] + mu[0] + dur_mu;
					double cost = f_label.t + b_label.t + inst.t[f_label.id][b_label.id];
					int q = f_label.q + b_label.q;
					if(rc > -inst.para.Tolerance)
						continue;
					if(is_sr){
						for(int h = 0; h < f_label.sr_set.length; h++){
							int s1 = f_label.sr_set[h];
							int s2 = b_label.sr_set[h];
							int s3 = f_label.sr_cov[h];
							int s4 = b_label.sr_cov[h];
							rc += sr_comp[h][s3 & s4];
							int s5 = s1 & s2 & (~s3) &(~s4);
							rc -= sr_comp[h][s5];
						}
						if(rc > -inst.para.Tolerance)
							continue;
					}
					ArrayList<Integer> route = new ArrayList<>();
					Label l = f_label;
					while(l != null){
						route.add(0, l.id);
						l = l.father;
					}
					l = b_label;
					while(l != null){
						route.add(l.id);
						l = l.father;
					}

					if(rc < min_cost){
						min_cost = rc;
						min_route = route;
					}
					if(f_label.WI == 0x00 && f_label.WII == 0x00
							&& b_label.WI == 0x00 && b_label.WII == 0x00
							&& (f_label.UI & b_label.UI) == 0x00 && (f_label.UII & b_label.UII) == 0x00){

						if(route_hash.contains(route) == false && extra_hash.contains(route) == false){
							routes.add(route);
							costs.add(cost);
							rcs.add(rc);
							capacities.add((double)q);
							route_hash.add(route);

							// elementary
							boolean ret = check_route(route);
							if(ret == false){
								System.out.println("label setting route error ");
								System.exit(0);
							}
						}

						if(routes.size() >= target_size)
							return min_route;
						if( (System.nanoTime() - inst.para.start_time) / 1e9 > time_limit)
							return min_route;
					}
				}
			}
		}
		return min_route;
	}

	public boolean dominate(ArrayList<ArrayList<Label>> check1, ArrayList<ArrayList<Label>> check2, Label la){
		// dominate
		for(int j = 0; j < 2; j++){
			ArrayList<Label> set;
			if(j == 0)
				set = check1.get(la.id);
			else
				set = check2.get(la.id);

			for(int i = 0; i < set.size(); i++){
				Label lb = set.get(i);
				if(lb.q > la.q)
					continue;
				if(lb.t > la.t)
					continue;
				if(is_relax){
					if(lb.rc > la.rc)
						continue;
				} else{
					if((lb.VI & la.VI) != lb.VI || (lb.VII & la.VII) != lb.VII)
						continue;
					if(lb.rc - lb.src <= la.rc){
						return true;
					}
					double sr_penalty = 0;
					double neg_sr_penalty = 0;
					if(is_sr){
						for(int k = 0; k < lb.sr_set.length; k++){
							int s1 = lb.sr_set[k];
							int s2 = la.sr_set[k];
							/**
							 * 首先：la的可拓展集合 包含于 lb可拓展集合
							 */
							sr_penalty += sr_comp[k][s1 & (~s2)];   // 当拓展到一些点集合：只影响 lb 的 sub set row 的违反情况，不影响 la 的 sub set row 减小情况；
							neg_sr_penalty += sr_comp[k][s1 & s2];  // 当拓展到一些点集合：会同时影响 lb 和 la 的 sub set row 的违反情况；
							if(lb.rc - sr_penalty > la.rc)            // 不能占优 当这种
								break;
							else if(lb.rc - lb.src + neg_sr_penalty <= la.rc){  // neg_sr_penalty 是 未来 la 和 lb 共同的 sr cost，当 lb - lb.src <= la.c - neg_sr_penalty 时，说明，lb 至少会 和 la 的reduce cost 一样小
								return true;
							}
						}
					}
					if(lb.rc - sr_penalty > la.rc)
						continue;
				}
				return true;
			}
		}

		for(int j = 0; j < 1; j++){
			ArrayList<Label> set;
			if(j == 0)
				set = check1.get(la.id);
			else
				set = check2.get(la.id);
			for(int i = 0; i < set.size(); i++){
				Label lb = set.get(i);
				if(la.q > lb.q)
					continue;
				if(la.t > lb.t)
					continue;
				if(is_relax){
					if(la.rc > lb.rc)
						continue;
				}
				else{
					if((la.VI & lb.VI) != la.VI || (la.VII & lb.VII) != la.VII)
						continue;
					if(la.rc - la.src <= lb.rc){
						set.remove(i);
						i--;
						continue;
					}
					double sr_penalty = 0;
					double neg_sr_penalty = 0;
					if(is_sr){
						for(int k = 0; k < la.sr_set.length; k++){
							int s1 = la.sr_set[k];
							int s2 = lb.sr_set[k];
							sr_penalty += sr_comp[k][s1 & (~s2)];
							neg_sr_penalty += sr_comp[k][s1 & s2];
							if(la.rc - sr_penalty > lb.rc)
								break;
							else if(la.rc - la.src + neg_sr_penalty <= lb.rc){
								break;
							}
						}
					}
					if(la.rc - sr_penalty > lb.rc)
						continue;
				}
				set.remove(i);
				i--;
			}
		}
		return false;
	}

	public void sr_handle(Label label, Label new_label){
		if(is_sr == false)
			return;
		new_label.sr_set = new int[label.sr_set.length];
		new_label.sr_cov = new int[label.sr_cov.length];
		new_label.src = 0.0;
		for(int j = 0; j < label.sr_set.length; j++){
			int s1 = label.sr_set[j];
			int s2 = sr_node_set[new_label.id][j];
			new_label.rc -= sr_comp[j][s1 & s2];
			new_label.sr_cov[j] = label.sr_cov[j] | (s1 & s2);
			int s3 = s1 & (~s2) | (~s1) & s2;
			new_label.sr_set[j] = s3 & (~new_label.sr_cov[j]);
			new_label.src += sr_comp[j][new_label.sr_set[j]];
		}
	}
	
	public void binary_handle(Label label, Label new_label){
		new_label.VI = (label.VI & ngVI[new_label.id]) | maskI[new_label.id] | capVI[new_label.q];
		new_label.VII = (label.VII & ngVII[new_label.id]) | maskII[new_label.id] | capVII[new_label.q];
		new_label.UI = label.UI | maskI[new_label.id];
		new_label.UII = label.UII | maskII[new_label.id];
		new_label.WI = label.WI | (label.UI & maskI[new_label.id]);
		new_label.WII = label.WII | (label.UII & maskII[new_label.id]);
	}

	/**
	 * for ng route
	 * 获取最好的label中的环
	 * @param route
	 * @return
	 */
	public ArrayList<ArrayList<Integer>> get_circles(ArrayList<Integer> route){
		ArrayList<ArrayList<Integer>> circles = new ArrayList<>();
		for(int i = 1; i < route.size() - 3; i++){
			for(int j = i + 1; j < route.size() - 1; j++){
				if(route.get(j) == route.get(i)){
					ArrayList<Integer> circle = new ArrayList<>();
					for(int k = i; k < j; k++){
						circle.add(route.get(k));
					}
					circles.add(circle);
					break;
				}
			}
		}
		return circles;
	}

	/**
	 * update ng route
	 * @param circle
	 */
	public void update_ng_set(ArrayList<Integer> circle){
		int start = circle.get(0);
		for(int i = 1; i < circle.size(); i++){
			int id = circle.get(i);
			ngVI[id] = ngVI[id] | maskI[start];
			ngVII[id] = ngVII[id] | maskII[start];
		}
	}

	public ArrayList<Integer> get_set(long VI, long VII){
		ArrayList<Integer> set = new ArrayList<>();
		for(int i = 0; i < inst.N; i++){
			if(maskI[i] != 0 && (VI & maskI[i]) == maskI[i] || maskII[i] != 0 && (VII & maskII[i]) == maskII[i]){
				set.add(i);
			}
		}
		return set;
	}

	public boolean check_route(ArrayList<Integer> route){
		// check constraints
		int sq = 0;
		int[] visit = new int[inst.N];
		for(int i = 1; i < route.size() - 1; i++){
			sq += inst.q[route.get(i)];
			visit[route.get(i)]++;
		}
		if(sq > inst.Q){
			System.out.println("error>> capacity violation: " + sq + " " + inst.Q);
			return false;
		}

		for(int i = 1; i < inst.N; i++){
			if(visit[i] > 1){
				System.out.println("error>> elementary violation " + route);
				return false;
			}
		}
		if(fc != null){
			for(int i = 0; i < route.size() - 1; i++){
				if(fc[route.get(i)][route.get(i + 1)] == -1){
					System.out.println("error>>forbid arc " + route.get(i) + "-" + route.get(i + 1) + " " + route);
					return false;
				}
			}
		}
		return true;
	}

	/**
	 * ----------------------------not used
	 */

	/**
	 * 前向检测
	 * @param label
	 */
	public void find_flabel(Label label){
		int[] r = {0, 6, 4, 1, 2, 3, 5, 8, 0};
		Label l = label;
		ArrayList<Integer> s = new ArrayList<>();
		while(l != null){
			s.add(0, l.id);
			l = l.father;
		}
		if(s.size() > r.length){
			return;
		}
		for(int i = 0; i < s.size(); i++){
			if(s.get(i) != r[i]){
				return;
			}
		}
		System.out.println("forward find: " + s);
	}

	/**
	 * 后向检测
	 * @param label
	 */
	public void find_blabel(Label label){
		int[] r = {0, 6, 4, 1, 2, 3, 5, 8, 0};
		Label l = label;
		ArrayList<Integer> s = new ArrayList<Integer>();
		while(l != null){
			s.add(l.id);
			l = l.father;
		}
		if(s.size() > r.length){
			return;
		}
		for(int i = 0; i < s.size(); i++){
			if(s.get(s.size() - 1 - i) != r[r.length - 1 - i]){
				return;
			}
		}
		System.out.println("backward find: " + s);
	}

	/**
	 *
	 * @return
	 */
	public int get_ng_count(){
		int count = 0;
		for(int i = 0; i < inst.N; i++){
			int size = 0;
			for(int j = 0; j < inst.N; j++){
				if(i == j)
					continue;
				if(maskI[j] != 0 && (ngVI[i] & maskI[j]) == maskI[j] || maskII[j] != 0 && (ngVII[i] & maskII[j]) == maskII[j])
					size++;
			}
			if(size > count)
				count = size;
		}
		return count;
	}

	/**
	 * 定位到 出错 状态 的 label
	 */
	public boolean indicate_flabel(Label label){
		ArrayList<Integer> r = new ArrayList<>(Arrays.asList(0,9,7,4));
		Label l = label;
		ArrayList<Integer> s = new ArrayList<>();
		while(l != null){
			s.add(0, l.id);
			l = l.father;
		}

		if(r.equals(s)){
			return true;
		}else{
			return false;
		}
	}

	public boolean indicate_blabel(Label label){
		ArrayList<Integer> r = new ArrayList<>(Arrays.asList(5, 2, 0));
		Label l = label;
		ArrayList<Integer> s = new ArrayList<>();
		while(l != null){
			s.add(l.id);
			l = l.father;
		}

		if(r.equals(s)){
			return true;
		}else{
			return false;
		}

	}

}
