package lrp.labelsetting;

import lrp.data.Instance;

/**
 * 算法逻辑可以根据 三列 的思想；
 * 初始化：
 * 1 前向搜索，将第三列的 最后一个 nodeId 在 容量为 0 时赋值为0
 * 2 后向搜索，将第三列，第一个 nodeId 在容量为 0 时赋值为 0；
 */

public class CapTwoCycleFree {

	// --------------------------------- instance information
	/**
	 *  instance information
	 */
	public Instance inst;

	//--------------------------------------------- 模型传递过来的数据
	/**
	 * 被禁止的变的集合
	 */
	public int[][] fc;
	/**
	 * 点的对偶变量值
	 */
	public double[] mu;
	/**
	 * 边的对偶变量值
	 */
	public double[][] amu;

	//--------------------------------------------- bound 需要的数据
	/**
	 * f[i][w]: 在节点i，使用容量 w，可以拓展的 reduced cost最小的片段；
	 */
	double[][] f;
	/**
	 * f_pre[i][w]：存储 f[i][w] 的前一个节点；
	 */
	int[][] f_pre;
	/**
	 * g[i][w]: 在节点i时，使用容量为w，可以拓展的次优的片段（用于防止循环）。
	 */
	double[][] g;
	/**
	 * [i][q]: 到达 i点，容量还剩余 q，后半段可拓展的label 的最小的 reduce cost
	 */
	public double[][] fcb;
	/**
	 * [i][q]: 到达 i点，容量还剩余 q，前半段可拓展的label 的最小的 reduce cost
	 */
	public double[][] bcb;

	/**
	 * 构造函数
	 * @param _inst
	 * @param _fc
	 */
	public CapTwoCycleFree(Instance _inst, int[][] _fc){
		inst = _inst;
		fc = _fc;
		fcb = new double[inst.N][inst.Q + 1];
		bcb = new double[inst.N][inst.Q + 1];
	}

	/**
	 * 前向拓展
	 * @return
	 */
	private double[][] f_two_cycle_free(){
		f = new double[inst.N+1][inst.Q + 1];
		f_pre = new int[inst.N+1][inst.Q + 1];
		g = new double[inst.N+1][inst.Q + 1];

		// 初始化
		for(int i = 0; i < inst.N+1; i++){
			for(int w = 0; w <= inst.Q; w++){
				f[i][w] = 1e10;
				f_pre[i][w] = -1;
				g[i][w] = 1e10;
			}
		}
		f[0][0] = 0;

		for(int w = 1; w <= inst.Q; w++){
			for(int i = 1; i < inst.N; i++){
				if(w < inst.q[i])
					continue;
				for(int j = 0; j < inst.N; j++){
					if(j == i || fc != null && fc[i][j] == -1)
						continue;

					if(f_pre[j][w - inst.q[i]] != i){
						double cost = f[j][w-inst.q[i]] + inst.t[i][j] + inst.s[i] - mu[i] - amu[i][j];
						if(cost < f[i][w]){
							if(f_pre[i][w] != j)
								g[i][w] = f[i][w];
							f[i][w] = cost;
							f_pre[i][w] = j;
						} else if(cost < g[i][w] && f_pre[i][w] != j){
							g[i][w] = cost;
						}
					} else{
						double cost = g[j][w-inst.q[i]] + inst.t[i][j] + inst.s[i] - mu[i] - amu[i][j];
						if(cost < f[i][w]){
							g[i][w] = f[i][w];
							f[i][w] = cost;
							f_pre[i][w] = j;
						} else if(cost < g[i][w] && f_pre[i][w] != j){
							g[i][w] = cost;
						}
					}
				}
			}
		}
		return f;
	}

	/**
	 * 后向拓展
	 * @return
	 */
	private double[][] b_two_cycle_free(){
		f = new double[inst.N][inst.Q + 1];
		f_pre = new int[inst.N][inst.Q + 1];
		g = new double[inst.N][inst.Q + 1];

		// 初始化
		for(int i = 0; i < inst.N; i++){
			for(int w = 0; w <= inst.Q; w++){
				f[i][w] = 1e10;
				f_pre[i][w] = -1;
				g[i][w] = 1e10;
			}
		}
		f[0][0] = 0;

		for(int w = 1; w <= inst.Q; w++){
			for(int i = 1; i < inst.N; i++){
				if(w < inst.q[i])
					continue;
				for(int j = 0; j < inst.N; j++){
					if(j == i || fc != null && fc[j][i] == -1)
						continue;
					if(f_pre[j][w - inst.q[i]] != i){
						double cost = f[j][w - inst.q[i]] + inst.t[j][i] + inst.s[i] - mu[i] - amu[i][j];
						if(cost < f[i][w]){
							if(f_pre[i][w] != j)
								g[i][w] = f[i][w];
							f[i][w] = cost;
							f_pre[i][w] = j;
						} else if(cost < g[i][w] && f_pre[i][w] != j){
							g[i][w] = cost;
						}
					} else{
						double cost = g[j][w - inst.q[i]] + inst.t[j][i] + inst.s[i] - mu[i] - amu[i][j];
						if(cost < f[i][w]){
							g[i][w] = f[i][w];
							f[i][w] = cost;
							f_pre[i][w] = j;
						} else if(cost < g[i][w] && f_pre[i][w] != j){
							g[i][w] = cost;
						}

					}
				}
			}
		}
		return f;
	}

	/**
	 * 总函数
	 * @param _mu
	 * @param _amu
	 */
	public void compute_bound(double[] _mu, double[][] _amu){
		mu = _mu;
		amu = _amu;

		// ---------------------------- forward bounding calculate
		double[][] f_state = f_two_cycle_free();

		// 不考虑该点对偶值（labelsetting 中考虑了）
		for(int i = 1; i < inst.N; i++){
			for(int q = 0; q <= inst.Q; q++){
				f_state[i][q] += mu[i];
			}
		}
		// 因为初始化的时候，将每个荣量值都设定为一个很大的数，但是有些容量值是没有被拓展到 (不可能存在的情况)
		for(int i = 0; i < inst.N; i++){
			for(int q = 1; q <= inst.Q; q++){
				if(f_state[i][q - 1] < f_state[i][q]){
					f_state[i][q] = f_state[i][q - 1];
				}
			}
		}
		for(int i = 0; i < inst.N; i++){
			for(int j = inst.q[i]; j <= inst.Q; j++){
				fcb[i][j] = f_state[i][inst.Q - j + inst.q[i]];
			}
		}

		// ---------------------------- backward bounding calculate
		double[][] b_state = b_two_cycle_free();
		for(int i = 1; i < inst.N; i++){
			for(int q = 0; q <= inst.Q; q++){
				b_state[i][q] += mu[i];
			}
		}
		for(int i = 0; i < inst.N; i++){
			for(int q = 1; q <= inst.Q; q++){
				if(b_state[i][q - 1] < b_state[i][q]){
					b_state[i][q] = b_state[i][q - 1];
				}
			}
		}
		for(int i = 0; i < inst.N; i++){
			for(int j = inst.q[i]; j <= inst.Q; j++){
				bcb[i][j] = b_state[i][inst.Q - j + inst.q[i]];
			}
		}
	}
}
