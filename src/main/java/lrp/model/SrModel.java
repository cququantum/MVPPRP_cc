package lrp.model;

import ilog.concert.*;
import ilog.cplex.IloCplex;
import java.util.ArrayList;
import java.util.HashMap;
import ilog.cplex.IloCplex.UnknownObjectException;

public final class SrModel {

	// --------------------------- instance information & parameter
	/**
	 * 客户点数量
 	 */
	public int n;
	/**
	 * 当 subset row 约束对应的对偶值小于 -sr_neg_tolerance 时才记录该约束的id
	 */
	public double sr_neg_tolerance = 1e-4;

	// --------------------------- columns & solution information
	/**
	 * 访问了点 i 的路径(列)的id集合
	 */
	public ArrayList<Integer>[] i2ri;

	// --------------------------- results information
	/**
	 * sr cuts 对应的对偶值
	 */
	public ArrayList<Double> sr_dual;

	// --------------------------- model information
	/**
	 * 模型
	 */
	public IloCplex cplex;
	/**
	 * 存储subset row 约束
	 */
	public ArrayList<IloRange> sr_ranges;
	/**
	 * 从基本模型中传进来的 column_variables 集合
	 */
	public ArrayList<IloNumVar> sr_columns;

	// --------------------------- cuts information
	/**
	 * 存储 subset row 约束对应的 点集（这里是3个点）
	 */
	public ArrayList<ArrayList<Integer>> sr_cuts;
	/**
	 * 存储 subset row 的右端项
	 */
	public ArrayList<Integer> sr_rights;

	/**
	 * 第一维：客户点 i
	 * 第二维：包含该客户点的，对偶值为负的（起作用的）subset row cut 在 neg_index 中的下标
	 */
	public ArrayList<ArrayList<Integer>> node_index;
	/**
	 * 对偶值为负（起作用的）的 sr_cuts id 从 0 开始
	 */
	public ArrayList<Integer> neg_index;
	/**
	 * 每组的长度
	 */
	public int seg_size;
	/**
	 * sr_node_set[i][j]：size: [n][(neg_subset_row size) / seg_size];
	 * i 客户点 id;
	 * j (neg_subset_row size) / seg_size, 根据 seg_size 分组，e.g. 一共有100个 sr-cut；那么 第二维就有 ceil（100.0/19.0）= 6 组;
	 * 表示：存储的是，包含 i 的，neg sr cut 在 neg_index 中的下标集合;
	 * 思想：用二维数组来存储一个一维数组,第二维是二进制形式;
	 * 这个数字对应的 sr_comp[j][k]:
	 */
	public int[][] sr_node_set;
	/**
	 * sr_comp[i][j]: 记录
	 * i 是第 i 个 segment（每个segment 保存19个 cut）
	 * 第一维：segment_id;
	 * 第二维：每个位置就是该位置的下标值；
	 * sr_comp[0][1] 这个位置 对偶值 和
	 */
	public double[][] sr_comp;

	/**
	 * 构建带 sr_cut的模型
	 * @param _cplex
	 * @param _sr_columns
	 * @param _i2ri
	 * @param _n
	 */
	public SrModel(IloCplex _cplex, ArrayList<IloNumVar> _sr_columns, ArrayList<Integer>[] _i2ri, int _n){
		n = _n;
		cplex = _cplex;
		sr_columns = _sr_columns;
		i2ri = _i2ri;
		sr_ranges = new ArrayList<>();
		sr_cuts = new ArrayList<>();
		sr_rights = new ArrayList<>();
		sr_dual = new ArrayList<>();
		neg_index = new ArrayList<>();
		node_index = new ArrayList<>();
		for(int i = 0; i < n; i++){
			node_index.add(new ArrayList<Integer>());
		}
	}

	/**
	 * 将列 加入 subset row 约束
	 * @param col
	 * @param column
	 * @return
	 * @throws IloException
	 */
	public IloColumn add_column(IloColumn col, ArrayList<Integer> column) throws IloException{
		int[]  col_map = new int[n];
		for(int i = 1; i < column.size() - 1; i++){
			int id = column.get(i);
			if(id < n)                     // 本研究路径中没有充电桩：TODO：可删除
				col_map[id]++;
		}

		ArrayList<Integer> sr = new ArrayList<>();
		ArrayList<Integer> srv = new ArrayList<>();

		for(int j = 0; j < sr_cuts.size(); j++){
			ArrayList<Integer> cut = sr_cuts.get(j);
			int count = 0;
			for(int k = 0; k < cut.size(); k++){
				count += col_map[cut.get(k)];
			}
			int v = (int) Math.floor(count / 2.0);
			if(v > 0){
				sr.add(j);
				srv.add(v);
			}
		}
		for(int j = 0; j < sr.size(); j++){
			col = col.and(cplex.column(sr_ranges.get(sr.get(j)), srv.get(j)));
		}
		return col;
	}

	/**
	 * 记录将生成的cut, 加入模型
	 * @param cut
	 * @throws IloException
	 */
	public void add_cut(ArrayList<Integer> cut) throws IloException{
		/**
		 ------------------------------ 存储加入模型的 sr cuts
		 */
		sr_cuts.add(cut);

		/**
		 ------------------------------ 将 sr cuts 加入模型
		 */
		// 路径 id 列表（ 只要访问了cut中三个点中的任意一个 都会包含在内）
		ArrayList<Integer> var = new ArrayList<>();
		// 路径 id 对应的次数
		ArrayList<Double> count = new ArrayList<>();
		// <路径id，此id在var中的位置>
		HashMap<Integer, Integer> hash = new HashMap<>();

		for(int j = 0; j < cut.size(); j++){
			int id = cut.get(j);
			ArrayList<Integer> col_set = i2ri[id];
			for(int k = 0; k < col_set.size(); k++){
				int cid = col_set.get(k);
				if(hash.containsKey(cid)){
					int p = hash.get(cid);
					count.set(p, count.get(p) + 1);
				} else{
					var.add(cid);
					count.add(1.0);
					hash.put(cid, var.size() - 1);
				}
			}
		}

		//  记录 cut 右端项
		int right = (int) Math.floor( ((double)cut.size())/2.0 );
		sr_rights.add(right);
		IloRange range = cplex.addRange(0.0, right);

		// cut 左端项
		IloNumExpr expr = cplex.numExpr(); 
		for(int j = 0; j < var.size(); j++){
			IloNumVar numvar = sr_columns.get(var.get(j));
			double coef = Math.floor(count.get(j) / 2.0);
			expr = cplex.sum(expr, cplex.prod(coef, numvar));
		}
		range.setExpr(expr);
		sr_ranges.add(range);
	}

	/**
	 * 获取 subset row 对应得对偶值
	 * @throws UnknownObjectException
	 * @throws IloException
	 */
	public void get_duals() throws UnknownObjectException, IloException{

		// 只取对偶值为负的，将正的对偶值赋值为0
		sr_dual.clear();
		for(int i = 0; i < sr_ranges.size(); i++){
			IloRange range = sr_ranges.get(i);
			sr_dual.add(Math.min(0, cplex.getDual(range)));
		}

		// 对偶值为负的 sub set row inequality 的下标
		neg_index.clear();
		for(int i = 0; i < sr_dual.size(); i++){
			if(sr_dual.get(i) < -sr_neg_tolerance){
				neg_index.add(i);
			}
		}

		for(int i = 0; i < n; i++){
			node_index.get(i).clear();
		}
		for(int i = 0; i < neg_index.size(); i++){
			int index = neg_index.get(i);
			ArrayList<Integer> cut = sr_cuts.get(index);
			for(int j = 0; j < cut.size(); j++){
				node_index.get(cut.get(j)).add(i);
			}
		}

		//
		build_sr_comp();
	}

	/**
	 * seg_size = 19;
	 * 构造：sr_node_set，sr_comp；
	 */
	public void build_sr_comp(){
		seg_size = 19;
		int[] mask = new int[seg_size];
		int m = 0x01;
		for(int i = 0; i < seg_size; i++){
			mask[i] = m;
			m *= 2;
		}

		int size = (int) Math.ceil( ((double) neg_index.size()) / seg_size);
		sr_node_set = new int[n][size];

		for(int i = 0; i < n; i++){
			if(i == 0){
				continue;
			}
			ArrayList<Integer> c_set = node_index.get(i);
			for(int j = 0; j < c_set.size(); j++){
				int id = c_set.get(j);
				int target = (int)Math.floor( (double)(id) / seg_size);
				sr_node_set[i][target] = sr_node_set[i][target] | mask[id - target * seg_size];
			}
		}

		sr_comp = new double[size][((int) Math.pow(2,seg_size))];   // 2的19次方
		for(int i = 0; i < size; i++){
			int S0 = 0;
			double v0 = 0;

			for(int j1 = 0; j1 < 2; j1++){
				int S1 = S0;
				double v1 = v0;

				if(j1 == 1){
					S1 = S0 | mask[0];
					v1 = v0 + sr_dual.get(neg_index.get(i * seg_size + 0));
				}

				if(i * seg_size + 0 == neg_index.size() - 1){
					sr_comp[i][S1] = v1;
				}
				else{
					for(int j2 = 0; j2 < 2; j2++){
						int S2 = S1;
						double v2 = v1;

						if(j2 == 1){
							S2 = S1 | mask[1];
							v2 = v1 + sr_dual.get(neg_index.get(i * seg_size + 1));
						}
						if(i * seg_size + 1 == neg_index.size() - 1){
							sr_comp[i][S2] = v2;
						}
						else{
							for(int j3 = 0; j3 < 2; j3++){
								int S3 = S2;
								double v3 = v2;
								if(j3 == 1){
									S3 = S2 | mask[2];
									v3 = v2 + sr_dual.get(neg_index.get(i * seg_size + 2));
								}
								if(i * seg_size + 2 == neg_index.size() - 1){
									sr_comp[i][S3] = v3;
								}
								else{
									for(int j4 = 0; j4 < 2; j4++){
										int S4 = S3;
										double v4 = v3;
										if(j4 == 1){
											S4 = S3 | mask[3];
											v4 = v3 + sr_dual.get(neg_index.get(i * seg_size + 3));
										}
										if(i * seg_size + 3 == neg_index.size() - 1){
											sr_comp[i][S4] = v4;
										}
										else{
											for(int j5 = 0; j5 < 2; j5++){
												int S5 = S4;
												double v5 = v4;
												if(j5 == 1){
													S5 = S4 | mask[4];
													v5 = v4 + sr_dual.get(neg_index.get(i * seg_size + 4));
												}
												if(i * seg_size + 4 == neg_index.size() - 1){
													sr_comp[i][S5] = v5;
												}
												else{
													for(int j6 = 0; j6 < 2; j6++){
														int S6 = S5;
														double v6 = v5;
														if(j6 == 1){
															S6 = S5 | mask[5];
															v6 = v5 + sr_dual.get(neg_index.get(i * seg_size + 5));
														}
														if(i * seg_size + 5 == neg_index.size() - 1){
															sr_comp[i][S6] = v6;
														}
														else{
															for(int j7 = 0; j7 < 2; j7++){
																int S7 = S6;
																double v7 = v6;
																if(j7 == 1){
																	S7 = S6 | mask[6];
																	v7 = v6 + sr_dual.get(neg_index.get(i * seg_size + 6));
																}
																if(i * seg_size + 6 == neg_index.size() - 1){
																	sr_comp[i][S7] = v7;
																}
																else{
																	for(int j8 = 0; j8 < 2; j8++){
																		int S8 = S7;
																		double v8 = v7;
																		if(j8 == 1){
																			S8 = S7 | mask[7];
																			v8 = v7 + sr_dual.get(neg_index.get(i * seg_size + 7));
																		}
																		if(i * seg_size + 7 == neg_index.size() - 1){
																			sr_comp[i][S8] = v8;
																		}
																		else{
																			for(int j9 = 0; j9 < 2; j9++){
																				int S9 = S8;
																				double v9 = v8;
																				if(j9 == 1){
																					S9 = S8 | mask[8];
																					v9 = v8 + sr_dual.get(neg_index.get(i * seg_size + 8));
																				}
																				if(i * seg_size + 8 == neg_index.size() - 1){
																					sr_comp[i][S9] = v9;
																				}
																				else{
																					for(int j10 = 0; j10 < 2; j10++){
																						int S10 = S9;
																						double v10 = v9;
																						if(j10 == 1){
																							S10 = S9 | mask[9];
																							v10 = v9 + sr_dual.get(neg_index.get(i * seg_size + 9));
																						}
																						if(i * seg_size + 9 == neg_index.size() - 1){
																							sr_comp[i][S10] = v10;
																						}
																						else{
																							for(int j11 = 0; j11 < 2; j11++){
																								int S11 = S10;
																								double v11 = v10;
																								if(j11 == 1){
																									S11 = S10 | mask[10];
																									v11 = v10 + sr_dual.get(neg_index.get(i * seg_size + 10));
																								}
																								if(i * seg_size + 10 == neg_index.size() - 1){
																									sr_comp[i][S11] = v11;
																								}
																								else{
																									for(int j12 = 0; j12 < 2; j12++){
																										int S12 = S11;
																										double v12 = v11;
																										if(j12 == 1){
																											S12 = S11 | mask[11];
																											v12 = v11 + sr_dual.get(neg_index.get(i * seg_size + 11));
																										}
																										if(i * seg_size + 11 == neg_index.size() - 1){
																											sr_comp[i][S12] = v12;
																										}
																										else{
																											for(int j13 = 0; j13 < 2; j13++){
																												int S13 = S12;
																												double v13 = v12;
																												if(j13 == 1){
																													S13 = S12 | mask[12];
																													v13 = v12 + sr_dual.get(neg_index.get(i * seg_size + 12));
																												}
																												if(i * seg_size + 12 == neg_index.size() - 1){
																													sr_comp[i][S13] = v13;
																												}
																												else{
																													for(int j14 = 0; j14 < 2; j14++){
																														int S14 = S13;
																														double v14 = v13;
																														if(j14 == 1){
																															S14 = S13 | mask[13];
																															v14 = v13 + sr_dual.get(neg_index.get(i * seg_size + 13));
																														}
																														if(i * seg_size + 13 == neg_index.size() - 1){
																															sr_comp[i][S14] = v14;
																														}
																														else{
																															for(int j15 = 0; j15 < 2; j15++){
																																int S15 = S14;
																																double v15 = v14;
																																if(j15 == 1){
																																	S15 = S14 | mask[14];
																																	v15 = v14 + sr_dual.get(neg_index.get(i * seg_size + 14));
																																}
																																if(i * seg_size + 14 == neg_index.size() - 1){
																																	sr_comp[i][S15] = v15;
																																}
																																else{
																																	for(int j16 = 0; j16 < 2; j16++){
																																		int S16 = S15;
																																		double v16 = v15;
																																		if(j16 == 1){
																																			S16 = S15 | mask[15];
																																			v16 = v15 + sr_dual.get(neg_index.get(i * seg_size + 15));
																																		}
																																		if(i * seg_size + 15 == neg_index.size() - 1){
																																			sr_comp[i][S16] = v16;
																																		}
																																		else{
																																			for(int j17 = 0; j17 < 2; j17++){
																																				int S17 = S16;
																																				double v17 = v16;
																																				if(j17 == 1){
																																					S17 = S16 | mask[16];
																																					v17 = v16 + sr_dual.get(neg_index.get(i * seg_size + 16));
																																				}
																																				if(i * seg_size + 16 == neg_index.size() - 1){
																																					sr_comp[i][S17] = v17;
																																				}
																																				else{
																																					for(int j18 = 0; j18 < 2; j18++){
																																						int S18 = S17;
																																						double v18 = v17;
																																						if(j18 == 1){
																																							S18 = S17 | mask[17];
																																							v18 = v17 + sr_dual.get(neg_index.get(i * seg_size + 17));
																																						}
																																						if(i * seg_size + 17 == neg_index.size() - 1){
																																							sr_comp[i][S18] = v18;
																																						}
																																						else{
																																							for(int j19 = 0; j19 < 2; j19++){
																																								int S19 = S18;
																																								double v19 = v18;

																																								if(j19 == 1){
																																									S19 = S18 | mask[18];
																																									v19 = v18 + sr_dual.get(neg_index.get(i * seg_size + 18));
																																								}
																																								sr_comp[i][S19] = v19;
																																							}
																																						}
																																					}
																																				}
																																			}
																																		}
																																	}
																																}
																															}
																														}
																													}
																												}
																											}
																										}
																									}
																								}
																							}
																						}
																					}
																				}
																			}
																		}
																	}
																}
															}
														}
													}
												}
											}
										}
									}
								}
							}
						}
					}
				}
			}
		}
	}

}
