package lrp.cut;

import java.util.ArrayList;
import java.util.HashSet;

public class CapSep2 {

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

	// ------------------------------- parameter
	public double violate_threshhold = 0.05;
	public double rl_gap = 1.2;
	public int limit_size = 12;

	// ------------------------------- return results
	/**
	 * cut set
	 */
	public ArrayList<ArrayList<Integer>> retS;
	/**
	 * cut 右端项
	 */
	public ArrayList<Integer> rightS;

	// ------------------------------- temporary data
	/**
	 * 以 2 进制的形式 存储 与 i 相连的点
	 * 第一维是 i，第二维是 与 i 相连的 点的二进制存储形式
	 */
	public ArrayList<Long> conI;
	public ArrayList<Long> conII;
	/**
	 * 2 进制 存储 从 0 到 n-1
	 */
	public ArrayList<Long> markS;

	/**
	 *
	 */
	public HashSet<String> hashState;

	/**
	 * 空构造函数
	 */
	public CapSep2(){};

	/**
	 * 构造函数
	 * @param _n
	 * @param _q
	 * @param _cap
	 * @param _map
	 */
	public CapSep2(int _n, int[] _q, int _cap, double[][] _map){
		n = _n;
		q = _q;
		cap = _cap;
		map = _map;
	}

	/**
	 *
	 * @param y:
	 * @param SI:
	 * @param SII:
	 * @param cut:存储找到的cut
	 * @param start:
	 * @param sq:累计容量
	 * @param v:累计的左端项流量
	 * @param depth: 控制递归的深度
	 * @param I:
	 * @param II:
	 */
	public void recursive(double[][]y, long SI, long SII, ArrayList<Integer> cut, int start, double sq, double v, int depth, long I, long II){
		if(depth > limit_size)
			return;

		// calculate the simga (d_{i})
		if(start >= 1 && start < n)
			sq += q[start];

		// calculate delta (S)
		if(start >= 1 && start < n){
			for(int i = 0; i < n; i++){ 
				if(i < 63){
					if((SI & markS.get(i)) != 0)
						v -= y[start][i];
					else
						v += y[i][start];
				} else{
					if((SII & markS.get(i)) != 0)
						v -= y[start][i];
					else
						v += y[i][start];
				}
			}
		}
		int right = (int) Math.ceil(sq/cap);
		if(v + violate_threshhold < right){
			retS.add((ArrayList<Integer>) cut.clone());
			rightS.add(right);
		}
		if(v - right > rl_gap)      // 大于一定的值说明很难再找到 cut 了
			return;

		// ------------------ extend to the next level
		for(int i = 1; i < n; i++){
			long mark = markS.get(i);
			if(depth == 0 || i < 63 && (I & mark) != 0 || i >= 63 && (II & mark) != 0){
				long tSI, tSII;
				if(i < 63){
					tSI = SI | mark;
					tSII = SII;
				} else{
					tSI = SI;
					tSII = SII | mark;
				}
				long tI = (I | conI.get(i)) & (~tSI);
				long tII = (II | conII.get(i)) & (~tSII);
				String stateKey = stateKey(tSI, tSII);
				if(hashState.contains(stateKey))
					continue;
				cut.add(i);

				hashState.add(stateKey);

				recursive(y, tSI, tSII, cut, i, sq, v, depth + 1, tI, tII);
				cut.remove(cut.size() - 1);
			}
		}

	}

	/**
	 * 主函数
	 */
	public void solve(){
		retS = new ArrayList<>();
		rightS = new ArrayList<>();
		conI = new ArrayList<>();
		conII = new ArrayList<>();
		markS = new ArrayList<>();
		hashState = new HashSet<>();
		for(int i = 0; i < n; i++){
			long mark = 0x01;
			long I = 0x00;
			long II = 0x00;
			for(int j = 0; j < n; j++){
				if(map[j][i] > 0 || map[i][j] > 0){
					if(j < 63)
						I = I | mark;
					else
						II = II | mark;
				}
				if(j == 62)
					mark = 0x01;
				else
					mark *= 2;
			}
			conI.add(I);
			conII.add(II);
		}
		long mark = 0x01;
		for(int i = 0; i < n; i++){
			markS.add(mark);
			if(i == 62)
				mark = 0x01;
			else
				mark *= 2;
		}

		ArrayList<Integer> cut = new ArrayList<>();
		long I = 0x00;
		long II = 0x00;
		long SI = 0x00;
		long SII = 0x00;

		recursive(map, SI, SII, cut, 0, 0, 0, 0, I, II);
	}

	private static String stateKey(long si, long sii) {
		return si + ":" + sii;
	}
	
}
