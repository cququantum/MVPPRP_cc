package lrp.labelsetting;

public class Label implements Comparable<Label> {

	/**
	 *  label id
	 */
	public int id;
	/**
	 * cumulative reduce cost
	 */
	public double rc;
	/**
	 * cumulative capacity
	 */
	public int q;
	/**
	 * cumulative time consumption (cost)
	 */
	public double t;
	/**
	 * set of forbidden customers
	 */
	public long VI;
	public long VII;
	/**
	 *  set of customers has been visited
	 */
	public long UI;
	public long UII;
	/**
	 *  set of customers has been visited multiple times
	 */
	public long WI;
	public long WII;
	/**
	 * 记录当前label 只有一个点在 cut 中的 cut id
	 */
	public int[] sr_set;
	/**
	 * 记录 当前label 只有一个点在 cut中的 cut 对应的 dual
	 * 表示 未来可能的 subset row 对偶值 潜在的成本，在占优的时候需要用；
	 */
	public double src;
	/**
	 * 记录该 label 真正违反的 subset row 的 id
	 * 访问两个点 和 访问三个点是一样的；
	 */
	public int[] sr_cov;
	/**
	 * father label
	 */
	public Label father;

	@Override
	public int compareTo(Label x)
	{
		if(rc < x.rc)
		{
			return -1;
		}
		else if(rc > x.rc)
		{
			return 1;
		}
		
		return 0;
	}
	
	public void clear()
	{
		if(sr_set != null)
		{
			sr_set = null;
		}
	}
}
