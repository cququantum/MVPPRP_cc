package lrp.data;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.Scanner;

public class Instance {

    public String name;            // instance name;
    public int M;                  // park lot number;
    public int N;                  // base station number;
    public double[] D;             // capacity of the depot
    public double[] fCost;         // fix cost
    public double[] s;             // service time consumption for node i ( i \in all the node);
    public int[] q;                // 容量
    public double[] lng;           // longitude
    public double[] lat;           // latitude
    public double[][] t;           // time matrix;

    // heuristic
    public double T;                // duration of route
    public int Q;                   // capacity of vehicle capacity
    public double vFix;             // fix cost of vehicle;
    public double[] e;
    public double[] l;
    public double[][] d;

    // Parameter
    public Parameter para;

    // 为子问题生成数据
    public int[] map;

    // Function to calculate the distance between two points for integer costs (code 0)
    public static double calculateIntegerDistance(double xA, double yA, double xB, double yB) {
        double distance = Math.sqrt(Math.pow((xA - xB), 2) + Math.pow((yA - yB), 2));
        return distance;  // Multiply by 100 and truncate
    }

    // null construct
    public Instance(){}

    // for capacity depot and capacity vehicle
    public Instance (String path) throws FileNotFoundException {
        para = new Parameter();
        para.start_time = System.nanoTime();

        Scanner cin = new Scanner(new BufferedReader(new FileReader(path)));

        N = cin.nextInt();
        M = cin.nextInt();

        lng = new double[N + 2*M + 1];
        lat = new double[N + 2*M + 1];

        for(int i = N + 1; i < N + M + 1; i++){
            lng[i] = cin.nextDouble();
            lat[i] = cin.nextDouble();
        }

        for(int i = 1; i < N + 1; i++){
            lng[i] = cin.nextDouble();
            lat[i] = cin.nextDouble();
        }

        for(int i = N + M + 1; i < N + 2*M +1; i++){
            lng[i] = lng[i - M];
            lat[i] = lat[i - M];
        }

        Q = cin.nextInt();
        D = new double[N + M + 1];
        for(int i = N + 1; i < N + M + 1; i++){
            D[i] = cin.nextDouble();
        }

        q = new int[N + 2*M + 1];
        for(int i = 1; i < N + 1; i++){
            q[i] = cin.nextInt();
        }

        fCost = new double[N + M + 1];
        for(int i = N + 1; i < N + M + 1; i++){
            fCost[i] = cin.nextDouble();
        }

        vFix = cin.nextDouble();

        //  depot 的服务时间 ——> 车辆启用成本, 客户点服务时间为 0；
        s = new double[N + 2*M + 1];
        for(int i = N + 1; i < N + M + 1; i++){
            s[i] = vFix;
        }

        int type = cin.nextInt();

        // 计算距离矩阵
        t = new double[N + 2*M + 1][N + 2*M + 1];
        d = new double[N + 2*M + 1][N + 2*M + 1];
        for(int i = 0; i < N + 2 * M + 1; i++){
            for(int j = 0; j < N + 2 * M + 1; j++){
                if(i == j)
                    t[i][j] = d[i][j] = 0;
                else
                    t[i][j] = d[i][j] = Double.MAX_VALUE;

            }
        }

        for(int i = 1; i < N + 2 * M + 1; i++){
            for(int j = i + 1; j < N + 2 * M + 1; j++){
                t[i][j] = t[j][i] = d[i][j] = d[j][i] = calculateIntegerDistance(lat[i],lng[i],lat[j],lng[j]);
            }
        }

        if(type == 0){
//            vFix *= 100;
//            for(int i = N + 1; i < N + M + 1; i++){
//                s[i] = vFix;
//            }
//            for(int i = N + 1; i < N + M + 1; i++){
//                fCost[i] = fCost[i] * 100;
//            }
            for(int i = 1; i < N + 2 * M + 1; i++){
                for(int j = i + 1; j < N + 2 * M + 1; j++){
                    double temp = t[i][j];
                    t[i][j] = t[j][i] = d[i][j] = d[j][i] = Math.round(temp*100);
                }
            }
        }
    }

    // for  capacity depot and capacity vehicle & control instance size
    public Instance (String path,int _n) throws FileNotFoundException {
        para = new Parameter();
        para.start_time = System.nanoTime();

        Scanner cin = new Scanner(new BufferedReader(new FileReader(path)));

        int oldN = cin.nextInt();
        N = _n;
        M = cin.nextInt();

        lng = new double[N + 2*M + 1];
        lat = new double[N + 2*M + 1];

        for(int i = N + 1; i < N + M + 1; i++){
            lng[i] = cin.nextDouble();
            lat[i] = cin.nextDouble();
        }

        for(int i = 1; i < oldN + 1; i++){
            if(i < N + 1){
                lng[i] = cin.nextDouble();
                lat[i] = cin.nextDouble();
            }else{
                double lng_out = cin.nextDouble();
                double lat_out = cin.nextDouble();
            }
        }

        for(int i = N + M + 1; i < N + 2*M +1; i++){
            lng[i] = lng[i - M];
            lat[i] = lat[i - M];
        }

        Q = cin.nextInt();
        D = new double[N + M + 1];
        for(int i = N + 1; i < N + M + 1; i++){
            D[i] = cin.nextDouble();
        }

        q = new int[N + 2*M + 1];
        for(int i = 1; i < oldN + 1; i++){
            if(i < N + 1){
                q[i] = cin.nextInt();
            }else{
                cin.nextInt();
            }
        }

        fCost = new double[N + M + 1];
        for(int i = N + 1; i < N + M + 1; i++){
            fCost[i] = cin.nextDouble() * 100;
        }

        vFix = cin.nextDouble() * 100;

        //  depot 的服务时间 ——> 车辆启用成本, 客户点服务时间为 0；
        s = new double[N + 2*M + 1];
        for(int i = N + 1; i < N + M + 1; i++){
            s[i] = vFix;
        }

        double type = cin.nextDouble();

        // 计算距离矩阵
        t = new double[N + 2*M + 1][N + 2*M + 1];
        d = new double[N + 2*M + 1][N + 2*M + 1];
        for(int i = 0; i < N + 2 * M + 1; i++){
            for(int j = 0; j < N + 2 * M + 1; j++){
                t[i][j] = d[i][j] = Double.MAX_VALUE;
            }
        }

        for(int i = 1; i < N + 2 * M + 1; i++){
            for(int j = i + 1; j < N + 2 * M + 1; j++){
                t[i][j] = t[j][i] = d[i][j] = d[j][i] = calculateIntegerDistance(lat[i],lng[i],lat[j],lng[j]);
            }
        }

        if(type == 0){
//            vFix *= 100;
//            for(int i = N + 1; i < N + M + 1; i++){
//                s[i] = vFix;
//            }
//            for(int i = N + 1; i < N + M + 1; i++){
//                fCost[i] = fCost[i] * 100;
//            }
            for(int i = 1; i < N + 2 * M + 1; i++){
                for(int j = i + 1; j < N + 2 * M + 1; j++){
                    double temp = t[i][j];
                    t[i][j] = t[j][i] = d[i][j] = d[j][i] = Math.round(temp*100);
                }
            }
        }

    }

    // copy instance
    public Instance copy(Instance _inst){
        Instance instance = new Instance();
        instance.para = _inst.para;
        instance.name = _inst.name;
        instance.M = _inst.M;
        instance.N = _inst.N;
        instance.D = _inst.D.clone();
        instance.Q = _inst.Q;
        instance.vFix = _inst.vFix;
        instance.fCost = _inst.fCost.clone();
        instance.s = _inst.s.clone();
        instance.q = _inst.q.clone();
        instance.lng = _inst.lng.clone();
        instance.lat = _inst.lat.clone();
        instance.t = new double[_inst.t.length][];
        instance.d = new double[_inst.d.length][];
        for(int i = 0; i < _inst.t.length; i++){
            instance.t[i] = _inst.t[i].clone();
            instance.d[i] = _inst.d[i].clone();
        }
        return instance;
    }

    // 为每个子问题 设定一个 instance
    public Instance set_instance(Instance _inst, ArrayList<Integer> seq){
        Instance instance = copy(_inst);
        instance.N = seq.size()-1;
        instance.map = new int[instance.N + 1];
        for(int i = 0; i < seq.size(); i++){
            instance.map[i] = seq.get(i);
        }
        instance.s = new double[instance.N];
        instance.q = new int[instance.N];
        for(int j = 0; j < instance.N; j++){
            instance.s[j] = _inst.s[instance.map[j]];
            instance.q[j] = _inst.q[instance.map[j]];
        }
        instance.t = new double[instance.N][instance.N];
        instance.d = new double[instance.N][instance.N];
        for(int i = 0; i < instance.N; i++){
            for(int j = 0; j < instance.N; j++){
                instance.t[i][j] = _inst.t[instance.map[i]][instance.map[j]];
                instance.d[i][j] = _inst.d[instance.map[i]][instance.map[j]];
            }
        }
        return instance;
    }

}
