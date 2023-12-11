//Basic PSO program
using System;

namespace ParticleSwarmOptimization
{
    public struct Parameters
    {
        public double omega, phip, phig;

        public Parameters(double omega, double phip, double phig) : this()
        {
            this.omega = omega;
            this.phip = phip;
            this.phig = phig;
        }
    }

    public struct State
    {
        public int iter;
        public double[] gbpos;
        public double gbval;
        public double[] min;
        public double[] max;
        public Parameters parameters;
        public double[][] pos;
        public double[][] vel;
        public double[][] bpos;
        public double[] bval;
        public int nParticles;
        public int nDims;

        public State(int iter, double[] gbpos, double gbval, double[] min, double[] max, Parameters parameters, double[][] pos, double[][] vel, double[][] bpos, double[] bval, int nParticles, int nDims) : this()
        {
            this.iter = iter;
            this.gbpos = gbpos;
            this.gbval = gbval;
            this.min = min;
            this.max = max;
            this.parameters = parameters;
            this.pos = pos;
            this.vel = vel;
            this.bpos = bpos;
            this.bval = bval;
            this.nParticles = nParticles;
            this.nDims = nDims;
        }

        public void Report(string testfunc)
        {
            Console.WriteLine("Test Function        : {0}", testfunc);
            Console.WriteLine("Iterations           : {0}", iter);
            Console.WriteLine("Global Best Position : {0}", string.Join(", ", gbpos));
            Console.WriteLine("Global Best Value    : {0}", gbval);
        }
    }

    class Program
    {
        public static State PsoInit(double[] min, double[] max, Parameters parameters, int nParticles)
        {
            var nDims = min.Length;
            double[][] pos = new double[nParticles][];
            for (int i = 0; i < nParticles; i++)
            {
                pos[i] = new double[min.Length];
                min.CopyTo(pos[i], 0);
            }
            double[][] vel = new double[nParticles][];
            for (int i = 0; i < nParticles; i++)
            {
                vel[i] = new double[nDims];
            }
            double[][] bpos = new double[nParticles][];
            for (int i = 0; i < nParticles; i++)
            {
                bpos[i] = new double[min.Length];
                min.CopyTo(bpos[i], 0);
            }
            double[] bval = new double[nParticles];
            for (int i = 0; i < nParticles; i++)
            {
                bval[i] = double.PositiveInfinity;
            }
            int iter = 0;
            double[] gbpos = new double[nDims];
            for (int i = 0; i < nDims; i++)
            {
                gbpos[i] = double.PositiveInfinity;
            }
            double gbval = double.PositiveInfinity;

            return new State(iter, gbpos, gbval, min, max, parameters, pos, vel, bpos, bval, nParticles, nDims);
        }

        static Random r = new Random();

        public static State Pso(Func<double[], double> fn, State y)
        {
            var p = y.parameters;
            double[] v = new double[y.nParticles];
            double[][] bpos = new double[y.nParticles][];
            for (int i = 0; i < y.nParticles; i++)
            {
                bpos[i] = new double[y.min.Length];
                y.min.CopyTo(bpos[i], 0);
            }
            double[] bval = new double[y.nParticles];
            double[] gbpos = new double[y.nDims];
            double gbval = double.PositiveInfinity;
            for (int j = 0; j < y.nParticles; j++)
            {
                // evaluate
                v[j] = fn.Invoke(y.pos[j]);
                // update
                if (v[j] < y.bval[j])
                {
                    y.pos[j].CopyTo(bpos[j], 0);
                    bval[j] = v[j];
                }
                else
                {
                    y.bpos[j].CopyTo(bpos[j], 0);
                    bval[j] = y.bval[j];
                }
                if (bval[j] < gbval)
                {
                    gbval = bval[j];
                    bpos[j].CopyTo(gbpos, 0);
                }
            }
            double rg = r.NextDouble();
            double[][] pos = new double[y.nParticles][];
            double[][] vel = new double[y.nParticles][];
            for (int i = 0; i < y.nParticles; i++)
            {
                pos[i] = new double[y.nDims];
                vel[i] = new double[y.nDims];
            }
            for (int j = 0; j < y.nParticles; j++)
            {
                // migrate
                double rp = r.NextDouble();
                bool ok = true;
                for (int k = 0; k < y.nDims; k++)
                {
                    vel[j][k] = 0.0;
                    pos[j][k] = 0.0;
                }
                for (int k = 0; k < y.nDims; k++)
                {
                    vel[j][k] = p.omega * y.vel[j][k] +
                                p.phip * rp * (bpos[j][k] - y.pos[j][k]) +
                                p.phig * rg * (gbpos[k] - y.pos[j][k]);
                    pos[j][k] = y.pos[j][k] + vel[j][k];
                    ok = ok && y.min[k] < pos[j][k] && y.max[k] > pos[j][k];
                }
                if (!ok)
                {
                    for (int k = 0; k < y.nDims; k++)
                    {
                        pos[j][k] = y.min[k] + (y.max[k] - y.min[k]) * r.NextDouble();
                    }
                }
            }
            Console.WriteLine("{0}\t{1}\t{2}\t{3}\t{4}\t{5}\t{6}\t{7}\t{8}\t{9}\t{10}\t{11}\t{12}\t{13}\t{14}\t{15}",
            //Console.WriteLine("{0}\t{1}\t{2}\t{3}\t{4}\t{5}\t{6}\t{7}\t{8}\t{9}\t{10}\t{11}\t{12}\t{13}\t{14}\t{15}\t{16}\t{17}\t{18}\t{19}",
                              //Console.WriteLine("{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12},{13},{14},{15},{16},{17},{18},{19}",
                              pos[0][0], pos[0][1],
                              pos[1][0], pos[1][1],
                              pos[2][0], pos[2][1],
                              pos[3][0], pos[3][1],
                              pos[4][0], pos[4][1],
                              pos[5][0], pos[5][1],
                              pos[6][0], pos[6][1],
                              //pos[7][0], pos[7][1],
                              //pos[8][0], pos[8][1],
                              pos[9][0], pos[9][1]);
            var iter = 1 + y.iter;
            return new State(iter, gbpos, gbval, y.min, y.max, y.parameters, pos, vel, bpos, bval, y.nParticles, y.nDims);
        }

        public static State Iterate(Func<double[], double> fn, int n, State y)
        {
            State r = y;
            if (n == int.MaxValue)
            {
                State old = y;
                while (true)
                {
                    r = Pso(fn, r);
                    if (r.Equals(old)) break;
                    old = r;
                }
            }
            else
            {
                for (int i = 0; i < n; i++)
                {
                    r = Pso(fn, r);
                }
            }
            return r;
        }

        public static double Mccormick(double[] x)
        {
            var a = x[0];
            var b = x[1];
            return Math.Sin(a + b) + (a - b) * (a - b) + 1.0 + 2.5 * b - 1.5 * a;
        }

        public static double Michalewicz(double[] x)
        {
            int m = 10;
            int d = x.Length;
            double sum = 0.0;
            for (int i = 1; i < d; i++)
            {
                var j = x[i - 1];
                var k = Math.Sin(i * j * j / Math.PI);
                sum += Math.Sin(j) * Math.Pow(k, 2.0 * m);
            }
            return -sum;
        }
        public static double Himmel(double[] x)
        {
            double f = -Math.Pow(Math.Pow(x[0] / 5, 2) + x[1] / 5 - 11, 2) - Math.Pow(Math.Pow(x[1] / 5, 2) + x[0] / 5 - 12, 2);
            return f;
        }

        static void Main(string[] args)
        {
            double vvv = -10.15 % 0.001;
            var state = PsoInit(
                new double[] { -18, -25 },
                new double[] { 25, 10 },
                //new double[] { 1, 1 },       //step
                new Parameters(0, 0.6, 0.3),
                50  //np
                );
            state = Iterate(Himmel, 50, state);    //$PARALLEL
            state.Report("Himmel");
            Console.WriteLine("f(-25, 0)        : {0}", Himmel(new double[] { -25, 0 }));
            /*var state = PsoInit(
                new double[] { -1.5, -3.0 },//min
                new double[] { 4.0, 4.0 }, //max
                new Parameters(0.0, 0.6, 0.3),   //param of model omega;phip;phig
                100				//number of particles
                );
            state = Iterate(Mccormick, 40, state);    //$PARALLEL 40 - number of iterations
            state.Report("McCormick");
            Console.WriteLine("f(-.54719, -1.54719) : {0}", Mccormick(new double[] { -.54719, -1.54719 }));
            Console.WriteLine();*/

            /*state = PsoInit(
                new double[] { -0.0, -0.0 },
                new double[] { Math.PI, Math.PI },
                new Parameters(0.3, 0.3, 0.3),
                1000
                );
            state = Iterate(Michalewicz, 30, state);
            state.Report("Michalewicz (2D)");
            Console.WriteLine("f(2.20, 1.57)        : {0}", Michalewicz(new double[] { 2.20, 1.57 }));*/
        }
    }
}