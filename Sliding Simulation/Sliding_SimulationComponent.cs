using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using Rhino;
using Rhino.Geometry;
using Rhino.Geometry.Intersect;

using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;

namespace Sliding_Simulation
{
    public class Sliding_SimulationComponent : GH_Component
    {
        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public Sliding_SimulationComponent()
          : base("Sliding_SimulationComponent", "滑行物理模拟",
            "基于物理碰撞的滑行模拟器",
            "ZT Tools",
            "Physics")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
 
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            // 0. 障碍物 (List)
            pManager.AddBrepParameter("Obstacles", "Obs", "障碍物", GH_ParamAccess.list);
            // 1. 起点
            pManager.AddPointParameter("StartPt", "P", "起始点", GH_ParamAccess.item);
            // 2. 初速度
            pManager.AddVectorParameter("InitVel", "V", "初速度", GH_ParamAccess.item);
            // 3. 半径
            pManager.AddNumberParameter("Radius", "R", "半径", GH_ParamAccess.item);
            // 4. 摩擦系数
            pManager.AddNumberParameter("Friction", "F", "摩擦系数", GH_ParamAccess.item);
            // 5. 自动落点
            pManager.AddBooleanParameter("AutoDrop", "Drop", "自动贴地", GH_ParamAccess.item);
            // 6. 时长
            pManager.AddNumberParameter("Duration", "T", "模拟时长", GH_ParamAccess.item);
            // 7. 步长
            pManager.AddNumberParameter("TimeStep", "Step", "时间步长", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            // 对应 ref object msg
            pManager.AddTextParameter("Message", "msg", "中文诊断报告", GH_ParamAccess.item);

            // 对应 ref object Xforms (List)
            pManager.AddTransformParameter("Xforms", "X", "运动变换矩阵", GH_ParamAccess.list);

            // 对应 ref object Traj (Curve)
            pManager.AddCurveParameter("Traj", "Traj", "运动轨迹", GH_ParamAccess.item);

            // 对应 ref object Vectors (List)
            pManager.AddVectorParameter("Vectors", "V", "速度向量", GH_ParamAccess.list);

            // 对应 ref object Speeds_Mps (List)
            pManager.AddNumberParameter("Speeds", "S", "速度(m/s)", GH_ParamAccess.list);

            // 对应 ref object G_Forces (List)
            pManager.AddNumberParameter("G_Forces", "G", "过载(G)", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>


        protected override void SolveInstance(IGH_DataAccess DA)
        {
            // --- A. 定义变量接收输入 ---
            List<Brep> Obstacles = new List<Brep>();
            Point3d StartPt = Point3d.Unset;
            Vector3d InitVel = Vector3d.Unset;
            double Radius = 0;
            double Friction = 0;
            bool AutoDrop = false;
            double Duration = 0;
            double TimeStep = 0;

            // --- B. 获取输入数据 (注意顺序要和 InputParams 一致) ---
            if (!DA.GetDataList(0, Obstacles)) return;
            if (!DA.GetData(1, ref StartPt)) return;
            if (!DA.GetData(2, ref InitVel)) return;
            if (!DA.GetData(3, ref Radius)) return;
            if (!DA.GetData(4, ref Friction)) return;
            if (!DA.GetData(5, ref AutoDrop)) return;
            if (!DA.GetData(6, ref Duration)) return;
            if (!DA.GetData(7, ref TimeStep)) return;

            // --- C. 核心逻辑 (原 RunScript 内容) ---
            if (Obstacles == null || Obstacles.Count == 0) return;

            StringBuilder log = new StringBuilder();
            int hardResetCount = 0;
            int coldStartCount = 0;
            double maxSpeedRec = 0.0;
            double maxGRec = 0.0;
            double totalDistRec = 0.0;

            // [v38.2] 停止时间变量
            double stopTimeRecorded = -1.0;

            // 物理常数
            double gVal = 9810.0;
            Vector3d gravityVec = new Vector3d(0, 0, -gVal);
            int solverIterations = 5;
            double airDragCoeff = (Friction < 1e-6) ? 0.0 : 0.00002;

            // 初始位置处理
            Point3d pos = StartPt;
            if (AutoDrop)
            {
                Ray3d dropRay = new Ray3d(StartPt, -Vector3d.ZAxis);
                Point3d[] hits = Intersection.RayShoot(dropRay, Obstacles.Select(b => (GeometryBase)b), 1);
                if (hits != null && hits.Length > 0)
                {
                    pos = hits[0] + Vector3d.ZAxis * (Radius * 1.05);
                    Vector3d dropVel = Vector3d.Zero;
                    for (int k = 0; k < 200; k++)
                    {
                        dropVel += gravityVec * TimeStep;
                        Point3d nextD = pos + dropVel * TimeStep;
                        SolvePositionAccumulated(Obstacles, ref nextD, Radius, 8);
                        dropVel *= 0.5; pos = nextD;
                        if (dropVel.Length < 0.1 && k > 20) break;
                    }
                }
            }

            // 循环准备
            int steps = (int)(Duration / TimeStep);
            var xformList = new List<Transform>();
            var speedList = new List<double>();
            var vecList = new List<Vector3d>();
            var gForceList = new List<double>();
            var pathPts = new List<Point3d>();

            Vector3d vel = InitVel * 1000.0;

            double initialKE = 0.5 * vel.SquareLength;
            double initialPE = gVal * pos.Z;
            double totalMechanicalEnergy = initialKE + initialPE;
            double accumulatedLossWork = 0.0;

            Vector3d smoothVisualNormal = Vector3d.ZAxis;
            Vector3d smoothVisualFwd = vel.Length > 0.1 ? vel : Vector3d.XAxis;
            bool wasColliding = false;

            xformList.Add(Transform.Identity);
            speedList.Add(vel.Length * 0.001);
            vecList.Add(vel * 0.001);
            gForceList.Add(1.0);
            pathPts.Add(pos);

            Point3d pProbe = pos;
            Vector3d nProbe = SolvePositionAccumulated(Obstacles, ref pProbe, Radius, 2);
            if (nProbe.Length > 0.001) { smoothVisualNormal = nProbe; nProbe.Unitize(); wasColliding = true; }
            Vector3d fwd0 = vel.Length > 0.1 ? vel : Vector3d.XAxis;
            Plane basePlane = new Plane(StartPt, fwd0, smoothVisualNormal);

            // === 物理循环 ===
            for (int i = 0; i < steps; i++)
            {
                Vector3d oldVel = vel;
                Point3d oldPos = pos;

                // 牛顿预测
                Vector3d newtonianVel = vel + gravityVec * TimeStep;
                Point3d nextPos = pos + newtonianVel * TimeStep;

                // 位置求解
                Vector3d contactNormal = SolvePositionAccumulated(Obstacles, ref nextPos, Radius, solverIterations);
                bool isColliding = (contactNormal.Length > 0.001);
                if (isColliding) contactNormal.Unitize(); else contactNormal = Vector3d.ZAxis;

                bool isImpact = isColliding && !wasColliding;

                // 几何投影
                if (isColliding)
                {
                    double vProj = newtonianVel * contactNormal;
                    if (vProj < 0) newtonianVel -= contactNormal * vProj;
                }

                double stepDist = pos.DistanceTo(nextPos);
                totalDistRec += stepDist;

                // 能量账本
                if (isImpact)
                {
                    double currentKE = 0.5 * newtonianVel.SquareLength;
                    double currentPE = gVal * nextPos.Z;
                    totalMechanicalEnergy = currentKE + currentPE;
                    accumulatedLossWork = 0.0;
                }
                else if (isColliding)
                {
                    double frictionWork = 0;
                    double normalForce = Math.Abs(gravityVec * contactNormal);
                    frictionWork = (normalForce * Friction) * stepDist;
                    double currentSpeed = vel.Length;
                    double dragWork = (airDragCoeff * currentSpeed * currentSpeed) * stepDist;
                    accumulatedLossWork += (frictionWork + dragWork);
                }
                else
                {
                    double currentSpeed = vel.Length;
                    double dragWork = (airDragCoeff * currentSpeed * currentSpeed) * stepDist;
                    accumulatedLossWork += dragWork;
                }

                double currentPE_Calc = gVal * nextPos.Z;
                double targetTotalKE = totalMechanicalEnergy - currentPE_Calc - accumulatedLossWork;
                if (targetTotalKE < 0) targetTotalKE = 0;
                double targetSpeed = Math.Sqrt(2.0 * targetTotalKE);

                // 速度更新与特殊情况处理
                if (isColliding && Friction < 1e-6)
                {
                    Vector3d slopeDir = gravityVec - contactNormal * (gravityVec * contactNormal);
                    double slopeAccel = slopeDir.Length;
                    if (slopeDir.Length > 1e-6) slopeDir.Unitize();

                    bool isMovingUp = (vel * slopeDir < 0);
                    double speedThreshold = slopeAccel * TimeStep * 2.0;

                    if (isMovingUp && targetSpeed < speedThreshold)
                    {
                        vel = Vector3d.Zero;
                        totalMechanicalEnergy = currentPE_Calc + accumulatedLossWork;
                        hardResetCount++;
                    }
                    else if (vel.Length < 1e-9 && slopeAccel > 1e-3)
                    {
                        vel = slopeDir * (slopeAccel * TimeStep);
                        totalMechanicalEnergy += 0.5 * vel.SquareLength;
                        coldStartCount++;
                    }
                    else
                    {
                        Vector3d baseDir = newtonianVel;
                        if (baseDir.Length > 1e-6) baseDir.Unitize();
                        else baseDir = (vel.Length > 0.1) ? vel / vel.Length : Vector3d.XAxis;

                        Vector3d sideDir = Vector3d.CrossProduct(contactNormal, slopeDir);
                        if (sideDir.Length > 1e-6) sideDir.Unitize();
                        double vSide = newtonianVel * sideDir;
                        if (Math.Abs(vSide) < 0.5)
                        {
                            baseDir = baseDir - sideDir * (baseDir * sideDir);
                            if (baseDir.Length > 1e-6) baseDir.Unitize();
                        }
                        vel = baseDir * targetSpeed;
                    }
                }
                else
                {
                    if (isColliding)
                    {
                        Vector3d slopeDir = gravityVec - contactNormal * (gravityVec * contactNormal);
                        if (slopeDir.Length > 1e-6) slopeDir.Unitize();
                        Vector3d sideDir = Vector3d.CrossProduct(contactNormal, slopeDir);
                        if (sideDir.Length > 1e-6) sideDir.Unitize();
                        Vector3d baseDir = newtonianVel;
                        if (baseDir.Length > 1e-6) baseDir.Unitize();

                        double vSide = newtonianVel * sideDir;
                        if (Math.Abs(vSide) < 0.5) baseDir = (baseDir - sideDir * (baseDir * sideDir));
                        if (baseDir.Length > 1e-6) baseDir.Unitize();

                        vel = baseDir * targetSpeed;

                        if (Friction > 1e-6)
                        {
                            double vSideFinal = vel * sideDir;
                            Vector3d vSlopeFinalVec = vel - sideDir * vSideFinal;
                            vel = vSlopeFinalVec + sideDir * (vSideFinal * 0.999);
                        }

                        double slopeKE = 0.5 * Math.Pow(vel * slopeDir, 2);
                        Vector3d slopeG = gravityVec - contactNormal * (gravityVec * contactNormal);
                        if (Friction > 1e-6 && targetSpeed < 5.0 && slopeKE < 0.1)
                        {
                            double slopeForce = slopeG.Length;
                            if (slopeForce < gVal * Friction) vel = Vector3d.Zero;
                        }
                    }
                    else
                    {
                        vel = newtonianVel;
                        if (vel.Length > 0.001 && targetTotalKE > 0)
                        {
                            double airTargetSpeed = Math.Sqrt(2.0 * targetTotalKE);
                            double ratio = airTargetSpeed / vel.Length;
                            if (ratio < 0.95 || ratio > 1.05) { vel.Unitize(); vel *= airTargetSpeed; }
                        }
                    }
                }

                if (!nextPos.IsValid || double.IsNaN(nextPos.X)) break;
                wasColliding = isColliding;

                // [v38.2] 停止监测逻辑
                if (vel.Length < 1e-6 && isColliding)
                {
                    if (stopTimeRecorded < 0) stopTimeRecorded = i * TimeStep;
                }
                else
                {
                    stopTimeRecorded = -1.0;
                }

                // 统计
                if (vel.Length > maxSpeedRec) maxSpeedRec = vel.Length;
                Vector3d accel = (vel - oldVel) / TimeStep;
                Vector3d netAccel = accel - gravityVec;
                double gForce = netAccel.Length / gVal;
                if (gForce > maxGRec) maxGRec = gForce;
                if (gForceList.Count > 0) gForce = gForce * 0.1 + gForceList[gForceList.Count - 1] * 0.9;

                pos = nextPos;

                Vector3d targetVisualNormal = isColliding ? contactNormal : Vector3d.ZAxis;
                smoothVisualNormal = smoothVisualNormal * 0.9 + targetVisualNormal * 0.1;
                if (smoothVisualNormal.Length > 1e-6) smoothVisualNormal.Unitize(); else smoothVisualNormal = Vector3d.ZAxis;

                Vector3d targetFwd = vel;
                if (targetFwd.Length < 10.0) targetFwd = smoothVisualFwd;
                smoothVisualFwd = smoothVisualFwd * 0.9 + targetFwd * 0.1;
                if (smoothVisualFwd.Length > 1e-6) smoothVisualFwd.Unitize(); else smoothVisualFwd = Vector3d.XAxis;

                Plane currentPlane = new Plane(pos, smoothVisualFwd, smoothVisualNormal);
                if (!currentPlane.IsValid) currentPlane = Plane.WorldXY;
                Transform xform = Transform.PlaneToPlane(basePlane, currentPlane);

                xformList.Add(xform);
                speedList.Add(vel.Length * 0.001);
                vecList.Add(vel * 0.001);
                gForceList.Add(gForce);

                if (pathPts.Count == 0 || pos.DistanceTo(pathPts[pathPts.Count - 1]) > 0.1) pathPts.Add(pos);
                if (pos.Z < -50000) break;
            }

            // --- D. 生成报告 ---
            log.AppendLine("╔══════════════════════════════════╗");
            log.AppendLine("║   🎱  物理模拟引擎 v38.2 CN      ║");
            log.AppendLine("╚══════════════════════════════════╝");
            log.AppendLine("");
            log.AppendLine($"[ 参数配置 ]");
            log.AppendLine($"• 模拟时长   : {Duration:F2} 秒");
            log.AppendLine($"• 时间步长   : {TimeStep:F4} 秒");
            log.AppendLine($"• 摩擦系数   : {Friction:F4}");
            log.AppendLine($"• 空气阻力   : {airDragCoeff:F6}");
            log.AppendLine("");
            log.AppendLine($"[ 统计数据 ]");
            log.AppendLine($"• 计算总帧数 : {steps}");
            log.AppendLine($"• 运动总里程 : {totalDistRec / 1000.0:F2} 米");
            log.AppendLine($"• 最高速度   : {maxSpeedRec / 1000.0:F2} 米/秒");
            log.AppendLine($"• 最大过载(G): {maxGRec:F2} G");
            log.AppendLine("");
            log.AppendLine($"[ 最终状态 ]");
            log.AppendLine($"• 坐标位置   : {pos.X:F1}, {pos.Y:F1}, {pos.Z:F1}");
            log.AppendLine($"• 当前速度   : {(vel.Length / 1000.0):F3} 米/秒");

            // [v38.2] 停止报告
            if (stopTimeRecorded >= 0)
                log.AppendLine($"• 停止时间   : {stopTimeRecorded:F2} 秒 🛑");
            else
                log.AppendLine($"• 停止时间   : (未完全静止) ⏳");

            log.AppendLine($"• 运动状态   : {(wasColliding ? "🚗 贴地滑行" : "✈️ 空中飞行")}");
            log.AppendLine("");
            log.AppendLine($"[ 能量审计 ] (单位: 约 J/kg)");
            double finalKE = 0.5 * (vel.Length / 1000.0) * (vel.Length / 1000.0);
            double finalPE = (gVal / 1000.0) * (pos.Z / 1000.0);
            double loss = accumulatedLossWork / 1000000.0;
            log.AppendLine($"• 当前动能 (KE) : {finalKE:F4}");
            log.AppendLine($"• 当前势能 (PE) : {finalPE:F4}");
            log.AppendLine($"• 热能损耗      : {loss:F4}");
            log.AppendLine($"• 系统总能      : {(finalKE + finalPE + loss):F4}");
            log.AppendLine($"• 初始总能      : {(totalMechanicalEnergy / 1000000.0):F4}");
            log.AppendLine("");
            log.AppendLine($"[ 零摩擦诊断报告 ]");
            log.AppendLine($"• 强制归零触发次数 : {hardResetCount}");
            log.AppendLine($"• 冷启动触发次数   : {coldStartCount}");
            if (hardResetCount > 0) log.AppendLine("  (⚠️ 成功拦截了芝诺悖论，已强制执行反向运动)");
            else log.AppendLine("  (✅ 运动平顺，未触发临界点重置)");

            // --- E. 输出数据 (SetData) ---
            // 对应 OutputParams 顺序: msg, Xforms, Traj, Vectors, Speeds, G_Forces
            DA.SetData(0, log.ToString());
            DA.SetDataList(1, xformList);
            if (pathPts.Count > 1)
                DA.SetData(2, new PolylineCurve(pathPts));
            else
                DA.SetData(2, null);
            DA.SetDataList(3, vecList);
            DA.SetDataList(4, speedList);
            DA.SetDataList(5, gForceList);
        }

        /// <summary>
        /// Provides an Icon for every component that will be visible in the User Interface.
        /// Icons need to be 24x24 pixels.
        /// You can add image files to your project resources and access them like this:
        /// return Resources.IconForThisComponent;
        /// </summary>
        protected override System.Drawing.Bitmap Icon => Properties.Resources.sliding_simulation_icon1;

        /// <summary>
        /// Each component must have a unique Guid to identify it. 
        /// It is vital this Guid doesn't change otherwise old ghx files 
        /// that use the old ID will partially fail during loading.
        /// </summary>
        public override Guid ComponentGuid => new Guid("25874fdf-cd04-4f08-a7ba-513e884d8d73");

        private Vector3d SolvePositionAccumulated(List<Brep> breps, ref Point3d currentPos, double radius, int iterations)
        {
            Vector3d finalAccumulatedNormal = Vector3d.Zero;
            for (int k = 0; k < iterations; k++)
            {
                Vector3d iterTotalPush = Vector3d.Zero;
                Vector3d iterNormalSum = Vector3d.Zero;
                int contactCount = 0;

                for (int i = 0; i < breps.Count; i++)
                {
                    Brep brep = breps[i];
                    if (brep == null) continue;

                    Point3d tmpCp;
                    ComponentIndex ci;
                    double u, v;
                    Vector3d tmpNormal;

                    // 寻找最近点
                    bool found = brep.ClosestPoint(currentPos, out tmpCp, out ci, out u, out v, radius * 1.5, out tmpNormal);

                    if (found)
                    {
                        double dist = currentPos.DistanceTo(tmpCp);
                        // 判定碰撞
                        if (dist <= radius + 0.01)
                        {
                            Vector3d pushDir = currentPos - tmpCp;
                            if (pushDir.Length < 0.001) pushDir = tmpNormal;
                            pushDir.Unitize();

                            double penetration = radius - dist;
                            iterTotalPush += pushDir * penetration;
                            iterNormalSum += pushDir;
                            contactCount++;
                        }
                    }
                }

                if (contactCount > 0)
                {
                    // 施加推力修正位置
                    currentPos += iterTotalPush * 0.5;
                    finalAccumulatedNormal = iterNormalSum;
                }
                else
                {
                    break;
                }
            }
            return finalAccumulatedNormal;
        }

    }
}