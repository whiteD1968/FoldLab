// Folded Plate Toolkit - Structure: tension and compression (STRESS)
// Rhino 8 Grasshopper C# Script component. Model units: FEET.
//
// Finds the forces in the plates and draws them as tension and compression line diagrams
// roof: lines that follow the principal tension (red) and compression (blue)
// through every plate, closer together where the stress is higher, carried
// across the folds into the next plate. Also colours each fold line by the force
// along it, colours the plates by stress, and draws a plan diagram beside the model.
//
// Method (linear elastic shell analysis):
//   - each plate is split into small triangles that carry both in-plane force and
//     slab bending (flat shell: CST membrane + DKT plate bending); folds are rigid,
//     monolithic joints, as in cast concrete;
//   - self-weight (150 pcf at the Designer's slab) plus live load on plan;
//   - supports are pinned: the points given in Supports, or else every point of the
//     roof that touches the ground;
//   - solved exactly (banded Cholesky), so there is no "not converged".
// The lines follow the mid-surface (membrane) forces, which is what the diagram draws.
// The Report adds the slab bending: moments, face stresses and deflection.
// If the roof as supported can move without resisting (a mechanism), nothing is
// drawn and the Report says so.
//
// Inputs (an input left unwired takes its value from the control window, section 9):
//   Plates    : the Plates output of the Fold Designer
//   Supports  : optional points (List Access): pinned supports, e.g. column heads.
//               Each pins the nearest point of the roof. Empty = everything on the ground.
//   LiveLoad  : psf on plan (default 20)
//   Fc        : psi, concrete strength (default 4000)
//   Refine    : triangles per plate edge (default 3; 2 to 6; higher = finer, slower)
//   Spacing   : ft, base distance between lines where the stress is highest (0 = automatic)
//   KipsPerLine : kips each tension or compression line carries (default 4.8 = one #4 bar
//               at 24 ksi; 0 = lines spaced by stress only)
//   SupportMode : 0 automatic (ground pinned, column points on bearings), 1 pinned,
//               2 bearings (default 0)
//   Bake      : button; bakes the diagram to the layer Structure (as the window's button)
// Every setting works without the control window: unwired inputs fall back to the window
// when it is on the canvas, and to the defaults above when it is not.
// Outputs:
//   Tension, Compression : the lines, at the top face of the slab
//   Folds     : fold lines; FoldForce gives the stress along each (psi, + tension)
//   FoldForce : one value per fold line
//   Plan      : plan diagram beside the model: plates, lines and supports
//   Field     : the plates coloured by stress (red tension, blue compression)
//   Report    : loads, reactions, peak stresses, reinforcement, flags
//
// Drawing: with the control window in the file, the window draws everything in
// colour (section 9) and this component's own preview can be off.
#region Usings
using System;
using System.Collections;
using System.Collections.Generic;
using System.Text;
using Rhino;
using Rhino.Geometry;
using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
#endregion

public class Script_Instance : GH_ScriptInstance
{
  // ---------- shared helpers ----------
  private static double Num(object o, double fallback)
  {
    if (o == null) return fallback;
    if (o is GH_Number) return ((GH_Number)o).Value;
    if (o is GH_Integer) return ((GH_Integer)o).Value;
    if (o is string || o is GH_String || o is IGH_Goo)
    {
      // text from a Panel, e.g. "0.3"
      double r;
      string txt = o.ToString().Trim();
      if (double.TryParse(txt, System.Globalization.NumberStyles.Float,
          System.Globalization.CultureInfo.InvariantCulture, out r)) return r;
      return fallback;
    }
    try { return Convert.ToDouble(o); } catch { return fallback; }
  }

  // Mitered solid: every vertex moves to where the offset planes of its
  // neighbouring plates meet, so plates join cleanly at the folds.
  // Vertices listed in "ground" stay at their own height (flat bearing edge).
  private static Mesh Thicken(Mesh m, double t, HashSet<int> ground)
  {
    int nV = m.Vertices.Count;
    double[,] M = new double[nV, 6];   // xx, xy, xz, yy, yz, zz
    double[,] B = new double[nV, 3];
    var faceIdx = new List<int[]>();
    var faceN = new List<Vector3d>();
    for (int f = 0; f < m.Faces.Count; f++)
    {
      MeshFace mf = m.Faces[f];
      int[] idx = mf.IsQuad ? new int[] { mf.A, mf.B, mf.C, mf.D } : new int[] { mf.A, mf.B, mf.C };
      faceIdx.Add(idx);
      Vector3d n = Vector3d.CrossProduct(
        m.Vertices.Point3dAt(idx[1]) - m.Vertices.Point3dAt(idx[0]),
        m.Vertices.Point3dAt(idx[2]) - m.Vertices.Point3dAt(idx[0]));
      faceN.Add(n);
      if (!n.Unitize()) { faceN[faceN.Count - 1] = Vector3d.Zero; continue; }
      faceN[faceN.Count - 1] = n;
      for (int k = 0; k < idx.Length; k++)
      {
        int v = idx[k];
        M[v, 0] += n.X * n.X; M[v, 1] += n.X * n.Y; M[v, 2] += n.X * n.Z;
        M[v, 3] += n.Y * n.Y; M[v, 4] += n.Y * n.Z; M[v, 5] += n.Z * n.Z;
        B[v, 0] += n.X * t / 2.0; B[v, 1] += n.Y * t / 2.0; B[v, 2] += n.Z * t / 2.0;
      }
    }
    // Free edges: hold each corner on the plane of its own edge, so the end of the
    // slab is a clean flat cut and keeps its true thickness. Where two free edges meet
    // at a corner (the zigzag of a section at a free end) that plane is the one through
    // both edges; along a single straight free edge it is square to the plate.
    var edgeUse = new Dictionary<long, int>();
    foreach (int[] idx in faceIdx)
      for (int k = 0; k < idx.Length; k++)
      {
        int p = idx[k], q = idx[(k + 1) % idx.Length];
        long key = (long)Math.Min(p, q) * nV + Math.Max(p, q);
        int n0; edgeUse.TryGetValue(key, out n0); edgeUse[key] = n0 + 1;
      }
    var freeDirs = new List<Vector3d>[nV];
    var freeNorm = new Vector3d[nV];
    for (int f = 0; f < faceIdx.Count; f++)
    {
      int[] idx = faceIdx[f];
      for (int k = 0; k < idx.Length; k++)
      {
        int p = idx[k], q = idx[(k + 1) % idx.Length];
        long key = (long)Math.Min(p, q) * nV + Math.Max(p, q);
        if (edgeUse[key] != 1) continue;
        Vector3d e = m.Vertices.Point3dAt(q) - m.Vertices.Point3dAt(p);
        if (!e.Unitize()) continue;
        if (freeDirs[p] == null) freeDirs[p] = new List<Vector3d>();
        if (freeDirs[q] == null) freeDirs[q] = new List<Vector3d>();
        freeDirs[p].Add(e); freeDirs[q].Add(-e);
        freeNorm[p] += faceN[f]; freeNorm[q] += faceN[f];
      }
    }
    for (int v = 0; v < nV; v++)
    {
      if (freeDirs[v] == null) continue;
      Vector3d wn = Vector3d.Zero;
      if (freeDirs[v].Count >= 2) wn = Vector3d.CrossProduct(freeDirs[v][0], freeDirs[v][1]);
      if (wn.Length < 0.05)          // the two free edges run straight on: cut square to the plate
      {
        Vector3d nf = freeNorm[v]; nf.Unitize();
        wn = Vector3d.CrossProduct(freeDirs[v][0], nf);
      }
      if (!wn.Unitize()) continue;
      // Both free edges in the same plate (an outer corner of the roof): the plane
      // through them is the plate itself, and holding the corner in it would squash
      // the slab to nothing. The plate normal alone already gives a square corner.
      { Vector3d nf = freeNorm[v]; if (nf.Unitize() && Math.Abs(wn * nf) > 0.9) continue; }
      double wgt = 100.0;
      M[v, 0] += wgt * wn.X * wn.X; M[v, 1] += wgt * wn.X * wn.Y; M[v, 2] += wgt * wn.X * wn.Z;
      M[v, 3] += wgt * wn.Y * wn.Y; M[v, 4] += wgt * wn.Y * wn.Z; M[v, 5] += wgt * wn.Z * wn.Z;
    }
    var off = new Vector3d[nV];
    double eps = 1e-6;
    for (int v = 0; v < nV; v++)
    {
      double zz = M[v, 5];
      if (ground != null && ground.Contains(v)) zz += 100.0;   // hold the height
      double a = M[v, 0] + eps, b = M[v, 1], c = M[v, 2];
      double d = M[v, 3] + eps, e = M[v, 4], g = zz + eps;
      double r0 = B[v, 0], r1 = B[v, 1], r2 = B[v, 2];
      double det = a * (d * g - e * e) - b * (b * g - e * c) + c * (b * e - d * c);
      if (Math.Abs(det) < 1e-30) { off[v] = Vector3d.Zero; continue; }
      double x = (r0 * (d * g - e * e) - b * (r1 * g - e * r2) + c * (r1 * e - d * r2)) / det;
      double y = (a * (r1 * g - e * r2) - r0 * (b * g - e * c) + c * (b * r2 - r1 * c)) / det;
      double z = (a * (d * r2 - r1 * e) - b * (b * r2 - r1 * c) + r0 * (b * e - d * c)) / det;
      off[v] = new Vector3d(x, y, z);
      // at very sharp folds the exact miter point runs far out and the solid
      // turns inside out; cap it at four half-thicknesses (a blunt miter)
      double lim = 4.0 * t;
      double ol = off[v].Length;
      if (ol > lim) off[v] = off[v] * (lim / ol);
    }

    Mesh s = new Mesh();
    for (int v = 0; v < nV; v++) s.Vertices.Add(m.Vertices.Point3dAt(v) - off[v]);
    for (int v = 0; v < nV; v++) s.Vertices.Add(m.Vertices.Point3dAt(v) + off[v]);
    var count = new Dictionary<long, int>();
    foreach (int[] idx in faceIdx)
    {
      int c = idx.Length;
      if (c == 4)
      {
        s.Faces.AddFace(idx[3], idx[2], idx[1], idx[0]);
        s.Faces.AddFace(idx[0] + nV, idx[1] + nV, idx[2] + nV, idx[3] + nV);
      }
      else
      {
        s.Faces.AddFace(idx[2], idx[1], idx[0]);
        s.Faces.AddFace(idx[0] + nV, idx[1] + nV, idx[2] + nV);
      }
      for (int k = 0; k < c; k++)
      {
        int p = idx[k], q = idx[(k + 1) % c];
        long key = (long)Math.Min(p, q) * nV + Math.Max(p, q);
        int n0; count.TryGetValue(key, out n0); count[key] = n0 + 1;
      }
    }
    foreach (int[] idx in faceIdx)
    {
      int c = idx.Length;
      for (int k = 0; k < c; k++)
      {
        int p = idx[k], q = idx[(k + 1) % c];
        long key = (long)Math.Min(p, q) * nV + Math.Max(p, q);
        if (count[key] == 1) s.Faces.AddFace(p, q, q + nV, p + nV);   // edge wall
      }
    }
    s.Compact();
    s.Unweld(0, true);            // flat-shade every plate so folds read as crisp edges
    s.Normals.ComputeNormals();
    return s;
  }

  // Sort interior creases into ridges (fold points toward the face normals)
  // and valleys. Coplanar edges are ignored.
  private static void Creases(Mesh m, List<Line> ridges, List<Line> valleys)
  {
    double tol = 1e-6;
    for (int e = 0; e < m.TopologyEdges.Count; e++)
    {
      int[] fs = m.TopologyEdges.GetConnectedFaces(e);
      if (fs == null || fs.Length != 2) continue;
      Line ln = m.TopologyEdges.EdgeLine(e);
      MeshFace f1 = m.Faces[fs[0]];
      Point3d a = m.Vertices.Point3dAt(f1.A);
      Point3d b = m.Vertices.Point3dAt(f1.B);
      Point3d c = m.Vertices.Point3dAt(f1.C);
      Vector3d n1 = Vector3d.CrossProduct(b - a, c - a);
      if (!n1.Unitize()) continue;
      MeshFace f2 = m.Faces[fs[1]];
      int[] idx = f2.IsQuad ? new int[] { f2.A, f2.B, f2.C, f2.D } : new int[] { f2.A, f2.B, f2.C };
      double best = 0.0; Point3d far = ln.From;
      for (int k = 0; k < idx.Length; k++)
      {
        Point3d q = m.Vertices.Point3dAt(idx[k]);
        double d = ln.DistanceTo(q, false);
        if (d > best) { best = d; far = q; }
      }
      double s = n1 * (far - ln.From);
      if (s < -tol * Math.Max(1.0, best)) ridges.Add(ln);
      else if (s > tol * Math.Max(1.0, best)) valleys.Add(ln);
    }
  }

  private static double AreaOf(Mesh m)
  {
    AreaMassProperties amp = AreaMassProperties.Compute(m);
    return amp == null ? 0.0 : amp.Area;
  }

  // The toolkit reads every length in the model as feet. Warn if the file is not in feet.
  private static string UnitNote()
  {
    RhinoDoc doc = RhinoDoc.ActiveDoc;
    if (doc == null || doc.ModelUnitSystem == UnitSystem.Feet) return null;
    return "This Rhino file is in " + doc.ModelUnitSystem.ToString().ToLower()
      + ", but the toolkit reads every length as feet. Set the file to feet (Units command, do not scale the model) or the sizes, slab thickness and quantities shown here are wrong.";
  }

  // ---------------- drawing (used when there is no control window) ----------------
  private object[] drawData = null;
  private BoundingBox drawBox = BoundingBox.Empty;

  public override BoundingBox ClippingBox { get { return WindowPresent() ? BoundingBox.Empty : drawBox; } }

  // The control window is present only if its component is on this canvas now. Its settings stay
  // in Rhino's memory after it is deleted, so the published data alone is not enough.
  private bool WindowPresent()
  {
    try
    {
      object id = AppDomain.CurrentDomain.GetData("FoldToolkit.WindowId");
      if (!(id is Guid) || GrasshopperDocument == null) return false;
      return GrasshopperDocument.FindObject((Guid)id, true) != null;
    }
    catch { return false; }
  }
  private bool lastBakeInput = false;

  public override void DrawViewportMeshes(IGH_PreviewArgs args)
  {
    if (WindowPresent() || drawData == null) return;
    try { Mesh fm = drawData[3] as Mesh; if (fm != null) args.Display.DrawMeshFalseColors(fm); } catch { }
  }

  public override void DrawViewportWires(IGH_PreviewArgs args)
  {
    if (WindowPresent() || drawData == null) return;
    try
    {
      int[] w = { 1, 2, 4 };
      var tb = drawData[0] as List<Polyline>[]; var cb = drawData[1] as List<Polyline>[];
      for (int b = 0; b < 3; b++)
      {
        foreach (Polyline pl in tb[b]) args.Display.DrawPolyline(pl, System.Drawing.Color.FromArgb(220, 30, 30), w[b]);
        foreach (Polyline pl in cb[b]) args.Display.DrawPolyline(pl, System.Drawing.Color.FromArgb(30, 80, 220), w[b]);
      }
      var fl = drawData[2] as Line[]; var fv = drawData[4] as double[];
      for (int i = 0; i < fl.Length; i++)
        args.Display.DrawLine(fl[i], fv[i] >= 0 ? System.Drawing.Color.FromArgb(200, 0, 0) : System.Drawing.Color.FromArgb(0, 60, 200), 2 + (int)Math.Round(5 * Math.Abs(fv[i])));
    }
    catch { }
  }

  // ---------------- helpers ----------------
  private static bool ToolOn(string key, bool fallback)
  {
    try
    {
      Dictionary<string, double> tools = AppDomain.CurrentDomain.GetData("FoldToolkit.Tools") as Dictionary<string, double>;
      double v;
      if (tools != null && tools.TryGetValue(key, out v)) return v > 0.5;
    }
    catch { }
    return fallback;
  }

  private Dictionary<string, double> win = null;
  private double W(string key, double fallback)
  {
    double v;
    if (win != null && win.TryGetValue(key, out v)) return v;
    return fallback;
  }

  private static Mesh AsMesh(object o)
  {
    if (o is Mesh) return (Mesh)o;
    if (o is GH_Mesh) return ((GH_Mesh)o).Value;
    return null;
  }

  private static Point3d AsPoint(object o, out bool ok)
  {
    ok = true;
    if (o is Point3d) return (Point3d)o;
    if (o is GH_Point) return ((GH_Point)o).Value;
    ok = false; return Point3d.Origin;
  }

  // principal stresses of a 2D tensor: s1 >= s2, angle of s1
  private static void Principal(double sx, double sy, double txy, out double s1, out double s2, out double ang)
  {
    double c = 0.5 * (sx + sy), r = Math.Sqrt(0.25 * (sx - sy) * (sx - sy) + txy * txy);
    s1 = c + r; s2 = c - r;
    ang = 0.5 * Math.Atan2(2.0 * txy, sx - sy);
  }

  private class Plate
  {
    public int Face;
    public Point3d O; public Vector3d E1, E2, N;
    public int M;                                   // 3 or 4 corners
    public Point2d[] C;                             // corners in plate coordinates
    public int[] Nodes;                             // global node of each local node
    public Point2d[] P;                             // local node positions
    public List<int[]> Tri = new List<int[]>();     // local triangles
    public List<int> TriElem = new List<int>();     // global element of each local triangle
    public int[][] SideNodes;                       // global nodes along each side, corner j to j+1
    public int[] SideNb;                            // neighbouring plate across each side, -1 if none
    public bool[] SideLoad;                         // side takes the load across the plate (fold or supported)
    public double[] Sx, Sy, Txy;                    // stress at local nodes (ksf), plate axes
    public Point2d To2(Point3d q) { Vector3d d = q - O; return new Point2d(d * E1, d * E2); }
    public Point3d To3(Point2d q) { return O + E1 * q.X + E2 * q.Y; }
  }

  // element data
  private List<int[]> elN;          // 3 global nodes
  private List<int> elPlate;
  private List<double[]> elS;       // sx, sy, txy in plate axes, ksf

  private List<Plate> plates;

  // which local triangle of plate p holds q; barycentrics out
  private static int Locate(Plate p, Point2d q, out double l0, out double l1, out double l2)
  {
    l0 = l1 = l2 = 0;
    for (int t = 0; t < p.Tri.Count; t++)
    {
      Point2d a = p.P[p.Tri[t][0]], b = p.P[p.Tri[t][1]], c = p.P[p.Tri[t][2]];
      double den = (b.Y - c.Y) * (a.X - c.X) + (c.X - b.X) * (a.Y - c.Y);
      if (Math.Abs(den) < 1e-16) continue;
      double u = ((b.Y - c.Y) * (q.X - c.X) + (c.X - b.X) * (q.Y - c.Y)) / den;
      double v = ((c.Y - a.Y) * (q.X - c.X) + (a.X - c.X) * (q.Y - c.Y)) / den;
      double w = 1 - u - v;
      double tol = -1e-9;
      if (u >= tol && v >= tol && w >= tol) { l0 = u; l1 = v; l2 = w; return t; }
    }
    return -1;
  }

  // the principal direction of one family at q in plate p (family 0 tension, 1 compression)
  private static bool PField(Plate p, Point2d q, int family, out Vector2d dir, out double val)
  {
    dir = Vector2d.Zero; val = 0;
    double l0, l1, l2;
    int t = Locate(p, q, out l0, out l1, out l2);
    if (t < 0) return false;
    int a = p.Tri[t][0], b = p.Tri[t][1], c = p.Tri[t][2];
    double sx = l0 * p.Sx[a] + l1 * p.Sx[b] + l2 * p.Sx[c];
    double sy = l0 * p.Sy[a] + l1 * p.Sy[b] + l2 * p.Sy[c];
    double txy = l0 * p.Txy[a] + l1 * p.Txy[b] + l2 * p.Txy[c];
    double s1, s2, ang;
    Principal(sx, sy, txy, out s1, out s2, out ang);
    if (family == 0) { val = s1; dir = new Vector2d(Math.Cos(ang), Math.Sin(ang)); }
    else { val = s2; dir = new Vector2d(-Math.Sin(ang), Math.Cos(ang)); }
    return true;
  }

  // one point of a line: where it is, which plate, how strong
  private struct LinePt { public Point3d X; public int Plate; public double Val; }

  // spatial hash of line points, for spacing the lines
  private Dictionary<long, List<int>> grid;
  private List<Point3d> gridPts;
  private List<int> gridLine;
  private double cell;
  private long Key(Point3d q, int dx, int dy, int dz)
  {
    long i = (long)Math.Floor(q.X / cell) + dx, j = (long)Math.Floor(q.Y / cell) + dy, k = (long)Math.Floor(q.Z / cell) + dz;
    return (i * 73856093L) ^ (j * 19349663L) ^ (k * 83492791L);
  }
  private bool TooClose(Point3d q, double d, int ownLine)
  {
    int reach = Math.Max(1, (int)Math.Ceiling(d / cell));
    double d2 = d * d;
    for (int dx = -reach; dx <= reach; dx++)
      for (int dy = -reach; dy <= reach; dy++)
        for (int dz = -reach; dz <= reach; dz++)
        {
          List<int> lst;
          if (!grid.TryGetValue(Key(q, dx, dy, dz), out lst)) continue;
          foreach (int i in lst)
            if (gridLine[i] != ownLine && gridPts[i].DistanceToSquared(q) < d2) return true;
        }
    return false;
  }
  private void AddToGrid(Point3d q, int line)
  {
    long k = Key(q, 0, 0, 0);
    List<int> lst;
    if (!grid.TryGetValue(k, out lst)) { lst = new List<int>(); grid[k] = lst; }
    lst.Add(gridPts.Count);
    gridPts.Add(q); gridLine.Add(line);
  }

  // follow one family from (plate, point) in direction sign; stops on weak or wrong-signed
  // stress, at a free edge, or near another line. Crosses folds into the next plate.
  private List<LinePt> Trace(int p0, Point2d start, Vector2d dir0, int family, double sMax, double baseSep, double h, int lineId, bool free = false, bool noSpacing = false)
  {
    var res = new List<LinePt>();
    int pi = p0; Point2d q = start; Vector2d prev = dir0;
    int guard = 0, crossings = 0;
    double cut = 0.03 * sMax;
    while (guard++ < 4000)
    {
      Plate p = plates[pi];
      Vector2d d1; double v1;
      if (!PField(p, q, family, out d1, out v1)) break;
      if (!free && ((family == 0 && v1 <= cut) || (family == 1 && v1 >= -cut))) break;
      if (d1 * prev < 0) d1 = -d1;
      Point3d X = p.To3(q);
      double sep = baseSep * Math.Min(3.0, Math.Sqrt(sMax / Math.Abs(v1)));
      if (!free && !noSpacing && res.Count > 2 && TooClose(X, 0.5 * sep, lineId)) break;
      // closing on itself
      if (res.Count > 30 && X.DistanceTo(res[0].X) < 0.5 * h) break;
      res.Add(new LinePt { X = X, Plate = pi, Val = v1 });
      // midpoint step
      Point2d qm = q + d1 * (0.5 * h);
      Vector2d d2; double v2;
      if (PField(p, qm, family, out d2, out v2)) { if (d2 * d1 < 0) d2 = -d2; } else d2 = d1;
      Point2d qn = q + d2 * h;
      double l0, l1, l2;
      if (Locate(p, qn, out l0, out l1, out l2) >= 0) { q = qn; prev = d2; continue; }
      // left the plate: find the side it crossed
      double bestT = double.MaxValue; int side = -1;
      for (int j = 0; j < p.M; j++)
      {
        Point2d a = p.C[j], b = p.C[(j + 1) % p.M];
        Vector2d r = qn - q, s = b - a;
        double den = r.X * s.Y - r.Y * s.X;
        if (Math.Abs(den) < 1e-14) continue;
        double t = ((a.X - q.X) * s.Y - (a.Y - q.Y) * s.X) / den;
        double u = ((a.X - q.X) * r.Y - (a.Y - q.Y) * r.X) / den;
        if (t >= -1e-9 && t <= 1 + 1e-9 && u >= -1e-9 && u <= 1 + 1e-9 && t < bestT) { bestT = t; side = j; }
      }
      if (side < 0) break;
      Point2d hit = q + (qn - q) * Math.Max(0, Math.Min(1, bestT));
      Point3d H = p.To3(hit);
      res.Add(new LinePt { X = H, Plate = pi, Val = v1 });
      int nb = p.SideNb[side];
      if (nb < 0 || ++crossings > 200) break;
      // carry the direction over the fold: same angle to the fold line
      Plate pq = plates[nb];
      Point3d A3 = p.To3(p.C[side]), B3 = p.To3(p.C[(side + 1) % p.M]);
      Vector3d tAx = B3 - A3; tAx.Unitize();
      Vector3d D = p.E1 * d2.X + p.E2 * d2.Y;
      double along = D * tAx, across = Math.Sqrt(Math.Max(0, 1 - along * along));
      Point3d cq = Point3d.Origin; for (int j = 0; j < pq.M; j++) cq += pq.To3(pq.C[j]); cq /= pq.M;
      Vector3d inQ = Vector3d.CrossProduct(pq.N, tAx); if (inQ * (cq - H) < 0) inQ = -inQ;
      Vector3d Dq = tAx * along + inQ * across;
      Vector2d dq = new Vector2d(Dq * pq.E1, Dq * pq.E2); dq.Unitize();
      pi = nb; q = pq.To2(H) + dq * (1e-4 * h); prev = dq;
    }
    return res;
  }

  // ---------------- shell element: membrane (CST) + plate bending (DKT, Batoz 1980) ----------------
  // Plate-bending strains (curvatures) at (xi, eta); nodal values (w, rx, ry) with rx = dw/dy, ry = -dw/dx.
  private static double DktB(Point2d A, Point2d B, Point2d C, double xi, double eta, double[,] Bout)
  {
    double x23 = B.X - C.X, x31 = C.X - A.X, x12 = A.X - B.X;
    double y23 = B.Y - C.Y, y31 = C.Y - A.Y, y12 = A.Y - B.Y;
    double l4 = x23 * x23 + y23 * y23, l5 = x31 * x31 + y31 * y31, l6 = x12 * x12 + y12 * y12;
    double P4 = -6 * x23 / l4, q4 = 3 * x23 * y23 / l4, t4 = -6 * y23 / l4, r4 = 3 * y23 * y23 / l4;
    double P5 = -6 * x31 / l5, q5 = 3 * x31 * y31 / l5, t5 = -6 * y31 / l5, r5 = 3 * y31 * y31 / l5;
    double P6 = -6 * x12 / l6, q6 = 3 * x12 * y12 / l6, t6 = -6 * y12 / l6, r6 = 3 * y12 * y12 / l6;
    double a = 1 - 2 * xi, b = 1 - 2 * eta;
    double[] Hxx = {
      P6 * a + (P5 - P6) * eta, q6 * a - (q5 + q6) * eta, -4 + 6 * (xi + eta) + r6 * a - eta * (r5 + r6),
      -P6 * a + eta * (P4 + P6), q6 * a - eta * (q6 - q4), -2 + 6 * xi + r6 * a + eta * (r4 - r6),
      -eta * (P5 + P4), eta * (q4 - q5), -eta * (r5 - r4) };
    double[] Hyx = {
      t6 * a + eta * (t5 - t6), 1 + r6 * a - eta * (r5 + r6), -q6 * a + eta * (q5 + q6),
      -t6 * a + eta * (t4 + t6), -1 + r6 * a + eta * (r4 - r6), -q6 * a - eta * (q4 - q6),
      -eta * (t4 + t5), eta * (r4 - r5), -eta * (q4 - q5) };
    double[] Hxe = {
      -P5 * b - xi * (P6 - P5), q5 * b - xi * (q5 + q6), -4 + 6 * (xi + eta) + r5 * b - xi * (r5 + r6),
      xi * (P4 + P6), xi * (q4 - q6), -xi * (r6 - r4),
      P5 * b - xi * (P4 + P5), q5 * b + xi * (q4 - q5), -2 + 6 * eta + r5 * b + xi * (r4 - r5) };
    double[] Hye = {
      -t5 * b - xi * (t6 - t5), 1 + r5 * b - xi * (r5 + r6), -q5 * b + xi * (q5 + q6),
      xi * (t4 + t6), xi * (r4 - r6), -xi * (q4 - q6),
      t5 * b - xi * (t4 + t5), -1 + r5 * b + xi * (r4 - r5), -q5 * b - xi * (q4 - q5) };
    double A2 = x31 * y12 - x12 * y31;
    for (int i = 0; i < 9; i++)
    {
      Bout[0, i] = (y31 * Hxx[i] + y12 * Hxe[i]) / A2;
      Bout[1, i] = (-x31 * Hyx[i] - x12 * Hye[i]) / A2;
      Bout[2, i] = (-x31 * Hxx[i] - x12 * Hxe[i] + y31 * Hyx[i] + y12 * Hye[i]) / A2;
    }
    return A2;
  }

  // 18 x 18 stiffness in global axes; nodes carry (ux, uy, uz, rx, ry, rz)
  private static double[,] ShellKe(Point2d A, Point2d B, Point2d C, Vector3d e1, Vector3d e2, Vector3d n, double E, double nu, double t)
  {
    double A2 = (B.X - A.X) * (C.Y - A.Y) - (C.X - A.X) * (B.Y - A.Y);
    double area = 0.5 * Math.Abs(A2);
    if (area < 1e-12) return null;
    var Kl = new double[18, 18];
    // membrane
    double[,] Bm = {
      { B.Y - C.Y, 0, C.Y - A.Y, 0, A.Y - B.Y, 0 },
      { 0, C.X - B.X, 0, A.X - C.X, 0, B.X - A.X },
      { C.X - B.X, B.Y - C.Y, A.X - C.X, C.Y - A.Y, B.X - A.X, A.Y - B.Y } };
    for (int i = 0; i < 3; i++) for (int j = 0; j < 6; j++) Bm[i, j] /= A2;
    double Dk = E / (1 - nu * nu);
    double[,] D = { { Dk, Dk * nu, 0 }, { Dk * nu, Dk, 0 }, { 0, 0, Dk * (1 - nu) / 2 } };
    for (int i = 0; i < 6; i++)
      for (int j = 0; j < 6; j++)
      {
        double s = 0;
        for (int k = 0; k < 3; k++) for (int l = 0; l < 3; l++) s += Bm[k, i] * D[k, l] * Bm[l, j];
        Kl[6 * (i / 2) + i % 2, 6 * (j / 2) + j % 2] += s * t * area;
      }
    // bending, three-point rule at the side midpoints
    double tb = t * t * t / 12.0;
    var Kb = new double[9, 9];
    var Bb = new double[3, 9];
    double[] gx = { 0.5, 0.5, 0.0 }, gy = { 0.0, 0.5, 0.5 };
    for (int gp = 0; gp < 3; gp++)
    {
      DktB(A, B, C, gx[gp], gy[gp], Bb);
      for (int i = 0; i < 9; i++)
        for (int j = 0; j < 9; j++)
        {
          double s = 0;
          for (int k = 0; k < 3; k++) for (int l = 0; l < 3; l++) s += Bb[k, i] * D[k, l] * tb * Bb[l, j];
          Kb[i, j] += s * area / 3.0;
        }
    }
    for (int i = 0; i < 9; i++)
      for (int j = 0; j < 9; j++)
        Kl[6 * (i / 3) + 2 + i % 3, 6 * (j / 3) + 2 + j % 3] += Kb[i, j];
    // a small stiffness for turning about the plate normal (the plate has none of its own)
    double kd = 0;
    for (int i = 0; i < 3; i++) kd += Kb[3 * i + 1, 3 * i + 1] + Kb[3 * i + 2, 3 * i + 2];
    kd = 1e-3 * kd / 6.0;
    for (int i = 0; i < 3; i++) Kl[6 * i + 5, 6 * i + 5] += kd;
    // to global axes, 3 x 3 block by block
    double[,] R = { { e1.X, e1.Y, e1.Z }, { e2.X, e2.Y, e2.Z }, { n.X, n.Y, n.Z } };
    var Kg = new double[18, 18];
    for (int I = 0; I < 6; I++)
      for (int J = 0; J < 6; J++)
      {
        var tmp = new double[3, 3];
        for (int a = 0; a < 3; a++)
          for (int be = 0; be < 3; be++)
          {
            double s = 0;
            for (int b = 0; b < 3; b++) s += Kl[3 * I + a, 3 * J + b] * R[b, be];
            tmp[a, be] = s;
          }
        for (int al = 0; al < 3; al++)
          for (int be = 0; be < 3; be++)
          {
            double s = 0;
            for (int a = 0; a < 3; a++) s += R[a, al] * tmp[a, be];
            Kg[3 * I + al, 3 * J + be] = s;
          }
      }
    return Kg;
  }

  // reverse Cuthill-McKee: numbers the points so the matrix stays narrow
  private static int[] Rcm(int n, List<int>[] adj)
  {
    var deg = new int[n];
    for (int i = 0; i < n; i++) deg[i] = adj[i].Count;
    var byDeg = new int[n]; var keys = new int[n];
    for (int i = 0; i < n; i++) { byDeg[i] = i; keys[i] = deg[i]; }
    Array.Sort(keys, byDeg);
    var seen = new bool[n];
    var mark = new int[n]; int stamp = 0;
    var order = new List<int>(n);
    Func<int, int> Far = s0 =>
    {
      stamp++;
      var frontier = new List<int> { s0 }; mark[s0] = stamp; int last = s0;
      while (frontier.Count > 0)
      {
        var next = new List<int>();
        foreach (int u in frontier) foreach (int v in adj[u]) if (mark[v] != stamp) { mark[v] = stamp; next.Add(v); }
        if (next.Count > 0) { last = next[0]; foreach (int v in next) if (deg[v] < deg[last]) last = v; }
        frontier = next;
      }
      return last;
    };
    foreach (int s0 in byDeg)
    {
      if (seen[s0]) continue;
      int start = Far(Far(s0));
      var queue = new Queue<int>();
      queue.Enqueue(start); seen[start] = true;
      while (queue.Count > 0)
      {
        int u = queue.Dequeue();
        order.Add(u);
        var nb = new List<int>();
        foreach (int v in adj[u]) if (!seen[v]) nb.Add(v);
        nb.Sort((x, y) => deg[x].CompareTo(deg[y]));
        foreach (int v in nb) { seen[v] = true; queue.Enqueue(v); }
      }
    }
    order.Reverse();
    return order.ToArray();
  }

  // corner cutting (Chaikin): softens a traced line into a drawn curve, ends kept
  private static Polyline Smooth(Polyline pl, int passes)
  {
    for (int k = 0; k < passes && pl.Count > 2; k++)
    {
      var o = new Polyline();
      o.Add(pl[0]);
      for (int i = 0; i + 1 < pl.Count; i++)
      {
        Point3d a = pl[i], b = pl[i + 1];
        if (i > 0) o.Add(a + (b - a) * 0.25);
        if (i + 1 < pl.Count - 1) o.Add(a + (b - a) * 0.75);
      }
      o.Add(pl[pl.Count - 1]);
      pl = o;
    }
    return pl;
  }

  // a bar hook at one end of a line, like the hooked ends of reinforcing bars
  private static Polyline Hook(Point3d end, Vector3d dir, double r)
  {
    dir.Z = 0; if (!dir.Unitize()) return null;
    Vector3d side = Vector3d.CrossProduct(Vector3d.ZAxis, dir);
    Point3d c = end + side * r;
    var h = new Polyline();
    for (int i = 0; i <= 10; i++)
    {
      double a = Math.PI * i / 10.0;
      h.Add(c - side * (r * Math.Cos(a)) + dir * (r * Math.Sin(a)));
    }
    h.Add(end + side * (2 * r) - dir * r);
    return h;
  }

  private static bool shownForModule(Vector3d n) { return true; }

  private static Point2d Apply(double[] t, Point2d q)
  {
    return new Point2d(t[0] * q.X - t[1] * q.Y + t[2], t[1] * q.X + t[0] * q.Y + t[3]);
  }

  // +1 ridge, -1 valley, 0 nearly flat (under 5 deg), seen from above
  private static int FoldKind(Mesh m, Vector3d[] fn, int f1, int f2, Line ln)
  {
    if (Vector3d.VectorAngle(fn[f1], fn[f2]) < 5.0 * Math.PI / 180.0) return 0;
    MeshFace o = m.Faces[f2];
    int[] idx = o.IsQuad ? new int[] { o.A, o.B, o.C, o.D } : new int[] { o.A, o.B, o.C };
    double best = 0; Point3d far = ln.From;
    foreach (int v in idx) { Point3d q = m.Vertices.Point3dAt(v); double d = ln.DistanceTo(q, false); if (d > best) { best = d; far = q; } }
    return fn[f1] * (far - ln.From) < 0 ? 1 : -1;
  }

  private static void Dashes(List<Line> to, Point3d a, Point3d b, double dash)
  {
    double len = a.DistanceTo(b);
    if (len < 1e-9) return;
    int nd = Math.Max(1, (int)Math.Round(len / (2 * dash)));
    double step = len / nd;
    Vector3d d = (b - a) / len;
    for (int i = 0; i < nd; i++) to.Add(new Line(a + d * (i * step + 0.25 * step), a + d * (i * step + 0.75 * step)));
  }

  private void RunScript(
    object Plates, List<object> Supports, object LiveLoad, object Fc, object Refine, object Spacing,
    object KipsPerLine, object SupportMode, object Bake,
    ref object Tension, ref object Compression, ref object Folds, ref object FoldForce,
    ref object Plan, ref object Field, ref object Report)
  {
    drawData = null; drawBox = BoundingBox.Empty;
    try { AppDomain.CurrentDomain.SetData("FoldToolkit.StressDraw", null); } catch { }
    bool hasWindow = WindowPresent();
    if (hasWindow && !ToolOn("StressOn", true)) { Report = "STRESS is switched off in the control window: tick \"Analyse tension and compression\" at the top of section 9. To run STRESS without the window, delete the window component from the canvas (closing its panel is not enough)."; return; }
    win = null;
    if (hasWindow) try { win = AppDomain.CurrentDomain.GetData("FoldToolkit.Stress") as Dictionary<string, double>; } catch { }

    Mesh mesh = AsMesh(Plates);
    if (mesh == null || mesh.Faces.Count == 0) { Report = "Connect the Plates output of the Fold Designer or the FOLDLAB reader to Plates."; return; }
    double LL = Num(LiveLoad, W("StrLL", 20.0)); if (LL < 0) LL = 0;
    double fc = Num(Fc, W("StrFc", 4000.0)); if (fc < 1000) fc = 4000.0;
    int n = (int)Math.Round(Num(Refine, W("StrRefine", 4.0))); n = Math.Max(2, Math.Min(6, n));
    double spacing = Num(Spacing, W("StrSpacing", 0.0));
    double perLine = Math.Max(0.0, Num(KipsPerLine, W("StrPerLine", 4.8)));     // kips each line carries (0 = lines spaced by stress)
    double eqForceT = 0, eqForceC = 0; int eqCountT = 0, eqCountC = 0;
    double slab = 0.333;
    try { object so = AppDomain.CurrentDomain.GetData("FoldToolkit.SlabFt"); if (so is double) slab = (double)so; } catch { }
    // a roof from the FOLDLAB reader carries its own slab thickness, which wins over the shared value
    { double us; string ss = mesh.GetUserString("FoldToolkit.SlabFt");
      if (ss != null && double.TryParse(ss, System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out us) && us > 0) slab = us; }

    double E = 57000.0 * Math.Sqrt(fc) * 0.144;   // ksf
    double nu = 0.2;
    double kToPsi = 1000.0 / 144.0;               // ksf -> psi

    var sw = System.Diagnostics.Stopwatch.StartNew();
    int nF = mesh.Faces.Count;
    BoundingBox bb = mesh.GetBoundingBox(true);
    double diag = bb.Diagonal.Length;
    double ground = bb.Min.Z, gTol = Math.Max(0.01, 1e-4 * diag);

    // normals, all on the upper side
    var fn = new Vector3d[nF];
    double zSum = 0;
    for (int f = 0; f < nF; f++)
    {
      MeshFace mf = mesh.Faces[f];
      Point3d a = mesh.Vertices.Point3dAt(mf.A), b = mesh.Vertices.Point3dAt(mf.B), c = mesh.Vertices.Point3dAt(mf.C);
      Vector3d nn = mf.IsQuad ? Vector3d.CrossProduct(c - a, mesh.Vertices.Point3dAt(mf.D) - b) : Vector3d.CrossProduct(b - a, c - a);
      zSum += nn.Z; nn.Unitize(); fn[f] = nn;
    }
    if (zSum < 0) for (int f = 0; f < nF; f++) fn[f] = -fn[f];

    // ---------------- refine: nodes shared by position ----------------
    var nodes = new List<Point3d>();
    var nodeHash = new Dictionary<long, List<int>>();
    double q0 = Math.Max(1e-6, 1e-7 * diag) * 10.0;
    Func<Point3d, int> NodeAt = (Point3d x) =>
    {
      long ix = (long)Math.Floor(x.X / q0), iy = (long)Math.Floor(x.Y / q0), iz = (long)Math.Floor(x.Z / q0);
      for (int dx = -1; dx <= 1; dx++) for (int dy = -1; dy <= 1; dy++) for (int dz = -1; dz <= 1; dz++)
      {
        long key = ((ix + dx) * 73856093L) ^ ((iy + dy) * 19349663L) ^ ((iz + dz) * 83492791L);
        List<int> lst;
        if (nodeHash.TryGetValue(key, out lst)) foreach (int i in lst) if (nodes[i].DistanceTo(x) < q0) return i;
      }
      long k0 = (ix * 73856093L) ^ (iy * 19349663L) ^ (iz * 83492791L);
      List<int> l0;
      if (!nodeHash.TryGetValue(k0, out l0)) { l0 = new List<int>(); nodeHash[k0] = l0; }
      l0.Add(nodes.Count); nodes.Add(x);
      return nodes.Count - 1;
    };

    plates = new List<Plate>();
    elN = new List<int[]>(); elPlate = new List<int>(); elS = new List<double[]>();
    var tvs = mesh.TopologyVertices; var tes = mesh.TopologyEdges;
    for (int f = 0; f < nF; f++)
    {
      MeshFace mf = mesh.Faces[f];
      int[] vi = mf.IsQuad ? new int[] { mf.A, mf.B, mf.C, mf.D } : new int[] { mf.A, mf.B, mf.C };
      int m = vi.Length;
      var c3 = new Point3d[m];
      for (int j = 0; j < m; j++) c3[j] = mesh.Vertices.Point3dAt(vi[j]);
      var p = new Plate { Face = f, M = m, N = fn[f], O = c3[0] };
      p.E1 = c3[1] - c3[0]; p.E1.Unitize();
      p.E2 = Vector3d.CrossProduct(p.N, p.E1); p.E2.Unitize();
      p.C = new Point2d[m];
      for (int j = 0; j < m; j++) p.C[j] = p.To2(c3[j]);
      // local grid
      var idx = new Dictionary<int, int>();      // (i*(n+1)+j) -> local node
      var gl = new List<int>(); var lp = new List<Point2d>();
      Func<int, int, int> Local = (i, j) =>
      {
        int key = i * (n + 1) + j, li;
        if (idx.TryGetValue(key, out li)) return li;
        double u = (double)i / n, v = (double)j / n;
        Point3d x;
        if (m == 4) x = c3[0] * ((1 - u) * (1 - v)) + c3[1] * (u * (1 - v)) + c3[2] * (u * v) + c3[3] * ((1 - u) * v);
        else x = c3[0] + (c3[1] - c3[0]) * u + (c3[2] - c3[0]) * v;
        li = gl.Count; idx[key] = li;
        gl.Add(NodeAt(x)); lp.Add(p.To2(x));
        return li;
      };
      if (m == 4)
      {
        for (int i = 0; i < n; i++)
          for (int j = 0; j < n; j++)
          {
            int a = Local(i, j), b = Local(i + 1, j), c = Local(i + 1, j + 1), d = Local(i, j + 1);
            p.Tri.Add(new int[] { a, b, c }); p.Tri.Add(new int[] { a, c, d });
          }
      }
      else
      {
        for (int i = 0; i < n; i++)
          for (int j = 0; i + j < n; j++)
          {
            int a = Local(i, j), b = Local(i + 1, j), d = Local(i, j + 1);
            p.Tri.Add(new int[] { a, b, d });
            if (i + j < n - 1) { int c = Local(i + 1, j + 1); p.Tri.Add(new int[] { b, c, d }); }
          }
      }
      // sides, in corner order
      p.SideNodes = new int[m][]; p.SideNb = new int[m]; p.SideLoad = new bool[m];
      for (int s = 0; s < m; s++)
      {
        var sn = new int[n + 1];
        for (int k = 0; k <= n; k++)
        {
          int li;
          if (m == 4)
          {
            if (s == 0) li = Local(k, 0); else if (s == 1) li = Local(n, k); else if (s == 2) li = Local(n - k, n); else li = Local(0, n - k);
          }
          else
          {
            if (s == 0) li = Local(k, 0); else if (s == 1) li = Local(n - k, k); else li = Local(0, n - k);
          }
          sn[k] = gl[li];
        }
        p.SideNodes[s] = sn;
        int ta = tvs.TopologyVertexIndex(vi[s]), tb = tvs.TopologyVertexIndex(vi[(s + 1) % m]);
        int e = tes.GetEdgeIndex(ta, tb);
        p.SideNb[s] = -1;
        if (e >= 0) { int[] fs = tes.GetConnectedFaces(e); if (fs != null && fs.Length == 2) p.SideNb[s] = fs[0] == f ? fs[1] : fs[0]; }
      }
      p.Nodes = gl.ToArray(); p.P = lp.ToArray();
      foreach (int[] t in p.Tri)
      {
        p.TriElem.Add(elN.Count);
        elN.Add(new int[] { p.Nodes[t[0]], p.Nodes[t[1]], p.Nodes[t[2]] });
        elPlate.Add(f);
        elS.Add(new double[3]);
      }
      plates.Add(p);
    }
    int N = nodes.Count;

    // ---------------- supports ----------------
    var fixedN = new bool[N];
    int nSup = 0, supPts = 0;
    var supCorners = new List<Point3d>();
    if (Supports != null)
      foreach (object o in Supports)
      {
        bool ok; Point3d sp = AsPoint(o, out ok); if (!ok) continue;
        supPts++;
        int best = -1; double bd = double.MaxValue;
        for (int i = 0; i < N; i++) { double d = nodes[i].DistanceToSquared(sp); if (d < bd) { bd = d; best = i; } }
        if (best >= 0 && !fixedN[best]) { fixedN[best] = true; nSup++; supCorners.Add(nodes[best]); }
      }
    bool designerSupports = false;
    if (supPts == 0)
    {
      // a Designer pattern on columns can pass its support points here
      Point3d[] given = null;
      try { given = AppDomain.CurrentDomain.GetData("FoldToolkit.SupportPts") as Point3d[]; } catch { }
      if (given != null && given.Length > 0)
        foreach (Point3d gp in given)
        {
          supPts++; designerSupports = true;
          int best = -1; double bd = double.MaxValue;
          for (int i = 0; i < N; i++) { double d = nodes[i].DistanceToSquared(gp); if (d < bd) { bd = d; best = i; } }
          if (best >= 0 && !fixedN[best]) { fixedN[best] = true; nSup++; supCorners.Add(nodes[best]); }
        }
    }
    if (supPts == 0)
    {
      for (int i = 0; i < N; i++) if (nodes[i].Z < ground + gTol) { fixedN[i] = true; nSup++; }
      for (int v = 0; v < mesh.Vertices.Count; v++) { Point3d mv = mesh.Vertices.Point3dAt(v); if (mv.Z < ground + gTol) supCorners.Add(mv); }
    }
    if (nSup == 0) { Report = "No supports: nothing touches the ground. Give column points to Supports."; return; }

    // How the supports hold the roof sideways.
    //   pinned:   held in every direction (footings that take the outward thrust of a frame or arch)
    //   bearings: held up everywhere, but sideways only enough to keep the roof in place, so the
    //             supports cannot act as a tie (a roof spanning between columns or walls)
    // Automatic: ground supports pinned, given column points on bearings.
    int supMode = (int)Math.Round(Num(SupportMode, W("StrSupMode", 0.0)));
    bool bearings = supMode == 2 || (supMode == 0 && supPts > 0);
    var fixMask = new int[N];
    for (int i = 0; i < N; i++) if (fixedN[i]) fixMask[i] = 7;
    string supNote = "pinned in every direction";
    if (bearings)
    {
      int ia = -1, ib = -1; double far = -1;
      for (int i = 0; i < N; i++) if (fixedN[i]) { if (ia < 0) ia = i; fixMask[i] = 4; }
      for (int i = 0; i < N; i++) if (fixedN[i]) { double d = Math.Pow(nodes[i].X - nodes[ia].X, 2) + Math.Pow(nodes[i].Y - nodes[ia].Y, 2); if (d > far) { far = d; ib = i; } }
      fixMask[ia] = 7;                                            // one support holds both directions
      if (ib >= 0 && ib != ia)                                    // the farthest one stops the roof turning
      {
        double ax = Math.Abs(nodes[ib].X - nodes[ia].X), ay = Math.Abs(nodes[ib].Y - nodes[ia].Y);
        fixMask[ib] = ax >= ay ? (4 | 2) : (4 | 1);
      }
      supNote = "on bearings: held up everywhere, held sideways at only two points so they cannot act as a tie";
    }

    // ---------------- loads: self-weight on the true area, live load on plan (kips) ----------------
    var Fn = new double[3 * N];
    double totalLoad = 0, swTotal = 0;
    for (int el = 0; el < elN.Count; el++)
    {
      Plate p = plates[elPlate[el]];
      int[] g = elN[el];
      double area = 0.5 * Vector3d.CrossProduct(nodes[g[1]] - nodes[g[0]], nodes[g[2]] - nodes[g[0]]).Length;
      double w = (0.150 * slab) * area + (LL / 1000.0) * area * Math.Abs(p.N.Z);
      swTotal += 0.150 * slab * area;
      totalLoad += w;
      for (int i = 0; i < 3; i++) Fn[3 * g[i] + 2] -= w / 3.0;
    }

    // ---------------- numbering: narrow band (reverse Cuthill-McKee), 6 per point ----------------
    var adj = new List<int>[N];
    for (int i = 0; i < N; i++) adj[i] = new List<int>();
    foreach (int[] g in elN)
      for (int a = 0; a < 3; a++)
        for (int b = 0; b < 3; b++)
          if (a != b && !adj[g[a]].Contains(g[b])) adj[g[a]].Add(g[b]);
    int[] order = Rcm(N, adj);
    var dofOf = new int[6 * N];
    for (int i = 0; i < dofOf.Length; i++) dofOf[i] = -1;
    int ndof = 0;
    foreach (int nd in order)
      for (int k = 0; k < 6; k++)
      {
        if (fixedN[nd] && k < 3 && ((fixMask[nd] >> k) & 1) == 1) continue;   // held in this direction; always free to rotate
        dofOf[6 * nd + k] = ndof++;
      }
    var first = new int[ndof];
    for (int i = 0; i < ndof; i++) first[i] = i;
    foreach (int[] g in elN)
    {
      int lo = int.MaxValue;
      for (int a = 0; a < 3; a++) for (int k = 0; k < 6; k++) { int dd = dofOf[6 * g[a] + k]; if (dd >= 0 && dd < lo) lo = dd; }
      for (int a = 0; a < 3; a++) for (int k = 0; k < 6; k++) { int dd = dofOf[6 * g[a] + k]; if (dd >= 0 && lo < first[dd]) first[dd] = lo; }
    }
    long profile = 0; double work = 0;
    for (int j = 0; j < ndof; j++) { long h0 = j - first[j] + 1; profile += h0; work += (double)h0 * h0; }
    if (profile > 80000000L || work > 6e10)
    {
      Report = string.Format("STRESS: the model is too large to solve here ({0} unknowns). Lower Refine in section 9 (try {1}).", ndof, Math.Max(2, n - 1));
      return;
    }
    var sky = new double[ndof][];
    for (int j = 0; j < ndof; j++) sky[j] = new double[j - first[j] + 1];

    // ---------------- assemble ----------------
    double Dk = E / (1 - nu * nu);
    for (int el = 0; el < elN.Count; el++)
    {
      Plate p = plates[elPlate[el]];
      int[] g = elN[el];
      double[,] Ke = ShellKe(p.To2(nodes[g[0]]), p.To2(nodes[g[1]]), p.To2(nodes[g[2]]), p.E1, p.E2, p.N, E, nu, slab);
      if (Ke == null) continue;
      for (int a = 0; a < 18; a++)
      {
        int da = dofOf[6 * g[a / 6] + a % 6];
        if (da < 0) continue;
        for (int b = 0; b < 18; b++)
        {
          int db = dofOf[6 * g[b / 6] + b % 6];
          if (db < da) continue;
          sky[db][da - first[db]] += Ke[a, b];
        }
      }
    }
    var Fv = new double[ndof];
    for (int i = 0; i < N; i++)
      for (int k = 0; k < 3; k++) { int dd = dofOf[6 * i + k]; if (dd >= 0) Fv[dd] += Fn[3 * i + k]; }

    // ---------------- solve: banded LDLt, exact (no iterations) ----------------
    var piv = new double[ndof];
    int weak = 0;
    for (int j = 0; j < ndof; j++)
    {
      int fj = first[j]; double[] cj = sky[j];
      double kjj = cj[j - fj];
      for (int i = fj + 1; i < j; i++)
      {
        int fi = first[i]; int k0 = Math.Max(fi, fj); double s = 0; double[] ci = sky[i];
        for (int k = k0; k < i; k++) s += ci[k - fi] * cj[k - fj];
        cj[i - fj] -= s;
      }
      double dj = cj[j - fj];
      for (int i = fj; i < j; i++) { double gv = cj[i - fj]; double lv = gv / piv[i]; cj[i - fj] = lv; dj -= gv * lv; }
      if (!(dj > 1e-8 * Math.Abs(kjj)))          // the structure can move here without resistance
      {
        weak++;
        dj = Math.Max(Math.Abs(kjj), 1e-12) * 1e-8;
      }
      piv[j] = dj;
    }
    var X = new double[ndof];
    Array.Copy(Fv, X, ndof);
    for (int j = 0; j < ndof; j++) { int fj = first[j]; double s = 0; double[] cj = sky[j]; for (int i = fj; i < j; i++) s += cj[i - fj] * X[i]; X[j] -= s; }
    for (int j = 0; j < ndof; j++) X[j] /= piv[j];
    for (int j = ndof - 1; j >= 0; j--) { int fj = first[j]; double[] cj = sky[j]; double xj = X[j]; for (int i = fj; i < j; i++) X[i] -= cj[i - fj] * xj; }
    sky = null;
    var U6 = new double[6 * N];
    for (int i = 0; i < 6 * N; i++) if (dofOf[i] >= 0) U6[i] = X[dofOf[i]];
    double solveSec = sw.Elapsed.TotalSeconds;

    // ---------------- reactions, stresses and moments, element by element ----------------
    var reaction = new double[3 * N];
    double sMaxT = 0, sMaxC = 0, surfT = 0, surfC = 0, mMax = 0, uMax = 0;
    for (int i = 0; i < N; i++) uMax = Math.Max(uMax, Math.Abs(U6[6 * i + 2]));
    double Db = E * slab * slab * slab / (12.0 * (1 - nu * nu));
    for (int el = 0; el < elN.Count; el++)
    {
      Plate p = plates[elPlate[el]];
      int[] g = elN[el];
      Point2d a = p.To2(nodes[g[0]]), b = p.To2(nodes[g[1]]), c = p.To2(nodes[g[2]]);
      double A2 = (b.X - a.X) * (c.Y - a.Y) - (c.X - a.X) * (b.Y - a.Y);
      if (Math.Abs(A2) < 1e-12) continue;
      // reactions at supported points
      bool anyFixed = fixedN[g[0]] || fixedN[g[1]] || fixedN[g[2]];
      if (anyFixed)
      {
        double[,] Ke = ShellKe(a, b, c, p.E1, p.E2, p.N, E, nu, slab);
        for (int r = 0; r < 18; r++)
        {
          int nd = g[r / 6], k = r % 6;
          if (!fixedN[nd] || k > 2 || ((fixMask[nd] >> k) & 1) == 0) continue;
          double s = 0;
          for (int q = 0; q < 18; q++) s += Ke[r, q] * U6[6 * g[q / 6] + q % 6];
          reaction[3 * nd + k] += s;
        }
      }
      // membrane stress (plate axes)
      var u6 = new double[6]; var db9 = new double[9];
      for (int i = 0; i < 3; i++)
      {
        Vector3d Ui = new Vector3d(U6[6 * g[i]], U6[6 * g[i] + 1], U6[6 * g[i] + 2]);
        Vector3d Ri = new Vector3d(U6[6 * g[i] + 3], U6[6 * g[i] + 4], U6[6 * g[i] + 5]);
        u6[2 * i] = Ui * p.E1; u6[2 * i + 1] = Ui * p.E2;
        db9[3 * i] = Ui * p.N; db9[3 * i + 1] = Ri * p.E1; db9[3 * i + 2] = Ri * p.E2;
      }
      double ex = ((b.Y - c.Y) * u6[0] + (c.Y - a.Y) * u6[2] + (a.Y - b.Y) * u6[4]) / A2;
      double ey = ((c.X - b.X) * u6[1] + (a.X - c.X) * u6[3] + (b.X - a.X) * u6[5]) / A2;
      double gxy = ((c.X - b.X) * u6[0] + (b.Y - c.Y) * u6[1] + (a.X - c.X) * u6[2] + (c.Y - a.Y) * u6[3] + (b.X - a.X) * u6[4] + (a.Y - b.Y) * u6[5]) / A2;
      double sx = Dk * (ex + nu * ey), sy = Dk * (ey + nu * ex), txy = Dk * (1 - nu) / 2 * gxy;
      elS[el] = new double[] { sx, sy, txy };
      double s1, s2, ang; Principal(sx, sy, txy, out s1, out s2, out ang);
      sMaxT = Math.Max(sMaxT, s1); sMaxC = Math.Min(sMaxC, s2);
      // bending moments at the centre (per ft), and the stress on both faces
      var Bb = new double[3, 9];
      DktB(a, b, c, 1.0 / 3.0, 1.0 / 3.0, Bb);
      double k1 = 0, k2 = 0, k3 = 0;
      for (int q = 0; q < 9; q++) { k1 += Bb[0, q] * db9[q]; k2 += Bb[1, q] * db9[q]; k3 += Bb[2, q] * db9[q]; }
      double mx = Db * (k1 + nu * k2), my = Db * (k2 + nu * k1), mxy = Db * (1 - nu) / 2 * k3;
      double m1, m2, mang; Principal(mx, my, mxy, out m1, out m2, out mang);
      mMax = Math.Max(mMax, Math.Max(Math.Abs(m1), Math.Abs(m2)));
      double fb = 6.0 / (slab * slab);
      for (int face = -1; face <= 1; face += 2)
      {
        double f1, f2, fang;
        Principal(sx + face * fb * mx, sy + face * fb * my, txy + face * fb * mxy, out f1, out f2, out fang);
        surfT = Math.Max(surfT, f1); surfC = Math.Min(surfC, f2);
      }
    }
    double reactZ = 0;
    for (int i = 0; i < N; i++) if (fixedN[i]) reactZ += reaction[3 * i + 2] - Fn[3 * i + 2];
    bool mechanism = weak > 0;
    if (mechanism)
    {
      Report = string.Format(
        "STRESS: this roof is a mechanism as supported: it can move without bending or stretching ({0} free movements found), " +
        "so no forces can be drawn.\n\nAdd supports (Supports input: column points), or make sure the roof reaches the ground " +
        "at both ends, then run again.", weak);
      return;
    }

    // smooth to the nodes of each plate (plate axes, so within one plate only)
    foreach (Plate p in plates)
    {
      int ln = p.P.Length;
      p.Sx = new double[ln]; p.Sy = new double[ln]; p.Txy = new double[ln];
      var wsum = new double[ln];
      for (int t = 0; t < p.Tri.Count; t++)
      {
        double[] s = elS[p.TriElem[t]];
        foreach (int li in p.Tri[t]) { p.Sx[li] += s[0]; p.Sy[li] += s[1]; p.Txy[li] += s[2]; wsum[li] += 1; }
      }
      for (int i = 0; i < ln; i++) if (wsum[i] > 0) { p.Sx[i] /= wsum[i]; p.Sy[i] /= wsum[i]; p.Txy[i] /= wsum[i]; }
    }

    // ---------------- lines ----------------
    double baseSep = spacing > 0 ? spacing : Math.Max(0.25, diag / 90.0);
    double h = baseSep / 5.0;
    double lift = slab / 2.0 + 0.03;
    var bandT = new List<Polyline>[] { new List<Polyline>(), new List<Polyline>(), new List<Polyline>() };
    var bandC = new List<Polyline>[] { new List<Polyline>(), new List<Polyline>(), new List<Polyline>() };
    var planT = new List<Polyline>[] { new List<Polyline>(), new List<Polyline>(), new List<Polyline>() };
    var planC = new List<Polyline>[] { new List<Polyline>(), new List<Polyline>(), new List<Polyline>() };
    var tensionOut = new List<Curve>(); var compOut = new List<Curve>();
    double planDx = bb.Max.X - bb.Min.X + Math.Max(10.0, 0.2 * (bb.Max.X - bb.Min.X));
    Vector3d planShift = new Vector3d(planDx, 0, 0);
    // plan style: 0 projected plan, 1 diagram (folds, outline, supports), 2 roof plates only
    int planStyle = (int)Math.Round(W("StrPlan", 1.0));
    if (planStyle < 0 || planStyle > 2) planStyle = 1;
    double maxSlope = W("StrMaxSlope", 50.0);
    var shown = new bool[nF];
    int hiddenPlates = 0;
    for (int f = 0; f < nF; f++)
    {
      double slope = Vector3d.VectorAngle(fn[f], Vector3d.ZAxis) * 180.0 / Math.PI;
      shown[f] = planStyle != 2 || slope <= maxSlope;
      if (!shown[f]) hiddenPlates++;
    }
    Func<Point3d, int, Point3d> PlanMap = (X3, pfi) => new Point3d(X3.X, X3.Y, ground) + planShift;

    // ---------------- one module, unrolled at true shape beside the plan ----------------
    // A module is a few neighbouring strips (plates between fold lines) along the whole span.
    // It is unfolded from its crown: first across its strips, then along each strip.
    bool modOn = W("StrModOn", 1.0) > 0.5;
    int[] ftags = null;
    try { ftags = AppDomain.CurrentDomain.GetData("FoldToolkit.FaceTags") as int[]; } catch { }
    var inMod = new bool[nF];
    var tfm = new double[nF][];
    string modNote = null;
    int modI0 = 0, modW = 0, modGaps = 0;
    if (modOn && (ftags == null || ftags.Length != nF)) { modOn = false; modNote = "  - The unrolled module needs plates straight from the Fold Designer."; }
    if (modOn)
    {
      var stripOf = new int[nF]; var stationOf = new int[nF];
      int maxStrip = 0;
      for (int f = 0; f < nF; f++) { stripOf[f] = ftags[f] % 10000; stationOf[f] = ftags[f] / 10000; maxStrip = Math.Max(maxStrip, stripOf[f]); }
      modW = Math.Max(1, Math.Min(maxStrip + 1, (int)Math.Round(W("StrModWidth", 2.0))));
      int want = (int)Math.Round(W("StrModFold", 0.0));            // 0 = the middle of the roof
      modI0 = want > 0 ? want - 1 : (maxStrip + 1 - modW) / 2;
      modI0 = Math.Max(0, Math.Min(maxStrip + 1 - modW, modI0));
      var modFaces = new List<int>();
      for (int f = 0; f < nF; f++) if (stripOf[f] >= modI0 && stripOf[f] < modI0 + modW && shownForModule(fn[f])) { inMod[f] = true; modFaces.Add(f); }
      if (modFaces.Count == 0) { modOn = false; modNote = "  - The module has no plates to unroll."; }
      else
      {
        Func<int, Point3d> Cen3 = (int f) => { Plate pp = plates[f]; Point3d c = Point3d.Origin; for (int j = 0; j < pp.M; j++) c += pp.To3(pp.C[j]); return c / pp.M; };
        int crownF = modFaces[0];
        foreach (int f in modFaces) if (Cen3(f).Z > Cen3(crownF).Z) crownF = f;
        int crownSt = stationOf[crownF];
        Action<int, int> Grow = (int pf, int sd) =>
        {
          Plate pp = plates[pf]; int qf = pp.SideNb[sd];
          Point3d A3 = pp.To3(pp.C[sd]), B3 = pp.To3(pp.C[(sd + 1) % pp.M]);
          Point2d A2 = Apply(tfm[pf], pp.C[sd]), B2 = Apply(tfm[pf], pp.C[(sd + 1) % pp.M]);
          Plate pq = plates[qf];
          Point2d qa = pq.To2(A3), qb = pq.To2(B3);
          double ang = Math.Atan2(B2.Y - A2.Y, B2.X - A2.X) - Math.Atan2(qb.Y - qa.Y, qb.X - qa.X);
          double cs = Math.Cos(ang), sn = Math.Sin(ang);
          tfm[qf] = new double[] { cs, sn, A2.X - (cs * qa.X - sn * qa.Y), A2.Y - (sn * qa.X + cs * qa.Y) };
        };
        tfm[crownF] = new double[] { 1, 0, 0, 0 };
        var queue = new Queue<int>(); queue.Enqueue(crownF);
        // across the module's strips at the crown
        while (queue.Count > 0)
        {
          int pf = queue.Dequeue(); Plate pp = plates[pf];
          for (int sd = 0; sd < pp.M; sd++)
          {
            int qf = pp.SideNb[sd];
            if (qf < 0 || !inMod[qf] || tfm[qf] != null || stationOf[qf] != crownSt) continue;
            Grow(pf, sd); queue.Enqueue(qf);
          }
        }
        // then along each strip, out from the crown
        foreach (int f in modFaces) if (tfm[f] != null) queue.Enqueue(f);
        while (queue.Count > 0)
        {
          int pf = queue.Dequeue(); Plate pp = plates[pf];
          for (int sd = 0; sd < pp.M; sd++)
          {
            int qf = pp.SideNb[sd];
            if (qf < 0 || !inMod[qf] || tfm[qf] != null || stripOf[qf] != stripOf[pf]) continue;
            Grow(pf, sd); queue.Enqueue(qf);
          }
        }
        bool grew = true;
        while (grew)
        {
          grew = false;
          foreach (int f in modFaces)
          {
            if (tfm[f] == null) continue;
            Plate pp = plates[f];
            for (int sd = 0; sd < pp.M; sd++) { int qf = pp.SideNb[sd]; if (qf >= 0 && inMod[qf] && tfm[qf] == null) { Grow(f, sd); grew = true; } }
          }
        }
        foreach (int f in modFaces) if (tfm[f] == null) { inMod[f] = false; }
        // stand it up: first station at the bottom, last at the top, beside the plan
        int stLo = int.MaxValue, stHi = int.MinValue;
        foreach (int f in modFaces) if (inMod[f]) { stLo = Math.Min(stLo, stationOf[f]); stHi = Math.Max(stHi, stationOf[f]); }
        double ax = 0, ay = 0, bx = 0, by = 0; int na = 0, nb = 0;
        foreach (int f in modFaces)
        {
          if (!inMod[f]) continue;
          Plate pp = plates[f]; double cx = 0, cy = 0;
          for (int j = 0; j < pp.M; j++) { Point2d q = Apply(tfm[f], pp.C[j]); cx += q.X / pp.M; cy += q.Y / pp.M; }
          if (stationOf[f] == stLo) { ax += cx; ay += cy; na++; }
          if (stationOf[f] == stHi) { bx += cx; by += cy; nb++; }
        }
        double phi = (na > 0 && nb > 0 && stHi > stLo) ? Math.PI / 2 - Math.Atan2(by / nb - ay / na, bx / nb - ax / na) : 0;
        double cp = Math.Cos(phi), sp = Math.Sin(phi);
        double minX = double.MaxValue, minY = double.MaxValue;
        foreach (int f in modFaces)
        {
          if (!inMod[f]) continue;
          double[] t = tfm[f];
          tfm[f] = new double[] { cp * t[0] - sp * t[1], sp * t[0] + cp * t[1], cp * t[2] - sp * t[3], sp * t[2] + cp * t[3] };
          Plate pp = plates[f];
          for (int j = 0; j < pp.M; j++) { Point2d q = Apply(tfm[f], pp.C[j]); minX = Math.Min(minX, q.X); minY = Math.Min(minY, q.Y); }
        }
        double modX = bb.Max.X + planDx + Math.Max(10.0, 0.15 * (bb.Max.X - bb.Min.X));
        foreach (int f in modFaces) if (inMod[f]) { tfm[f][2] += modX - minX; tfm[f][3] += bb.Min.Y - minY; }
      }
    }
    Func<Point3d, int, Point3d> ModMap = (X3, pfi) => { Point2d u = Apply(tfm[pfi], plates[pfi].To2(X3)); return new Point3d(u.X, u.Y, ground); };
    // drawn lines in plan: whole lines, cut only where the net opens; each with its strongest band
    var planPieces = new List<Polyline>[] { new List<Polyline>(), new List<Polyline>() };
    var pieceBand = new List<int>[] { new List<int>(), new List<int>() };
    var planHooks = new List<Polyline>();
    bool hooksOn = W("StrHooks", 1.0) > 0.5;
    int nLinesT = 0, nLinesC = 0;
    for (int family = 0; family < 2; family++)
    {
      double sMax = family == 0 ? sMaxT : -sMaxC;
      if (sMax <= 1e-9) continue;
      grid = new Dictionary<long, List<int>>(); gridPts = new List<Point3d>(); gridLine = new List<int>(); cell = baseSep;
      // seeds: element centres, strongest first
      var seeds = new List<int>();
      var sv = new double[elN.Count];
      for (int el = 0; el < elN.Count; el++)
      {
        double s1, s2, ang; Principal(elS[el][0], elS[el][1], elS[el][2], out s1, out s2, out ang);
        sv[el] = family == 0 ? s1 : -s2;
        if (sv[el] > 0.05 * sMax) seeds.Add(el);
      }
      seeds.Sort((x, y) => sv[y].CompareTo(sv[x]));
      int lineId = 0;
      // Equal force: walk the section that crosses the strongest part of the field at right angles
      // (along the other family's line), add up the force it carries, and start one line for every
      // perLine kips. Then the number of lines counts the force and their spacing is the bar spacing.
      var eqSeeds = new List<KeyValuePair<int, Point2d>>();
      if (perLine > 0)
      {
        int pk = -1; double pv = 0;
        for (int el = 0; el < elN.Count; el++)
        {
          int[] g0 = elN[el];
          if (fixedN[g0[0]] || fixedN[g0[1]] || fixedN[g0[2]]) continue;   // leave out the spikes at the supports
          if (sv[el] > pv) { pv = sv[el]; pk = el; }
        }
        if (pk >= 0)
        {
          int pi0 = elPlate[pk]; Plate p0 = plates[pi0]; int[] g1 = elN[pk];
          Point2d c0 = p0.To2((nodes[g1[0]] + nodes[g1[1]] + nodes[g1[2]]) / 3.0);
          Vector2d ds0; double dv0;
          if (PField(p0, c0, 1 - family, out ds0, out dv0))
          {
            List<LinePt> sa = Trace(pi0, c0, ds0, 1 - family, sMax, baseSep, h / 4.0, -7, true, true);
            List<LinePt> sb2 = Trace(pi0, c0, -ds0, 1 - family, sMax, baseSep, h / 4.0, -7, true, true);
            var sec = new List<LinePt>();
            for (int i = sb2.Count - 1; i >= 1; i--) sec.Add(sb2[i]);
            sec.AddRange(sa);
            double F = 0, next = 0.5 * perLine, fPrev = 0; int count = 0;
            for (int i = 0; i < sec.Count; i++)
            {
              Plate pp = plates[sec[i].Plate]; Point2d q2 = pp.To2(sec[i].X);
              Vector2d dd; double vv;
              double fNow = PField(pp, q2, family, out dd, out vv) ? Math.Max(0, family == 0 ? vv : -vv) * slab : 0;
              if (i > 0)
              {
                double dF = 0.5 * (fNow + fPrev) * sec[i].X.DistanceTo(sec[i - 1].X);
                while (F + dF >= next) { eqSeeds.Add(new KeyValuePair<int, Point2d>(sec[i].Plate, q2)); next += perLine; count++; }
                F += dF;
              }
              fPrev = fNow;
            }
            if (family == 0) { eqForceT = F; eqCountT = count; } else { eqForceC = F; eqCountC = count; }
          }
        }
      }
      var seedList = new List<KeyValuePair<int, Point2d>>(eqSeeds);
      int nEq = eqSeeds.Count;
      foreach (int el in seeds) seedList.Add(new KeyValuePair<int, Point2d>(-1 - el, Point2d.Origin));
      for (int si = 0; si < seedList.Count; si++)
      {
        bool isEq = si < nEq;
        int pIdx; Point2d c2;
        if (isEq) { pIdx = seedList[si].Key; c2 = seedList[si].Value; }
        else
        {
          int el = -1 - seedList[si].Key;
          pIdx = elPlate[el];
          int[] g = elN[el];
          Point3d cen = (nodes[g[0]] + nodes[g[1]] + nodes[g[2]]) / 3.0;
          double sep = baseSep * Math.Min(3.0, Math.Sqrt(sMax / sv[el]));
          if (TooClose(cen, sep, -1)) continue;
          c2 = plates[pIdx].To2(cen);
        }
        Plate p = plates[pIdx];
        Vector2d d0; double v0;
        if (!PField(p, c2, family, out d0, out v0)) continue;
        lineId++;
        List<LinePt> fwd = Trace(pIdx, c2, d0, family, sMax, baseSep, h, lineId, false, isEq);
        List<LinePt> back = Trace(pIdx, c2, -d0, family, sMax, baseSep, h, lineId, false, isEq);
        var all = new List<LinePt>();
        for (int i = back.Count - 1; i >= 1; i--) all.Add(back[i]);
        all.AddRange(fwd);
        double len = 0; for (int i = 1; i < all.Count; i++) len += all[i].X.DistanceTo(all[i - 1].X);
        if (all.Count < 3 || len < 2.0 * baseSep) continue;
        foreach (LinePt lp in all) AddToGrid(lp.X, lineId);
        if (family == 0) nLinesT++; else nLinesC++;
        // split into strength bands; lift to the top face
        var full = new Polyline();
        int curBand = -1, maxBand = 0; Polyline cur = null;
        var pieces = new List<Polyline>(); var piece = new Polyline();
        var pStartTrue = new List<bool>(); var pEndTrue = new List<bool>(); bool curStartTrue = true;
        var mpieces = new List<Polyline>(); var mpiece = new Polyline();
        var mStartTrue = new List<bool>(); var mEndTrue = new List<bool>(); bool mCurStartTrue = true;
        for (int i = 0; i < all.Count; i++)
        {
          double fr = Math.Abs(all[i].Val) / sMax;
          int band = isEq ? 1 : (fr > 0.5 ? 2 : (fr > 0.2 ? 1 : 0));   // equal-force lines: the spacing shows the force
          Point3d up = all[i].X + plates[all[i].Plate].N * lift;
          Point3d pl = PlanMap(all[i].X, all[i].Plate);
          if (!shown[all[i].Plate]) { if (piece.Count > 1) { pieces.Add(piece); pStartTrue.Add(curStartTrue); pEndTrue.Add(false); } piece = new Polyline(); curStartTrue = false; }
          else
          {
            if (piece.Count > 0 && piece[piece.Count - 1].DistanceTo(pl) > 4.0 * h) { if (piece.Count > 1) { pieces.Add(piece); pStartTrue.Add(curStartTrue); pEndTrue.Add(false); } piece = new Polyline(); curStartTrue = false; }
            piece.Add(pl);
          }
          if (modOn)
          {
            int pf2 = all[i].Plate;
            if (!inMod[pf2]) { if (mpiece.Count > 1) { mpieces.Add(mpiece); mStartTrue.Add(mCurStartTrue); mEndTrue.Add(false); } mpiece = new Polyline(); mCurStartTrue = false; }
            else
            {
              Point3d mq = ModMap(all[i].X, pf2);
              if (mpiece.Count > 0 && mpiece[mpiece.Count - 1].DistanceTo(mq) > 4.0 * h) { if (mpiece.Count > 1) { mpieces.Add(mpiece); mStartTrue.Add(mCurStartTrue); mEndTrue.Add(false); } mpiece = new Polyline(); mCurStartTrue = false; }
              mpiece.Add(mq);
            }
          }
          maxBand = Math.Max(maxBand, band);
          full.Add(up);
          if (band != curBand)
          {
            if (cur != null) { cur.Add(up); if (cur.Count > 1) (family == 0 ? bandT : bandC)[curBand].Add(cur); }
            cur = new Polyline(); curBand = band;
          }
          cur.Add(up);
        }
        if (cur != null && cur.Count > 1) (family == 0 ? bandT : bandC)[curBand].Add(cur);
        if (piece.Count > 1) { pieces.Add(piece); pStartTrue.Add(curStartTrue); pEndTrue.Add(true); }
        if (mpiece.Count > 1) { mpieces.Add(mpiece); mStartTrue.Add(mCurStartTrue); mEndTrue.Add(true); }
        if (full.Count > 1) (family == 0 ? tensionOut : compOut).Add(new PolylineCurve(full));
        for (int set = 0; set < 2; set++)
        {
          List<Polyline> ps = set == 0 ? pieces : mpieces;
          List<bool> st0 = set == 0 ? pStartTrue : mStartTrue, st1 = set == 0 ? pEndTrue : mEndTrue;
          for (int pi = 0; pi < ps.Count; pi++)
          {
            if (ps[pi].Length < 1.5 * baseSep) continue;
            Polyline sm = Smooth(ps[pi], 2);
            planPieces[family].Add(sm); pieceBand[family].Add(maxBand);
            if (family == 0 && hooksOn)
            {
              // hooks only at the real ends of a bar, not where the drawing cuts it
              int m = sm.Count;
              if (st0[pi]) { Polyline hk = Hook(sm[0], sm[0] - sm[Math.Min(2, m - 1)], 0.3 * baseSep); if (hk != null) planHooks.Add(hk); }
              if (st1[pi]) { Polyline hk = Hook(sm[m - 1], sm[m - 1] - sm[Math.Max(0, m - 3)], 0.3 * baseSep); if (hk != null) planHooks.Add(hk); }
            }
          }
        }
      }
    }

    for (int fam = 0; fam < 2; fam++)
      for (int k = 0; k < planPieces[fam].Count; k++) (fam == 0 ? planT : planC)[pieceBand[fam][k]].Add(planPieces[fam][k]);

    // ---------------- forces along the folds ----------------
    var foldLines = new List<Line>(); var foldVal = new List<double>(); var foldPsi = new List<double>();
    var segEl = new Dictionary<long, List<int>>();
    for (int el = 0; el < elN.Count; el++)
      for (int i = 0; i < 3; i++)
      {
        int a = elN[el][i], b = elN[el][(i + 1) % 3];
        long key = (long)Math.Min(a, b) * 1000003L + Math.Max(a, b);
        List<int> lst; if (!segEl.TryGetValue(key, out lst)) { lst = new List<int>(); segEl[key] = lst; }
        lst.Add(el);
      }
    double foldMax = 0;
    for (int e = 0; e < tes.Count; e++)
    {
      int[] fs = tes.GetConnectedFaces(e);
      if (fs == null || fs.Length != 2) continue;
      if (Vector3d.VectorAngle(fn[fs[0]], fn[fs[1]]) < 5.0 * Math.PI / 180.0) continue;
      Plate p = plates[fs[0]];
      int side = -1;
      Line el3 = tes.EdgeLine(e);
      for (int s = 0; s < p.M; s++)
      {
        Point3d a = p.To3(p.C[s]), b = p.To3(p.C[(s + 1) % p.M]);
        if ((a.DistanceTo(el3.From) < 1e-6 && b.DistanceTo(el3.To) < 1e-6) || (a.DistanceTo(el3.To) < 1e-6 && b.DistanceTo(el3.From) < 1e-6)) { side = s; break; }
      }
      if (side < 0) continue;
      Vector3d t3 = el3.Direction; t3.Unitize();
      double sum = 0, wsum = 0;
      int[] sn = p.SideNodes[side];
      for (int k = 0; k < n; k++)
      {
        long key = (long)Math.Min(sn[k], sn[k + 1]) * 1000003L + Math.Max(sn[k], sn[k + 1]);
        List<int> lst; if (!segEl.TryGetValue(key, out lst)) continue;
        double segL = nodes[sn[k]].DistanceTo(nodes[sn[k + 1]]);
        foreach (int el in lst)
        {
          Plate pp = plates[elPlate[el]];
          double tx = t3 * pp.E1, ty = t3 * pp.E2;
          double[] s = elS[el];
          sum += (s[0] * tx * tx + s[1] * ty * ty + 2 * s[2] * tx * ty) * segL; wsum += segL;
        }
      }
      if (wsum <= 0) continue;
      double st = sum / wsum;
      Vector3d upv = fn[fs[0]] + fn[fs[1]]; upv.Unitize();
      double cosA = Math.Max(0.3, upv * fn[fs[0]]);
      Vector3d shift = upv * (slab / 2.0 / cosA + 0.04);
      foldLines.Add(new Line(el3.From + shift, el3.To + shift));
      foldPsi.Add(st * kToPsi);
      foldMax = Math.Max(foldMax, Math.Abs(st));
    }
    foreach (double v in foldPsi) foldVal.Add(foldMax > 0 ? v / (foldMax * kToPsi) : 0);

    // ---------------- colour field, per plate, on the top face ----------------
    var field = new Mesh();
    double cMax = Math.Max(sMaxT, -sMaxC);
    foreach (Plate p in plates)
    {
      int baseV = field.Vertices.Count;
      for (int i = 0; i < p.P.Length; i++)
      {
        field.Vertices.Add(p.To3(p.P[i]) + p.N * (slab / 2.0 + 0.01));
        double s1, s2, ang; Principal(p.Sx[i], p.Sy[i], p.Txy[i], out s1, out s2, out ang);
        double s = Math.Abs(s1) >= Math.Abs(s2) ? s1 : s2;
        double fr = cMax > 0 ? Math.Max(-1, Math.Min(1, s / cMax)) : 0;
        int r = fr > 0 ? 255 : (int)(255 * (1 + fr)), gg = (int)(255 * (1 - Math.Abs(fr))), bl = fr < 0 ? 255 : (int)(255 * (1 - fr));
        field.VertexColors.Add(System.Drawing.Color.FromArgb(r, gg, bl));
      }
      foreach (int[] t in p.Tri) field.Faces.AddFace(baseV + t[0], baseV + t[1], baseV + t[2]);
    }
    field.Normals.ComputeNormals();

    // ---------------- plan: plate edges and supports ----------------
    var planEdges = new List<Line>(); var planHeavy = new List<Line>();
    double dashL = Math.Max(0.15, diag / 250.0);
    if (planStyle == 0)
    {
      for (int e = 0; e < tes.Count; e++)
      {
        Line l = tes.EdgeLine(e);
        planEdges.Add(new Line(new Point3d(l.From.X, l.From.Y, ground) + planShift, new Point3d(l.To.X, l.To.Y, ground) + planShift));
      }
    }
    else
    {
      // diagram: every fold one thin line, the outline a little heavier, near-flat creases left out.
      // Roof plates only (style 2): steep side plates are left out and the roof's edge becomes the outline.
      for (int e = 0; e < tes.Count; e++)
      {
        Line l = tes.EdgeLine(e);
        Line pln = new Line(new Point3d(l.From.X, l.From.Y, ground) + planShift, new Point3d(l.To.X, l.To.Y, ground) + planShift);
        int[] fs = tes.GetConnectedFaces(e);
        int nShown = 0; if (fs != null) foreach (int f in fs) if (shown[f]) nShown++;
        if (nShown == 0) continue;
        if (fs.Length < 2 || nShown == 1) { planHeavy.Add(pln); continue; }
        if (FoldKind(mesh, fn, fs[0], fs[1], l) != 0) planEdges.Add(pln);
      }
    }
    if (modOn)
    {
      double tolM = 1e-3 * Math.Max(1.0, baseSep);
      for (int f = 0; f < nF; f++)
      {
        if (!inMod[f]) continue;
        Plate pp = plates[f];
        for (int sd = 0; sd < pp.M; sd++)
        {
          Point3d A3 = pp.To3(pp.C[sd]), B3 = pp.To3(pp.C[(sd + 1) % pp.M]);
          Point3d a2 = ModMap(A3, f), b2 = ModMap(B3, f);
          int qf = pp.SideNb[sd];
          bool joined = qf >= 0 && inMod[qf] && ModMap(A3, qf).DistanceTo(a2) < tolM && ModMap(B3, qf).DistanceTo(b2) < tolM;
          if (joined)
          {
            if (qf < f) continue;
            if (FoldKind(mesh, fn, f, qf, new Line(A3, B3)) != 0) planEdges.Add(new Line(a2, b2));
          }
          else
          {
            planHeavy.Add(new Line(a2, b2));
            if (qf >= 0 && inMod[qf]) modGaps++;            // counted from both sides
          }
        }
      }
      modGaps /= 2;
    }
    var planSup = new List<Polyline>();
    double sq = Math.Max(0.4, diag / 150.0);
    var supPlan = new List<Point3d>();
    foreach (Point3d spt in supCorners) supPlan.Add(new Point3d(spt.X, spt.Y, ground) + planShift);
    if (modOn)
      for (int f = 0; f < nF; f++)
      {
        if (!inMod[f]) continue;
        Plate pp = plates[f];
        for (int j = 0; j < pp.M; j++)
        {
          Point3d c3 = pp.To3(pp.C[j]);
          int nd = NodeAt(c3);
          if (nd < N && fixedN[nd] && (supPts > 0 || c3.Z < ground + gTol)) supPlan.Add(ModMap(c3, f));
        }
      }
    foreach (Point3d c0 in supPlan)
      planSup.Add(new Polyline(new Point3d[] { c0 + new Vector3d(-sq, -sq, 0), c0 + new Vector3d(sq, -sq, 0), c0 + new Vector3d(sq, sq, 0), c0 + new Vector3d(-sq, sq, 0), c0 + new Vector3d(-sq, -sq, 0) }));
    var planOut = new List<Curve>();
    foreach (Line l in planEdges) planOut.Add(new LineCurve(l));
    foreach (Line l in planHeavy) planOut.Add(new LineCurve(l));
    foreach (Polyline pl in planSup) planOut.Add(new PolylineCurve(pl));
    for (int b = 0; b < 3; b++) { foreach (Polyline pl in planT[b]) planOut.Add(new PolylineCurve(pl)); foreach (Polyline pl in planC[b]) planOut.Add(new PolylineCurve(pl)); }
    foreach (Polyline pl in planHooks) planOut.Add(new PolylineCurve(pl));

    // publish for the control window to draw
    drawData = new object[] { bandT, bandC, foldLines.ToArray(), field, foldVal.ToArray(), planT, planC, planEdges.ToArray(), planSup, planHeavy.ToArray(), planStyle, planHooks };
    drawBox = bb; drawBox.Union(field.GetBoundingBox(false));
    foreach (Line l in planEdges) { drawBox.Union(l.From); drawBox.Union(l.To); }
    foreach (Line l in planHeavy) { drawBox.Union(l.From); drawBox.Union(l.To); }
    try { AppDomain.CurrentDomain.SetData("FoldToolkit.StressDraw", drawData); } catch { }

    // ---------------- report ----------------
    double tPsi = sMaxT * kToPsi, cPsi = -sMaxC * kToPsi;
    double allowC = 0.45 * fc, crack = 7.5 * Math.Sqrt(fc);
    double tIn = slab * 12.0;
    double asMax = sMaxT / 144.0 * tIn * 12.0 / 24.0;     // in2/ft: tension force per ft / 24 ksi
    double dEff = Math.Max(1.0, tIn - 1.0);
    double asBend = mMax * 12.0 / (24.0 * 0.875 * dEff);   // in2/ft for the peak slab moment
    double sTpsi = surfT * kToPsi, sCpsi = -surfC * kToPsi;
    double spanH = Math.Max(bb.Max.X - bb.Min.X, bb.Max.Y - bb.Min.Y);
    var sb = new StringBuilder();
    sb.AppendLine("STRESS: TENSION AND COMPRESSION (shell analysis: in-plane forces and slab bending)");
    sb.AppendLine(string.Format("Loads: self-weight {0:0.#} in slab at 150 pcf + live {1:0} psf on plan | f'c {2:0} psi", tIn, LL, fc));
    sb.AppendLine();
    sb.AppendLine("MODEL");
    sb.AppendLine(string.Format("  Plates / triangles / points {0} / {1} / {2}  (refine {3})", nF, elN.Count, N, n));
    sb.AppendLine(string.Format("  Supports                    {0} points, {1}", nSup, supNote));
    sb.AppendLine("                              " + (supPts == 0 ? "everything touching the ground" : (designerSupports ? string.Format("the {0} shared support points (from the Fold Designer or the FOLDLAB reader; wire Supports to be sure which)", supPts) : string.Format("from {0} given points", supPts))));
    sb.AppendLine(string.Format("  Solve                       {0} unknowns, exact solve in {1:0.0} s (total {2:0.0} s)", ndof, solveSec, sw.Elapsed.TotalSeconds));
    sb.AppendLine();
    sb.AppendLine("EQUILIBRIUM");
    sb.AppendLine(string.Format("  Total load                  {0:0.0} kips (self-weight {1:0.0})", totalLoad, swTotal));
    sb.AppendLine(string.Format("  Vertical reactions          {0:0.0} kips  (should match the load)", reactZ));
    sb.AppendLine();
    sb.AppendLine("FORCE FLOW IN THE PLATES (mid-surface: what the lines show)");
    sb.AppendLine(string.Format("  Peak tension                {0:0} psi  (concrete cracks above about {1:0} psi; steel takes the tension)", tPsi, crack));
    sb.AppendLine(string.Format("  Peak compression            {0:0} psi", cPsi));
    sb.AppendLine(string.Format("  Steel for peak tension      {0:0.00} in2 per ft at 24 ksi  (#4 at {1:0.#} in)", asMax, asMax > 0 ? Math.Min(18.0, 0.20 / asMax * 12.0) : 18.0));
    sb.AppendLine(string.Format("  Lines drawn                 {0} tension, {1} compression", nLinesT, nLinesC));
    if (perLine > 0) sb.AppendLine(string.Format("  Line density                one line = {0:0.0} kips ({1:0.0} #4 bars at 24 ksi). Across the strongest section: {2:0} kips of tension = {3} lines, {4:0} kips of compression = {5} lines", perLine, perLine / 4.8, eqForceT, eqCountT, eqForceC, eqCountC));
    sb.AppendLine();
    sb.AppendLine("SLAB BENDING ADDED (top and bottom faces)");
    sb.AppendLine(string.Format("  Peak moment in the slab     {0:0.00} kip-ft per ft  -> steel {1:0.00} in2 per ft (#4 at {2:0.#} in)", mMax, asBend, asBend > 0 ? Math.Min(18.0, 0.20 / asBend * 12.0) : 18.0));
    sb.AppendLine(string.Format("  Peak tension on a face      {0:0} psi", sTpsi));
    sb.AppendLine(string.Format("  Peak compression on a face  {0:0} psi  of {1:0} allowed (0.45 f'c)  {2}", sCpsi, allowC, sCpsi <= allowC ? "OK" : "TOO HIGH"));
    sb.AppendLine(string.Format("  Largest deflection          {0:0.00} in  (span / {1:0} over the longest plan side)", uMax * 12.0, uMax > 0 ? spanH / uMax : 99999));
    sb.AppendLine();
    // can the roof unroll flat in one piece? at every inner corner the plate angles must add to 360 deg
    int innerCorners = 0, offCorners = 0; double worst = 0; Point3d worstAt = Point3d.Origin;
    for (int tv = 0; tv < tvs.Count; tv++)
    {
      int[] ce = tvs.ConnectedEdges(tv);
      bool inner = ce != null && ce.Length > 0;
      if (inner) foreach (int e in ce) { int[] fs = tes.GetConnectedFaces(e); if (fs == null || fs.Length != 2) { inner = false; break; } }
      if (!inner) continue;
      innerCorners++;
      Point3d v0 = tvs[tv]; double sumA = 0;
      foreach (int f in tvs.ConnectedFaces(tv))
      {
        Plate pp = plates[f];
        for (int j = 0; j < pp.M; j++)
        {
          Point3d cj = pp.To3(pp.C[j]);
          if (cj.DistanceTo(v0) > 1e-6) continue;
          Vector3d u1 = pp.To3(pp.C[(j + 1) % pp.M]) - cj, u2 = pp.To3(pp.C[(j + pp.M - 1) % pp.M]) - cj;
          sumA += Vector3d.VectorAngle(u1, u2);
        }
      }
      double defect = 360.0 - sumA * 180.0 / Math.PI;
      if (Math.Abs(defect) > 0.5) offCorners++;
      if (Math.Abs(defect) > Math.Abs(worst)) { worst = defect; worstAt = v0; }
    }
    sb.AppendLine("CAN IT UNROLL FLAT?");
    if (offCorners == 0) sb.AppendLine(string.Format("  Yes: at all {0} inner corners the plate angles add up to 360 deg, so the roof unfolds from one flat sheet, like the paper model.", innerCorners));
    else sb.AppendLine(string.Format("  No: at {0} of {1} inner corners the plate angles do not add up to 360 deg (worst {2:+0.0;-0.0} deg at {3:0.0}, {4:0.0}, {5:0.0} ft). Short of 360 the unroll opens a gap; over 360 it would overlap. This roof cannot be made from one flat sheet folded up: it is cast, or built from separate panels.", offCorners, innerCorners, worst, worstAt.X, worstAt.Y, worstAt.Z));
    if (modOn) sb.AppendLine(string.Format("  Unrolled module: strips {0} to {1}, drawn at true shape beside the plan{2}.", modI0 + 1, modI0 + modW, modGaps > 0 ? string.Format(", with {0} slits where it cannot lie flat", modGaps) : ", in one piece"));
    if (modNote != null) sb.AppendLine(modNote);
    sb.AppendLine();
    sb.AppendLine("FLAGS");
    { string un = UnitNote(); if (un != null) sb.AppendLine("  - " + un); }
    if (Math.Abs(reactZ - totalLoad) > 0.01 * totalLoad) sb.AppendLine("  - Reactions and load differ by more than 1%: treat these results with caution and send the Report to check.");
    if (sCpsi > allowC) sb.AppendLine("  - Compression above the allowable: deepen the folds or thicken the slab where the blue lines crowd.");
    if (uMax > 0 && spanH / uMax < 240) sb.AppendLine("  - Deflection is more than span / 240: the roof is too flexible. Deepen the folds or add supports.");
    if (mMax > 0 && asBend > asMax) sb.AppendLine("  - Slab bending needs more steel than the force flow: the plates are wide for their thickness. Narrower plates or a thicker slab help.");
    if (planStyle == 2) sb.AppendLine(string.Format("  - Roof plates only: {0} plates steeper than {1:0} deg are left out of the plan, so it shows the roof surface only; the roof's edge is the heavier outline. The 3D view still shows every plate.", hiddenPlates, maxSlope));
    if (planStyle == 1) sb.AppendLine("  - Diagram: folds as thin lines, outline heavier, supports as solid squares; tension lines end in bar hooks. Untick compression for a reinforcement-only sheet.");
    sb.AppendLine("  - Red lines follow the principal tension, blue the principal compression. Lines are closer and heavier where the stress is higher: reinforcement follows the red lines.");
    sb.AppendLine("  - Fold lines are coloured by the stress along them: red folds work as ties, blue folds as struts.");

    // ---------------- bake ----------------
    bool bakeNow = false;
    try { object wb = AppDomain.CurrentDomain.GetData("FoldToolkit.StressBake"); if (wb is bool && (bool)wb) { bakeNow = true; AppDomain.CurrentDomain.SetData("FoldToolkit.StressBake", false); } } catch { }
    { bool bIn = Bake is bool ? (bool)Bake : (Bake is GH_Boolean && ((GH_Boolean)Bake).Value);
      if (bIn && !lastBakeInput) bakeNow = true;
      lastBakeInput = bIn; }
    if (bakeNow && RhinoDoc.ActiveDoc != null)
    {
      try
      {
        RhinoDoc doc = RhinoDoc.ActiveDoc;
        Rhino.DocObjects.RhinoObject[] old = doc.Objects.FindByUserString("FoldToolkitStress", "*", true);
        if (old != null) foreach (Rhino.DocObjects.RhinoObject ro in old) doc.Objects.Delete(ro, true);
        int parent = doc.Layers.FindByFullPath("Structure", -1);
        if (parent < 0) parent = doc.Layers.Add(new Rhino.DocObjects.Layer { Name = "Structure" });
        Guid pid = doc.Layers[parent].Id;
        Func<string, System.Drawing.Color, int> Lay = (nm, col) =>
        {
          int li = doc.Layers.FindByFullPath("Structure::" + nm, -1);
          if (li < 0) li = doc.Layers.Add(new Rhino.DocObjects.Layer { Name = nm, ParentLayerId = pid, Color = col });
          return li;
        };
        // colours and weights as set in the control window (section 9)
        Func<string, int, System.Drawing.Color> WCol = (key, def) => System.Drawing.Color.FromArgb(unchecked((int)(long)W(key, def)));
        System.Drawing.Color red = WCol("StrColT", unchecked((int)0xFFDC1E1E)), blue = WCol("StrColC", unchecked((int)0xFF1E50DC));
        System.Drawing.Color foldCol = WCol("StrColF", unchecked((int)0xFF141414)), supCol = WCol("StrColS", unchecked((int)0xFF000000));
        double wT = Math.Max(1, W("StrWT", 2)), wC = Math.Max(1, W("StrWC", 1)), wF = Math.Max(1, W("StrWF", 1));
        bool bandW = W("StrBandW", 1) > 0.5;
        double[] bf = bandW ? new double[] { 0.6, 1.0, 1.8 } : new double[] { 1, 1, 1 };
        int lT = Lay("Tension", red), lC = Lay("Compression", blue), lF = Lay("Fold forces", System.Drawing.Color.Black),
            lP = Lay("Plan", System.Drawing.Color.FromArgb(60, 60, 60)), lM = Lay("Field", System.Drawing.Color.White);
        double mmPerPx = 0.13;            // screen pixels to print millimetres
        int baked = 0;
        Func<int, System.Drawing.Color, double, Rhino.DocObjects.ObjectAttributes> Att = (li, col, w) =>
        {
          var a = new Rhino.DocObjects.ObjectAttributes();
          a.LayerIndex = li; a.ColorSource = Rhino.DocObjects.ObjectColorSource.ColorFromObject; a.ObjectColor = col;
          a.PlotWeightSource = Rhino.DocObjects.ObjectPlotWeightSource.PlotWeightFromObject; a.PlotWeight = w;
          a.SetUserString("FoldToolkitStress", "1");
          return a;
        };
        for (int b = 0; b < 3; b++)
        {
          foreach (Polyline pl in bandT[b]) { doc.Objects.AddPolyline(pl, Att(lT, red, mmPerPx * wT * bf[b])); baked++; }
          foreach (Polyline pl in bandC[b]) { doc.Objects.AddPolyline(pl, Att(lC, blue, mmPerPx * wC * bf[b])); baked++; }
          foreach (Polyline pl in planT[b]) { doc.Objects.AddPolyline(pl, Att(lP, red, mmPerPx * wT * bf[b])); baked++; }
          foreach (Polyline pl in planC[b]) { doc.Objects.AddPolyline(pl, Att(lP, blue, mmPerPx * wC * bf[b])); baked++; }
        }
        for (int i = 0; i < foldLines.Count; i++)
        { doc.Objects.AddLine(foldLines[i], Att(lF, foldVal[i] >= 0 ? red : blue, 0.18 + 0.7 * Math.Abs(foldVal[i]))); baked++; }
        foreach (Polyline pl in planHooks) { doc.Objects.AddPolyline(pl, Att(lP, red, mmPerPx * wT)); baked++; }
        foreach (Line l in planEdges) { doc.Objects.AddLine(l, Att(lP, foldCol, mmPerPx * wF)); baked++; }
        foreach (Line l in planHeavy) { doc.Objects.AddLine(l, Att(lP, foldCol, mmPerPx * (wF + 1))); baked++; }
        foreach (Polyline pl in planSup)
        {
          doc.Objects.AddPolyline(pl, Att(lP, supCol, 0.18)); baked++;
          {
            // solid black squares, for supports
            int solid = doc.HatchPatterns.FindName("Solid") != null ? doc.HatchPatterns.FindName("Solid").Index : -1;
            if (solid >= 0)
            {
              Hatch[] hs = Hatch.Create(new PolylineCurve(pl), solid, 0, 1, doc.ModelAbsoluteTolerance);
              if (hs != null) foreach (Hatch hh in hs) { doc.Objects.AddHatch(hh, Att(lP, supCol, 0.1)); baked++; }
            }
          }
        }
        { var a = Att(lM, System.Drawing.Color.White, 0.1); doc.Objects.AddMesh(field, a); baked++; }
        doc.Views.Redraw();
        sb.AppendLine(string.Format("  - Baked {0} objects on layer Structure: Tension, Compression, Fold forces, Plan, Field. Line weights print heavier where the stress is higher.", baked));
      }
      catch (Exception ex) { sb.AppendLine("  - Baking failed: " + ex.Message); }
    }

    Tension = tensionOut;
    Compression = compOut;
    Folds = foldLines;
    FoldForce = foldPsi;
    Plan = planOut;
    Field = field;
    Report = sb.ToString();
  }

}
