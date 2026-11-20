// Folded Plate Toolkit - Fold Designer
// Rhino 8 Grasshopper C# Script component. Model units: FEET.
//
// One path, one cross-section, and controls for how the section changes along
// the path. Every input is optional; with nothing connected it draws a default
// gable frame.
//
// SHAPE
//   Path        : planar curve the folds travel along (frame, arch, roof line)
//   Segments    : straight pieces used for a curved path (default 12)
//   Divisions   : extra stations inside each straight piece (default 1)
//   Profile     : cross-section drawn by hand.
//                 A POLYLINE is used exactly as drawn (any zigzag, spacing, depth)
//                 and replaces Bays / BayWidth / Depth / Variation.
//                 A SMOOTH CURVE is the overall shape of the section; folds are
//                 generated along it from Bays, Depth and Variation.
//   ProfileEnd  : a second cross-section with the same number of points; the
//                 section morphs from Profile to ProfileEnd along the path.
// GENERATED SECTION (used when no Profile is given)
//   Bays, BayWidth (ft), Depth (ft)
//   Variation   : 0 to 1. Randomizes each fold's width and depth (non-repeating).
//   Seed        : whole number; a different seed gives a different variation.
// ALONG THE PATH
//   Style       : 0 parallel folds, 1 counter-running (reverse) folds,
//                 2 zigzag (herringbone) folds. Add 10 for ridge to ridge
//                 (11, 12); otherwise ridge to valley. The window does this.
//   Amount      : 0 to 1, strength of the style. Counter-running turns about
//                 mid-depth: 0.5 flattens the folds, 1 turns a ridge into a valley
//   Period      : stations from one reversal to the next (default 1). Larger
//                 values stretch the diamonds along the path. (Input added at
//                 the bottom of the component.)
//   Lift        : ft. Raises the whole section where the fold is fully reversed,
//                 so the reversed fold sits above (or, if negative, below) the
//                 fold as drawn. Positive is always up, however the curves were
//                 drawn and whether or not Flip is on. Engel's "centre fold elevated over edge fold"
//                 is counter-running, Amount 1, Lift = fold depth.
//   DepthAlong  : list of multipliers spread along the path, e.g. 1, 0.3, 1
//   WidthAlong  : list of multipliers spread along the path, e.g. 1, 0.6
//                 (set both of these inputs to List Access)
// BUILD
//   Flip        : swap ridge and valley sides
//   Thickness   : in
// LAYOUT (new inputs, added at the bottom of the component)
//   Path2       : a second curve. Folds run between Path and Path2 instead of
//                 straight across. Uses Segments x Divisions stations.
//   Fan         : degrees. Fold lines fan about a vertical axis set back behind
//                 the START of the path (radial and conical folds). 360 closes
//                 the ring. The path must be drawn in a vertical plane.
//   FanRadius   : ft. Inner radius of the fan: how far the axis sits behind the
//                 start of the path. Empty or 0 = a safe radius worked out from
//                 the fold depth and slab thickness.
//   MomentDepth : true makes fold depth follow the bending moment of the path
//                 (pinned at both ends, uniform load): deep where bending is high.
//
// Outputs: Plates, Solid, Ridges, Valleys, Columns, Report
// Plates stay planar quads where the section is constant; where it changes
// along the path they are split into planar triangles.
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

  private static bool Flag(object o)
  {
    if (o == null) return false;
    if (o is bool) return (bool)o;
    if (o is GH_Boolean) return ((GH_Boolean)o).Value;
    return Num(o, 0.0) > 0.5;
  }

  private static List<double> NumList(object o)
  {
    var r = new List<double>();
    if (o == null) return r;
    if (!(o is string) && o is IEnumerable)
    {
      foreach (object item in (IEnumerable)o)
      {
        double v = Num(item, double.NaN);
        if (!double.IsNaN(v)) r.Add(v);
      }
      return r;
    }
    double s = Num(o, double.NaN);
    if (!double.IsNaN(s)) r.Add(s);
    return r;
  }

  // value of a list of multipliers at t (0 to 1) along the path
  private static double Along(List<double> vals, double t)
  {
    if (vals == null || vals.Count == 0) return 1.0;
    if (vals.Count == 1) return vals[0];
    double x = Math.Max(0.0, Math.Min(1.0, t)) * (vals.Count - 1);
    int i = (int)Math.Floor(x);
    if (i >= vals.Count - 1) return vals[vals.Count - 1];
    return vals[i] + (vals[i + 1] - vals[i]) * (x - i);
  }

  private static Curve AsCurve(object o)
  {
    if (o is Curve) return (Curve)o;
    if (o is GH_Curve) return ((GH_Curve)o).Value;
    if (o is Polyline) return new PolylineCurve((Polyline)o);
    if (o is Line) return new LineCurve((Line)o);
    if (o is Arc) return new ArcCurve((Arc)o);
    if (o == null) return null;
    // anything else Grasshopper itself can read as a curve (lines, arcs, referenced objects)
    Curve c = null;
    try { if (GH_Convert.ToCurve(o, ref c, GH_Conversion.Both)) return c; } catch { }
    return null;
  }

  // one label per straight run of a drawn path (or one for the whole curve)
  private static void PathDims(Curve c, List<Point3d> dp, List<string> dt)
  {
    if (c == null) return;
    Polyline pl;
    if (c.TryGetPolyline(out pl) && pl.Count >= 2 && pl.Count <= 16)
    {
      for (int i = 0; i < pl.Count - 1; i++)
      {
        double len = pl[i].DistanceTo(pl[i + 1]);
        if (len < 1e-6) continue;
        dp.Add(pl[i] + (pl[i + 1] - pl[i]) * 0.5);
        dt.Add(string.Format("{0:0.0} ft", len));
      }
    }
    else
    {
      dp.Add(c.PointAtNormalizedLength(0.5));
      dt.Add(string.Format("path {0:0.0} ft", c.GetLength()));
    }
  }

  private static string Received(object o)
  {
    if (o == null) return "nothing";
    Curve c = AsCurve(o);
    if (c == null) return "something that is not a curve (" + o.GetType().Name + ")";
    Polyline pl;
    if (c.TryGetPolyline(out pl)) return string.Format("polyline, {0} points, {1:0.0} ft", pl.Count, c.GetLength());
    return string.Format("smooth curve, {0:0.0} ft", c.GetLength());
  }

  // Read a hand-drawn cross-section: w = position across, d = offset from the path.
  // A polyline is used exactly as drawn. A smooth curve is treated as the
  // overall shape of the section and folds are generated along it.
  private static bool ReadProfile(object o, List<double> w, List<double> d,
    List<double> w0, List<double> d0,
    int bays, double depth, double vary, int seed, out bool smooth)
  {
    smooth = false;
    Curve c = AsCurve(o);
    if (c == null) return false;
    var pts = new List<Point3d>();
    Polyline pl;
    if (c.TryGetPolyline(out pl)) pts.AddRange(pl);
    else
    {
      Point3d[] dv;
      c.DivideByCount(2 * bays, true, out dv);
      if (dv != null) pts.AddRange(dv);
      smooth = true;
    }
    if (pts.Count < 3) return false;
    Vector3d chord = pts[pts.Count - 1] - pts[0];
    if (!chord.Unitize()) return false;
    Plane pp; double dev;
    Plane.FitPlaneToPoints(pts, out pp, out dev);
    Vector3d perp = Vector3d.CrossProduct(pp.ZAxis, chord);
    if (!perp.Unitize()) return false;
    for (int i = 0; i < pts.Count; i++)
    {
      Vector3d v = pts[i] - pts[0];
      w.Add(v * chord); d.Add(v * perp);
    }
    if (smooth)
    {
      // the drawn curve is the base line; remember it before adding the folds
      for (int i = 0; i < w.Count; i++) { w0.Add(w[i]); d0.Add(d[i]); }
      // push alternate points to either side of the curve to make the folds
      var rnd = new Random(seed);
      int n = w.Count;
      var w2 = new List<double>(); var d2 = new List<double>();
      for (int i = 0; i < n; i++)
      {
        int i0 = Math.Max(0, i - 1), i1 = Math.Min(n - 1, i + 1);
        double tw = w[i1] - w[i0], td = d[i1] - d[i0];
        double tl = Math.Sqrt(tw * tw + td * td);
        if (tl < 1e-12) { tw = 1.0; td = 0.0; tl = 1.0; }
        double off = (depth / 2.0) * (1.0 + vary * 0.8 * (2.0 * rnd.NextDouble() - 1.0));
        if (i % 2 == 0) off = -off;
        w2.Add(w[i] + off * (-td / tl));
        d2.Add(d[i] + off * (tw / tl));
      }
      for (int i = 0; i < n; i++) { w[i] = w2[i]; d[i] = d2[i]; }
    }
    double wMin = double.MaxValue, wMax = double.MinValue, dMin = double.MaxValue, dMax = double.MinValue;
    for (int i = 0; i < w.Count; i++)
    {
      wMin = Math.Min(wMin, w[i]); wMax = Math.Max(wMax, w[i]);
      dMin = Math.Min(dMin, d[i]); dMax = Math.Max(dMax, d[i]);
    }
    double cw = (wMin + wMax) / 2.0, cd = (dMin + dMax) / 2.0;
    for (int i = 0; i < w.Count; i++)
    {
      w[i] -= cw; d[i] -= cd;
      if (smooth) { w0[i] -= cw; d0[i] -= cd; }
      else { w0.Add(w[i]); d0.Add(0.0); }     // polyline: folds measured from the centre line
    }
    return true;
  }

  // Shape of the bending moment along the path, as fold-depth multipliers (0.25 to 1).
  // Supports: pinned at both ends of the path (two-hinged). Load: uniform along the path.
  private static double[] MomentShape(List<Point3d> st, out string info)
  {
    info = null;
    int n = st.Count;
    Point3d A = st[0], B = st[n - 1];
    Vector3d h = new Vector3d(B.X - A.X, B.Y - A.Y, 0.0);
    double L = h.Length;
    if (n < 3 || L < 1e-6)
    {
      info = "MomentDepth needs at least 3 stations and two ends that are apart in plan; it was ignored.";
      return null;
    }
    h.Unitize();
    var x = new double[n]; var y = new double[n]; var p = new double[n]; var arc = new double[n];
    for (int k = 0; k < n; k++)
    {
      Vector3d v = st[k] - A;
      x[k] = v * h;
      y[k] = (st[k].Z - A.Z) - (B.Z - A.Z) * x[k] / L;
      if (k > 0) { double seg = st[k].DistanceTo(st[k - 1]); p[k] += seg / 2.0; p[k - 1] += seg / 2.0; arc[k] = arc[k - 1] + seg; }
    }
    double total = 0.0, va = 0.0;
    for (int k = 0; k < n; k++) { total += p[k]; va += p[k] * (L - x[k]) / L; }
    var m0 = new double[n];
    double num = 0.0, den = 0.0;
    for (int j = 0; j < n; j++)
    {
      double sum = va * x[j];
      for (int k = 0; k < j; k++) sum -= p[k] * (x[j] - x[k]);
      m0[j] = sum;
      num += sum * y[j] * p[j]; den += y[j] * y[j] * p[j];
    }
    double thrust = den > 1e-9 * total * L * L ? num / den : 0.0;
    var mult = new double[n];
    double mMax = 0.0; int jMax = 0;
    for (int j = 0; j < n; j++)
    {
      double m = Math.Abs(m0[j] - thrust * y[j]);
      mult[j] = m;
      if (m > mMax) { mMax = m; jMax = j; }
    }
    if (mMax < 1e-4 * total * L / 8.0)
    {
      for (int j = 0; j < n; j++) mult[j] = 1.0;
      info = "the path is close to the pressure line for uniform load, so bending is negligible and fold depth is left constant";
      return mult;
    }
    for (int j = 0; j < n; j++) mult[j] = 0.25 + 0.75 * mult[j] / mMax;
    info = string.Format("pinned at both ends of the path, uniform load along the path; thrust = {0:0.00} x total load; peak bending at {1:0}% of the path length",
      thrust / total, 100.0 * arc[jMax] / Math.Max(1e-9, arc[n - 1]));
    return mult;
  }

  // Ridge and valley lines moved from the mid-surface onto the upper face of the
  // slab, so a shaded solid does not hide them. Every corner is moved once, to the
  // point that lies halfT above all the plates meeting there (the mitred top corner,
  // as in the solid), so lines that meet on the mid-surface still meet on top.
  private static void FoldsOnTop(Mesh m, double halfT, List<Line> ridgesTop, List<Line> valleysTop)
  {
    var tv = m.TopologyVertices;
    // face normals, all turned to the same side: the side that faces up overall
    var fn = new Vector3d[m.Faces.Count];
    double zSum = 0.0;
    for (int f = 0; f < m.Faces.Count; f++)
    {
      MeshFace mf = m.Faces[f];
      Point3d a = m.Vertices.Point3dAt(mf.A), b = m.Vertices.Point3dAt(mf.B), c = m.Vertices.Point3dAt(mf.C);
      Vector3d n = mf.IsQuad ? Vector3d.CrossProduct(c - a, m.Vertices.Point3dAt(mf.D) - b) : Vector3d.CrossProduct(b - a, c - a);
      zSum += n.Z;
      n.Unitize();
      fn[f] = n;
    }
    if (zSum < 0) for (int f = 0; f < fn.Length; f++) fn[f] = -fn[f];

    // the top corner over each topology vertex: least squares on the offset planes
    var top = new Point3d[tv.Count];
    for (int v = 0; v < tv.Count; v++)
    {
      int[] faces = tv.ConnectedFaces(v);
      double xx = 1e-6, xy = 0, xz = 0, yy = 1e-6, yz = 0, zz = 1e-6, bx = 0, by = 0, bz = 0;
      foreach (int f in faces)
      {
        Vector3d n = fn[f];
        xx += n.X * n.X; xy += n.X * n.Y; xz += n.X * n.Z; yy += n.Y * n.Y; yz += n.Y * n.Z; zz += n.Z * n.Z;
        bx += n.X * halfT; by += n.Y * halfT; bz += n.Z * halfT;
      }
      double det = xx * (yy * zz - yz * yz) - xy * (xy * zz - yz * xz) + xz * (xy * yz - yy * xz);
      Vector3d off = Vector3d.Zero;
      if (Math.Abs(det) > 1e-12)
      {
        off = new Vector3d(
          (bx * (yy * zz - yz * yz) - xy * (by * zz - yz * bz) + xz * (by * yz - yy * bz)) / det,
          (xx * (by * zz - yz * bz) - bx * (xy * zz - yz * xz) + xz * (xy * bz - by * xz)) / det,
          (xx * (yy * bz - by * yz) - xy * (xy * bz - by * xz) + bx * (xy * yz - yy * xz)) / det);
      }
      if (off.Length > 4.0 * halfT) { off.Unitize(); off *= 4.0 * halfT; }   // very sharp folds: cap it, like the solid
      Point3d p = tv[v];
      top[v] = p + off + (faces.Length > 0 ? fn[faces[0]] * 0.01 : Vector3d.Zero);
    }

    for (int e = 0; e < m.TopologyEdges.Count; e++)
    {
      int[] fs = m.TopologyEdges.GetConnectedFaces(e);
      if (fs == null || fs.Length != 2) continue;
      Line ln = m.TopologyEdges.EdgeLine(e);
      MeshFace f1 = m.Faces[fs[0]], f2 = m.Faces[fs[1]];
      Vector3d n1 = Vector3d.CrossProduct(m.Vertices.Point3dAt(f1.B) - m.Vertices.Point3dAt(f1.A), m.Vertices.Point3dAt(f1.C) - m.Vertices.Point3dAt(f1.A));
      if (!n1.Unitize()) continue;
      // ridge or valley, the same test the Ridges and Valleys outputs use
      int[] idx = f2.IsQuad ? new int[] { f2.A, f2.B, f2.C, f2.D } : new int[] { f2.A, f2.B, f2.C };
      double best = 0.0; Point3d far = ln.From;
      foreach (int v in idx)
      {
        Point3d q = m.Vertices.Point3dAt(v);
        double d = ln.DistanceTo(q, false);
        if (d > best) { best = d; far = q; }
      }
      double s = n1 * (far - ln.From);
      bool isRidge = s < -1e-6 * Math.Max(1.0, best);
      bool isValley = s > 1e-6 * Math.Max(1.0, best);
      if (!isRidge && !isValley) continue;
      IndexPair ends = m.TopologyEdges.GetTopologyVertices(e);
      Line onTop = new Line(top[ends.I], top[ends.J]);
      if (isRidge) ridgesTop.Add(onTop); else valleysTop.Add(onTop);
    }
  }

  private void RunScript(
    object Path, object Segments, object Divisions, object Profile, object ProfileEnd,
    object Bays, object BayWidth, object Depth, object Variation, object Seed,
    object Style, object Amount, object DepthAlong, object WidthAlong,
    object Flip, object Thickness,
    object Path2, object Fan, object MomentDepth, object FanRadius, object Period, object Lift,
    ref object Plates, ref object Solid, ref object Ridges, ref object Valleys,
    ref object Columns, ref object Report)
  {
    int segs = (int)Math.Round(Num(Segments, 12.0)); if (segs < 2) segs = 12;
    int divs = (int)Math.Round(Num(Divisions, 1.0)); if (divs < 1) divs = 1;
    int bays = (int)Math.Round(Num(Bays, 4.0)); if (bays < 1) bays = 4;
    double bayW = Num(BayWidth, 10.0); if (bayW <= 0) bayW = 10.0;
    double depth = Num(Depth, 4.0); if (depth <= 0) depth = 4.0;
    double vary = Math.Max(0.0, Math.Min(1.0, Num(Variation, 0.0)));
    int seed = (int)Math.Round(Num(Seed, 1.0));
    int styleRaw = (int)Math.Round(Num(Style, 0.0));
    int style = styleRaw % 10; if (style < 0 || style > 2) style = 0;
    int revEnds = styleRaw >= 10 ? 1 : 0;   // the window adds 10 for ridge to ridge
    double amount = Math.Max(0.0, Math.Min(1.0, Num(Amount, 1.0)));
    int period = (int)Math.Round(Num(Period, 1.0)); if (period < 1) period = 1;   // stations from one reversal to the next
    double lift = Num(Lift, 0.0);   // ft the whole section is raised where the fold is fully reversed
    List<double> depthAlong = NumList(DepthAlong);
    List<double> widthAlong = NumList(WidthAlong);
    bool flip = Flag(Flip);
    double thk = Num(Thickness, 5.0); if (thk <= 0) thk = 5.0;
    var notes = new List<string>();

    try { AppDomain.CurrentDomain.SetData("FoldToolkit.SupportPts", null); } catch { }

    // ---------------- path ----------------
    Curve crv = AsCurve(Path);
    var raw = new List<Point3d>();
    if (crv == null)
    {
      raw.Add(new Point3d(0, 0, 0)); raw.Add(new Point3d(0, 0, 14));
      raw.Add(new Point3d(0, 30, 22)); raw.Add(new Point3d(0, 60, 14));
      raw.Add(new Point3d(0, 60, 0));
      notes.Add("No Path connected: showing the default gable frame.");
    }
    else
    {
      Polyline pl;
      if (crv.TryGetPolyline(out pl)) raw.AddRange(pl);
      else
      {
        Point3d[] dv;
        crv.DivideByCount(segs, true, out dv);
        if (dv != null) raw.AddRange(dv);
        notes.Add("Curved path sampled into " + segs + " straight pieces.");
      }
    }
    for (int k = raw.Count - 1; k > 0; k--)
      if (raw[k].DistanceTo(raw[k - 1]) < 1e-6) raw.RemoveAt(k);
    if (raw.Count < 2) { Report = "Path needs at least two distinct points."; return; }

    // plane of the path (from the corner points, before adding stations)
    Plane pln;
    if (raw.Count == 2)
    {
      Vector3d dir = raw[1] - raw[0];
      Vector3d up = Math.Abs(dir.Z) < 0.99 * dir.Length ? Vector3d.ZAxis : Vector3d.XAxis;
      pln = new Plane(raw[0], dir, up);
    }
    else
    {
      double dev;
      Plane.FitPlaneToPoints(raw, out pln, out dev);
      if (dev > 0.01) notes.Add(string.Format("Path is not planar (off by {0:0.00} ft); it was flattened onto its best-fit plane.", dev));
    }
    Vector3d nrm = pln.ZAxis;
    double lead = Math.Abs(nrm.X) > 1e-6 ? nrm.X : (Math.Abs(nrm.Y) > 1e-6 ? nrm.Y : nrm.Z);
    if (lead < 0) pln = new Plane(pln.Origin, pln.YAxis, pln.XAxis);

    // ---------------- cross-section ----------------
    var wA = new List<double>(); var dA = new List<double>();
    var wB = new List<double>(); var dB = new List<double>();
    var w0A = new List<double>(); var d0A = new List<double>();   // base line under the folds
    var w0B = new List<double>(); var d0B = new List<double>();
    bool smoothA, smoothB;
    bool drawn = ReadProfile(Profile, wA, dA, w0A, d0A, bays, depth, vary, seed, out smoothA);
    if (!drawn)
    {
      wA.Clear(); dA.Clear(); w0A.Clear(); d0A.Clear();
      var rnd = new Random(seed);
      int n = 2 * bays + 1;
      double x = 0.0;
      for (int i = 0; i < n; i++)
      {
        if (i > 0) x += (bayW / 2.0) * (1.0 + vary * 0.6 * (2.0 * rnd.NextDouble() - 1.0));
        double dd = (depth / 2.0) * (1.0 + vary * 0.8 * (2.0 * rnd.NextDouble() - 1.0));
        wA.Add(x);
        dA.Add((i % 2 == 1) ? dd : -dd);
      }
      for (int i = 0; i < n; i++) { wA[i] -= x / 2.0; w0A.Add(wA[i]); d0A.Add(0.0); }
    }
    bool morph = ReadProfile(ProfileEnd, wB, dB, w0B, d0B, bays, depth, vary, seed, out smoothB);
    if (morph && wB.Count != wA.Count)
    {
      notes.Add(string.Format("ProfileEnd has {0} points but the section has {1}; ProfileEnd ignored.", wB.Count, wA.Count));
      morph = false;
    }
    int row = wA.Count;
    int nx = row - 1;
    double avgHalf = (wA[row - 1] - wA[0]) / Math.Max(1, nx);
    double side = flip ? -1.0 : 1.0;

    // ---------------- stations ----------------
    // Corners of the path, plus Divisions along each run. Extra stations are kept
    // clear of each bend by the distance the section needs to turn the corner,
    // so plates do not cross on the inside of the bend.
    double dmMax = 1.0;
    foreach (double m in depthAlong) dmMax = Math.Max(dmMax, Math.Abs(m));
    double reach = 0.0;
    for (int i = 0; i < row; i++)
    {
      reach = Math.Max(reach, Math.Abs(d0A[i]) + dmMax * Math.Abs(dA[i] - d0A[i]));
      if (morph) reach = Math.Max(reach, Math.Abs(d0B[i]) + dmMax * Math.Abs(dB[i] - d0B[i]));
    }
    if (style != 0) reach += Math.Abs(lift);
    var margin = new double[raw.Count];
    for (int k = 1; k < raw.Count - 1; k++)
    {
      Vector3d e0 = raw[k] - raw[k - 1], e1 = raw[k + 1] - raw[k];
      e0.Unitize(); e1.Unitize();
      double cs = e0 * e1;
      margin[k] = 1.15 * reach * Math.Sqrt(Math.Max(0.0, 1.0 - cs) / Math.Max(1e-9, 1.0 + cs));
    }
    var pts = new List<Point3d>();
    bool squeezed = false, tight = false;
    for (int k = 0; k < raw.Count - 1; k++)
    {
      Vector3d seg = raw[k + 1] - raw[k];
      double runLen = seg.Length; seg.Unitize();
      double m0 = margin[k], m1 = margin[k + 1];
      pts.Add(raw[k]);
      if (m0 + m1 > runLen) squeezed = true;
      else if (m0 + m1 > 0.6 * runLen) tight = true;
      if (divs > 1 && m0 + m1 < 0.95 * runLen)
        for (int j = 0; j <= divs; j++)
        {
          double sAlong = m0 + (runLen - m0 - m1) * j / (double)divs;
          if (sAlong > 1e-6 && sAlong < runLen - 1e-6) pts.Add(raw[k] + seg * sAlong);
        }
    }
    pts.Add(raw[raw.Count - 1]);
    int np = pts.Count;

    var pu = new List<double>(); var pv = new List<double>();
    for (int k = 0; k < np; k++)
    {
      double u, v;
      pln.ClosestParameter(pts[k], out u, out v);
      pu.Add(u); pv.Add(v);
    }
    var nu = new List<double>(); var nv = new List<double>();
    var cum = new List<double>(); cum.Add(0.0);
    for (int k = 0; k < np - 1; k++)
    {
      double du = pu[k + 1] - pu[k], dv2 = pv[k + 1] - pv[k];
      double len = Math.Sqrt(du * du + dv2 * dv2);
      nu.Add(-dv2 / len); nv.Add(du / len);
      cum.Add(cum[k] + len);
    }
    double pathLen = cum[np - 1];

    Vector3d lvl = Vector3d.CrossProduct(pln.ZAxis, Vector3d.ZAxis);
    bool hasLvl = lvl.Unitize();
    double gu = hasLvl ? lvl * pln.XAxis : 0.0, gv = hasLvl ? lvl * pln.YAxis : 0.0;
    bool groundStart = false, groundEnd = false;
    var ou = new List<double>(); var ov = new List<double>();
    double sharpest = 180.0;
    for (int k = 0; k < np; k++)
    {
      if (k == 0 || k == np - 1)
      {
        int sg = (k == 0) ? 0 : np - 2;
        double gn = gu * nu[sg] + gv * nv[sg];
        if (hasLvl && Math.Abs(gn) > 0.7)   // only legs steeper than about 45 degrees
        {
          ou.Add(gu / gn); ov.Add(gv / gn);
          if (k == 0) groundStart = true; else groundEnd = true;
        }
        else { ou.Add(nu[sg]); ov.Add(nv[sg]); }
      }
      else
      {
        double dot = nu[k - 1] * nu[k] + nv[k - 1] * nv[k];
        double den = Math.Max(1.0 + dot, 0.05);
        ou.Add((nu[k - 1] + nu[k]) / den); ov.Add((nv[k - 1] + nv[k]) / den);
        double ang = 180.0 - Math.Acos(Math.Max(-1.0, Math.Min(1.0, dot))) * 180.0 / Math.PI;
        if (ang < sharpest) sharpest = ang;
      }
    }

    // Near an end that is seated on the ground, ease from the level direction
    // back to the mitered direction, so the first stations do not dip below ground.
    {
      double easeLen = 1.5 * reach;
      for (int pass = 0; pass < 2; pass++)
      {
        if (pass == 0 && !groundStart) continue;
        if (pass == 1 && !groundEnd) continue;
        int ke = (pass == 0) ? 0 : np - 1;
        for (int j = 0; j < np; j++)
        {
          if (j == ke) continue;
          double dist = Math.Abs(cum[j] - cum[ke]);
          if (dist < easeLen)
          {
            double f = dist / easeLen;
            ou[j] = ou[ke] * (1.0 - f) + ou[j] * f;
            ov[j] = ov[ke] * (1.0 - f) + ov[j] * f;
          }
        }
      }
    }

    // ---------------- layout: one path, a fan, or two paths ----------------
    Curve crv2 = AsCurve(Path2);
    bool twoPaths = (crv2 != null && crv != null);
    if (crv2 != null && crv == null) notes.Add("Path2 needs Path as well; Path2 was ignored.");
    double fanDeg = Math.Max(-360.0, Math.Min(360.0, Num(Fan, 0.0)));
    bool fanOn = !twoPaths && Math.Abs(fanDeg) > 1e-6;
    if (twoPaths && Math.Abs(fanDeg) > 1e-6) notes.Add("Fan is ignored when Path2 is connected.");
    if (fanOn && Math.Abs(pln.ZAxis.Z) > 0.05)
    {
      notes.Add("Fan needs a path drawn in a vertical plane (Front or Right view); Fan was ignored.");
      fanOn = false;
    }
    bool closed = fanOn && Math.Abs(fanDeg) >= 359.9;
    if (closed)
    {
      // a ring can only close if the fan angle is the same at every station
      bool widthVaries = false;
      foreach (double wv in widthAlong) if (Math.Abs(wv - 1.0) > 1e-9) widthVaries = true;
      if (widthVaries)
      {
        closed = false;
        notes.Add("WidthAlong changes the fan angle along the path, so the 360 degree ring is left open along one seam. Disconnect WidthAlong to close it.");
      }
    }
    double fanRad = fanDeg * Math.PI / 180.0;

    // Inner radius of the fan: the start of the path sits this far from the axis,
    // so fold lines never reach the axis and plates do not cross at the centre.
    double fanR = 0.0, fanRNeeded = 0.0;
    bool fanRAuto = true;
    Point3d fanAxis = pts[0];
    if (fanOn)
    {
      fanRNeeded = 1.25 * reach;                                           // clears the fold depth
      double fanRAutoVal = fanRNeeded + row * (thk / 12.0) / Math.Max(0.2, Math.Abs(fanRad));  // and keeps plates wider than the slab
      double asked = Num(FanRadius, 0.0);
      if (asked > 0.0) { fanR = asked; fanRAuto = false; } else fanR = fanRAutoVal;
      Vector3d outDir = lvl;
      if ((pts[np - 1] - pts[0]) * outDir < 0) outDir = -outDir;           // away from the axis, along the path
      fanAxis = pts[0] - outDir * fanR;
    }

    double wLo = double.MaxValue, wHi = double.MinValue;
    for (int i = 0; i < row; i++) { wLo = Math.Min(wLo, wA[i]); wHi = Math.Max(wHi, wA[i]); }
    double wSpan = Math.Max(1e-9, wHi - wLo);

    var stA = new List<Point3d>(); var stB = new List<Point3d>();
    var stM = new List<Point3d>(); var stN = new List<Vector3d>();
    var stO = new List<Vector3d>();   // direction the fold offsets are measured along, per station
    bool cornerMatched = false, eased = false;
    if (twoPaths)
    {
      // If both paths are polylines with the same number of points, put a station
      // on every corner (corner 1 to corner 1, corner 2 to corner 2 ...) and
      // subdivide each run. Otherwise space the stations evenly along both curves.
      Polyline plA = null, plB = null;
      bool isPlA = crv.TryGetPolyline(out plA);
      bool isPlB = crv2.TryGetPolyline(out plB);
      cornerMatched = isPlA && isPlB && plA != null && plB != null && plA.Count == plB.Count && plA.Count >= 2;
      squeezed = false; tight = false;
      if (cornerMatched)
      {
        // Work on the line midway between the two paths. Extra stations on each
        // run are kept clear of the corners by the distance the section needs to
        // turn them, exactly as in single-path mode, so plates do not cross on
        // the inside of a corner.
        int nc = plA.Count;
        var midC = new List<Point3d>();
        for (int k = 0; k < nc; k++) midC.Add(plA[k] + (plB[k] - plA[k]) * 0.5);
        var clear = new double[nc];
        for (int k = 1; k < nc - 1; k++)
        {
          Vector3d c0 = midC[k] - midC[k - 1], c1 = midC[k + 1] - midC[k];
          if (!c0.Unitize() || !c1.Unitize()) continue;
          double cs = c0 * c1;
          clear[k] = 1.15 * reach * Math.Sqrt(Math.Max(0.0, 1.0 - cs) / Math.Max(1e-9, 1.0 + cs));
        }
        for (int k = 0; k < nc - 1; k++)
        {
          double runLen = midC[k].DistanceTo(midC[k + 1]);
          double m0 = clear[k], m1 = clear[k + 1];
          stA.Add(plA[k]); stB.Add(plB[k]);
          if (runLen < 1e-9) continue;
          if (m0 + m1 > runLen) squeezed = true;
          else if (m0 + m1 > 0.6 * runLen) tight = true;
          if (divs > 1 && m0 + m1 < 0.95 * runLen)
            for (int j = 0; j <= divs; j++)
            {
              double sAlong = m0 + (runLen - m0 - m1) * j / (double)divs;
              if (sAlong > 1e-6 && sAlong < runLen - 1e-6)
              {
                double fr = sAlong / runLen;
                stA.Add(plA[k] + (plA[k + 1] - plA[k]) * fr);
                stB.Add(plB[k] + (plB[k + 1] - plB[k]) * fr);
              }
            }
        }
        stA.Add(plA[nc - 1]);
        stB.Add(plB[nc - 1]);
      }
      else
      {
        if (isPlA && isPlB && plA != null && plB != null)
          notes.Add(string.Format("The two paths are polylines with different numbers of points ({0} and {1}), so stations are spaced evenly and can miss the corners. Give both paths the same number of points to follow every corner.", plA.Count, plB.Count));
        int nEven = segs * divs + 1;
        for (int k = 0; k < nEven; k++)
        {
          double tk = k / (double)(nEven - 1);
          stA.Add(crv.PointAtNormalizedLength(tk));
          stB.Add(crv2.PointAtNormalizedLength(tk));
        }
      }
      int nSt = stA.Count;
      // make the two curves run the same way
      if (stA[0].DistanceTo(stB[0]) + stA[nSt - 1].DistanceTo(stB[nSt - 1])
          > stA[0].DistanceTo(stB[nSt - 1]) + stA[nSt - 1].DistanceTo(stB[0])) stB.Reverse();
      for (int k = 0; k < nSt; k++) stM.Add(stA[k] + (stB[k] - stA[k]) * 0.5);
      np = nSt;
      cum.Clear(); cum.Add(0.0);
      for (int k = 1; k < np; k++) cum.Add(cum[k - 1] + stM[k].DistanceTo(stM[k - 1]));
      pathLen = cum[np - 1];
      Vector3d prevN = Vector3d.ZAxis;
      for (int k = 0; k < np; k++)
      {
        Vector3d t0 = stM[k] - stM[Math.Max(0, k - 1)], t1 = stM[Math.Min(np - 1, k + 1)] - stM[k];
        t0.Unitize(); t1.Unitize();
        Vector3d tan = t0 + t1;                              // bisects the turn at this station
        Vector3d nn = Vector3d.CrossProduct(stB[k] - stA[k], tan);
        if (!nn.Unitize()) nn = prevN;
        if (nn * prevN < 0) nn = -nn;
        prevN = nn;
        stN.Add(nn);
      }
      // Spread every change of direction over a length the section can follow.
      // Without this the offset fold lines cross on the inside of a corner.
      double win = 1.5 * reach;
      double spacing = pathLen / Math.Max(1, np - 1);
      if (!cornerMatched && win > spacing)
      {
        var smooth = new List<Vector3d>();
        for (int k = 0; k < np; k++)
        {
          Vector3d acc = Vector3d.Zero;
          for (int j = 0; j < np; j++)
          {
            double dist = Math.Abs(cum[j] - cum[k]);
            if (dist < win) acc += stN[j];   // even weighting turns the section at a steady rate
          }
          if (!acc.Unitize()) acc = stN[k];
          smooth.Add(acc);
        }
        stN = smooth;
        eased = true;
      }
      else if (!cornerMatched && reach > 0.25 * spacing)
        notes.Add("Stations are too far apart to ease the section around corners: raise Segments or Divisions.");
      groundStart = false; groundEnd = false; sharpest = 180.0;
      // Seat the ends on the ground: where an end rises steeply, slide the fold
      // offsets along the level direction so every fold line bears at the height
      // of the two path ends, instead of being cut square to the path.
      for (int k = 0; k < np; k++) stO.Add(stN[k]);
      // Stations sitting on sharp corners with no easing: lengthen the offset so
      // the fold keeps its full depth measured square to the runs either side (a miter).
      if (!eased)
        for (int k = 1; k < np - 1; k++)
        {
          Vector3d e0 = stM[k] - stM[k - 1], e1 = stM[k + 1] - stM[k];
          if (!e0.Unitize() || !e1.Unitize()) continue;
          double half = Math.Sqrt(Math.Max(0.1, (1.0 + e0 * e1) / 2.0));   // cosine of half the turn
          stO[k] = stN[k] * (1.0 / half);
        }
      for (int pass = 0; pass < 2 && np > 1; pass++)
      {
        int k = (pass == 0) ? 0 : np - 1;
        Vector3d tan = (pass == 0) ? stM[1] - stM[0] : stM[np - 1] - stM[np - 2];
        if (!tan.Unitize()) continue;
        Vector3d nn = stN[k];
        Vector3d level = nn * tan.Z - tan * nn.Z;          // lies in the plane of nn and tan, with no rise
        if (!level.Unitize()) continue;
        double along = level * nn;
        if (Math.Abs(along) > 0.7)                         // only ends steeper than about 45 degrees
        {
          stO[k] = level * (1.0 / along);
          if (pass == 0) groundStart = true; else groundEnd = true;
          // Ease from the level direction back to square-to-the-surface over the
          // first stretch of the leg. Offsets square to a leaning leg point partly
          // downward, which pushed the first stations below the ground.
          double easeLen = 1.5 * reach;
          for (int j = 0; j < np; j++)
          {
            if (j == k) continue;
            double dist = Math.Abs(cum[j] - cum[k]);
            if (dist < easeLen) { double f = dist / easeLen; stO[j] = stO[k] * (1.0 - f) + stO[j] * f; }
          }
        }
      }
      notes.RemoveAll(note => note.Contains("flattened") || note.Contains("sampled into"));
    }

    // depth following the bending moment
    bool momentOn = Flag(MomentDepth);
    double[] mm = null;
    string momentInfo = null;
    if (momentOn)
    {
      mm = MomentShape(twoPaths ? stM : pts, out momentInfo);
      if (mm == null) { notes.Add(momentInfo); momentOn = false; }
    }

    // position of a point of the section: k = station, w = across, d = offset from the path
    Func<int, double, double, Point3d> Pos = (k, w, d) =>
    {
      if (twoPaths)
        return stA[k] + (stB[k] - stA[k]) * (0.5 + w / wSpan) + stO[k] * (side * d);
      double u = pu[k] + side * d * ou[k], v = pv[k] + side * d * ov[k];
      if (!fanOn) return pln.PointAt(u, v, w);
      Point3d q = pln.PointAt(u, v, 0.0);
      q.Transform(Transform.Rotation((w / wSpan) * fanRad, Vector3d.ZAxis, fanAxis));
      return q;
    };

    // Which way is up for the apex lift. The side the fold offsets are measured
    // to depends on how the curves were drawn, so find it from the station where
    // that direction is most vertical. A positive lift then always raises the fold,
    // whatever the drawing direction and whether or not Flip is ticked.
    double outSign = 1.0;
    {
      double mostVertical = 1e-6;
      for (int k = 0; k < np; k++)
      {
        Vector3d dir = twoPaths ? stO[k] : pln.XAxis * ou[k] + pln.YAxis * ov[k];
        if (Math.Abs(dir.Z) > mostVertical) { mostVertical = Math.Abs(dir.Z); outSign = dir.Z > 0 ? 1.0 : -1.0; }
      }
    }

    if (style != 0 && amount < 1e-6 && Math.Abs(lift) < 1e-9)
      notes.Add("Style is " + (style == 1 ? "counter-running" : "zigzag") + " but Style amount and Apex lift are both 0, so the folds are unchanged. Raise Style amount (try 0.5).");
    else if (style != 0 && revEnds == 1 && np < 3)
      notes.Add("Ridge to ridge needs a station in the middle of the span. Raise Stations each to 2 or more.");
    else if (style != 0 && revEnds == 1 && (np - 1) % 2 == 1)
      notes.Add(string.Format("Ridge to ridge over {0} spans: with an odd number the full turn falls between stations, so the folds never fully reverse. Change Stations each to get an even number of spans.", np - 1));
    else if (style != 0)
      notes.Add(string.Format("Style: {0} over {1} stations. {2}", revEnds == 0 ? "ridge to valley" : "ridge to ridge", np,
        style == 1 && amount < 0.999 ? string.Format("Style amount {0:0.00} turns the folds only part way; at 0.5 they flatten out, at 1 a ridge becomes a valley.", amount) : ""));

    // ---------------- vertices ----------------
    double[,] dd2 = new double[np, row];
    Point3d[,] grid = new Point3d[np, row];
    double dLo = double.MaxValue, dHi = double.MinValue;
    double w00 = 0.0, d00 = 0.0;
    Mesh mesh = new Mesh();
    for (int k = 0; k < np; k++)
    {
      double t = pathLen > 0 ? cum[k] / pathLen : 0.0;
      double dm = Along(depthAlong, t) * (momentOn ? mm[k] : 1.0), wm = Along(widthAlong, t);
      // how far this station is into a reversal: 0 = as drawn, 1 = fully reversed.
      // The path is split into half-turns of about "period" stations each. Ridge to
      // valley uses an odd number, so a ridge at the start ends as a valley; ridge to
      // ridge uses an even number, so the folds come back to where they began.
      double rev = 0.0;
      if (np > 1)
      {
        // pick the number of half-turns nearest "period" stations each that is odd
        // (ridge to valley) or even (ridge to ridge) and lands every turn on a station
        int want = Math.Max(1, (int)Math.Round((np - 1) / (double)period));
        int halfTurns = revEnds == 0 ? 1 : 2;
        for (int h = want; h >= 1; h--)
          if (h % 2 == (revEnds == 0 ? 1 : 0) && (np - 1) % h == 0) { halfTurns = h; break; }
        double ph = (double)k / (np - 1) * halfTurns;
        double frc = ph % 2.0;
        rev = frc <= 1.0 ? frc : 2.0 - frc;
      }
      // fold height at this station, so counter-running turns about mid-depth:
      // ridges and valleys swap but the surface keeps the same depth and height
      double fdLo = double.MaxValue, fdHi = double.MinValue;
      for (int i = 0; i < row; i++)
      {
        double fdi = dA[i] - d0A[i];
        if (morph) fdi += ((dB[i] - d0B[i]) - fdi) * t;
        fdLo = Math.Min(fdLo, fdi); fdHi = Math.Max(fdHi, fdi);
      }
      double fdMid = 0.5 * (fdLo + fdHi);
      // the two outer edges of the roof at this station: zigzag folds stay between them
      double wEdgeLo = double.MaxValue, wEdgeHi = double.MinValue;
      foreach (int ie in new[] { 0, row - 1 })
      {
        double bwe = w0A[ie], fwe = wA[ie] - w0A[ie];
        if (morph) { bwe += (w0B[ie] - w0A[ie]) * t; fwe += ((wB[ie] - w0B[ie]) - fwe) * t; }
        double we = (bwe + dm * fwe) * wm;
        wEdgeLo = Math.Min(wEdgeLo, we); wEdgeHi = Math.Max(wEdgeHi, we);
      }
      for (int i = 0; i < row; i++)
      {
        double bw = w0A[i], bd = d0A[i];                    // base line
        double fw = wA[i] - w0A[i], fd = dA[i] - d0A[i];    // fold, measured from the base line
        if (morph)
        {
          bw += (w0B[i] - w0A[i]) * t; bd += (d0B[i] - d0A[i]) * t;
          fw += ((wB[i] - w0B[i]) - fw) * t; fd += ((dB[i] - d0B[i]) - fd) * t;
        }
        if (style == 1) { double keep = 1.0 - 2.0 * amount * rev; fd = fdMid + (fd - fdMid) * keep; }   // counter-running
        double w = (bw + dm * fw) * wm;
        double d = bd + dm * fd;
        if (style != 0) d += lift * rev * outSign * side;                                      // apex lift: positive is always up
        if (style == 2)                                                                        // zigzag
        {
          bool outerEdge = !closed && (i == 0 || i == row - 1);
          if (!outerEdge) w = Math.Max(wEdgeLo, Math.Min(wEdgeHi, w + amount * avgHalf * rev));   // edges stay on the paths
        }
        if (k == 0 && i == 0) { w00 = w; d00 = d; }
        dd2[k, i] = d;
        dLo = Math.Min(dLo, d); dHi = Math.Max(dHi, d);
        grid[k, i] = (closed && i == nx) ? grid[k, 0] : Pos(k, w, d);
        mesh.Vertices.Add(grid[k, i]);
      }
    }

    // ---------------- plates ----------------
    int quads = 0, tris = 0;
    var faceTags = new List<int>();   // station * 10000 + fold, one per plate, for fabrication labels
    for (int k = 0; k < np - 1; k++)
      for (int i = 0; i < nx; i++)
      {
        int i1 = (closed && i + 1 == nx) ? 0 : i + 1;
        int a = k * row + i, b = k * row + i1, c = (k + 1) * row + i1, d = (k + 1) * row + i;
        Point3d A = grid[k, i], B = grid[k, i1], C = grid[k + 1, i1], D = grid[k + 1, i];
        bool ab = A.DistanceTo(B) < 1e-6, dc = D.DistanceTo(C) < 1e-6;
        if (ab && dc) continue;
        if (ab) { mesh.Faces.AddFace(a, c, d); faceTags.Add(k * 10000 + i); tris++; continue; }
        if (dc) { mesh.Faces.AddFace(a, b, c); faceTags.Add(k * 10000 + i); tris++; continue; }
        Vector3d fn = Vector3d.CrossProduct(B - A, C - A);
        double size = A.DistanceTo(C);
        bool flat = fn.Unitize() && Math.Abs(fn * (D - A)) < 1e-6 * Math.Max(1.0, size);
        if (flat) { mesh.Faces.AddFace(a, b, c, d); faceTags.Add(k * 10000 + i); quads++; }
        else
        {
          // Which diagonal to fold along. Prefer the one that leaves the plate
          // flattest; if both bend it about equally (a true reverse fold), fold
          // along the diagonal joining the two higher corners.
          Vector3d p1 = Vector3d.CrossProduct(B - A, C - A), p2 = Vector3d.CrossProduct(C - A, D - A);
          Vector3d p3 = Vector3d.CrossProduct(B - A, D - A), p4 = Vector3d.CrossProduct(C - B, D - B);
          double bendAC = Vector3d.VectorAngle(p1, p2), bendBD = Vector3d.VectorAngle(p3, p4);
          if (double.IsNaN(bendAC)) bendAC = Math.PI;
          if (double.IsNaN(bendBD)) bendBD = Math.PI;
          bool useAC;
          if (Math.Abs(bendAC - bendBD) > 0.1 * Math.Max(bendAC, bendBD)) useAC = bendAC < bendBD;
          else useAC = dd2[k, i] + dd2[k + 1, i + 1] >= dd2[k, i + 1] + dd2[k + 1, i];
          if (useAC)
          { mesh.Faces.AddFace(a, b, c); mesh.Faces.AddFace(a, c, d); }
          else
          { mesh.Faces.AddFace(a, b, d); mesh.Faces.AddFace(b, c, d); }
          faceTags.Add(k * 10000 + i); faceTags.Add(k * 10000 + i);
          tris += 2;
        }
      }
    if (mesh.Faces.Count == 0) { Report = "No plates could be built from these inputs."; return; }
    // face normals should point the way the ridges go
    {
      Vector3d want = Pos(0, w00, d00 + 1.0) - grid[0, 0];
      for (int f = 0; f < mesh.Faces.Count; f++)
      {
        MeshFace mf = mesh.Faces[f];
        Vector3d n0 = Vector3d.CrossProduct(
          mesh.Vertices.Point3dAt(mf.B) - mesh.Vertices.Point3dAt(mf.A),
          mesh.Vertices.Point3dAt(mf.C) - mesh.Vertices.Point3dAt(mf.A));
        if (n0.Length < 1e-9) continue;
        if (n0 * want < 0) mesh.Flip(true, true, true);
        break;
      }
    }
    mesh.Normals.ComputeNormals();

    var ridges = new List<Line>();
    var valleys = new List<Line>();
    Creases(mesh, ridges, valleys);

    var cols = new List<Point3d>();
    var ground = new HashSet<int>();
    for (int i = 0; i < row; i++)
    {
      cols.Add(grid[0, i]);
      cols.Add(grid[np - 1, i]);
      if (groundStart) ground.Add(i);
      if (groundEnd) ground.Add((np - 1) * row + i);
    }

    double tFt = thk / 12.0;
    Mesh solid = Thicken(mesh, tFt, ground);
    double area = AreaOf(mesh);
    BoundingBox bb = mesh.GetBoundingBox(true);

    // widest plate (longest cross edge)
    double sMax = 0.0;
    int sMaxK = 0, sMaxI = 0;
    for (int k = 0; k < np; k++)
      for (int i = 0; i < nx; i++)
      {
        double wEdge = mesh.Vertices.Point3dAt(k * row + i).DistanceTo(mesh.Vertices.Point3dAt(k * row + i + 1));
        if (wEdge > sMax) { sMax = wEdge; sMaxK = k; sMaxI = i; }
      }

    // ---------------- dimensions for the viewport ----------------
    // Published for the control window, which draws them over the model.
    try
    {
      var dl = new List<Line>(); var dp = new List<Point3d>(); var dt = new List<string>();
      double diag = bb.Diagonal.Length;
      double pad = 0.04 * diag, tick = 0.012 * diag;
      double x0 = bb.Min.X, x1 = bb.Max.X, y0 = bb.Min.Y, y1 = bb.Max.Y, z0 = bb.Min.Z, z1 = bb.Max.Z;
      // overall size: along X, along Y, and height
      Point3d ax0 = new Point3d(x0, y0 - pad, z0), ax1 = new Point3d(x1, y0 - pad, z0);
      dl.Add(new Line(ax0, ax1));
      dl.Add(new Line(ax0 + new Vector3d(0, -tick, 0), ax0 + new Vector3d(0, tick, 0)));
      dl.Add(new Line(ax1 + new Vector3d(0, -tick, 0), ax1 + new Vector3d(0, tick, 0)));
      dp.Add(new Point3d((x0 + x1) / 2.0, y0 - pad, z0)); dt.Add(string.Format("{0:0.0} ft", x1 - x0));
      Point3d ay0 = new Point3d(x1 + pad, y0, z0), ay1 = new Point3d(x1 + pad, y1, z0);
      dl.Add(new Line(ay0, ay1));
      dl.Add(new Line(ay0 + new Vector3d(-tick, 0, 0), ay0 + new Vector3d(tick, 0, 0)));
      dl.Add(new Line(ay1 + new Vector3d(-tick, 0, 0), ay1 + new Vector3d(tick, 0, 0)));
      dp.Add(new Point3d(x1 + pad, (y0 + y1) / 2.0, z0)); dt.Add(string.Format("{0:0.0} ft", y1 - y0));
      Point3d az0 = new Point3d(x1 + pad, y0 - pad, z0), az1 = new Point3d(x1 + pad, y0 - pad, z1);
      dl.Add(new Line(az0, az1));
      dl.Add(new Line(az0 + new Vector3d(-tick, 0, 0), az0 + new Vector3d(tick, 0, 0)));
      dl.Add(new Line(az1 + new Vector3d(-tick, 0, 0), az1 + new Vector3d(tick, 0, 0)));
      dp.Add(new Point3d(x1 + pad, y0 - pad, (z0 + z1) / 2.0)); dt.Add(string.Format("{0:0.0} ft high", z1 - z0));
      // length of each run of the drawn paths
      PathDims(crv != null ? crv : new PolylineCurve(new Polyline(raw)), dp, dt);
      if (twoPaths) PathDims(crv2, dp, dt);
      // straight-line distance between the two ends
      Point3d e0 = twoPaths ? stM[0] : pts[0], e1 = twoPaths ? stM[np - 1] : pts[np - 1];
      if (e0.DistanceTo(e1) > 0.5 && np > 2)
      {
        dl.Add(new Line(e0, e1));
        dp.Add(e0 + (e1 - e0) * 0.5); dt.Add(string.Format("end to end {0:0.0} ft", e0.DistanceTo(e1)));
      }
      // the widest plate
      Point3d wp0 = grid[sMaxK, sMaxI], wp1 = grid[sMaxK, sMaxI + 1];
      dl.Add(new Line(wp0, wp1));
      dp.Add(wp0 + (wp1 - wp0) * 0.5); dt.Add(string.Format("widest plate {0:0.0} ft", sMax));
      AppDomain.CurrentDomain.SetData("FoldToolkit.Dims", new object[] { dl.ToArray(), dp.ToArray(), dt.ToArray() });
    }
    catch { }
    try
    {
      // creases on the upper face, for the control window to draw in colour
      var rTop = new List<Line>(); var vTop = new List<Line>();
      FoldsOnTop(mesh, tFt / 2.0, rTop, vTop);
      AppDomain.CurrentDomain.SetData("FoldToolkit.Folds", new object[] { rTop.ToArray(), vTop.ToArray() });
    }
    catch { }

    string[] styleName = { "parallel", "counter-running", "zigzag" };
    var sb = new StringBuilder();
    sb.AppendLine("FOLD DESIGNER");
    sb.AppendLine(string.Format("Section: {0}, {1} fold lines | style: {2}{3} | slab {4:0.##} in",
      drawn ? ((smoothA ? "folds generated along the drawn curve" : "drawn polyline") + (morph ? ", morphing to ProfileEnd" : "")) : (vary > 0 ? "generated, varied (seed " + seed + ")" : "generated, regular"),
      row, styleName[style], style == 0 ? "" : string.Format(" ({0:0.00}, reversing every {1} station{2}{3})", amount, period, period == 1 ? "" : "s", Math.Abs(lift) > 1e-9 ? string.Format(", apex lift {0:0.##} ft", lift) : ""), thk));
    sb.AppendLine("Layout: " + (twoPaths ? "folds run between Path and Path2"
      : fanOn ? string.Format("folds fan {0:0.#} degrees about a vertical axis {1:0.0} ft behind the start of the path ({2} inner radius){3}",
          fanDeg, fanR, fanRAuto ? "automatic" : "your", closed ? ", closed ring" : "")
      : "folds run parallel, across the path"));
    if (momentOn) sb.AppendLine("Fold depth follows bending: " + momentInfo + ".");
    sb.AppendLine();
    sb.AppendLine("INPUTS RECEIVED");
    sb.AppendLine("  Path        " + Received(Path));
    sb.AppendLine("  Path2       " + Received(Path2));
    sb.AppendLine("  Profile     " + Received(Profile));
    sb.AppendLine("  ProfileEnd  " + Received(ProfileEnd));
    sb.AppendLine("  FanRadius   " + (Num(FanRadius, 0.0) > 0.0 ? string.Format("{0:0.##} ft", Num(FanRadius, 0.0)) : "not set (automatic)"));
    sb.AppendLine(string.Format("  Fan {0:0.#} deg | MomentDepth {1} | DepthAlong {2} value(s) | WidthAlong {3} value(s)",
      fanDeg, momentOn ? "on" : "off", depthAlong.Count, widthAlong.Count));
    sb.AppendLine();
    sb.AppendLine("GEOMETRY");
    sb.AppendLine(string.Format("  Path length                {0:0.0} ft in {1} stations", pathLen, np));
    sb.AppendLine(string.Format("  Overall size               {0:0.0} x {1:0.0} x {2:0.0} ft", bb.Max.X - bb.Min.X, bb.Max.Y - bb.Min.Y, bb.Max.Z - bb.Min.Z));
    sb.AppendLine(string.Format("  Plates                     {0} planar quads, {1} planar triangles", quads, tris));
    sb.AppendLine(string.Format("  Ridge / valley creases     {0} / {1}", ridges.Count, valleys.Count));
    sb.AppendLine(string.Format("  Section height (overall)   {0:0.00} ft  (rise of the drawn section plus its folds)", dHi - dLo));
    sb.AppendLine(string.Format("  Widest plate               {0:0.00} ft  (width / thickness {1:0})", sMax, sMax * 12.0 / thk));
    if (np > 2) sb.AppendLine(string.Format("  Sharpest corner in path    {0:0} deg", sharpest));
    double big = Math.Max(bb.Max.X - bb.Min.X, Math.Max(bb.Max.Y - bb.Min.Y, bb.Max.Z - bb.Min.Z));
    sb.AppendLine(string.Format("  Slab thickness             {0:0.##} in = {1:0.000} ft  (1/{2:0} of the overall size)", thk, tFt, big / tFt));
    sb.AppendLine(string.Format("  Formwork area              {0:0} ft2", area));
    sb.AppendLine(string.Format("  Concrete volume            {0:0.0} ft3 ({1:0.0} yd3)", area * tFt, area * tFt / 27.0));
    sb.AppendLine(string.Format("  Self-weight                {0:0} lb total", area * tFt * 150.0));
    sb.AppendLine();
    sb.AppendLine("FLAGS");
    { string un = UnitNote(); if (un != null) sb.AppendLine("  - " + un); }
    foreach (string note in notes) sb.AppendLine("  - " + note);
    if (drawn && !smoothA) sb.AppendLine("  - Profile is a polyline, so it is used exactly as drawn; Bays, BayWidth, Depth and Variation are ignored.");
    if (drawn && smoothA) sb.AppendLine("  - Profile is a smooth curve: Bays and Depth set the folds along it; BayWidth is ignored.");
    if (depthAlong.Count == 1 && Math.Abs(depthAlong[0] - 1.0) > 1e-9) sb.AppendLine(string.Format("  - DepthAlong has a single value, so all fold depths are scaled by {0:0.##}.", depthAlong[0]));
    if (widthAlong.Count == 1 && Math.Abs(widthAlong[0] - 1.0) > 1e-9) sb.AppendLine(string.Format("  - WidthAlong has a single value, so the whole width is scaled by {0:0.##}.", widthAlong[0]));
    if (fanOn) sb.AppendLine("  - Fan: the width of the section is spread over the fan angle, so BayWidth has no effect. The path starts at the inner edge of the fan, not at its centre.");
    if (fanOn && !fanRAuto && fanR < fanRNeeded) sb.AppendLine(string.Format("  - FanRadius is smaller than the section needs ({0:0.0} ft): fold lines reach the axis and plates cross at the centre.", fanRNeeded));
    if (fanOn && style == 2 && amount > 0.3) sb.AppendLine("  - Zigzag in a fan shifts every other station by an angle, which is a large sideways step at the rim. Lower Amount for a calmer surface.");
    if (fanOn) sb.AppendLine(string.Format("  - Plates at the inner edge are about {0:0.00} ft wide.", fanR * Math.Abs(fanRad) / Math.Max(1, nx)));
    if (twoPaths && cornerMatched) sb.AppendLine("  - Two paths: both are polylines with the same number of points, so a station sits on every corner and corners stay sharp. Stations each subdivides the runs; Path pieces is not used.");
    if (twoPaths && !cornerMatched) sb.AppendLine("  - Two paths: stations are spaced evenly along both curves, so sharp corners are rounded off. Raise Path pieces to follow the curves more closely.");
    if (twoPaths && !groundStart && !groundEnd) sb.AppendLine("  - Neither end of the paths rises steeply, so both ends are cut square as free edges.");
    if (momentOn) sb.AppendLine("  - Fold depth runs from 25% where bending is least to 100% where it peaks. This shapes the depth; it is not a stress check.");
    if (style != 0 && period > 1 && np - 1 < 2 * period) sb.AppendLine(string.Format("  - The path has {0} stations, fewer than one full reversal ({1}): lower Period or add stations.", np, 2 * period + 1));
    if (style != 0 && np < 3) sb.AppendLine("  - Style needs at least 3 stations along the path: raise Divisions.");
    if ((depthAlong.Count > 1 || widthAlong.Count > 1 || morph) && np < 3) sb.AppendLine("  - Changes along the path need more stations: raise Divisions.");
    if (big / tFt > 300.0) sb.AppendLine("  - The slab is very thin next to the size of the model; zoom in on a free edge to see it.");
    if (groundStart || groundEnd) sb.AppendLine("  - Ends that rise from the ground are cut level so every fold line bears at the same height.");
    if (sMax * 12.0 / thk > 30.0) sb.AppendLine("  - Slender plates: width/thickness above 30, check plate buckling.");
    if (squeezed) sb.AppendLine("  - The section is taller than a bend in the path can take: plates cross on the inside of the bend. Flatten the section, soften the bend, or lengthen the runs beside it.");
    else if (tight) sb.AppendLine("  - The section is tall for a bend in the path: little straight run is left between bends.");
    if (np > 2 && sharpest < 60.0) sb.AppendLine("  - Very sharp corner in the path: fold lines may cross on the inside of the bend.");
    if (tris > 0) sb.AppendLine("  - Where plates are triangles, slab thickness at the fold points is a best fit.");
    sb.AppendLine("  - At folds sharper than about 30 degrees the slab edge is blunted instead of running to a point.");
    sb.AppendLine("  - Geometry only, not stress-checked. Compare with the nearest precedent.");

    // share the report and the solid with the control window, if it is open
    try
    {
      AppDomain.CurrentDomain.SetData("FoldToolkit.Report", sb.ToString());
      AppDomain.CurrentDomain.SetData("FoldToolkit.Solid", solid);
      AppDomain.CurrentDomain.SetData("FoldToolkit.SlabFt", tFt);
      AppDomain.CurrentDomain.SetData("FoldToolkit.Row", row);   // points per station, so a vertex number gives station and fold line
      // for the fabrication components: which station and fold each plate belongs to,
      // and the direction that runs across the folds
      AppDomain.CurrentDomain.SetData("FoldToolkit.FaceTags", faceTags.ToArray());
      Vector3d acrossDir = twoPaths ? (stB[np / 2] - stA[np / 2]) : pln.ZAxis;
      acrossDir.Unitize();
      AppDomain.CurrentDomain.SetData("FoldToolkit.Across", acrossDir);
    }
    catch { }

    Plates = mesh;
    Solid = solid;
    Ridges = ridges;
    Valleys = valleys;
    Columns = cols;
    Report = sb.ToString();
  }
}
