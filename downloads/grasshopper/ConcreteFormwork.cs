// Folded Plate Toolkit - Concrete Formwork and Falsework
// Rhino 8 Grasshopper C# Script component. Model units: FEET.
//
// Takes the folded surface and produces what is needed to cast it in place:
// a labelled formwork layout, the props under the soffit, and a falsework
// plan and section.
//
// Inputs:
//   Plates      : the Plates output of the Fold Designer (or of a precedent)
//   PropSpacing : ft, grid spacing of the props (default 4)
//   SheetWidth  : ft, form sheet width (default 4)
//   SheetLength : ft, form sheet length (default 8)
//   LayoutGap   : ft, clear distance from the model to the flat layout (default 20)
//   SectionAt   : 0 to 1, where the section is cut across the roof (default 0.5)
//   LabelScale  : multiplier on the label text size (default 1; 0 hides the labels)
// Outputs:
//   Layout   : every form panel laid flat, as closed outlines
//   Props    : vertical props from the ground to the soffit
//   Plan     : falsework plan on the ground: roof outline, fold lines, prop positions
//   Section  : the roof cut by a vertical plane, with the props in that plane
//   Report   : counts, areas, sheets, prop lengths, repetition
//
// Labels are drawn in the viewport, on the model and on the layout:
// S03-F12 is station 3, fold 12; a and b are the two triangles of one cell.
// Form panels are the soffit faces taken at the mid-surface of the slab.
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

  // labels, kept for the viewport. They are drawn as text lying on each panel,
  // sized to fit it. When drawn, each one is turned to read the right way round
  // from wherever the camera is.
  private List<Point3d> tagCentres = new List<Point3d>();
  private List<Vector3d> tagX = new List<Vector3d>();
  private List<Vector3d> tagY = new List<Vector3d>();
  private List<double> tagHeights = new List<double>();
  private List<string> tagTexts = new List<string>();
  private BoundingBox tagBox = BoundingBox.Empty;
  private double labelScale = 1.0;
  private double labelLift = 0.5;   // ft the text is raised off the plate's mid-surface, toward the viewer

  public override BoundingBox ClippingBox { get { return tagBox; } }

  public override void DrawViewportWires(IGH_PreviewArgs args)
  {
    try
    {
      if (labelScale <= 0) return;
      System.Drawing.Color ink = System.Drawing.Color.FromArgb(20, 20, 20);
      bool parallel = args.Viewport.IsParallelProjection;
      Vector3d camDir = args.Viewport.CameraDirection;
      Vector3d camUp = args.Viewport.CameraUp;
      Point3d camAt = args.Viewport.CameraLocation;
      for (int i = 0; i < tagCentres.Count; i++)
      {
        Point3d c = tagCentres[i];
        Vector3d x = tagX[i], y = tagY[i];
        Vector3d look = parallel ? camDir : c - camAt;
        Vector3d n = Vector3d.CrossProduct(x, y);
        if (n * look > 0) x = -x;                    // seen from behind: mirror it back
        if (y * camUp < 0) { x = -x; y = -y; }       // upside down: turn it round
        // The slab has thickness and the label sits on its mid-surface, so lift the
        // text clear of whichever face is toward the camera. Otherwise the slab hides it.
        Vector3d toward = Vector3d.CrossProduct(x, y);
        if (toward.Unitize()) c = c + toward * labelLift;
        double ht = tagHeights[i] * labelScale;
        double wt = 0.62 * ht * tagTexts[i].Length;
        Point3d o = c - x * (wt / 2.0) - y * (ht / 2.0);
        args.Display.Draw3dText(tagTexts[i], ink, new Plane(o, x, y), ht, "Arial");
      }
    }
    catch { }
  }

  // label centred at c: x = along the panel's long edge, y = across it, ht = text height
  private void AddTag(string text, Point3d c, Vector3d x, Vector3d y, double ht)
  {
    tagCentres.Add(c); tagX.Add(x); tagY.Add(y);
    tagHeights.Add(ht);
    tagTexts.Add(text);
    double reach = 0.5 * 0.62 * ht * text.Length * Math.Max(1.0, labelScale) + ht;
    tagBox.Union(c + new Vector3d(reach, reach, reach)); tagBox.Union(c - new Vector3d(reach, reach, reach));
  }

  // the control window can switch this tool off; without the window it is always on
  private static bool ToolOn(string key)
  {
    try
    {
      Dictionary<string, double> tools = AppDomain.CurrentDomain.GetData("FoldToolkit.Tools") as Dictionary<string, double>;
      double v;
      if (tools != null && tools.TryGetValue(key, out v)) return v > 0.5;
    }
    catch { }
    return true;
  }

  private static Mesh AsMesh(object o)
  {
    if (o is Mesh) return (Mesh)o;
    if (o is GH_Mesh) return ((GH_Mesh)o).Value;
    return null;
  }

  private void RunScript(
    object Plates, object PropSpacing, object SheetWidth, object SheetLength, object LayoutGap, object SectionAt,
    object LabelScale,
    ref object Layout, ref object Props, ref object Plan, ref object Section, ref object Report)
  {
    tagCentres = new List<Point3d>(); tagX = new List<Vector3d>(); tagY = new List<Vector3d>();
    tagHeights = new List<double>(); tagTexts = new List<string>(); tagBox = BoundingBox.Empty;
    labelScale = Num(LabelScale, 1.0); if (labelScale < 0) labelScale = 0.0;
    // half the slab thickness plus a little, if the Fold Designer has published it
    labelLift = 0.5;
    try
    {
      object st = AppDomain.CurrentDomain.GetData("FoldToolkit.SlabFt");
      if (st is double && (double)st > 0) labelLift = (double)st / 2.0 + 0.08;
    }
    catch { }
    if (!ToolOn("FormOn")) { Report = "FORMWORK is switched off in the control window (section 7)."; return; }
    Mesh mesh = AsMesh(Plates);
    if (mesh == null || mesh.Faces.Count == 0) { Report = "Connect the Plates output of the Fold Designer to Plates."; return; }
    double spacing = Num(PropSpacing, 4.0); if (spacing < 0.5) spacing = 4.0;
    double sheetW = Num(SheetWidth, 4.0); if (sheetW <= 0) sheetW = 4.0;
    double sheetL = Num(SheetLength, 8.0); if (sheetL <= 0) sheetL = 8.0;
    double gap = Num(LayoutGap, 20.0); if (gap < 0) gap = 20.0;
    double secAt = Math.Max(0.0, Math.Min(1.0, Num(SectionAt, 0.5)));

    BoundingBox bb = mesh.GetBoundingBox(true);
    double ground = bb.Min.Z;

    // ---------------- plates and labels ----------------
    int[] tags = null;
    try { tags = AppDomain.CurrentDomain.GetData("FoldToolkit.FaceTags") as int[]; } catch { }
    bool tagged = tags != null && tags.Length == mesh.Faces.Count;
    int nF = mesh.Faces.Count;
    var label = new string[nF];
    var corners = new List<Point3d>[nF];
    var seen = new Dictionary<int, int>();
    for (int f = 0; f < nF; f++)
    {
      MeshFace mf = mesh.Faces[f];
      var c = new List<Point3d>();
      c.Add(mesh.Vertices.Point3dAt(mf.A)); c.Add(mesh.Vertices.Point3dAt(mf.B)); c.Add(mesh.Vertices.Point3dAt(mf.C));
      if (mf.IsQuad) c.Add(mesh.Vertices.Point3dAt(mf.D));
      corners[f] = c;
      if (tagged)
      {
        int t = tags[f];
        string baseName = string.Format("S{0:00}-F{1:00}", t / 10000 + 1, t % 10000 + 1);
        if (mf.IsQuad) label[f] = baseName;
        else
        {
          int n; seen.TryGetValue(t, out n); seen[t] = n + 1;
          label[f] = baseName + (n == 0 ? "a" : "b");
        }
      }
      else label[f] = string.Format("P{0:000}", f + 1);
    }

    // ---------------- flat layout of the form panels ----------------
    var layout = new List<Polyline>();
    var shapeCount = new Dictionary<string, int>();
    double totalArea = 0.0, steepArea = 0.0;
    int steep = 0, oversize = 0;
    double rowWidth = Math.Max(40.0, 1.6 * Math.Sqrt(Math.Max(1.0, AreaOf(mesh))));
    double curX = 0.0, curY = 0.0, rowH = 0.0, pad = 1.5;
    Point3d origin = new Point3d(bb.Min.X, bb.Max.Y + gap, ground);
    for (int f = 0; f < nF; f++)
    {
      List<Point3d> c = corners[f];
      int m = c.Count;
      // longest edge becomes the bottom of the flat panel
      int e0 = 0; double best = -1.0;
      for (int k = 0; k < m; k++) { double len = c[k].DistanceTo(c[(k + 1) % m]); if (len > best) { best = len; e0 = k; } }
      Vector3d xAxis = c[(e0 + 1) % m] - c[e0];
      if (!xAxis.Unitize()) continue;
      Vector3d nrm = Vector3d.CrossProduct(c[1] - c[0], c[2] - c[0]);
      if (!nrm.Unitize()) continue;
      Vector3d yAxis = Vector3d.CrossProduct(nrm, xAxis);
      var uv = new List<Point2d>();
      double uMin = double.MaxValue, uMax = double.MinValue, vMin = double.MaxValue, vMax = double.MinValue;
      for (int k = 0; k < m; k++)
      {
        Vector3d d = c[k] - c[e0];
        double u = d * xAxis, v = d * yAxis;
        uv.Add(new Point2d(u, v));
        uMin = Math.Min(uMin, u); uMax = Math.Max(uMax, u); vMin = Math.Min(vMin, v); vMax = Math.Max(vMax, v);
      }
      double w = uMax - uMin, h = vMax - vMin;
      if (curX > 0 && curX + w > rowWidth) { curX = 0.0; curY += rowH + pad; rowH = 0.0; }
      var pl = new Polyline();
      for (int k = 0; k <= m; k++)
      {
        Point2d q = uv[k % m];
        pl.Add(origin.X + curX + (q.X - uMin), origin.Y + curY + (q.Y - vMin), ground);
      }
      layout.Add(pl);
      // Where the label goes and how tall it can be. A triangle's label sits at its
      // centroid and is sized to stay inside the triangle, so the two triangles of
      // one cell do not write over each other along their shared edge.
      double cu, cv, lblH;
      int nch = Math.Max(1, label[f].Length);
      if (m == 3)
      {
        Point2d apex = uv[(e0 + 2) % 3];             // the corner opposite the longest edge
        double triH = Math.Abs(apex.Y);
        cu = (best + apex.X) / 3.0; cv = apex.Y / 3.0;
        double th = Math.Max(1e-6, triH);
        double skew = Math.Abs(apex.X - best / 2.0) / th;   // how far the apex leans to one end
        lblH = 0.85 * Math.Min((2.0 * best / 3.0) / (0.62 * nch + best / (2.0 * th) + skew), 2.0 * triH / 3.0);
      }
      else
      {
        cu = (uMin + uMax) / 2.0; cv = (vMin + vMax) / 2.0;
        lblH = Math.Min(0.45 * h, w / (0.7 * nch));
      }
      lblH = Math.Max(0.08, Math.Min(2.0, lblH));
      // labels: on the flat panel and on the model
      Point3d flatC = new Point3d(origin.X + curX + (cu - uMin), origin.Y + curY + (cv - vMin), ground);
      AddTag(label[f], flatC + new Vector3d(0, 0, 0.02), Vector3d.XAxis, Vector3d.YAxis, lblH);
      if (nF <= 600)
      {
        // on the model: lying on the plate, readable from above or outside
        Vector3d up = nrm, ty = yAxis;
        if (up.Z < -1e-9) { up = -up; ty = -ty; }
        Point3d onPlate = c[e0] + xAxis * cu + yAxis * cv;
        AddTag(label[f], onPlate, xAxis, ty, lblH);
      }
      curX += w + pad; rowH = Math.Max(rowH, h);

      // area, steepness, sheet fit, shape signature
      double area = 0.0;
      for (int k = 1; k < m - 1; k++) area += 0.5 * Vector3d.CrossProduct(c[k] - c[0], c[k + 1] - c[0]).Length;
      totalArea += area;
      double tilt = Math.Acos(Math.Min(1.0, Math.Abs(nrm.Z))) * 180.0 / Math.PI;
      if (tilt > 45.0) { steep++; steepArea += area; }
      double shortSide = Math.Min(w, h), longSide = Math.Max(w, h);
      if (shortSide > Math.Min(sheetW, sheetL) + 1e-6 || longSide > Math.Max(sheetW, sheetL) + 1e-6) oversize++;
      var lens = new List<double>();
      for (int k = 0; k < m; k++) lens.Add(Math.Round(c[k].DistanceTo(c[(k + 1) % m]), 2));
      lens.Sort();
      var sig = new StringBuilder();
      foreach (double l in lens) sig.Append(l.ToString("0.00")).Append("|");
      sig.Append(Math.Round(area, 1).ToString("0.0"));
      string key = sig.ToString();
      int cnt; shapeCount.TryGetValue(key, out cnt); shapeCount[key] = cnt + 1;
    }

    // ---------------- props under the soffit ----------------
    var props = new List<Line>();
    double pMin = double.MaxValue, pMax = 0.0, pTotal = 0.0;
    int tall = 0;
    int nxp = (int)Math.Floor((bb.Max.X - bb.Min.X) / spacing), nyp = (int)Math.Floor((bb.Max.Y - bb.Min.Y) / spacing);
    double offX = ((bb.Max.X - bb.Min.X) - nxp * spacing) / 2.0, offY = ((bb.Max.Y - bb.Min.Y) - nyp * spacing) / 2.0;
    if ((long)(nxp + 1) * (nyp + 1) > 20000)
    {
      Report = "PropSpacing is too small for this roof (more than 20,000 props). Increase it.";
      return;
    }
    for (int ix = 0; ix <= nxp; ix++)
      for (int iy = 0; iy <= nyp; iy++)
      {
        Point3d foot = new Point3d(bb.Min.X + offX + ix * spacing, bb.Min.Y + offY + iy * spacing, ground);
        Ray3d ray = new Ray3d(foot + new Vector3d(0, 0, -1.0), Vector3d.ZAxis);
        double tHit = Rhino.Geometry.Intersect.Intersection.MeshRay(mesh, ray);
        if (tHit < 0) continue;
        double height = tHit - 1.0;
        if (height < 1.0) continue;                      // too low to need a prop
        props.Add(new Line(foot, new Point3d(foot.X, foot.Y, ground + height)));
        pMin = Math.Min(pMin, height); pMax = Math.Max(pMax, height); pTotal += height;
        if (height > 20.0) tall++;
      }

    // ---------------- falsework plan (on the ground) ----------------
    var plan = new List<Curve>();
    Transform flatten = Transform.PlanarProjection(new Plane(new Point3d(0, 0, ground), Vector3d.ZAxis));
    Polyline[] naked = mesh.GetNakedEdges();
    if (naked != null)
      foreach (Polyline np2 in naked)
      {
        Polyline cp = np2.Duplicate(); cp.Transform(flatten);
        plan.Add(new PolylineCurve(cp));
      }
    var ridges = new List<Line>(); var valleys = new List<Line>();
    Creases(mesh, ridges, valleys);
    foreach (Line ln in ridges) { Line l2 = ln; l2.Transform(flatten); plan.Add(new LineCurve(l2)); }
    foreach (Line ln in valleys) { Line l2 = ln; l2.Transform(flatten); plan.Add(new LineCurve(l2)); }
    double cross = Math.Min(0.6, spacing / 5.0);
    foreach (Line pr in props)
    {
      Point3d p = pr.From;
      plan.Add(new LineCurve(new Point3d(p.X - cross, p.Y, p.Z), new Point3d(p.X + cross, p.Y, p.Z)));
      plan.Add(new LineCurve(new Point3d(p.X, p.Y - cross, p.Z), new Point3d(p.X, p.Y + cross, p.Z)));
    }

    // ---------------- section ----------------
    // Cut along the direction the folds travel, at a chosen position across the roof.
    Vector3d across = Vector3d.XAxis;
    try
    {
      object av = AppDomain.CurrentDomain.GetData("FoldToolkit.Across");
      if (tagged && av is Vector3d) across = (Vector3d)av;
    }
    catch { }
    across.Z = 0.0;
    if (!across.Unitize()) across = Vector3d.XAxis;
    double aMin = double.MaxValue, aMax = double.MinValue;
    foreach (Point3d corner in bb.GetCorners())
    {
      double a = new Vector3d(corner.X, corner.Y, 0.0) * across;
      aMin = Math.Min(aMin, a); aMax = Math.Max(aMax, a);
    }
    double aCut = aMin + (aMax - aMin) * secAt;
    Plane cut = new Plane(new Point3d(across.X * aCut, across.Y * aCut, ground), across);
    var section = new List<Curve>();
    Polyline[] cuts = Rhino.Geometry.Intersect.Intersection.MeshPlane(mesh, cut);
    if (cuts != null) foreach (Polyline cp in cuts) section.Add(new PolylineCurve(cp));
    int inSection = 0;
    foreach (Line pr in props)
    {
      double a = new Vector3d(pr.From.X, pr.From.Y, 0.0) * across;
      if (Math.Abs(a - aCut) <= spacing / 2.0 + 1e-6)
      {
        // show the prop in the cut plane, at the height of the soffit there
        Vector3d shift = across * (aCut - a);
        Point3d foot = pr.From + shift;
        Ray3d ray = new Ray3d(foot + new Vector3d(0, 0, -1.0), Vector3d.ZAxis);
        double tHit = Rhino.Geometry.Intersect.Intersection.MeshRay(mesh, ray);
        if (tHit < 0) continue;
        section.Add(new LineCurve(foot, new Point3d(foot.X, foot.Y, ground + tHit - 1.0)));
        inSection++;
      }
    }
    // ground line in the section
    Vector3d alongDir = Vector3d.CrossProduct(Vector3d.ZAxis, across);
    double half = 0.6 * bb.Diagonal.Length;
    Point3d gc = cut.ClosestPoint(bb.Center); gc.Z = ground;
    section.Add(new LineCurve(gc - alongDir * half, gc + alongDir * half));


    // ---------------- report ----------------
    double sheetArea = sheetW * sheetL;
    int sheets = (int)Math.Ceiling(totalArea * 1.15 / sheetArea);
    var sb = new StringBuilder();
    sb.AppendLine("CONCRETE FORMWORK AND FALSEWORK");
    sb.AppendLine(string.Format("{0} form panels | labels: {1}", layout.Count, tagged ? "station and fold (S03-F12)" : "numbered in order (no station data from this source)"));
    sb.AppendLine();
    sb.AppendLine("FORMWORK");
    sb.AppendLine(string.Format("  Soffit form area           {0:0} ft2", totalArea));
    sb.AppendLine(string.Format("  Form sheets, {0:0.#} x {1:0.#} ft    about {2}  (area plus 15% waste)", sheetW, sheetL, sheets));
    sb.AppendLine(string.Format("  Panels larger than a sheet {0}  (these need a joint in the form)", oversize));
    sb.AppendLine(string.Format("  Panels steeper than 45 deg {0}  ({1:0} ft2 needing a top form as well)", steep, steepArea));
    sb.AppendLine(string.Format("  Different panel shapes     {0} of {1}", shapeCount.Count, layout.Count));
    sb.AppendLine(string.Format("  Repetition                 each shape is used {0:0.0} times on average", layout.Count / (double)Math.Max(1, shapeCount.Count)));
    sb.AppendLine();
    sb.AppendLine("FALSEWORK");
    sb.AppendLine(string.Format("  Prop grid                  {0:0.##} ft each way", spacing));
    sb.AppendLine(string.Format("  Props                      {0}", props.Count));
    if (props.Count > 0)
    {
      sb.AppendLine(string.Format("  Shortest / longest prop    {0:0.0} / {1:0.0} ft", pMin, pMax));
      sb.AppendLine(string.Format("  Total prop length          {0:0} ft", pTotal));
      sb.AppendLine(string.Format("  Props shown in the section {0}", inSection));
    }
    sb.AppendLine();
    sb.AppendLine("FLAGS");
    { string un = UnitNote(); if (un != null) sb.AppendLine("  - " + un); }
    if (nF > 600) sb.AppendLine("  - More than 600 panels: labels are shown on the flat layout only, to keep the model readable.");
    if (labelScale <= 0) sb.AppendLine("  - Labels are hidden (LabelScale is 0).");
    else sb.AppendLine(string.Format("  - Labels are written on each panel at a size that fits it, times LabelScale ({0:0.##}). Set LabelScale to 0 to hide them.", labelScale));
    if (tall > 0) sb.AppendLine(string.Format("  - {0} props are over 20 ft: these need braced towers, not single props.", tall));
    if (shapeCount.Count > 0.5 * layout.Count && layout.Count > 12) sb.AppendLine("  - Most panels are different from each other: every form has to be set out and cut individually.");
    if (steep > 0) sb.AppendLine("  - Steep panels need forms on both faces, or the concrete will slump.");
    if (props.Count == 0) sb.AppendLine("  - No props were placed: the soffit is under 1 ft above the ground everywhere, or PropSpacing is larger than the roof.");
    sb.AppendLine("  - Props are placed and counted, not sized. Bearers along the fold lines are implied by the plan, not drawn separately.");
    sb.AppendLine("  - Form panels are taken at the mid-surface of the slab; true soffit panels are slightly smaller at valleys and larger at ridges.");
    sb.AppendLine("  - The plan is drawn on the ground under the model and the section in its cut plane. Bake them and use Make2D for sheets.");

    // ---------------- bake (button in the control window) ----------------
    bool bakeNow = false;
    try
    {
      object wb = AppDomain.CurrentDomain.GetData("FoldToolkit.FormBake");
      if (wb is bool && (bool)wb) { bakeNow = true; AppDomain.CurrentDomain.SetData("FoldToolkit.FormBake", false); }
    }
    catch { }
    if (bakeNow && RhinoDoc.ActiveDoc != null)
    {
      try
      {
        RhinoDoc doc = RhinoDoc.ActiveDoc;
        Rhino.DocObjects.RhinoObject[] old = doc.Objects.FindByUserString("FoldToolkitForm", "*", true);
        if (old != null) foreach (Rhino.DocObjects.RhinoObject ro in old) doc.Objects.Delete(ro, true);
        int parent = doc.Layers.FindByFullPath("Formwork", -1);
        if (parent < 0) parent = doc.Layers.Add(new Rhino.DocObjects.Layer { Name = "Formwork" });
        Guid parentId = doc.Layers[parent].Id;
        string[] names = { "Form panels", "Props", "Plan", "Section", "Labels" };
        System.Drawing.Color[] cols = {
          System.Drawing.Color.FromArgb(200, 120, 40), System.Drawing.Color.FromArgb(90, 90, 90),
          System.Drawing.Color.FromArgb(0, 110, 200), System.Drawing.Color.FromArgb(200, 0, 0),
          System.Drawing.Color.FromArgb(20, 20, 20) };
        var atts = new Rhino.DocObjects.ObjectAttributes[names.Length];
        for (int L = 0; L < names.Length; L++)
        {
          int li = doc.Layers.FindByFullPath("Formwork::" + names[L], -1);
          if (li < 0) li = doc.Layers.Add(new Rhino.DocObjects.Layer { Name = names[L], ParentLayerId = parentId, Color = cols[L] });
          atts[L] = new Rhino.DocObjects.ObjectAttributes();
          atts[L].LayerIndex = li;
          atts[L].SetUserString("FoldToolkitForm", names[L]);
        }
        int baked = 0;
        foreach (Polyline pl in layout) { doc.Objects.AddPolyline(pl, atts[0]); baked++; }
        foreach (Line ln in props) { doc.Objects.AddLine(ln, atts[1]); baked++; }
        foreach (Curve c in plan) { doc.Objects.AddCurve(c, atts[2]); baked++; }
        foreach (Curve c in section) { doc.Objects.AddCurve(c, atts[3]); baked++; }
        if (labelScale > 0)
        {
          for (int i = 0; i < tagCentres.Count; i++)
          {
            // text lying on the panel, on its upper face, reading the way it is seen from above
            Vector3d x = tagX[i], y = tagY[i];
            Vector3d n = Vector3d.CrossProduct(x, y);
            if (n.Z < 0) { x = -x; n = -n; }
            if (Vector3d.CrossProduct(n, x) * Vector3d.YAxis < -1e-9 && Math.Abs(n.Z) > 0.7) x = -x;   // keep flat labels upright on plan
            y = Vector3d.CrossProduct(n, x);
            n.Unitize();
            Point3d cc = tagCentres[i] + n * labelLift;
            double ht = tagHeights[i] * labelScale;
            double wt = 0.62 * ht * tagTexts[i].Length;
            Plane tp = new Plane(cc - x * (wt / 2.0) - y * (ht / 2.0), x, y);
            if (doc.Objects.AddText(tagTexts[i], tp, ht, "Arial", false, false, atts[4]) != Guid.Empty) baked++;
          }
        }
        doc.Views.Redraw();
        sb.AppendLine(string.Format("  - Baked {0} objects on layer Formwork: Form panels, Props, Plan, Section, Labels. The formwork baked before was replaced.", baked));
      }
      catch (Exception ex) { sb.AppendLine("  - Baking failed: " + ex.Message); }
    }

    Layout = layout;
    Props = props;
    Plan = plan;
    Section = section;
    Report = sb.ToString();
  }
}
