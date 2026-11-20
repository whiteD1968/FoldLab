// Folded Plate Toolkit - Model Kit (laser cutting)
// Rhino 8 Grasshopper C# Script component. Model units: FEET.
//
// Unrolls the folded surface into flat pieces at model scale, with cut lines,
// score lines for the folds, glue tabs and labels. Every piece unrolls exactly:
// no stretching.
//
// Inputs:
//   Plates     : the Plates output of the Fold Designer
//   Scale      : model scale as 1 : Scale (default 48, i.e. 1/4 in = 1 ft)
//   Direction  : 0 = bands across the folds, one per station (the folds become
//                score lines; pieces join at the stations). Default.
//                1 = strips along the folds, one per fold (pieces join at the folds).
//   TabWidth   : in, at model size (default 0.25; 0 = no tabs)
//   BedWidth   : in, laser bed width (default 18)
//   BedLength  : in, laser bed length (default 32)
//   LayoutGap  : ft, clear distance from the model to the kit (default 20)
//   LabelScale : multiplier on label size (default 1 = 3/16 in letters; 0 = no labels)
//   Dash       : in, at model size, length of each dash on fold lines and tab folds (default 0.125)
//   Gap        : in, at model size, uncut bridge between dashes (default 0.125; 0 = solid line)
//   CutColor   : colour of the cut outlines (default red)
//   EtchColor  : colour of the labels, and of the fold lines unless FoldColor is set (default blue)
//   FoldColor  : optional, colour of ridge, valley and tab fold lines
//   Bake       : button. Puts the kit into Rhino on layer Model Kit, one sublayer per laser
//                operation (Cut, Score ridge, Score valley, Tab fold, Etch labels) plus
//                Labels on model, replacing the kit baked before.
// Outputs:
//   Cut      : closed outlines, including tabs
//   Mountain : score lines that fold as ridges
//   Valley   : score lines that fold as valleys
//   TabFold  : dashed fold line at the root of each glue tab
//   Engrave  : labels as single-line letters (one laser pass per stroke)
//   Report   : scale, piece count, sizes, bed fit
//
// Any input left unwired takes its value from the control window (section 6,
// Model kit) when the window component is in the file. A wired input always wins.
//
// The kit is drawn at true model size in the file's units (feet), so exporting
// with units converts it correctly: 1 ft in the file is 12 in on the laser.
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

  // labels for the viewport: centre, height and text. Each is turned when drawn
  // so it reads the right way round from wherever the camera is.
  private List<Point3d> lblCentres = new List<Point3d>();
  private List<double> lblHeights = new List<double>();
  private List<string> lblTexts = new List<string>();
  private BoundingBox lblBox = BoundingBox.Empty;
  // the same names on the 3D model: centre, outward normal, reading direction, height
  private List<Point3d> mdlCentres = new List<Point3d>();
  private List<Vector3d> mdlNormals = new List<Vector3d>(), mdlX = new List<Vector3d>();
  private List<double> mdlHeights = new List<double>();
  private List<string> mdlTexts = new List<string>();
  // the kit's own coloured preview
  private List<Curve> drawCut = new List<Curve>(), drawFold = new List<Curve>(), drawEtch = new List<Curve>();
  private System.Drawing.Color colCut = System.Drawing.Color.Red, colFold = System.Drawing.Color.Blue, colEtch = System.Drawing.Color.Blue;

  public override BoundingBox ClippingBox { get { return lblBox; } }

  public override void DrawViewportWires(IGH_PreviewArgs args)
  {
    try
    {
      System.Drawing.Color ink = colEtch;
      foreach (Curve c in drawCut) args.Display.DrawCurve(c, colCut, 2);
      foreach (Curve c in drawFold) args.Display.DrawCurve(c, colFold, 2);
      bool parallel = args.Viewport.IsParallelProjection;
      Vector3d camDir = args.Viewport.CameraDirection;
      Vector3d camUp = args.Viewport.CameraUp;
      Point3d camAt = args.Viewport.CameraLocation;
      for (int i = 0; i < lblCentres.Count; i++)
      {
        Point3d lc0 = lblCentres[i];
        Vector3d x = Vector3d.XAxis, y = Vector3d.YAxis;
        Vector3d look = parallel ? camDir : lc0 - camAt;
        if (Vector3d.CrossProduct(x, y) * look > 0) x = -x;   // seen from below: mirror it back
        if (y * camUp < 0) { x = -x; y = -y; }                // upside down: turn it round
        foreach (Polyline pl in StrokeText(lblTexts[i], lc0, x, y, lblHeights[i]))
          args.Display.DrawPolyline(pl, ink, 2);
      }
      for (int i = 0; i < mdlCentres.Count; i++)
      {
        Point3d mc = mdlCentres[i];
        Vector3d n = mdlNormals[i], x = mdlX[i];
        Vector3d look = parallel ? camDir : mc - camAt;
        if (n * look > 0) n = -n;                              // seen from the other side
        Vector3d y = Vector3d.CrossProduct(n, x);
        if (y * camUp < 0) { x = -x; y = -y; }                // upside down: turn it round
        foreach (Polyline pl in StrokeText(mdlTexts[i], mc + n * 0.02, x, y, mdlHeights[i]))
          args.Display.DrawPolyline(pl, colEtch, 2);
      }
    }
    catch { }
  }

  // Single-line lettering (one stroke per line, like a CAM engraving font), so the
  // laser traces each letter once instead of outlining it. Grid: 4 wide, 6 tall.
  private static string Glyph(char ch)
  {
    switch (ch)
    {
      case '0': return "1,0 3,0 4,1 4,5 3,6 1,6 0,5 0,1 1,0";
      case '1': return "1,5 2,6 2,0";
      case '2': return "0,5 1,6 3,6 4,5 4,4 0,0 4,0";
      case '3': return "0,5 1,6 3,6 4,5 4,4 3,3 4,2 4,1 3,0 1,0 0,1;1.5,3 3,3";
      case '4': return "3,0 3,6 0,2 4,2";
      case '5': return "4,6 0,6 0,3.5 3,3.5 4,2.5 4,1 3,0 1,0 0,1";
      case '6': return "4,5 3,6 1,6 0,5 0,1 1,0 3,0 4,1 4,2.5 3,3.5 0,3.5";
      case '7': return "0,6 4,6 1.5,0";
      case '8': return "1,3 0,4 0,5 1,6 3,6 4,5 4,4 3,3 1,3 0,2 0,1 1,0 3,0 4,1 4,2 3,3";
      case '9': return "0,1 1,0 3,0 4,1 4,5 3,6 1,6 0,5 0,3.5 1,2.5 4,2.5";
      case 'S': return "4,5 3,6 1,6 0,5 0,4 1,3 3,3 4,2 4,1 3,0 1,0 0,1";
      case 'F': return "0,0 0,6 4,6;0,3 3,3";
      case '-': return "1,3 3,3";
    }
    return "";
  }

  private const double GlyphAdvance = 5.6;   // grid units from one letter to the next

  private static double StrokeWidth(string text, double ht)
  {
    return (text.Length * GlyphAdvance - (GlyphAdvance - 4.0)) * ht / 6.0;
  }

  // text centred on 'centre', reading along x, letter height ht
  private static List<Polyline> StrokeText(string text, Point3d centre, Vector3d x, Vector3d y, double ht)
  {
    var res = new List<Polyline>();
    double u = ht / 6.0;
    Point3d o = centre - x * (StrokeWidth(text, ht) / 2.0) - y * (ht / 2.0);
    System.Globalization.CultureInfo inv = System.Globalization.CultureInfo.InvariantCulture;
    for (int i = 0; i < text.Length; i++)
    {
      string g = Glyph(text[i]);
      if (g.Length == 0) continue;
      foreach (string stroke in g.Split(';'))
      {
        var pl = new Polyline();
        foreach (string pt in stroke.Split(' '))
        {
          string[] xy = pt.Split(',');
          if (xy.Length != 2) continue;
          double gx = double.Parse(xy[0], inv) + i * GlyphAdvance, gy = double.Parse(xy[1], inv);
          pl.Add(o + x * (gx * u) + y * (gy * u));
        }
        if (pl.Count > 1) res.Add(pl);
      }
    }
    return res;
  }

  // values from the control window, used for any input left unwired here
  private Dictionary<string, double> kit = null;
  private double W(string key, double fallback)
  {
    double v;
    if (kit != null && kit.TryGetValue(key, out v)) return v;
    return fallback;
  }
  private System.Drawing.Color WC(string key, System.Drawing.Color fallback)
  {
    double v;
    if (kit != null && kit.TryGetValue(key, out v)) return System.Drawing.Color.FromArgb(unchecked((int)(long)v));
    return fallback;
  }

  private static System.Drawing.Color Col(object o, System.Drawing.Color fallback)
  {
    if (o is System.Drawing.Color) return (System.Drawing.Color)o;
    if (o is GH_Colour) return ((GH_Colour)o).Value;
    return fallback;
  }

  // A dashed line from P to Q. Dash and gap are stretched together by a small factor so the
  // pattern fits the line exactly. endOnDash: whole dashes at both ends (tab roots, so the
  // corners are cut). Otherwise a half gap at both ends (folds inside a piece, so the
  // dashes never nick the outline). gap 0 gives one solid line.
  private static void AddDashes(List<Curve> to, Point3d P, Point3d Q, double dash, double gap, bool endOnDash)
  {
    double len = P.DistanceTo(Q);
    if (len < 1e-9) return;
    if (gap <= 0) { to.Add(new LineCurve(P, Q)); return; }
    Vector3d dir = (Q - P) / len;
    if (endOnDash)
    {
      int nd = (int)Math.Round((len + gap) / (dash + gap));
      if (nd < 2) nd = 2; if (nd > 2000) nd = 2000;
      double k = len / (nd * dash + (nd - 1) * gap);
      for (int q = 0; q < nd; q++)
        to.Add(new LineCurve(P + dir * (q * (dash + gap) * k), P + dir * ((q * (dash + gap) + dash) * k)));
    }
    else
    {
      int nd = (int)Math.Round(len / (dash + gap));
      if (nd < 1) nd = 1; if (nd > 2000) nd = 2000;
      double k = len / (nd * (dash + gap));
      for (int q = 0; q < nd; q++)
        to.Add(new LineCurve(P + dir * ((gap / 2.0 + q * (dash + gap)) * k), P + dir * ((gap / 2.0 + q * (dash + gap) + dash) * k)));
    }
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

  private static double Cross2(Point2d a, Point2d b, Point2d c)
  {
    return (b.X - a.X) * (c.Y - a.Y) - (b.Y - a.Y) * (c.X - a.X);
  }

  private void RunScript(
    object Plates, object Scale, object Direction, object TabWidth, object BedWidth, object BedLength,
    object LayoutGap, object LabelScale, object Dash, object Gap, object CutColor, object EtchColor, object FoldColor, object Bake,
    ref object Cut, ref object Mountain, ref object Valley, ref object TabFold, ref object Engrave, ref object Report)
  {
    lblCentres = new List<Point3d>(); lblHeights = new List<double>(); lblTexts = new List<string>(); lblBox = BoundingBox.Empty;
    mdlCentres = new List<Point3d>(); mdlNormals = new List<Vector3d>(); mdlX = new List<Vector3d>(); mdlHeights = new List<double>(); mdlTexts = new List<string>();
    if (!ToolOn("KitOn")) { Report = "MODELKIT is switched off in the control window (section 6)."; drawCut = new List<Curve>(); drawFold = new List<Curve>(); return; }
    var modelLabels = new List<Curve>();
    double slabFt = 0.5;
    try { object so = AppDomain.CurrentDomain.GetData("FoldToolkit.SlabFt"); if (so is double) slabFt = (double)so; } catch { }
    Mesh mesh = AsMesh(Plates);
    if (mesh == null || mesh.Faces.Count == 0) { Report = "Connect the Plates output of the Fold Designer to Plates."; return; }
    kit = null;
    try { kit = AppDomain.CurrentDomain.GetData("FoldToolkit.Kit") as Dictionary<string, double>; } catch { }
    double scale = Num(Scale, W("KitScale", 48.0)); if (scale <= 0) scale = 48.0;
    int direction = (int)Math.Round(Num(Direction, W("KitDirection", 0.0))); if (direction != 1) direction = 0;
    double tabIn = Num(TabWidth, W("KitTab", 0.25)); if (tabIn < 0) tabIn = 0.0;
    double bedW = Num(BedWidth, W("KitBedW", 18.0)); if (bedW <= 0) bedW = 18.0;
    double bedL = Num(BedLength, W("KitBedL", 32.0)); if (bedL <= 0) bedL = 32.0;
    double gap = Num(LayoutGap, 20.0); if (gap < 0) gap = 20.0;
    double lblScale = Num(LabelScale, W("KitLabel", 1.0)); if (lblScale < 0) lblScale = 0.0;

    int[] tags = null; int row = 0;
    try
    {
      tags = AppDomain.CurrentDomain.GetData("FoldToolkit.FaceTags") as int[];
      object r = AppDomain.CurrentDomain.GetData("FoldToolkit.Row");
      if (r is int) row = (int)r;
    }
    catch { }
    if (tags == null || tags.Length != mesh.Faces.Count || row < 2)
    {
      Report = "The model kit needs plates straight from the Fold Designer (dated 16 Nov or later), so it knows which station and fold each plate belongs to.";
      return;
    }

    double sc = 1.0 / scale;                 // feet in the building -> feet of model
    double tab = tabIn / 12.0;               // tab width in file units (ft), at model size
    double pad = 0.5 / 12.0;                 // half an inch between pieces
    BoundingBox bb = mesh.GetBoundingBox(true);
    double ground = bb.Min.Z;

    // ---------------- triangles, grouped into pieces ----------------
    // each entry: v0, v1, v2, group, order
    var tris = new List<int[]>();
    for (int f = 0; f < mesh.Faces.Count; f++)
    {
      MeshFace mf = mesh.Faces[f];
      int k = tags[f] / 10000, i = tags[f] % 10000;
      int grp = direction == 0 ? k : i, ord = direction == 0 ? i : k;
      tris.Add(new int[] { mf.A, mf.B, mf.C, grp, ord, f });
      if (mf.IsQuad) tris.Add(new int[] { mf.A, mf.C, mf.D, grp, ord, f });
    }
    var groups = new SortedDictionary<int, List<int[]>>();
    foreach (int[] t in tris)
    {
      if (!groups.ContainsKey(t[3])) groups[t[3]] = new List<int[]>();
      groups[t[3]].Add(t);
    }

    var cut = new List<Curve>(); var mountain = new List<Curve>(); var valley = new List<Curve>(); var engrave = new List<Curve>(); var tabFold = new List<Curve>();
    int pieces = 0, tooBig = 0, ringCut = 0, skipped = 0, tabCount = 0, lblShrunk = 0;
    int lastGroup = -1;
    foreach (int gk in groups.Keys) lastGroup = Math.Max(lastGroup, gk);
    double maxW = 0.0, maxH = 0.0;
    double flowW = 6.0 * bedL / 12.0;        // wrap the layout after about six bed lengths
    double curX = 0.0, curY = 0.0, rowH = 0.0;
    Point3d origin = new Point3d(bb.Min.X, bb.Min.Y - gap, ground);
    double dashIn = Num(Dash, W("KitDash", 0.125)); if (dashIn < 0.01) dashIn = 0.01;   // in, at model size
    double gapIn = Num(Gap, W("KitGap", 0.125)); if (gapIn < 0) gapIn = 0.0;           // 0 = one solid score line
    double dashFt = dashIn / 12.0, gapFt = gapIn / 12.0;
    colCut = Col(CutColor, WC("KitCut", System.Drawing.Color.Red));
    colEtch = Col(EtchColor, WC("KitEtch", System.Drawing.Color.Blue));
    colFold = Col(FoldColor, EtchColor != null ? colEtch : WC("KitFold", colEtch));        // fold lines follow the etch colour unless given their own
    int nRidge = 0, nValley = 0;

    foreach (KeyValuePair<int, List<int[]>> g in groups)
    {
      List<int[]> list = g.Value;
      list.Sort((p, q) => p[4] != q[4] ? p[4].CompareTo(q[4]) : p[5].CompareTo(q[5]));

      // ---- unfold the chain of triangles into the plane ----
      var pos = new Dictionary<int, Point2d>();
      var done = new List<int[]>();
      var todo = new List<int[]>(list);
      {
        int[] t0 = todo[0]; todo.RemoveAt(0);
        Point3d A = mesh.Vertices.Point3dAt(t0[0]), B = mesh.Vertices.Point3dAt(t0[1]), C = mesh.Vertices.Point3dAt(t0[2]);
        double L = A.DistanceTo(B);
        if (L < 1e-9) { skipped++; continue; }
        double dA = C.DistanceTo(A), dB = C.DistanceTo(B);
        double ax = (dA * dA - dB * dB + L * L) / (2.0 * L);
        double hh = Math.Sqrt(Math.Max(0.0, dA * dA - ax * ax));
        pos[t0[0]] = new Point2d(0, 0); pos[t0[1]] = new Point2d(L, 0); pos[t0[2]] = new Point2d(ax, hh);
        done.Add(t0);
      }
      bool progress = true;
      while (todo.Count > 0 && progress)
      {
        progress = false;
        for (int n = 0; n < todo.Count; n++)
        {
          int[] t = todo[n];
          int placed = (pos.ContainsKey(t[0]) ? 1 : 0) + (pos.ContainsKey(t[1]) ? 1 : 0) + (pos.ContainsKey(t[2]) ? 1 : 0);
          if (placed < 2) continue;
          if (placed == 3)
          {
            // all three corners already have a place: only keep it if it fits there
            bool fits = true;
            for (int e = 0; e < 3; e++)
            {
              int u = t[e], v = t[(e + 1) % 3];
              double d3 = mesh.Vertices.Point3dAt(u).DistanceTo(mesh.Vertices.Point3dAt(v));
              if (Math.Abs(pos[u].DistanceTo(pos[v]) - d3) > 1e-4 * Math.Max(1.0, d3)) fits = false;
            }
            if (fits) done.Add(t); else ringCut++;
            todo.RemoveAt(n); progress = true; break;
          }
          int iu = -1, iv = -1, ip = -1;
          for (int e = 0; e < 3; e++)
            if (!pos.ContainsKey(t[e])) ip = t[e];
          for (int e = 0; e < 3; e++)
            if (t[e] != ip) { if (iu < 0) iu = t[e]; else iv = t[e]; }
          Point2d U = pos[iu], V = pos[iv];
          Point3d P3 = mesh.Vertices.Point3dAt(ip);
          double base2 = U.DistanceTo(V);
          if (base2 < 1e-9) { todo.RemoveAt(n); skipped++; progress = true; break; }
          double dU = P3.DistanceTo(mesh.Vertices.Point3dAt(iu)), dV = P3.DistanceTo(mesh.Vertices.Point3dAt(iv));
          double along = (dU * dU - dV * dV + base2 * base2) / (2.0 * base2);
          double off = Math.Sqrt(Math.Max(0.0, dU * dU - along * along));
          double ex = (V.X - U.X) / base2, ey = (V.Y - U.Y) / base2;
          // open the fold flat: put the new corner on the far side of the shared edge
          double sideSign = 1.0;
          foreach (int[] dn in done)
          {
            int w = -1, hit = 0;
            for (int e = 0; e < 3; e++) { if (dn[e] == iu || dn[e] == iv) hit++; else w = dn[e]; }
            if (hit == 2 && w >= 0) { sideSign = Cross2(U, V, pos[w]) > 0 ? -1.0 : 1.0; break; }
          }
          pos[ip] = new Point2d(U.X + along * ex - sideSign * off * ey, U.Y + along * ey + sideSign * off * ex);
          done.Add(t); todo.RemoveAt(n); progress = true; break;
        }
      }
      skipped += todo.Count;
      if (done.Count == 0) continue;

      // ---- size, turn the long way along X, and place in the layout ----
      double uMin = double.MaxValue, uMax = double.MinValue, vMin = double.MaxValue, vMax = double.MinValue;
      foreach (Point2d p in pos.Values)
      {
        uMin = Math.Min(uMin, p.X); uMax = Math.Max(uMax, p.X); vMin = Math.Min(vMin, p.Y); vMax = Math.Max(vMax, p.Y);
      }
      bool turn = (vMax - vMin) > (uMax - uMin);
      double w2 = (turn ? vMax - vMin : uMax - uMin) * sc, h2 = (turn ? uMax - uMin : vMax - vMin) * sc;
      if (curX > 0 && curX + w2 > flowW) { curX = 0.0; curY -= rowH + pad + 2.0 * tab; rowH = 0.0; }
      double ox = origin.X + curX, oy = origin.Y + curY - h2;
      // 2D model-space point of a mesh vertex
      Func<Point2d, Point3d> place = p =>
      {
        double a = turn ? (p.Y - vMin) : (p.X - uMin);
        double b = turn ? (uMax - p.X) : (p.Y - vMin);
        return new Point3d(ox + a * sc, oy + b * sc, ground);
      };
      var at = new Dictionary<int, Point3d>();
      foreach (KeyValuePair<int, Point2d> kv in pos) at[kv.Key] = place(kv.Value);

      // ---- edges: outline, tabs, score lines ----
      var dirEdges = new Dictionary<long, int[]>();      // directed edge -> the triangle it belongs to
      long big = (long)mesh.Vertices.Count + 1;
      foreach (int[] t in done)
      {
        // make every triangle counter-clockwise in the layout
        Point3d a3 = at[t[0]], b3 = at[t[1]], c3 = at[t[2]];
        bool ccw = (b3.X - a3.X) * (c3.Y - a3.Y) - (b3.Y - a3.Y) * (c3.X - a3.X) > 0;
        int[] o = ccw ? new int[] { t[0], t[1], t[2] } : new int[] { t[0], t[2], t[1] };
        for (int e = 0; e < 3; e++) dirEdges[o[e] * big + o[(e + 1) % 3]] = t;
      }
      var next = new Dictionary<int, int>();
      foreach (KeyValuePair<long, int[]> kv in dirEdges)
      {
        int u = (int)(kv.Key / big), v = (int)(kv.Key % big);
        if (dirEdges.ContainsKey(v * big + u))
        {
          // interior edge: a fold. Draw it once and decide ridge or valley from the 3D model.
          if (u < v)
          {
            int[] t1 = kv.Value, t2 = dirEdges[v * big + u];
            Point3d p0 = mesh.Vertices.Point3dAt(t1[0]), p1 = mesh.Vertices.Point3dAt(t1[1]), p2 = mesh.Vertices.Point3dAt(t1[2]);
            Vector3d n1 = Vector3d.CrossProduct(p1 - p0, p2 - p0);
            int far = -1;
            for (int e = 0; e < 3; e++) if (t2[e] != u && t2[e] != v) far = t2[e];
            if (n1.Unitize() && far >= 0)
            {
              Point3d eu = mesh.Vertices.Point3dAt(u);
              double s = n1 * (mesh.Vertices.Point3dAt(far) - eu);
              double reach = Math.Max(1.0, mesh.Vertices.Point3dAt(far).DistanceTo(eu));
              if (s < -1e-6 * reach) { AddDashes(mountain, at[u], at[v], dashFt, gapFt, false); nRidge++; }
              else if (s > 1e-6 * reach) { AddDashes(valley, at[u], at[v], dashFt, gapFt, false); nValley++; }
            }
          }
        }
        else next[u] = v;                                // boundary edge, interior on its left
      }
      if (next.Count < 3) continue;
      var loop = new Polyline();
      int start = -1;
      foreach (int key in next.Keys) { start = key; break; }
      int cur = start, guard = 0;
      do
      {
        int nx2 = next[cur];
        Point3d P = at[cur], Q = at[nx2];
        loop.Add(P);
        // a glue tab on the edges that meet the next piece
        bool tabEdge = false;
        if (tab > 0 && g.Key != lastGroup)       // the last piece has nothing to glue to
        {
          int ka = cur / row, ia = cur % row, kb = nx2 / row, ib = nx2 % row;
          if (direction == 0) tabEdge = (ka == kb && ka == g.Key + 1);
          else tabEdge = (ia == ib && ia == g.Key + 1);
        }
        double len = P.DistanceTo(Q);
        if (tabEdge && len > 3.0 * tab)
        {
          Vector3d dir = (Q - P) / len;
          Vector3d outward = new Vector3d(dir.Y, -dir.X, 0.0);   // interior is on the left
          double inset = Math.Min(tab, len / 4.0);
          loop.Add(P + dir * inset + outward * tab);
          loop.Add(Q - dir * inset + outward * tab);
          tabCount++;
          // dashed fold line along the root of the tab: starts and ends with a dash
          AddDashes(tabFold, P, Q, dashFt, gapFt, true);
        }
        cur = nx2; guard++;
      } while (cur != start && next.ContainsKey(cur) && guard < 100000);
      loop.Add(loop[0]);
      cut.Add(new PolylineCurve(loop));
      pieces++;

      // ---- label ----
      string name = (direction == 0 ? "S" : "F") + (g.Key + 1).ToString("00");
      double pieceWin = w2 * 12.0, pieceHin = h2 * 12.0;
      maxW = Math.Max(maxW, pieceWin); maxH = Math.Max(maxH, pieceHin);
      if (Math.Min(pieceWin, pieceHin) > Math.Min(bedW, bedL) + 1e-6 || Math.Max(pieceWin, pieceHin) > Math.Max(bedW, bedL) + 1e-6) tooBig++;
      if (lblScale > 0)
      {
        // put it in the roomiest triangle of the piece, at the centre of the largest
        // circle that fits there, and no bigger than that circle allows
        Point3d c0 = Point3d.Origin; double rBest = -1.0;
        foreach (int[] tf in done)
        {
          Point3d tA = at[tf[0]], tB = at[tf[1]], tC = at[tf[2]];
          double la = tB.DistanceTo(tC), lb = tC.DistanceTo(tA), lc = tA.DistanceTo(tB), perim = la + lb + lc;
          if (perim < 1e-12) continue;
          double rin = Vector3d.CrossProduct(tB - tA, tC - tA).Length / perim;
          if (rin > rBest)
          {
            rBest = rin;
            c0 = new Point3d((la * tA.X + lb * tB.X + lc * tC.X) / perim, (la * tA.Y + lb * tB.Y + lc * tC.Y) / perim, ground + 0.001);
          }
        }
        if (rBest > 0)
        {
          double ht = 0.1875 / 12.0 * lblScale;                     // 3/16 in letters at LabelScale 1
          double wide = StrokeWidth(name, 1.0);
          double fit = 1.7 * rBest / Math.Sqrt(1.0 + wide * wide);  // keeps the text inside the circle
          if (ht > fit) { ht = fit; lblShrunk++; }
          lblCentres.Add(c0); lblHeights.Add(ht); lblTexts.Add(name);
          lblBox.Union(c0 - new Vector3d(ht * wide, ht * wide, 0)); lblBox.Union(c0 + new Vector3d(ht * wide, ht * wide, 0));
          foreach (Polyline pl in StrokeText(name, c0, Vector3d.XAxis, Vector3d.YAxis, ht)) engrave.Add(new PolylineCurve(pl));
        }

        // the same name on the 3D model, in the roomiest plate of this piece
        double mBest = -1.0; Point3d mC = Point3d.Origin; Vector3d mN = Vector3d.ZAxis, mX = Vector3d.XAxis;
        foreach (int[] t in list)
        {
          Point3d tA = mesh.Vertices.Point3dAt(t[0]), tB = mesh.Vertices.Point3dAt(t[1]), tC = mesh.Vertices.Point3dAt(t[2]);
          double la = tB.DistanceTo(tC), lb = tC.DistanceTo(tA), lc = tA.DistanceTo(tB), perim = la + lb + lc;
          if (perim < 1e-12) continue;
          Vector3d nn = Vector3d.CrossProduct(tB - tA, tC - tA);
          double rin = nn.Length / perim;
          if (rin <= mBest || !nn.Unitize()) continue;
          mBest = rin;
          mC = new Point3d((la * tA.X + lb * tB.X + lc * tC.X) / perim, (la * tA.Y + lb * tB.Y + lc * tC.Y) / perim, (la * tA.Z + lb * tB.Z + lc * tC.Z) / perim);
          if (nn.Z < 0) nn = -nn;                                  // the upper face, seen from above
          mN = nn;
          // read along the most level direction in the plate
          Vector3d xx = Vector3d.CrossProduct(Vector3d.ZAxis, nn);
          if (!xx.Unitize()) xx = Vector3d.XAxis;
          mX = xx;
        }
        if (mBest > 0)
        {
          double wide = StrokeWidth(name, 1.0);
          double mh = 1.2 * mBest / Math.Sqrt(1.0 + wide * wide) * Math.Min(1.0, lblScale);
          Point3d onTop = mC + mN * (slabFt / 2.0 + 0.02);
          mdlCentres.Add(onTop); mdlNormals.Add(mN); mdlX.Add(mX); mdlHeights.Add(mh); mdlTexts.Add(name);
          lblBox.Union(onTop - new Vector3d(mh * wide, mh * wide, mh * wide)); lblBox.Union(onTop + new Vector3d(mh * wide, mh * wide, mh * wide));
          foreach (Polyline pl in StrokeText(name, onTop, mX, Vector3d.CrossProduct(mN, mX), mh)) modelLabels.Add(new PolylineCurve(pl));
        }
      }
      curX += w2 + pad + 2.0 * tab; rowH = Math.Max(rowH, h2);
    }

    // ---------------- report ----------------
    var sb = new StringBuilder();
    sb.AppendLine("MODEL KIT");
    sb.AppendLine(string.Format("Scale 1 : {0:0.##} | pieces are {1} | tabs {2:0.##} in", scale,
      direction == 0 ? "bands across the folds, one per station" : "strips along the folds, one per fold", tabIn));
    sb.AppendLine();
    sb.AppendLine("PIECES");
    sb.AppendLine(string.Format("  Pieces                     {0}", pieces));
    sb.AppendLine(string.Format("  Largest piece              {0:0.0} x {1:0.0} in", maxW, maxH));
    sb.AppendLine(string.Format("  Laser bed                  {0:0.#} x {1:0.#} in", bedW, bedL));
    sb.AppendLine(string.Format("  Pieces too big for the bed {0}", tooBig));
    sb.AppendLine(string.Format("  Model size (roof)          {0:0.0} x {1:0.0} x {2:0.0} in",
      (bb.Max.X - bb.Min.X) * sc * 12.0, (bb.Max.Y - bb.Min.Y) * sc * 12.0, (bb.Max.Z - bb.Min.Z) * sc * 12.0));
    sb.AppendLine(string.Format("  Score lines                {0} ridge, {1} valley", nRidge, nValley));
    sb.AppendLine(string.Format("  Glue tabs                  {0}, each {1:0.##} in deep", tabCount, tabIn));
    sb.AppendLine();
    sb.AppendLine("FLAGS");
    { string un = UnitNote(); if (un != null) sb.AppendLine("  - " + un); }
    if (tabIn > 0 && tabCount == 0 && pieces > 1) sb.AppendLine("  - No tabs were made: the edges between pieces are shorter than three tab widths. Reduce TabWidth.");
    if (tabCount > 0 && tabIn < 0.02 * Math.Max(maxW, maxH)) sb.AppendLine("  - Tabs are small next to the pieces: zoom in on an edge between two pieces to see them.");
    if (tooBig > 0) sb.AppendLine("  - Some pieces are larger than the laser bed: use a smaller scale (a bigger Scale number), switch Direction, or split those pieces by hand.");
    if (ringCut > 0) sb.AppendLine("  - The roof closes into a ring, which cannot lie flat in one piece: each ring was cut open and is missing the plate at the cut.");
    if (skipped > 0) sb.AppendLine(string.Format("  - {0} very small or disconnected plates were left out.", skipped));
    if (lblShrunk > 0) sb.AppendLine(string.Format("  - {0} labels were made smaller to fit inside their piece.", lblShrunk));
    sb.AppendLine("  - Labels are single-line letters, 3/16 in tall at LabelScale 1: engrave or score them, do not cut.");
    if (gapIn > 0) sb.AppendLine(string.Format("  - Fold lines and tab folds are dashed: {0:0.###} in dash, {1:0.###} in gap, adjusted slightly on each line so the pattern fits. Tab folds end on a dash; folds inside a piece end on a bridge so they do not nick the outline.", dashIn, gapIn));
    else sb.AppendLine("  - Gap is 0: fold lines and tab folds are solid lines. Score them, do not cut through.");
    if (kit != null) sb.AppendLine("  - Inputs left unwired on this component take their values from the control window (section 6). A wired input always wins.");
    if (lblScale > 0) sb.AppendLine("  - Piece names match the formwork: S03 is the band between stations 3 and 4 counted from the start of the path, F03 is the third fold counted across. The same names are drawn on the model.");
    sb.AppendLine("  - Ridge scores fold away from you and valley scores toward you, with the labelled face up.");
    sb.AppendLine("  - Tabs sit on the edge that meets the next piece. They are plain glue tabs; material thickness is not allowed for.");
    sb.AppendLine("  - Drawn at true model size in feet: when exporting for the laser, let Rhino convert units (1 ft = 12 in).");

    // ---------------- preview colours and baking ----------------
    drawCut = new List<Curve>(cut);
    drawFold = new List<Curve>(mountain); drawFold.AddRange(valley); drawFold.AddRange(tabFold);
    foreach (Curve c in cut) lblBox.Union(c.GetBoundingBox(false));
    bool bake = false;
    if (Bake is bool) bake = (bool)Bake; else if (Bake is GH_Boolean) bake = ((GH_Boolean)Bake).Value;
    try
    {
      // the "Bake model kit" button in the control window: one bake per click
      object wb = AppDomain.CurrentDomain.GetData("FoldToolkit.KitBake");
      if (wb is bool && (bool)wb) { bake = true; AppDomain.CurrentDomain.SetData("FoldToolkit.KitBake", false); }
    }
    catch { }
    if (bake && RhinoDoc.ActiveDoc != null)
    {
      try
      {
        RhinoDoc doc = RhinoDoc.ActiveDoc;
        // take out the kit baked last time (only objects this component tagged)
        Rhino.DocObjects.RhinoObject[] old = doc.Objects.FindByUserString("FoldToolkitKit", "*", true);
        if (old != null) foreach (Rhino.DocObjects.RhinoObject ro in old) doc.Objects.Delete(ro, true);
        // one parent layer, one sublayer per laser operation, coloured as set
        int parent = doc.Layers.FindByFullPath("Model Kit", -1);
        if (parent < 0) parent = doc.Layers.Add(new Rhino.DocObjects.Layer { Name = "Model Kit" });
        Guid parentId = doc.Layers[parent].Id;
        string[] layerNames = { "Cut", "Score ridge", "Score valley", "Tab fold", "Etch labels", "Labels on model" };
        List<Curve>[] sets = { cut, mountain, valley, tabFold, engrave, modelLabels };
        System.Drawing.Color[] cols = { colCut, colFold, colFold, colFold, colEtch, colEtch };
        int baked = 0;
        for (int L = 0; L < layerNames.Length; L++)
        {
          int li = doc.Layers.FindByFullPath("Model Kit::" + layerNames[L], -1);
          if (li < 0) li = doc.Layers.Add(new Rhino.DocObjects.Layer { Name = layerNames[L], ParentLayerId = parentId, Color = cols[L] });
          else
          {
            // keep the layer colour in step with the colour chosen now
            Rhino.DocObjects.Layer lay = doc.Layers[li];
            if (lay.Color.ToArgb() != cols[L].ToArgb()) { lay.Color = cols[L]; doc.Layers.Modify(lay, li, true); }
          }
          var att = new Rhino.DocObjects.ObjectAttributes();
          att.LayerIndex = li;
          att.ColorSource = Rhino.DocObjects.ObjectColorSource.ColorFromLayer;
          att.SetUserString("FoldToolkitKit", layerNames[L]);
          foreach (Curve c in sets[L]) { doc.Objects.AddCurve(c, att); baked++; }
        }
        doc.Views.Redraw();
        sb.AppendLine(string.Format("  - Baked {0} curves into Rhino on layer Model Kit: Cut, Score ridge, Score valley, Tab fold, Etch labels, Labels on model. The kit baked before was replaced. Set laser power and speed per sublayer (or by its colour).", baked));
      }
      catch (Exception ex) { sb.AppendLine("  - Baking failed: " + ex.Message); }
    }

    Cut = cut;
    Mountain = mountain;
    Valley = valley;
    TabFold = tabFold;
    Engrave = engrave;
    Report = sb.ToString();
  }
}
