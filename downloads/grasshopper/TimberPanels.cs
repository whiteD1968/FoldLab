// Folded Plate Toolkit - Timber Panels
// Rhino 8 Grasshopper C# Script component (TIMBER). Model units: FEET.
//
// Turns every plate into a timber panel of real thickness, centred on the
// plate's mid-surface. Where two plates meet, both panels are cut on the plane
// that halves the fold (a mitre), so they close without gaps. Free edges are cut
// square, and edges on the ground are cut level so the panel stands flat.
// Produces the panels, flat cutting blanks nested on stock sheets, an edge table
// with bevel angles, and a stock check.
//
// Inputs (an input left unwired takes its value from the control window, section 8):
//   Plates      : the Plates output of the Fold Designer
//   Material    : 0 plywood (3/4 in, 48 x 96 in sheets), 1 LVL panel (1 3/4 in,
//                 48 x 288 in), 2 CLT (4 1/8 in, 120 x 480 in). Default 0.
//   Thickness   : in, panel thickness as built (0 = the material's standard)
//   StockWidth  : in, sheet or panel width available (0 = the material's standard)
//   StockLength : in, sheet or panel length available (0 = the material's standard)
//   Scale       : 1 = full size; 4 = a 1:4 prototype. Thickness and stock are what
//                 you cut at that scale; blanks and the edge table are drawn and
//                 measured at that scale. Default 1.
//   LayoutGap   : ft, clear distance from the model to the sheets (default 20)
//   LabelScale  : multiplier on label size (default 1; 0 = no labels)
// Outputs:
//   Panels    : one closed solid per plate, at building size
//   Cut       : flat blank outlines, nested on the sheets (cut these through)
//   Bevel     : the inner edge of each bevel on the blank (mark or engrave)
//   Sheets    : stock sheet outlines
//   Engrave   : panel names and edge numbers, single-line letters
//   EdgeTable : one line per edge, comma separated (paste into a spreadsheet)
//   Report    : material, counts, stock, volume, weight, bevel range, flags
//
// Panel names match the formwork and the model kit: S03-F12 is the plate between
// stations 3 and 4, twelfth plate across; a and b are the two triangles of
// one cell. Edges are numbered E001, E002, ... and written on both panels they join.
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
      case 'E': return "4,6 0,6 0,0 4,0;0,3 3,3";
      case 'a': return "3,4 3,0;3,3 2,4 1,4 0,3 0,1 1,0 2,0 3,1";
      case 'b': return "0,6 0,0;0,3 1,4 2,4 3,3 3,1 2,0 1,0 0,1";
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
  }  // panel names drawn on the model, turned to the camera when drawn
  private List<Point3d> tagC = new List<Point3d>();
  private List<Vector3d> tagN = new List<Vector3d>(), tagX = new List<Vector3d>();
  private List<double> tagH = new List<double>();
  private List<string> tagT = new List<string>();
  private BoundingBox tagBox = BoundingBox.Empty;

  public override BoundingBox ClippingBox { get { return tagBox; } }

  public override void DrawViewportWires(IGH_PreviewArgs args)
  {
    try
    {
      System.Drawing.Color ink = System.Drawing.Color.FromArgb(20, 20, 20);
      bool parallel = args.Viewport.IsParallelProjection;
      Vector3d camDir = args.Viewport.CameraDirection;
      Vector3d camUp = args.Viewport.CameraUp;
      Point3d camAt = args.Viewport.CameraLocation;
      for (int i = 0; i < tagC.Count; i++)
      {
        Point3d c = tagC[i];
        Vector3d n = tagN[i], x = tagX[i];
        Vector3d look = parallel ? camDir : c - camAt;
        if (n * look > 0) n = -n;
        Vector3d y = Vector3d.CrossProduct(n, x);
        if (y * camUp < 0) { x = -x; y = -y; }
        foreach (Polyline pl in StrokeText(tagT[i], c, x, y, tagH[i])) args.Display.DrawPolyline(pl, ink, 2);
      }
    }
    catch { }
  }


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

  // values from the control window, used for any input left unwired here
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

  // inches to the nearest 1/16, e.g. 47 3/8
  private static string Frac16(double inches)
  {
    long s = (long)Math.Round(inches * 16.0);
    long whole = s / 16, rest = s % 16;
    if (rest == 0) return whole.ToString();
    long num = rest, den = 16;
    while (num % 2 == 0) { num /= 2; den /= 2; }
    return (whole > 0 ? whole + " " : "") + num + "/" + den;
  }

  // where two 2D lines (point + direction) cross
  private static Point2d Meet(Point2d p, Vector2d d, Point2d q, Vector2d e, Point2d fallback)
  {
    double den = d.X * e.Y - d.Y * e.X;
    if (Math.Abs(den) < 1e-12) return fallback;
    double t = ((q.X - p.X) * e.Y - (q.Y - p.Y) * e.X) / den;
    return new Point2d(p.X + d.X * t, p.Y + d.Y * t);
  }

  private class Blank
  {
    public string Name;
    public List<Point2d> Outer = new List<Point2d>();          // cut outline
    public List<Point2d[]> Inner = new List<Point2d[]>();      // bevel lines
    public List<string> EdgeIds = new List<string>();          // per side, "" for none
    public double W, H;                                          // after turning, in ft at cut size
    public Point3d At;                                           // lower-left corner once placed
    public bool Rotated90;
  }

  private void RunScript(
    object Plates, object Material, object Thickness, object StockWidth, object StockLength,
    object Scale, object LayoutGap, object LabelScale,
    ref object Panels, ref object Cut, ref object Bevel, ref object Sheets,
    ref object Engrave, ref object EdgeTable, ref object Report)
  {
    tagC = new List<Point3d>(); tagN = new List<Vector3d>(); tagX = new List<Vector3d>();
    tagH = new List<double>(); tagT = new List<string>(); tagBox = BoundingBox.Empty;
    if (!ToolOn("TimberOn")) { Report = "TIMBER is switched off in the control window (section 8)."; return; }
    win = null;
    try { win = AppDomain.CurrentDomain.GetData("FoldToolkit.Timber") as Dictionary<string, double>; } catch { }

    Mesh mesh = AsMesh(Plates);
    if (mesh == null || mesh.Faces.Count == 0) { Report = "Connect the Plates output of the Fold Designer to Plates."; return; }

    // ---------------- material ----------------
    string[] matName = { "plywood", "LVL panel", "CLT, 3 layers" };
    double[] matThk = { 0.75, 1.75, 4.125 };       // in
    double[] matW = { 48, 48, 120 };               // in
    double[] matL = { 96, 288, 480 };              // in
    double[] matPcf = { 34, 37, 30 };              // lb/ft3
    int mat = (int)Math.Round(Num(Material, W("TimMat", 0.0))); if (mat < 0 || mat > 2) mat = 0;
    double thkIn = Num(Thickness, W("TimThk", 0.0)); if (thkIn <= 0) thkIn = matThk[mat];
    double stockW = Num(StockWidth, W("TimStockW", 0.0)); if (stockW <= 0) stockW = matW[mat];
    double stockL = Num(StockLength, W("TimStockL", 0.0)); if (stockL <= 0) stockL = matL[mat];
    if (stockW > stockL) { double sw = stockW; stockW = stockL; stockL = sw; }
    double scale = Num(Scale, W("TimScale", 1.0)); if (scale < 1) scale = 1.0;
    double gap = Num(LayoutGap, 20.0); if (gap < 0) gap = 20.0;
    double lblScale = Num(LabelScale, W("TimLabel", 1.0)); if (lblScale < 0) lblScale = 0.0;

    double T = thkIn * scale / 12.0;          // panel thickness at building size, ft
    double half = T / 2.0;
    double toCut = 1.0 / scale;               // building ft -> ft as cut

    // ---------------- names, as in the formwork ----------------
    int[] tags = null;
    try { tags = AppDomain.CurrentDomain.GetData("FoldToolkit.FaceTags") as int[]; } catch { }
    int nF = mesh.Faces.Count;
    var name = new string[nF];
    if (tags != null && tags.Length == nF)
    {
      var seen = new Dictionary<int, int>();
      for (int f = 0; f < nF; f++)
      {
        int t = tags[f];
        string baseName = string.Format("S{0:00}-F{1:00}", t / 10000 + 1, t % 10000 + 1);
        if (mesh.Faces[f].IsQuad) name[f] = baseName;
        else
        {
          int n; seen.TryGetValue(t, out n); seen[t] = n + 1;
          name[f] = baseName + (n == 0 ? "a" : "b");
        }
      }
    }
    else for (int f = 0; f < nF; f++) name[f] = string.Format("P{0:000}", f + 1);

    // ---------------- normals, all on the upper side ----------------
    var fn = new Vector3d[nF];
    double zSum = 0.0;
    for (int f = 0; f < nF; f++)
    {
      MeshFace mf = mesh.Faces[f];
      Point3d a = mesh.Vertices.Point3dAt(mf.A), b = mesh.Vertices.Point3dAt(mf.B), c = mesh.Vertices.Point3dAt(mf.C);
      Vector3d n = mf.IsQuad ? Vector3d.CrossProduct(c - a, mesh.Vertices.Point3dAt(mf.D) - b) : Vector3d.CrossProduct(b - a, c - a);
      zSum += n.Z;
      n.Unitize();
      fn[f] = n;
    }
    if (zSum < 0) for (int f = 0; f < nF; f++) fn[f] = -fn[f];

    BoundingBox bb = mesh.GetBoundingBox(true);
    double ground = bb.Min.Z;
    double gTol = Math.Max(0.01, 1e-4 * bb.Diagonal.Length);

    // ---------------- edges: numbering and table ----------------
    var tvs = mesh.TopologyVertices;
    var tes = mesh.TopologyEdges;
    var edgeId = new Dictionary<int, string>();
    var table = new List<string>();
    table.Add("Edge,Panel A,Panel B,Joint,Length (in),Fold angle (deg),Bevel from square (deg),Long point");
    double bevMin = 999, bevMax = 0; int steepBevel = 0, sharpFold = 0, nJoints = 0;
    // order the edges by the first panel's name so the table reads in order
    var order = new List<int>();
    for (int e = 0; e < tes.Count; e++)
    {
      int[] fs = tes.GetConnectedFaces(e);
      if (fs != null && fs.Length == 2) order.Add(e);
    }
    order.Sort((p, q) =>
    {
      int[] fp = tes.GetConnectedFaces(p), fq = tes.GetConnectedFaces(q);
      string np = string.CompareOrdinal(name[fp[0]], name[fp[1]]) < 0 ? name[fp[0]] : name[fp[1]];
      string nq = string.CompareOrdinal(name[fq[0]], name[fq[1]]) < 0 ? name[fq[0]] : name[fq[1]];
      int r = string.CompareOrdinal(np, nq);
      return r != 0 ? r : p.CompareTo(q);
    });
    int en = 0;
    foreach (int e in order)
    {
      int[] fs = tes.GetConnectedFaces(e);
      int f1 = fs[0], f2 = fs[1];
      if (string.CompareOrdinal(name[f1], name[f2]) > 0) { int sw = f1; f1 = f2; f2 = sw; }
      Line ln = tes.EdgeLine(e);
      double defl = Vector3d.VectorAngle(fn[f1], fn[f2]) * 180.0 / Math.PI;   // how far the fold turns
      string id = string.Format("E{0:000}", ++en);
      edgeId[e] = id;
      nJoints++;
      string joint, longPt;
      if (defl < 0.5) { joint = "butt (flat)"; longPt = "-"; }
      else
      {
        // ridge or valley, seen from above
        MeshFace o = mesh.Faces[f2];
        int[] idx = o.IsQuad ? new int[] { o.A, o.B, o.C, o.D } : new int[] { o.A, o.B, o.C };
        double best = 0; Point3d far = ln.From;
        foreach (int v in idx) { Point3d q = mesh.Vertices.Point3dAt(v); double d = ln.DistanceTo(q, false); if (d > best) { best = d; far = q; } }
        bool ridge = fn[f1] * (far - ln.From) < 0;
        joint = ridge ? "mitre, ridge" : "mitre, valley";
        longPt = ridge ? "top face" : "underside";
      }
      double bevelDeg = defl / 2.0;
      if (defl >= 0.5) { bevMin = Math.Min(bevMin, bevelDeg); bevMax = Math.Max(bevMax, bevelDeg); }
      if (bevelDeg > 45.0) steepBevel++;
      if (180.0 - defl < 60.0) sharpFold++;
      table.Add(string.Format(System.Globalization.CultureInfo.InvariantCulture, "{0},{1},{2},{3},{4},{5:0.0},{6:0.0},{7}",
        id, name[f1], name[f2], joint, Frac16(ln.Length * toCut * 12.0), 180.0 - defl, bevelDeg, longPt));
    }

    // ---------------- panels ----------------
    var panels = new List<Brep>();
    var panelNames = new List<string>();
    var blanks = new List<Blank>();
    double areaMid = 0.0, volume = 0.0;
    int failed = 0;
    var shapes = new HashSet<string>();
    for (int f = 0; f < nF; f++)
    {
      MeshFace mf = mesh.Faces[f];
      int[] vi = mf.IsQuad ? new int[] { mf.A, mf.B, mf.C, mf.D } : new int[] { mf.A, mf.B, mf.C };
      int m = vi.Length;
      var P = new Point3d[m];
      for (int j = 0; j < m; j++) P[j] = mesh.Vertices.Point3dAt(vi[j]);
      Vector3d n = fn[f];

      // the cutting plane of each side j (from corner j to j+1)
      var side = new Plane[m];
      var sideEdge = new int[m];
      var sideKind = new string[m];
      for (int j = 0; j < m; j++)
      {
        Point3d a = P[j], b = P[(j + 1) % m];
        Vector3d d = b - a; d.Unitize();
        int ta = tvs.TopologyVertexIndex(vi[j]), tb = tvs.TopologyVertexIndex(vi[(j + 1) % m]);
        int e = tes.GetEdgeIndex(ta, tb);
        sideEdge[j] = e;
        int g = -1;
        if (e >= 0)
        {
          int[] fs = tes.GetConnectedFaces(e);
          if (fs != null && fs.Length == 2) g = fs[0] == f ? fs[1] : fs[0];
        }
        Vector3d nrm;
        if (g >= 0) { nrm = Vector3d.CrossProduct(d, n + fn[g]); sideKind[j] = "mitre"; }
        else if (a.Z < ground + gTol && b.Z < ground + gTol && Math.Abs(n.Z) < 0.95) { nrm = Vector3d.ZAxis; sideKind[j] = "ground"; }
        else { nrm = Vector3d.CrossProduct(d, n); sideKind[j] = "free"; }
        if (!nrm.Unitize()) nrm = Vector3d.CrossProduct(d, n);
        side[j] = new Plane(a, nrm);
      }
      Plane topPl = new Plane(P[0] + n * half, n), botPl = new Plane(P[0] - n * half, n);
      var Tp = new Point3d[m]; var Bp = new Point3d[m];
      for (int j = 0; j < m; j++)
      {
        Plane s0 = side[(j + m - 1) % m], s1 = side[j];
        Point3d hit;
        Tp[j] = Rhino.Geometry.Intersect.Intersection.PlanePlanePlane(s0, s1, topPl, out hit) && hit.DistanceTo(P[j]) < 20 * T + 1 ? hit : P[j] + n * half;
        Bp[j] = Rhino.Geometry.Intersect.Intersection.PlanePlanePlane(s0, s1, botPl, out hit) && hit.DistanceTo(P[j]) < 20 * T + 1 ? hit : P[j] - n * half;
      }

      // the solid
      var parts = new List<Brep>();
      double tol = Math.Max(1e-4, RhinoDoc.ActiveDoc != null ? RhinoDoc.ActiveDoc.ModelAbsoluteTolerance : 0.001);
      if (m == 4)
      {
        parts.Add(Brep.CreateFromCornerPoints(Tp[0], Tp[1], Tp[2], Tp[3], tol));
        parts.Add(Brep.CreateFromCornerPoints(Bp[3], Bp[2], Bp[1], Bp[0], tol));
      }
      else
      {
        parts.Add(Brep.CreateFromCornerPoints(Tp[0], Tp[1], Tp[2], tol));
        parts.Add(Brep.CreateFromCornerPoints(Bp[2], Bp[1], Bp[0], tol));
      }
      for (int j = 0; j < m; j++)
      {
        int k = (j + 1) % m;
        parts.Add(Brep.CreateFromCornerPoints(Tp[k], Tp[j], Bp[j], Bp[k], tol));
      }
      parts.RemoveAll(x => x == null);
      Brep[] joined = parts.Count > 0 ? Brep.JoinBreps(parts, tol) : null;
      if (joined != null && joined.Length == 1) { panels.Add(joined[0]); panelNames.Add(name[f]); }
      else { failed++; if (joined != null) foreach (Brep jb in joined) { panels.Add(jb); panelNames.Add(name[f]); } }

      // area and volume (mean of the two faces times thickness)
      double aMid = 0, aTop = 0, aBot = 0;
      for (int j = 1; j + 1 < m; j++)
      {
        aMid += 0.5 * Vector3d.CrossProduct(P[j] - P[0], P[j + 1] - P[0]).Length;
        aTop += 0.5 * Vector3d.CrossProduct(Tp[j] - Tp[0], Tp[j + 1] - Tp[0]).Length;
        aBot += 0.5 * Vector3d.CrossProduct(Bp[j] - Bp[0], Bp[j + 1] - Bp[0]).Length;
      }
      areaMid += aMid;
      volume += 0.5 * (aTop + aBot) * T;

      // label on the model, on the top face
      if (lblScale > 0)
      {
        Point3d cen = Point3d.Origin; foreach (Point3d q in Tp) cen += q; cen /= m;
        Vector3d xx = Vector3d.CrossProduct(Vector3d.ZAxis, n); if (!xx.Unitize()) xx = Vector3d.XAxis;
        double ht = Math.Min(0.25 * Math.Sqrt(aMid), 1.5) * lblScale;
        double wide = StrokeWidth(name[f], ht);
        double fitW = 0.8 * Math.Sqrt(aMid);
        if (wide > fitW) ht *= fitW / wide;
        Point3d at = cen + n * 0.02;
        tagC.Add(at); tagN.Add(n); tagX.Add(xx); tagH.Add(ht); tagT.Add(name[f]);
        tagBox.Union(at + new Vector3d(ht * 4, ht * 4, ht * 4)); tagBox.Union(at - new Vector3d(ht * 4, ht * 4, ht * 4));
      }

      // ---- the flat blank, in the plate's own frame, at cut size ----
      Vector3d ux = P[1] - P[0]; ux.Unitize();
      Vector3d uy = Vector3d.CrossProduct(n, ux); uy.Unitize();
      Func<Point3d, Point2d> flat = q => new Point2d((q - P[0]) * ux * toCut, (q - P[0]) * uy * toCut);
      var t2 = new Point2d[m]; var b2 = new Point2d[m]; var p2 = new Point2d[m];
      for (int j = 0; j < m; j++) { t2[j] = flat(Tp[j]); b2[j] = flat(Bp[j]); p2[j] = flat(P[j]); }
      double sgn = 0; for (int j = 0; j < m; j++) { Point2d a = p2[j], b = p2[(j + 1) % m]; sgn += a.X * b.Y - a.Y * b.X; }
      var lineP = new Point2d[m]; var lineD = new Vector2d[m];
      var bl = new Blank { Name = name[f] };
      var innerP = new Point2d[m];
      for (int j = 0; j < m; j++)
      {
        int k = (j + 1) % m;
        Vector2d d = new Vector2d(p2[k].X - p2[j].X, p2[k].Y - p2[j].Y); d.Unitize();
        Vector2d outN = sgn > 0 ? new Vector2d(d.Y, -d.X) : new Vector2d(-d.Y, d.X);
        double dt = outN.X * t2[j].X + outN.Y * t2[j].Y, db = outN.X * b2[j].X + outN.Y * b2[j].Y;
        lineP[j] = dt >= db ? t2[j] : b2[j];
        lineD[j] = d;
        if (Math.Abs(dt - db) > 1e-5)
        {
          Point2d ia = dt >= db ? b2[j] : t2[j], ib = dt >= db ? b2[k] : t2[k];
          bl.Inner.Add(new Point2d[] { ia, ib });
        }
        bl.EdgeIds.Add(sideEdge[j] >= 0 && edgeId.ContainsKey(sideEdge[j]) ? edgeId[sideEdge[j]] : "");
      }
      for (int j = 0; j < m; j++)
      {
        int jp = (j + m - 1) % m;
        bl.Outer.Add(Meet(lineP[jp], lineD[jp], lineP[j], lineD[j], t2[j]));
      }
      // turn it to its tightest rectangle, long side along x
      double bestA = double.MaxValue, bestAng = 0;
      for (int j = 0; j < m; j++)
      {
        Point2d a = bl.Outer[j], b = bl.Outer[(j + 1) % m];
        double ang = -Math.Atan2(b.Y - a.Y, b.X - a.X);
        double c = Math.Cos(ang), s = Math.Sin(ang);
        double x0 = double.MaxValue, x1 = double.MinValue, y0 = double.MaxValue, y1 = double.MinValue;
        foreach (Point2d q in bl.Outer) { double x = q.X * c - q.Y * s, y = q.X * s + q.Y * c; x0 = Math.Min(x0, x); x1 = Math.Max(x1, x); y0 = Math.Min(y0, y); y1 = Math.Max(y1, y); }
        double ar = (x1 - x0) * (y1 - y0);
        if (ar < bestA - 1e-12) { bestA = ar; bestAng = (x1 - x0) >= (y1 - y0) ? ang : ang + Math.PI / 2.0; }
      }
      {
        double c = Math.Cos(bestAng), s = Math.Sin(bestAng);
        Func<Point2d, Point2d> rot = q => new Point2d(q.X * c - q.Y * s, q.X * s + q.Y * c);
        for (int j = 0; j < bl.Outer.Count; j++) bl.Outer[j] = rot(bl.Outer[j]);
        for (int j = 0; j < bl.Inner.Count; j++) bl.Inner[j] = new Point2d[] { rot(bl.Inner[j][0]), rot(bl.Inner[j][1]) };
        double x0 = double.MaxValue, y0 = double.MaxValue, x1 = double.MinValue, y1 = double.MinValue;
        foreach (Point2d q in bl.Outer) { x0 = Math.Min(x0, q.X); y0 = Math.Min(y0, q.Y); x1 = Math.Max(x1, q.X); y1 = Math.Max(y1, q.Y); }
        for (int j = 0; j < bl.Outer.Count; j++) bl.Outer[j] = new Point2d(bl.Outer[j].X - x0, bl.Outer[j].Y - y0);
        for (int j = 0; j < bl.Inner.Count; j++) bl.Inner[j] = new Point2d[] { new Point2d(bl.Inner[j][0].X - x0, bl.Inner[j][0].Y - y0), new Point2d(bl.Inner[j][1].X - x0, bl.Inner[j][1].Y - y0) };
        bl.W = x1 - x0; bl.H = y1 - y0;
      }
      blanks.Add(bl);
      // same shape: same side lengths in order, to 1/8 in
      var sig = new StringBuilder();
      for (int j = 0; j < m; j++) sig.Append(Math.Round(bl.Outer[j].DistanceTo(bl.Outer[(j + 1) % m]) * 96.0)).Append('|');
      shapes.Add(sig.ToString());
    }

    // ---------------- nest the blanks on sheets (shelves) ----------------
    double sL = stockL / 12.0, sW = stockW / 12.0, kerf = 0.5 / 12.0;
    var order2 = new List<int>();
    for (int i = 0; i < blanks.Count; i++) order2.Add(i);
    order2.Sort((p, q) => blanks[q].H.CompareTo(blanks[p].H));
    var cut = new List<Curve>(); var bevel = new List<Curve>(); var sheets = new List<Curve>(); var engrave = new List<Curve>();
    Point3d origin = new Point3d(bb.Min.X, bb.Min.Y - gap, ground);
    int sheet = 0; double cx = kerf, cy = kerf, shelf = 0; int oversize = 0; double usedArea = 0;
    double sheetStep = sL + Math.Max(1.0, 0.1 * sL);
    var overs = new List<int>();
    foreach (int i in order2)
    {
      Blank bl = blanks[i];
      if (bl.W + 2 * kerf > sL + 1e-9 || bl.H + 2 * kerf > sW + 1e-9) { overs.Add(i); oversize++; continue; }
      if (cx + bl.W + kerf > sL) { cx = kerf; cy += shelf + kerf; shelf = 0; }
      if (cy + bl.H + kerf > sW) { sheet++; cx = kerf; cy = kerf; shelf = 0; }
      bl.At = origin + new Vector3d(sheet * sheetStep + cx, -(cy + bl.H), 0);   // sheets run along x, rows go toward -y
      cx += bl.W + kerf; shelf = Math.Max(shelf, bl.H);
      usedArea += bl.W * bl.H;
    }
    int sheetCount = blanks.Count - oversize > 0 ? sheet + 1 : 0;
    for (int s = 0; s < sheetCount; s++)
    {
      Point3d o = origin + new Vector3d(s * sheetStep, 0, 0);
      sheets.Add(new PolylineCurve(new Point3d[] { o, o + new Vector3d(sL, 0, 0), o + new Vector3d(sL, -sW, 0), o + new Vector3d(0, -sW, 0), o }));
    }
    // oversize blanks go in a row below the sheets
    double ox = 0;
    foreach (int i in overs)
    {
      Blank bl = blanks[i];
      bl.At = origin + new Vector3d(ox, -(sW + 2.0 + bl.H), 0);
      ox += bl.W + 1.0;
    }
    // draw them
    double cutLbl = 0.75 / 12.0 * lblScale;      // 3/4 in names on the blanks
    double edgeLbl = 0.375 / 12.0 * lblScale;    // 3/8 in edge numbers
    foreach (Blank bl in blanks)
    {
      Vector3d at = new Vector3d(bl.At);
      var pl = new Polyline();
      foreach (Point2d q in bl.Outer) pl.Add(new Point3d(q.X, q.Y, 0) + at);
      pl.Add(pl[0]);
      cut.Add(new PolylineCurve(pl));
      foreach (Point2d[] seg in bl.Inner) bevel.Add(new LineCurve(new Point3d(seg[0].X, seg[0].Y, 0) + at, new Point3d(seg[1].X, seg[1].Y, 0) + at));
      if (lblScale <= 0) continue;
      Point3d cen = Point3d.Origin; foreach (Point2d q in bl.Outer) cen += new Point3d(q.X, q.Y, 0); cen /= bl.Outer.Count;
      double ht = Math.Min(cutLbl, 0.2 * Math.Min(bl.W, bl.H));
      foreach (Polyline s in StrokeText(bl.Name, cen + at, Vector3d.XAxis, Vector3d.YAxis, ht)) engrave.Add(new PolylineCurve(s));
      int m = bl.Outer.Count;
      for (int j = 0; j < m; j++)
      {
        string id = bl.EdgeIds[j];
        if (id == "") continue;
        // side j of the blank runs between corners j and j+1
        Point2d a = bl.Outer[j], b = bl.Outer[(j + 1) % m];
        Vector3d along = new Vector3d(b.X - a.X, b.Y - a.Y, 0); double len = along.Length; if (len < 1e-9) continue;
        along /= len;
        Point3d mid = new Point3d((a.X + b.X) / 2, (a.Y + b.Y) / 2, 0) + at;
        Vector3d inward = cen + at - mid; inward -= along * (inward * along); if (!inward.Unitize()) continue;
        double eh = Math.Min(edgeLbl, 0.12 * len);
        Vector3d ex = along, ey = Vector3d.CrossProduct(Vector3d.ZAxis, ex);
        if (ey * inward < 0) { ex = -ex; ey = -ey; }       // letters stand on the edge, reading from inside
        foreach (Polyline s in StrokeText(id, mid + inward * (eh * 1.6 + 0.25 / 12.0), ex, ey, eh)) engrave.Add(new PolylineCurve(s));
      }
    }

    // ---------------- report ----------------
    double sheetArea = sheetCount * sL * sW;
    var sb = new StringBuilder();
    sb.AppendLine("TIMBER PANELS");
    sb.AppendLine(string.Format("{0}, {1} in thick | stock {2} x {3} in | {4}", matName[mat], Frac16(thkIn), Frac16(stockW), Frac16(stockL),
      scale > 1 ? "prototype at 1 : " + scale.ToString("0.##") : "full size"));
    sb.AppendLine();
    sb.AppendLine("PANELS");
    sb.AppendLine(string.Format("  Panels                     {0} ({1} different shapes)", nF, shapes.Count));
    sb.AppendLine(string.Format("  Joints between panels      {0}", nJoints));
    if (bevMax > 0) sb.AppendLine(string.Format("  Bevels from square         {0:0.0} to {1:0.0} deg", bevMin, bevMax));
    sb.AppendLine(string.Format("  Panel area (mid-surface)   {0:0} ft2 at building size", areaMid));
    sb.AppendLine(string.Format("  Timber volume              {0:0.0} ft3 at building size ({1:0.00} ft3 as cut)", volume, volume * toCut * toCut * toCut));
    sb.AppendLine(string.Format("  Weight                     {0:0} lb at building size ({1:0} lb as cut)", volume * matPcf[mat], volume * toCut * toCut * toCut * matPcf[mat]));
    sb.AppendLine();
    sb.AppendLine("STOCK");
    sb.AppendLine(string.Format("  Sheets needed              {0}", sheetCount));
    if (sheetCount > 0) sb.AppendLine(string.Format("  Sheet use                  {0:0}% of the sheet area is inside a blank's rectangle", 100.0 * usedArea / sheetArea));
    sb.AppendLine(string.Format("  Panels too big for stock   {0}", oversize));
    sb.AppendLine();
    sb.AppendLine("FLAGS");
    { string un = UnitNote(); if (un != null) sb.AppendLine("  - " + un); }
    if (tags == null || tags.Length != nF) sb.AppendLine("  - These plates are not straight from the Fold Designer, so panels are numbered P001, P002... instead of by station and fold.");
    if (oversize > 0) sb.AppendLine(string.Format("  - {0} panels are larger than the stock: they sit below the sheets. Use larger stock (LVL or CLT), a smaller fold width, or split them.", oversize));
    if (steepBevel > 0) sb.AppendLine(string.Format("  - {0} joints need a bevel over 45 deg from square: past the limit of a table or track saw. Cut them on the 5-axis or with the robot.", steepBevel));
    if (sharpFold > 0) sb.AppendLine(string.Format("  - {0} folds are sharper than 60 deg: the mitred long points are thin and fragile. Consider a butt joint with a fillet there.", sharpFold));
    if (failed > 0) sb.AppendLine(string.Format("  - {0} panels did not close into a single solid (very sharp or very small corners). Check them in Rhino.", failed));
    if (shapes.Count > 0.8 * nF && nF > 12) sb.AppendLine("  - Almost every panel is a different shape: CNC cutting from the drawings is the only practical route.");
    sb.AppendLine("  - Panels are centred on the plate mid-surface. Mitres halve each fold, so both panels at a joint get the same bevel.");
    sb.AppendLine("  - Cut is the outline to cut through; Bevel marks where each bevel ends on the face. Long point says which face is wider at the joint.");
    sb.AppendLine("  - Sheets and blanks are drawn at true cut size in feet: export with units (1 ft = 12 in). Kerf allowance 1/2 in between blanks.");
    sb.AppendLine("  - Joints are not designed here yet: splines, screws or finger joints come in the next build.");

    // ---------------- bake (button in the control window) ----------------
    bool bakeNow = false;
    try
    {
      object wb = AppDomain.CurrentDomain.GetData("FoldToolkit.TimberBake");
      if (wb is bool && (bool)wb) { bakeNow = true; AppDomain.CurrentDomain.SetData("FoldToolkit.TimberBake", false); }
    }
    catch { }
    if (bakeNow && RhinoDoc.ActiveDoc != null)
    {
      try
      {
        RhinoDoc doc = RhinoDoc.ActiveDoc;
        Rhino.DocObjects.RhinoObject[] old = doc.Objects.FindByUserString("FoldToolkitTimber", "*", true);
        if (old != null) foreach (Rhino.DocObjects.RhinoObject ro in old) doc.Objects.Delete(ro, true);
        int parent = doc.Layers.FindByFullPath("Timber", -1);
        if (parent < 0) parent = doc.Layers.Add(new Rhino.DocObjects.Layer { Name = "Timber" });
        Guid parentId = doc.Layers[parent].Id;
        string[] names = { "Panels", "Cut", "Bevel", "Sheets", "Engrave" };
        System.Drawing.Color[] cols = {
          System.Drawing.Color.FromArgb(205, 160, 100), System.Drawing.Color.Red, System.Drawing.Color.FromArgb(255, 140, 0),
          System.Drawing.Color.FromArgb(120, 120, 120), System.Drawing.Color.Blue };
        var atts = new Rhino.DocObjects.ObjectAttributes[names.Length];
        for (int L = 0; L < names.Length; L++)
        {
          int li = doc.Layers.FindByFullPath("Timber::" + names[L], -1);
          if (li < 0) li = doc.Layers.Add(new Rhino.DocObjects.Layer { Name = names[L], ParentLayerId = parentId, Color = cols[L] });
          atts[L] = new Rhino.DocObjects.ObjectAttributes();
          atts[L].LayerIndex = li;
          atts[L].SetUserString("FoldToolkitTimber", names[L]);
        }
        int baked = 0;
        for (int i = 0; i < panels.Count; i++)
        {
          Rhino.DocObjects.ObjectAttributes pa = atts[0].Duplicate();
          pa.Name = panelNames[i];
          doc.Objects.AddBrep(panels[i], pa); baked++;
        }
        foreach (Curve c in cut) { doc.Objects.AddCurve(c, atts[1]); baked++; }
        foreach (Curve c in bevel) { doc.Objects.AddCurve(c, atts[2]); baked++; }
        foreach (Curve c in sheets) { doc.Objects.AddCurve(c, atts[3]); baked++; }
        foreach (Curve c in engrave) { doc.Objects.AddCurve(c, atts[4]); baked++; }
        doc.Views.Redraw();
        sb.AppendLine(string.Format("  - Baked {0} objects on layer Timber: Panels (each named), Cut, Bevel, Sheets, Engrave. The timber baked before was replaced.", baked));
      }
      catch (Exception ex) { sb.AppendLine("  - Baking failed: " + ex.Message); }
    }

    Panels = panels;
    Cut = cut;
    Bevel = bevel;
    Sheets = sheets;
    Engrave = engrave;
    EdgeTable = table;
    Report = sb.ToString();
  }
}
