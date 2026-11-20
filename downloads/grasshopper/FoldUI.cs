// Folded Plate Toolkit - Control Window
// Rhino 8 Grasshopper C# Script component. Opens a floating window with every
// Fold Designer control, so students do not need to touch the Grasshopper canvas.
//
// Input:   Show (optional toggle; empty or True opens the window)
// Outputs: named exactly like the Fold Designer inputs. Wire each to the input
//          of the same name:
//   Path, Segments, Divisions, Profile, ProfileEnd, Bays, BayWidth, Depth,
//   Variation, Seed, Style, Amount, DepthAlong, WidthAlong, Flip, Thickness,
//   Path2, Fan, MomentDepth, FanRadius, Period, Lift
//
// The window opens when the Grasshopper file opens. Settings and picked curves
// are saved inside the Rhino file. The Report and the Bake button need the
// Fold Designer version dated 16 Nov or later.

#region Usings
using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.Text;
using Rhino;
using Rhino.Geometry;
using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using EF = Eto.Forms;
using ED = Eto.Drawing;
#endregion

public class Script_Instance : GH_ScriptInstance
{
  const string SECTION = "FoldToolkit";
  static readonly string Token = Guid.NewGuid().ToString();
  static Script_Instance current;
  static Dictionary<string, double> vals = new Dictionary<string, double>();
  static Dictionary<string, string> texts = new Dictionary<string, string>();
  static Dictionary<string, Guid> ids = new Dictionary<string, Guid>();
  static bool loaded = false, hooked = false, busy = false;
  static EF.Form form;
  static EF.TextArea reportBox;
  static string lastReport = null;

  // name, default
  static readonly string[] NumKeys = {
    "Segments", "Divisions", "Bays", "BayWidth", "Depth", "Variation", "Seed", "Style", "Amount",
    "Flip", "Thickness", "Fan", "MomentDepth", "FanRadius", "Period", "Lift", "ShowDims",
    "KitScale", "KitDirection", "KitTab", "KitBedW", "KitBedL", "KitLabel", "KitDash", "KitGap",
    "KitCut", "KitEtch", "KitFold", "KitOn", "FormOn", "ShowFolds", "RidgeCol", "ValleyCol",
    "TimberOn", "TimMat", "TimThk", "TimStockW", "TimStockL", "TimScale", "TimLabel",
    "StressOn", "StrLL", "StrFc", "StrRefine", "StrSpacing", "ShowTension", "ShowComp", "ShowFoldF", "ShowField", "ShowPlan", "StrPlan",
    "StrColT", "StrColC", "StrColF", "StrColS", "StrWT", "StrWC", "StrWF", "StrBandW", "StrHooks", "StrMaxSlope", "StrModOn", "StrModFold", "StrModWidth", "RevEnds", "StrSupMode", "StrPerLine" };
  static readonly double[] NumDefaults = {
    12, 4, 8, 10, 4, 0, 1, 0, 0.3,
    0, 5, 0, 0, 0, 1, 0, 1,
    48, 0, 0.25, 18, 32, 1, 0.125, 0.125,
    -65536, -16776961, -16776961, 1, 1, 1, -16744448, -16728065,
    1, 0, 0, 0, 0, 1, 1,
    0, 20, 4000, 4, 0, 1, 1, 1, 0, 1, 1,
    -2351586, -14790436, -15461356, -16777216, 2, 1, 1, 1, 1, 50, 1, 0, 2, 0, 0, 4.8 };   // colours are ARGB numbers
  static readonly string[] CurveKeys = { "Path", "Path2", "Profile", "ProfileEnd" };
  static readonly string[] TextKeys = { "DepthAlong", "WidthAlong" };

  // ---------- state ----------
  static void Defaults()
  {
    for (int i = 0; i < NumKeys.Length; i++) vals[NumKeys[i]] = NumDefaults[i];
    foreach (string k in TextKeys) texts[k] = "";
  }

  static void Load()
  {
    Defaults();
    RhinoDoc doc = RhinoDoc.ActiveDoc;
    if (doc == null) return;
    foreach (string k in NumKeys)
    {
      string s = doc.Strings.GetValue(SECTION, k);
      double v;
      if (!string.IsNullOrEmpty(s) && double.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out v)) vals[k] = v;
    }
    foreach (string k in TextKeys)
    {
      string s = doc.Strings.GetValue(SECTION, k);
      if (s != null) texts[k] = s;
    }
    foreach (string k in CurveKeys)
    {
      string s = doc.Strings.GetValue(SECTION, k);
      Guid g;
      if (!string.IsNullOrEmpty(s) && Guid.TryParse(s, out g)) ids[k] = g;
    }
  }

  static void Save()
  {
    RhinoDoc doc = RhinoDoc.ActiveDoc;
    if (doc == null) return;
    try
    {
      foreach (string k in NumKeys) doc.Strings.SetString(SECTION, k, vals[k].ToString(CultureInfo.InvariantCulture));
      foreach (string k in TextKeys) doc.Strings.SetString(SECTION, k, texts[k]);
      foreach (string k in CurveKeys) doc.Strings.SetString(SECTION, k, ids.ContainsKey(k) ? ids[k].ToString() : "");
    }
    catch { }
  }

  // ask Grasshopper to recompute with the new values
  static void Changed()
  {
    if (busy) return;
    Save();
    Script_Instance me = current;
    if (me == null) return;
    try
    {
      GH_Document gh = me.Component.OnPingDocument();
      if (gh != null) gh.ScheduleSolution(25, d => me.Component.ExpireSolution(false));
    }
    catch { }
  }

  static Curve PickedCurve(string key)
  {
    if (!ids.ContainsKey(key)) return null;
    RhinoDoc doc = RhinoDoc.ActiveDoc;
    if (doc == null) return null;
    Rhino.DocObjects.RhinoObject obj = doc.Objects.FindId(ids[key]);
    if (obj == null) return null;
    Curve c = obj.Geometry as Curve;
    return c == null ? null : c.DuplicateCurve();
  }

  static List<double> ParseList(string s)
  {
    var r = new List<double>();
    if (string.IsNullOrEmpty(s)) return r;
    foreach (string part in s.Split(new char[] { ',', ';', ' ', '\t', '\n', '\r' }, StringSplitOptions.RemoveEmptyEntries))
    {
      double v;
      if (double.TryParse(part, NumberStyles.Float, CultureInfo.InvariantCulture, out v)) r.Add(v);
    }
    return r;
  }

  // a picked curve was edited in Rhino: recompute
  static void OnReplace(object sender, Rhino.DocObjects.RhinoReplaceObjectEventArgs e)
  {
    try
    {
      foreach (Guid g in ids.Values) if (g == e.ObjectId) { Changed(); return; }
    }
    catch { }
  }

  // ---------- dimensions drawn over the model ----------
  // The Fold Designer publishes lines and labels; this component draws them.
  // Its preview must be on (it is by default).
  static bool ReadDims(out Line[] lines, out Point3d[] pts, out string[] texts)
  {
    lines = null; pts = null; texts = null;
    try
    {
      object[] d = AppDomain.CurrentDomain.GetData("FoldToolkit.Dims") as object[];
      if (d == null || d.Length < 3) return false;
      lines = d[0] as Line[]; pts = d[1] as Point3d[]; texts = d[2] as string[];
      return lines != null && pts != null && texts != null && pts.Length == texts.Length;
    }
    catch { return false; }
  }

  static bool ReadFolds(out Line[] ridges, out Line[] valleys)
  {
    ridges = null; valleys = null;
    try
    {
      object[] d = AppDomain.CurrentDomain.GetData("FoldToolkit.Folds") as object[];
      if (d == null || d.Length < 2) return false;
      ridges = d[0] as Line[]; valleys = d[1] as Line[];
      return ridges != null && valleys != null;
    }
    catch { return false; }
  }

  static System.Drawing.Color ArgbOf(string key)
  {
    return System.Drawing.Color.FromArgb(unchecked((int)(long)vals[key]));
  }

  static object[] ReadStress()
  {
    try
    {
      if (!vals.ContainsKey("StressOn") || vals["StressOn"] < 0.5) return null;
      object[] d = AppDomain.CurrentDomain.GetData("FoldToolkit.StressDraw") as object[];
      return d != null && d.Length >= 9 ? d : null;
    }
    catch { return null; }
  }

  static bool On(string key) { return vals.ContainsKey(key) && vals[key] > 0.5; }

  public override void DrawViewportMeshes(IGH_PreviewArgs args)
  {
    try
    {
      object[] sd = ReadStress();
      if (sd == null || !On("ShowField")) return;
      Mesh fm = sd[3] as Mesh;
      if (fm != null) args.Display.DrawMeshFalseColors(fm);
    }
    catch { }
  }

  public override BoundingBox ClippingBox
  {
    get
    {
      Line[] lines; Point3d[] pts; string[] texts;
      BoundingBox bb = BoundingBox.Empty;
      object[] sdb = ReadStress();
      if (sdb != null)
      {
        Mesh fm = sdb[3] as Mesh; if (fm != null) bb.Union(fm.GetBoundingBox(false));
        Line[] pe = sdb[7] as Line[]; if (pe != null) foreach (Line ln in pe) { bb.Union(ln.From); bb.Union(ln.To); }
      }
      Line[] rl, vl;
      if (vals.ContainsKey("ShowFolds") && vals["ShowFolds"] > 0.5 && ReadFolds(out rl, out vl))
      {
        foreach (Line ln in rl) { bb.Union(ln.From); bb.Union(ln.To); }
        foreach (Line ln in vl) { bb.Union(ln.From); bb.Union(ln.To); }
      }
      if (!vals.ContainsKey("ShowDims") || vals["ShowDims"] < 0.5) return bb;
      if (!ReadDims(out lines, out pts, out texts)) return bb;
      foreach (Line ln in lines) { bb.Union(ln.From); bb.Union(ln.To); }
      foreach (Point3d p in pts) bb.Union(p);
      return bb;
    }
  }

  public override void DrawViewportWires(IGH_PreviewArgs args)
  {
    try
    {
      object[] sd = ReadStress();
      if (sd != null)
      {
        // colours and weights from section 9
        System.Drawing.Color cT = ArgbOf("StrColT"), cC = ArgbOf("StrColC"), cF = ArgbOf("StrColF"), cS = ArgbOf("StrColS");
        double wT = Math.Max(1, vals["StrWT"]), wC = Math.Max(1, vals["StrWC"]), wF = Math.Max(1, vals["StrWF"]);
        double[] bf = On("StrBandW") ? new double[] { 0.6, 1.0, 1.8 } : new double[] { 1, 1, 1 };
        Func<double, int> Px = w => Math.Max(1, (int)Math.Round(w));
        var tb = sd[0] as List<Polyline>[]; var cb = sd[1] as List<Polyline>[];
        var pt = sd[5] as List<Polyline>[]; var pc = sd[6] as List<Polyline>[];
        for (int b = 0; b < 3; b++)
        {
          if (On("ShowTension")) foreach (Polyline pl in tb[b]) args.Display.DrawPolyline(pl, cT, Px(wT * bf[b]));
          if (On("ShowComp")) foreach (Polyline pl in cb[b]) args.Display.DrawPolyline(pl, cC, Px(wC * bf[b]));
          if (On("ShowPlan"))
          {
            if (On("ShowTension")) foreach (Polyline pl in pt[b]) args.Display.DrawPolyline(pl, cT, Px(wT * bf[b]));
            if (On("ShowComp")) foreach (Polyline pl in pc[b]) args.Display.DrawPolyline(pl, cC, Px(wC * bf[b]));
          }
        }
        if (On("ShowFoldF"))
        {
          var fl = sd[2] as Line[]; var fv = sd[4] as double[];
          for (int i = 0; i < fl.Length; i++)
            args.Display.DrawLine(fl[i], fv[i] >= 0 ? cT : cC, 2 + (int)Math.Round(5 * Math.Abs(fv[i])));
        }
        if (On("ShowPlan"))
        {
          var pe = sd[7] as Line[]; var ps = sd[8] as List<Polyline>;
          foreach (Line ln in pe) args.Display.DrawLine(ln, cF, Px(wF));
          if (sd.Length > 9) { var ph = sd[9] as Line[]; if (ph != null) foreach (Line ln in ph) args.Display.DrawLine(ln, cF, Px(wF + 1)); }
          if (sd.Length > 11 && On("ShowTension")) { var hk = sd[11] as List<Polyline>; if (hk != null) foreach (Polyline pl in hk) args.Display.DrawPolyline(pl, cT, Px(wT)); }
          foreach (Polyline pl in ps) args.Display.DrawPolygon(pl.ToArray(), cS, true);
        }
      }
    }
    catch { }
    try
    {
      Line[] rl, vl;
      if (vals.ContainsKey("ShowFolds") && vals["ShowFolds"] > 0.5 && ReadFolds(out rl, out vl))
      {
        System.Drawing.Color rc = ArgbOf("RidgeCol"), vc = ArgbOf("ValleyCol");
        foreach (Line ln in rl) args.Display.DrawLine(ln, rc, 3);
        foreach (Line ln in vl) args.Display.DrawLine(ln, vc, 3);
      }
    }
    catch { }
    try
    {
      if (!vals.ContainsKey("ShowDims") || vals["ShowDims"] < 0.5) return;
      Line[] lines; Point3d[] pts; string[] texts;
      if (!ReadDims(out lines, out pts, out texts)) return;
      System.Drawing.Color ink = System.Drawing.Color.FromArgb(30, 30, 30);
      foreach (Line ln in lines) args.Display.DrawLine(ln, ink, 2);
      for (int i = 0; i < pts.Length; i++)
        args.Display.DrawDot(pts[i], texts[i], System.Drawing.Color.White, System.Drawing.Color.Black);
    }
    catch { }
  }

  // ---------- window ----------
  static EF.Label Lbl(string t, int w)
  {
    return new EF.Label { Text = t, Width = w, VerticalAlignment = EF.VerticalAlignment.Center };
  }

  // returns the row's controls so a caller can grey the row out when it has no effect
  static EF.Control[] NumberRow(EF.DynamicLayout lay, string key, string label, double min, double max, int decimals, string unit)
  {
    double scale = Math.Pow(10, decimals);
    var slider = new EF.Slider { MinValue = 0, MaxValue = (int)Math.Round((max - min) * scale), Width = 150 };
    var step = new EF.NumericStepper { MinValue = min, MaxValue = max, DecimalPlaces = decimals, Increment = 1.0 / scale, Width = 70 };
    double v = Math.Max(min, Math.Min(max, vals[key]));
    slider.Value = (int)Math.Round((v - min) * scale);
    step.Value = v;
    slider.ValueChanged += (s, e) =>
    {
      if (busy) return;
      double nv = min + slider.Value / scale;
      busy = true; step.Value = nv; busy = false;
      vals[key] = nv; Changed();
    };
    step.ValueChanged += (s, e) =>
    {
      if (busy) return;
      double nv = step.Value;
      busy = true; slider.Value = (int)Math.Round((nv - min) * scale); busy = false;
      vals[key] = nv; Changed();
    };
    EF.Label name = Lbl(label, 96);
    lay.AddRow(name, slider, step, Lbl(unit, 28));
    return new EF.Control[] { name, slider, step };
  }

  static void SetEnabled(EF.Control[] row, bool on)
  {
    foreach (EF.Control c in row) c.Enabled = on;
  }

  static void CheckRow(EF.DynamicLayout lay, string key, string label)
  {
    var cb = new EF.CheckBox { Text = label, Checked = vals[key] > 0.5 };
    cb.CheckedChanged += (s, e) => { vals[key] = (cb.Checked == true) ? 1 : 0; Changed(); };
    lay.AddRow(cb);
  }

  // a colour swatch; the colour is kept as one ARGB number
  static EF.Control[] ColorRow(EF.DynamicLayout lay, string key, string label)
  {
    int argb = unchecked((int)(long)vals[key]);
    var cp = new EF.ColorPicker { Value = ED.Color.FromArgb((argb >> 16) & 255, (argb >> 8) & 255, argb & 255) };
    cp.ValueChanged += (s, e) =>
    {
      ED.Color c = cp.Value;
      vals[key] = unchecked((int)(0xFF000000u | ((uint)c.Rb << 16) | ((uint)c.Gb << 8) | (uint)c.Bb));
      Changed();
    };
    EF.Label name = Lbl(label, 96);
    lay.AddRow(name, cp, null);
    return new EF.Control[] { name, cp };
  }

  static void TextRow(EF.DynamicLayout lay, string key, string label, string hint)
  {
    var tb = new EF.TextBox { Text = texts[key], PlaceholderText = hint };
    tb.TextChanged += (s, e) => { texts[key] = tb.Text ?? ""; Changed(); };
    lay.AddRow(Lbl(label, 96), tb);
  }

  static void CurveRow(EF.DynamicLayout lay, string key, string label)
  {
    var state = Lbl(PickedCurve(key) != null ? "set" : "none", 44);
    var pick = new EF.Button { Text = "Pick", Width = 60 };
    var clear = new EF.Button { Text = "Clear", Width = 60 };
    pick.Click += (s, e) =>
    {
      try
      {
        Rhino.DocObjects.ObjRef oref;
        Rhino.Commands.Result rc = Rhino.Input.RhinoGet.GetOneObject("Select the " + label + " curve", false,
          Rhino.DocObjects.ObjectType.Curve, out oref);
        if (rc == Rhino.Commands.Result.Success && oref != null)
        {
          ids[key] = oref.ObjectId;
          state.Text = "set";
          RhinoDoc doc = RhinoDoc.ActiveDoc;
          if (doc != null) { doc.Objects.UnselectAll(); doc.Views.Redraw(); }
          Changed();
        }
      }
      catch { }
    };
    clear.Click += (s, e) => { ids.Remove(key); state.Text = "none"; Changed(); };
    lay.AddRow(Lbl(label, 96), pick, clear, state, null);
  }

  static EF.GroupBox Group(string title, EF.DynamicLayout inner)
  {
    inner.Padding = new ED.Padding(8);
    inner.Spacing = new ED.Size(6, 5);
    return new EF.GroupBox { Text = title, Content = inner };
  }

  static void BuildForm()
  {
    var root = new EF.DynamicLayout { Padding = new ED.Padding(10), Spacing = new ED.Size(6, 8) };

    var g1 = new EF.DynamicLayout();
    CurveRow(g1, "Path", "Path");
    CurveRow(g1, "Path2", "Second path");
    CurveRow(g1, "Profile", "Section");
    CurveRow(g1, "ProfileEnd", "End section");
    root.AddRow(Group("1  Curves drawn in Rhino", g1));

    var g2 = new EF.DynamicLayout();
    NumberRow(g2, "Bays", "Folds", 1, 40, 0, "");
    NumberRow(g2, "BayWidth", "Fold width", 2, 40, 1, "ft");
    NumberRow(g2, "Depth", "Fold depth", 0.5, 30, 1, "ft");
    NumberRow(g2, "Variation", "Variation", 0, 1, 2, "");
    NumberRow(g2, "Seed", "Seed", 1, 50, 0, "");
    root.AddRow(Group("2  Section", g2));

    var g3 = new EF.DynamicLayout();
    var style = new EF.DropDown();
    style.Items.Add("Parallel folds");
    style.Items.Add("Counter-running folds");
    style.Items.Add("Zigzag folds");
    style.SelectedIndex = Math.Max(0, Math.Min(2, (int)Math.Round(vals["Style"])));
    g3.AddRow(Lbl("Style", 96), style);
    var endsDd = new EF.DropDown();
    endsDd.Items.Add("Ridge to valley (a ridge ends as a valley)");
    endsDd.Items.Add("Ridge to ridge (turns and comes back)");
    endsDd.SelectedIndex = vals["RevEnds"] > 0.5 ? 1 : 0;
    EF.Label endsLbl = Lbl("Reversal", 96);
    g3.AddRow(endsLbl, endsDd);
    endsDd.SelectedIndexChanged += (s, e) => { vals["RevEnds"] = endsDd.SelectedIndex; Changed(); };
    EF.Control[] rowEnds = new EF.Control[] { endsLbl, endsDd };
    EF.Control[] rowAmount = NumberRow(g3, "Amount", "Style amount", 0, 1, 2, "");
    EF.Control[] rowPeriod = NumberRow(g3, "Period", "Reverse every", 1, 12, 0, "stn");
    EF.Control[] rowLift = NumberRow(g3, "Lift", "Apex lift", -30, 30, 1, "ft");
    // these three only act on counter-running and zigzag folds
    bool styled = style.SelectedIndex != 0;
    SetEnabled(rowAmount, styled); SetEnabled(rowPeriod, styled); SetEnabled(rowLift, styled); SetEnabled(rowEnds, styled);
    style.SelectedIndexChanged += (s, e) =>
    {
      vals["Style"] = style.SelectedIndex;
      bool on = style.SelectedIndex != 0;
      // a style with no amount does nothing, so give it a starting value
      // (counter-running needs 1 for a ridge to turn fully into a valley)
      if (on && (vals["Amount"] < 1e-6 || (style.SelectedIndex == 1 && vals["Amount"] < 0.999)) && Math.Abs(vals["Lift"]) < 1e-9)
      {
        double av = style.SelectedIndex == 1 ? 1.0 : 0.5;
        vals["Amount"] = av;
        busy = true;
        foreach (EF.Control c in rowAmount) { var ns = c as EF.NumericStepper; if (ns != null) ns.Value = av; var sl = c as EF.Slider; if (sl != null) sl.Value = (int)Math.Round(av * 100); }
        busy = false;
      }
      SetEnabled(rowAmount, on); SetEnabled(rowPeriod, on); SetEnabled(rowLift, on); SetEnabled(rowEnds, on);
      Changed();
    };
    NumberRow(g3, "Segments", "Path pieces", 2, 40, 0, "");
    NumberRow(g3, "Divisions", "Stations each", 1, 12, 0, "");
    TextRow(g3, "DepthAlong", "Depth along", "e.g. 1, 0.3, 1");
    TextRow(g3, "WidthAlong", "Width along", "e.g. 1, 0.6");
    CheckRow(g3, "MomentDepth", "Fold depth follows bending");
    root.AddRow(Group("3  Along the path", g3));

    var g4 = new EF.DynamicLayout();
    EF.Control[] rowFan = NumberRow(g4, "Fan", "Fan angle", 0, 360, 0, "deg");
    EF.Control[] rowFanR = NumberRow(g4, "FanRadius", "Fan radius", 0, 100, 1, "ft");
    // the radius only matters once the fan is on
    SetEnabled(rowFanR, vals["Fan"] > 0.5);
    ((EF.Slider)rowFan[1]).ValueChanged += (s, e) => SetEnabled(rowFanR, vals["Fan"] > 0.5);
    ((EF.NumericStepper)rowFan[2]).ValueChanged += (s, e) => SetEnabled(rowFanR, vals["Fan"] > 0.5);
    root.AddRow(Group("4  Fan (0 = off; radius 0 = automatic)", g4));

    var g5 = new EF.DynamicLayout();
    NumberRow(g5, "Thickness", "Slab", 2, 24, 1, "in");
    CheckRow(g5, "Flip", "Flip ridges and valleys");
    CheckRow(g5, "ShowDims", "Show dimensions on the model");
    CheckRow(g5, "ShowFolds", "Show ridges and valleys on the model");
    ColorRow(g5, "RidgeCol", "Ridge colour");
    ColorRow(g5, "ValleyCol", "Valley colour");
    root.AddRow(Group("5  Build", g5));

    var g6 = new EF.DynamicLayout();
    var kitOn = new EF.CheckBox { Text = "Make the model kit (off = faster while designing)", Checked = vals["KitOn"] > 0.5 };
    g6.AddRow(kitOn);
    var kitRows = new List<EF.Control>();
    var kitDir = new EF.DropDown();
    kitDir.Items.Add("Bands across the folds");
    kitDir.Items.Add("Strips along the folds");
    kitDir.SelectedIndex = vals["KitDirection"] > 0.5 ? 1 : 0;
    kitDir.SelectedIndexChanged += (s, e) => { vals["KitDirection"] = kitDir.SelectedIndex; Changed(); };
    EF.Label kitDirLbl = Lbl("Pieces", 96);
    g6.AddRow(kitDirLbl, kitDir);
    kitRows.Add(kitDirLbl); kitRows.Add(kitDir);
    kitRows.AddRange(NumberRow(g6, "KitScale", "Scale 1 :", 12, 200, 0, ""));
    kitRows.AddRange(NumberRow(g6, "KitTab", "Tab width", 0, 1, 2, "in"));
    kitRows.AddRange(NumberRow(g6, "KitDash", "Dash", 0.02, 1, 2, "in"));
    kitRows.AddRange(NumberRow(g6, "KitGap", "Gap", 0, 1, 2, "in"));
    kitRows.AddRange(NumberRow(g6, "KitLabel", "Label size", 0, 3, 1, "x"));
    kitRows.AddRange(NumberRow(g6, "KitBedW", "Bed width", 8, 60, 0, "in"));
    kitRows.AddRange(NumberRow(g6, "KitBedL", "Bed length", 8, 100, 0, "in"));
    kitRows.AddRange(ColorRow(g6, "KitCut", "Cut colour"));
    kitRows.AddRange(ColorRow(g6, "KitEtch", "Label colour"));
    kitRows.AddRange(ColorRow(g6, "KitFold", "Fold colour"));
    var bakeKit = new EF.Button { Text = "Bake model kit into Rhino" };
    bakeKit.Click += (s, e) =>
    {
      try { AppDomain.CurrentDomain.SetData("FoldToolkit.KitBake", true); } catch { }
      Changed();
    };
    g6.AddRow(bakeKit);
    kitRows.Add(bakeKit);
    SetEnabled(kitRows.ToArray(), vals["KitOn"] > 0.5);
    kitOn.CheckedChanged += (s, e) =>
    {
      vals["KitOn"] = (kitOn.Checked == true) ? 1 : 0;
      SetEnabled(kitRows.ToArray(), vals["KitOn"] > 0.5);
      Changed();
    };
    root.AddRow(Group("6  Model kit for the laser (gap 0 = solid lines)", g6));

    var g7 = new EF.DynamicLayout();
    var formOn = new EF.CheckBox { Text = "Make formwork drawings (off = faster while designing)", Checked = vals["FormOn"] > 0.5 };
    g7.AddRow(formOn);
    var bakeForm = new EF.Button { Text = "Bake formwork into Rhino" };
    bakeForm.Enabled = vals["FormOn"] > 0.5;
    bakeForm.Click += (s, e) =>
    {
      try { AppDomain.CurrentDomain.SetData("FoldToolkit.FormBake", true); } catch { }
      Changed();
    };
    g7.AddRow(bakeForm);
    formOn.CheckedChanged += (s, e) =>
    {
      vals["FormOn"] = (formOn.Checked == true) ? 1 : 0;
      bakeForm.Enabled = vals["FormOn"] > 0.5;
      Changed();
    };
    root.AddRow(Group("7  Formwork for casting in place", g7));

    var g8 = new EF.DynamicLayout();
    var timOn = new EF.CheckBox { Text = "Make timber panels (off = faster while designing)", Checked = vals["TimberOn"] > 0.5 };
    g8.AddRow(timOn);
    var timRows = new List<EF.Control>();
    var timMat = new EF.DropDown();
    timMat.Items.Add("Plywood (3/4 in, 4 x 8 ft sheets)");
    timMat.Items.Add("LVL panel (1 3/4 in, 4 x 24 ft)");
    timMat.Items.Add("CLT, 3 layers (4 1/8 in, 10 x 40 ft)");
    timMat.SelectedIndex = Math.Max(0, Math.Min(2, (int)Math.Round(vals["TimMat"])));
    timMat.SelectedIndexChanged += (s, e) => { vals["TimMat"] = timMat.SelectedIndex; Changed(); };
    EF.Label timMatLbl = Lbl("Material", 96);
    g8.AddRow(timMatLbl, timMat);
    timRows.Add(timMatLbl); timRows.Add(timMat);
    g8.AddRow(Lbl("0 below = use the material's standard size", 300));
    timRows.AddRange(NumberRow(g8, "TimThk", "Thickness", 0, 12, 3, "in"));
    timRows.AddRange(NumberRow(g8, "TimStockW", "Stock width", 0, 240, 0, "in"));
    timRows.AddRange(NumberRow(g8, "TimStockL", "Stock length", 0, 600, 0, "in"));
    timRows.AddRange(NumberRow(g8, "TimScale", "Scale 1 :", 1, 20, 0, ""));
    timRows.AddRange(NumberRow(g8, "TimLabel", "Label size", 0, 3, 1, "x"));
    var bakeTim = new EF.Button { Text = "Bake timber into Rhino" };
    bakeTim.Click += (s, e) =>
    {
      try { AppDomain.CurrentDomain.SetData("FoldToolkit.TimberBake", true); } catch { }
      Changed();
    };
    g8.AddRow(bakeTim);
    timRows.Add(bakeTim);
    SetEnabled(timRows.ToArray(), vals["TimberOn"] > 0.5);
    timOn.CheckedChanged += (s, e) =>
    {
      vals["TimberOn"] = (timOn.Checked == true) ? 1 : 0;
      SetEnabled(timRows.ToArray(), vals["TimberOn"] > 0.5);
      Changed();
    };
    root.AddRow(Group("8  Timber panels (scale 1 = full size)", g8));

    var g9 = new EF.DynamicLayout();
    var strOn = new EF.CheckBox { Text = "Analyse tension and compression (slower; off while shaping)", Checked = vals["StressOn"] > 0.5 };
    g9.AddRow(strOn);
    var strRows = new List<EF.Control>();
    strRows.AddRange(NumberRow(g9, "StrLL", "Live load", 0, 100, 0, "psf"));
    strRows.AddRange(NumberRow(g9, "StrFc", "Concrete f'c", 2500, 8000, 0, "psi"));
    strRows.AddRange(NumberRow(g9, "StrRefine", "Refine", 2, 6, 0, ""));
    strRows.AddRange(NumberRow(g9, "StrSpacing", "Line spacing", 0, 10, 1, "ft"));
    strRows.AddRange(NumberRow(g9, "StrPerLine", "Kips per line", 0, 20, 1, "k"));
    {
      var supDd = new EF.DropDown();
      supDd.Items.Add("Automatic: ground pinned, column points on bearings");
      supDd.Items.Add("Pinned: held in every direction (footings take thrust)");
      supDd.Items.Add("Bearings: held up, held sideways only enough to stay put");
      supDd.SelectedIndex = Math.Max(0, Math.Min(2, (int)Math.Round(vals["StrSupMode"])));
      supDd.SelectedIndexChanged += (s, e) => { vals["StrSupMode"] = supDd.SelectedIndex; Changed(); };
      EF.Label supLbl = Lbl("Supports", 96);
      g9.AddRow(supLbl, supDd);
      strRows.Add(supLbl); strRows.Add(supDd);
    }
    foreach (string[] cbk in new string[][] {
      new[] { "ShowTension", "Tension lines (red)" }, new[] { "ShowComp", "Compression lines (blue)" },
      new[] { "ShowFoldF", "Forces along the folds" }, new[] { "ShowField", "Colour the plates by stress" },
      new[] { "ShowPlan", "Plan diagram beside the model" } })
    {
      string key = cbk[0];
      var cbx = new EF.CheckBox { Text = cbk[1], Checked = vals[key] > 0.5 };
      cbx.CheckedChanged += (s, e) => { vals[key] = (cbx.Checked == true) ? 1 : 0; Changed(); };
      g9.AddRow(cbx);
      strRows.Add(cbx);
    }
    var planDd = new EF.DropDown();
    planDd.Items.Add("Plan: projected, every plate edge");
    planDd.Items.Add("Plan: diagram (folds, outline, supports)");
    planDd.Items.Add("Plan: roof plates only (steep side plates left out)");
    planDd.SelectedIndex = Math.Max(0, Math.Min(2, (int)Math.Round(vals["StrPlan"])));
    planDd.SelectedIndexChanged += (s, e) => { vals["StrPlan"] = planDd.SelectedIndex; Changed(); };
    EF.Label planLbl = Lbl("Diagram", 96);
    g9.AddRow(planLbl, planDd);
    strRows.Add(planLbl); strRows.Add(planDd);
    strRows.AddRange(NumberRow(g9, "StrMaxSlope", "Roof plates up to", 10, 89, 0, "deg slope"));
    {
      var modCb = new EF.CheckBox { Text = "One module unrolled at true shape, beside the plan", Checked = vals["StrModOn"] > 0.5 };
      modCb.CheckedChanged += (s, e) => { vals["StrModOn"] = (modCb.Checked == true) ? 1 : 0; Changed(); };
      g9.AddRow(modCb);
      strRows.Add(modCb);
    }
    strRows.AddRange(NumberRow(g9, "StrModFold", "Module from strip", 0, 60, 0, "(0 = middle)"));
    strRows.AddRange(NumberRow(g9, "StrModWidth", "Module strips", 1, 12, 0, ""));
    g9.AddRow(Lbl("Drawing", 300));
    strRows.AddRange(ColorRow(g9, "StrColT", "Tension colour"));
    strRows.AddRange(NumberRow(g9, "StrWT", "Tension weight", 1, 8, 0, "px"));
    strRows.AddRange(ColorRow(g9, "StrColC", "Compression colour"));
    strRows.AddRange(NumberRow(g9, "StrWC", "Compr. weight", 1, 8, 0, "px"));
    strRows.AddRange(ColorRow(g9, "StrColF", "Fold line colour"));
    strRows.AddRange(NumberRow(g9, "StrWF", "Fold weight", 1, 8, 0, "px"));
    strRows.AddRange(ColorRow(g9, "StrColS", "Support colour"));
    foreach (string[] cbk in new string[][] {
      new[] { "StrBandW", "Heavier lines where the stress is higher" }, new[] { "StrHooks", "Bar hooks at the ends of tension lines" } })
    {
      string key = cbk[0];
      var cbx = new EF.CheckBox { Text = cbk[1], Checked = vals[key] > 0.5 };
      cbx.CheckedChanged += (s, e) => { vals[key] = (cbx.Checked == true) ? 1 : 0; Changed(); };
      g9.AddRow(cbx);
      strRows.Add(cbx);
    }
    var bakeStr = new EF.Button { Text = "Bake structure diagram into Rhino" };
    bakeStr.Click += (s, e) =>
    {
      try { AppDomain.CurrentDomain.SetData("FoldToolkit.StressBake", true); } catch { }
      Changed();
    };
    g9.AddRow(bakeStr);
    strRows.Add(bakeStr);
    SetEnabled(strRows.ToArray(), vals["StressOn"] > 0.5);
    strOn.CheckedChanged += (s, e) =>
    {
      vals["StressOn"] = (strOn.Checked == true) ? 1 : 0;
      SetEnabled(strRows.ToArray(), vals["StressOn"] > 0.5);
      Changed();
    };
    root.AddRow(Group("9  Structure: tension and compression (line spacing 0 = automatic)", g9));

    reportBox = new EF.TextArea { ReadOnly = true, Wrap = false, Height = 190, Font = ED.Fonts.Monospace(8.5f) };
    reportBox.Text = "The report appears here once the Fold Designer has run.";
    root.AddRow(reportBox);

    var bake = new EF.Button { Text = "Bake solid into Rhino" };
    bake.Click += (s, e) =>
    {
      try
      {
        Mesh m = AppDomain.CurrentDomain.GetData("FoldToolkit.Solid") as Mesh;
        RhinoDoc doc = RhinoDoc.ActiveDoc;
        if (m != null && doc != null) { doc.Objects.AddMesh(m); doc.Views.Redraw(); }
      }
      catch { }
    };
    var reset = new EF.Button { Text = "Reset values" };
    reset.Click += (s, e) =>
    {
      Defaults();
      EF.Form old = form; form = null;
      try { if (old != null) old.Close(); } catch { }
      Changed();
      OpenForm();
    };
    var buttons = new EF.DynamicLayout { Spacing = new ED.Size(6, 6) };
    buttons.AddRow(bake, reset);
    root.AddRow(buttons);
    root.AddRow(null);

    form = new EF.Form
    {
      Title = "Folded Plate Designer",
      ClientSize = new ED.Size(420, 860),
      Resizable = true,
      Tag = Token,
      Content = new EF.Scrollable { Content = root, Border = EF.BorderType.None, ExpandContentWidth = true }
    };
    try { form.Owner = Rhino.UI.RhinoEtoApp.MainWindow; } catch { }
    form.Closed += (s, e) => { if (ReferenceEquals(form, s)) form = null; };

    var timer = new EF.UITimer { Interval = 0.5 };
    timer.Elapsed += (s, e) =>
    {
      try
      {
        if (form == null) { timer.Stop(); return; }
        string t = AppDomain.CurrentDomain.GetData("FoldToolkit.Report") as string;
        if (t != null && t != lastReport && reportBox != null) { lastReport = t; reportBox.Text = t; }
      }
      catch { }
    };
    timer.Start();
  }

  static void OpenForm()
  {
    // close a window left over from an earlier version of this script
    try
    {
      EF.Form other = AppDomain.CurrentDomain.GetData("FoldToolkit.Form") as EF.Form;
      if (other != null && !ReferenceEquals(other, form)) { try { other.Close(); } catch { } }
    }
    catch { }
    if (form == null)
    {
      lastReport = null;
      BuildForm();
      AppDomain.CurrentDomain.SetData("FoldToolkit.Form", form);
      form.Show();
    }
  }

  private void RunScript(
    object Show,
    ref object Path, ref object Segments, ref object Divisions, ref object Profile, ref object ProfileEnd,
    ref object Bays, ref object BayWidth, ref object Depth, ref object Variation, ref object Seed,
    ref object Style, ref object Amount, ref object DepthAlong, ref object WidthAlong,
    ref object Flip, ref object Thickness, ref object Path2, ref object Fan,
    ref object MomentDepth, ref object FanRadius, ref object Period, ref object Lift)
  {
    current = this;
    if (!loaded) { Load(); loaded = true; }
    if (!hooked) { RhinoDoc.ReplaceRhinoObject += OnReplace; hooked = true; }

    bool show = true;
    if (Show is bool) show = (bool)Show;
    else if (Show is GH_Boolean) show = ((GH_Boolean)Show).Value;
    if (show) { try { OpenForm(); } catch (Exception ex) { Print("Could not open the window: " + ex.Message); } }

    // model kit settings travel to the MODELKIT component without wires
    try
    {
      var kitVals = new Dictionary<string, double>();
      foreach (string k in NumKeys) if (k.StartsWith("Kit")) kitVals[k] = vals[k];
      AppDomain.CurrentDomain.SetData("FoldToolkit.Kit", kitVals);
      var tools = new Dictionary<string, double>();
      tools["KitOn"] = vals["KitOn"]; tools["FormOn"] = vals["FormOn"]; tools["TimberOn"] = vals["TimberOn"]; tools["StressOn"] = vals["StressOn"]; tools["RevEnds"] = vals["RevEnds"];
      var strVals = new Dictionary<string, double>();
      foreach (string k in NumKeys) if (k.StartsWith("Str")) strVals[k] = vals[k];
      AppDomain.CurrentDomain.SetData("FoldToolkit.Stress", strVals);
      var timVals = new Dictionary<string, double>();
      foreach (string k in NumKeys) if (k.StartsWith("Tim")) timVals[k] = vals[k];
      AppDomain.CurrentDomain.SetData("FoldToolkit.Timber", timVals);
      AppDomain.CurrentDomain.SetData("FoldToolkit.Tools", tools);
      try { AppDomain.CurrentDomain.SetData("FoldToolkit.WindowId", Component.InstanceGuid); } catch { }   // so others can tell the window is really on the canvas
    }
    catch { }

    Path = PickedCurve("Path");
    Path2 = PickedCurve("Path2");
    Profile = PickedCurve("Profile");
    ProfileEnd = PickedCurve("ProfileEnd");
    Segments = vals["Segments"];
    Divisions = vals["Divisions"];
    Bays = vals["Bays"];
    BayWidth = vals["BayWidth"];
    Depth = vals["Depth"];
    Variation = vals["Variation"];
    Seed = vals["Seed"];
    Style = vals["Style"] + (vals["Style"] > 0.5 && vals["RevEnds"] > 0.5 ? 10 : 0);   // tens digit: ridge to ridge
    Amount = vals["Amount"];
    List<double> da = ParseList(texts["DepthAlong"]);
    List<double> wa = ParseList(texts["WidthAlong"]);
    DepthAlong = da.Count > 0 ? da : null;
    WidthAlong = wa.Count > 0 ? wa : null;
    Flip = vals["Flip"] > 0.5;
    Thickness = vals["Thickness"];
    Fan = vals["Fan"];
    MomentDepth = vals["MomentDepth"] > 0.5;
    FanRadius = vals["FanRadius"];
    Period = vals["Period"];
    Lift = vals["Lift"];
  }
}
