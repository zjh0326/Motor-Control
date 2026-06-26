using UnityEngine;
using UnityEngine.UI;
using TMPro;
using System.Collections.Generic;

public class RealtimeGraph : MonoBehaviour
{
    [SerializeField] private RawImage graphImage;

    [Header("Theme")]
    [SerializeField] private Color bgColor = new Color(0.92f, 0.92f, 0.92f);
    [SerializeField] private Color gridColor = new Color(0.75f, 0.75f, 0.75f);
    [SerializeField] private Color axisColor = new Color(0.4f, 0.4f, 0.4f);

    [Header("Line Colors")]
    [SerializeField] private Color posColor = new Color(0f, 0.5f, 0.2f);
    [SerializeField] private Color spdColor = new Color(0f, 0.3f, 0.65f);
    [SerializeField] private Color curColor = new Color(0.7f, 0.1f, 0.1f);

    [Header("Y-axis Ranges")]
    [SerializeField] private float posRange = 6f;
    [SerializeField] private float spdRange = 8f;
    [SerializeField] private float curRange = 10f;

    [Header("Y-axis Tick")]
    [SerializeField] private float yTickStep = 2.5f;

    [Header("Time")]
    [SerializeField] private float timeWindow = 6f;

    private MotorControllerUI motorUI;
    private Texture2D tex;

    private List<float> times = new List<float>();
    private List<float> posData = new List<float>();
    private List<float> spdData = new List<float>();
    private List<float> curData = new List<float>();

    private float graphTime;
    private int marginLeft = 80, marginRight = 10;
    private int marginTop = 10, marginBottom = 25;

    private TextMeshProUGUI lblX;
    private List<TextMeshProUGUI> yLabels = new List<TextMeshProUGUI>();
    private Color32[] pixels;
    private int texW, texH;

    void Start()
    {
        motorUI = FindFirstObjectByType<MotorControllerUI>();
        if (motorUI == null || graphImage == null)
        {
            Debug.LogError("RealtimeGraph: missing MotorControllerUI or RawImage");
            enabled = false;
            return;
        }
        graphTime = 0f;

        texW = Mathf.Max(100, (int)graphImage.rectTransform.rect.width);
        texH = Mathf.Max(60, (int)graphImage.rectTransform.rect.height);
        tex = new Texture2D(texW, texH);
        tex.filterMode = FilterMode.Point;
        graphImage.texture = tex;
        pixels = new Color32[texW * texH];

        float maxR = Mathf.Max(posRange, spdRange, curRange);

        CreateLegendLabel("lblPos", posColor, "Position", new Vector2(-10, -10));
        CreateLegendLabel("lblSpd", spdColor, "Speed", new Vector2(-10, -28));
        CreateLegendLabel("lblTor", curColor, "Torque", new Vector2(-10, -46));

        lblX = CreateAxisLabel("lblX", "t: 0.0s", new Vector2(0, marginBottom + 2), new Vector2(0.5f, 0), new Vector2(0.5f, 0), TextAlignmentOptions.Center);

        CreateAxisLabel("lblUnit", "rad / (rad/s) / Nm", new Vector2(marginLeft - 20, -(marginTop + 20)), new Vector2(0, 1), new Vector2(0, 1), TextAlignmentOptions.Right);

        CreateYTicks(maxR, yTickStep);
    }

    void CreateYTicks(float maxR, float step)
    {
        for (int i = 0; i < yLabels.Count; i++)
            if (yLabels[i] != null) Destroy(yLabels[i].gameObject);
        yLabels.Clear();

        float rawH = graphImage.rectTransform.rect.height;

        for (float v = -maxR; v <= maxR + 0.001f; v += step)
        {
            float t = (v + maxR) / (2f * maxR);
            float yFromTop = marginTop + (1f - t) * (rawH - marginTop - marginBottom);

            TextMeshProUGUI tmp = CreateAxisLabel("lblY_" + v.ToString("F1"),
                v.ToString("F1"),
                new Vector2(marginLeft - 20, -yFromTop),
                new Vector2(0, 1), new Vector2(0, 0.5f), TextAlignmentOptions.Right);
            yLabels.Add(tmp);
        }
    }

    void CreateLegendLabel(string name, Color color, string text, Vector2 pos)
    {
        GameObject box = new GameObject(name + "_box", typeof(Image));
        box.transform.SetParent(graphImage.transform, false);
        RectTransform brt = box.GetComponent<RectTransform>();
        brt.anchorMin = new Vector2(1, 1);
        brt.anchorMax = new Vector2(1, 1);
        brt.pivot = new Vector2(1, 1);
        brt.sizeDelta = new Vector2(12, 12);
        brt.anchoredPosition = new Vector2(pos.x - 52, pos.y + 1);
        box.GetComponent<Image>().color = color;

        GameObject lbl = new GameObject(name, typeof(TextMeshProUGUI));
        lbl.transform.SetParent(graphImage.transform, false);
        TextMeshProUGUI tmp = lbl.GetComponent<TextMeshProUGUI>();
        tmp.text = text;
        tmp.fontSize = 11;
        tmp.color = Color.black;
        tmp.alignment = TextAlignmentOptions.Left;
        RectTransform lrt = lbl.GetComponent<RectTransform>();
        lrt.anchorMin = new Vector2(1, 1);
        lrt.anchorMax = new Vector2(1, 1);
        lrt.pivot = new Vector2(1, 1);
        lrt.sizeDelta = new Vector2(80, 14);
        lrt.anchoredPosition = pos;
    }

    TextMeshProUGUI CreateAxisLabel(string name, string text, Vector2 pos, Vector2 anchor, Vector2 pivot, TextAlignmentOptions alignment = TextAlignmentOptions.Center)
    {
        GameObject lbl = new GameObject(name, typeof(TextMeshProUGUI));
        lbl.transform.SetParent(graphImage.transform, false);
        TextMeshProUGUI tmp = lbl.GetComponent<TextMeshProUGUI>();
        tmp.text = text;
        tmp.fontSize = 10;
        tmp.color = Color.black;
        tmp.alignment = alignment;
        RectTransform lrt = lbl.GetComponent<RectTransform>();
        lrt.anchorMin = anchor;
        lrt.anchorMax = anchor;
        lrt.pivot = pivot;
        lrt.sizeDelta = new Vector2(50, 14);
        lrt.anchoredPosition = pos;
        return tmp;
    }

    void Update()
    {
        if (motorUI.MotorEnabled)
        {
            graphTime += Time.deltaTime;

            var fb = motorUI.LastFeedback;
            times.Add(graphTime);
            posData.Add(fb.position);
            spdData.Add(fb.speed);
            curData.Add(fb.torque);  // 使用扭矩数据

            while (times.Count > 0 && times[0] < graphTime - timeWindow)
            {
                times.RemoveAt(0);
                posData.RemoveAt(0);
                spdData.RemoveAt(0);
                curData.RemoveAt(0);
            }
        }

        if (lblX != null)
            lblX.text = $"t: {graphTime:F1}s";

        Draw();
    }

    int PL { get { return marginLeft; } }
    int PR { get { return tex.width - marginRight; } }
    int PT { get { return marginTop; } }
    int PB { get { return tex.height - marginBottom; } }

    void Draw()
    {
        int w = tex.width, h = tex.height;

        for (int x = 0; x < w; x++)
            for (int y = 0; y < h; y++)
                tex.SetPixel(x, y, bgColor);

        int xL = PL, xR = PR, yT = PT, yB = PB;
        int pw = xR - xL, ph = yB - yT;
        float maxR = Mathf.Max(posRange, spdRange, curRange);

        for (float v = -maxR; v <= maxR + 0.001f; v += yTickStep)
        {
            float t = (v + maxR) / (2f * maxR);
            int gy = yT + (int)(t * ph);
            for (int x = xL; x < xR; x++)
                tex.SetPixel(x, gy, gridColor);
        }

        for (int i = 1; i <= 4; i++)
        {
            int gx = xL + i * pw / 5;
            for (int y = yT; y < yB; y++)
                tex.SetPixel(gx, y, gridColor);
        }

        int midY = (yT + yB) / 2;
        for (int x = xL; x < xR; x++)
            tex.SetPixel(x, midY, axisColor);

        for (int x = xL; x <= xR; x++) tex.SetPixel(x, yT, axisColor);
        for (int x = xL; x <= xR; x++) tex.SetPixel(x, yB, axisColor);
        for (int y = yT; y <= yB; y++) tex.SetPixel(xL, y, axisColor);
        for (int y = yT; y <= yB; y++) tex.SetPixel(xR, y, axisColor);

        if (times.Count >= 2)
        {
            float t0 = times[0], t1 = times[times.Count - 1];
            float tr = Mathf.Max(t1 - t0, 0.01f);
            int mid = midY;

            DrawCurve(times, posData, posRange, t0, tr, xL, xR, mid, ph, posColor);
            DrawCurve(times, spdData, spdRange, t0, tr, xL, xR, mid, ph, spdColor);
            DrawCurve(times, curData, curRange, t0, tr, xL, xR, mid, ph, curColor);
        }

        tex.Apply();
    }

    void DrawCurve(List<float> ts, List<float> vals, float range,
                   float t0, float tr, int xL, int xR, int midY, int ph, Color c)
    {
        int half = ph / 2 - 1;
        for (int i = 0; i < ts.Count - 1; i++)
        {
            int x0 = xL + Mathf.RoundToInt((ts[i] - t0) / tr * (xR - xL));
            int x1 = xL + Mathf.RoundToInt((ts[i + 1] - t0) / tr * (xR - xL));
            int y0 = midY + Mathf.RoundToInt(Mathf.Clamp(vals[i], -range, range) / range * half);
            int y1 = midY + Mathf.RoundToInt(Mathf.Clamp(vals[i + 1], -range, range) / range * half);
            Segment(x0, y0, x1, y1, c);
        }
    }

    void Segment(int x0, int y0, int x1, int y1, Color c)
    {
        int dx = Mathf.Abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = -Mathf.Abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = dx + dy;
        while (true)
        {
            if (x0 >= 0 && x0 < tex.width && y0 >= 0 && y0 < tex.height)
                tex.SetPixel(x0, y0, c);
            if (x0 == x1 && y0 == y1) break;
            int e2 = 2 * err;
            if (e2 >= dy) { err += dy; x0 += sx; }
            if (e2 <= dy) { err += dx; y0 += sy; }
        }
    }
}
