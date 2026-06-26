using UnityEngine;
using UnityEngine.UI;
using TMPro;
using System.Collections.Generic;

public class ParameterPanel : MonoBehaviour
{
    [SerializeField] private Transform rowContainer;
    [SerializeField] private float rowHeight = 28f;
    [SerializeField] private float pollInterval = 0.5f;

    private MotorControllerUI motorUI;
    private Dictionary<ushort, TextMeshProUGUI> valueLabels = new Dictionary<ushort, TextMeshProUGUI>();
    private Dictionary<ushort, TMP_InputField> inputFields = new Dictionary<ushort, TMP_InputField>();
    private Button autoPollBtn;

    struct ParamDef
    {
        public string name;
        public ushort index;
        public bool isFloat;
        public bool canWrite;
    }

    private List<ParamDef> paramDefs = new List<ParamDef>
    {
        new ParamDef { name = "Run Mode",      index = 0x7005, isFloat = false, canWrite = true },
        new ParamDef { name = "Iq Ref",        index = 0x7006, isFloat = true,  canWrite = true },
        new ParamDef { name = "Speed Ref",     index = 0x700A, isFloat = true,  canWrite = true },
        new ParamDef { name = "Torque Limit",  index = 0x700B, isFloat = true,  canWrite = true },
        new ParamDef { name = "Current Kp",    index = 0x7010, isFloat = true,  canWrite = true },
        new ParamDef { name = "Current Ki",    index = 0x7011, isFloat = true,  canWrite = true },
        new ParamDef { name = "Current Filt",  index = 0x7014, isFloat = true,  canWrite = true },
        new ParamDef { name = "Position Ref",  index = 0x7016, isFloat = true,  canWrite = true },
        new ParamDef { name = "Speed Limit",   index = 0x7017, isFloat = true,  canWrite = true },
        new ParamDef { name = "Current Limit", index = 0x7018, isFloat = true,  canWrite = true },
        new ParamDef { name = "Mech Pos",      index = 0x7019, isFloat = true,  canWrite = false },
        new ParamDef { name = "Iq Filt",       index = 0x701A, isFloat = true,  canWrite = false },
        new ParamDef { name = "Mech Vel",      index = 0x701B, isFloat = true,  canWrite = false },
        new ParamDef { name = "Bus Voltage",   index = 0x701C, isFloat = true,  canWrite = false },
        new ParamDef { name = "Rotation",      index = 0x701D, isFloat = false, canWrite = true },
        new ParamDef { name = "Position Kp",   index = 0x701E, isFloat = true,  canWrite = true },
        new ParamDef { name = "Speed Kp",      index = 0x701F, isFloat = true,  canWrite = true },
        new ParamDef { name = "Speed Ki",      index = 0x7020, isFloat = true,  canWrite = true },
    };

    void Start()
    {
        motorUI = FindFirstObjectByType<MotorControllerUI>();
        if (motorUI == null || rowContainer == null)
        {
            Debug.LogError("ParameterPanel: missing MotorControllerUI or RowContainer");
            enabled = false;
            return;
        }

        motorUI.OnGetParamResponse += OnParamResponse;

        foreach (Transform child in rowContainer)
            Destroy(child.gameObject);

        for (int i = 0; i < paramDefs.Count; i++)
            CreateRow(i, paramDefs[i]);

        AddAutoPollButton();

        RectTransform crt = rowContainer.GetComponent<RectTransform>();
        crt.anchorMin = new Vector2(0, 1);
        crt.anchorMax = new Vector2(0, 1);
        crt.pivot = new Vector2(0, 1);
        crt.sizeDelta = new Vector2(450, 0);

        ContentSizeFitter csf = rowContainer.GetComponent<ContentSizeFitter>();
        if (csf == null) csf = rowContainer.gameObject.AddComponent<ContentSizeFitter>();
        csf.horizontalFit = ContentSizeFitter.FitMode.PreferredSize;
        csf.verticalFit = ContentSizeFitter.FitMode.PreferredSize;
    }

    void CreateRow(int idx, ParamDef def)
    {
        GameObject row = new GameObject("row_" + def.name.Replace(" ", ""), typeof(RectTransform));
        row.transform.SetParent(rowContainer, false);
        LayoutElement rowLe = row.AddComponent<LayoutElement>();
        rowLe.preferredHeight = rowHeight;
        rowLe.flexibleWidth = 1;

        HorizontalLayoutGroup hlg = row.AddComponent<HorizontalLayoutGroup>();
        hlg.childForceExpandWidth = false;
        hlg.childForceExpandHeight = true;
        hlg.childAlignment = TextAnchor.MiddleLeft;
        hlg.spacing = 4;
        hlg.padding = new RectOffset(4, 4, 0, 0);

        AddText(row, def.name, 120, 13, TextAlignmentOptions.Left, TextAnchor.MiddleLeft);
        TextMeshProUGUI valLabel = AddText(row, "---", 80, 13, TextAlignmentOptions.Right, TextAnchor.MiddleRight);
        valueLabels[def.index] = valLabel;

        TMP_InputField input = AddInputField(row, 80);
        inputFields[def.index] = input;

        Button btnRead = AddButton(row, "Rd", 40, () => motorUI.SendGetParam(def.index));
        Button btnWrite = AddButton(row, "Wr", 40, () =>
        {
            float val;
            if (!float.TryParse(input.text, out val)) return;
            if (def.isFloat)
                motorUI.SendSetParam(def.index, val);
            else
                motorUI.SendSetParamMode(def.index, (byte)val);
        });
        btnWrite.interactable = def.canWrite;
    }

    void AddAutoPollButton()
    {
        GameObject row = new GameObject("row_AutoPoll", typeof(RectTransform));
        row.transform.SetParent(rowContainer, false);
        row.transform.SetAsFirstSibling();
        LayoutElement rowLe = row.AddComponent<LayoutElement>();
        rowLe.preferredHeight = rowHeight;
        rowLe.flexibleWidth = 1;

        HorizontalLayoutGroup hlg = row.AddComponent<HorizontalLayoutGroup>();
        hlg.childForceExpandWidth = false;
        hlg.childForceExpandHeight = true;
        hlg.childAlignment = TextAnchor.MiddleLeft;
        hlg.spacing = 4;
        hlg.padding = new RectOffset(4, 4, 0, 0);

        TextMeshProUGUI label = AddText(row, "Auto Poll", 120, 13, TextAlignmentOptions.Left, TextAnchor.MiddleLeft);

        autoPollBtn = AddButton(row, "Start", 166, () =>
        {
            if (motorUI.IsAutoPolling)
            {
                motorUI.StopAutoPoll();
                SetAutoPollButtonText(false);
            }
            else
            {
                List<ushort> indices = new List<ushort>();
                foreach (var def in paramDefs)
                    indices.Add(def.index);
                motorUI.StartAutoPoll(indices, pollInterval);
                SetAutoPollButtonText(true);
            }
        });
    }

    void SetAutoPollButtonText(bool polling)
    {
        if (autoPollBtn == null) return;
        var tmp = autoPollBtn.GetComponentInChildren<TMPro.TextMeshProUGUI>();
        if (tmp != null)
        {
            tmp.text = polling ? "Stop" : "Start";
            tmp.color = polling ? Color.red : Color.black;
        }
    }

    TextMeshProUGUI AddText(GameObject parent, string text, float width, float fontSize,
                            TextAlignmentOptions align, TextAnchor anchor = TextAnchor.MiddleLeft)
    {
        GameObject go = new GameObject("txt_" + text, typeof(TextMeshProUGUI));
        go.transform.SetParent(parent.transform, false);
        LayoutElement le = go.AddComponent<LayoutElement>();
        le.preferredWidth = width;
        le.flexibleHeight = 1;

        TextMeshProUGUI tmp = go.GetComponent<TextMeshProUGUI>();
        tmp.text = text;
        tmp.fontSize = fontSize;
        tmp.color = Color.black;
        tmp.alignment = align;

        RectTransform rt = go.GetComponent<RectTransform>();
        rt.anchorMin = new Vector2(0, 0);
        rt.anchorMax = new Vector2(0, 1);
        rt.pivot = new Vector2(0, 0.5f);
        return tmp;
    }

    TMP_InputField AddInputField(GameObject parent, float width)
    {
        GameObject go = new GameObject("input", typeof(RectTransform));
        go.transform.SetParent(parent.transform, false);
        LayoutElement le = go.AddComponent<LayoutElement>();
        le.preferredWidth = width;
        le.flexibleHeight = 1;

        Image bg = go.AddComponent<Image>();
        bg.color = Color.white;

        TMP_InputField input = go.AddComponent<TMP_InputField>();

        GameObject textArea = new GameObject("TextArea", typeof(RectTransform));
        textArea.transform.SetParent(go.transform, false);
        RectTransform taRt = textArea.GetComponent<RectTransform>();
        taRt.anchorMin = Vector2.zero;
        taRt.anchorMax = Vector2.one;
        taRt.sizeDelta = Vector2.zero;
        taRt.offsetMin = new Vector2(2, 2);
        taRt.offsetMax = new Vector2(-2, -2);

        GameObject placeholder = new GameObject("Placeholder", typeof(TextMeshProUGUI));
        placeholder.transform.SetParent(textArea.transform, false);
        TextMeshProUGUI phText = placeholder.GetComponent<TextMeshProUGUI>();
        phText.text = "";
        phText.fontSize = 12;
        phText.color = Color.gray;

        GameObject textComp = new GameObject("Text", typeof(TextMeshProUGUI));
        textComp.transform.SetParent(textArea.transform, false);
        TextMeshProUGUI tmpText = textComp.GetComponent<TextMeshProUGUI>();
        tmpText.fontSize = 12;
        tmpText.color = Color.black;
        tmpText.alignment = TextAlignmentOptions.Right;

        input.textComponent = tmpText;
        input.placeholder = phText;
        input.textViewport = textArea.GetComponent<RectTransform>();
        input.contentType = TMP_InputField.ContentType.DecimalNumber;

        return input;
    }

    Button AddButton(GameObject parent, string text, float width, UnityEngine.Events.UnityAction onClick)
    {
        GameObject go = new GameObject("btn_" + text, typeof(Image), typeof(Button));
        go.transform.SetParent(parent.transform, false);
        LayoutElement le = go.AddComponent<LayoutElement>();
        le.preferredWidth = width;
        le.flexibleHeight = 1;

        Button btn = go.GetComponent<Button>();
        btn.onClick.AddListener(onClick);
        btn.colors = new ColorBlock
        {
            normalColor = new Color(0.85f, 0.85f, 0.85f),
            highlightedColor = new Color(0.7f, 0.7f, 0.7f),
            pressedColor = new Color(0.5f, 0.5f, 0.5f),
            disabledColor = new Color(0.9f, 0.9f, 0.9f),
            colorMultiplier = 1,
            fadeDuration = 0.1f
        };

        GameObject txt = new GameObject("txt", typeof(TextMeshProUGUI));
        txt.transform.SetParent(go.transform, false);
        TextMeshProUGUI tmp = txt.GetComponent<TextMeshProUGUI>();
        tmp.text = text;
        tmp.fontSize = 12;
        tmp.color = Color.black;
        tmp.alignment = TextAlignmentOptions.Center;
        RectTransform trt = txt.GetComponent<RectTransform>();
        trt.anchorMin = Vector2.zero;
        trt.anchorMax = Vector2.one;
        trt.sizeDelta = Vector2.zero;

        return btn;
    }

    void OnParamResponse(ushort index, byte[] data)
    {
        TextMeshProUGUI label;
        if (!valueLabels.TryGetValue(index, out label)) return;

        ParamDef def = paramDefs.Find(p => p.index == index);
        if (def.isFloat)
        {
            float val = System.BitConverter.ToSingle(data, 4);
            label.text = val.ToString("F3");
        }
        else
        {
            label.text = data[4].ToString();
        }
    }

    void OnDestroy()
    {
        if (motorUI != null)
        {
            motorUI.OnGetParamResponse -= OnParamResponse;
            motorUI.StopAutoPoll();
        }
    }
}
