using System.IO;
using UnityEngine;
using UnityEngine.UI;

public class CsvLogger : MonoBehaviour
{
    [SerializeField] private Button toggleButton;
    [SerializeField] private float logInterval = 0.01f;

    private MotorControllerUI motorUI;
    private StreamWriter writer;
    private string currentFilePath;
    private float nextLogTime;
    private float startTime;
    private bool isRecording;

    public bool IsRecording => isRecording;

    void Start()
    {
        motorUI = FindFirstObjectByType<MotorControllerUI>();
        if (motorUI == null)
        {
            Debug.LogError("CsvLogger: MotorControllerUI not found");
            enabled = false;
            return;
        }

        UpdateButtonText();
    }

    public void OnToggleLogClick()
    {
        if (isRecording)
            StopLogging();
        else
            StartLogging();
    }

    private void StartLogging()
    {
        string dir = Application.dataPath + "/../Logs/";
        Directory.CreateDirectory(dir);
        string filename = $"MotorLog_{System.DateTime.Now:yyyy-MM-dd_HH-mm-ss}.csv";
        currentFilePath = dir + filename;
        writer = new StreamWriter(currentFilePath, false);
        writer.WriteLine("Time_s,Position_rad,Speed_rad_s,Torque_Nm,Temperature_C,ErrorCode,ErrorMsg");
        writer.Flush();
        startTime = Time.time;
        nextLogTime = 0f;
        isRecording = true;
        Debug.Log($"Recording started: {currentFilePath}");
        UpdateButtonText();
    }

    private void StopLogging()
    {
        if (writer != null)
        {
            writer.Close();
            writer = null;
            Debug.Log($"Recording saved: {currentFilePath}");
        }
        isRecording = false;
        currentFilePath = null;
        UpdateButtonText();
    }

    void Update()
    {
        if (!isRecording || writer == null) return;
        if (Time.time < nextLogTime) return;
        nextLogTime = Time.time + logInterval;

        var fb = motorUI.LastFeedback;
        float elapsed = Time.time - startTime;
        writer.WriteLine($"{elapsed:F3},{fb.position:F4},{fb.speed:F4},{fb.torque:F4},{fb.temperature:F1},0x{fb.errorCode:X2},{fb.errorMsg}");
        writer.Flush();
    }

    private void UpdateButtonText()
    {
        if (toggleButton == null) return;
        Text t = toggleButton.GetComponentInChildren<Text>();
        TMPro.TextMeshProUGUI tmp = toggleButton.GetComponentInChildren<TMPro.TextMeshProUGUI>();
        string text = isRecording ? "Stop Record" : "Start Record";
        ColorBlock colors = toggleButton.colors;
        if (isRecording)
        {
            colors.normalColor = new Color(0.8f, 0.2f, 0.2f);
            colors.selectedColor = new Color(0.6f, 0.1f, 0.1f);
        }
        else
        {
            colors.normalColor = Color.white;
            colors.selectedColor = Color.white;
        }
        toggleButton.colors = colors;
        if (tmp != null)
        {
            tmp.text = text;
            tmp.color = isRecording ? Color.red : Color.black;
        }
        else if (t != null)
        {
            t.text = text;
            t.color = isRecording ? Color.red : Color.black;
        }
    }

    void OnDestroy()
    {
        if (writer != null)
        {
            writer.Close();
            writer = null;
        }
    }
}
