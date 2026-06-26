using UnityEngine;
using TMPro;

public class StatusDisplay : MonoBehaviour
{
    [SerializeField] private MotorControllerUI motorUI;
    [SerializeField] private TMP_Text posText;
    [SerializeField] private TMP_Text speedText;
    [SerializeField] private TMP_Text currentText;
    [SerializeField] private TMP_Text torqueText;  // 显示扭矩数值
    [SerializeField] private TMP_Text tempText;
    [SerializeField] private TMP_Text errorText;
    [SerializeField] private TMP_Text encoderText;  // 显示磁编码器数据
    [SerializeField] private TMP_Text encoder2Text;  // 显示磁编码器2数据

    void Update()
    {
        if (motorUI == null) return;
        MotorFeedback fb = motorUI.LastFeedback;

        posText.text = $"Pos: {fb.position:F3} rad";
        speedText.text = $"Spd: {fb.speed:F3} rad/s";
        currentText.text = $"Iq Fdb: {motorUI.MotorCurrent:F3} A";
        if (torqueText != null)
            torqueText.text = $"Torque: {fb.torque:F3} Nm";
        tempText.text = $"Temp: {fb.temperature:F1} °C";
        errorText.text = $"Error: {fb.errorMsg}";
        errorText.color = fb.errorCode == 0 ? Color.black : Color.red;

        // 显示磁编码器数据
        if (encoderText != null && motorUI != null)
        {
            encoderText.text = $"Encoder: {motorUI.EncoderAngle:F3}° (Raw: {motorUI.EncoderRaw})";
        }
        
        // 显示磁编码器2数据
        if (encoder2Text != null && motorUI != null)
        {
            encoder2Text.text = $"Encoder2: {motorUI.Encoder2Angle:F3}° (Raw: {motorUI.Encoder2Raw})";
        }
    }
}
