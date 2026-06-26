using UnityEngine;

public class SerialTest : MonoBehaviour
{
    private UartManager uart;
    private AtFrameDecoder decoder;

    void Start()
    {
        decoder = new AtFrameDecoder();

        uart = gameObject.AddComponent<UartManager>();
        uart.OnDataReceived += OnRawData;

        decoder.OnFrameDecoded += frame =>
        {
            Debug.Log($"Frame decoded: {frame}");
            MotorFeedback fb = MotorFeedback.Parse(frame);
            Debug.Log($"Feedback: {fb}");
        };
    }

    private void OnRawData(byte[] data)
    {
        string hex = System.BitConverter.ToString(data);
        Debug.Log($"Raw({data.Length}): {hex}");
        decoder.Feed(data);
    }
}
