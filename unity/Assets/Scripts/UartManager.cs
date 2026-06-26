using System;
using System.IO.Ports;
using System.Threading;
using UnityEngine;

public class UartManager : MonoBehaviour
{
    public string portName = "COM7";
    public int baudRate = 115200;

    private SerialPort serialPort;
    private Thread readThread;
    private volatile bool isRunning;
    private readonly object portLock = new object();
    private readonly object writeLock = new object();

    public event Action<byte[]> OnDataReceived;

    public void Open()
    {
        if (serialPort != null && serialPort.IsOpen) return;

        serialPort = new SerialPort(portName, baudRate, Parity.None, 8, StopBits.One);
        serialPort.ReadTimeout = 1000;
        serialPort.Open();

        isRunning = true;
        readThread = new Thread(ReadLoop) { IsBackground = true };
        readThread.Start();

        Debug.Log($"UART opened: {portName} @ {baudRate}");
    }

    public void Close()
    {
        isRunning = false;

        if (readThread != null && readThread.IsAlive)
            readThread.Join(500);

        lock (portLock)
        {
            if (serialPort != null && serialPort.IsOpen)
            {
                serialPort.Close();
                serialPort.Dispose();
            }
            serialPort = null;
        }
        Debug.Log("UART closed");
    }

    public void Send(byte[] data)
    {
        lock (writeLock)
        {
            if (serialPort != null && serialPort.IsOpen)
                serialPort.Write(data, 0, data.Length);
        }
    }

    private void ReadLoop()
    {
        byte[] buf = new byte[1024];

        while (isRunning)
        {
            try
            {
                int count;
                lock (portLock)
                {
                    if (serialPort == null || !serialPort.IsOpen) continue;
                    count = serialPort.Read(buf, 0, buf.Length);
                }
                if (count > 0)
                {
                    byte[] chunk = new byte[count];
                    Array.Copy(buf, chunk, count);
                    OnDataReceived?.Invoke(chunk);
                }
            }
            catch (TimeoutException) { }
            catch (Exception ex)
            {
                if (isRunning)
                    Debug.LogError($"UART read error: {ex.Message}");
            }
        }
    }

    void OnDestroy()
    {
        Close();
    }
}
