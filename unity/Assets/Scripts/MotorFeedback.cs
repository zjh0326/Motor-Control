using System;

public struct MotorFeedback
{
    public float position;    // rad
    public float speed;       // rad/s
    public float torque;      // Nm (from feedback frame Byte4-5, range ±17Nm)
    public float current;     // A (from GetParam 0x701A iqf polling — not yet implemented)
    public float temperature; // °C
    public byte errorCode;
    public string errorMsg;

    public static MotorFeedback Parse(AtFrame frame)
    {
        MotorFeedback fb = new MotorFeedback();
        uint commType = (frame.canId >> 24) & 0xFF;

        if ((commType == 0x02 || commType == 0x18) && frame.dataLen >= 8)
        {
            fb.errorCode = (byte)((frame.canId >> 16) & 0xFF);
            fb.errorMsg = GetErrorMsg(fb.errorCode);

            ushort posRaw = (ushort)((frame.data[0] << 8) | frame.data[1]);
            fb.position = (posRaw - 32768.0f) * 4.0f * (float)Math.PI / 32768.0f;

            ushort spdRaw = (ushort)((frame.data[2] << 8) | frame.data[3]);
            fb.speed = (spdRaw - 32768.0f) * 44.0f / 32768.0f;

            ushort torRaw = (ushort)((frame.data[4] << 8) | frame.data[5]);
            fb.torque = (torRaw - 32768.0f) * 17.0f / 32768.0f;

            ushort tempRaw = (ushort)((frame.data[6] << 8) | frame.data[7]);
            fb.temperature = tempRaw * 0.1f;
        }

        return fb;
    }

    private static string GetErrorMsg(byte error)
    {
        if (error == 0x00) return "no error";
        if ((error & 0x01) != 0) return "over voltage";
        if ((error & 0x02) != 0) return "driver fault";
        if ((error & 0x04) != 0) return "over temp";
        if ((error & 0x08) != 0) return "encoder fault";
        if ((error & 0x10) != 0) return "overload";
        if ((error & 0x20) != 0) return "not calibrated";
        if ((error & 0x80) != 0) return "hall fault";
        return "unknown";
    }

    public override string ToString()
    {
        return $"Pos={position:F3}rad  Spd={speed:F3}rad/s  Tor={torque:F3}Nm  " +
               $"Temp={temperature:F1}°C  Err=0x{errorCode:X2}({errorMsg})";
    }
}
