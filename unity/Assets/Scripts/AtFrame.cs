using System;

public struct AtFrame
{
    public uint canId;     // 29-bit CAN ID
    public byte dataLen;   // 0-8
    public byte[] data;    // payload

    public static byte[] Encode(uint canId, byte[] data, int len)
    {
        if (len > 8) len = 8;
        byte[] frame = new byte[9 + len];
        frame[0] = 0x41;
        frame[1] = 0x54;
        frame[2] = (byte)((canId >> 24) & 0xFF);
        frame[3] = (byte)((canId >> 16) & 0xFF);
        frame[4] = (byte)((canId >> 8) & 0xFF);
        frame[5] = (byte)(canId & 0xFF);
        frame[6] = (byte)len;
        Array.Copy(data, 0, frame, 7, len);
        frame[7 + len] = 0x0D;
        frame[8 + len] = 0x0A;
        return frame;
    }

    public override string ToString()
    {
        string hex = BitConverter.ToString(data, 0, dataLen);
        return $"ID=0x{canId:X8} Len={dataLen} Data={hex}";
    }
}

public class AtFrameDecoder
{
    private enum State { WaitHead1, WaitHead2, ReadCanId, ReadLen, ReadData, WaitTail1, WaitTail2 }

    private State state = State.WaitHead1;
    private byte[] buf = new byte[4];
    private int ptr;
    private uint canId;
    private byte dataLen;
    private byte[] data = new byte[8];

    public event Action<AtFrame> OnFrameDecoded;

    public void Feed(byte[] bytes, int offset, int count)
    {
        for (int i = offset; i < offset + count; i++)
            FeedByte(bytes[i]);
    }

    public void Feed(byte[] bytes)
    {
        Feed(bytes, 0, bytes.Length);
    }

    private void FeedByte(byte b)
    {
        switch (state)
        {
            case State.WaitHead1:
                if (b == 0x41) state = State.WaitHead2;
                break;

            case State.WaitHead2:
                if (b == 0x54) { state = State.ReadCanId; ptr = 0; }
                else state = State.WaitHead1;
                break;

            case State.ReadCanId:
                buf[ptr++] = b;
                if (ptr == 4)
                {
                    canId = (uint)(buf[0] << 24 | buf[1] << 16 | buf[2] << 8 | buf[3]);
                    state = State.ReadLen;
                }
                break;

            case State.ReadLen:
                dataLen = b;
                if (dataLen > 8) dataLen = 8;
                state = State.ReadData;
                ptr = 0;
                break;

            case State.ReadData:
                if (ptr < dataLen) data[ptr++] = b;
                if (ptr == dataLen) state = State.WaitTail1;
                break;

            case State.WaitTail1:
                if (b == 0x0D) state = State.WaitTail2;
                else state = State.WaitHead1;
                break;

            case State.WaitTail2:
                if (b == 0x0A)
                {
                    byte[] frameData = new byte[dataLen];
                    Array.Copy(data, frameData, dataLen);
                    OnFrameDecoded?.Invoke(new AtFrame
                    {
                        canId = canId,
                        dataLen = dataLen,
                        data = frameData
                    });
                }
                state = State.WaitHead1;
                break;

            default:
                state = State.WaitHead1;
                break;
        }
    }

    public void Reset()
    {
        state = State.WaitHead1;
        ptr = 0;
    }
}
