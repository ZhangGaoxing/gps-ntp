using System.Diagnostics;
using System.Globalization;
using System.IO.Ports;
using System.Net;
using System.Net.Sockets;
using System.Runtime.InteropServices;
using System.Text;

// 使用的串口名称
const string SERIAL_NAME = "/dev/ttyUSB0";
// 1 s = 10000000 ticks
const long TICK_2_SECOND = 10000000;
// NTP 时间戳起始时间
DateTime ntpStart = new DateTime(1900, 1, 1, 0, 0, 0, DateTimeKind.Utc);
// 本机时钟最后更新时间
DateTime lastUpdatedTime = DateTime.Now.ToUniversalTime();

// GPS 串口初始化
using SerialPort gps = new SerialPort(SERIAL_NAME)
{
    BaudRate = 9600,
    Encoding = Encoding.UTF8,
    ReadTimeout = 500,
    WriteTimeout = 500,
};
gps.DataReceived += GpsFrameReceived;
gps.Open();

// NTP 服务初始化
using Socket ntpServer = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
IPEndPoint ip = new IPEndPoint(IPAddress.Any, 123);
ntpServer.Bind(ip);
new Thread(NtpFrameReceived)
{
    IsBackground = true
}.Start();

/// <summary>
/// NTP 报文接收与发送
/// </summary>
void NtpFrameReceived()
{
    Span<byte> receiveFrame = stackalloc byte[48];

    while (true)
    {
        EndPoint clientPoint = new IPEndPoint(IPAddress.Any, 0);
        ntpServer.ReceiveFrom(receiveFrame, ref clientPoint);
        DateTime receiveTime = DateTime.UtcNow;
        Console.WriteLine($"{receiveTime.ToLocalTime()} Receive: {string.Join(' ', receiveFrame.ToArray())}");

        Span<byte> sendFrame = GenerateNtpFrame(receiveFrame, DateTime.UtcNow);
        ntpServer.SendTo(sendFrame, clientPoint);
        DateTime sendTime = DateTime.UtcNow;
        Console.WriteLine($"{sendTime.ToLocalTime()} Send: {string.Join(' ', sendFrame.ToArray())}");
    }
}

/// <summary>
/// 生成 NTP 报文
/// </summary>
Span<byte> GenerateNtpFrame(Span<byte> receivedFrame, DateTime receiveTime)
{
    Span<byte> ntpFrame = stackalloc byte[48]
    {
        0x1c, 0x01, 0x11, 0xe9, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    };

    // Client Transmit Timestamp => Server Origin Timestamp
    for (int i = 0; i < 8; i++)
    {
        ntpFrame[24 + i] = receivedFrame[40 + i];
    }

    // 本机时钟最后更新时间
    long referenceTicks = (lastUpdatedTime - ntpStart).Ticks;
    uint referenceTimeInt = (uint)(referenceTicks / TICK_2_SECOND);
    uint referenceTimeFract = (uint)(referenceTicks % TICK_2_SECOND);
    var referenceTimeIntByte = BitConverter.GetBytes(referenceTimeInt);
    var referenceTimeFractByte = BitConverter.GetBytes(referenceTimeFract);

    // 接收报文时间
    long receiveTicks = (receiveTime - ntpStart).Ticks;
    uint receiveTimeInt = (uint)(receiveTicks / TICK_2_SECOND);
    uint receiveTimeFract = (uint)(receiveTicks % TICK_2_SECOND);
    var receiveTimeIntByte = BitConverter.GetBytes(receiveTimeInt);
    var receiveTimeFractByte = BitConverter.GetBytes(receiveTimeFract);

    // 发送报文时间
    long transmitTicks = (DateTime.UtcNow - ntpStart).Ticks;
    uint transmitTimeInt = (uint)(receiveTicks / TICK_2_SECOND);
    uint transmitTimeFract = (uint)(receiveTicks % TICK_2_SECOND);
    var transmitTimeIntByte = BitConverter.GetBytes(receiveTimeInt);
    var transmitTimeFractByte = BitConverter.GetBytes(receiveTimeFract);

    if (BitConverter.IsLittleEndian)
    {
        for (int i = 0; i < 4; i++)
        {
            ntpFrame[19 - i] = referenceTimeIntByte[i];
            ntpFrame[23 - i] = referenceTimeFractByte[i];

            ntpFrame[35 - i] = receiveTimeIntByte[i];
            ntpFrame[39 - i] = receiveTimeFractByte[i];

            ntpFrame[43 - i] = transmitTimeIntByte[i];
            ntpFrame[47 - i] = transmitTimeFractByte[i];
        }
    }
    else
    {
        for (int i = 0; i < 4; i++)
        {
            ntpFrame[16 + i] = referenceTimeIntByte[i];
            ntpFrame[20 + i] = referenceTimeFractByte[i];

            ntpFrame[32 + i] = receiveTimeIntByte[i];
            ntpFrame[36 + i] = receiveTimeFractByte[i];

            ntpFrame[40 + i] = transmitTimeIntByte[i];
            ntpFrame[44 + i] = transmitTimeFractByte[i];
        }
    }

    return ntpFrame.ToArray();
}

/// <summary>
/// GPS 报文处理
/// </summary>
void GpsFrameReceived(object sender, SerialDataReceivedEventArgs e)
{
    string frame = gps.ReadLine();

    if (frame.StartsWith("$GPRMC"))
    {
        // $GPRMC,UTC 时间,定位状态,纬度,纬度半球,经度,经度半球,速度,航向,UTC 日期,磁偏角,磁偏角方向,指示模式*校验和
        // $GPRMC,013717.00,A,3816.57392,N,10708.73951,E,0.467,,050722,,,A*78
        string[] field = frame.Split(',');

        // 帧数据有效
        if (!field[12].StartsWith("N"))
        {
            // 获取 GPS 时间
            string time = field[1][0..6];
            string date = field[9];
            DateTime utcNow = DateTime.ParseExact($"{date}{time}", "ddMMyyHHmmss", CultureInfo.InvariantCulture);

            // 更新系统时间
            UpdateSystemTime(utcNow);

            lastUpdatedTime = utcNow;

            Console.WriteLine($"{lastUpdatedTime.ToLocalTime()}: {frame}");
        }
    }
}

void UpdateSystemTime(DateTime time)
{
    ProcessStartInfo processInfo;
    if (RuntimeInformation.IsOSPlatform(OSPlatform.Windows))
    {
        processInfo = new ProcessStartInfo
        {
            FileName = "powershell.exe",
            Arguments = $"Set-Date \"\"\"{time.ToLocalTime().ToString("yyyy-MM-dd HH:mm:ss")}\"\"\"",
            RedirectStandardOutput = true,
            UseShellExecute = false,
            CreateNoWindow = true,
        };
    }
    else
    {
        processInfo = new ProcessStartInfo
        {
            FileName = "date",
            Arguments = $"-s \"{time.ToLocalTime().ToString("yyyy-MM-dd HH:mm:ss")}\"",
            RedirectStandardOutput = true,
            UseShellExecute = false,
            CreateNoWindow = true,
        };
    }

    var process = Process.Start(processInfo);
    process.WaitForExit();
}

Console.Read();
