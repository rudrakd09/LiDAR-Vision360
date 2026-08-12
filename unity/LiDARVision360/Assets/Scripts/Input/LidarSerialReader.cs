using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Threading;
using UnityEngine;

public class LidarSerialReader : MonoBehaviour
{
    public string portName = "COM3";
    public int baudRate = 115200;

    private SerialPort sp;
    private Thread readThread;
    private bool running = true;

    private List<(float angle, float distance)> latestScan = new List<(float, float)>();
    private bool newScanReady = false;

    // 👇 Reference to visualizer
    public LidarCubes visualizer;

    void Start()
    {
        sp = new SerialPort(portName, baudRate);
        sp.Open();

        readThread = new Thread(ReadSerial);
        readThread.Start();
    }

    void ReadSerial()
    {
        List<(float, float)> tempScan = new List<(float, float)>();

        while (running)
        {
            try
            {
                string line = sp.ReadLine().Trim();

                if (line == "<START>")
                {
                    tempScan.Clear();
                }
                else if (line == "<END>")
                {
                    lock (latestScan)
                    {
                        latestScan = new List<(float, float)>(tempScan);
                        newScanReady = true;
                    }
                }
                else
                {
                    string[] parts = line.Split(',');

                    float angle = float.Parse(parts[0]);
                    float dist = float.Parse(parts[1]);

                    tempScan.Add((angle, dist));
                }
            }
            catch { }
        }
    }

    void Update()
    {
        if (newScanReady)
        {
            List<(float, float)> scanCopy;

            lock (latestScan)
            {
                scanCopy = new List<(float, float)>(latestScan);
                newScanReady = false;
            }

            // 👇 Send data to visualizer
            if (visualizer != null)
            {
                visualizer.UpdateScan(scanCopy);
            }
        }
    }

    void OnApplicationQuit()
    {
        running = false;

        if (readThread != null && readThread.IsAlive)
            readThread.Abort();

        if (sp != null && sp.IsOpen)
            sp.Close();
    }
}
