using System;
using System.Collections.Generic;
using System.Net.Sockets;
using System.IO;
using System.Globalization;
using UnityEngine;

public class LidarTCPClient : MonoBehaviour
{
    TcpClient client;
    StreamReader reader;

    List<(float angle, float dist)> scan = new List<(float, float)>();
    bool collecting = false;

    public LidarCubes visualizer;
    public LidarBeep beepSystem;

    void Start()
    {
        client = new TcpClient("127.0.0.1", 5005);
        reader = new StreamReader(client.GetStream());

        Debug.Log("Connected to Python server");
    }

    void Update()
    {
        while (client.Available > 0)
        {
            string line = reader.ReadLine();

            if (line == "<START>")
            {
                scan.Clear();
                collecting = true;
            }
            else if (line == "<END>")
            {
                collecting = false;

                // Send to visualizer
                if (visualizer != null)
                    visualizer.UpdateScan(scan);
                if (beepSystem != null)
                    beepSystem.UpdateScan(scan);
            }
            else if (collecting)
            {
                string[] parts = line.Split(',');

                float angle = float.Parse(parts[0], CultureInfo.InvariantCulture);
                float dist = float.Parse(parts[1], CultureInfo.InvariantCulture);

                scan.Add((angle, dist));
            }
        }
    }

    void OnApplicationQuit()
    {
        reader.Close();
        client.Close();
    }
}
