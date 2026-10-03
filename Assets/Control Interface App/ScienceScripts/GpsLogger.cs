using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using TMPro;
using UnityEngine;

public class GpsLogger : MonoBehaviour
{
    [SerializeField] private GameObject statusTextObject;
    [SerializeField] private bool startLoggingOnAwake = false;
    [SerializeField] private string csvFilePrefix = "gps_log";
    [SerializeField] private string logDirectory = "gps_logs";

    private TextMeshProUGUI statusText;
    private bool isLogging = false;
    private string logFilePath;
    private readonly List<GpsLogEntry> logEntries = new List<GpsLogEntry>();
    private GpsLogEntry latestLoggedEntry;

    private class GpsLogEntry
    {
        public DateTime Timestamp;
        public double Latitude;
        public double Longitude;
        public bool IsPositionOfInterest;

        public GpsLogEntry(DateTime timestamp, double latitude, double longitude, bool isPositionOfInterest)
        {
            Timestamp = timestamp;
            Latitude = latitude;
            Longitude = longitude;
            IsPositionOfInterest = isPositionOfInterest;
        }
    }

    private void Awake()
    {
        statusText = statusTextObject?.GetComponent<TextMeshProUGUI>();
    }

    private void Start()
    {
        if (startLoggingOnAwake)
        {
            StartLogging();
        }
    }

    public void StartLogging()
    {
        if (isLogging)
        {
            UpdateStatus("Logging already active.");
            return;
        }

        string directoryPath = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.UserProfile), logDirectory);
        Directory.CreateDirectory(directoryPath);

        string timestamp = DateTime.Now.ToString("yyyy-MM-dd_HH-mm-ss");
        logFilePath = Path.Combine(directoryPath, $"{csvFilePrefix}_{timestamp}.csv");

        logEntries.Clear();
        latestLoggedEntry = null;

        File.WriteAllText(logFilePath, "timestamp,latitude,longitude,is_position_of_interest\n");

        TcpMessageReceiver.gpsReceived.AddListener(OnGpsReceived);
        isLogging = true;

        UpdateStatus($"Logging started: {Path.GetFileName(logFilePath)}");
    }

    public void StopLogging()
    {
        if (!isLogging)
        {
            UpdateStatus("Logging is not currently active.");
            return;
        }

        TcpMessageReceiver.gpsReceived.RemoveListener(OnGpsReceived);
        isLogging = false;

        UpdateStatus("Logging stopped.");
    }

    public void MarkMostRecentPositionAsPOI()
    {
        if (!isLogging)
        {
            UpdateStatus("Cannot mark POI while logging is stopped.");
            return;
        }

        if (latestLoggedEntry == null)
        {
            UpdateStatus("No GPS position has been logged yet.");
            return;
        }

        latestLoggedEntry.IsPositionOfInterest = true;
        RewriteLogFile();

        UpdateStatus("Most recent position marked as a position of interest.");
    }

    private void OnGpsReceived(string message)
    {
        if (!isLogging || string.IsNullOrWhiteSpace(message))
        {
            return;
        }

        string[] parts = message.Split(';');
        if (parts.Length < 3)
        {
            return;
        }

        bool latParsed = double.TryParse(parts[1], out double latitude);
        bool lonParsed = double.TryParse(parts[2], out double longitude);

        if (!latParsed || !lonParsed)
        {
            UpdateStatus("Received malformed GPS message.");
            return;
        }

        latestLoggedEntry = new GpsLogEntry(DateTime.Now, latitude, longitude, false);
        logEntries.Add(latestLoggedEntry);
        AppendEntryToFile(latestLoggedEntry);

        UpdateStatus($"Logged GPS: {latitude.ToString(CultureInfo.InvariantCulture)}, {longitude.ToString(CultureInfo.InvariantCulture)}");
    }

    private void AppendEntryToFile(GpsLogEntry entry)
    {
        if (string.IsNullOrEmpty(logFilePath))
        {
            return;
        }

        string csvLine = string.Format(
            CultureInfo.InvariantCulture,
            "{0},{1},{2},{3}\n",
            entry.Timestamp.ToString("O"),
            entry.Latitude.ToString(CultureInfo.InvariantCulture),
            entry.Longitude.ToString(CultureInfo.InvariantCulture),
            entry.IsPositionOfInterest ? "true" : "false");

        File.AppendAllText(logFilePath, csvLine);
    }

    private void RewriteLogFile()
    {
        if (string.IsNullOrEmpty(logFilePath))
        {
            return;
        }

        using (StreamWriter writer = new StreamWriter(logFilePath, false))
        {
            writer.WriteLine("timestamp,latitude,longitude,is_position_of_interest");

            foreach (GpsLogEntry entry in logEntries)
            {
                writer.WriteLine(string.Format(
                    CultureInfo.InvariantCulture,
                    "{0},{1},{2},{3}",
                    entry.Timestamp.ToString("O"),
                    entry.Latitude.ToString(CultureInfo.InvariantCulture),
                    entry.Longitude.ToString(CultureInfo.InvariantCulture),
                    entry.IsPositionOfInterest ? "true" : "false"));
            }
        }
    }

    private void UpdateStatus(string message)
    {
        if (statusText != null)
        {
            statusText.text = message;
        }
    }

    private void OnDestroy()
    {
        if (isLogging)
        {
            TcpMessageReceiver.gpsReceived.RemoveListener(OnGpsReceived);
        }
    }
}
