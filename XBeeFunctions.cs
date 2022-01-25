using System;
using System.Collections.Generic;
using System.IO.Ports;

namespace SpasticityClient
{
    public class XBeePacket
    {
        public string StartDelimiter { get; set; }
        public int Length { get; set; }
        public string RSSI { get; set; }
        public string FrameType { get; set; }
        public string Address16bit { get; set; }
        public string ReceiveOption { get; set; }
        public List<string> Data { get; set; }
        public string CheckSum { get; set; }
    }

    public static class XBeeFunctions
    {
        // Why need a 2 string dictionary with source 16 addresses?
        public static Dictionary<string, string> Source16Addresses = new Dictionary<string, string>();

        // Create a list of lists of strings by following function taking packet hex data, left hex data and total expected char length
        public static List<List<string>> ParseRFDataHex(List<string> packetHexData, List<string> leftHexData, int totalExpectedCharLength)
        {
            // Initialize list of list of strings
            List<List<string>> returnHex = new List<List<string>>();
            var noNextStart = false;
            // Append packethexdata to lefthex data
            leftHexData.AddRange(packetHexData);

            while (leftHexData.Count >= totalExpectedCharLength)
            {
                bool searchNext = true;
                int searchIdx = 1;
                while (searchNext)
                {
                    searchNext = false;
                    var idx = leftHexData.IndexOf("7E", searchIdx);
                    if (idx >= 0 && leftHexData.Count > idx + 2 && leftHexData[idx + 1] == "00" && leftHexData[idx + 2] == "41")
                    {
                        var parsedList = leftHexData.GetRange(0, idx);
                        returnHex.Add(parsedList);
                        leftHexData.RemoveRange(0, idx);
                    }
                    else
                    {
                        if (idx < 0)
                        {
                            searchNext = false;
                            noNextStart = true;
                        }
                        else
                        {
                            leftHexData.RemoveRange(0, idx);
                            searchNext = true;
                        }
                    }
                }

                if (noNextStart)
                    break;
            }
            return returnHex;
        }

        // Takes hexFull and parses it into xBee packets
        public static string ParsePacketHex(List<string> hexFull, List<XBeePacket> packets)
        {
            var leftHex = string.Empty;
            var isWrongStart = false;

            while (hexFull.Count > 15)
            {
                if (hexFull[0] == "7E")
                {
                    var length = int.Parse(hexFull[1] + hexFull[2], System.Globalization.NumberStyles.HexNumber);
                    if (length != 74)
                    {
                        isWrongStart = true;
                    }
                    // Parse contents into XBeePacket vars as above
                    else
                    {
                        var frameType = hexFull[3];

                        if (frameType == "81")
                        {
                            if (hexFull.Count < length + 4)
                                break;

                            var packetDataBytes = hexFull.GetRange(4, length);

                            var source16Addess = string.Join("", packetDataBytes.GetRange(0, 2));

                            var RSSI = packetDataBytes[2];
                            var receiveOption = packetDataBytes[3];
                            var data = packetDataBytes.GetRange(4, length - 5);
                            var checkSum = packetDataBytes[length - 1];

                            XBeePacket xbeePacket = new XBeePacket();
                            xbeePacket.StartDelimiter = hexFull[0];
                            xbeePacket.Length = length;
                            xbeePacket.FrameType = frameType;
                            xbeePacket.Address16bit = source16Addess;
                            xbeePacket.ReceiveOption = receiveOption;
                            xbeePacket.Data = data;
                            xbeePacket.CheckSum = checkSum;
                            packets.Add(xbeePacket);
                            hexFull.RemoveRange(0, 4 + length);

                        }
                        else
                            isWrongStart = true;
                    }
                }
                else
                {
                    isWrongStart = true;
                }

                if (isWrongStart)
                {
                    if (hexFull.Count > 1)
                    {
                        var idx = hexFull.IndexOf("7E", 1);
                        if (idx >= 0)
                        {
                            hexFull.RemoveRange(0, idx);
                        }
                        else
                        {
                            break;
                        }
                    }
                }
            }
            if (hexFull.Count > 0)
                return string.Join("-", hexFull) + "-";
            else
                return "";
        }

        // Get names of usable ports
        public static List<string> GetPortNamesByBaudrate(int baudRate)
        {
            List<string> portNames = new List<string>();

            foreach (string portName in SerialPort.GetPortNames())
            {
                using (SerialPort serialPort = new SerialPort(portName, baudRate, Parity.None, 8, StopBits.One))
                {
                    try
                    {
                        serialPort.Open();
                        portNames.Add(portName);
                    }
                    catch (Exception ex)
                    {
                        if (ex.Message.Contains("Access to the port") && ex.Message.Contains("is denied."))
                        {
                            portNames.Add(portName);
                        }
                    }
                    finally
                    {
                        serialPort.Close();
                    }
                }
            }
            if (portNames.Count > 0)
                portNames.Insert(0, "");
            return portNames;
        }
    }
}
