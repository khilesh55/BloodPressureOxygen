using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Threading;
using System.IO.Ports;

namespace SpasticityClient
{
    public class XBeeData : IDisposable
    {
        #region public properties
        public bool IsCancelled { get; set; }
        #endregion

        // Initialize a serial port
        private SerialPort serialPort = null;

        #region construct

        // 2 methods to construct an XBeeData, w/ or w/o baudrate
        public XBeeData(string portName)
        {
            serialPort = new SerialPort(portName, 57600, Parity.None, 8, StopBits.One);
        }

        public XBeeData(string portName, int baudrate)
        {
            serialPort = new SerialPort(portName, baudrate, Parity.None, 8, StopBits.One);
        }
        #endregion

        // Dispose of the serial port
        public void Dispose()
        {
            Stop();
        }

        // Read from serial port
        public void Read(int keepRecords, ChartModel chartModel)
        {
            try
            {
                serialPort.Open();
                var remainHex = string.Empty;
                var packetRemainData = new List<string>();

                //infinite loop will keep running and adding to EMGInfo until IsCancelled is set to true
                while (IsCancelled == false)
                {
                    //Check if any bytes of data received in serial buffer
                    var totalbytes = serialPort.BytesToRead;

                    Thread.Sleep(30);

                    if (totalbytes > 0)
                    {
                        //Load all the serial data to buffer
                        var buffer = new byte[totalbytes];
                        serialPort.Read(buffer, 0, buffer.Length);

                        //convert bytes to hex to better visualize and parse. 
                        //TODO: it can be updated in the future to parse with byte to increase preformance if needed
                        var hexFull = BitConverter.ToString(buffer);

                        //remainhex is empty string
                        hexFull = remainHex + hexFull;

                        var packets = new List<XBeePacket>();

                        //remainHex is all that is left when legitimate packets have been added to packets
                        remainHex = XBeeFunctions.ParsePacketHex(hexFull.Split('-').ToList(), packets);

                        foreach (var packet in packets)
                        {
                            //Total transmitted data is 30 byte long. 1 more byte should be checksum. prefixchar is the extra header due to API Mode
                            int prefixCharLength = 8;
                            int byteArrayLength = 69;
                            int checkSumLength = 1;
                            int totalExpectedCharLength = prefixCharLength + byteArrayLength + checkSumLength;

                            //Based on above variables to parse data coming from SerialPort. Next fun is performed sequentially to all packets
                            var packetDatas = XBeeFunctions.ParseRFDataHex(packet.Data, packetRemainData, totalExpectedCharLength);

                            foreach (var packetData in packetDatas)
                            {
                                //Make sure it's 25 charactors long. It's same as the arduino receiver code for checking the length. This was previously compared to totalExpectedCharLength but looks like packetDatas - packetData only contains the data part anyway therefore compare to byteArrayLength
                                //Also modify data defn to be packetData itself
                                if (packetData.Count == byteArrayLength)
                                {
                                    var data = packetData;

                                    #region Convert string to byte for later MSB and LSB combination- 16 bit to 18 bit
                                    //convert timestamp
                                    var TIME2MSB = Convert.ToByte(data[8], 16);
                                    var TIME2LSB = Convert.ToByte(data[9], 16);
                                    var TIME1MSB = Convert.ToByte(data[10], 16);
                                    var TIME1LSB = Convert.ToByte(data[11], 16);

                                    #region IMU A Velocity and Orientation
                                    //convert IMU angular velocity in x, y, z --- A
                                    var AGVLx2MSB_A = Convert.ToByte(data[12], 16);
                                    var AGVLx2LSB_A = Convert.ToByte(data[13], 16);
                                    var AGVLx1MSB_A = Convert.ToByte(data[14], 16);
                                    var AGVLx1LSB_A = Convert.ToByte(data[15], 16);

                                    var AGVLy2MSB_A = Convert.ToByte(data[16], 16);
                                    var AGVLy2LSB_A = Convert.ToByte(data[17], 16);
                                    var AGVLy1MSB_A = Convert.ToByte(data[18], 16);
                                    var AGVLy1LSB_A = Convert.ToByte(data[19], 16);

                                    var AGVLz2MSB_A = Convert.ToByte(data[20], 16);
                                    var AGVLz2LSB_A = Convert.ToByte(data[21], 16);
                                    var AGVLz1MSB_A = Convert.ToByte(data[22], 16);
                                    var AGVLz1LSB_A = Convert.ToByte(data[23], 16);

                                    //convert IMU orientation in x, y, z --- A
                                    var ORIEx2MSB_A = Convert.ToByte(data[24], 16);
                                    var ORIEx2LSB_A = Convert.ToByte(data[25], 16);
                                    var ORIEx1MSB_A = Convert.ToByte(data[26], 16);
                                    var ORIEx1LSB_A = Convert.ToByte(data[27], 16);

                                    var ORIEy2MSB_A = Convert.ToByte(data[28], 16);
                                    var ORIEy2LSB_A = Convert.ToByte(data[29], 16);
                                    var ORIEy1MSB_A = Convert.ToByte(data[30], 16);
                                    var ORIEy1LSB_A = Convert.ToByte(data[31], 16);

                                    var ORIEz2MSB_A = Convert.ToByte(data[32], 16);
                                    var ORIEz2LSB_A = Convert.ToByte(data[33], 16);
                                    var ORIEz1MSB_A = Convert.ToByte(data[34], 16);
                                    var ORIEz1LSB_A = Convert.ToByte(data[35], 16);
                                    #endregion

                                    #region IMU B Velocity and Orientation
                                    //convert IMU angular velocity in x, y, z --- B
                                    var AGVLx2MSB_B = Convert.ToByte(data[36], 16);
                                    var AGVLx2LSB_B = Convert.ToByte(data[37], 16);
                                    var AGVLx1MSB_B = Convert.ToByte(data[38], 16);
                                    var AGVLx1LSB_B = Convert.ToByte(data[39], 16);

                                    var AGVLy2MSB_B = Convert.ToByte(data[40], 16);
                                    var AGVLy2LSB_B = Convert.ToByte(data[41], 16);
                                    var AGVLy1MSB_B = Convert.ToByte(data[42], 16);
                                    var AGVLy1LSB_B = Convert.ToByte(data[43], 16);

                                    var AGVLz2MSB_B = Convert.ToByte(data[44], 16);
                                    var AGVLz2LSB_B = Convert.ToByte(data[45], 16);
                                    var AGVLz1MSB_B = Convert.ToByte(data[46], 16);
                                    var AGVLz1LSB_B = Convert.ToByte(data[47], 16);

                                    //convert IMU orientation in x, y, z --- B
                                    var ORIEx2MSB_B = Convert.ToByte(data[48], 16);
                                    var ORIEx2LSB_B = Convert.ToByte(data[49], 16);
                                    var ORIEx1MSB_B = Convert.ToByte(data[50], 16);
                                    var ORIEx1LSB_B = Convert.ToByte(data[51], 16);

                                    var ORIEy2MSB_B = Convert.ToByte(data[52], 16);
                                    var ORIEy2LSB_B = Convert.ToByte(data[53], 16);
                                    var ORIEy1MSB_B = Convert.ToByte(data[54], 16);
                                    var ORIEy1LSB_B = Convert.ToByte(data[55], 16);

                                    var ORIEz2MSB_B = Convert.ToByte(data[56], 16);
                                    var ORIEz2LSB_B = Convert.ToByte(data[57], 16);
                                    var ORIEz1MSB_B = Convert.ToByte(data[58], 16);
                                    var ORIEz1LSB_B = Convert.ToByte(data[59], 16);
                                    #endregion

                                    //convert rectified EMG
                                    var EMGMSB = Convert.ToByte(data[60], 16);
                                    var EMGLSB = Convert.ToByte(data[61], 16);

                                    //convert force
                                    var FORMSB = Convert.ToByte(data[62], 16);
                                    var FORLSB = Convert.ToByte(data[63], 16);
                                    #endregion

                                    #region MSB LSB combination
                                    float elapsedTime = (long)((TIME2MSB & 0xFF) << 24 | (TIME2LSB & 0xFF) << 16 | (TIME1MSB & 0xFF) << 8 | (TIME1LSB & 0xFF));

                                    float angVelX = (long)((AGVLx2MSB_B & 0xFF) << 24 | (AGVLx2LSB_B & 0xFF) << 16 | (AGVLx1MSB_B & 0xFF) << 8 | (AGVLx1LSB_B & 0xFF));
                                    float angVelY = (long)((AGVLy2MSB_B & 0xFF) << 24 | (AGVLy2LSB_B & 0xFF) << 16 | (AGVLy1MSB_B & 0xFF) << 8 | (AGVLy1LSB_B & 0xFF));
                                    float angVelZ = (long)((AGVLz2MSB_B & 0xFF) << 24 | (AGVLz2LSB_B & 0xFF) << 16 | (AGVLz1MSB_B & 0xFF) << 8 | (AGVLz1LSB_B & 0xFF));

                                    float orientX = (long)((ORIEx2MSB_B & 0xFF) << 24 | (ORIEx2LSB_B & 0xFF) << 16 | (ORIEx1MSB_B & 0xFF) << 8 | (ORIEx1LSB_B & 0xFF));
                                    float orientY = (long)((ORIEy2MSB_B & 0xFF) << 24 | (ORIEy2LSB_B & 0xFF) << 16 | (ORIEy1MSB_B & 0xFF) << 8 | (ORIEy1LSB_B & 0xFF));
                                    float orientZ = (long)((ORIEz2MSB_B & 0xFF) << 24 | (ORIEz2LSB_B & 0xFF) << 16 | (ORIEz1MSB_B & 0xFF) << 8 | (ORIEz1LSB_B & 0xFF));

                                    float emg = (int)((EMGMSB & 0xFF) << 8 | (EMGLSB & 0xFF));
                                    float force = (int)((FORMSB & 0xFF) << 8 | (FORLSB & 0xFF));
                                    #endregion

                                    #region Send data to chart model
                                    if (chartModel.EMGValues.Count > keepRecords)
                                    {
                                        chartModel.EMGValues.RemoveAt(0);
                                        chartModel.AngleValues.RemoveAt(0);
                                        chartModel.AngularVelocityValues.RemoveAt(0);
                                        chartModel.ForceValues.RemoveAt(0);
                                    }

                                    var now = DateTime.Now;
                                    chartModel.SetAxisLimits(now);

                                    chartModel.EMGValues.Add(new MeasureModel
                                    {
                                        DateTime = now,
                                        Value = emg
                                    });
                                    chartModel.ForceValues.Add(new MeasureModel
                                    {
                                        DateTime = now,
                                        Value = force
                                    });
                                    chartModel.AngleValues.Add(new MeasureModel
                                    {
                                        DateTime = now,
                                        Value = orientX
                                    });
                                    chartModel.AngularVelocityValues.Add(new MeasureModel
                                    {
                                        DateTime = now,
                                        Value = angVelX
                                    });
                                    #endregion

                                    Thread.Sleep(30);
                                }
                            }
                        }
                    }
                }
            }
            finally
            {
                Stop();
            }
        }

        // Stop reading
        public void Stop()
        {
            if (serialPort != null)
            {
                try
                {
                    serialPort.Close();
                    serialPort.Dispose();
                }
                catch
                {
                }
            }
        }
    }
}
