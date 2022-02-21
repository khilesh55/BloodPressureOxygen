using System;
using System.Collections.Generic;
using System.Linq;
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

        public XBeeData(string portName)
        {
            serialPort = new SerialPort(portName, 57600, Parity.None, 8, StopBits.One);
        }

        #endregion

        // Dispose of the serial port
        public void Dispose()
        {
            Stop();
        }

        // Read from serial port
        public void StartIC(int keepRecords, ChartModel chartModel)
        {
            try
            {
                serialPort.Open();

                var remainHex = string.Empty;
                var packetRemainData = new List<string>();
                
                float timeDiff = 0;
                float lastElapsedTime = 0;
                float angVel = 0;

                float forceDiff = 0;
                float initialForce = 0;

                var counter = 1;
                var loopIndex = 1;

                List<float> angVelArray = new List<float>();
                List<float> forceCalArray = new List<float>();

                //infinite loop will keep running and adding to EMGInfo until IsCancelled is set to true
                while (IsCancelled == false)
                {
                    //Check if any bytes of data received in serial buffer
                    var totalbytes = serialPort.BytesToRead;
                    Thread.Sleep(30);

                    if (loopIndex == 1) { var nowstart = DateTime.Now; };
                    if (totalbytes > 0)
                    {
                        //Load all the serial data to buffer
                        var buffer = new byte[totalbytes];
                        var nowticks = DateTime.Now;
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
                            //Total transmitted data is [] byte long. 1 more byte should be checksum. prefixchar is the extra header due to API Mode
                            int prefixCharLength = 8;
                            int byteArrayLength = 10;
                            int checkSumLength = 1;
                            int totalExpectedCharLength = prefixCharLength + byteArrayLength + checkSumLength;

                            //Based on above variables to parse data coming from SerialPort. Next fun is performed sequentially to all packets
                            var packetDatas = XBeeFunctions.ParseRFDataHex(packet.Data, packetRemainData, totalExpectedCharLength);

                            foreach (var packetData in packetDatas)
                            {
                                //Make sure it's 25 charactors long. It's same as the arduino receiver code for checking the length. This was previously compared to totalExpectedCharLength but looks like packetDatas - packetData only contains the data part anyway therefore compare to byteArrayLength
                                //Also modify data defn to be packetData itself
                                if (packetData.Count == (prefixCharLength+byteArrayLength+checkSumLength))
                                {
                                    var data = packetData;

                                    #region Convert string to byte for later MSB and LSB combination- 16 bit to 8 bit

                                    #region Time
                                    //convert timestamp
                                    var TIME2MSB = Convert.ToByte(data[8], 16);
                                    var TIME2LSB = Convert.ToByte(data[9], 16);
                                    var TIME1MSB = Convert.ToByte(data[10], 16);
                                    var TIME1LSB = Convert.ToByte(data[11], 16);
                                    var BLOCMSB  = Convert.ToByte(data[12], 16);
                                    var BLOCLSB  = Convert.ToByte(data[13], 16);
                                    var BLOX2MSB = Convert.ToByte(data[14], 16);
                                    var BLOX2LSB = Convert.ToByte(data[15], 16);
                                    var BLOX1MSB = Convert.ToByte(data[16], 16);
                                    var BLOX1LSB = Convert.ToByte(data[17], 16);

                                    #endregion

                                    #region MSB LSB combination
                                    float elapsedTime = (long)((TIME2MSB & 0xFF) << 24 | (TIME2LSB & 0xFF) << 16 | (TIME1MSB & 0xFF) << 8 | (TIME1LSB & 0xFF));
                                    float bloodCon = (int)(BLOCMSB & 0xFF) << 8 | (BLOCLSB & 0xFF);
                                    float bloodOx = (long)((BLOX2MSB & 0xFF) << 24 | (BLOX2LSB & 0xFF) << 16 | (BLOX1MSB & 0xFF) << 8 | (BLOX1LSB & 0xFF));
                                    float angle = (int)((POTANGLEMSB & 0xFF) << 8 | (POTANGLELSB & 0xFF));
                                    #endregion

                                    #region Calculate angVel from angle
                                        timeDiff = elapsedTime - lastElapsedTime;
                                        angleDiff = angle - lastAngle;

                                        var angVelResult = Math.Abs(angleDiff /(timeDiff/1000));
                                        
                                        if (counter < (angleBufferSize+1))
                                        {
                                            angleArray.Add(angle);
                                            angVelArray.Add(0);
                                            angVel = 0;
                                        }
                                        else
                                        {   
                                            angleArray.Add(angle);
                                            angVelArray.Add(angVelResult);
                                            angVel = Math.Abs((angleArray[angleBufferSize] - angleArray[0])/(timeDiff*angleBufferSize/1000));
                                            angleArray.RemoveAt(0);
                                            angVelArray.RemoveAt(0);
                                        }

                                    angVel = MovingAverage(angleBufferSize, angVelArray);
                                    
                                    lastElapsedTime = elapsedTime;
                                    lastAngle = angle;
                                    #endregion

                                    #region Calibrate out starting force bias 
                                    if (counter < 20)
                                    {
                                        forceCalArray.Add(force);
                                    }
                                    initialForce = forceCalArray.Min();
                                    forceDiff = force - initialForce;
                                    #endregion

                                    #region Send data to chart model
                                    if (chartModel.EMGValues.Count > keepRecords)
                                    {
                                        chartModel.AngularVelocityValues.RemoveAt(0);
                                        chartModel.ForceValues.RemoveAt(0);
                                    }

                                    //var nowticks = DateTime.Now;

                                    chartModel.ForceValues.Add(new MeasureModel { DateTime = nowticks, Value = forceDiff });
                                    chartModel.AngularVelocityValues.Add(new MeasureModel { DateTime = nowticks, Value = angVel });
                                    chartModel.SetAxisLimits(nowticks);
                                    #endregion

                                    #region Send data to Excel collection
                                    chartModel.SessionDatas.Add(new SessionData
                                    {
                                        TimeStamp = (long)elapsedTime,
                                        Angle_deg = angle,
                                        AngVel_degpersec = angVel,
                                        EMG_mV = emg,
                                        Force_N = forceDiff
                                    }); ;
                                    #endregion

                                    counter++;
                                    loopIndex++;
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

        public void Read(int keepRecords, ChartModel chartModel)
        {
            try
            {
                serialPort.Open();

                var remainHex = string.Empty;
                var packetRemainData = new List<string>();
                
                float timeDiff = 0;
                float lastElapsedTime = 0;
                float angVel = 0;

                float forceDiff = 0;
                float initialForce = 0;

                var counter = 1;
                var loopIndex = 1;

                List<float> angVelArray = new List<float>();
                List<float> forceCalArray = new List<float>();

                //infinite loop will keep running and adding to EMGInfo until IsCancelled is set to true
                while (IsCancelled == false)
                {
                    //Check if any bytes of data received in serial buffer
                    var totalbytes = serialPort.BytesToRead;
                    Thread.Sleep(30);

                    if (loopIndex == 1) { var nowstart = DateTime.Now; };
                    if (totalbytes > 0)
                    {
                        //Load all the serial data to buffer
                        var buffer = new byte[totalbytes];
                        var nowticks = DateTime.Now;
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
                            //Total transmitted data is [] byte long. 1 more byte should be checksum. prefixchar is the extra header due to API Mode
                            int prefixCharLength = 8;
                            int byteArrayLength = 25;
                            int checkSumLength = 1;
                            int totalExpectedCharLength = prefixCharLength + byteArrayLength + checkSumLength;

                            //Based on above variables to parse data coming from SerialPort. Next fun is performed sequentially to all packets
                            var packetDatas = XBeeFunctions.ParseRFDataHex(packet.Data, packetRemainData, totalExpectedCharLength);

                            foreach (var packetData in packetDatas)
                            {
                                //Make sure it's 25 charactors long. It's same as the arduino receiver code for checking the length. This was previously compared to totalExpectedCharLength but looks like packetDatas - packetData only contains the data part anyway therefore compare to byteArrayLength
                                //Also modify data defn to be packetData itself
                                if (packetData.Count == (prefixCharLength+byteArrayLength+checkSumLength))
                                {
                                    var data = packetData;

                                    #region Convert string to byte for later MSB and LSB combination- 16 bit to 8 bit

                                    #region Time
                                    //convert timestamp
                                    var TIME2MSB = Convert.ToByte(data[8], 16);
                                    var TIME2LSB = Convert.ToByte(data[9], 16);
                                    var TIME1MSB = Convert.ToByte(data[10], 16);
                                    var TIME1LSB = Convert.ToByte(data[11], 16);
                                    #endregion

                                    #region EMG and Force
                                    //convert rectified EMG
                                    var EMGMSB = Convert.ToByte(data[12], 16);
                                    var EMGLSB = Convert.ToByte(data[13], 16);

                                    //convert force
                                    var FORMSB = Convert.ToByte(data[14], 16);
                                    var FORLSB = Convert.ToByte(data[15], 16);
                                    #endregion

                                    #region Potentiometer Values
                                    //convert potentiometer edge computer angle and angvel values. Note POTANGVEL values are not used because the packets are coming out one byte short and 
                                    // therefore there is a problem with angVel. Instead AngVel is computed here instead of edge computed on the device's Feather M0
                                    var POTANGLEMSB = Convert.ToByte(data[16], 16);
                                    var POTANGLELSB = Convert.ToByte(data[17], 16);
                                    #endregion

                                    #region MSB LSB combination

                                    float elapsedTime = (long)((TIME2MSB & 0xFF) << 24 | (TIME2LSB & 0xFF) << 16 | (TIME1MSB & 0xFF) << 8 | (TIME1LSB & 0xFF));
                                    float emg = (int)(EMGMSB & 0xFF) << 8 | (EMGLSB & 0xFF);
                                    float force = (int)((FORMSB & 0xFF) << 8 | (FORLSB & 0xFF));
                                    float angle = (int)((POTANGLEMSB & 0xFF) << 8 | (POTANGLELSB & 0xFF));
                                    #endregion

                                    #region Calculate angVel from angle
                                        timeDiff = elapsedTime - lastElapsedTime;
                                        angleDiff = angle - lastAngle;

                                        var angVelResult = Math.Abs(angleDiff /(timeDiff/1000));
                                        
                                        if (counter < (angleBufferSize+1))
                                        {
                                            angleArray.Add(angle);
                                            angVelArray.Add(0);
                                            angVel = 0;
                                        }
                                        else
                                        {   
                                            
                                            
                                            angleArray.Add(angle);
                                            angVelArray.Add(angVelResult);
                                            angVel = Math.Abs((angleArray[angleBufferSize] - angleArray[0])/(timeDiff*angleBufferSize/1000));
                                            angleArray.RemoveAt(0);
                                            angVelArray.RemoveAt(0);
                                        }

                                    angVel = MovingAverage(angleBufferSize, angVelArray);
                                    
                                    lastElapsedTime = elapsedTime;
                                    lastAngle = angle;
                                    #endregion

                                    #region Calibrate out starting force bias 
                                    if (counter < 20)
                                    {
                                        forceCalArray.Add(force);
                                    }
                                    initialForce = forceCalArray.Min();
                                    forceDiff = force - initialForce;
                                    #endregion

                                    #region Send data to chart model
                                    if (chartModel.EMGValues.Count > keepRecords)
                                    {
                                        chartModel.EMGValues.RemoveAt(0);
                                        chartModel.AngleValues.RemoveAt(0);
                                        chartModel.AngularVelocityValues.RemoveAt(0);
                                        chartModel.ForceValues.RemoveAt(0);
                                    }

                                    //var nowticks = DateTime.Now;

                                    chartModel.EMGValues.Add(new MeasureModel { DateTime = nowticks, Value = emg });
                                    chartModel.ForceValues.Add(new MeasureModel { DateTime = nowticks, Value = forceDiff });
                                    chartModel.AngleValues.Add(new MeasureModel { DateTime = nowticks, Value = angle });
                                    chartModel.AngularVelocityValues.Add(new MeasureModel { DateTime = nowticks, Value = angVel });
                                    
                                    chartModel.SetAxisLimits(nowticks);
                                    #endregion

                                    #region Send data to Excel collection
                                    chartModel.SessionDatas.Add(new SessionData
                                    {
                                        TimeStamp = (long)elapsedTime,
                                        Angle_deg = angle,
                                        AngVel_degpersec = angVel,
                                        EMG_mV = emg,
                                        Force_N = forceDiff
                                    }); ;
                                    #endregion

                                    counter++;
                                    loopIndex++;
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

        public void Write(int keepRecords, ChartModel chartModel)
        {
            try
            {
                serialPort.Open();
                var writeHex = string.Empty;
                //call writePacketHex;
            }
            finally
            {
                Stop();
            }
        }

        public float MovingAverage(int arrayLength, List<float> angleArray)
        {
            float movingSum = angleArray.Sum();
            float movingAverage = movingSum / arrayLength;
            return movingAverage;
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
#endregion