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
        public void Read(int keepRecords, ChartModel chartModel)
        {
            try
            {
                serialPort.Open();
                //var nowstart = DateTime.Now;
                var remainHex = string.Empty;
                var packetRemainData = new List<string>();
                
                float timeDiff = 0;
                float lastElapsedTime = 0;
                float lastAngle = 0;
                float angleDiff = 0;
                float angVel = 0;

                float forceDiff = 0;
                float initialForce = 0;

                var counter = 1;
                var loopIndex = 1;
                int angleBufferSize = 10;
                List<float> angleArray = new List<float>();
                List<float> angVelArray = new List<float>();
                List<float> forceCalArray = new List<float>();
                //var nowstart = DateTime.Now;

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

                                    #region IMU A Velocity and Orientation
                                    ////convert IMU angular velocity in x, y, z --- A
                                    //var AGVLx2MSB_A = Convert.ToByte(data[12], 16);
                                    //var AGVLx2LSB_A = Convert.ToByte(data[13], 16);
                                    //var AGVLx1MSB_A = Convert.ToByte(data[14], 16);
                                    //var AGVLx1LSB_A = Convert.ToByte(data[15], 16);

                                    //var AGVLy2MSB_A = Convert.ToByte(data[16], 16);
                                    //var AGVLy2LSB_A = Convert.ToByte(data[17], 16);
                                    //var AGVLy1MSB_A = Convert.ToByte(data[18], 16);
                                    //var AGVLy1LSB_A = Convert.ToByte(data[19], 16);

                                    //var AGVLz2MSB_A = Convert.ToByte(data[20], 16);
                                    //var AGVLz2LSB_A = Convert.ToByte(data[21], 16);
                                    //var AGVLz1MSB_A = Convert.ToByte(data[22], 16);
                                    //var AGVLz1LSB_A = Convert.ToByte(data[23], 16);

                                    ////convert IMU orientation in x, y, z --- A
                                    //var ORIEx2MSB_A = Convert.ToByte(data[24], 16);
                                    //var ORIEx2LSB_A = Convert.ToByte(data[25], 16);
                                    //var ORIEx1MSB_A = Convert.ToByte(data[26], 16);
                                    //var ORIEx1LSB_A = Convert.ToByte(data[27], 16);

                                    //var ORIEy2MSB_A = Convert.ToByte(data[28], 16);
                                    //var ORIEy2LSB_A = Convert.ToByte(data[29], 16);
                                    //var ORIEy1MSB_A = Convert.ToByte(data[30], 16);
                                    //var ORIEy1LSB_A = Convert.ToByte(data[31], 16);

                                    //var ORIEz2MSB_A = Convert.ToByte(data[32], 16);
                                    //var ORIEz2LSB_A = Convert.ToByte(data[33], 16);
                                    //var ORIEz1MSB_A = Convert.ToByte(data[34], 16);
                                    //var ORIEz1LSB_A = Convert.ToByte(data[35], 16);
                                    //#endregion

                                    //#region IMU B Velocity and Orientation
                                    ////convert IMU angular velocity in x, y, z --- B
                                    //var AGVLx2MSB_B = Convert.ToByte(data[36], 16);
                                    //var AGVLx2LSB_B = Convert.ToByte(data[37], 16);
                                    //var AGVLx1MSB_B = Convert.ToByte(data[38], 16);
                                    //var AGVLx1LSB_B = Convert.ToByte(data[39], 16);

                                    //var AGVLy2MSB_B = Convert.ToByte(data[40], 16);
                                    //var AGVLy2LSB_B = Convert.ToByte(data[41], 16);
                                    //var AGVLy1MSB_B = Convert.ToByte(data[42], 16);
                                    //var AGVLy1LSB_B = Convert.ToByte(data[43], 16);

                                    //var AGVLz2MSB_B = Convert.ToByte(data[44], 16);
                                    //var AGVLz2LSB_B = Convert.ToByte(data[45], 16);
                                    //var AGVLz1MSB_B = Convert.ToByte(data[46], 16);
                                    //var AGVLz1LSB_B = Convert.ToByte(data[47], 16);

                                    ////convert IMU orientation in x, y, z --- B
                                    //var ORIEx2MSB_B = Convert.ToByte(data[48], 16);
                                    //var ORIEx2LSB_B = Convert.ToByte(data[49], 16);
                                    //var ORIEx1MSB_B = Convert.ToByte(data[50], 16);
                                    //var ORIEx1LSB_B = Convert.ToByte(data[51], 16);

                                    //var ORIEy2MSB_B = Convert.ToByte(data[52], 16);
                                    //var ORIEy2LSB_B = Convert.ToByte(data[53], 16);
                                    //var ORIEy1MSB_B = Convert.ToByte(data[54], 16);
                                    //var ORIEy1LSB_B = Convert.ToByte(data[55], 16);

                                    //var ORIEz2MSB_B = Convert.ToByte(data[56], 16);
                                    //var ORIEz2LSB_B = Convert.ToByte(data[57], 16);
                                    //var ORIEz1MSB_B = Convert.ToByte(data[58], 16);
                                    //var ORIEz1LSB_B = Convert.ToByte(data[59], 16);
                                    #endregion

                                    #region EMG and Force
                                    //convert rectified EMG
                                    var EMGMSB = Convert.ToByte(data[12], 16);
                                    var EMGLSB = Convert.ToByte(data[13], 16);

                                    //convert force
                                    var FORMSB = Convert.ToByte(data[14], 16);
                                    var FORLSB = Convert.ToByte(data[15], 16);
                                    #endregion

                                    #region Quaternion Values
                                    ////convert quaternion values --- A
                                    //var QUATw2MSB_A = Convert.ToByte(data[64], 16);
                                    //var QUATw2LSB_A = Convert.ToByte(data[65], 16);
                                    //var QUATw1MSB_A = Convert.ToByte(data[66], 16);
                                    //var QUATw1LSB_A = Convert.ToByte(data[67], 16);

                                    //var QUATx2MSB_A = Convert.ToByte(data[68], 16);
                                    //var QUATx2LSB_A = Convert.ToByte(data[69], 16);
                                    //var QUATx1MSB_A = Convert.ToByte(data[70], 16);
                                    //var QUATx1LSB_A = Convert.ToByte(data[71], 16);

                                    //var QUATy2MSB_A = Convert.ToByte(data[72], 16);
                                    //var QUATy2LSB_A = Convert.ToByte(data[73], 16);
                                    //var QUATy1MSB_A = Convert.ToByte(data[74], 16);
                                    //var QUATy1LSB_A = Convert.ToByte(data[75], 16);

                                    //var QUATz2MSB_A = Convert.ToByte(data[76], 16);
                                    //var QUATz2LSB_A = Convert.ToByte(data[77], 16);
                                    //var QUATz1MSB_A = Convert.ToByte(data[78], 16);
                                    //var QUATz1LSB_A = Convert.ToByte(data[79], 16);

                                    ////convert quaternion values --- B
                                    //var QUATw2MSB_B = Convert.ToByte(data[80], 16);
                                    //var QUATw2LSB_B = Convert.ToByte(data[81], 16);
                                    //var QUATw1MSB_B = Convert.ToByte(data[82], 16);
                                    //var QUATw1LSB_B = Convert.ToByte(data[83], 16);

                                    //var QUATx2MSB_B = Convert.ToByte(data[84], 16);
                                    //var QUATx2LSB_B = Convert.ToByte(data[85], 16);
                                    //var QUATx1MSB_B = Convert.ToByte(data[86], 16);
                                    //var QUATx1LSB_B = Convert.ToByte(data[87], 16);

                                    //var QUATy2MSB_B = Convert.ToByte(data[88], 16);
                                    //var QUATy2LSB_B = Convert.ToByte(data[89], 16);
                                    //var QUATy1MSB_B = Convert.ToByte(data[90], 16);
                                    //var QUATy1LSB_B = Convert.ToByte(data[91], 16);

                                    //var QUATz2MSB_B = Convert.ToByte(data[92], 16);
                                    //var QUATz2LSB_B = Convert.ToByte(data[93], 16);
                                    //var QUATz1MSB_B = Convert.ToByte(data[94], 16);
                                    //var QUATz1LSB_B = Convert.ToByte(data[95], 16);
                                    #endregion

                                    #region Potentiometer Values
                                    //convert potentiometer edge computer angle and angvel values. Note POTANGVEL values are not used because the packets are coming out one byte short and 
                                    // therefore there is a problem with angVel. Instead AngVel is computed here instead of edge computed on the device's Feather M0
                                    var POTANGLEMSB = Convert.ToByte(data[16], 16);
                                    var POTANGLELSB = Convert.ToByte(data[17], 16);
                                    #endregion

                                    #region MSB LSB combination
                                    //float angVelX_A = (long)((AGVLx2MSB_A & 0xFF) << 24 | (AGVLx2LSB_A & 0xFF) << 16 | (AGVLx1MSB_A & 0xFF) << 8 | (AGVLx1LSB_A & 0xFF));
                                    //float angVelY_A = (long)((AGVLy2MSB_A & 0xFF) << 24 | (AGVLy2LSB_A & 0xFF) << 16 | (AGVLy1MSB_A & 0xFF) << 8 | (AGVLy1LSB_A & 0xFF));
                                    //float angVelZ_A = (long)((AGVLz2MSB_A & 0xFF) << 24 | (AGVLz2LSB_A & 0xFF) << 16 | (AGVLz1MSB_A & 0xFF) << 8 | (AGVLz1LSB_A & 0xFF));

                                    //float orientX_A = (long)((ORIEx2MSB_A & 0xFF) << 24 | (ORIEx2LSB_A & 0xFF) << 16 | (ORIEx1MSB_A & 0xFF) << 8 | (ORIEx1LSB_A & 0xFF));
                                    //float orientY_A = (long)((ORIEy2MSB_A & 0xFF) << 24 | (ORIEy2LSB_A & 0xFF) << 16 | (ORIEy1MSB_A & 0xFF) << 8 | (ORIEy1LSB_A & 0xFF));
                                    //float orientZ_A = (long)((ORIEz2MSB_A & 0xFF) << 24 | (ORIEz2LSB_A & 0xFF) << 16 | (ORIEz1MSB_A & 0xFF) << 8 | (ORIEz1LSB_A & 0xFF));

                                    //float angVelX_B = (long)((AGVLx2MSB_B & 0xFF) << 24 | (AGVLx2LSB_B & 0xFF) << 16 | (AGVLx1MSB_B & 0xFF) << 8 | (AGVLx1LSB_B & 0xFF));
                                    //float angVelY_B = (long)((AGVLy2MSB_B & 0xFF) << 24 | (AGVLy2LSB_B & 0xFF) << 16 | (AGVLy1MSB_B & 0xFF) << 8 | (AGVLy1LSB_B & 0xFF));
                                    //float angVelZ_B = (long)((AGVLz2MSB_B & 0xFF) << 24 | (AGVLz2LSB_B & 0xFF) << 16 | (AGVLz1MSB_B & 0xFF) << 8 | (AGVLz1LSB_B & 0xFF));

                                    //float orientX_B = (long)((ORIEx2MSB_B & 0xFF) << 24 | (ORIEx2LSB_B & 0xFF) << 16 | (ORIEx1MSB_B & 0xFF) << 8 | (ORIEx1LSB_B & 0xFF));
                                    //float orientY_B = (long)((ORIEy2MSB_B & 0xFF) << 24 | (ORIEy2LSB_B & 0xFF) << 16 | (ORIEy1MSB_B & 0xFF) << 8 | (ORIEy1LSB_B & 0xFF));
                                    //float orientZ_B = (long)((ORIEz2MSB_B & 0xFF) << 24 | (ORIEz2LSB_B & 0xFF) << 16 | (ORIEz1MSB_B & 0xFF) << 8 | (ORIEz1LSB_B & 0xFF));

                                    //float quatW_A = (long)((QUATw2MSB_A & 0xFF) << 24 | (QUATw2LSB_A & 0xFF) << 16 | (QUATw1MSB_A & 0xFF) << 8 | (QUATw1LSB_A & 0xFF)) / 1000;
                                    //float quatX_A = (long)((QUATx2MSB_A & 0xFF) << 24 | (QUATx2LSB_A & 0xFF) << 16 | (QUATx1MSB_A & 0xFF) << 8 | (QUATx1LSB_A & 0xFF)) / 1000;
                                    //float quatY_A = (long)((QUATy2MSB_A & 0xFF) << 24 | (QUATy2LSB_A & 0xFF) << 16 | (QUATy1MSB_A & 0xFF) << 8 | (QUATy1LSB_A & 0xFF)) / 1000;
                                    //float quatZ_A = (long)((QUATz2MSB_A & 0xFF) << 24 | (QUATz2LSB_A & 0xFF) << 16 | (QUATz1MSB_A & 0xFF) << 8 | (QUATz1LSB_A & 0xFF)) / 1000;

                                    //float quatW_B = (long)((QUATw2MSB_B & 0xFF) << 24 | (QUATw2LSB_B & 0xFF) << 16 | (QUATw1MSB_B & 0xFF) << 8 | (QUATw1LSB_B & 0xFF)) / 1000;
                                    //float quatX_B = (long)((QUATx2MSB_B & 0xFF) << 24 | (QUATx2LSB_B & 0xFF) << 16 | (QUATx1MSB_B & 0xFF) << 8 | (QUATx1LSB_B & 0xFF)) / 1000;
                                    //float quatY_B = (long)((QUATy2MSB_B & 0xFF) << 24 | (QUATy2LSB_B & 0xFF) << 16 | (QUATy1MSB_B & 0xFF) << 8 | (QUATy1LSB_B & 0xFF)) / 1000;
                                    //float quatZ_B = (long)((QUATz2MSB_B & 0xFF) << 24 | (QUATz2LSB_B & 0xFF) << 16 | (QUATz1MSB_B & 0xFF) << 8 | (QUATz1LSB_B & 0xFF)) / 1000;

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