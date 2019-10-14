using System;

using Tech_MCS_SDK;
using Tech_MCS_Angle_SDK;

using ROS2;
using System.Numerics;

namespace TechMCS_ROS2_console
{
    class Program
    {
        private static cMCSHub device;
        private const int dataLength = 100;
        static int numData = 9;
        private static UInt16[,] intDataArray1 = new UInt16[9, dataLength];
        static bool capturando = false;
        static cMCSSensor sensor;
        
        private Node node;
        private static IPublisher<sensor_msgs.msg.Imu> imu_data_pub;
        
        private static void startDevice()
        {
            if (device != null)
                device.Disconnect();

            device = new cMCSHub();
            sensor = device.Sensors[0];

            FUNCTION_RESULT result = device.AutoDetectPort();
            if (result == FUNCTION_RESULT.FunctionOK)
            {
                result = device.Connect();
                capturando = true;
                setNoResetFrameReceivedEvent(true);
                device.StartCapture(CAPTURE_INFORMATION.DigitalData, false, false, false);
            }
        }

        private void stopDevice()
        {
            capturando = false;
            setNoResetFrameReceivedEvent(false);
            device.StopCapture();
        }

        public static void NewFrameReceived(object sender, EventArgs e)
        {
            float[] tempFloatData = new float[9];
            UInt16[] tempIntData = new UInt16[9];
            int numFrame = device.getNumFrames() - 1;
            
            tempIntData = sensor.getDigitalFrame(numFrame).toArray();
            
            for (int j = 0; j < numData; ++j)
            {
                intDataArray1[j, dataLength - 1] = tempIntData[j];
                Array.Copy(intDataArray1, dataLength * j + 1, intDataArray1, dataLength * j, dataLength - 1);
            }

            for (int col = 0; col < dataLength; col++)
            {
                sensor_msgs.msg.Imu imu_data_msg = new sensor_msgs.msg.Imu();

                imu_data_msg.Linear_acceleration.X = intDataArray1[col, 0];
                imu_data_msg.Linear_acceleration.Y = intDataArray1[col, 1];
                imu_data_msg.Linear_acceleration.Z = intDataArray1[col, 2];
                imu_data_msg.Angular_velocity.X = intDataArray1[col, 3];
                imu_data_msg.Angular_velocity.Y = intDataArray1[col, 4];
                imu_data_msg.Angular_velocity.Z = intDataArray1[col, 5];
                Quaternion q = Quaternion.CreateFromYawPitchRoll(intDataArray1[col, 6], intDataArray1[col, 7], intDataArray1[col, 8]);
                imu_data_msg.Orientation.X = q.X;
                imu_data_msg.Orientation.Y = q.Y;
                imu_data_msg.Orientation.Z = q.Z;
                imu_data_msg.Orientation.W = q.W;

                imu_data_pub.Publish(imu_data_msg);
            }
        }

        public static void setNoResetFrameReceivedEvent(bool setNoReset)
        {
            if (setNoReset)
                device.DigitalFrameReceived += new EventHandler(NewFrameReceived);
            else
                device.DigitalFrameReceived -= new EventHandler(NewFrameReceived);
        }

        static void Main(string[] args)
        {
            Console.WriteLine("Hello World!");
            RCLdotnet.Init();
            INode node = RCLdotnet.CreateNode("imu_publisher");
            imu_data_pub = node.CreatePublisher<sensor_msgs.msg.Imu>("techMCS_imu_data");
            startDevice();
        }
    }
}
