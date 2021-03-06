using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Windows;

namespace SpasticityClient
{
    class MainWindowViewModel: INotifyPropertyChanged
    {
        private ChartModel _chartModel;
        private string _portName;

        public List<string> PortNames { get; internal set; }

        public ChartModel ChartModel
        {
            get { return _chartModel; }
            set
            {
                _chartModel = value;
                NotifyPropertyChanged("ChartModel");
            }
        }

        public string PortName
        {
            get { return _portName; }
            set
            {
                _portName = value;
                ChartModel = new ChartModel(_portName);
                NotifyPropertyChanged("PortName");
            }
        }

        public MainWindowViewModel()
        {
            Syncfusion.Licensing.SyncfusionLicenseProvider.RegisterLicense("MjQyNDEwQDMxMzgyZTMxMmUzMGNKSzduS2FuZktxRmJERmkySTJsZjVCWlFPeVRGM3pMa1NPYWlVeUttSzA9");
            PortNames = XBeeFunctions.GetPortNamesByBaudrate(57600);

            if (PortNames.Count >= 1)
            {
                if (PortNames.Count == 1)
                    _portName = PortNames[0];

            }
            else
            {
                MessageBox.Show("At least one port needed for chart application to work");
                //If no COM port added, then close window.
                MainWindow mainWindow = Application.Current.Windows.OfType<MainWindow>().FirstOrDefault();
                mainWindow.Close();
            }
        }

        public event PropertyChangedEventHandler PropertyChanged;

        protected virtual void OnPropertyChanged(string propertyName = null)
        {
            if (PropertyChanged != null)
                PropertyChanged.Invoke(this, new PropertyChangedEventArgs(propertyName));
        }

        protected virtual void NotifyPropertyChanged([CallerMemberName] String propertyName = "")
        {
            var handler = PropertyChanged;
            if (handler != null)
                handler(this, new PropertyChangedEventArgs(propertyName));
        }
    }
}

