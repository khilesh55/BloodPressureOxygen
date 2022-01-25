using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Windows.Threading;

namespace SpasticityClient
{
    /// <summary>
    /// Interaction logic for EMGChart.xaml
    /// </summary>
    public partial class EMGChart : UserControl
    {
        public EMGChart()
        {
            InitializeComponent();
            //while (this.HasContent)
            //{
            //    this.Refresh();
            //    Thread.Sleep(100);
            //}
        }
    }
    public static class ExtensionMethods
    {
        private static Action EmptyDelegate = delegate () { };
        public static void Refresh(this UIElement uiElement)
        {
            uiElement.Dispatcher.Invoke(DispatcherPriority.Render, EmptyDelegate);
        }
    }
    
}
