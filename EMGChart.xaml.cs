using System;
using System.Windows;
using System.Windows.Controls;
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
