using System.Windows;
using System.Windows.Forms;
using Syncfusion.XlsIO;
using System.Drawing;
using System.IO;

namespace SpasticityClient
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        private MainWindowViewModel mainWindowViewModel;

        public MainWindow()
        {
            
            InitializeComponent();
            mainWindowViewModel = (MainWindowViewModel)this.DataContext;
        }

        
    }
}
