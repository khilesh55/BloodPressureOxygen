using Prism.Commands;
using System;
using System.Windows.Input;

namespace SpasticityClient
{
    public static class ApplicationCommands
    {
        public static CompositeCommand SaveCommand = new CompositeCommand();
        public static CompositeCommand QueryCommand = new CompositeCommand();
        public static CompositeCommand PeakCommand = new CompositeCommand();
        
        public static CompositeCommand EnterICCommand = new CompositeCommand();
        public static CompositeCommand StartICCommand = new CompositeCommand();
        public static CompositeCommand StopICCommand = new CompositeCommand();

        public static CompositeCommand EnterBPCommand = new CompositeCommand();
        public static CompositeCommand StartBPCommand = new CompositeCommand();
        public static CompositeCommand StopBPCommand = new CompositeCommand();
    }
}
