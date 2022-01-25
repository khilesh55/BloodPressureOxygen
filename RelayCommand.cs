using Prism.Commands;
using System;
using System.Windows.Input;

namespace SpasticityClient
{
    public static class ApplicationCommands
    {
        public static CompositeCommand ReadCommand = new CompositeCommand();
        public static CompositeCommand SaveCommand = new CompositeCommand();
        public static CompositeCommand StopCommand = new CompositeCommand();
    }
}
